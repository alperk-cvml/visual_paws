#!/usr/bin/env python3
"""lidar_odometry — LiDAR odometry from any PointCloud2, via KISS-ICP.

  /lidar/points ──► KISS-ICP (adaptive-threshold point-to-point ICP,
                    constant-velocity prior, voxel hash map) ──► /odom_lidar
                                                        (nav_msgs/Odometry)

Backend choice: KISS-ICP is pure-LiDAR and sensor-agnostic — no IMU sync, no
ring index, no per-point time requirement (deskew is optional), ~5 parameters,
CPU-only. That makes it the right *first* backend for this project, where the
module's near-term job is a metric reference odometry for the monocular depth
scale work (Alternative 2: compare kinetic motion against monodepth-cloud
motion), typically replayed from bags recorded with whatever lidar is at hand.
Tightly-coupled LiDAR-inertial systems (FAST-LIO2, Point-LIO, DLIO) track a
trotting body better but demand a hard-synced high-rate IMU and per-point
timestamps — the documented upgrade path once a lidar is body-mounted for
production. See this module's README for the comparison.

Frames: KISS-ICP estimates the sensor's motion in the frame of its first scan.
The node looks up the static base_link←<cloud frame> extrinsic from TF and
conjugates poses with it, so `odom` is the base_link pose at the first scan
and the output follows the vehicle interface contract (odom → base_link,
twist in base_link). If the extrinsic never appears (standalone bag replay
with no robot_state_publisher), it falls back to identity after a grace
period and the odometry is reported in the lidar frame.

TF: this node does NOT publish odom→base_link by default. That edge belongs
to the robot_localization EKF (or the sim ground-truth publisher). Enable
publish_tf only for standalone debugging when nothing else owns the edge.

Uncertainty is explicit: twist covariance is a per-sample estimate; pose
covariance grows with traveled distance and turned angle, because ICP
odometry drifts — this is odometry, not localization.
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from kiss_icp.config import load_config
from kiss_icp.kiss_icp import KissICP

from .cloud_conversion import extract_xyz_and_times

MIN_POINTS = 100


def _quat_to_rot(q) -> np.ndarray:
    """geometry_msgs/Quaternion → 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rot_to_quat(rot: np.ndarray) -> Quaternion:
    """3x3 rotation matrix → geometry_msgs/Quaternion (Shepperd's method)."""
    m = rot
    t = np.trace(m)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, \
            (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, \
            (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, \
            0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, \
            (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


def _rot_to_rotvec(rot: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix → axis-angle rotation vector (radians)."""
    cos_angle = np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cos_angle))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([rot[2, 1] - rot[1, 2],
                     rot[0, 2] - rot[2, 0],
                     rot[1, 0] - rot[0, 1]]) / (2.0 * np.sin(angle))
    return axis * angle


class LidarOdometryNode(Node):
    """KISS-ICP LiDAR odometry publisher."""

    def __init__(self):
        super().__init__('lidar_odometry')

        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('odom_topic', '/odom_lidar')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # odom→base_link normally belongs to the EKF / ground-truth publisher.
        self.declare_parameter('publish_tf', False)
        # KISS-ICP: range gate (m) and map voxel size (m; <=0 → max_range/100).
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('voxel_size', 0.0)
        self.declare_parameter('max_points_per_voxel', 20)
        self.declare_parameter('initial_threshold', 2.0)
        self.declare_parameter('min_motion_th', 0.1)
        # Motion compensation; needs a per-point time field in the cloud.
        self.declare_parameter('deskew', False)
        # Grace period for the base_link←cloud-frame extrinsic to appear on
        # TF before falling back to identity (odometry in the lidar frame).
        self.declare_parameter('wait_for_extrinsic_sec', 10.0)
        # 1-sigma twist noise (m/s, rad/s).
        self.declare_parameter('twist_linear_stddev', 0.05)
        self.declare_parameter('twist_angular_stddev', 0.02)
        # Pose drift: 1-sigma position error per metre traveled and yaw error
        # per radian turned. Drives the growing pose covariance.
        self.declare_parameter('drift_per_metre', 0.01)
        self.declare_parameter('drift_per_radian', 0.01)

        p = self.get_parameter
        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value
        self._publish_tf = bool(p('publish_tf').value)
        self._max_range = float(p('max_range').value)
        self._min_range = float(p('min_range').value)
        self._voxel_size = float(p('voxel_size').value)
        self._max_points_per_voxel = int(p('max_points_per_voxel').value)
        self._initial_threshold = float(p('initial_threshold').value)
        self._min_motion_th = float(p('min_motion_th').value)
        self._deskew = bool(p('deskew').value)
        self._extrinsic_wait = float(p('wait_for_extrinsic_sec').value)
        self._sigma_v = float(p('twist_linear_stddev').value)
        self._sigma_w = float(p('twist_angular_stddev').value)
        self._drift_per_m = float(p('drift_per_metre').value)
        self._drift_per_rad = float(p('drift_per_radian').value)

        # ── State ────────────────────────────────────────────────────────
        self._kiss: KissICP | None = None      # built on the first usable cloud
        self._T_bl: np.ndarray | None = None   # base_link ← cloud frame (4x4)
        self._T_lb: np.ndarray | None = None   # inverse of the above
        self._extrinsic_final = False
        self._first_cloud_monotonic: float | None = None
        self._last_stamp: Time | None = None
        self._dist = 0.0                       # ∫|v| dt, for the drift model
        self._angle = 0.0                      # ∫|ω| dt
        self._warned_no_time_field = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._odom_pub = self.create_publisher(
            Odometry, p('odom_topic').value, 10)
        self._tf_broadcaster = None
        if self._publish_tf:
            from tf2_ros import TransformBroadcaster
            self._tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().warn(
                'publish_tf enabled — make sure no EKF or ground-truth node '
                'also publishes odom→base_link (one publisher per TF edge).')

        # Depth 1 on purpose: registration slower than the scan rate must
        # drop backlog and always process the newest cloud, not queue up lag.
        qos = QoSProfile(depth=1,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, p('input_topic').value, self._on_cloud, qos)

        self.get_logger().info(
            '\n'
            '============================================\n'
            '  LiDAR Odometry (KISS-ICP)\n'
            '============================================\n'
            '  Point cloud: %s\n'
            '  Publishing:  %s (%s → %s)\n'
            '  Range gate:  %.1f–%.1f m, deskew=%s\n'
            '============================================\n'
            % (p('input_topic').value, p('odom_topic').value,
               self._odom_frame, self._base_frame,
               self._min_range, self._max_range, self._deskew)
        )

    # ── setup helpers ───────────────────────────────────────────────────

    def _build_pipeline(self, have_time_field: bool):
        deskew = self._deskew and have_time_field
        if self._deskew and not have_time_field \
                and not self._warned_no_time_field:
            self.get_logger().warn(
                'deskew requested but the cloud has no per-point time field '
                '— running without motion compensation.')
            self._warned_no_time_field = True
        config = load_config(None)
        config.data.max_range = self._max_range
        config.data.min_range = self._min_range
        config.data.deskew = deskew
        config.mapping.voxel_size = (
            self._voxel_size if self._voxel_size > 0.0
            else self._max_range / 100.0)
        config.mapping.max_points_per_voxel = self._max_points_per_voxel
        config.adaptive_threshold.initial_threshold = self._initial_threshold
        config.adaptive_threshold.min_motion_th = self._min_motion_th
        self._kiss = KissICP(config)
        self._deskew_active = deskew

    def _reset(self, reason: str):
        self.get_logger().warn(f'{reason} — resetting odometry.')
        self._kiss = None
        self._last_stamp = None
        self._dist = 0.0
        self._angle = 0.0

    def _resolve_extrinsic(self, cloud_frame: str) -> bool:
        """Latch the static base_link←cloud extrinsic; True when decided."""
        if self._extrinsic_final:
            return True
        if cloud_frame == self._base_frame:
            self._T_bl = np.eye(4)
        else:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._base_frame, cloud_frame, Time())
                self._T_bl = np.eye(4)
                self._T_bl[:3, :3] = _quat_to_rot(tf.transform.rotation)
                tr = tf.transform.translation
                self._T_bl[:3, 3] = (tr.x, tr.y, tr.z)
            except Exception as exc:
                now = time.monotonic()
                if self._first_cloud_monotonic is None:
                    self._first_cloud_monotonic = now
                if now - self._first_cloud_monotonic < self._extrinsic_wait:
                    self.get_logger().info(
                        f'waiting for TF {self._base_frame} <- {cloud_frame} '
                        f'({exc})', throttle_duration_sec=2.0)
                    return False
                self._T_bl = np.eye(4)
                self.get_logger().warn(
                    f'TF {self._base_frame} <- {cloud_frame} never appeared '
                    f'— publishing odometry in the {cloud_frame} frame.')
        self._T_lb = np.linalg.inv(self._T_bl)
        self._extrinsic_final = True
        return True

    # ── main callback ───────────────────────────────────────────────────

    def _on_cloud(self, msg: PointCloud2):
        stamp = Time.from_msg(msg.header.stamp)
        if (self._last_stamp is not None
                and (stamp - self._last_stamp).nanoseconds < -1_000_000_000):
            self._reset('input time jumped backward (bag restart?)')

        if not self._resolve_extrinsic(msg.header.frame_id):
            return

        xyz, times = extract_xyz_and_times(msg)
        if len(xyz) < MIN_POINTS:
            self.get_logger().warn(
                f'cloud has only {len(xyz)} finite points, skipping',
                throttle_duration_sec=5.0)
            return

        if self._kiss is None:
            self._build_pipeline(have_time_field=times is not None)

        timestamps = (times if self._deskew_active and times is not None
                      else np.zeros(len(xyz)))
        self._kiss.register_frame(xyz, timestamps)

        # Conjugate lidar-frame motion into base_link:
        # T_base(k) = T_bl · T_lidar(k) · T_bl⁻¹  (odom = base pose at scan 0)
        pose_base = self._T_bl @ self._kiss.last_pose @ self._T_lb
        delta_base = self._T_bl @ self._kiss.last_delta @ self._T_lb

        dt = None
        if self._last_stamp is not None:
            dt = (stamp - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = stamp

        v_base = np.zeros(3)
        w_base = np.zeros(3)
        if dt is not None and 0.0 < dt < 1.0:
            v_base = delta_base[:3, 3] / dt
            rotvec = _rot_to_rotvec(delta_base[:3, :3])
            w_base = rotvec / dt
            self._dist += float(np.linalg.norm(delta_base[:3, 3]))
            self._angle += float(np.linalg.norm(rotvec))

        self._publish(msg.header.stamp, pose_base, v_base, w_base)

    # ── output ──────────────────────────────────────────────────────────

    def _publish(self, stamp, pose: np.ndarray,
                 v_base: np.ndarray, w_base: np.ndarray):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = float(pose[0, 3])
        odom.pose.pose.position.y = float(pose[1, 3])
        odom.pose.pose.position.z = float(pose[2, 3])
        odom.pose.pose.orientation = _rot_to_quat(pose[:3, :3])

        odom.twist.twist.linear.x = float(v_base[0])
        odom.twist.twist.linear.y = float(v_base[1])
        odom.twist.twist.linear.z = float(v_base[2])
        odom.twist.twist.angular.x = float(w_base[0])
        odom.twist.twist.angular.y = float(w_base[1])
        odom.twist.twist.angular.z = float(w_base[2])

        twist_cov = np.zeros(36)
        twist_cov[[0, 7, 14]] = self._sigma_v ** 2
        twist_cov[[21, 28, 35]] = self._sigma_w ** 2
        odom.twist.covariance = list(twist_cov)

        # Pose covariance: drift model, grows with distance and turn angle.
        var_p = (self._drift_per_m * self._dist) ** 2 + 1e-6
        var_r = (self._drift_per_rad * self._angle) ** 2 + 1e-6
        pose_cov = np.zeros(36)
        pose_cov[[0, 7, 14]] = var_p
        pose_cov[[21, 28, 35]] = var_r
        odom.pose.covariance = list(pose_cov)

        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = odom.pose.pose.position.x
            tf.transform.translation.y = odom.pose.pose.position.y
            tf.transform.translation.z = odom.pose.pose.position.z
            tf.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
