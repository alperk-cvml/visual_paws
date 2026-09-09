#!/usr/bin/env python3
"""ground_plane_estimation — measure the robot's distance to the ground plane.

Purpose: test the hypothesis that monocular metric depth error is a scale
factor (depth_pred ≈ s · depth_true). Ground geometry gives an observable for
s that needs no ground-truth depth image:

  point cloud ──► transform to base frame ──► RANSAC ground plane ─┐
                                                                   ▼
        /ground_plane/distance        (base_link ⊥ distance to the plane)
        /ground_plane/camera_height   (camera origin ⊥ distance to the plane)
        /ground_plane/scale           (camera_height / expected camera height)

Why camera_height is the scale observable: a cloud with per-frame scale s is
p_cloud = s·p_true in the camera frame. The rigid TF into the base frame
rotates those points and shifts them by the (true-metric) camera offset, but
the *camera-origin-relative* geometry is only rotated — so the perpendicular
distance from the camera origin to the fitted plane is exactly s × the true
camera height above ground. The base_link distance mixes in the un-scaled TF
translation and is published for plotting/sanity, not for estimating s.

The expected camera height comes from one of two references:
  * `height_reference_frame` (e.g. `odom` under LOCALIZATION_MODE=ground_truth,
    where the ground is the z=0 plane): expected = z of the cloud frame origin
    in that frame, looked up per cloud.
  * `expected_base_height` (> 0 disables the TF reference): nominal standing
    height of base_link above ground; expected = that + the camera's z offset
    in the base frame. Assumes a level body.
If neither is available the scale topic is simply not published.

One node instance handles one input cloud; the launch file spawns one node
per topic so every depth backend of the comparison pipeline gets its own
/ground_plane/<key>/* channel on identical frames.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from .plane_fit import fit_ground_plane

UP = np.array([0.0, 0.0, 1.0])


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


class GroundPlaneNode(Node):
    """Per-cloud ground plane distance estimator."""

    def __init__(self):
        super().__init__('ground_plane')

        self.declare_parameter('input_topic', '/depth_camera/points')
        self.declare_parameter('output_namespace', '/ground_plane')
        self.declare_parameter('base_frame', 'base_link')
        # Upper bound on processing rate; clouds arriving faster are dropped.
        self.declare_parameter('target_fps', 5.0)
        # Candidate gate: point range in the cloud frame (m, cloud units).
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 8.0)
        # Keep only the lowest fraction of candidates (by base-frame z) before
        # RANSAC. Percentile-based so it survives an unknown depth scale.
        self.declare_parameter('candidate_z_percentile', 60.0)
        self.declare_parameter('max_points', 4000)
        # RANSAC
        self.declare_parameter('ransac_iterations', 120)
        self.declare_parameter('ransac_threshold', 0.03)
        self.declare_parameter('max_ground_angle_deg', 35.0)
        self.declare_parameter('min_inliers', 100)
        # Scale reference (see module docstring). expected_base_height > 0
        # wins over height_reference_frame; both empty/zero → no /scale topic.
        self.declare_parameter('expected_base_height', 0.0)
        self.declare_parameter('height_reference_frame', '')
        self.declare_parameter('publish_marker', True)

        p = self.get_parameter
        self._base_frame = p('base_frame').value
        self._min_range = float(p('min_range').value)
        self._max_range = float(p('max_range').value)
        self._z_percentile = float(p('candidate_z_percentile').value)
        self._max_points = int(p('max_points').value)
        self._iterations = int(p('ransac_iterations').value)
        self._threshold = float(p('ransac_threshold').value)
        self._max_angle = float(np.deg2rad(p('max_ground_angle_deg').value))
        self._min_inliers = int(p('min_inliers').value)
        self._expected_base_height = float(p('expected_base_height').value)
        self._height_ref_frame = str(p('height_reference_frame').value)
        self._min_period = 1.0 / max(float(p('target_fps').value), 1e-3)
        self._last_stamp = None
        self._rng = np.random.default_rng(0)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        ns = str(p('output_namespace').value).rstrip('/')
        self._pub_distance = self.create_publisher(Range, f'{ns}/distance', 10)
        self._pub_cam_height = self.create_publisher(Range, f'{ns}/camera_height', 10)
        self._pub_scale = self.create_publisher(Float32, f'{ns}/scale', 10)
        self._pub_marker = (
            self.create_publisher(Marker, f'{ns}/marker', 10)
            if bool(p('publish_marker').value) else None
        )

        topic = str(p('input_topic').value)
        self.create_subscription(
            PointCloud2, topic, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f'ground plane estimation: {topic} -> {ns}/* '
            f'(base_frame={self._base_frame})')

    # ── helpers ──────────────────────────────────────────────────────────

    def _lookup(self, target: str, source: str, stamp: Time):
        """TF target←source at `stamp` as (R, t); falls back to latest."""
        try:
            tf = self._tf_buffer.lookup_transform(
                target, source, stamp, timeout=Duration(seconds=0.1))
        except Exception:
            tf = self._tf_buffer.lookup_transform(target, source, Time())
        tr = tf.transform.translation
        return (_quat_to_rot(tf.transform.rotation),
                np.array([tr.x, tr.y, tr.z]))

    def _expected_camera_height(self, cloud_frame: str, stamp: Time,
                                cam_in_base: np.ndarray):
        """True-metric camera height above ground, or None if no reference."""
        if self._expected_base_height > 0.0:
            return self._expected_base_height + float(cam_in_base[2])
        if self._height_ref_frame:
            try:
                _, t = self._lookup(self._height_ref_frame, cloud_frame, stamp)
                return float(t[2])
            except Exception as exc:
                self.get_logger().warning(
                    f'height reference {self._height_ref_frame} unavailable: {exc}',
                    throttle_duration_sec=5.0)
        return None

    def _range_msg(self, header_stamp, frame: str, value: float) -> Range:
        msg = Range()
        msg.header.stamp = header_stamp
        msg.header.frame_id = frame
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.0
        msg.min_range = 0.0
        msg.max_range = float(self._max_range)
        msg.range = float(value)
        return msg

    # ── main callback ────────────────────────────────────────────────────

    def _on_cloud(self, msg: PointCloud2):
        stamp = Time.from_msg(msg.header.stamp)
        if (self._last_stamp is not None
                and (stamp - self._last_stamp).nanoseconds < self._min_period * 1e9):
            return

        points = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if len(points) < self._min_inliers:
            self.get_logger().warning(
                f'cloud has only {len(points)} finite points, skipping',
                throttle_duration_sec=5.0)
            return

        cloud_frame = msg.header.frame_id
        if cloud_frame == self._base_frame:
            rot, trans = np.eye(3), np.zeros(3)
        else:
            try:
                rot, trans = self._lookup(self._base_frame, cloud_frame, stamp)
            except Exception as exc:
                self.get_logger().warning(
                    f'TF {self._base_frame} <- {cloud_frame} unavailable: {exc}',
                    throttle_duration_sec=5.0)
                return

        # Range gate in the cloud frame, then keep the low points (base-frame
        # z percentile) so walls and obstacles don't dominate RANSAC.
        ranges = np.linalg.norm(points, axis=1)
        points = points[(ranges > self._min_range) & (ranges < self._max_range)]
        if len(points) < self._min_inliers:
            return
        points_base = points @ rot.T + trans
        z_cut = np.percentile(points_base[:, 2], self._z_percentile)
        candidates = points_base[points_base[:, 2] <= z_cut]
        if len(candidates) > self._max_points:
            candidates = candidates[self._rng.choice(
                len(candidates), size=self._max_points, replace=False)]

        fit = fit_ground_plane(
            candidates, UP,
            iterations=self._iterations,
            threshold=self._threshold,
            max_angle_rad=self._max_angle,
            min_inliers=self._min_inliers,
            rng=self._rng,
        )
        if fit is None:
            self.get_logger().warning(
                'no ground plane found in cloud', throttle_duration_sec=5.0)
            return
        self._last_stamp = stamp

        base_distance = fit.signed_distance(np.zeros(3))
        camera_height = fit.signed_distance(trans)

        self._pub_distance.publish(
            self._range_msg(msg.header.stamp, self._base_frame, base_distance))
        self._pub_cam_height.publish(
            self._range_msg(msg.header.stamp, cloud_frame, camera_height))

        expected = self._expected_camera_height(cloud_frame, stamp, trans)
        if expected is not None and expected > 1e-3:
            self._pub_scale.publish(Float32(data=float(camera_height / expected)))

        if self._pub_marker is not None:
            self._publish_marker(msg.header.stamp, fit)

    def _publish_marker(self, header_stamp, fit):
        """Thin slab on the fitted plane under the robot, for RViz."""
        # Project the base origin onto the plane; align the marker z with n.
        n = fit.normal
        centre = -fit.signed_distance(np.zeros(3)) * n
        seed = UP if abs(n @ UP) < 0.9 else np.array([1.0, 0.0, 0.0])
        x_axis = np.cross(seed, n)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(n, x_axis)

        marker = Marker()
        marker.header.stamp = header_stamp
        marker.header.frame_id = self._base_frame
        marker.ns = 'ground_plane'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(
            x=float(centre[0]), y=float(centre[1]), z=float(centre[2]))
        marker.pose.orientation = _rot_to_quat(
            np.column_stack((x_axis, y_axis, n)))
        marker.scale.x, marker.scale.y, marker.scale.z = 2.0, 2.0, 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = \
            0.1, 0.8, 0.2, 0.4
        marker.lifetime = Duration(seconds=3.0 * self._min_period).to_msg()
        self._pub_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GroundPlaneNode()
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
