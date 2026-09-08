#!/usr/bin/env python3
"""
quadruped_leg_odometry — leg-kinematics odometry (LKO) node.

Fuses the two proprioceptive sources into one body-motion estimate:

  /joint_states (12 leg joints) ──► forward kinematics + Jacobians ─┐
  /imu/data     (gyro + accel)  ──► attitude + angular velocity ────┤
                                                                    ▼
                                                            /odom_lko
                                                     (nav_msgs/Odometry)

Principle: a stance foot is stationary in the world, so its body-relative
velocity J(q)·q̇ mirrors the body's own motion. Averaging over stance feet
gives the body linear velocity; the IMU supplies attitude and angular
velocity; integrating the rotated velocity yields the odometry pose.

Stance detection is kinematic (feet within a height band of the lowest
foot, measured along gravity via the IMU attitude) so the node needs no
simulator-specific contact topics and runs unchanged on real hardware.

Uncertainty is explicit: twist covariance is the per-sample estimate
(inflated when fewer than two feet are in stance); pose covariance grows
with traveled distance and turned angle, because integrated leg odometry
drifts — this is odometry, not localization.

TF: this node does NOT publish odom→base_link by default. That edge belongs
to the robot_localization EKF (or the sim ground-truth publisher). Enable
publish_tf only for standalone debugging when nothing else owns the edge.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState

from .imu_attitude import ComplementaryAttitude, quat_to_rot
from .kinematics import JOINT_NAMES, LEGS, LEG_SIGNS, body_velocity_from_stance, leg_fk


class LegOdometryNode(Node):
    """Leg-kinematics + IMU odometry publisher."""

    def __init__(self):
        super().__init__('leg_odometry')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom_lko')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # odom→base_link normally belongs to the EKF / ground-truth publisher.
        self.declare_parameter('publish_tf', False)
        # Prefer the orientation quaternion in the Imu message when the source
        # provides one (Gazebo does); fall back to the complementary filter.
        self.declare_parameter('use_imu_orientation', True)
        # Feet within this height (m) of the lowest foot count as stance.
        self.declare_parameter('stance_height_band', 0.008)
        # 1-sigma twist noise (m/s, rad/s) with full stance support.
        self.declare_parameter('twist_linear_stddev', 0.02)
        self.declare_parameter('twist_angular_stddev', 0.01)
        # Pose drift: 1-sigma position error per metre traveled and yaw error
        # per radian turned. Drives the growing pose covariance.
        self.declare_parameter('drift_per_metre', 0.05)
        self.declare_parameter('drift_per_radian', 0.05)
        self.declare_parameter('imu_attitude_gain', 0.5)

        p = self.get_parameter
        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value
        self._publish_tf = bool(p('publish_tf').value)
        self._use_imu_orientation = bool(p('use_imu_orientation').value)
        self._stance_band = float(p('stance_height_band').value)
        self._sigma_v = float(p('twist_linear_stddev').value)
        self._sigma_w = float(p('twist_angular_stddev').value)
        self._drift_per_m = float(p('drift_per_metre').value)
        self._drift_per_rad = float(p('drift_per_radian').value)

        # ── State ────────────────────────────────────────────────────────
        self._attitude = ComplementaryAttitude(
            accel_gain=float(p('imu_attitude_gain').value))
        self._q_ob = np.array([1.0, 0.0, 0.0, 0.0])   # base→odom rotation
        self._have_attitude = False
        self._omega = np.zeros(3)                     # body angular velocity
        self._pos = np.zeros(3)                       # base_link in odom
        self._dist = 0.0                              # ∫|v| dt, for drift model
        self._angle = 0.0                             # ∫|ω_z| dt
        self._last_joint_stamp: Time | None = None
        self._last_imu_stamp: Time | None = None
        self._prev_q: dict[str, np.ndarray] | None = None  # for FD fallback
        self._warned_no_attitude = False

        self._odom_pub = self.create_publisher(Odometry, p('odom_topic').value, 10)
        self._tf_broadcaster = None
        if self._publish_tf:
            from tf2_ros import TransformBroadcaster
            self._tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().warn(
                'publish_tf enabled — make sure no EKF or ground-truth node '
                'also publishes odom→base_link (one publisher per TF edge).')

        self.create_subscription(
            Imu, p('imu_topic').value, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            JointState, p('joint_states_topic').value, self._joint_cb, 10)

        self.get_logger().info(
            '\n'
            '============================================\n'
            '  Leg Kinematics Odometry (LKO)\n'
            '============================================\n'
            '  Joint states: %s\n'
            '  IMU:          %s\n'
            '  Publishing:   %s (%s → %s)\n'
            '============================================\n'
            % (p('joint_states_topic').value, p('imu_topic').value,
               p('odom_topic').value, self._odom_frame, self._base_frame)
        )

    # ── IMU sub-component ───────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        stamp = Time.from_msg(msg.header.stamp)
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])
        self._omega = gyro

        o = msg.orientation
        imu_orientation_valid = (
            msg.orientation_covariance[0] >= 0.0
            and (o.w * o.w + o.x * o.x + o.y * o.y + o.z * o.z) > 1e-6
        )

        if self._use_imu_orientation and imu_orientation_valid:
            self._q_ob = np.array([o.w, o.x, o.y, o.z])
            self._attitude.q = self._q_ob.copy()
            self._have_attitude = True
        else:
            if self._last_imu_stamp is not None:
                dt = (stamp - self._last_imu_stamp).nanoseconds * 1e-9
                if 0.0 < dt < 0.5:
                    accel = np.array([msg.linear_acceleration.x,
                                      msg.linear_acceleration.y,
                                      msg.linear_acceleration.z])
                    self._q_ob = self._attitude.update(gyro, accel, dt)
                    self._have_attitude = True
        self._last_imu_stamp = stamp

    # ── Leg-odometry sub-component ──────────────────────────────────────
    def _joint_cb(self, msg: JointState):
        stamp = Time.from_msg(msg.header.stamp)
        dt = None
        if self._last_joint_stamp is not None:
            dt = (stamp - self._last_joint_stamp).nanoseconds * 1e-9

        idx = {name: i for i, name in enumerate(msg.name)}
        have_velocity = len(msg.velocity) == len(msg.name)

        q_by_leg: dict[str, np.ndarray] = {}
        qd_by_leg: dict[str, np.ndarray] = {}
        try:
            for leg in LEGS:
                ids = [idx[j] for j in JOINT_NAMES[leg]]
                q_by_leg[leg] = np.array([msg.position[i] for i in ids])
                if have_velocity:
                    qd_by_leg[leg] = np.array([msg.velocity[i] for i in ids])
        except KeyError:
            return  # not the leg joint states (or incomplete message)

        if not have_velocity:
            # Finite-difference fallback when the broadcaster omits velocities.
            if self._prev_q is None or dt is None or not 0.0 < dt < 0.5:
                self._prev_q = q_by_leg
                self._last_joint_stamp = stamp
                return
            qd_by_leg = {leg: (q_by_leg[leg] - self._prev_q[leg]) / dt
                         for leg in LEGS}
        self._prev_q = q_by_leg

        if not self._have_attitude and not self._warned_no_attitude:
            self.get_logger().warn(
                'No IMU attitude yet — integrating with identity orientation '
                'until /imu/data arrives.')
            self._warned_no_attitude = True

        r_ob = quat_to_rot(self._q_ob)

        foot_pos: dict[str, np.ndarray] = {}
        foot_vel: dict[str, np.ndarray] = {}
        for leg in LEGS:
            x_sign, y_sign = LEG_SIGNS[leg]
            pos, jac = leg_fk(q_by_leg[leg], x_sign, y_sign)
            foot_pos[leg] = pos
            foot_vel[leg] = jac @ qd_by_leg[leg]

        # Stance = feet within a height band of the lowest foot, with height
        # measured along gravity (rotate feet into the odom orientation).
        heights = {leg: (r_ob @ foot_pos[leg])[2] for leg in LEGS}
        floor = min(heights.values())
        stance = [leg for leg in LEGS if heights[leg] - floor < self._stance_band]

        v_body = body_velocity_from_stance(
            [foot_pos[leg] for leg in stance],
            [foot_vel[leg] for leg in stance],
            self._omega,
        )

        # ── Integrate pose ──────────────────────────────────────────────
        if dt is not None and 0.0 < dt < 0.5:
            v_odom = r_ob @ v_body
            self._pos += v_odom * dt
            self._dist += float(np.linalg.norm(v_body)) * dt
            self._angle += abs(float(self._omega[2])) * dt
        self._last_joint_stamp = stamp

        self._publish(msg.header.stamp, v_body, len(stance))

    # ── Output ──────────────────────────────────────────────────────────
    def _publish(self, stamp, v_body: np.ndarray, n_stance: int):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = float(self._pos[0])
        odom.pose.pose.position.y = float(self._pos[1])
        odom.pose.pose.position.z = float(self._pos[2])
        w, x, y, z = self._q_ob
        odom.pose.pose.orientation.w = float(w)
        odom.pose.pose.orientation.x = float(x)
        odom.pose.pose.orientation.y = float(y)
        odom.pose.pose.orientation.z = float(z)

        odom.twist.twist.linear.x = float(v_body[0])
        odom.twist.twist.linear.y = float(v_body[1])
        odom.twist.twist.linear.z = float(v_body[2])
        odom.twist.twist.angular.x = float(self._omega[0])
        odom.twist.twist.angular.y = float(self._omega[1])
        odom.twist.twist.angular.z = float(self._omega[2])

        # Twist covariance: per-sample uncertainty; a single stance foot makes
        # the ω×p lever arm unobservable, so inflate hard below two feet.
        inflate = 1.0 if n_stance >= 2 else 100.0
        var_v = (self._sigma_v ** 2) * inflate
        var_w = self._sigma_w ** 2
        twist_cov = np.zeros(36)
        twist_cov[[0, 7, 14]] = var_v
        twist_cov[[21, 28, 35]] = var_w
        odom.twist.covariance = list(twist_cov)

        # Pose covariance: drift model, grows with distance and turn angle.
        var_p = (self._drift_per_m * self._dist) ** 2 + 1e-6
        var_yaw = (self._drift_per_rad * self._angle) ** 2 + 1e-6
        var_tilt = 0.02 ** 2  # roll/pitch are gravity-referenced, bounded
        pose_cov = np.zeros(36)
        pose_cov[[0, 7, 14]] = var_p
        pose_cov[[21, 28]] = var_tilt
        pose_cov[35] = var_yaw
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
    node = LegOdometryNode()
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
