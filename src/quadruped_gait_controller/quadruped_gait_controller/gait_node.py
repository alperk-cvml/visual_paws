#!/usr/bin/env python3
"""
quadruped_gait_controller — diagonal trot gait node.

Architecture:
  /cmd_vel  (geometry_msgs/Twist)
       │
       ▼
  GaitNode  ── Float64MultiArray ──►  /quadruped_joint_controller/commands

Controller in simulation:
  position_controllers/JointGroupPositionController
  Topic: /quadruped_joint_controller/commands
  Type:  std_msgs/Float64MultiArray
  Joint order (must match ros2_control.yaml):
    FL_hip_roll, FL_hip_pitch, FL_knee,
    FR_hip_roll, FR_hip_pitch, FR_knee,
    RL_hip_roll, RL_hip_pitch, RL_knee,
    RR_hip_roll, RR_hip_pitch, RR_knee

Gait: diagonal trot
  Phase A [0, 0.5): FL + RR swing, FR + RL stance
  Phase B [0.5, 1): FR + RL swing, FL + RR stance
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# ── Robot geometry (metres — must match URDF properties) ────────────────────
HIP_X_OFFSET = 0.10    # fore-aft distance from body centre to hip
HIP_Y_OFFSET = 0.065   # lateral distance from body centre to hip
HIP_LENGTH   = 0.04    # hip abduction link length (along Y)
L1           = 0.085   # thigh length
L2           = 0.095   # calf length

# ── Standing pose (radians, match URDF initial_value) ───────────────────────
STAND_HIP_ROLL  = 0.0
STAND_HIP_PITCH = 0.5
STAND_KNEE      = -1.2

# ── Gait parameters ─────────────────────────────────────────────────────────
STRIDE_PERIOD   = 0.4    # seconds per full stride
STEP_HEIGHT     = 0.015  # metres — swing foot lift
MAX_STEP_LENGTH = 0.08   # metres — maximum stride stroke
MAX_LATERAL     = 0.04   # metres — maximum lateral stroke
PUBLISH_RATE_HZ = 50     # Hz — control loop rate
CMD_TIMEOUT     = 0.5    # seconds — stop if /cmd_vel goes stale (dead publisher)

# ── Leg table ────────────────────────────────────────────────────────────────
LEGS = ('FL', 'FR', 'RL', 'RR')
LEG_SIGNS = {
    'FL': ( 1,  1),   # (x_sign, y_sign)
    'FR': ( 1, -1),
    'RL': (-1,  1),
    'RR': (-1, -1),
}
# Diagonal trot pairs
SWING_A = {'FL', 'RR'}


# ─────────────────────────────────────────────────────────────────────────────
# Analytic leg IK
# ─────────────────────────────────────────────────────────────────────────────

def leg_ik(foot_x: float, foot_y: float, foot_z: float,
           y_sign: int) -> tuple[float, float, float]:
    """
    Analytic IK for a 3-DOF leg (hip_roll, hip_pitch, knee).

    Args:
        foot_x: desired foot position x in body frame (forward +)
        foot_y: desired foot position y in body frame (left +)
        foot_z: desired foot position z in body frame (up +, typically negative)
        y_sign: +1 for left legs, -1 for right legs

    Returns:
        (hip_roll, hip_pitch, knee) radians
    """
    # Hip roll (abduction) — coronal plane
    dy = foot_y - y_sign * HIP_Y_OFFSET
    dz = foot_z                             # negative (foot below hip)
    
    # Effective leg-chain reach after removing the fixed HIP_LENGTH offset
    r_coronal = math.sqrt(dy**2 + dz**2)
    r_chain = math.sqrt(max(r_coronal**2 - HIP_LENGTH**2, 0.0))
    
    # hip_roll incorporates the offset created by HIP_LENGTH
    hip_roll = math.atan2(dy, -dz) - math.atan2(y_sign * HIP_LENGTH, r_chain)



    # Sagittal-plane 2-link IK (thigh=L1, calf=L2)
    r = math.sqrt(foot_x**2 + r_chain**2)
    r = min(max(r, 1e-6), L1 + L2 - 1e-4)

    cos_knee = (L1**2 + L2**2 - r**2) / (2.0 * L1 * L2)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    knee = -(math.pi - math.acos(cos_knee))   # negative per URDF convention

    # URDF pitch axes are +y, so positive hip_pitch/knee swing the foot
    # backward (-x): the sagittal angles are solved against -foot_x.
    alpha = math.atan2(-foot_x, r_chain)
    cos_gamma = (L1**2 + r**2 - L2**2) / (2.0 * L1 * r)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    hip_pitch = alpha + math.acos(cos_gamma)

    return hip_roll, hip_pitch, knee


# ─────────────────────────────────────────────────────────────────────────────
# Gait Node
# ─────────────────────────────────────────────────────────────────────────────

class GaitNode(Node):
    """Diagonal trot gait controller."""

    def __init__(self):
        super().__init__('gait_controller')

        self._pub = self.create_publisher(
            Float64MultiArray,
            '/quadruped_joint_controller/commands',
            10,
        )
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_omega = 0.0
        self._last_cmd_time = None

        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0

        self._phase = 0.0
        dt = 1.0 / PUBLISH_RATE_HZ
        self._phase_vel = 1.0 / STRIDE_PERIOD
        self._timer = self.create_timer(dt, self._tick)

        # Pre-compute nominal standing foot position (relative to hip)
        z_stand = -(L1 * math.cos(STAND_HIP_PITCH) +
                    L2 * math.cos(STAND_HIP_PITCH + STAND_KNEE))
        x_stand = -(L1 * math.sin(STAND_HIP_PITCH) +
                    L2 * math.sin(STAND_HIP_PITCH + STAND_KNEE))
        self._z_stand = z_stand
        self._x_stand = x_stand

        self.get_logger().info(
            '\n'
            '============================================\n'
            '  Quadruped Gait Controller — diagonal trot\n'
            '============================================\n'
            '  Subscribing: /cmd_vel\n'
            '  Publishing:  /quadruped_joint_controller/commands\n'
            '  Rate: %d Hz  |  Stride: %.2f s\n'
            '============================================\n'
            % (PUBLISH_RATE_HZ, STRIDE_PERIOD)
        )

    def _cmd_vel_cb(self, msg: Twist):
        self._cmd_vx = float(msg.linear.x)
        self._cmd_vy = float(msg.linear.y)
        self._cmd_omega = float(msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _tick(self):
        dt = 1.0 / PUBLISH_RATE_HZ

        # Watchdog: a quit teleop or crashed planner must not leave the last
        # command latched forever.
        if self._last_cmd_time is not None:
            age = (self.get_clock().now() - self._last_cmd_time).nanoseconds
            if age > CMD_TIMEOUT * 1e9:
                self._cmd_vx = 0.0
                self._cmd_vy = 0.0
                self._cmd_omega = 0.0

        # Smooth velocity with acceleration limits
        dvx = self._cmd_vx - self._vx
        dvy = self._cmd_vy - self._vy
        domega = self._cmd_omega - self._omega

        accel_x = 0.8  # m/s^2
        accel_y = 0.8  # m/s^2
        accel_w = 2.0  # rad/s^2

        self._vx += max(-accel_x * dt, min(accel_x * dt, dvx))
        self._vy += max(-accel_y * dt, min(accel_y * dt, dvy))
        self._omega += max(-accel_w * dt, min(accel_w * dt, domega))

        moving = abs(self._vx) > 0.001 or abs(self._vy) > 0.001 or abs(self._omega) > 0.001

        if abs(self._cmd_vx) < 0.001 and abs(self._cmd_vy) < 0.001 and abs(self._cmd_omega) < 0.001:
            if not moving:
                self._phase = 0.0
                self._publish_stand()
                return

        self._phase = (self._phase + self._phase_vel * dt) % 1.0

        self._publish_trot()

    def _compute_angles(self) -> list[float]:
        """Return flat list of 12 joint angles in LEGS order."""
        angles: list[float] = []
        for prefix in LEGS:
            x_sign, y_sign = LEG_SIGNS[prefix]

            # True position of the foot relative to body center
            body_x = x_sign * HIP_X_OFFSET + self._x_stand
            body_y = y_sign * (HIP_Y_OFFSET + HIP_LENGTH)
            
            # Linear velocity required at the foot
            v_x = self._vx - body_y * self._omega
            v_y = self._vy + body_x * self._omega
            
            # Required stroke (amplitude)
            leg_step_x = v_x * (STRIDE_PERIOD / 2.0)
            leg_step_y = v_y * (STRIDE_PERIOD / 2.0)
            
            # Cap the stroke to avoid IK failure
            leg_step_x = max(-MAX_STEP_LENGTH, min(MAX_STEP_LENGTH, leg_step_x))
            leg_step_y = max(-MAX_LATERAL, min(MAX_LATERAL, leg_step_y))

            # Local phase: A-pair starts at 0, B-pair offset by 0.5
            if prefix in SWING_A:
                leg_phase = self._phase
            else:
                leg_phase = (self._phase + 0.5) % 1.0

            in_swing = leg_phase < 0.5

            if in_swing:
                t = leg_phase / 0.5          # 0→1
                foot_x_off = leg_step_x * (t - 0.5)
                foot_y_off = leg_step_y * (t - 0.5)
                lift       = STEP_HEIGHT * math.sin(math.pi * t)
            else:
                t = (leg_phase - 0.5) / 0.5  # 0→1
                foot_x_off = leg_step_x * (0.5 - t)
                foot_y_off = leg_step_y * (0.5 - t)
                lift       = 0.0

            # Nominal foot position relative to hip in X, and relative to body center in Y/Z 
            nom_x = self._x_stand
            nom_y = y_sign * (HIP_Y_OFFSET + HIP_LENGTH)
            nom_z = self._z_stand

            hr, hp, kn = leg_ik(
                nom_x + foot_x_off,
                nom_y + foot_y_off,
                nom_z  + lift,
                y_sign,
            )
            angles += [hr, hp, kn]
        return angles

    def _publish_trot(self):
        msg = Float64MultiArray()
        msg.data = self._compute_angles()
        self._pub.publish(msg)

    def _publish_stand(self):
        msg = Float64MultiArray()
        msg.data = [
            STAND_HIP_ROLL, STAND_HIP_PITCH, STAND_KNEE,  # FL
            STAND_HIP_ROLL, STAND_HIP_PITCH, STAND_KNEE,  # FR
            STAND_HIP_ROLL, STAND_HIP_PITCH, STAND_KNEE,  # RL
            STAND_HIP_ROLL, STAND_HIP_PITCH, STAND_KNEE,  # RR
        ]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
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
