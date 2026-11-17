#!/usr/bin/env python3
"""
quadruped_gait_controller — Spot Micro gait node.

  /cmd_vel (geometry_msgs/Twist)
      │  clamp + acceleration limit + watchdog
      ▼
  BezierGait  →  footholds in the nominal body frame N
      │
      ▼
  body_ik (quadruped_kinematics)  →  12 joint angles
      │
      ▼
  /quadruped_joint_controller/commands (std_msgs/Float64MultiArray)

Joint order is quadruped_kinematics.JOINT_ORDER, which is the contract with
quadruped_gazebo/config/ros2_control.yaml:
    FL hip_roll, hip_pitch, knee, FR …, RL …, RR …

States
    STANDING_UP  ramp body height from crouch to the commanded stand
    STANDING     hold the stand pose, gait phase parked at 0
    WALKING      cycle the gait; on a zero command finish the current cycle
                 so every foot is back at its neutral foothold, then stand

Every geometric and gait parameter is a ROS parameter (see
config/gait_params.yaml). The link lengths come from quadruped_kinematics
and are not parameters: they are the robot.
"""

import math
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from quadruped_kinematics import SPOT_MICRO, body_ik, neutral_feet

from .gait import GAIT_LIBRARY, BezierGait, GaitParams


class State(Enum):
    STANDING_UP = 'standing_up'
    STANDING = 'standing'
    WALKING = 'walking'


def _smoothstep(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class GaitNode(Node):
    """Velocity command → joint positions via a Bézier-swing gait."""

    def __init__(self):
        super().__init__('gait_controller')

        # ── Parameters ──────────────────────────────────────────────────
        d = self.declare_parameter
        d('joint_command_topic', '/quadruped_joint_controller/commands')
        d('cmd_vel_topic', '/cmd_vel')
        d('control_rate', 100.0)          # Hz
        d('cmd_vel_timeout', 0.5)         # s; stale command → stop
        d('max_vx', 0.4)                  # m/s
        d('max_vy', 0.25)                 # m/s
        d('max_wz', 1.2)                  # rad/s
        d('linear_accel', 0.6)            # m/s²
        d('angular_accel', 2.5)           # rad/s²
        d('gait_type', 'trot')            # trot | walk
        # 0 (or negative) keeps the gait's tuned default from GAIT_LIBRARY.
        d('period', 0.0)                  # s per cycle
        d('duty', 0.0)                    # stance fraction
        d('swing_clearance', 0.0)         # m
        d('body_shift_gain', -1.0)        # 0..1, shift body toward support centroid
        d('body_shift_tau', 0.0)          # s, low-pass on that shift
        d('max_step_length', 0.10)        # m
        d('stance_depth', 0.0)            # m, push below ground while in stance
        d('body_height', 0.16)            # m, base_link above ground
        d('body_x', 0.0)                  # m, body shift over the feet
        d('body_y', 0.0)
        d('body_roll', 0.0)               # rad
        d('body_pitch', 0.0)
        d('body_yaw', 0.0)
        d('foot_x_offset', 0.0)           # m, shift all footholds fore/aft
        d('foot_y_offset', 0.0)           # m, widen the stance
        d('crouch_height', 0.09)          # m, start of the stand-up ramp
        d('stand_up_duration', 1.0)       # s

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._rate = float(g('control_rate'))
        self._timeout = float(g('cmd_vel_timeout'))
        self._max_v = (float(g('max_vx')), float(g('max_vy')), float(g('max_wz')))
        self._lin_acc = float(g('linear_accel'))
        self._ang_acc = float(g('angular_accel'))
        self._height = float(g('body_height'))
        self._body_xy = (float(g('body_x')), float(g('body_y')))
        self._rpy = (float(g('body_roll')), float(g('body_pitch')), float(g('body_yaw')))
        self._foot_x = float(g('foot_x_offset'))
        self._foot_y = float(g('foot_y_offset'))
        self._crouch = float(g('crouch_height'))
        self._stand_up_s = max(float(g('stand_up_duration')), 1e-3)

        gait_type = str(g('gait_type'))
        if gait_type not in GAIT_LIBRARY:
            raise ValueError(f'gait_type must be one of {sorted(GAIT_LIBRARY)}, got {gait_type!r}')
        def override(name):
            value = float(g(name))
            return value if value > 0.0 else None

        self._params = GaitParams.from_library(
            gait_type,
            period=override('period'),
            duty=override('duty'),
            clearance=override('swing_clearance'),
            body_shift_gain=(float(g('body_shift_gain'))
                             if float(g('body_shift_gain')) >= 0.0 else None),
            body_shift_tau=override('body_shift_tau'),
            max_step_length=float(g('max_step_length')),
            stance_depth=float(g('stance_depth')),
        )
        self._shift_gain = self._params.body_shift_gain
        self._shift_tau = max(self._params.body_shift_tau, 1e-3)
        self._neutral = neutral_feet(self._height, SPOT_MICRO, self._foot_x, self._foot_y)
        self._gait = BezierGait(self._params, self._neutral)

        # ── State ───────────────────────────────────────────────────────
        self._cmd = np.zeros(3)           # latest clamped (vx, vy, wz)
        self._vel = np.zeros(3)           # acceleration-limited command
        self._last_cmd_time = None
        self._phase = 0.0
        self._shift = np.zeros(2)         # low-passed body shift in N
        self._state = State.STANDING_UP
        self._t0 = self.get_clock().now()
        self._last_tick = self._t0

        self._pub = self.create_publisher(Float64MultiArray, g('joint_command_topic'), 10)
        self.create_subscription(Twist, g('cmd_vel_topic'), self._cmd_vel_cb, 10)
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            '\n'
            '============================================\n'
            '  Spot Micro gait controller\n'
            '============================================\n'
            f'  gait: {gait_type}  period {self._params.period:.2f} s  '
            f'duty {self._params.duty:.2f}  clearance {self._params.clearance*1e3:.0f} mm  '
            f'body shift {self._shift_gain:.1f}\n'
            f'  body height {self._height:.3f} m, step cap {self._params.max_step_length*1e3:.0f} mm\n'
            f'  cmd_vel: {g("cmd_vel_topic")}  →  {g("joint_command_topic")} @ {self._rate:.0f} Hz\n'
            '============================================'
        )

    # ── Inputs ──────────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        self._cmd = np.array([
            _clamp(float(msg.linear.x), self._max_v[0]),
            _clamp(float(msg.linear.y), self._max_v[1]),
            _clamp(float(msg.angular.z), self._max_v[2]),
        ])
        self._last_cmd_time = self.get_clock().now()

    # ── Control loop ────────────────────────────────────────────────────
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        dt = min(max(dt, 0.0), 0.1)   # sim-time jumps must not fling a foot

        if self._state is State.STANDING_UP:
            elapsed = (now - self._t0).nanoseconds * 1e-9
            s = _smoothstep(elapsed / self._stand_up_s)
            height = self._crouch + (self._height - self._crouch) * s
            self._publish(neutral_feet(height, SPOT_MICRO, self._foot_x, self._foot_y))
            if s >= 1.0:
                self._state = State.STANDING
            return

        # Watchdog: a dead teleop or planner must not latch the last command.
        if self._last_cmd_time is not None and \
                (now - self._last_cmd_time).nanoseconds * 1e-9 > self._timeout:
            self._cmd = np.zeros(3)

        self._ramp_velocity(dt)
        moving = bool(np.any(np.abs(self._vel) > 1e-3))

        if self._state is State.STANDING:
            if not moving:
                self._settle_shift(np.zeros(2), dt)
                self._publish(self._neutral)
                return
            self._state = State.WALKING
            self._phase = 0.0

        # WALKING
        step = dt / self._params.period
        if not moving and self._phase + step >= 1.0:
            # Cycle complete with zero velocity: all feet at neutral, so the
            # transition to a static stand is seamless.
            self._phase = 0.0
            self._state = State.STANDING
            self._settle_shift(np.zeros(2), dt)
            self._publish(self._neutral)
            return
        self._phase = (self._phase + step) % 1.0
        self._settle_shift(self._shift_gain * self._gait.support_centroid(self._phase), dt)
        self._publish(self._gait.feet(self._phase, *self._vel))

    def _settle_shift(self, target: np.ndarray, dt: float):
        alpha = min(dt / self._shift_tau, 1.0)
        self._shift = self._shift + alpha * (target - self._shift)

    def _ramp_velocity(self, dt: float):
        limits = np.array([self._lin_acc, self._lin_acc, self._ang_acc]) * dt
        delta = self._cmd - self._vel
        self._vel = self._vel + np.clip(delta, -limits, limits)

    def _publish(self, feet: dict):
        position = (self._body_xy[0] + self._shift[0], self._body_xy[1] + self._shift[1], 0.0)
        q = body_ik(feet, position, self._rpy, SPOT_MICRO)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q]
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
