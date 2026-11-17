#!/usr/bin/env python3
"""
Dummy keyboard controller for the quadruped robot.

Reads raw keystrokes from stdin (works inside a Docker TTY without X11).

Keys:
  W / S - move forward / backward
  A / D - strafe left / right
  Q / E - turn left / right
  ESC / Ctrl+C - quit

Publishes geometry_msgs/Twist to /cmd_vel
"""

import sys
import os
import time
import tty
import termios
import threading
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Velocity constants
LINEAR_SPEED = 0.3   # m/s forward / backward
LATERAL_SPEED = 0.2  # m/s strafing
ANGULAR_SPEED = 1.0  # rad/s turning
PUBLISH_RATE_HZ = 20

# Terminal auto-repeat delivers the first char immediately, then nothing for
# ~500 ms before repeats start. The hold timeout must exceed that gap or the
# command flickers to zero on every fresh key press.
HOLD_TIMEOUT = 0.6        # s without input before the command is zeroed
STOP_PUBLISH_DURATION = 0.5  # s of zero Twists after a release, then go silent
                             # (so an idle teleop doesn't fight Nav2 on /cmd_vel)

KEY_MAP = {
    'w': ( LINEAR_SPEED,  0.0, 0.0),
    's': (-LINEAR_SPEED,  0.0, 0.0),
    'a': (0.0,  LATERAL_SPEED, 0.0),
    'd': (0.0, -LATERAL_SPEED, 0.0),
    'q': (0.0, 0.0,  ANGULAR_SPEED),
    'e': (0.0, 0.0, -ANGULAR_SPEED),
}


def get_key(timeout=0.05):
    """Read a single keypress from stdin with a timeout. Returns '' on timeout."""
    if not sys.stdin.isatty():
        return ''
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return ''


class KeyboardControllerNode(Node):
    """Publishes Twist commands based on W/A/S/D key presses."""

    def __init__(self):
        super().__init__('keyboard_controller')

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_cmd)

        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_z = 0.0
        self._last_key_time = 0.0
        self._stop_publish_until = 0.0
        self._lock = threading.Lock()
        self._running = True

        self.get_logger().info(
            "\n"
            "==========================================\n"
            "  Quadruped Test Keyboard Controller\n"
            "==========================================\n"
            "  W - Forward\n"
            "  S - Backward\n"
            "  A - Strafe Left\n"
            "  D - Strafe Right\n"
            "  Q - Turn Left\n"
            "  E - Turn Right\n"
            "  ESC / Ctrl+C - Quit\n"
            "==========================================\n"
            "  Publishing to: /cmd_vel\n"
            "  Keep this terminal focused!\n"
            "==========================================\n"
        )

        # Start the key-reading loop in a background thread
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

    def _key_loop(self):
        """Continuously reads keypresses and updates velocity state."""
        if not sys.stdin.isatty():
            self.get_logger().warn(
                "stdin is not a TTY — keyboard input disabled. "
                "Run with 'docker compose up' (not detached) or "
                "'docker run -it'."
            )
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)   # cbreak: chars available immediately, Ctrl+C still works
            while self._running:
                key = get_key(timeout=0.05)

                if not key:
                    # No release events exist: zero the command only after the
                    # hold timeout, so the auto-repeat delay doesn't cut it.
                    with self._lock:
                        active = (self._linear_x or self._linear_y
                                  or self._angular_z)
                        if active and time.monotonic() - self._last_key_time > HOLD_TIMEOUT:
                            self._set_command(0.0, 0.0, 0.0)
                    continue

                if key == '\x1b':
                    # Bare ESC quits; ESC followed by more bytes is an escape
                    # sequence (arrow keys etc.) — drain and ignore it.
                    seq = get_key(timeout=0.02)
                    if seq:
                        while get_key(timeout=0.005):
                            pass
                        continue
                    self._quit()
                    break

                if key == '\x03':
                    self._quit()
                    break

                key_lower = key.lower()
                with self._lock:
                    if key_lower in KEY_MAP:
                        self._set_command(*KEY_MAP[key_lower])
                    else:
                        self._set_command(0.0, 0.0, 0.0)

        except Exception as e:
            self.get_logger().error(f'Key loop error: {e}')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _set_command(self, lx, ly, az):
        """Update the velocity command. Caller must hold self._lock."""
        self._linear_x = lx
        self._linear_y = ly
        self._angular_z = az
        self._last_key_time = time.monotonic()

    def _quit(self):
        self.get_logger().info('Quit key pressed — shutting down.')
        with self._lock:
            self._set_command(0.0, 0.0, 0.0)
        try:
            self._pub.publish(Twist())  # explicit stop before the timer dies
        except Exception:
            pass
        self._running = False
        rclpy.shutdown()

    def _publish_cmd(self):
        msg = Twist()
        now = time.monotonic()
        with self._lock:
            msg.linear.x = self._linear_x
            msg.linear.y = self._linear_y
            msg.angular.z = self._angular_z
        if msg.linear.x or msg.linear.y or msg.angular.z:
            self._stop_publish_until = now + STOP_PUBLISH_DURATION
        elif now >= self._stop_publish_until:
            return  # idle: stay off /cmd_vel so other publishers can own it
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node._running = False
        if rclpy.ok():
            try:
                node._pub.publish(Twist())  # stop the robot on Ctrl+C
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
