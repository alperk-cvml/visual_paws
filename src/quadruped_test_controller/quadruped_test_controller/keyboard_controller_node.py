#!/usr/bin/env python3
"""
Dummy keyboard controller for the quadruped robot.

Reads raw keystrokes from stdin (works inside a Docker TTY without X11).

Keys:
  W - move forward
  S - move backward
  A - strafe left
  D - strafe right
  Q / ESC / Ctrl+C - quit

Publishes geometry_msgs/Twist to /cmd_vel
"""

import sys
import os
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
PUBLISH_RATE_HZ = 20

KEY_MAP = {
    'w': ( LINEAR_SPEED,  0.0),
    's': (-LINEAR_SPEED,  0.0),
    'a': (0.0,  LATERAL_SPEED),
    'd': (0.0, -LATERAL_SPEED),
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
            "  Q / ESC / Ctrl+C - Quit\n"
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
            prev_key = None
            while self._running:
                key = get_key(timeout=0.05)

                if not key:
                    # Timeout — if we had a key held, nothing changes (no release event)
                    # We send zeros whenever no key is active (handled via prev_key tracking)
                    if prev_key is not None:
                        with self._lock:
                            self._linear_x = 0.0
                            self._linear_y = 0.0
                        prev_key = None
                    continue

                key_lower = key.lower()

                # Quit conditions
                if key_lower == 'q' or key == '\x1b' or key == '\x03':
                    self.get_logger().info('Quit key pressed — shutting down.')
                    self._running = False
                    rclpy.shutdown()
                    break

                if key_lower in KEY_MAP:
                    lx, ly = KEY_MAP[key_lower]
                    with self._lock:
                        self._linear_x = lx
                        self._linear_y = ly
                    prev_key = key_lower
                else:
                    with self._lock:
                        self._linear_x = 0.0
                        self._linear_y = 0.0
                    prev_key = None

        except Exception as e:
            self.get_logger().error(f'Key loop error: {e}')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _publish_cmd(self):
        msg = Twist()
        with self._lock:
            msg.linear.x = self._linear_x
            msg.linear.y = self._linear_y
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
