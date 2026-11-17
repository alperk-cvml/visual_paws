"""IMU attitude sub-component: Mahony-style complementary filter.

The Gazebo IMU conveniently supplies an absolute orientation quaternion, but
the real ICM-42688-class part measures only angular velocity and linear
acceleration. This filter makes the leg-odometry node hardware-ready: it
integrates the gyro and corrects roll/pitch toward the gravity direction
measured by the accelerometer. Yaw is gyro-only and drifts — downstream
consumers must treat heading as odometric, not absolute.

Quaternions are [w, x, y, z], rotating base_link vectors into the odom frame.
"""

import math

import numpy as np

GRAVITY = 9.80665  # m/s^2 (REP-103)


def quat_mult(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def _quat_exp(v: np.ndarray) -> np.ndarray:
    """Quaternion for a rotation vector v (axis * angle)."""
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.array([1.0, *(0.5 * v)])
    axis = v / angle
    half = 0.5 * angle
    return np.array([math.cos(half), *(math.sin(half) * axis)])


class ComplementaryAttitude:
    """Gyro integration with accelerometer tilt correction (Mahony kP term)."""

    def __init__(self, accel_gain: float = 0.5, accel_reject: float = 0.3):
        """
        Args:
            accel_gain: proportional pull (1/s) of roll/pitch toward the
                accelerometer gravity direction. Higher converges faster but
                lets locomotion accelerations disturb the attitude more.
            accel_reject: fractional deviation of |accel| from g beyond which
                the accelerometer correction is skipped for that sample.
        """
        self._kp = accel_gain
        self._reject = accel_reject
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> np.ndarray:
        """Advance the attitude by one IMU sample; returns the new quaternion."""
        omega = np.asarray(gyro, dtype=float)

        accel = np.asarray(accel, dtype=float)
        accel_norm = float(np.linalg.norm(accel))
        if abs(accel_norm - GRAVITY) < self._reject * GRAVITY:
            # Measured vs predicted "up" direction in the body frame. A resting
            # accelerometer reads +g along body-up (specific force).
            up_meas = accel / accel_norm
            up_pred = quat_to_rot(self.q).T @ np.array([0.0, 0.0, 1.0])
            omega = omega + self._kp * np.cross(up_meas, up_pred)

        # q_dot = 0.5 * q ⊗ (0, omega)  →  right-multiply by the body-rate step
        self.q = quat_mult(self.q, _quat_exp(omega * dt))
        self.q /= np.linalg.norm(self.q)
        return self.q
