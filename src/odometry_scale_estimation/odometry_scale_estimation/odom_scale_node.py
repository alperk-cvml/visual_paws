#!/usr/bin/env python3
"""odometry_scale_estimation — odometry-vs-odometry scale probe.

The second "solve depth kinetically" probe (Alternative 2, alongside the
ground-plane probe of Alternative 1): compare the motion reported by a
metric odometry source against the motion reported by a scale-bearing one,
over the last few seconds, and the depth scale factor falls out:

  reference odometry  ─┐  (metric truth: leg kinematics + IMU, /odom_lko)
                       ├──► sliding window ──► matched chords ──► fit
  estimate odometry   ─┘  (scale rides along: KISS-ICP on the monodepth
                           cloud, /odom_lidar)
                                │
        /odom_scale/scale       s in chord_est ≈ s·chord_ref (+ b)
        /odom_scale/shift       b (affine model only; displacement-space)
        /odom_scale/residual    fit RMS (m) — hypothesis quality
        /odom_scale/ref_motion  longest reference chord (m) — excitation
        /odom_scale/marker      both window trajectories overlaid (RViz)

Chord lengths (straight-line displacement between two instants) are
invariant to each odometry's arbitrary origin and orientation, so the two
streams need no extrinsic calibration — only time alignment, which comes
from the message stamps (`time_offset` absorbs a known constant lag).
Both streams must track the same rigid body point (`base_link` under the
vehicle interface contract).

The fit math lives in `trajectory_compare.py` (ROS-free). Model choice is a
parameter (`model:=scalar|affine`) mirroring `pointcloud_scale_correction`:
scalar is the working hypothesis, affine the suspected fallback — if the
scalar fit's residual stays high or the affine intercept is consistently
non-zero, the error is not a pure scale.

Estimates are published raw, per window; smoothing and gating are the
consumer's job (`pointcloud_scale_correction` already does both), same as
the ground-plane probe.

One node instance compares one estimate stream against the reference; the
launch file spawns one node per estimate topic so several odometry sources
can be probed side by side under /odom_scale/<key>/*.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from .trajectory_compare import (
    MODELS, TrajectoryBuffer, align_estimate, chord_pairs, make_model,
    motion_in_body,
)

#: window displacements shorter than this (m) get no arrow (RViz rejects
#: zero-length arrows; a sub-millimetre one is standing-still noise anyway)
MIN_ARROW_LENGTH = 1e-3


class OdomScaleNode(Node):
    """Windowed scale estimate between two nav_msgs/Odometry streams."""

    def __init__(self):
        super().__init__('odom_scale')

        # Streams. reference = metric truth; estimate = carries the scale.
        self.declare_parameter('reference_topic', '/odom_lko')
        self.declare_parameter('estimate_topic', '/odom_lidar')
        self.declare_parameter('output_namespace', '/odom_scale')
        # Error model: one of trajectory_compare.MODELS.
        self.declare_parameter('model', 'scalar')
        # Comparison window (s) and chord construction.
        self.declare_parameter('window_duration', 5.0)
        self.declare_parameter('min_pair_dt', 0.5)
        self.declare_parameter('max_samples', 40)
        # Reference interpolation validity: never interpolate across a gap
        # longer than this (s) — a dropout is not motion.
        self.declare_parameter('max_interp_gap', 0.5)
        # A position step larger than this (m) between consecutive samples
        # means the odometry re-initialised; the buffer restarts.
        self.declare_parameter('max_position_jump', 1.0)
        # Constant lag of the estimate stream's stamps (s), added to them.
        self.declare_parameter('time_offset', 0.0)
        # Fit gates (see trajectory_compare.ScaleModel).
        self.declare_parameter('min_motion', 0.1)
        self.declare_parameter('min_pairs', 3)
        self.declare_parameter('min_scale', 0.02)
        self.declare_parameter('max_scale', 50.0)
        self.declare_parameter('min_spread', 0.05)  # affine model only
        # Output cadence and staleness.
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('stale_timeout', 5.0)  # 0 disables
        self.declare_parameter('publish_marker', True)
        # Frame the reference odometry poses live in (trajectory overlay).
        self.declare_parameter('marker_frame', 'odom')
        # Robot body frame the motion-vector arrows originate from.
        self.declare_parameter('base_frame', 'base_link')

        p = self.get_parameter
        model_name = str(p('model').value)
        if model_name not in MODELS:
            raise ValueError(f'model must be one of {sorted(MODELS)}, '
                             f'got {model_name!r}')
        model_kwargs = {
            'min_motion': float(p('min_motion').value),
            'min_pairs': int(p('min_pairs').value),
            'min_scale': float(p('min_scale').value),
            'max_scale': float(p('max_scale').value),
        }
        if model_name == 'affine':
            model_kwargs['min_spread'] = float(p('min_spread').value)
        self._model = make_model(model_name, **model_kwargs)

        window = float(p('window_duration').value)
        max_gap = float(p('max_interp_gap').value)
        max_jump = float(p('max_position_jump').value)
        self._ref_buffer = TrajectoryBuffer(window, max_gap, max_jump)
        self._est_buffer = TrajectoryBuffer(window, max_gap, max_jump)
        self._min_pair_dt = float(p('min_pair_dt').value)
        self._max_samples = int(p('max_samples').value)
        self._time_offset = float(p('time_offset').value)
        self._stale_timeout = float(p('stale_timeout').value)
        self._marker_frame = str(p('marker_frame').value)
        self._base_frame = str(p('base_frame').value)
        self._period = 1.0 / max(float(p('publish_rate').value), 1e-3)
        # Latest pose orientation per stream (w, x, y, z): rotates that
        # stream's window displacement into its own body frame, so the
        # motion arrows share the base_link origin.
        self._ref_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self._est_orientation = np.array([1.0, 0.0, 0.0, 0.0])

        ns = str(p('output_namespace').value).rstrip('/')
        self._pub_scale = self.create_publisher(Float32, f'{ns}/scale', 10)
        # Displacement-space intercept — a diagnostic, not (numerically) the
        # depth-space shift of pointcloud_scale_correction. Affine only.
        self._pub_shift = (
            self.create_publisher(Float32, f'{ns}/shift', 10)
            if model_name == 'affine' else None
        )
        self._pub_residual = self.create_publisher(
            Float32, f'{ns}/residual', 10)
        self._pub_ref_motion = self.create_publisher(
            Float32, f'{ns}/ref_motion', 10)
        self._pub_marker = (
            self.create_publisher(MarkerArray, f'{ns}/marker', 10)
            if bool(p('publish_marker').value) else None
        )

        ref_topic = str(p('reference_topic').value)
        est_topic = str(p('estimate_topic').value)
        self.create_subscription(
            Odometry, ref_topic, self._on_reference, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, est_topic, self._on_estimate, qos_profile_sensor_data)
        self.create_timer(self._period, self._on_timer)

        self.get_logger().info(
            f'odometry scale estimation ({model_name}): '
            f'{est_topic} vs {ref_topic} -> {ns}/* '
            f'(window {window:.1f}s)')

    # ── stream intake ────────────────────────────────────────────────────

    @staticmethod
    def _stamp_sec(msg: Odometry) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    @staticmethod
    def _position(msg: Odometry) -> np.ndarray:
        pos = msg.pose.pose.position
        return np.array([pos.x, pos.y, pos.z])

    @staticmethod
    def _orientation(msg: Odometry) -> np.ndarray | None:
        """Pose orientation as (w, x, y, z); None unless a unit quaternion
        (an all-zero quaternion is an uninitialised odometry message)."""
        q = msg.pose.pose.orientation
        arr = np.array([q.w, q.x, q.y, q.z])
        return arr if abs(arr @ arr - 1.0) < 0.01 else None

    def _on_reference(self, msg: Odometry):
        self._ref_buffer.append(self._stamp_sec(msg), self._position(msg))
        q = self._orientation(msg)
        if q is not None:
            self._ref_orientation = q

    def _on_estimate(self, msg: Odometry):
        self._est_buffer.append(
            self._stamp_sec(msg) + self._time_offset, self._position(msg))
        q = self._orientation(msg)
        if q is not None:
            self._est_orientation = q

    # ── windowed fit ─────────────────────────────────────────────────────

    def _on_timer(self):
        ref_latest = self._ref_buffer.latest_time
        est_latest = self._est_buffer.latest_time
        if ref_latest is None or est_latest is None:
            self.get_logger().warning(
                'waiting for both odometry streams '
                f'(reference: {"yes" if ref_latest is not None else "no"}, '
                f'estimate: {"yes" if est_latest is not None else "no"})',
                throttle_duration_sec=5.0)
            return

        # Anchor the window on the newest COMMON time so a lagging stream
        # shrinks the overlap instead of invalidating the interpolation.
        t_end = min(ref_latest, est_latest)
        if self._stale_timeout > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - t_end > self._stale_timeout:
                self.get_logger().warning(
                    f'newest common sample is {now - t_end:.1f}s old, '
                    'skipping', throttle_duration_sec=5.0)
                return

        # Sample grid = the estimate's own stamps (the sparser stream);
        # the reference is interpolated onto it.
        est_times, est_pts = self._est_buffer.snapshot(t_end)
        ref_pts, valid = self._ref_buffer.interpolate(est_times)
        est_times, est_pts, ref_pts = \
            est_times[valid], est_pts[valid], ref_pts[valid]

        d_ref, d_est = chord_pairs(
            est_times, ref_pts, est_pts,
            min_dt=self._min_pair_dt, max_samples=self._max_samples)
        ref_motion = float(d_ref.max()) if len(d_ref) else 0.0
        # Excitation goes out even when the fit doesn't — a scale flatline
        # with ref_motion near zero reads "standing", not "healthy".
        self._pub_ref_motion.publish(Float32(data=ref_motion))

        fit = self._model.fit(d_ref, d_est)
        if fit is None:
            self.get_logger().warning(
                f'no usable scale fit ({len(d_ref)} pairs, reference '
                f'motion {ref_motion:.3f}m in window)',
                throttle_duration_sec=5.0)
            return

        self._pub_scale.publish(Float32(data=fit.scale))
        if self._pub_shift is not None and fit.shift is not None:
            self._pub_shift.publish(Float32(data=fit.shift))
        self._pub_residual.publish(Float32(data=fit.rms))

        if self._pub_marker is not None and len(est_times) >= 2:
            self._publish_marker(ref_pts, est_pts)

    # ── visualization ────────────────────────────────────────────────────

    def _marker_base(self, marker_id: int, frame: str,
                     rgb: tuple[float, float, float]) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame
        marker.ns = 'odom_scale'
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=3.0 * self._period).to_msg()
        return marker

    def _line_strip(self, marker_id: int, pts: np.ndarray,
                    rgb: tuple[float, float, float]) -> Marker:
        marker = self._marker_base(marker_id, self._marker_frame, rgb)
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.02
        marker.points = [
            Point(x=float(x), y=float(y), z=float(z)) for x, y, z in pts]
        return marker

    def _motion_arrow(self, marker_id: int, vec: np.ndarray,
                      rgb: tuple[float, float, float]) -> Marker:
        """Arrow from the base_link origin along a body-frame motion vector.

        Start/end-point form: the arrow's direction and length ARE the
        vector, no orientation quaternion involved."""
        marker = self._marker_base(marker_id, self._base_frame, rgb)
        marker.type = Marker.ARROW
        marker.scale.x, marker.scale.y, marker.scale.z = 0.03, 0.06, 0.08
        marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(vec[0]), y=float(vec[1]), z=float(vec[2])),
        ]
        return marker

    def _publish_marker(self, ref_pts: np.ndarray, est_pts: np.ndarray):
        """Trajectory overlay + motion-vector arrows, for RViz.

        Overlay (in `marker_frame`): the estimate window is rigidly aligned
        (rotation + translation, never scaled) onto the reference, so a
        scale error is visible as one track outrunning the other.

        Arrows (from the `base_frame` origin): each stream's displacement
        over the window, rotated into the body frame by that stream's own
        attitude — direction and length are the motion vector, so the scale
        error is the length ratio and a heading disagreement is the angle
        between the arrows. Near-zero displacements get no arrow.
        """
        markers = [
            self._line_strip(0, ref_pts, (0.1, 0.8, 0.2)),    # reference
            self._line_strip(1, align_estimate(ref_pts, est_pts),
                             (0.9, 0.4, 0.1)),                # estimate
        ]
        for marker_id, pts, quat, rgb in (
                (2, ref_pts, self._ref_orientation, (0.1, 0.8, 0.2)),
                (3, est_pts, self._est_orientation, (0.9, 0.4, 0.1))):
            vec = motion_in_body(pts[-1] - pts[0], quat)
            if np.linalg.norm(vec) >= MIN_ARROW_LENGTH:
                markers.append(self._motion_arrow(marker_id, vec, rgb))
        self._pub_marker.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = OdomScaleNode()
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
