#!/usr/bin/env python3
"""pointcloud_scale_correction — rescale monocular depth point clouds.

Closes the loop opened by `ground_plane_estimation`: that module *measures*
the monocular depth scale error per frame; this one *applies* the correction
to the point cloud before it reaches consumers (elevation mapping, ORB-SLAM3
via depth, planning):

  /depth/points ──────────────┐
                              ▼
  /ground_plane/scale ──► CorrectionModel ──► /depth/points_corrected
  (/ground_plane/shift,       │
   future affine estimator) ──┘

The error-model math lives in `correction_models.py` (ROS-free). The node
only moves data: it holds the latest smoothed model state, corrects each
cloud, and republishes it with the ORIGINAL header — same frame_id, same
stamp, per the repo TF rule (never restamp or reframe a cloud).

Model choice is a parameter (`model:=scalar|affine`), so if the ground-plane
experiment shows the error is affine rather than a pure scale, switching is
a config change plus a shift estimate — no pipeline rewiring.

When no (or only stale) estimates are available, `stale_policy` decides:
  * hold        — keep applying the last good estimate (default)
  * passthrough — republish the cloud uncorrected
  * drop        — publish nothing (consumers see a gap, not wrong data)
Until the FIRST estimate arrives, `hold` behaves like `drop` — there is
nothing to hold yet, and silently passing scaled-wrong clouds through would
defeat the module's purpose.

One node instance handles one cloud; the launch file spawns one node per
input topic (mirroring ground_plane.launch.py) so every depth-comparison
channel can get its own corrected twin.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32

from .correction_models import MODELS, ScaleObservation, make_model

STALE_POLICIES = ('hold', 'passthrough', 'drop')


class ScaleCorrectionNode(Node):
    """Applies a depth error model to one point cloud stream."""

    def __init__(self):
        super().__init__('scale_correction')

        self.declare_parameter('input_topic', '/depth_camera/points')
        # '' derives `<input_topic>_corrected`.
        self.declare_parameter('output_topic', '')
        # Estimate sources. `scale_topic` is required; `shift_topic` is
        # optional and only meaningful for the affine model (no estimator
        # publishes a depth-space shift yet — odometry_scale_estimation's
        # /odom_scale/shift is a displacement-space diagnostic, see its
        # README and correction_models.AffineCorrection).
        self.declare_parameter('scale_topic', '/ground_plane/scale')
        self.declare_parameter('shift_topic', '')
        # Error model: one of correction_models.MODELS.
        self.declare_parameter('model', 'scalar')
        # Observation smoothing (per parameter, inside the model).
        self.declare_parameter('smoothing', 'ema')  # none | ema | median
        self.declare_parameter('ema_alpha', 0.2)
        self.declare_parameter('median_window', 5)
        # Observation gate: scales outside this range are bad plane fits.
        self.declare_parameter('min_scale', 0.05)
        self.declare_parameter('max_scale', 20.0)
        # Estimates older than this (s) are stale → stale_policy applies.
        # 0 disables the check (an estimate never goes stale).
        self.declare_parameter('estimate_timeout', 5.0)
        self.declare_parameter('stale_policy', 'hold')

        p = self.get_parameter
        model_name = str(p('model').value)
        if model_name not in MODELS:
            raise ValueError(f'model must be one of {sorted(MODELS)}, '
                             f'got {model_name!r}')
        self._model = make_model(
            model_name,
            smoothing=str(p('smoothing').value),
            ema_alpha=float(p('ema_alpha').value),
            median_window=int(p('median_window').value),
            min_scale=float(p('min_scale').value),
            max_scale=float(p('max_scale').value),
        )

        self._timeout = float(p('estimate_timeout').value)
        self._stale_policy = str(p('stale_policy').value)
        if self._stale_policy not in STALE_POLICIES:
            raise ValueError(f'stale_policy must be one of {STALE_POLICIES}, '
                             f'got {self._stale_policy!r}')

        self._last_shift = 0.0
        self._rejected = 0

        input_topic = str(p('input_topic').value)
        output_topic = str(p('output_topic').value) or \
            f'{input_topic}_corrected'
        if output_topic == input_topic:
            raise ValueError('output_topic must differ from input_topic')

        self._pub_cloud = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        # Currently-applied parameters, for rqt_plot next to the raw
        # /ground_plane/scale estimate.
        self._pub_applied = {
            key: self.create_publisher(
                Float32, f'{output_topic}/applied_{key}', 10)
            for key in self._model.state()
        }

        scale_topic = str(p('scale_topic').value)
        self.create_subscription(Float32, scale_topic, self._on_scale, 10)
        shift_topic = str(p('shift_topic').value)
        if shift_topic:
            self.create_subscription(Float32, shift_topic, self._on_shift, 10)
        self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data)

        self.get_logger().info(
            f'scale correction ({model_name}): {input_topic} -> '
            f'{output_topic}, estimates from {scale_topic}'
            + (f' + {shift_topic}' if shift_topic else ''))

    # ── observation intake ───────────────────────────────────────────────

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scale(self, msg: Float32):
        obs = ScaleObservation(
            stamp=self._now_sec(), scale=float(msg.data),
            shift=self._last_shift)
        if not self._model.update(obs):
            self._rejected += 1
            self.get_logger().warning(
                f'rejected scale observation {msg.data:.3f} '
                f'({self._rejected} total, gate '
                f'[{self._model._min_scale}, {self._model._max_scale}])',
                throttle_duration_sec=5.0)

    def _on_shift(self, msg: Float32):
        # Shift rides along with the next scale observation so both pass
        # through the same smoothing/staleness path.
        self._last_shift = float(msg.data)

    # ── cloud path ───────────────────────────────────────────────────────

    def _estimate_usable(self) -> bool:
        if not self._model.ready():
            return False
        if self._timeout <= 0.0:
            return True
        return (self._now_sec() - self._model.last_stamp) <= self._timeout

    def _on_cloud(self, msg: PointCloud2):
        if not self._estimate_usable():
            if self._stale_policy == 'passthrough':
                self._pub_cloud.publish(msg)
                return
            elif self._stale_policy == 'hold' and self._model.ready():
                pass  # fall through and apply the last good estimate
            else:  # drop, or hold with nothing to hold yet
                self.get_logger().warning(
                    'no usable scale estimate, dropping cloud',
                    throttle_duration_sec=5.0)
                return

        points = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        corrected = self._model.correct(
            points.astype(np.float32, copy=False))

        # Original header verbatim: same frame_id, same stamp (repo TF rule
        # — a corrected cloud is still the same measurement instant).
        out = point_cloud2.create_cloud_xyz32(
            msg.header, corrected.astype(np.float32, copy=False))
        self._pub_cloud.publish(out)

        for key, value in self._model.state().items():
            if value is not None:
                self._pub_applied[key].publish(Float32(data=float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = ScaleCorrectionNode()
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
