"""Side-by-side comparison of monocular metric depth models.

Runs every configured backend on the *same* RGB frame and publishes one metric
depth image per backend under a shared namespace:

    <output_namespace>/<key>/image_raw     32FC1, metres
    <output_namespace>/<key>/camera_info

The launch file fans each pair out to its own ``point_cloud_xyz_node``, giving
one point cloud per model in a common frame so they overlay directly in RViz.

Running the models in one process is deliberate: rate-limiting several separate
nodes would let them drift onto different source frames, and comparing depth
maps of different moments tells you nothing about the models.

All published depth images carry the **source image header verbatim** — same
timestamp, same frame_id. Downstream point clouds therefore land in the camera
optical frame with the source timestamp, as the TF contract requires.
"""

import time
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

import torch

from .backends import BACKEND_TYPES, Intrinsics, build_backend


class DepthComparisonNode(Node):
    """Fans one RGB stream out to N metric depth streams, one per backend."""

    def __init__(self):
        super().__init__('depth_comparison_node')

        # ── Shared parameters ─────────────────────────────────────────────
        self.declare_parameter('input_image_topic', '/front_camera/image_raw')
        self.declare_parameter('input_camera_info_topic', '/front_camera/camera_info')
        self.declare_parameter('output_namespace', '/depth_compare')
        self.declare_parameter('backends', ['gt', 'dav2_vkitti'])
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('target_fps', 5.0)
        self.declare_parameter('log_stats_every', 20)

        input_topic = self.get_parameter('input_image_topic').value
        camera_info_topic = self.get_parameter('input_camera_info_topic').value
        self.namespace = self.get_parameter('output_namespace').value.rstrip('/')
        backend_keys = list(self.get_parameter('backends').value)
        device_str = self.get_parameter('device').value
        target_fps = self.get_parameter('target_fps').value
        self.log_stats_every = int(self.get_parameter('log_stats_every').value)

        if not backend_keys:
            raise RuntimeError('No backends configured — set the `backends` parameter.')

        # ── Device ────────────────────────────────────────────────────────
        if device_str == 'cuda' and not torch.cuda.is_available():
            self.get_logger().warn('CUDA requested but unavailable — falling back to CPU')
            device_str = 'cpu'
        self.device = torch.device(device_str)

        self.bridge = CvBridge()
        self._latest_camera_info: CameraInfo | None = None

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Build backends ────────────────────────────────────────────────
        self.inference_backends = []   # (backend, depth_pub, info_pub)
        self.stats: dict[str, dict[str, Any]] = {}

        # A backend that fails to load is dropped rather than taking the node
        # down with it. Comparing three models is still useful when the fourth
        # has a broken dependency, and losing the whole comparison to one bad
        # checkpoint is the worse failure.
        failed: list[str] = []
        for key in backend_keys:
            try:
                backend = self._build_backend(key)
                if backend.consumes_rgb:
                    self.get_logger().info(f'Loading {backend.describe()}')
                    backend.load(self.device)
            except Exception as exc:
                failed.append(key)
                self.get_logger().error(
                    f'Backend "{key}" unavailable, dropping it from the '
                    f'comparison: {exc}'
                )
                continue

            depth_pub, info_pub = self._make_publishers(key)
            self.stats[key] = {'frames': 0, 'total_ms': 0.0, 'last': None}
            if backend.consumes_rgb:
                self.inference_backends.append((backend, depth_pub, info_pub))
            else:
                self.get_logger().info(f'Wiring {backend.describe()}')
                self._wire_relay(key, backend, depth_pub, info_pub)

        if not self.stats:
            raise RuntimeError(
                f'Every configured backend failed to load: {", ".join(failed)}'
            )
        if failed:
            self.get_logger().warn(
                f'Comparing {len(self.stats)} of {len(backend_keys)} sources; '
                f'unavailable: {", ".join(failed)}'
            )

        # ── Rate limiting ─────────────────────────────────────────────────
        self.min_interval = 1.0 / target_fps if target_fps > 0 else 0.0
        self.last_inference_time = 0.0
        self.frame_count = 0

        # ── Input subscriptions ───────────────────────────────────────────
        self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, self.sensor_qos
        )
        self.create_subscription(
            Image, input_topic, self._image_callback, self.sensor_qos
        )

        self.get_logger().info(
            f'Comparing {len(self.stats)} depth sources on {input_topic} '
            f'at up to {target_fps} Hz  →  {self.namespace}/<key>/image_raw'
        )

    # ──────────────────────────────────────────────────────────────────────
    # Setup helpers
    # ──────────────────────────────────────────────────────────────────────
    def _build_backend(self, key: str):
        """Declare a backend's parameters under ``<key>.`` and instantiate it."""
        self.declare_parameter(f'{key}.type', '')
        backend_type = self.get_parameter(f'{key}.type').value
        if not backend_type:
            raise RuntimeError(
                f'Backend "{key}" has no type; set {key}.type to one of '
                f'{sorted(BACKEND_TYPES)}'
            )
        if backend_type not in BACKEND_TYPES:
            raise RuntimeError(
                f'Backend "{key}" has unknown type "{backend_type}"; '
                f'expected one of {sorted(BACKEND_TYPES)}'
            )

        cfg = {}
        for name, default in BACKEND_TYPES[backend_type].param_defaults().items():
            self.declare_parameter(f'{key}.{name}', default)
            cfg[name] = self.get_parameter(f'{key}.{name}').value

        return build_backend(backend_type, key, cfg, self.get_logger())

    def _make_publishers(self, key: str) -> tuple:
        return (
            self.create_publisher(Image, f'{self.namespace}/{key}/image_raw', self.sensor_qos),
            self.create_publisher(CameraInfo, f'{self.namespace}/{key}/camera_info', self.sensor_qos),
        )

    def _wire_relay(self, key, backend, depth_pub, info_pub):
        """Republish an existing depth topic into the comparison namespace."""
        def on_depth(msg: Image):
            depth_pub.publish(msg)  # verbatim: never restamp or reframe
            self._record_relay_stats(key, msg)

        self.create_subscription(
            Image, backend.cfg['depth_topic'], on_depth, self.sensor_qos
        )
        self.create_subscription(
            CameraInfo, backend.cfg['camera_info_topic'],
            lambda msg: info_pub.publish(msg), self.sensor_qos,
        )

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────
    def _camera_info_callback(self, msg: CameraInfo):
        self._latest_camera_info = msg

    def _image_callback(self, msg: Image):
        now = time.monotonic()
        if (now - self.last_inference_time) < self.min_interval:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        info = self._latest_camera_info
        intrinsics = Intrinsics.from_camera_info(info) if info is not None else None

        for backend, depth_pub, info_pub in self.inference_backends:
            t0 = time.monotonic()
            try:
                depth = backend.infer(bgr, intrinsics)
            except Exception as exc:
                self.get_logger().warn(
                    f'[{backend.key}] inference failed: {exc}', throttle_duration_sec=5.0
                )
                continue
            elapsed_ms = (time.monotonic() - t0) * 1000.0

            if depth.shape[:2] != bgr.shape[:2]:
                self.get_logger().warn(
                    f'[{backend.key}] returned {depth.shape[:2]} for a '
                    f'{bgr.shape[:2]} frame; the RGB CameraInfo will not match '
                    'and the point cloud will be distorted.',
                    throttle_duration_sec=10.0,
                )

            depth_msg = self.bridge.cv2_to_imgmsg(
                np.ascontiguousarray(depth, dtype=np.float32), encoding='32FC1'
            )
            depth_msg.header = msg.header  # source timestamp + frame_id, unchanged
            depth_pub.publish(depth_msg)

            if info is not None:
                info.header = msg.header
                info_pub.publish(info)

            self._record_inference_stats(backend.key, depth, elapsed_ms)

        self.last_inference_time = time.monotonic()
        self.frame_count += 1
        if self.log_stats_every > 0 and self.frame_count % self.log_stats_every == 0:
            self._log_stats()

    # ──────────────────────────────────────────────────────────────────────
    # Comparison statistics
    # ──────────────────────────────────────────────────────────────────────
    @staticmethod
    def _summarise(depth: np.ndarray) -> tuple[float, float, float] | None:
        """Median and 5th/95th percentile over valid pixels, in metres."""
        valid = depth[np.isfinite(depth) & (depth > 0.0)]
        if valid.size == 0:
            return None
        return (
            float(np.percentile(valid, 5)),
            float(np.median(valid)),
            float(np.percentile(valid, 95)),
        )

    def _record_inference_stats(self, key, depth, elapsed_ms):
        entry = self.stats[key]
        entry['frames'] += 1
        entry['total_ms'] += elapsed_ms
        entry['last'] = self._summarise(depth)

    def _record_relay_stats(self, key, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception:
            return
        entry = self.stats[key]
        entry['frames'] += 1
        entry['last'] = self._summarise(np.asarray(depth))

    def _log_stats(self):
        """Log a per-backend depth distribution table.

        The percentiles are the quickest read on the scale problem: a model
        whose p05/median/p95 sit well above the reference channel's is
        overestimating near-field geometry, which is what blows up the
        accumulated elevation map.
        """
        lines = [f'depth comparison over {self.frame_count} frames',
                 f'  {"backend":<18} {"p05":>8} {"median":>8} {"p95":>8}  {"avg ms":>8}']
        for key, entry in self.stats.items():
            summary = entry['last']
            if summary is None:
                lines.append(f'  {key:<18} {"— no valid pixels —":>36}')
                continue
            p05, median, p95 = summary
            avg_ms = entry['total_ms'] / entry['frames'] if entry['frames'] else 0.0
            timing = f'{avg_ms:8.1f}' if avg_ms > 0 else f'{"relay":>8}'
            lines.append(
                f'  {key:<18} {p05:8.2f} {median:8.2f} {p95:8.2f}  {timing}'
            )
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = DepthComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
