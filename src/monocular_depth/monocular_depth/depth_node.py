"""
Metric3D V2 — metric monocular depth estimation node for ROS 2.

Subscribes to an RGB image topic and publishes a metric depth image (32FC1,
values in metres).

Metric3D V2 is **intrinsics-conditioned**: it predicts depth in a canonical
camera space with a fixed 1000 px focal length, and metric scale is recovered
afterwards from the real camera's focal length. That is why it replaced Depth
Anything V2 here — DA-V2's metric heads end in ``sigmoid(x) * max_depth``, so
their scale is inherited from a training domain and drifts badly off it, while
Metric3D takes its scale from the camera that actually took the picture. The
side-by-side evidence lives in ``src/depth_comparison`` (Phase 1 comparison of
DA-V2 VKITTI, DA-V2 Hypersim and Metric3D V2 on identical Gazebo frames).

Because scale comes from the camera, **this node cannot run without
CameraInfo**. Frames that arrive before the first CameraInfo are dropped with a
throttled warning rather than published at an arbitrary scale.

Published depth is consumed by:
  - depth_image_proc  → PointCloud2
  - ORB-SLAM3 RGBD    → loop-closure-corrected odometry  (Phase 2)
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import torch

#: Input resolution the ViT variants are trained at, as (height, width).
_CANONICAL_INPUT_SIZE = (616, 1064)

#: Focal length of the canonical camera Metric3D predicts in, in pixels.
_CANONICAL_FOCAL = 1000.0

#: ImageNet statistics in RGB order, as used upstream. The padding value is the
#: channel mean, so padded borders normalise to zero.
_MEAN = np.array([123.675, 116.28, 103.53], dtype=np.float32)
_STD = np.array([58.395, 57.12, 57.375], dtype=np.float32)


class DepthNode(Node):
    """ROS 2 node that runs Metric3D V2 metric inference on RGB images."""

    def __init__(self):
        super().__init__('depth_node')

        # ── Declare parameters ────────────────────────────────────────────
        self.declare_parameter('model_name', 'metric3d_vit_small')  # _vit_small | _vit_large | _vit_giant2
        self.declare_parameter('repo', 'yvanyin/metric3d')
        self.declare_parameter('max_depth', 300.0)          # clamp on the de-canonicalised output
        self.declare_parameter('input_image_topic', '/front_camera/image_raw')
        self.declare_parameter('input_camera_info_topic', '')  # empty = alongside the image topic
        self.declare_parameter('output_depth_topic', '/depth/image_raw')
        self.declare_parameter('output_camera_info_topic', '/depth/camera_info')
        self.declare_parameter('device', 'cuda')            # cuda | cpu
        self.declare_parameter('target_fps', 10.0)          # rate-limit inference

        # ── Read parameters ───────────────────────────────────────────────
        self.model_name = self.get_parameter('model_name').value
        self.repo = self.get_parameter('repo').value
        self.max_depth = self.get_parameter('max_depth').value
        input_topic = self.get_parameter('input_image_topic').value
        camera_info_in = self.get_parameter('input_camera_info_topic').value
        output_topic = self.get_parameter('output_depth_topic').value
        camera_info_topic = self.get_parameter('output_camera_info_topic').value
        device_str = self.get_parameter('device').value
        self.target_fps = self.get_parameter('target_fps').value

        # ── Device setup ──────────────────────────────────────────────────
        if device_str == 'cuda' and not torch.cuda.is_available():
            self.get_logger().warn('CUDA requested but not available — falling back to CPU')
            device_str = 'cpu'
        self.device = torch.device(device_str)
        self.get_logger().info(f'Using device: {self.device}')

        # ── Load model ────────────────────────────────────────────────────
        self.model = self._load_model()
        self.get_logger().info(
            f'Metric3D V2 loaded  |  model={self.model_name}  '
            f'max_depth={self.max_depth}m  device={self.device}  '
            f'(intrinsics-conditioned)'
        )

        # ── CV bridge ─────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Rate limiting ─────────────────────────────────────────────────
        self.min_interval = 1.0 / self.target_fps if self.target_fps > 0 else 0.0
        self.last_inference_time = 0.0
        self.frame_count = 0
        self.total_inference_time = 0.0

        # ── Latest camera info (metric scale + forwarded with depth) ──────
        self._latest_camera_info: CameraInfo | None = None

        # ── QoS: sensor data (best effort for camera streams) ─────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, input_topic, self._image_callback, sensor_qos
        )
        # The matching CameraInfo carries the focal length Metric3D needs to
        # de-canonicalise its prediction, and is re-published alongside the
        # depth image for depth_image_proc.
        if not camera_info_in:
            camera_info_in = input_topic.rsplit('/', 1)[0] + '/camera_info'
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_in, self._camera_info_callback, sensor_qos
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.depth_pub = self.create_publisher(Image, output_topic, sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, sensor_qos)

        self.get_logger().info(
            f'Subscribed to {input_topic} (+ {camera_info_in})  →  '
            f'publishing on {output_topic}'
        )

    # ──────────────────────────────────────────────────────────────────────
    # Model loading
    # ──────────────────────────────────────────────────────────────────────
    def _load_model(self):
        """Load the Metric3D V2 model from the torch hub cache."""
        self.get_logger().info(f'Loading {self.model_name} from {self.repo} via torch.hub')
        # Weights are pre-fetched into the torch hub cache at image build time,
        # so this resolves offline in the container.
        model = torch.hub.load(
            self.repo, self.model_name, pretrain=True, trust_repo=True
        )
        return model.to(self.device).eval()

    # ──────────────────────────────────────────────────────────────────────
    # Inference
    # ──────────────────────────────────────────────────────────────────────
    def _infer(self, bgr: np.ndarray, fx: float) -> np.ndarray:
        """Return metric depth in metres, float32, shaped like ``bgr``.

        The pre/post-processing steps (canonical resize, mean-value padding,
        ImageNet normalisation, un-pad, de-canonical rescale) follow the
        reference inference procedure from the upstream repo. Deviating from it
        silently breaks metric scale, so keep the steps in this order.

        The same procedure appears in ``depth_comparison``'s ``metric3d_v2``
        backend. The duplication is deliberate — the comparison harness is an
        experiment that will be retired, and this production node should not
        depend on it — but a fix to one belongs in the other.
        """
        h, w = bgr.shape[:2]
        rgb = bgr[:, :, ::-1]  # upstream normalisation statistics are RGB-ordered

        # ── Resize into canonical space, preserving aspect ratio ─────────────
        scale = min(_CANONICAL_INPUT_SIZE[0] / h, _CANONICAL_INPUT_SIZE[1] / w)
        resized = cv2.resize(
            rgb, (int(round(w * scale)), int(round(h * scale))),
            interpolation=cv2.INTER_LINEAR,
        )

        # ── Pad to the exact canonical size with the channel means ──────────
        rh, rw = resized.shape[:2]
        pad_h, pad_w = _CANONICAL_INPUT_SIZE[0] - rh, _CANONICAL_INPUT_SIZE[1] - rw
        pad_top, pad_left = pad_h // 2, pad_w // 2
        pad_bottom, pad_right = pad_h - pad_top, pad_w - pad_left
        padded = cv2.copyMakeBorder(
            resized, pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT, value=_MEAN.tolist(),
        )

        # ── Normalise and run ───────────────────────────────────────────────
        tensor = torch.from_numpy(
            ((padded.astype(np.float32) - _MEAN) / _STD).transpose(2, 0, 1)
        )[None].to(self.device)

        with torch.no_grad():
            pred_depth, _confidence, _outputs = self.model.inference({'input': tensor})

        # ── Undo the padding, then return to the source resolution ──────────
        pred_depth = pred_depth.squeeze()
        pred_depth = pred_depth[
            pad_top:pred_depth.shape[0] - pad_bottom,
            pad_left:pred_depth.shape[1] - pad_right,
        ]
        pred_depth = torch.nn.functional.interpolate(
            pred_depth[None, None], (h, w), mode='bilinear', align_corners=False,
        ).squeeze()

        # ── De-canonicalise: canonical focal 1000 px → this camera's focal ──
        # `fx` is for the source image; the same `scale` that took the image
        # into canonical space applies to the focal length.
        canonical_to_real = (fx * scale) / _CANONICAL_FOCAL
        pred_depth = torch.clamp(pred_depth * canonical_to_real, 0.0, self.max_depth)

        return pred_depth.cpu().numpy().astype(np.float32)

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────
    def _camera_info_callback(self, msg: CameraInfo):
        """Cache the latest CameraInfo — intrinsics source and depth companion."""
        self._latest_camera_info = msg

    def _image_callback(self, msg: Image):
        """Run Metric3D V2 inference on the incoming RGB image."""
        # Rate limiting — skip frame if we're ahead of the target FPS
        now = time.monotonic()
        if (now - self.last_inference_time) < self.min_interval:
            return

        info = self._latest_camera_info
        if info is None:
            self.get_logger().warn(
                'Metric3D recovers metric scale from the camera focal length; '
                'no CameraInfo has arrived yet — dropping frames until it does.',
                throttle_duration_sec=5.0,
            )
            return

        try:
            # Convert ROS Image → OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # The focal length only describes this frame if the CameraInfo was
        # calibrated at the image's resolution. A mismatch scales every metric
        # depth value by the same wrong factor, so say so loudly.
        h, w = cv_image.shape[:2]
        if (info.width, info.height) != (w, h):
            self.get_logger().warn(
                f'CameraInfo is {info.width}x{info.height} but the image is '
                f'{w}x{h}; the focal length does not describe this frame and '
                'metric depth will be scaled wrongly.',
                throttle_duration_sec=10.0,
            )

        # ── Inference ─────────────────────────────────────────────────
        t0 = time.monotonic()
        try:
            depth = self._infer(cv_image, fx=float(info.k[0]))
        except Exception as e:
            self.get_logger().error(f'Metric3D inference failed: {e}')
            return
        t1 = time.monotonic()

        self.last_inference_time = t1
        self.frame_count += 1
        self.total_inference_time += (t1 - t0)

        # Log throughput every 50 frames
        if self.frame_count % 50 == 0:
            avg_ms = (self.total_inference_time / self.frame_count) * 1000
            self.get_logger().info(
                f'Depth inference  |  frames={self.frame_count}  '
                f'avg={avg_ms:.1f}ms  ({1000/avg_ms:.1f} FPS)'
            )

        # ── Publish depth image ──────────────────────────────────────
        depth_msg = self.bridge.cv2_to_imgmsg(
            np.ascontiguousarray(depth, dtype=np.float32), encoding='32FC1'
        )
        depth_msg.header = msg.header  # preserve frame_id + timestamp
        self.depth_pub.publish(depth_msg)

        # Re-publish CameraInfo with the depth timestamp so depth_image_proc
        # can synchronize the two.
        info.header.stamp = msg.header.stamp
        info.header.frame_id = msg.header.frame_id
        self.camera_info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
