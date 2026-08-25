"""
Depth Anything V2 — metric monocular depth estimation node for ROS 2.

Subscribes to an RGB image topic and publishes a metric depth image (32FC1,
values in metres) using the Depth Anything V2 model with outdoor (Virtual
KITTI) weights.

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


class DepthNode(Node):
    """ROS 2 node that runs Depth Anything V2 metric inference on RGB images."""

    def __init__(self):
        super().__init__('depth_node')

        # ── Declare parameters ────────────────────────────────────────────
        self.declare_parameter('model_type', 'vits')        # vits | vitb | vitl
        self.declare_parameter('encoder', 'vits')           # alias kept for compat
        self.declare_parameter('max_depth', 80.0)           # outdoor (vkitti)
        self.declare_parameter('input_image_topic', '/front_camera/image_raw')
        self.declare_parameter('output_depth_topic', '/depth/image_raw')
        self.declare_parameter('output_camera_info_topic', '/depth/camera_info')
        self.declare_parameter('device', 'cuda')            # cuda | cpu
        self.declare_parameter('target_fps', 10.0)          # rate-limit inference
        self.declare_parameter('model_path', '')             # empty = auto-resolve

        # ── Read parameters ───────────────────────────────────────────────
        self.model_type = self.get_parameter('model_type').value
        self.max_depth = self.get_parameter('max_depth').value
        input_topic = self.get_parameter('input_image_topic').value
        output_topic = self.get_parameter('output_depth_topic').value
        camera_info_topic = self.get_parameter('output_camera_info_topic').value
        device_str = self.get_parameter('device').value
        self.target_fps = self.get_parameter('target_fps').value
        model_path = self.get_parameter('model_path').value

        # ── Device setup ──────────────────────────────────────────────────
        if device_str == 'cuda' and not torch.cuda.is_available():
            self.get_logger().warn('CUDA requested but not available — falling back to CPU')
            device_str = 'cpu'
        self.device = torch.device(device_str)
        self.get_logger().info(f'Using device: {self.device}')

        # ── Load model ────────────────────────────────────────────────────
        self.model = self._load_model(model_path)
        self.get_logger().info(
            f'Depth Anything V2 loaded  |  encoder={self.model_type}  '
            f'max_depth={self.max_depth}m  device={self.device}'
        )

        # ── CV bridge ─────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Rate limiting ─────────────────────────────────────────────────
        self.min_interval = 1.0 / self.target_fps if self.target_fps > 0 else 0.0
        self.last_inference_time = 0.0
        self.frame_count = 0
        self.total_inference_time = 0.0

        # ── Latest camera info (forwarded with depth) ─────────────────────
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
        # Subscribe to the matching CameraInfo so we can re-publish it with
        # the depth image (needed by depth_image_proc).
        camera_info_in = input_topic.rsplit('/', 1)[0] + '/camera_info'
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_in, self._camera_info_callback, sensor_qos
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.depth_pub = self.create_publisher(Image, output_topic, sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, sensor_qos)

        self.get_logger().info(
            f'Subscribed to {input_topic}  →  publishing on {output_topic}'
        )

    # ──────────────────────────────────────────────────────────────────────
    # Model loading
    # ──────────────────────────────────────────────────────────────────────
    def _load_model(self, model_path: str):
        """Load the Depth Anything V2 metric depth model."""
        import sys
        # Ensure the metric_depth variant is imported (it has max_depth support).
        # The base repo's depth_anything_v2.dpt does NOT accept max_depth.
        metric_path = '/opt/depth_anything_v2/metric_depth'
        if metric_path not in sys.path:
            sys.path.insert(0, metric_path)
        from depth_anything_v2.dpt import DepthAnythingV2

        # Model configs per encoder variant
        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        }

        if self.model_type not in model_configs:
            self.get_logger().error(
                f'Unknown model_type "{self.model_type}". '
                f'Valid: {list(model_configs.keys())}'
            )
            raise ValueError(f'Unknown model_type: {self.model_type}')

        config = model_configs[self.model_type]
        model = DepthAnythingV2(**config, max_depth=self.max_depth)

        # Resolve checkpoint path
        if not model_path:
            model_path = f'/models/depth_anything_v2_metric_vkitti_{self.model_type}.pth'

        self.get_logger().info(f'Loading weights from {model_path}')
        state_dict = torch.load(model_path, map_location=self.device, weights_only=True)
        model.load_state_dict(state_dict)
        model = model.to(self.device).eval()

        return model

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────
    def _camera_info_callback(self, msg: CameraInfo):
        """Cache the latest CameraInfo for re-publication with depth frames."""
        self._latest_camera_info = msg

    def _image_callback(self, msg: Image):
        """Run Depth Anything V2 inference on the incoming RGB image."""
        # Rate limiting — skip frame if we're ahead of the target FPS
        now = time.monotonic()
        if (now - self.last_inference_time) < self.min_interval:
            return

        try:
            # Convert ROS Image → OpenCV BGR (model expects BGR input)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # ── Inference ─────────────────────────────────────────────────
        t0 = time.monotonic()
        with torch.no_grad():
            depth = self.model.infer_image(cv_image)  # returns numpy (H, W), float32, metres
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
        depth_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding='32FC1')
        depth_msg.header = msg.header  # preserve frame_id + timestamp
        self.depth_pub.publish(depth_msg)

        # Re-publish CameraInfo with the depth timestamp so depth_image_proc
        # can synchronize the two.
        if self._latest_camera_info is not None:
            info_msg = self._latest_camera_info
            info_msg.header.stamp = msg.header.stamp
            info_msg.header.frame_id = msg.header.frame_id
            self.camera_info_pub.publish(info_msg)


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
