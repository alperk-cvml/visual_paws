"""
Video Camera Simulator — plays pre-recorded video as a ROS 2 camera topic.

Reads frames from a video file (MP4/AVI/MKV) and publishes them as
sensor_msgs/Image + sensor_msgs/CameraInfo at a configurable rate.

This replaces the live Gazebo or hardware camera for offline testing of
the monocular_depth (Depth Anything V2) and localization (ORB-SLAM3)
pipelines using previously recorded footage.

Published topics:
  - /camera/image_raw      (sensor_msgs/Image, BGR8)
  - /camera/camera_info    (sensor_msgs/CameraInfo)
"""

import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class VideoNode(Node):
    """ROS 2 node that reads video frames and publishes them as camera images."""

    def __init__(self):
        super().__init__('video_camera_sim')

        # ── Declare parameters ────────────────────────────────────────────
        self.declare_parameter('video_path', '/videos/input.mp4')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera_link_optical')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('start_delay', 0.0)

        # Camera intrinsics (defaults from URDF: 640×480, hfov=1.3962634 rad)
        # fx = fy = width / (2 * tan(hfov / 2)) ≈ 381.36
        self.declare_parameter('fx', 381.36)
        self.declare_parameter('fy', 381.36)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        # ── Read parameters ───────────────────────────────────────────────
        self.video_path = self.get_parameter('video_path').value
        
        # Robustly parse parameters to handle string/int/float types
        try:
            self.fps = float(self.get_parameter('fps').value)
        except (ValueError, TypeError):
            self.fps = 10.0

        try:
            self.loop = bool(self.get_parameter('loop').value)
        except (ValueError, TypeError):
            self.loop = True

        try:
            self.width = int(self.get_parameter('width').value)
        except (ValueError, TypeError):
            self.width = 640

        try:
            self.height = int(self.get_parameter('height').value)
        except (ValueError, TypeError):
            self.height = 480

        try:
            self.start_delay = max(
                0.0, float(self.get_parameter('start_delay').value))
        except (ValueError, TypeError):
            self.start_delay = 0.0
        self._playback_start_time = time.monotonic() + self.start_delay

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        try:
            self.fx = float(self.get_parameter('fx').value)
        except (ValueError, TypeError):
            self.fx = 381.36

        try:
            self.fy = float(self.get_parameter('fy').value)
        except (ValueError, TypeError):
            self.fy = 381.36

        try:
            self.cx = float(self.get_parameter('cx').value)
        except (ValueError, TypeError):
            self.cx = 320.0

        try:
            self.cy = float(self.get_parameter('cy').value)
        except (ValueError, TypeError):
            self.cy = 240.0

        # ── Open video file ───────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'Cannot open video file: {self.video_path}')
            raise FileNotFoundError(f'Cannot open video file: {self.video_path}')

        self.video_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.video_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        target_res_str = f"{self.width}×{self.height}" if (self.width > 0 and self.height > 0) else "Original"
        self.get_logger().info(
            f'Opened video: {self.video_path}\n'
            f'  Original Res: {self.video_width}×{self.video_height}\n'
            f'  Publish Res:  {target_res_str}\n'
            f'  Native FPS:   {self.video_fps:.1f}\n'
            f'  Total frames: {self.total_frames}\n'
            f'  Publish FPS:  {self.fps:.1f}\n'
            f'  Start delay:  {self.start_delay:.1f}s\n'
            f'  Loop:         {self.loop}'
        )

        # ── CV bridge ─────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Build the static CameraInfo message ──────────────────────────
        self._camera_info_msg = self._build_camera_info()

        # ── QoS: sensor data profile (best-effort, keep last 1) ──────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.image_pub = self.create_publisher(Image, image_topic, sensor_qos)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, camera_info_topic, sensor_qos
        )

        self.get_logger().info(
            f'Publishing on {image_topic}  +  {camera_info_topic}'
        )

        # ── Stats ─────────────────────────────────────────────────────────
        self.frames_published = 0

        # ── Timer: publish at target FPS ─────────────────────────────────
        timer_period = 1.0 / self.fps if self.fps > 0 else 1.0
        self.timer = self.create_timer(timer_period, self._timer_callback)

        # ── Parameter callback for dynamic adjustments ───────────────────
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _on_parameters_changed(self, params):
        """Callback to handle dynamic parameters updates at runtime."""
        from rcl_interfaces.msg import SetParametersResult

        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == 'fps':
                try:
                    new_fps = float(param.value)
                    if new_fps <= 0:
                        raise ValueError("FPS must be greater than 0")
                    self.fps = new_fps
                    
                    # Update the timer
                    self.timer.cancel()
                    timer_period = 1.0 / self.fps
                    self.timer = self.create_timer(timer_period, self._timer_callback)
                    self.get_logger().info(f'Updated publish FPS dynamically to: {self.fps:.1f} Hz')
                except Exception as e:
                    result.successful = False
                    result.reason = f"Invalid FPS: {e}"
            elif param.name == 'width':
                try:
                    new_width = int(param.value)
                    self.width = new_width
                    # Rebuild static camera info
                    self._camera_info_msg = self._build_camera_info()
                    self.get_logger().info(f'Updated publish width dynamically to: {self.width}')
                except Exception as e:
                    result.successful = False
                    result.reason = f"Invalid width: {e}"
            elif param.name == 'height':
                try:
                    new_height = int(param.value)
                    self.height = new_height
                    # Rebuild static camera info
                    self._camera_info_msg = self._build_camera_info()
                    self.get_logger().info(f'Updated publish height dynamically to: {self.height}')
                except Exception as e:
                    result.successful = False
                    result.reason = f"Invalid height: {e}"
            elif param.name == 'loop':
                self.loop = bool(param.value)
                self.get_logger().info(f'Updated loop parameter dynamically to: {self.loop}')

        return result

    # ──────────────────────────────────────────────────────────────────────
    # Camera info construction
    # ──────────────────────────────────────────────────────────────────────
    def _build_camera_info(self) -> CameraInfo:
        """Build a CameraInfo message from the configured intrinsics."""
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = self.width if self.width > 0 else self.video_width
        msg.height = self.height if self.height > 0 else self.video_height
        msg.distortion_model = 'plumb_bob'

        # No distortion (pre-recorded video assumed undistorted)
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Intrinsic camera matrix (3×3, row-major)
        msg.k = [
            self.fx, 0.0,     self.cx,
            0.0,     self.fy, self.cy,
            0.0,     0.0,     1.0,
        ]

        # Rectification matrix (identity for monocular)
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # Projection matrix (3×4)
        msg.p = [
            self.fx, 0.0,     self.cx, 0.0,
            0.0,     self.fy, self.cy, 0.0,
            0.0,     0.0,     1.0,     0.0,
        ]

        return msg

    # ──────────────────────────────────────────────────────────────────────
    # Timer callback — read and publish one frame
    # ──────────────────────────────────────────────────────────────────────
    def _timer_callback(self):
        if time.monotonic() < self._playback_start_time:
            return

        ret, frame = self.cap.read()

        if not ret:
            if self.loop:
                self.get_logger().info(
                    f'End of video ({self.frames_published} frames published). '
                    f'Looping...'
                )
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error('Failed to loop video — stopping.')
                    self.timer.cancel()
                    return
            else:
                self.get_logger().info(
                    f'End of video ({self.frames_published} frames published). '
                    f'Stopping.'
                )
                self.timer.cancel()
                return

        # Resize image if configured
        if self.width > 0 and self.height > 0:
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

        # ── Publish Image ────────────────────────────────────────────
        now = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        # ── Publish CameraInfo (timestamp-synced) ────────────────────
        info_msg = self._camera_info_msg
        info_msg.header.stamp = now
        self.camera_info_pub.publish(info_msg)

        self.frames_published += 1

        # Log throughput every 100 frames
        if self.frames_published % 100 == 0:
            self.get_logger().info(
                f'Video playback  |  frames={self.frames_published}  '
                f'@ {self.fps:.1f} Hz'
            )

    # ──────────────────────────────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Video capture released.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
