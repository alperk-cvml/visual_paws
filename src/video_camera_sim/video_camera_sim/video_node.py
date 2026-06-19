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

        # Camera intrinsics (defaults from URDF: 640×480, hfov=1.3962634 rad)
        # fx = fy = width / (2 * tan(hfov / 2)) ≈ 381.36
        self.declare_parameter('fx', 381.36)
        self.declare_parameter('fy', 381.36)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        # ── Read parameters ───────────────────────────────────────────────
        self.video_path = self.get_parameter('video_path').value
        self.fps = self.get_parameter('fps').value
        self.loop = self.get_parameter('loop').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        # ── Open video file ───────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'Cannot open video file: {self.video_path}')
            raise FileNotFoundError(f'Cannot open video file: {self.video_path}')

        self.video_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.video_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.get_logger().info(
            f'Opened video: {self.video_path}\n'
            f'  Resolution:  {self.video_width}×{self.video_height}\n'
            f'  Native FPS:  {self.video_fps:.1f}\n'
            f'  Total frames: {self.total_frames}\n'
            f'  Publish FPS: {self.fps:.1f}\n'
            f'  Loop:        {self.loop}'
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

    # ──────────────────────────────────────────────────────────────────────
    # Camera info construction
    # ──────────────────────────────────────────────────────────────────────
    def _build_camera_info(self) -> CameraInfo:
        """Build a CameraInfo message from the configured intrinsics."""
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = self.video_width
        msg.height = self.video_height
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
