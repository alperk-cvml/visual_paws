#!/usr/bin/env python3
"""
simple_elevation_mapping_node.py
---------------------------------
Subscribes to a PointCloud2 topic (output of depth_image_proc), maintains a
2.5-D elevation grid, and publishes it as a grid_map_msgs/GridMap.

The grid is kept in the robot frame (robot_base_frame_id). Each new point
cloud is inserted by looking up the transform from the sensor frame to the
map frame, projecting every point onto the XY plane, and recording the
maximum Z value per cell.

Topics
------
  Input:
    ~/input_pointcloud   (sensor_msgs/PointCloud2)
  Output:
    ~/elevation_map      (grid_map_msgs/GridMap)
"""

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from grid_map_msgs.msg import GridMap, GridMapInfo
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Header
from geometry_msgs.msg import Pose, Point, Quaternion

import tf2_ros
from tf2_ros import TransformException
from tf2_msgs.msg import TFMessage


def pointcloud2_to_xyz_array(cloud_msg: PointCloud2) -> np.ndarray:
    """
    Fast extraction of (x, y, z) from a PointCloud2 message.
    Returns shape (N, 3) float32 array.  Invalid (NaN/Inf) points are dropped.
    """
    # Locate x, y, z field offsets
    offset = {f.name: f.offset for f in cloud_msg.fields}
    point_step = cloud_msg.point_step
    data = cloud_msg.data

    n_points = cloud_msg.width * cloud_msg.height
    xyz = np.frombuffer(bytes(data), dtype=np.uint8)
    # interpret as bytes, extract x y z as float32
    xyz_structured = np.lib.stride_tricks.as_strided(
        xyz,
        shape=(n_points, point_step),
        strides=(point_step, 1)
    )
    x = xyz_structured[:, offset['x']:offset['x'] + 4].view(np.float32).flatten()
    y = xyz_structured[:, offset['y']:offset['y'] + 4].view(np.float32).flatten()
    z = xyz_structured[:, offset['z']:offset['z'] + 4].view(np.float32).flatten()

    pts = np.stack([x, y, z], axis=1)
    valid = np.isfinite(pts).all(axis=1)
    return pts[valid]


class ElevationMappingNode(Node):

    def __init__(self):
        super().__init__('elevation_mapping')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('input_pointcloud_topic', '/depth_camera/depth_image/points')
        self.declare_parameter('map_frame_id', 'odom')
        self.declare_parameter('sensor_frame_id', 'depth_camera_link')
        self.declare_parameter('length_x', 3.0)       # metres
        self.declare_parameter('length_y', 3.0)       # metres
        self.declare_parameter('resolution', 0.05)    # metres/cell
        self.declare_parameter('publish_rate', 2.0)   # Hz

        topic       = self.get_parameter('input_pointcloud_topic').value
        self.map_frame  = self.get_parameter('map_frame_id').value
        self.sens_frame = self.get_parameter('sensor_frame_id').value
        self.len_x  = self.get_parameter('length_x').value
        self.len_y  = self.get_parameter('length_y').value
        self.res    = self.get_parameter('resolution').value
        pub_rate    = self.get_parameter('publish_rate').value

        # Grid dimensions
        self.n_x = int(round(self.len_x / self.res))
        self.n_y = int(round(self.len_y / self.res))

        self.get_logger().info(
            f'Elevation map: {self.n_x}×{self.n_y} cells @ {self.res*100:.1f} cm/cell'
        )

        # Elevation grid (NaN = unknown)
        self._grid = np.full((self.n_x, self.n_y), np.nan, dtype=np.float32)

        # ── TF ────────────────────────────────────────────────────────────────
        # Manual subscriptions instead of TransformListener so we can use
        # volatile QoS to match the Gazebo bridge publisher.
        self._tf_buffer = tf2_ros.Buffer()
        tf_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        tf_static_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(TFMessage, '/tf', self._tf_callback, tf_qos)
        self.create_subscription(TFMessage, '/tf_static', self._tf_static_callback, tf_static_qos)

        # ── Subscribers ───────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._pc_sub = self.create_subscription(
            PointCloud2, topic, self._pc_callback, qos
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._map_pub = self.create_publisher(GridMap, '~/elevation_map', 1)
        self._pub_timer = self.create_timer(1.0 / pub_rate, self._publish_map)

        self.get_logger().info(
            f'Listening on [{topic}], publishing GridMap on [~/elevation_map]'
        )

    # ─────────────────────────────────────────────────────────────────────────
    def _tf_callback(self, msg: TFMessage):
        """Feed dynamic transforms into the buffer."""
        for t in msg.transforms:
            self._tf_buffer.set_transform(t, 'default_authority')

    def _tf_static_callback(self, msg: TFMessage):
        """Feed static transforms into the buffer."""
        for t in msg.transforms:
            self._tf_buffer.set_transform_static(t, 'default_authority')

    # ─────────────────────────────────────────────────────────────────────────
    def _pc_callback(self, msg: PointCloud2):
        """Insert incoming point cloud into the elevation grid."""
        # Get transform: sensor_frame → map_frame
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'TF lookup failed ({msg.header.frame_id} → {self.map_frame}): {ex}',
                throttle_duration_sec=5.0
            )
            return

        # Extract raw XYZ
        pts = pointcloud2_to_xyz_array(msg)   # (N, 3)
        if pts.size == 0:
            return

        # Apply transform
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        R = self._quat_to_rot(q.x, q.y, q.z, q.w)
        pts_world = (R @ pts.T).T + np.array([t.x, t.y, t.z])

        # Map-centre (track robot position in future; fixed at origin for now)
        cx, cy = 0.0, 0.0

        # Convert to grid indices (grid_map convention: index (0,0) is TOP-LEFT spatially, i.e. max X, max Y)
        ix = np.floor((cx + self.len_x / 2.0 - pts_world[:, 0]) / self.res).astype(int)
        iy = np.floor((cy + self.len_y / 2.0 - pts_world[:, 1]) / self.res).astype(int)
        z  = pts_world[:, 2].astype(np.float32)

        # Clamp to grid bounds
        mask = (ix >= 0) & (ix < self.n_x) & (iy >= 0) & (iy < self.n_y)
        ix, iy, z = ix[mask], iy[mask], z[mask]

        # Max-height fusion (simple but robust)
        for i in range(len(ix)):
            val = z[i]
            cur = self._grid[ix[i], iy[i]]
            if np.isnan(cur) or val > cur:
                self._grid[ix[i], iy[i]] = val

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_map(self):
        """Publish the current elevation grid as a GridMap message."""
        now = self.get_clock().now().to_msg()

        msg = GridMap()
        msg.header = Header(stamp=now, frame_id=self.map_frame)

        msg.info = GridMapInfo(
            header=Header(stamp=now, frame_id=self.map_frame),
            resolution=self.res,
            length_x=self.len_x,
            length_y=self.len_y,
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )

        msg.layers = ['elevation']
        msg.basic_layers = ['elevation']

        # Flatten row-major (grid_map convention: row = y, col = x)
        layer_data = Float32MultiArray()
        layout = MultiArrayLayout()
        layout.dim = [
            MultiArrayDimension(label='column_index', size=self.n_x, stride=self.n_x * self.n_y),
            MultiArrayDimension(label='row_index',    size=self.n_y, stride=self.n_y),
        ]
        layout.data_offset = 0
        layer_data.layout = layout
        # grid_map stores data column-major (Fortran order)
        flat = self._grid.flatten(order='F').tolist()
        layer_data.data = flat

        msg.data = [layer_data]

        self._map_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _quat_to_rot(x, y, z, w) -> np.ndarray:
        """Convert quaternion to 3×3 rotation matrix."""
        n = math.sqrt(x*x + y*y + z*z + w*w)
        if n < 1e-9:
            return np.eye(3)
        x, y, z, w = x/n, y/n, z/n, w/n
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
        ], dtype=np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = ElevationMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
