#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
from sensor_msgs.msg import PointCloud2
from grid_map_msgs.msg import GridMap
from tf_transformations import quaternion_matrix

from elevation_mapping_cupy.elevation_mapping_node import ElevationMappingNode, _pointcloud2_xyz_f32


class SimpleElevationMappingNode(ElevationMappingNode):
    """
    Subclass of ElevationMappingNode that adds:
    1. Robust TF lookup error handling and timestamp conversions.
    2. Immediate GridMap publishing on pointcloud arrival.
    3. Neutral GridMap output pose (zero orientation and z=0) to prevent visual tilting in Foxglove/RViz.
    4. Diagnostic logging counters for tracking runtime status.
    """

    def safe_lookup_transform(self, target_frame, source_frame, time):
        try:
            lookup_time = time
            if hasattr(time, 'sec') and hasattr(time, 'nanosec'):
                lookup_time = rclpy.time.Time.from_msg(time)
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time
            )
        except tf2_ros.TransformException:
            # Fall back to latest available transform if exact timestamp lookup fails
            try:
                return self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
            except tf2_ros.TransformException as e:
                self.get_logger().warning(
                    f"Transform from '{source_frame}' to '{target_frame}' not available: {e}",
                    throttle_duration_sec=5.0
                )
                return None
        except Exception as e:
            self.get_logger().warning(
                f"Unexpected error looking up transform from '{source_frame}' to '{target_frame}': {e}",
                throttle_duration_sec=5.0
            )
            return None

    def publish_map(self, key: str) -> None:
        try:
            if not hasattr(self, '_publish_call_count'):
                self._publish_call_count = 0
            self._publish_call_count += 1
            if self._publish_call_count % 10 == 1:
                self.get_logger().info(f"publish_map called #{self._publish_call_count} for key='{key}'")

            if self._map_q is None:
                if not hasattr(self, '_publish_warn_count'):
                    self._publish_warn_count = 0
                self._publish_warn_count += 1
                if self._publish_warn_count % 5 == 1:
                    self.get_logger().warn(
                        f"publish_map: _map_q is None (no successful pose_update yet). "
                        f"_last_t={'set' if self._last_t is not None else 'None'}, "
                        f"pointclouds_processed={self._pointcloud_process_counter}"
                    )
                return
            center = self._get_map_center()
            gm = GridMap()
            gm.header.frame_id = self.map_frame
            gm.header.stamp = self._last_t if self._last_t is not None else self.get_clock().now().to_msg()
            gm.info.resolution = self._map.resolution
            actual_map_length = (self._map.cell_n - 2) * self._map.resolution
            gm.info.length_x = actual_map_length
            gm.info.length_y = actual_map_length
            if self._map_t is not None:
                gm.info.pose.position.x = self._map_t.x
                gm.info.pose.position.y = self._map_t.y
                # Keep pose.x/y as map center in map_frame, but force neutral z and orientation for visualizers
                gm.info.pose.position.z = 0.0
            else:
                gm.info.pose.position.x = float(center[0])
                gm.info.pose.position.y = float(center[1])
                gm.info.pose.position.z = 0.0

            gm.info.pose.orientation.x = 0.0
            gm.info.pose.orientation.y = 0.0
            gm.info.pose.orientation.z = 0.0
            gm.info.pose.orientation.w = 1.0
            gm.layers = []
            gm.basic_layers = self.my_publishers[key]["basic_layers"]

            for layer in self.my_publishers[key].get("layers", []):
                gm.layers.append(layer)
                self._map.get_map_with_name_ref(layer, self._map_data)
                map_data_for_gridmap = self._map_data
                gm.data.append(self._numpy_to_multiarray(map_data_for_gridmap, layout="gridmap_column"))

            gm.outer_start_index = 0
            gm.inner_start_index = 0
            self._publishers_dict[key].publish(gm)
            if not hasattr(self, '_pub_success_count'):
                self._pub_success_count = 0
            self._pub_success_count += 1
            if self._pub_success_count % 10 == 1:
                self.get_logger().info(f"Published GridMap #{self._pub_success_count} on {key}")
        except Exception as e:
            self.get_logger().error(f"Error in publish_map: {e}", throttle_duration_sec=2.0)

    def pointcloud_callback(self, msg: PointCloud2, sub_key: str) -> None:
        self._last_t = msg.header.stamp
        self.pose_update()
        if self._pointcloud_process_counter == 0:
            self.get_logger().info(
                f"First pointcloud received on '{sub_key}': "
                f"{msg.width}x{msg.height} pts, frame='{msg.header.frame_id}'"
            )

        pts = _pointcloud2_xyz_f32(msg)
        if pts.size == 0:
            return

        frame_sensor_id = msg.header.frame_id
        if not frame_sensor_id:
            raise ValueError("PointCloud2 header.frame_id is empty.")

        if frame_sensor_id == self.map_frame:
            t_np = np.zeros(3, dtype=np.float32)
            R = np.eye(3, dtype=np.float32)
        else:
            transform_sensor_to_map = self.safe_lookup_transform(
                self.map_frame,
                frame_sensor_id,
                msg.header.stamp,
            )
            if transform_sensor_to_map is None:
                self.get_logger().warning(
                    f"pointcloud_callback: safe_lookup_transform returned None for {frame_sensor_id} -> {self.map_frame}",
                    throttle_duration_sec=3.0
                )
                return

            t = transform_sensor_to_map.transform.translation
            q = transform_sensor_to_map.transform.rotation
            t_np = np.array([t.x, t.y, t.z], dtype=np.float32)
            R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)

        self._map.input_pointcloud(pts, ["x", "y", "z"], R, t_np, 0, 0)
        self._pointcloud_process_counter += 1
        if self._pointcloud_process_counter % 10 == 1:
            self.get_logger().info(f"Processed pointcloud #{self._pointcloud_process_counter}")

        for pub_key in self.my_publishers:
            self.publish_map(pub_key)

    def pose_update(self) -> None:
        if self._last_t is None:
            return
        transform = self.safe_lookup_transform(
            self.map_frame,
            self.base_frame,
            self._last_t
        )
        if transform is None:
            if not hasattr(self, '_pose_warn_count'):
                self._pose_warn_count = 0
            self._pose_warn_count += 1
            if self._pose_warn_count % 10 == 1:
                self.get_logger().warn(
                    f"pose_update: TF '{self.map_frame}' -> '{self.base_frame}' not available"
                )
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        trans = np.array([t.x, t.y, t.z], dtype=np.float32)
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)
        self._map.move_to(trans, rot)
        self._map_t = t
        self._map_q = q
        if not hasattr(self, '_pose_update_count'):
            self._pose_update_count = 0
        self._pose_update_count += 1
        if self._pose_update_count % 10 == 1:
            self.get_logger().info(f"pose_update #{self._pose_update_count}: map_t=[{t.x:.2f}, {t.y:.2f}, {t.z:.2f}]")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleElevationMappingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
