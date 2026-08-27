"""Relay backend — republishes an existing ROS depth topic into the comparison.

Used for the ground-truth channel: the Gazebo depth sensor already publishes
metric depth, so there is nothing to infer. The relay exists so the reference
appears in the comparison namespace alongside the inferred streams and gets the
same point cloud treatment in the launch file.

The depth image and its CameraInfo are republished **verbatim** — same
timestamp, same frame_id. Restamping or reframing a depth image is the
single most common way to corrupt the downstream point cloud.
"""

from typing import Any

import numpy as np

from .base import DepthBackend, Intrinsics


class RelayBackend(DepthBackend):
    """Passes an existing depth topic through, unmodified."""

    consumes_rgb = False

    @staticmethod
    def param_defaults() -> dict[str, Any]:
        return {
            'depth_topic': '/depth_camera/depth_image',
            'camera_info_topic': '/depth_camera/camera_info',
        }

    def infer(self, bgr: np.ndarray, intrinsics: Intrinsics | None) -> np.ndarray:
        raise NotImplementedError(
            'RelayBackend does not run inference; the node subscribes to '
            f'{self.cfg["depth_topic"]} and republishes it directly.'
        )

    def describe(self) -> str:
        return f'{self.key}: relay of {self.cfg["depth_topic"]} (reference channel)'
