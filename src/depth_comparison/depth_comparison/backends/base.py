"""Backend interface for the depth model comparison node.

A backend is one depth source in the comparison. Backends fall into two kinds:

``consumes_rgb = True``
    Inference backends. The node hands each of them the *same* BGR frame and
    they return a metric depth image. This is what makes the comparison fair —
    every model sees identical pixels with an identical timestamp.

``consumes_rgb = False``
    Relay backends. These republish an existing ROS depth topic (a Gazebo depth
    sensor, a recorded RGBD stream) into the comparison namespace unchanged.
    The node wires them up directly instead of calling :meth:`infer`.
"""

from abc import ABC, abstractmethod
from typing import Any, NamedTuple

import numpy as np


class Intrinsics(NamedTuple):
    """Pinhole intrinsics for the frame being processed, in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, info) -> 'Intrinsics':
        k = info.k
        return cls(
            fx=float(k[0]), fy=float(k[4]),
            cx=float(k[2]), cy=float(k[5]),
            width=int(info.width), height=int(info.height),
        )


class DepthBackend(ABC):
    """One depth source in the comparison.

    Subclasses declare their configuration through :meth:`param_defaults`. The
    node declares each entry as a ROS parameter named ``<key>.<name>``, resolves
    it, and passes the results back as ``cfg``. That keeps every backend knob
    overridable from the launch file and docker-compose without the node
    knowing what any particular backend needs.
    """

    #: Whether the node should feed RGB frames to this backend (see module docstring).
    consumes_rgb: bool = True

    def __init__(self, key: str, cfg: dict[str, Any], logger):
        self.key = key
        self.cfg = cfg
        self.logger = logger

    @staticmethod
    @abstractmethod
    def param_defaults() -> dict[str, Any]:
        """Parameter names and defaults this backend accepts under ``<key>.``."""

    def load(self, device) -> None:
        """Load weights onto ``device``. Called once, before the first :meth:`infer`."""

    @abstractmethod
    def infer(self, bgr: np.ndarray, intrinsics: Intrinsics | None) -> np.ndarray:
        """Return metric depth in metres, float32, shaped ``(H, W)`` like ``bgr``.

        ``intrinsics`` is ``None`` until the matching CameraInfo has arrived.
        Backends that need intrinsics to recover metric scale should say so by
        raising, rather than silently returning an arbitrarily scaled result.
        """

    def describe(self) -> str:
        """One-line summary logged at startup."""
        return self.key
