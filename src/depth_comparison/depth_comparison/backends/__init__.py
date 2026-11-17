"""Depth backends available to the comparison node.

To add a model, implement :class:`~depth_comparison.backends.base.DepthBackend`
and register the class here. Nothing else in the module needs to change — the
node declares the backend's parameters from ``param_defaults()`` and the launch
file fans out a point cloud for every configured backend key.
"""

from .base import DepthBackend, Intrinsics
from .depth_anything_v2 import DepthAnythingV2Backend
from .metric3d_v2 import Metric3DV2Backend
from .relay import RelayBackend

#: Maps the ``<key>.type`` parameter onto a backend class.
BACKEND_TYPES: dict[str, type[DepthBackend]] = {
    'depth_anything_v2': DepthAnythingV2Backend,
    'metric3d_v2': Metric3DV2Backend,
    'relay': RelayBackend,
}


def build_backend(backend_type: str, key: str, cfg: dict, logger) -> DepthBackend:
    """Instantiate the backend class registered under ``backend_type``."""
    if backend_type not in BACKEND_TYPES:
        raise ValueError(
            f'[{key}] unknown backend type "{backend_type}", '
            f'expected one of {sorted(BACKEND_TYPES)}'
        )
    return BACKEND_TYPES[backend_type](key, cfg, logger)


__all__ = [
    'BACKEND_TYPES',
    'DepthBackend',
    'DepthAnythingV2Backend',
    'Intrinsics',
    'Metric3DV2Backend',
    'RelayBackend',
    'build_backend',
]
