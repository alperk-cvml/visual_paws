"""Unit tests for the backend registry and configuration contract.

These run without ROS, CUDA, or model weights — they cover the wiring that
decides which model runs and how its parameters are declared, which is where
a misconfigured comparison silently produces the wrong answer.
"""

import numpy as np
import pytest

from depth_comparison.backends import (
    BACKEND_TYPES,
    DepthAnythingV2Backend,
    Intrinsics,
    Metric3DV2Backend,
    RelayBackend,
    build_backend,
)


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _defaults(cls):
    return dict(cls.param_defaults())


def test_every_registered_type_declares_its_parameters():
    for name, cls in BACKEND_TYPES.items():
        defaults = cls.param_defaults()
        assert isinstance(defaults, dict), f'{name} must return a dict of defaults'
        # ROS 2 infers a parameter's type from its default, so `None` and empty
        # containers would leave the parameter undeclarable.
        for key, value in defaults.items():
            assert value is not None, f'{name}.{key} default must not be None'


def test_build_backend_rejects_unknown_type():
    with pytest.raises(ValueError, match='unknown backend type'):
        build_backend('no_such_model', 'k', {}, _Logger())


def test_build_backend_returns_registered_class():
    backend = build_backend('relay', 'gt', _defaults(RelayBackend), _Logger())
    assert isinstance(backend, RelayBackend)


def test_relay_does_not_consume_rgb():
    """The node must wire relays to a topic, not call infer() on them."""
    assert RelayBackend.consumes_rgb is False
    assert DepthAnythingV2Backend.consumes_rgb is True
    assert Metric3DV2Backend.consumes_rgb is True


def test_relay_infer_is_an_error():
    backend = RelayBackend('gt', _defaults(RelayBackend), _Logger())
    with pytest.raises(NotImplementedError):
        backend.infer(np.zeros((4, 4, 3), np.uint8), None)


def test_describe_works_before_load():
    """The node logs describe() before calling load(), so it must not depend
    on anything load() sets up. Regression test: DA-V2 read self.max_depth,
    which only existed after load(), and every channel was dropped at startup.
    """
    for name, cls in BACKEND_TYPES.items():
        backend = cls(name, _defaults(cls), _Logger())
        text = backend.describe()
        assert name in text, f'{name}.describe() should identify the channel'


def test_dav2_resolves_config_at_construction():
    """Bad config should fail before the expensive weight load, not during it."""
    cfg = _defaults(DepthAnythingV2Backend) | {'encoder': 'vitxl'}
    with pytest.raises(ValueError, match='unknown encoder'):
        DepthAnythingV2Backend('bad', cfg, _Logger())

    cfg = _defaults(DepthAnythingV2Backend) | {'dataset': 'nyu', 'max_depth': 0.0}
    with pytest.raises(ValueError, match='no default max_depth'):
        DepthAnythingV2Backend('bad', cfg, _Logger())


def test_dav2_checkpoint_path_tracks_dataset_and_encoder():
    cfg = _defaults(DepthAnythingV2Backend) | {'dataset': 'hypersim', 'encoder': 'vitb'}
    backend = DepthAnythingV2Backend('h', cfg, _Logger())
    assert backend.model_path.endswith('depth_anything_v2_metric_hypersim_vitb.pth')
    # max_depth left at 0 must resolve to the checkpoint's trained-in scale.
    assert backend.max_depth == 20.0


def test_dav2_defaults_cover_both_comparison_checkpoints():
    """`dataset` is what separates the vkitti and hypersim channels."""
    from depth_comparison.backends.depth_anything_v2 import DATASET_MAX_DEPTH

    assert set(DATASET_MAX_DEPTH) == {'vkitti', 'hypersim'}
    assert DATASET_MAX_DEPTH['vkitti'] > DATASET_MAX_DEPTH['hypersim']
    assert _defaults(DepthAnythingV2Backend)['dataset'] in DATASET_MAX_DEPTH


def test_metric3d_requires_intrinsics():
    """Metric3D recovers scale from focal length; without it there is no metre."""
    backend = Metric3DV2Backend('m3d', _defaults(Metric3DV2Backend), _Logger())
    with pytest.raises(RuntimeError, match='focal length'):
        backend.infer(np.zeros((4, 4, 3), np.uint8), None)


def test_mmcv_shim_exposes_exactly_what_metric3d_imports():
    """The shim must cover Metric3D's four symbols and refuse the rest.

    Skipped where mmengine is absent (it only ships in the container image).
    """
    pytest.importorskip('mmengine')
    import os
    import sys

    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'compat'))
    from mmcv.utils import Config, DictAction, collect_env, get_git_hash  # noqa: F401

    import mmcv.utils as shim
    # Dunder probes must look like ordinary attribute misses, or the import
    # machinery reports a misleading ImportError while resolving `from ... import`.
    with pytest.raises(AttributeError):
        shim.__path__
    with pytest.raises(ImportError, match='not provided by the mmengine'):
        shim.imwrite


def test_intrinsics_from_camera_info():
    class _Info:
        k = [525.0, 0.0, 319.5, 0.0, 524.0, 239.5, 0.0, 0.0, 1.0]
        width = 640
        height = 480

    intr = Intrinsics.from_camera_info(_Info())
    assert intr == Intrinsics(525.0, 524.0, 319.5, 239.5, 640, 480)
