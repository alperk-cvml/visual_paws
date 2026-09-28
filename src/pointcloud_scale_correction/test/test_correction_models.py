"""Host-runnable tests for the ROS-free correction models (pure numpy)."""

import numpy as np
import pytest

from pointcloud_scale_correction.correction_models import (
    AffineCorrection, ParameterFilter, ScaleObservation, ScalarCorrection,
    make_model,
)


def _cloud(n=200, seed=0):
    """Random camera-optical-frame cloud with strictly positive depth."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-2.0, 2.0, size=(n, 2))
    z = rng.uniform(0.5, 8.0, size=(n, 1))
    return np.column_stack((xy, z)).astype(np.float32)


# ── ParameterFilter ──────────────────────────────────────────────────────

def test_filter_none_takes_last_value():
    f = ParameterFilter(mode='none')
    f.update(1.0)
    assert f.update(3.0) == 3.0


def test_filter_ema_converges():
    f = ParameterFilter(mode='ema', ema_alpha=0.5)
    assert f.update(2.0) == 2.0  # first observation seeds the filter
    for _ in range(20):
        f.update(4.0)
    assert abs(f.value - 4.0) < 1e-3


def test_filter_median_rejects_single_outlier():
    f = ParameterFilter(mode='median', median_window=5)
    for _ in range(4):
        f.update(2.0)
    assert f.update(100.0) == 2.0


def test_filter_unknown_mode_raises():
    with pytest.raises(ValueError):
        ParameterFilter(mode='kalman')


# ── ScalarCorrection ─────────────────────────────────────────────────────

def test_scalar_divides_by_scale():
    model = ScalarCorrection(smoothing='none')
    assert not model.ready()
    assert model.update(ScaleObservation(stamp=0.0, scale=2.0))
    assert model.ready()
    cloud = _cloud()
    np.testing.assert_allclose(model.correct(cloud), cloud / 2.0, rtol=1e-6)


def test_scalar_recovers_true_cloud():
    # depth_pred = s * depth_true; correcting with s must undo it exactly.
    s = 1.8
    true = _cloud()
    model = ScalarCorrection(smoothing='none')
    model.update(ScaleObservation(stamp=0.0, scale=s))
    np.testing.assert_allclose(model.correct(true * s), true, rtol=1e-5)


def test_scalar_gate_rejects_garbage():
    model = ScalarCorrection(smoothing='none')
    assert not model.update(ScaleObservation(stamp=0.0, scale=-1.0))
    assert not model.update(ScaleObservation(stamp=0.0, scale=np.nan))
    assert not model.update(ScaleObservation(stamp=0.0, scale=1000.0))
    assert not model.ready()


def test_scalar_does_not_mutate_input_and_passes_nans():
    model = ScalarCorrection(smoothing='none')
    model.update(ScaleObservation(stamp=0.0, scale=2.0))
    cloud = _cloud()
    cloud[0] = np.nan
    original = cloud.copy()
    out = model.correct(cloud)
    np.testing.assert_array_equal(cloud, original)
    assert np.all(np.isnan(out[0]))
    assert np.all(np.isfinite(out[1:]))


def test_scalar_tracks_observation_stamp():
    model = ScalarCorrection(smoothing='none')
    model.update(ScaleObservation(stamp=42.0, scale=2.0))
    assert model.last_stamp == 42.0


# ── AffineCorrection ─────────────────────────────────────────────────────

def test_affine_with_zero_shift_matches_scalar():
    cloud = _cloud()
    obs = ScaleObservation(stamp=0.0, scale=1.7, shift=0.0)
    scalar, affine = ScalarCorrection(smoothing='none'), \
        AffineCorrection(smoothing='none')
    scalar.update(obs)
    affine.update(obs)
    np.testing.assert_allclose(
        affine.correct(cloud), scalar.correct(cloud), rtol=1e-5)


def test_affine_recovers_true_cloud():
    # depth_pred = s * depth_true + b, applied along each projective ray.
    s, b = 1.5, 0.4
    true = _cloud()
    z_true = true[:, 2]
    z_pred = s * z_true + b
    pred = true * (z_pred / z_true)[:, None]
    model = AffineCorrection(smoothing='none')
    model.update(ScaleObservation(stamp=0.0, scale=s, shift=b))
    np.testing.assert_allclose(model.correct(pred), true, rtol=1e-4)


def test_affine_invalidates_nonpositive_corrected_depth():
    # A point closer than the shift would correct to z <= 0 → NaN, never a
    # reflection through the camera.
    model = AffineCorrection(smoothing='none')
    model.update(ScaleObservation(stamp=0.0, scale=1.0, shift=0.5))
    cloud = np.array([[0.0, 0.0, 0.3], [0.0, 0.0, 2.0]], dtype=np.float32)
    out = model.correct(cloud)
    assert np.all(np.isnan(out[0]))
    np.testing.assert_allclose(out[1], [0.0, 0.0, 1.5], atol=1e-6)


def test_affine_state_reports_both_parameters():
    model = AffineCorrection(smoothing='none')
    model.update(ScaleObservation(stamp=0.0, scale=2.0, shift=0.25))
    assert model.state() == {'scale': 2.0, 'shift': 0.25}


# ── factory ──────────────────────────────────────────────────────────────

def test_factory_builds_registered_models():
    assert isinstance(make_model('scalar'), ScalarCorrection)
    assert isinstance(make_model('affine'), AffineCorrection)


def test_factory_rejects_unknown_model():
    with pytest.raises(ValueError, match='unknown correction model'):
        make_model('polynomial')
