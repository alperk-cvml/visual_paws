"""Host-runnable tests for the ROS-free trajectory comparison (pure numpy)."""

import numpy as np
import pytest

from odometry_scale_estimation.trajectory_compare import (
    AffineScaleModel, ScalarScaleModel, TrajectoryBuffer, align_estimate,
    chord_pairs, make_model, motion_in_body, quat_to_rot,
)


def _walk(times, speed=0.4, seed=0, wobble=0.0):
    """Curved planar walk sampled at `times`; wobble adds position noise."""
    rng = np.random.default_rng(seed)
    t = np.asarray(times, dtype=float)
    pts = np.column_stack((
        speed * t,
        0.3 * np.sin(0.5 * t),
        0.02 * np.sin(2.0 * t),
    ))
    if wobble > 0.0:
        pts = pts + rng.normal(0.0, wobble, size=pts.shape)
    return pts


def _pairs_for(scale, shift=0.0, noise=0.0, seed=0, duration=5.0, hz=10.0):
    """Chord pairs from a synthetic walk whose estimate chords are affine."""
    times = np.arange(0.0, duration, 1.0 / hz)
    ref = _walk(times)
    d_ref, _ = chord_pairs(times, ref, ref, min_dt=0.3)
    rng = np.random.default_rng(seed)
    d_est = scale * d_ref + shift + rng.normal(0.0, noise, size=d_ref.shape)
    return d_ref, d_est


# ── TrajectoryBuffer ─────────────────────────────────────────────────────

def test_buffer_prunes_outside_window():
    buf = TrajectoryBuffer(window=2.0)
    for t in np.arange(0.0, 5.0, 0.5):
        buf.append(t, [t, 0.0, 0.0])
    times, _ = buf.snapshot(4.5)
    assert times[0] >= 2.5 and times[-1] == 4.5


def test_buffer_clears_on_backwards_time():
    buf = TrajectoryBuffer(window=10.0)
    buf.append(1.0, [0.0, 0.0, 0.0])
    buf.append(2.0, [0.1, 0.0, 0.0])
    buf.append(0.5, [0.2, 0.0, 0.0])  # bag looped / node restarted
    assert len(buf) == 1 and buf.latest_time == 0.5


def test_buffer_clears_on_position_jump():
    buf = TrajectoryBuffer(window=10.0, max_jump=1.0)
    buf.append(1.0, [5.0, 0.0, 0.0])
    buf.append(2.0, [0.0, 0.0, 0.0])  # odometry re-initialised at origin
    assert len(buf) == 1


def test_buffer_rejects_non_finite():
    buf = TrajectoryBuffer(window=10.0)
    buf.append(1.0, [np.nan, 0.0, 0.0])
    assert len(buf) == 0


def test_interpolation_between_samples():
    buf = TrajectoryBuffer(window=10.0, max_gap=2.0, max_jump=5.0)
    buf.append(0.0, [0.0, 0.0, 0.0])
    buf.append(1.0, [2.0, 0.0, 0.0])
    pts, valid = buf.interpolate([0.5])
    assert valid[0]
    np.testing.assert_allclose(pts[0], [1.0, 0.0, 0.0])


def test_interpolation_invalid_across_gap_and_outside_range():
    buf = TrajectoryBuffer(window=100.0, max_gap=0.5)
    for t in (0.0, 0.2, 0.4, 3.0, 3.2):  # dropout between 0.4 and 3.0
        buf.append(t, [0.1 * t, 0.0, 0.0])
    pts, valid = buf.interpolate([0.3, 1.5, 3.1, 5.0])
    assert list(valid) == [True, False, True, False]
    assert np.all(np.isnan(pts[~valid]))


# ── chord_pairs ──────────────────────────────────────────────────────────

def test_chord_pairs_respects_min_dt():
    times = np.arange(0.0, 2.0, 0.5)
    pts = _walk(times)
    d_ref, d_est = chord_pairs(times, pts, pts, min_dt=1.0)
    # Pairs: (0,1.0) (0,1.5) (0.5,1.5) — with 4 samples and min_dt=1.0.
    assert len(d_ref) == 3
    np.testing.assert_allclose(d_ref, d_est)


def test_chord_pairs_subsamples_to_cap():
    times = np.arange(0.0, 10.0, 0.01)  # 1000 samples
    pts = _walk(times)
    d_ref, _ = chord_pairs(times, pts, pts, min_dt=0.1, max_samples=20)
    assert len(d_ref) <= 20 * 19 / 2


def test_chord_pairs_too_few_samples():
    d_ref, d_est = chord_pairs(np.array([0.0]), np.zeros((1, 3)),
                               np.zeros((1, 3)))
    assert len(d_ref) == 0 and len(d_est) == 0


# ── ScalarScaleModel ─────────────────────────────────────────────────────

def test_scalar_recovers_scale_from_scaled_trajectory():
    # est trajectory = s × ref trajectory (what KISS-ICP on a uniformly
    # scaled cloud produces) → chords scale by s exactly.
    times = np.arange(0.0, 5.0, 0.1)
    ref = _walk(times)
    est = 2.4 * ref
    d_ref, d_est = chord_pairs(times, ref, est, min_dt=0.5)
    fit = ScalarScaleModel().fit(d_ref, d_est)
    assert fit is not None
    assert abs(fit.scale - 2.4) < 1e-9
    assert fit.shift is None
    assert fit.rms < 1e-9


def test_scalar_tolerates_noise():
    d_ref, d_est = _pairs_for(scale=1.6, noise=0.01)
    fit = ScalarScaleModel().fit(d_ref, d_est)
    assert abs(fit.scale - 1.6) < 0.05
    assert fit.rms < 0.05


def test_scalar_rejects_standing_still():
    times = np.arange(0.0, 5.0, 0.1)
    ref = _walk(times, speed=0.0) * 0.0 + 0.001  # ~1 mm of "motion"
    d_ref, d_est = chord_pairs(times, ref, ref, min_dt=0.5)
    assert ScalarScaleModel(min_motion=0.1).fit(d_ref, d_est) is None


def test_scalar_rejects_empty_and_reports_excitation():
    assert ScalarScaleModel().fit(np.empty(0), np.empty(0)) is None
    d_ref, d_est = _pairs_for(scale=1.0)
    fit = ScalarScaleModel().fit(d_ref, d_est)
    assert abs(fit.ref_motion - d_ref.max()) < 1e-12


def test_pair_gate_drops_icp_jump():
    d_ref, d_est = _pairs_for(scale=1.5)
    d_est = d_est.copy()
    d_est[3] += 500.0  # one wild ICP jump
    fit = ScalarScaleModel().fit(d_ref, d_est)
    assert abs(fit.scale - 1.5) < 0.01
    assert fit.n_pairs == len(d_ref) - 1


def test_scalar_rejects_all_outlier_window():
    d_ref, d_est = _pairs_for(scale=1.0)
    # All-outlier estimate chords: everything gated away → too few pairs.
    assert ScalarScaleModel().fit(d_ref, d_est * 1e4) is None


# ── AffineScaleModel ─────────────────────────────────────────────────────

def test_affine_recovers_scale_and_shift():
    d_ref, d_est = _pairs_for(scale=1.8, shift=0.15)
    fit = AffineScaleModel().fit(d_ref, d_est)
    assert fit is not None
    assert abs(fit.scale - 1.8) < 1e-6
    assert abs(fit.shift - 0.15) < 1e-6


def test_affine_with_zero_shift_matches_scalar():
    d_ref, d_est = _pairs_for(scale=1.3, noise=0.005)
    scalar = ScalarScaleModel().fit(d_ref, d_est)
    affine = AffineScaleModel().fit(d_ref, d_est)
    assert abs(affine.scale - scalar.scale) < 0.05
    assert abs(affine.shift) < 0.02


def test_affine_rejects_degenerate_spread():
    # Equal chords everywhere: s and b are indistinguishable.
    d_ref = np.full(20, 0.5)
    d_est = 2.0 * d_ref
    assert AffineScaleModel(min_spread=0.05).fit(d_ref, d_est) is None


# ── factory ──────────────────────────────────────────────────────────────

def test_factory_builds_registered_models():
    assert isinstance(make_model('scalar'), ScalarScaleModel)
    assert isinstance(make_model('affine'), AffineScaleModel)


def test_factory_rejects_unknown_model():
    with pytest.raises(ValueError, match='unknown scale model'):
        make_model('umeyama')


# ── align_estimate (marker overlay) ──────────────────────────────────────

def test_align_recovers_rigid_transform():
    times = np.arange(0.0, 5.0, 0.2)
    ref = _walk(times)
    angle = 0.7
    rot = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                    [np.sin(angle), np.cos(angle), 0.0],
                    [0.0, 0.0, 1.0]])
    est = ref @ rot.T + np.array([3.0, -1.0, 0.5])
    np.testing.assert_allclose(align_estimate(ref, est), ref, atol=1e-9)


def test_align_preserves_scale_mismatch():
    # Alignment must NOT absorb the scale — that's the observable.
    times = np.arange(0.0, 5.0, 0.2)
    ref = _walk(times)
    est = 2.0 * ref
    aligned = align_estimate(ref, est)
    span = np.linalg.norm(aligned[-1] - aligned[0])
    assert abs(span - 2.0 * np.linalg.norm(ref[-1] - ref[0])) < 1e-9


def test_align_two_points_falls_back_to_translation():
    ref = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    est = ref + 5.0
    aligned = align_estimate(ref, est)
    np.testing.assert_allclose(aligned.mean(axis=0), ref.mean(axis=0))


# ── motion_in_body (arrow markers) ───────────────────────────────────────

def test_quat_to_rot_identity_and_yaw():
    np.testing.assert_allclose(quat_to_rot([1.0, 0.0, 0.0, 0.0]), np.eye(3))
    # 90° yaw: body +x points along odom +y.
    half = np.sqrt(0.5)
    rot = quat_to_rot([half, 0.0, 0.0, half])
    np.testing.assert_allclose(rot @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                               atol=1e-12)


def test_motion_in_body_undoes_heading():
    # Robot yawed 90° left, moving straight ahead: the odom-frame
    # displacement is +y, but in the body frame the motion is pure +x.
    half = np.sqrt(0.5)
    vec = motion_in_body([0.0, 2.0, 0.0], [half, 0.0, 0.0, half])
    np.testing.assert_allclose(vec, [2.0, 0.0, 0.0], atol=1e-12)


def test_motion_in_body_preserves_length():
    rng = np.random.default_rng(3)
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    disp = rng.normal(size=3)
    assert abs(np.linalg.norm(motion_in_body(disp, q))
               - np.linalg.norm(disp)) < 1e-12
