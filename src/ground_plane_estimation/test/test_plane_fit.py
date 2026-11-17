"""Host-runnable unit tests for the RANSAC ground plane fit (no ROS needed)."""

import numpy as np

from ground_plane_estimation.plane_fit import fit_ground_plane

UP = np.array([0.0, 0.0, 1.0])


def _ground(rng, n=3000, height=0.2, noise=0.005):
    """Noisy horizontal plane z = -height, 4x4 m footprint."""
    xy = rng.uniform(-2.0, 2.0, size=(n, 2))
    z = -height + rng.normal(0.0, noise, size=n)
    return np.column_stack((xy, z))


def test_recovers_horizontal_plane_with_outliers():
    rng = np.random.default_rng(1)
    points = np.vstack((
        _ground(rng, height=0.2),
        rng.uniform([-2, -2, 0.0], [2, 2, 1.5], size=(500, 3)),  # clutter above
    ))
    fit = fit_ground_plane(points, UP, rng=rng)
    assert fit is not None
    assert fit.normal @ UP > 0.99
    # Signed distance of the origin = robot height above the plane.
    assert abs(fit.signed_distance(np.zeros(3)) - 0.2) < 0.01
    assert fit.inliers > 2000


def test_distance_scales_with_point_scale():
    """The core hypothesis observable: scaling the cloud scales the distance."""
    rng = np.random.default_rng(2)
    points = _ground(rng, height=0.25, noise=0.002)
    for scale in (0.5, 1.0, 3.0):
        fit = fit_ground_plane(points * scale, UP, rng=rng)
        assert fit is not None
        assert abs(fit.signed_distance(np.zeros(3)) - 0.25 * scale) < 0.02 * scale


def test_rejects_steep_plane():
    rng = np.random.default_rng(3)
    points = _ground(rng)
    tilt = np.deg2rad(50.0)
    rot = np.array([
        [np.cos(tilt), 0.0, np.sin(tilt)],
        [0.0, 1.0, 0.0],
        [-np.sin(tilt), 0.0, np.cos(tilt)],
    ])
    fit = fit_ground_plane(points @ rot.T, UP,
                           max_angle_rad=np.deg2rad(35.0), rng=rng)
    assert fit is None


def test_rejects_insufficient_points():
    rng = np.random.default_rng(4)
    assert fit_ground_plane(_ground(rng, n=10), UP, min_inliers=50, rng=rng) is None
    assert fit_ground_plane(np.empty((0, 3)), UP, rng=rng) is None


# ── depth-space affine fit ────────────────────────────────────────────────

def _affine_scene(rng, s, b, n=3000, height=0.3, pitch=0.35):
    """Ground pixels seen by a camera pitched down; returns (points, up_cam, h)."""
    # Optical frame: x right, y down, z forward. Up = -y tilted by pitch.
    up_cam = np.array([0.0, -np.cos(pitch), np.sin(pitch)])
    u = rng.uniform(-0.5, 0.5, n)
    v = rng.uniform(-0.3, 0.9, n)
    rays = np.column_stack((u, v, np.ones(n)))
    d = rays @ up_cam
    keep = d < -0.05
    rays, d = rays[keep], d[keep]
    z_true = -height / d
    z_pred = s * z_true + b + rng.normal(0, 0.01, len(z_true))
    return rays * z_pred[:, None], up_cam, height


def test_ground_ray_depths_recovers_true_depth():
    from ground_plane_estimation.plane_fit import ground_ray_depths
    rng = np.random.default_rng(3)
    pts, up, h = _affine_scene(rng, 1.0, 0.0)
    z_pred, z_true = ground_ray_depths(pts, up, h)
    assert np.abs(z_pred - z_true).max() < 0.06


def test_joint_affine_fit_recovers_scale_and_shift():
    from ground_plane_estimation.plane_fit import (
        fit_depth_affine, ground_ray_depths)
    rng = np.random.default_rng(4)
    pts, up, h = _affine_scene(rng, 1.6, 0.4)
    z_pred, z_true = ground_ray_depths(pts, up, h)
    # Obstacles whose rays also point down must be rejected as outliers.
    z_pred = np.concatenate((z_pred, rng.uniform(0.5, 5.0, 300)))
    z_true = np.concatenate((z_true, rng.uniform(0.5, 5.0, 300)))
    fit = fit_depth_affine(z_pred, z_true, scale_init=1.5)
    assert fit is not None
    assert abs(fit.scale - 1.6) < 0.05 and abs(fit.shift - 0.4) < 0.05


def test_known_scale_solves_shift_only():
    from ground_plane_estimation.plane_fit import (
        fit_depth_affine, ground_ray_depths)
    rng = np.random.default_rng(5)
    pts, up, h = _affine_scene(rng, 1.3, -0.25)
    z_pred, z_true = ground_ray_depths(pts, up, h)
    fit = fit_depth_affine(z_pred, z_true, known_scale=1.3)
    assert fit is not None and abs(fit.shift + 0.25) < 0.02
    # Constant true depth has no spread: the joint fit must refuse.
    assert fit_depth_affine(np.full(500, 2.0), np.full(500, 1.5),
                            scale_init=1.3) is None
