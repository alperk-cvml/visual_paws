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
