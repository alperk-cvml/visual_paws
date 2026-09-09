"""RANSAC ground-plane fitting on bare numpy point arrays.

Kept free of ROS imports so the estimator can be unit-tested on the host.

Plane convention: unit normal n oriented along the supplied `up` vector and
scalar offset d such that every plane point p satisfies n·p + d = 0. The
signed distance of a query point q is then n·q + d — positive when q is on
the `up` side of the plane (the robot standing above the ground).
"""

from dataclasses import dataclass

import numpy as np

# Sample triangles with sides shorter than this span nothing; their cross
# product is numerically meaningless.
_MIN_NORMAL_NORM = 1e-8


@dataclass(frozen=True)
class PlaneFit:
    """Result of a successful ground-plane fit."""

    normal: np.ndarray   # (3,) unit vector, oriented along `up`
    offset: float        # n·p + offset = 0 for points p on the plane
    inliers: int         # inlier count of the refined fit
    candidates: int      # points the fit was run on
    rms: float           # RMS inlier residual (m)

    def signed_distance(self, point) -> float:
        """Signed distance of `point` to the plane (positive on the up side)."""
        return float(self.normal @ np.asarray(point, dtype=np.float64) + self.offset)


def _refine(points: np.ndarray, up: np.ndarray):
    """Least-squares plane through `points` (SVD), normal oriented along `up`."""
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1]
    if normal @ up < 0.0:
        normal = -normal
    return normal, float(-(normal @ centroid))


def fit_ground_plane(
    points: np.ndarray,
    up: np.ndarray,
    *,
    iterations: int = 120,
    threshold: float = 0.03,
    max_angle_rad: float = 0.6,
    min_inliers: int = 50,
    rng: np.random.Generator | None = None,
) -> PlaneFit | None:
    """Fit the dominant near-horizontal plane in `points`.

    points          (N,3) candidate points, already expressed in the frame
                    whose +`up` axis is "away from the ground" (base frame).
    up              (3,) unit vector the plane normal must roughly align with.
    iterations      RANSAC hypothesis count.
    threshold       inlier distance threshold (m).
    max_angle_rad   maximum angle between plane normal and `up`; hypotheses
                    steeper than this (walls, obstacles) are rejected.
    min_inliers     reject fits supported by fewer points.

    Returns None when no acceptable plane exists.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < max(3, min_inliers):
        return None
    up = np.asarray(up, dtype=np.float64)
    up = up / np.linalg.norm(up)
    cos_max = np.cos(max_angle_rad)
    rng = rng or np.random.default_rng()

    n_pts = len(points)
    best_count = 0
    best_mask = None
    for _ in range(iterations):
        i, j, k = rng.choice(n_pts, size=3, replace=False)
        normal = np.cross(points[j] - points[i], points[k] - points[i])
        norm = np.linalg.norm(normal)
        if norm < _MIN_NORMAL_NORM:
            continue
        normal /= norm
        if normal @ up < 0.0:
            normal = -normal
        if normal @ up < cos_max:
            continue
        offset = -(normal @ points[i])
        mask = np.abs(points @ normal + offset) < threshold
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask

    if best_mask is None or best_count < min_inliers:
        return None

    # Two refinement rounds: LSQ on the consensus set, then re-select inliers
    # against the refined plane and fit once more.
    mask = best_mask
    for _ in range(2):
        normal, offset = _refine(points[mask], up)
        if normal @ up < cos_max:
            return None
        mask = np.abs(points @ normal + offset) < threshold
        if int(mask.sum()) < min_inliers:
            return None

    residuals = points[mask] @ normal + offset
    return PlaneFit(
        normal=normal,
        offset=offset,
        inliers=int(mask.sum()),
        candidates=n_pts,
        rms=float(np.sqrt(np.mean(residuals**2))),
    )
