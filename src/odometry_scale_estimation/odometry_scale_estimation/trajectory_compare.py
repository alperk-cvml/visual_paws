"""Trajectory-vs-trajectory scale estimation math.

ROS-free (pure numpy, no rclpy/tf2 imports) so the math is unit-testable on
the host without a ROS install — same policy as
`ground_plane_estimation/plane_fit.py`.

The observable is the CHORD: the straight-line displacement between two
instants of the same rigid body, reported by two different odometry sources.
A chord's length is invariant to each odometry's arbitrary frame origin and
orientation, so the two streams never need extrinsic alignment — only time
alignment. If the estimate stream rides on a monocular depth cloud with a
per-frame scale error `s` (KISS-ICP on a uniformly scaled world estimates a
uniformly scaled motion), then over a window

    chord_est ≈ s · chord_ref              (scalar model)

with the suspected fallback

    chord_est ≈ s · chord_ref + b          (affine model)

Chords are used instead of integrated path length because the path length of
a noisy trajectory is biased upward by the noise itself; a chord between two
sufficiently separated samples is not.

NOTE on `b`: this intercept lives in displacement space. It captures the
estimate stream's noise floor (ICP jitter shows up as a constant offset on
every chord) and is a *diagnostic* for a non-scalar depth error — it is NOT
numerically the depth-space shift of `pointcloud_scale_correction`'s affine
model. See the module README before wiring it there.

Each hypothesis is one `ScaleModel` subclass. To add a new model, subclass
`ScaleModel` and register it in `MODELS` — nothing in the node changes.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


class TrajectoryBuffer:
    """Sliding window of timestamped positions from one odometry stream.

    Keeps (t, xyz) samples sorted by time, discards samples older than
    `window` behind the newest one, and clears itself when the stream
    restarts — a timestamp going backwards (bag loop, node restart) or a
    position jump larger than `max_jump` (odometry re-initialised at the
    origin) both invalidate every chord across the discontinuity.
    """

    def __init__(self, window: float, max_gap: float = 0.5,
                 max_jump: float = 1.0):
        self._window = float(window)
        self._max_gap = float(max_gap)
        self._max_jump = float(max_jump)
        self._times: list[float] = []
        self._positions: list[np.ndarray] = []

    def __len__(self) -> int:
        return len(self._times)

    @property
    def latest_time(self) -> float | None:
        return self._times[-1] if self._times else None

    def append(self, t: float, position) -> None:
        position = np.asarray(position, dtype=float)
        if not (np.isfinite(t) and np.all(np.isfinite(position))):
            return
        if self._times:
            if t <= self._times[-1]:
                self.clear()  # time went backwards: stream restarted
            elif (np.linalg.norm(position - self._positions[-1])
                  > self._max_jump):
                self.clear()  # discontinuous pose: odometry re-initialised
        self._times.append(float(t))
        self._positions.append(position)
        self.prune(self._times[-1])

    def clear(self) -> None:
        self._times.clear()
        self._positions.clear()

    def prune(self, now: float) -> None:
        cutoff = now - self._window
        while self._times and self._times[0] < cutoff:
            self._times.pop(0)
            self._positions.pop(0)

    def snapshot(self, t_end: float) -> tuple[np.ndarray, np.ndarray]:
        """Samples in [t_end − window, t_end] as (times, positions)."""
        times = np.asarray(self._times)
        positions = (np.asarray(self._positions).reshape(-1, 3)
                     if self._positions else np.empty((0, 3)))
        keep = (times >= t_end - self._window) & (times <= t_end)
        return times[keep], positions[keep]

    def interpolate(self, ts) -> tuple[np.ndarray, np.ndarray]:
        """Linear interpolation at query times `ts`.

        Returns (positions (N,3), valid (N,)). A query is invalid outside
        the buffered time range or when its bracketing samples are more than
        `max_gap` apart (interpolating across a dropout fabricates motion).
        Invalid rows are NaN.
        """
        ts = np.atleast_1d(np.asarray(ts, dtype=float))
        out = np.full((len(ts), 3), np.nan)
        if len(self._times) < 2:
            return out, np.zeros(len(ts), dtype=bool)
        times = np.asarray(self._times)
        positions = np.asarray(self._positions)
        idx = np.clip(np.searchsorted(times, ts), 1, len(times) - 1)
        gap = times[idx] - times[idx - 1]
        valid = (ts >= times[0]) & (ts <= times[-1]) & (gap <= self._max_gap)
        for k in range(3):
            out[:, k] = np.interp(ts, times, positions[:, k])
        out[~valid] = np.nan
        return out, valid


def chord_pairs(times: np.ndarray, ref_pts: np.ndarray, est_pts: np.ndarray,
                min_dt: float = 0.5, max_samples: int = 40,
                ) -> tuple[np.ndarray, np.ndarray]:
    """Matched chord lengths (d_ref, d_est) between all sample pairs.

    `times`, `ref_pts`, `est_pts` are time-aligned samples of both streams
    (the reference already interpolated at the estimate's stamps). Every
    ordered pair at least `min_dt` apart contributes one chord; pairs of all
    separations are kept on purpose — the affine model needs chord-length
    spread, and long/short chords provide it even at constant speed. Samples
    are thinned evenly to `max_samples` first to bound the O(n²) pairing.
    """
    n = len(times)
    if n > max_samples:
        keep = np.round(np.linspace(0, n - 1, max_samples)).astype(int)
        times, ref_pts, est_pts = times[keep], ref_pts[keep], est_pts[keep]
        n = max_samples
    if n < 2:
        return np.empty(0), np.empty(0)

    dt = times[None, :] - times[:, None]
    mask = np.triu(dt >= min_dt)
    i, j = np.nonzero(mask)
    d_ref = np.linalg.norm(ref_pts[j] - ref_pts[i], axis=1)
    d_est = np.linalg.norm(est_pts[j] - est_pts[i], axis=1)
    return d_ref, d_est


@dataclass(frozen=True)
class ScaleFit:
    """One windowed estimate of the odometry scale parameters.

    scale      — multiplicative error `s` in chord_est ≈ s·chord_ref + b.
    shift      — displacement-space intercept `b` (m); None for the scalar
                 model, which pins it to 0 by construction.
    rms        — residual RMS of the fit over the gated pairs (m).
    n_pairs    — chord pairs that survived the gate and entered the fit.
    ref_motion — longest reference chord in the window (m): the excitation.
                 A fit is only as good as the motion that produced it.
    """

    scale: float
    shift: float | None
    rms: float
    n_pairs: int
    ref_motion: float


class ScaleModel(ABC):
    """Base class: fit scale parameters to matched chord pairs.

    Contract for subclasses: `fit()` returns a `ScaleFit`, or None when the
    window cannot support an estimate — too little motion, too few pairs, a
    degenerate or garbage solution. The node treats None as "publish
    nothing this cycle", never as an error.
    """

    name = 'abstract'

    def __init__(self, min_motion: float = 0.1, min_pairs: int = 3,
                 min_scale: float = 0.02, max_scale: float = 50.0,
                 min_chord: float = 0.01):
        # Excitation gate: a standing robot yields chords made of pure noise.
        self._min_motion = float(min_motion)
        self._min_pairs = int(min_pairs)
        # Per-pair ratio gate against ICP jumps / interpolation glitches.
        # Pairs with a reference chord below `min_chord` bypass it (their
        # ratio is noise-dominated, but they carry the noise-floor/shift
        # information and get ~zero weight in the scalar fit anyway).
        self._min_scale = float(min_scale)
        self._max_scale = float(max_scale)
        self._min_chord = float(min_chord)

    def fit(self, d_ref: np.ndarray, d_est: np.ndarray) -> ScaleFit | None:
        finite = np.isfinite(d_ref) & np.isfinite(d_est)
        d_ref, d_est = d_ref[finite], d_est[finite]
        if len(d_ref) == 0:
            return None
        ref_motion = float(d_ref.max())
        if ref_motion < self._min_motion:
            return None

        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = d_est / d_ref
        keep = (d_ref < self._min_chord) | (
            (ratio >= self._min_scale) & (ratio <= self._max_scale))
        d_ref, d_est = d_ref[keep], d_est[keep]
        if len(d_ref) < self._min_pairs:
            return None

        solved = self._solve(d_ref, d_est)
        if solved is None:
            return None
        scale, shift = solved
        if not np.isfinite(scale) or scale <= 0.0:
            return None
        residual = d_est - scale * d_ref - (shift or 0.0)
        return ScaleFit(
            scale=float(scale), shift=shift,
            rms=float(np.sqrt(np.mean(residual ** 2))),
            n_pairs=len(d_ref), ref_motion=ref_motion)

    @abstractmethod
    def _solve(self, d_ref: np.ndarray, d_est: np.ndarray
               ) -> tuple[float, float | None] | None:
        """(scale, shift) from gated pairs, or None if degenerate."""


class ScalarScaleModel(ScaleModel):
    """Pure multiplicative error: chord_est = s · chord_ref.

    Least squares through the origin, s = Σ(d_ref·d_est) / Σ(d_ref²) —
    long chords carry the most weight, which is where the signal-to-noise
    is best.
    """

    name = 'scalar'

    def _solve(self, d_ref, d_est):
        denom = float(d_ref @ d_ref)
        if denom < 1e-12:
            return None
        return float(d_ref @ d_est) / denom, None


class AffineScaleModel(ScaleModel):
    """Affine error: chord_est = s · chord_ref + b.

    Ordinary least squares with an intercept. Fitting an intercept needs
    SPREAD in the reference chord lengths (all-equal chords make s and b
    indistinguishable), so windows with std(d_ref) < `min_spread` are
    rejected rather than answered with an arbitrary split. Chord pairs of
    mixed separations (see `chord_pairs`) provide the spread naturally.
    """

    name = 'affine'

    def __init__(self, min_spread: float = 0.05, **kwargs):
        super().__init__(**kwargs)
        self._min_spread = float(min_spread)

    def _solve(self, d_ref, d_est):
        if float(np.std(d_ref)) < self._min_spread:
            return None
        design = np.column_stack((d_ref, np.ones_like(d_ref)))
        (scale, shift), *_ = np.linalg.lstsq(design, d_est, rcond=None)
        return float(scale), float(shift)


MODELS: dict[str, type[ScaleModel]] = {
    ScalarScaleModel.name: ScalarScaleModel,
    AffineScaleModel.name: AffineScaleModel,
}


def make_model(name: str, **kwargs) -> ScaleModel:
    """Instantiate a registered scale model by name."""
    try:
        cls = MODELS[name]
    except KeyError:
        raise ValueError(f'unknown scale model {name!r}, '
                         f'expected one of {sorted(MODELS)}') from None
    return cls(**kwargs)


def quat_to_rot(q) -> np.ndarray:
    """Unit quaternion (w, x, y, z) → 3x3 rotation matrix."""
    w, x, y, z = np.asarray(q, dtype=float)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def motion_in_body(displacement: np.ndarray, q_body_to_odom) -> np.ndarray:
    """Express an odom-frame displacement in the body frame.

    `q_body_to_odom` is the odometry pose orientation (w, x, y, z) at the
    window end — nav_msgs/Odometry pose orientation is exactly that. Each
    stream's displacement is rotated by ITS OWN attitude, so the two motion
    arrows share the base_link origin without any cross-stream extrinsic:
    a heading disagreement between the streams stays visible as an angle
    between the arrows, a scale error as a length mismatch.
    """
    return quat_to_rot(q_body_to_odom).T @ np.asarray(displacement, float)


def align_estimate(ref_pts: np.ndarray, est_pts: np.ndarray) -> np.ndarray:
    """Rigidly align the estimate trajectory onto the reference (Kabsch).

    Rotation + translation ONLY — no scaling — so when the two windows are
    overlaid (the RViz marker) a scale error stays visible as a length
    mismatch instead of being absorbed by the alignment. Fewer than three
    points, or a degenerate point set, falls back to centroid translation.
    """
    ref_pts = np.asarray(ref_pts, dtype=float)
    est_pts = np.asarray(est_pts, dtype=float)
    ref_c = ref_pts.mean(axis=0)
    est_c = est_pts.mean(axis=0)
    a = est_pts - est_c
    b = ref_pts - ref_c
    if len(ref_pts) < 3:
        return a + ref_c
    h = a.T @ b
    if not np.all(np.isfinite(h)) or np.linalg.norm(h) < 1e-12:
        return a + ref_c
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return a @ rot.T + ref_c
