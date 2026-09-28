"""Depth error models used to correct monocular point clouds.

ROS-free (pure numpy, no rclpy/tf2 imports) so the math is unit-testable on
the host without a ROS install — same policy as
`ground_plane_estimation/plane_fit.py`.

The working hypothesis (see documentation/ground_plane_estimation.md) is

    depth_pred ≈ s · depth_true              (scalar model)

with the suspected fallback

    depth_pred ≈ s · depth_true + b          (affine model)

Each hypothesis is one `CorrectionModel` subclass. The node feeds every model
the same two things and knows nothing about the math inside:

  * `update(ScaleObservation)` — a new estimate of the error parameters
    (today: `/ground_plane/scale`; a future estimator may add `shift`).
  * `correct(points)` — an (N, 3) cloud in the *camera optical frame*
    (z = depth along the optical axis), returning the corrected copy.

To add a new error model, subclass `CorrectionModel` and register it in
`MODELS` — nothing in the node changes.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ScaleObservation:
    """One measurement of the depth error parameters.

    stamp  — seconds on any single monotonic clock; only used for staleness.
    scale  — multiplicative error `s` in depth_pred ≈ s·depth_true + b.
    shift  — additive error `b` in metres; 0.0 when the estimator cannot
             observe it (the ground-plane probe alone cannot separate s
             from b — a shift estimate has to come from a second observable).
    """

    stamp: float
    scale: float
    shift: float = 0.0


class ParameterFilter:
    """Streaming smoother for one scalar parameter.

    Modes: `none` (last value wins), `ema` (exponential moving average),
    `median` (rolling median — robust to single bad plane fits).
    """

    MODES = ('none', 'ema', 'median')

    def __init__(self, mode: str = 'ema', ema_alpha: float = 0.2,
                 median_window: int = 5):
        if mode not in self.MODES:
            raise ValueError(f'unknown smoothing mode {mode!r}, '
                             f'expected one of {self.MODES}')
        self._mode = mode
        self._alpha = float(ema_alpha)
        self._window: deque = deque(maxlen=max(int(median_window), 1))
        self._value: float | None = None

    def update(self, raw: float) -> float:
        if self._mode == 'ema':
            self._value = raw if self._value is None else \
                self._alpha * raw + (1.0 - self._alpha) * self._value
        elif self._mode == 'median':
            self._window.append(raw)
            self._value = float(np.median(self._window))
        else:
            self._value = raw
        return self._value

    @property
    def value(self) -> float | None:
        return self._value


class CorrectionModel(ABC):
    """Base class: consume observations, correct point clouds.

    Contract for subclasses:
      * `update()` and `correct()` are called at independent rates; the model
        holds whatever state it needs in between.
      * `correct()` returns a NEW array, never mutates its input, and lets
        non-finite rows pass through unchanged (NaN in → NaN out).
      * `ready()` is False until enough observations have arrived to correct
        anything; the node decides what to do with unready/stale models.
      * `state()` reports the currently-applied parameters for logging and
        the per-parameter status topics.
    """

    name = 'abstract'

    def __init__(self, smoothing: str = 'ema', ema_alpha: float = 0.2,
                 median_window: int = 5, min_scale: float = 0.05,
                 max_scale: float = 20.0):
        # Gate against garbage observations (a sign-flipped or degenerate
        # plane fit shows up as a negative or wild scale).
        self._min_scale = float(min_scale)
        self._max_scale = float(max_scale)
        self._filter_args = {'mode': smoothing, 'ema_alpha': ema_alpha,
                             'median_window': median_window}
        self._last_stamp: float | None = None

    def _make_filter(self) -> ParameterFilter:
        return ParameterFilter(**self._filter_args)

    def update(self, obs: ScaleObservation) -> bool:
        """Feed one observation. Returns False if the gate rejected it."""
        if not np.isfinite(obs.scale) or not np.isfinite(obs.shift):
            return False
        if not (self._min_scale <= obs.scale <= self._max_scale):
            return False
        self._ingest(obs)
        self._last_stamp = obs.stamp
        return True

    @abstractmethod
    def _ingest(self, obs: ScaleObservation) -> None:
        """Fold a gate-approved observation into the model state."""

    @abstractmethod
    def correct(self, points: np.ndarray) -> np.ndarray:
        """Corrected copy of an (N, 3) camera-optical-frame cloud."""

    @abstractmethod
    def ready(self) -> bool:
        """Report whether `correct()` has parameters to apply."""

    @abstractmethod
    def state(self) -> dict:
        """Currently-applied parameters, e.g. {'scale': 1.8, 'shift': 0.0}."""

    @property
    def last_stamp(self) -> float | None:
        """Stamp of the last accepted observation (staleness checks)."""
        return self._last_stamp


class ScalarCorrection(CorrectionModel):
    """Pure multiplicative error: p_true = p_pred / s.

    Uniform scaling commutes with rotation, so this is valid in any frame,
    not just the camera optical frame.
    """

    name = 'scalar'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._scale = self._make_filter()

    def _ingest(self, obs: ScaleObservation) -> None:
        self._scale.update(obs.scale)

    def correct(self, points: np.ndarray) -> np.ndarray:
        return points / self._scale.value

    def ready(self) -> bool:
        return self._scale.value is not None

    def state(self) -> dict:
        return {'scale': self._scale.value}


class AffineCorrection(CorrectionModel):
    """Affine depth error: depth_pred = s · depth_true + b.

    Correction happens in depth space: z_true = (z_pred − b) / s. A
    projective point is p = z · (x/z, y/z, 1), and the back-projection ray
    (x/z, y/z, 1) is depth-independent, so the whole point moves along its
    ray by the depth ratio:

        p_true = p_pred · (z_true / z_pred)

    This REQUIRES the cloud in the camera optical frame (z = depth along the
    optical axis, REP-103 optical convention) — unlike the scalar model, an
    additive depth shift does not commute with rotation. Points at or behind
    the camera (z ≤ eps) and points whose corrected depth would be ≤ 0 are
    NaN-ed out rather than reflected through the origin.

    With shift ≡ 0 this reduces exactly to `ScalarCorrection`. It is wired
    up now so that the moment an estimator for `b` exists, only the
    observation source changes. (The IMU/leg-odometry motion probe now
    exists — `odometry_scale_estimation` — but its affine intercept lives in
    displacement space, not depth space; a true depth-space `b` needs the
    two probes combined, or a two-plane fit.)
    """

    name = 'affine'

    #: depths below this (metres) are treated as degenerate
    Z_EPS = 1e-6

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._scale = self._make_filter()
        self._shift = self._make_filter()

    def _ingest(self, obs: ScaleObservation) -> None:
        self._scale.update(obs.scale)
        self._shift.update(obs.shift)

    def correct(self, points: np.ndarray) -> np.ndarray:
        s, b = self._scale.value, self._shift.value
        z = points[:, 2]
        with np.errstate(invalid='ignore', divide='ignore'):
            z_true = (z - b) / s
            factor = np.where(z > self.Z_EPS, z_true / z, np.nan)
            factor = np.where(z_true > 0.0, factor, np.nan)
        return points * factor[:, None]

    def ready(self) -> bool:
        return self._scale.value is not None

    def state(self) -> dict:
        return {'scale': self._scale.value, 'shift': self._shift.value}


MODELS: dict[str, type[CorrectionModel]] = {
    ScalarCorrection.name: ScalarCorrection,
    AffineCorrection.name: AffineCorrection,
}


def make_model(name: str, **kwargs) -> CorrectionModel:
    """Instantiate a registered correction model by name."""
    try:
        cls = MODELS[name]
    except KeyError:
        raise ValueError(f'unknown correction model {name!r}, '
                         f'expected one of {sorted(MODELS)}') from None
    return cls(**kwargs)
