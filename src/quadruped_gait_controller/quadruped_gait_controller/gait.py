"""Bezier-swing gait generator (Spot Micro / spot_mini_mini lineage).

Pure numpy, no ROS: the node feeds it a global gait phase and the body
velocity command and gets back per-leg foot offsets from the neutral
footholds, expressed in the nominal body frame N (see
quadruped_kinematics.body). The node then adds body pose and runs body IK.

Cycle timing
------------
A gait is a duty factor (stance fraction of the cycle) plus, per leg, the
cycle fraction at which its swing starts. Each leg's local phase runs
stance first (0 → duty) then swing (duty → 1).

    trot  duty 0.50   FR+RL swing at 0.0, FL+RR swing at 0.5
    walk  duty 0.75   RL, FL, RR, FR swing one at a time (lateral sequence:
                      a rear leg, then the front leg on the same side)

Foot trajectories
-----------------
Stance: the foot sweeps linearly from +Δ/2 to −Δ/2, where Δ is the ground
displacement the body makes during one stance period at the commanded
velocity, evaluated at that foot (so yaw rate turns into tangential strides).
Swing: an 11th-order Bézier curve from −Δ/2 back to +Δ/2 lifted by the
clearance (control points after Hyun et al. 2014, "High speed trot-running",
as used by the MIT Cheetah and spot_mini_mini). The curve starts and ends
tangent to the ground with zero velocity, so foot touchdown is soft.

The two segments meet at both ends, so foot position is continuous across
the whole cycle regardless of where the velocity command changes.
"""

import math
from dataclasses import dataclass, field
from math import comb

import numpy as np

from quadruped_kinematics.geometry import LEGS

# Per-gait defaults, tuned in headless Gazebo on the Spot Micro model (flat
# ground, position control through gz_ros2_control's velocity loop):
#
#   trot  0.4 s cycle, duty 0.7: 0.12 s swings with 4-foot support in between.
#         Walks straight at 0.2–0.3 m/s with <2° roll/pitch and turns at the
#         commanded rate. Duty 0.5 (a true flight trot) tips the body on its
#         diagonal support and falls within seconds under position control.
#   walk  1.0 s cycle, duty 0.75: one foot in the air at a time, body shifted
#         80 % of the way into the support triangle (Spot Micro's body-shift
#         phases). Statically stable and slow; turning is sloppy.
#
# `swing_start` is each leg's swing onset as a fraction of the cycle.
GAIT_LIBRARY: dict[str, dict] = {
    'trot': {
        'duty': 0.7,
        'period': 0.4,
        'clearance': 0.02,
        'body_shift_gain': 0.0,
        'body_shift_tau': 0.1,
        'swing_start': {'FL': 0.5, 'FR': 0.0, 'RL': 0.0, 'RR': 0.5},
    },
    'walk': {
        'duty': 0.75,
        'period': 1.0,
        'clearance': 0.03,
        'body_shift_gain': 0.8,
        'body_shift_tau': 0.08,
        'swing_start': {'RL': 0.0, 'FL': 0.25, 'RR': 0.5, 'FR': 0.75},
    },
}

# Hyun et al. 2014 Bézier control points, normalised: x in units of the half
# stride L, z in units of the clearance h.
_BEZIER_X = np.array([-1.0, -1.4, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 1.5, 1.5, 1.4, 1.0])
_BEZIER_Z = np.array([0.0, 0.0, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 1.1, 0.0, 0.0])
_BEZIER_N = len(_BEZIER_X) - 1
_BINOM = np.array([comb(_BEZIER_N, k) for k in range(_BEZIER_N + 1)], dtype=float)


def bezier_swing(s: float, half_stride: float, clearance: float) -> tuple[float, float]:
    """Swing foot (along-stride, up) at swing progress s ∈ [0, 1]."""
    s = min(max(s, 0.0), 1.0)
    k = np.arange(_BEZIER_N + 1)
    basis = _BINOM * s ** k * (1.0 - s) ** (_BEZIER_N - k)
    return float(basis @ _BEZIER_X) * half_stride, float(basis @ _BEZIER_Z) * clearance


@dataclass
class GaitParams:
    period: float = 0.4            # s per full cycle
    duty: float = 0.7              # stance fraction of the cycle
    swing_start: dict[str, float] = field(
        default_factory=lambda: dict(GAIT_LIBRARY['trot']['swing_start']))
    clearance: float = 0.02        # m swing height
    max_step_length: float = 0.10  # m, |Δ| cap (total foot travel per cycle)
    stance_depth: float = 0.0      # m, sinusoidal push below ground in stance
    body_shift_gain: float = 0.0   # 0..1 of the way toward the support centroid
    body_shift_tau: float = 0.1    # s, low-pass on the body shift

    @classmethod
    def from_library(cls, gait: str, **overrides) -> 'GaitParams':
        """Tuned defaults for a gait, with any override that is not None applied."""
        entry = GAIT_LIBRARY[gait]
        params = cls(
            duty=entry['duty'], period=entry['period'], clearance=entry['clearance'],
            body_shift_gain=entry['body_shift_gain'], body_shift_tau=entry['body_shift_tau'],
            swing_start=dict(entry['swing_start']))
        for key, value in overrides.items():
            if value is not None:
                setattr(params, key, value)
        return params

    @property
    def swing_duration(self) -> float:
        return (1.0 - self.duty) * self.period

    @property
    def stance_duration(self) -> float:
        return self.duty * self.period


class BezierGait:
    """Foot offsets from neutral footholds for a global gait phase."""

    def __init__(self, params: GaitParams, neutral_feet: dict[str, np.ndarray]):
        if not 0.0 < params.duty < 1.0:
            raise ValueError('duty factor must be in (0, 1)')
        if params.period <= 0.0:
            raise ValueError('period must be positive')
        self.params = params
        self.neutral = {leg: np.asarray(neutral_feet[leg], dtype=float) for leg in LEGS}

    # ── timing ──────────────────────────────────────────────────────────
    def leg_phase(self, phase: float, leg: str) -> tuple[float, bool]:
        """(local phase ∈ [0, 1), in_stance) for a leg at global phase."""
        local = (phase - self.params.swing_start[leg] + self.params.duty) % 1.0
        return local, local < self.params.duty

    def stance_mask(self, phase: float) -> dict[str, bool]:
        return {leg: self.leg_phase(phase, leg)[1] for leg in LEGS}

    def support_centroid(self, phase: float) -> np.ndarray:
        """Centroid (2,) of the neutral footholds currently in stance, in N.

        For a crawl this is the support triangle's centre: shifting the body
        toward it keeps the centre of mass inside the polygon while one leg
        is in the air (Spot Micro's body-shift phases). For a trot the two
        diagonal feet straddle the origin and the centroid stays near zero.
        """
        stance = [self.neutral[leg][:2] for leg, up in self.stance_mask(phase).items() if up]
        if not stance:
            return np.zeros(2)
        return np.mean(stance, axis=0)

    # ── stride planning ────────────────────────────────────────────────
    def stride_vectors(self, vx: float, vy: float, wz: float) -> dict[str, np.ndarray]:
        """Ground displacement Δ (2,) of each foot over one stance period.

        Body velocity at foothold r is v + ω × r; the foot must travel the
        opposite way relative to the body, so Δ is the body's displacement
        during stance. All legs are scaled by one factor when any Δ exceeds
        the step cap, which keeps a turn a turn instead of distorting it.
        """
        t_stance = self.params.stance_duration
        strides = {}
        for leg in LEGS:
            rx, ry = self.neutral[leg][:2]
            strides[leg] = np.array([vx - wz * ry, vy + wz * rx]) * t_stance
        longest = max(float(np.linalg.norm(d)) for d in strides.values())
        if longest > self.params.max_step_length:
            scale = self.params.max_step_length / longest
            strides = {leg: d * scale for leg, d in strides.items()}
        return strides

    # ── trajectories ───────────────────────────────────────────────────
    def foot_offsets(self, phase: float, vx: float, vy: float, wz: float) -> dict[str, np.ndarray]:
        """Offset (3,) of each foot from its neutral foothold, in N."""
        strides = self.stride_vectors(vx, vy, wz)
        offsets = {}
        for leg in LEGS:
            local, in_stance = self.leg_phase(phase, leg)
            delta = strides[leg]
            length = float(np.linalg.norm(delta))
            direction = delta / length if length > 1e-9 else np.zeros(2)
            if in_stance:
                s = local / self.params.duty
                # A small push below the nominal ground keeps the loaded
                # legs pressing under position control, so a swing foot
                # touching down early cannot unload them (spot_mini_mini's
                # penetration depth).
                along = (0.5 - s) * length
                up = -self.params.stance_depth * math.sin(math.pi * s)
            else:
                s = (local - self.params.duty) / (1.0 - self.params.duty)
                along, up = bezier_swing(s, length / 2.0, self.params.clearance)
            offsets[leg] = np.array([direction[0] * along, direction[1] * along, up])
        return offsets

    def feet(self, phase: float, vx: float, vy: float, wz: float) -> dict[str, np.ndarray]:
        """Absolute foot positions in N."""
        offsets = self.foot_offsets(phase, vx, vy, wz)
        return {leg: self.neutral[leg] + offsets[leg] for leg in LEGS}
