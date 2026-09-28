"""Gait generator tests (no ROS)."""

import numpy as np
import pytest

from quadruped_kinematics import SPOT_MICRO, neutral_feet
from quadruped_kinematics.geometry import LEGS, LEG_SIGNS
from quadruped_gait_controller.gait import GAIT_LIBRARY, BezierGait, GaitParams, bezier_swing

HEIGHT = 0.17


def _gait(kind='trot', **kw):
    return BezierGait(GaitParams.from_library(kind, **kw), neutral_feet(HEIGHT, SPOT_MICRO))


def test_bezier_swing_endpoints_and_lift():
    x0, z0 = bezier_swing(0.0, 0.05, 0.04)
    x1, z1 = bezier_swing(1.0, 0.05, 0.04)
    assert (x0, z0) == pytest.approx((-0.05, 0.0))
    assert (x1, z1) == pytest.approx((0.05, 0.0))
    zs = [bezier_swing(s, 0.05, 0.04)[1] for s in np.linspace(0, 1, 101)]
    assert min(zs) >= -1e-12
    assert 0.6 * 0.04 < max(zs) <= 0.04 * 1.1 + 1e-9


def test_bezier_swing_touchdown_is_soft():
    # Zero vertical velocity at lift-off and touchdown (first two / last two
    # control points coincide in z).
    eps = 1e-4
    _, z_a = bezier_swing(eps, 0.05, 0.04)
    _, z_b = bezier_swing(1.0 - eps, 0.05, 0.04)
    assert z_a < 1e-6 and z_b < 1e-6


def test_trot_pairs_diagonal_legs():
    gait = _gait('trot')
    swung = set()
    for phase in np.linspace(0, 1, 200, endpoint=False):
        mask = gait.stance_mask(phase)
        assert mask['FL'] == mask['RR']
        assert mask['FR'] == mask['RL']
        # the two pairs never swing together; with duty > 0.5 they overlap in stance
        assert mask['FL'] or mask['FR']
        swung |= {leg for leg, up in mask.items() if not up}
    assert swung == set(LEGS)


def test_flight_trot_alternates_pairs_every_half_cycle():
    gait = _gait('trot', duty=0.5)
    for phase in np.linspace(0, 1, 50, endpoint=False):
        mask = gait.stance_mask(phase)
        assert mask['FL'] != mask['FR']


def test_walk_swings_one_leg_at_a_time_in_lateral_sequence():
    gait = _gait('walk')
    order = []
    for phase in np.linspace(0, 1, 400, endpoint=False):
        swinging = [leg for leg, st in gait.stance_mask(phase).items() if not st]
        assert len(swinging) == 1
        if not order or order[-1] != swinging[0]:
            order.append(swinging[0])
    assert order == ['RL', 'FL', 'RR', 'FR']


def test_library_swing_starts_cover_every_leg():
    for kind, entry in GAIT_LIBRARY.items():
        assert set(entry['swing_start']) == set(LEGS), kind


def test_zero_command_keeps_stance_feet_at_neutral():
    gait = _gait('trot')
    for phase in np.linspace(0, 1, 20, endpoint=False):
        offsets = gait.foot_offsets(phase, 0.0, 0.0, 0.0)
        for leg in LEGS:
            assert np.allclose(offsets[leg][:2], 0.0)
            assert offsets[leg][2] >= 0.0


def test_forward_stride_direction_and_length():
    gait = _gait('trot', period=0.6, duty=0.5)
    strides = gait.stride_vectors(0.1, 0.0, 0.0)
    for leg in LEGS:
        assert strides[leg] == pytest.approx([0.1 * 0.5 * 0.6, 0.0])   # v * duty * period


def test_stride_cap_scales_all_legs_uniformly():
    gait = _gait('trot', period=0.6, max_step_length=0.05)
    strides = gait.stride_vectors(1.0, 0.0, 0.5)
    lengths = [np.linalg.norm(d) for d in strides.values()]
    assert max(lengths) == pytest.approx(0.05)
    # ratio between legs preserved: recompute uncapped
    free = _gait('trot', period=0.6, max_step_length=10.0).stride_vectors(1.0, 0.0, 0.5)
    ratios = [np.linalg.norm(strides[leg]) / np.linalg.norm(free[leg]) for leg in LEGS]
    assert np.allclose(ratios, ratios[0])


def test_yaw_command_gives_tangential_strides():
    gait = _gait('trot')
    strides = gait.stride_vectors(0.0, 0.0, 1.0)
    for leg in LEGS:
        r = gait.neutral[leg][:2]
        assert np.dot(strides[leg], r) == pytest.approx(0.0, abs=1e-12)
        assert r[0] * strides[leg][1] - r[1] * strides[leg][0] > 0.0   # counter-clockwise


def test_foot_trajectory_is_continuous_over_the_cycle():
    gait = _gait('trot')
    phases = np.linspace(0, 1, 2000, endpoint=False)
    prev = gait.foot_offsets(phases[-1], 0.2, 0.05, 0.3)
    for phase in phases:
        cur = gait.foot_offsets(phase, 0.2, 0.05, 0.3)
        for leg in LEGS:
            assert np.linalg.norm(cur[leg] - prev[leg]) < 2e-3, (leg, phase)
        prev = cur


def test_stance_foot_moves_backward_when_walking_forward():
    gait = _gait('trot')
    for leg in LEGS:
        # find a phase where this leg is mid-stance
        for phase in np.linspace(0, 1, 100, endpoint=False):
            local, in_stance = gait.leg_phase(phase, leg)
            if in_stance and 0.2 < local / gait.params.duty < 0.8:
                a = gait.foot_offsets(phase, 0.2, 0.0, 0.0)[leg]
                b = gait.foot_offsets(phase + 0.01, 0.2, 0.0, 0.0)[leg]
                assert b[0] < a[0]
                assert a[2] == 0.0
                break


def test_swing_feet_lift_and_reach_within_ik_workspace():
    from quadruped_kinematics import body_ik, hip_origin, leg_fk
    gait = _gait('trot')
    for phase in np.linspace(0, 1, 60, endpoint=False):
        feet = gait.feet(phase, 0.4, 0.25, 1.2)
        q = body_ik(feet, clamp=False)
        for i, leg in enumerate(LEGS):
            back = hip_origin(leg) + leg_fk(q[3 * i:3 * i + 3], LEG_SIGNS[leg][1])
            assert np.allclose(back, feet[leg], atol=1e-9)


def test_invalid_params_rejected():
    with pytest.raises(ValueError):
        BezierGait(GaitParams(duty=1.0), neutral_feet(HEIGHT))
    with pytest.raises(ValueError):
        BezierGait(GaitParams(period=0.0), neutral_feet(HEIGHT))


def test_stance_depth_pushes_only_in_stance():
    gait = _gait('trot', stance_depth=0.006)
    seen_push = False
    for phase in np.linspace(0, 1, 100, endpoint=False):
        offsets = gait.foot_offsets(phase, 0.1, 0.0, 0.0)
        for leg in LEGS:
            _, in_stance = gait.leg_phase(phase, leg)
            if in_stance:
                assert -0.006 - 1e-12 <= offsets[leg][2] <= 0.0
                seen_push |= offsets[leg][2] < -0.003
            else:
                assert offsets[leg][2] >= 0.0
    assert seen_push
    # still continuous at the stance/swing boundaries
    prev = gait.foot_offsets(0.999, 0.1, 0.0, 0.0)
    for phase in np.linspace(0, 1, 1000, endpoint=False):
        cur = gait.foot_offsets(phase, 0.1, 0.0, 0.0)
        for leg in LEGS:
            assert np.linalg.norm(cur[leg] - prev[leg]) < 2e-3
        prev = cur


def test_support_centroid_walk_points_into_the_triangle():
    gait = _gait('walk')
    for phase in np.linspace(0, 1, 40, endpoint=False):
        mask = gait.stance_mask(phase)
        swinging = next(leg for leg, up in mask.items() if not up)
        c = gait.support_centroid(phase)
        # centroid lies away from the swinging leg's corner
        x_sign, y_sign = LEG_SIGNS[swinging]
        assert np.sign(c[0]) == -x_sign and np.sign(c[1]) == -y_sign
        assert np.linalg.norm(c) == pytest.approx(
            np.hypot(SPOT_MICRO.hip_x, SPOT_MICRO.hip_y + SPOT_MICRO.hip_link) / 3)


def test_support_centroid_trot_is_zero():
    gait = _gait('trot')
    for phase in np.linspace(0, 1, 40, endpoint=False):
        assert np.allclose(gait.support_centroid(phase), 0.0, atol=1e-12)


def test_library_defaults_and_overrides():
    walk = GaitParams.from_library('walk')
    assert walk.duty == 0.75 and walk.period == 1.0 and walk.body_shift_gain == 0.8
    trot = GaitParams.from_library('trot', period=None, clearance=0.05)
    assert trot.period == GAIT_LIBRARY['trot']['period']
    assert trot.clearance == 0.05
    assert trot.duty == 0.7
