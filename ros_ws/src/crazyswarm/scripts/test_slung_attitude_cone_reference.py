"""Fixed-center cone attitude and gravity-balanced cable reference derivatives."""

import importlib

import numpy as np
import pytest

from slung_circle_reference import CircleTrajectoryConfig
from slung_reference import _hat, _rotation_exp


def _module():
    return importlib.import_module("slung_attitude_cone_reference")


def _reference(**changes):
    return _module().PayloadConeReference([1., -2., .6], 20.,
        CircleTrajectoryConfig.from_mapping(dict(mode="attitude_cone", **changes)))


def test_cone_configuration_and_default_timing_keep_center_fixed():
    reference = _reference()
    assert reference.duration == 40.
    assert reference.end_time == 60.
    assert reference.initial_yaw == 0.
    np.testing.assert_array_equal(reference.initial_rotation, np.eye(3))
    np.testing.assert_array_equal(reference.entry_position, [1., -2., .6])
    np.testing.assert_array_equal(reference.center_position, [1., -2., .6])
    for now in np.linspace(0., 70., 301):
        sample = reference.evaluate(now)
        np.testing.assert_array_equal(sample["position"], [1., -2., .6])
        np.testing.assert_array_equal(sample["velocity"], np.zeros(3))
        np.testing.assert_array_equal(sample["acceleration"], np.zeros(3))


def test_explicit_center_is_copied_and_preserves_configured_hover_height():
    center = [0., 0.]
    config = CircleTrajectoryConfig(mode="attitude_cone", center_xy=center)
    center[:] = [99., 99.]
    reference = _module().PayloadConeReference([1., 2., .6], 20., config)
    np.testing.assert_array_equal(reference.entry_position, [0., 0., .6])


@pytest.mark.parametrize("direction,sign", [("ccw", 1.), ("cw", -1.)])
def test_body_z_traces_ten_degree_cone_without_full_body_yaw_spin(direction, sign):
    reference = _reference(direction=direction)
    angle = np.deg2rad(10.)
    previous = None
    for now in np.linspace(25., 55., 301):
        sample = reference.evaluate(now)
        phase, rotation = sample["cone_phase_rad"], sample["rotation"]
        expected_axis = [np.cos(phase)*np.sin(angle), np.sin(phase)*np.sin(angle), np.cos(angle)]
        np.testing.assert_allclose(rotation[:, 2], expected_axis, atol=1e-14)
        np.testing.assert_allclose(rotation.T@rotation, np.eye(3), atol=1e-14)
        assert np.linalg.det(rotation) == pytest.approx(1.)
        assert (np.trace(rotation)-1.)/2. == pytest.approx(np.cos(angle), abs=1e-14)
        assert abs(sample["yaw"]) < np.deg2rad(1.)
        if previous is not None:
            assert sign*(phase-previous) > 0.
        previous = phase
    assert previous == pytest.approx(sign*2.*np.pi)


@pytest.mark.parametrize("now", [0., 20., 25., 55., 60., 70.])
def test_phase_boundaries_are_stationary_and_final_attitude_is_level(now):
    reference = _reference()
    sample = reference.evaluate(now)
    expected = {0.: "waiting", 20.: "cone_tilt_in", 25.: "attitude_cone",
                55.: "cone_tilt_out", 60.: "complete", 70.: "complete"}
    assert sample["phase"] == expected[now]
    np.testing.assert_array_equal(sample["body_rate"], np.zeros(3))
    np.testing.assert_array_equal(sample["body_rate_dot"], np.zeros(3))
    assert sample["cone_phase_rate"] == sample["cone_phase_acceleration"] == 0.
    if now in (0., 20., 60., 70.):
        np.testing.assert_array_equal(sample["rotation"], np.eye(3))
    if now >= 60.:
        assert sample["progress"] == sample["cone_progress"] == 1.


@pytest.mark.parametrize("direction", ["ccw", "cw"])
@pytest.mark.parametrize("now", [20., 21.7, 25., 27.5, 34.5, 40., 48., 55., 57.3, 60.])
def test_rotation_rate_acceleration_and_euler_metadata_match_finite_differences(direction, now):
    reference = _reference(direction=direction)
    step = 1.e-4
    before, middle, after = [reference.evaluate(t) for t in (now-step, now, now+step)]
    # The jerk changes at phase joins. A second-order one-sided derivative
    # avoids the O(step) truncation of a centered derivative across that join.
    def derivative(name):
        if now in (20., 25., 55., 60.):
            second = reference.evaluate(now+2.*step)
            return (-3.*middle[name]+4.*after[name]-second[name])/(2.*step)
        return (after[name]-before[name])/(2.*step)

    np.testing.assert_allclose(derivative("rotation"),
                               middle["rotation"]@_hat(middle["body_rate"]), atol=2e-9)
    np.testing.assert_allclose(derivative("body_rate"),
                               middle["body_rate_dot"], atol=2e-9)
    assert derivative("cone_phase_rad") == pytest.approx(
        middle["cone_phase_rate"], abs=2e-9)
    assert derivative("cone_phase_rate") == pytest.approx(
        middle["cone_phase_acceleration"], abs=2e-9)
    assert derivative("yaw") == pytest.approx(middle["yaw_rate"], abs=2e-9)
    assert derivative("yaw_rate") == pytest.approx(
        middle["yaw_acceleration"], abs=2e-9)


def test_multiple_cycles_extend_sweep_and_progress_without_changing_transition_time():
    reference = _reference(revolutions=2.)
    assert reference.duration == 70.
    assert reference.evaluate(25.)["phase"] == "attitude_cone"
    middle = reference.evaluate(55.)
    assert middle["cone_phase_rad"] == pytest.approx(2.*np.pi)
    assert middle["cone_progress"] == pytest.approx(.5)
    assert reference.evaluate(85.)["phase"] == "cone_tilt_out"
    assert reference.evaluate(90.)["progress"] == 1.


@pytest.mark.parametrize("direction", ["ccw", "cw"])
def test_each_cone_cycle_repeats_the_same_thirty_second_reference(direction):
    reference = _reference(revolutions=3., direction=direction)
    sign = 1. if direction == "ccw" else -1.
    assert reference.duration == 100.
    for offset in (0., 3., 10., 15., 22., 29.9):
        first = reference.evaluate(25.+offset)
        for cycle in (1, 2):
            sample = reference.evaluate(25.+30.*cycle+offset)
            assert sample["cone_phase_rad"] == pytest.approx(first["cone_phase_rad"]+sign*2.*np.pi*cycle)
            for field in ("rotation", "body_rate", "body_rate_dot"):
                np.testing.assert_allclose(sample[field], first[field], atol=1.e-13)
    for cycle in (1, 2):
        sample = reference.evaluate(25.+30.*cycle)
        assert sample["phase"] == "attitude_cone"
        assert sample["cone_phase_rad"] == pytest.approx(sign*2.*np.pi*cycle)
        assert sample["cone_progress"] == pytest.approx(cycle/3.)
        np.testing.assert_allclose(sample["rotation"][:, 2],
            [np.sin(np.deg2rad(10.)), 0., np.cos(np.deg2rad(10.))], atol=1.e-13)
        np.testing.assert_allclose(sample["body_rate"], 0., atol=1.e-13)
        np.testing.assert_allclose(sample["body_rate_dot"], 0., atol=1.e-13)
        left = reference.evaluate(25.+30.*cycle-1.e-5)
        right = reference.evaluate(25.+30.*cycle+1.e-5)
        np.testing.assert_allclose(left["rotation"], right["rotation"], atol=1.e-12)
        np.testing.assert_allclose(left["body_rate"], right["body_rate"], atol=1.e-12)
        np.testing.assert_allclose(left["body_rate_dot"], right["body_rate_dot"], atol=1.e-7)


@pytest.mark.parametrize("field,value", [
    ("cone_half_angle_deg", 0.), ("cone_half_angle_deg", -10.), ("cone_half_angle_deg", 90.),
    ("cone_half_angle_deg", np.nan), ("cone_half_angle_deg", True),
    ("cone_period_s", 0.), ("cone_period_s", np.inf), ("cone_period_s", "30"),
    ("cone_transition_s", -1.), ("cone_transition_s", np.nan),
    ("revolutions", .5), ("revolutions", 1.5),
])
def test_invalid_cone_parameters_are_rejected(field, value):
    with pytest.raises(ValueError):
        CircleTrajectoryConfig.from_mapping(dict(mode="attitude_cone", **{field: value}))


def test_new_cone_fields_are_validated_even_in_hover_and_outputs_do_not_alias():
    with pytest.raises(ValueError):
        CircleTrajectoryConfig(mode="hover", cone_period_s=np.nan)
    reference = _reference()
    with pytest.raises(ValueError):
        reference.evaluate(np.nan)
    for values in (reference.entry_position, reference.center_position, reference.initial_rotation,
                   reference.evaluate(30.)["rotation"]):
        values[:] = 99.
    np.testing.assert_array_equal(reference.entry_position, [1., -2., .6])
    np.testing.assert_array_equal(reference.initial_rotation, np.eye(3))


@pytest.mark.parametrize("position,start", [([0., 0.], 20.), ([0., np.nan, .6], 20.),
                                            ([0., 0., .6], np.inf)])
def test_invalid_initial_positions_and_times_are_rejected(position, start):
    with pytest.raises(ValueError):
        _module().PayloadConeReference(position, start, CircleTrajectoryConfig(mode="attitude_cone"))


def _allocation():
    attachments = np.array([[.04, -.04, -.04], [0., .03, -.03], [.025, .025, .025]])
    matrix = np.vstack((np.tile(np.eye(3), (1, 3)),
                        np.hstack([_hat(attachments[:, i]) for i in range(3)])))
    allocation = matrix.T@np.linalg.inv(matrix@matrix.T)
    proposed = np.array([.03, 0., 0., -.02, .015, 0., -.02, -.015, 0.])
    bias = (np.eye(9)-allocation@matrix)@proposed
    return attachments, matrix, allocation, bias, np.array([0., 0., .023*9.80665])


def test_cable_directions_reconstruct_the_static_gravity_wrench_at_tilt():
    helper = _module().equilibrium_cable_reference
    _, matrix, allocation, bias, gravity = _allocation()
    for now in (21.7, 25., 34.5, 40., 48., 57.3):
        sample = _reference().evaluate(now)
        rotation = sample["rotation"]
        cable = helper(rotation, sample["body_rate"], sample["body_rate_dot"], allocation, bias, gravity)
        world_forces = rotation@(allocation@np.r_[rotation.T@gravity, np.zeros(3)]+bias).reshape(3, 3).T
        tensions = np.linalg.norm(world_forces, axis=0)
        transmitted = -cable["directions"]*tensions
        wrench = matrix@(rotation.T@transmitted).T.reshape(-1)
        np.testing.assert_allclose(rotation@wrench[:3], gravity, atol=1e-15)
        np.testing.assert_allclose(wrench[3:], np.zeros(3), atol=1e-15)
        assert np.all(cable["directions"][2] < 0.)


@pytest.mark.parametrize("now", [20., 21.7, 25., 34.5, 40., 48., 55., 57.3, 60.])
def test_cable_rates_match_motion_and_unit_sphere_constraints(now):
    helper = _module().equilibrium_cable_reference
    _, _, allocation, bias, gravity = _allocation()
    step = 1.e-4
    values = []
    for time in (now-step, now, now+step):
        sample = _reference().evaluate(time)
        values.append(helper(sample["rotation"], sample["body_rate"], sample["body_rate_dot"],
                             allocation, bias, gravity))
    before, middle, after = values
    q, qdot, qddot = (middle[name] for name in ("directions", "direction_rates", "direction_accelerations"))
    def derivative(name):
        if now in (20., 25., 55., 60.):
            sample = _reference().evaluate(now+2.*step)
            second = helper(sample["rotation"], sample["body_rate"], sample["body_rate_dot"],
                            allocation, bias, gravity)
            return (-3.*middle[name]+4.*after[name]-second[name])/(2.*step)
        return (after[name]-before[name])/(2.*step)

    np.testing.assert_allclose(derivative("directions"), qdot, atol=2e-9)
    np.testing.assert_allclose(derivative("direction_rates"), qddot, atol=2e-9)
    np.testing.assert_allclose(np.sum(q*q, axis=0), np.ones(3), atol=1e-14)
    np.testing.assert_allclose(np.sum(q*qdot, axis=0), np.zeros(3), atol=1e-14)
    np.testing.assert_allclose(np.sum(q*qddot, axis=0), -np.sum(qdot*qdot, axis=0), atol=1e-14)


@pytest.mark.parametrize("index,value", [(0, np.zeros((3, 3))), (1, [0., np.nan, 0.]),
    (2, [0., 0.]), (3, np.zeros((9, 3))), (4, np.zeros((3, 3))), (5, [0., np.inf, 0.])])
def test_cable_helper_rejects_invalid_inputs(index, value):
    helper = _module().equilibrium_cable_reference
    _, _, allocation, bias, gravity = _allocation()
    args = [np.eye(3), np.zeros(3), np.zeros(3), allocation, bias, gravity]
    args[index] = value
    with pytest.raises(ValueError):
        helper(*args)


def test_cable_helper_rejects_zero_allocated_tension():
    helper = _module().equilibrium_cable_reference
    with pytest.raises(ValueError):
        helper(np.eye(3), np.zeros(3), np.zeros(3), np.zeros((9, 6)), np.zeros(9), np.zeros(3))


def test_cable_directions_match_the_existing_payload_controller_equilibrium_allocator():
    from slung_load_controller import SlungLoadConfig, SlungLoadController
    attachments, _, _, _, gravity = _allocation()
    controller = SlungLoadController(SlungLoadConfig(
        payload_mass_kg=.023, gravity_mps2=9.80665, payload_size_m=np.array([.08, .06, .05]),
        attachment_points_m=attachments, link_length_m=.85,
        position_gain=np.array([3., 3., 3.75]), velocity_gain=np.array([3.12, 3.12, 3.12]),
        integral_gain=np.array([1.6, 1.6, 1.6]), integral_limit=np.array([.5, .5, .5]),
        c1=.5, force_norm_epsilon=1.e-9, tension_pinv_tolerance=1.e-9,
        link_kq=40., link_komega=20., link_integral_gain=0.,
        link_integral_limit=np.array([.3, .3, .3]), outward_bias_fraction=.25))
    matrix = controller._allocation_matrix()
    allocation = matrix.T@np.linalg.inv(matrix@matrix.T)
    bias = controller._internal_bias(matrix, .023, 9.80665)
    for now in (20., 21.7, 34.5, 40., 48., 57.3, 60.):
        sample = _reference().evaluate(now)
        actual = _module().equilibrium_cable_reference(sample["rotation"], sample["body_rate"],
            sample["body_rate_dot"], allocation, bias, gravity)
        np.testing.assert_allclose(actual["directions"],
            controller.equilibrium_link_units(sample["rotation"]), atol=1.e-14)
