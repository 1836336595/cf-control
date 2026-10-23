"""Payload circle geometry and formation derivatives without ROS."""

import importlib

import numpy as np
import pytest


def test_missing_trajectory_config_preserves_legacy_hover():
    module = importlib.import_module("slung_circle_reference")
    config = module.CircleTrajectoryConfig.from_mapping({})
    assert config.mode == "hover"


def test_config_copies_explicit_center_before_caller_mutates_it():
    module = importlib.import_module("slung_circle_reference")
    center = [1., 2.]
    config = module.CircleTrajectoryConfig.from_mapping(
        {"mode": "circle", "center_xy": center, "direction": "cw"})
    center[0] = 9.
    np.testing.assert_array_equal(config.center_xy, [1., 2.])
    assert config.direction == "cw"


@pytest.mark.parametrize("mapping", [None, [], "circle", 2., True])
def test_non_mapping_trajectory_config_is_rejected(mapping):
    module = importlib.import_module("slung_circle_reference")
    with pytest.raises(ValueError):
        module.CircleTrajectoryConfig.from_mapping(mapping)


@pytest.mark.parametrize("name,value", [
    ("mode", "orbit"), ("mode", None), ("direction", "left"),
    ("direction", True), ("radius_m", 0.), ("radius_m", -.5),
    ("radius_m", np.nan), ("radius_m", np.inf), ("radius_m", True),
    ("radius_m", ".5"), ("speed_mps", 0.), ("speed_mps", -1.),
    ("speed_mps", np.nan), ("revolutions", 0.), ("revolutions", -1.),
    ("revolutions", np.inf), ("revolutions", False), ("ramp_s", 0.),
    ("ramp_s", -1.), ("ramp_s", np.nan), ("settle_s", -1.),
    ("settle_s", np.nan), ("settle_s", True), ("center_xy", [1.]),
    ("center_xy", [1., 2., 3.]), ("center_xy", [[1., 2.]]),
    ("center_xy", [np.nan, 0.]), ("center_xy", [False, 0.]),
    ("center_xy", ["1", 0.]), ("center_xy", np.array([1j, 0.])),
    ("unknown_option", 1.),
])
def test_invalid_fields_are_rejected_even_with_hover_mode(name, value):
    module = importlib.import_module("slung_circle_reference")
    with pytest.raises(ValueError):
        module.CircleTrajectoryConfig.from_mapping({"mode": "hover", name: value})


def test_direct_config_construction_cannot_bypass_validation():
    module = importlib.import_module("slung_circle_reference")
    with pytest.raises(ValueError):
        module.CircleTrajectoryConfig(radius_m=-.5)


def _circle(**options):
    module = importlib.import_module("slung_circle_reference")
    config = module.CircleTrajectoryConfig.from_mapping(dict(mode="circle", **options))
    return module.PayloadCircleReference(np.array([2., -1., .6]), 10., config)


def test_default_circle_enters_at_hover_and_finishes_before_validation_deadline():
    circle = _circle()
    np.testing.assert_allclose(circle.entry_position, [2., -1., .6])
    np.testing.assert_allclose(circle.center_position, [2.5, -1., .6])
    assert circle.duration == pytest.approx(36.41592653589793)
    assert circle.end_time == pytest.approx(46.41592653589793)


def test_explicit_center_sets_the_left_entry_point_at_hover_height():
    circle = _circle(center_xy=[1., 3.])
    np.testing.assert_allclose(circle.entry_position, [.5, 3., .6])
    np.testing.assert_allclose(circle.evaluate(0.)["position"], [.5, 3., .6])


@pytest.mark.parametrize("direction,sign", [("ccw", 1.), ("cw", -1.)])
def test_circle_stays_at_fixed_height_with_body_x_pointing_inward(direction, sign):
    circle = _circle(direction=direction)
    previous_radius = None
    for now in np.linspace(10., circle.end_time, 201):
        sample = circle.evaluate(now)
        radial = sample["position"] - circle.center_position
        assert np.linalg.norm(radial[:2]) == pytest.approx(.5, abs=1e-14)
        assert sample["position"][2] == .6
        assert sample["velocity"][2] == sample["acceleration"][2] == 0.
        np.testing.assert_allclose(sample["rotation"][:, 0], -radial / .5, atol=1e-14)
        np.testing.assert_allclose(sample["rotation"].T @ sample["rotation"], np.eye(3), atol=1e-14)
        assert np.linalg.det(sample["rotation"]) == pytest.approx(1.)
        if previous_radius is not None:
            cross_z = previous_radius[0]*radial[1] - previous_radius[1]*radial[0]
            assert sign*cross_z > 0.
        previous_radius = radial


@pytest.mark.parametrize("revolutions", [1., .01, 2.25])
def test_peak_linear_speed_is_bounded_for_full_and_short_arcs(revolutions):
    circle = _circle(revolutions=revolutions)
    samples = [circle.evaluate(t) for t in np.linspace(10., circle.end_time, 1001)]
    speeds = np.array([np.linalg.norm(item["velocity"]) for item in samples])
    assert np.max(speeds) <= .10 + 1e-14
    if revolutions == .01:
        assert circle.duration == pytest.approx(10.)
        assert np.max(speeds) == pytest.approx(.006283185307179586)
    else:
        assert np.max(speeds) == pytest.approx(.10)
    assert samples[-1]["yaw"] == pytest.approx(2*np.pi*revolutions)


@pytest.mark.parametrize("elapsed", [0., 1.25, 5., 12., 31.41592653589793,
                                      35., 36.41592653589793])
def test_circle_pva_and_rotation_rates_match_finite_differences(elapsed):
    circle = _circle()
    now, step = 10. + elapsed, 1.e-4
    left, middle, right = [circle.evaluate(t) for t in (now-step, now, now+step)]
    np.testing.assert_allclose((right["position"] - left["position"])/(2*step),
                               middle["velocity"], atol=2e-9)
    np.testing.assert_allclose((right["velocity"] - left["velocity"])/(2*step),
                               middle["acceleration"], atol=2e-9)
    assert (right["yaw"]-left["yaw"])/(2*step) == pytest.approx(
        middle["yaw_rate"], abs=1e-9)
    assert (right["yaw_rate"]-left["yaw_rate"])/(2*step) == pytest.approx(
        middle["yaw_acceleration"], abs=1e-9)
    omega = np.array([[0., -middle["yaw_rate"], 0.],
                      [middle["yaw_rate"], 0., 0.], [0., 0., 0.]])
    np.testing.assert_allclose((right["rotation"]-left["rotation"])/(2*step),
                               middle["rotation"] @ omega, atol=2e-9)


def test_waiting_and_complete_samples_are_stationary_with_clamped_progress():
    circle = _circle(revolutions=.25)
    samples = [circle.evaluate(t) for t in (0., 10., circle.end_time, circle.end_time+10.)]
    assert [item["phase"] for item in samples] == ["waiting", "circle", "complete", "complete"]
    assert [item["progress"] for item in samples] == [0., 0., 1., 1.]
    np.testing.assert_allclose(samples[-1]["position"], [2.5, -1.5, .6], atol=1e-14)
    for item in samples:
        np.testing.assert_array_equal(item["velocity"], np.zeros(3))
        np.testing.assert_array_equal(item["acceleration"], np.zeros(3))
        assert item["yaw_rate"] == item["yaw_acceleration"] == 0.


@pytest.mark.parametrize("hover,start", [([1., 2.], 0.), ([0., np.nan, 0.], 0.),
                                         ([0., 0., .6], np.nan), ([0., 0., .6], True)])
def test_invalid_circle_initial_state_is_rejected(hover, start):
    module = importlib.import_module("slung_circle_reference")
    with pytest.raises(ValueError):
        module.PayloadCircleReference(hover, start, module.CircleTrajectoryConfig(mode="circle"))


def test_nonfinite_evaluation_time_is_rejected_and_outputs_do_not_alias_state():
    circle = _circle()
    with pytest.raises(ValueError):
        circle.evaluate(np.nan)
    sample = circle.evaluate(10.)
    sample["position"][:] = 99.
    entry = circle.entry_position
    entry[:] = 99.
    center = circle.center_position
    center[:] = 99.
    np.testing.assert_allclose(circle.evaluate(10.)["position"], [2., -1., .6])


def test_rotating_reference_includes_tangential_centripetal_and_coriolis_terms():
    module = importlib.import_module("slung_circle_reference")
    result = module.rotate_vehicle_references(
        [[2., 2., 3.]], [[.1, 1.2, .3]], [[1.01, .02, .03]],
        [1., 2., 3.], [.1, .2, .3], [.01, .02, .03], np.pi/2., 2., 3.)
    np.testing.assert_allclose(result[0], [[1., 3., 3.]], atol=1e-14)
    np.testing.assert_allclose(result[1], [[-2.9, .2, .3]], atol=1e-14)
    np.testing.assert_allclose(result[2], [[-2.99, -6.98, .03]], atol=1e-14)


def _moving_formation(now):
    payload_position = np.array([1., 2., .6])+np.array([.2, -.1, .3])*now
    payload_velocity, payload_acceleration = np.array([.2, -.1, .3]), np.zeros(3)
    initial = np.array([[.2, .3, .8], [-.4, .2, .7], [.1, -.5, .9]])
    velocity = np.array([[.01, .02, .03], [-.02, .01, .02], [.03, -.01, .01]])
    acceleration = np.array([[.03, .01, -.02], [.01, -.02, .03], [-.01, .03, .02]])
    return (payload_position+initial+now*velocity+.5*now**2*acceleration,
            payload_velocity+velocity+now*acceleration,
            payload_acceleration+acceleration,
            payload_position, payload_velocity, payload_acceleration,
            .4+.3*now+.15*now**2, .3+.3*now, .3)


def test_rotating_formation_pva_matches_derivatives_about_a_moving_payload():
    module = importlib.import_module("slung_circle_reference")
    now, step = .8, 1.e-4
    before, middle, after = [module.rotate_vehicle_references(*_moving_formation(t))
                             for t in (now-step, now, now+step)]
    np.testing.assert_allclose((after[0]-before[0])/(2*step), middle[1], atol=3e-9)
    np.testing.assert_allclose((after[1]-before[1])/(2*step), middle[2], atol=3e-9)
    initial = _moving_formation(now)[0]
    for i, j in ((0, 1), (1, 2), (0, 2)):
        assert np.linalg.norm(middle[0][i]-middle[0][j]) == pytest.approx(
            np.linalg.norm(initial[i]-initial[j]), abs=1e-14)


def test_identity_rotation_preserves_inputs_and_returns_independent_arrays():
    module = importlib.import_module("slung_circle_reference")
    arguments = list(_moving_formation(.8))
    arguments[-3:] = [0., 0., 0.]
    copies = [item.copy() for item in arguments[:6]]
    result = module.rotate_vehicle_references(*arguments)
    for expected, actual in zip(copies, result):
        np.testing.assert_allclose(actual, expected, atol=1e-14)
        actual[:] = 99.
    for actual, expected in zip(arguments[:6], copies):
        np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize("index,value", [
    (0, [1., 2., 3.]), (0, []), (0, [[1., 2.]]),
    (1, np.zeros((2, 3))), (2, np.full((3, 3), np.nan)),
    (3, [1., 2.]), (4, [1., np.inf, 0.]), (5, [True, 0., 0.]),
    (6, np.nan), (7, np.inf), (8, True),
])
def test_malformed_rotating_formation_inputs_are_rejected(index, value):
    module = importlib.import_module("slung_circle_reference")
    arguments = list(_moving_formation(.8))
    arguments[index] = value
    with pytest.raises(ValueError):
        module.rotate_vehicle_references(*arguments)
