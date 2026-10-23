"""Exact circular figure-eight lobes and their analytic motion derivatives."""

import importlib

import numpy as np
import pytest


def _module():
    return importlib.import_module("slung_circle_reference")


def _figure_eight(**options):
    module = _module()
    config = module.CircleTrajectoryConfig.from_mapping(
        dict(mode="figure_eight", **options))
    return module.PayloadFigureEightReference([0., 0., .6], 10., config)


def test_figure_eight_mode_accepts_integer_cycles_and_default_crossing():
    reference = _figure_eight()
    np.testing.assert_array_equal(reference.entry_position, [0., 0., .6])
    np.testing.assert_array_equal(reference.center_position, [0., 0., .6])
    np.testing.assert_array_equal(reference.lobe_centers, [[.5, 0., .6], [-.5, 0., .6]])
    assert reference.duration == pytest.approx(72.83185307179586)
    assert reference.end_time == pytest.approx(82.83185307179586)


@pytest.mark.parametrize("cycles", [.01, .5, 1.5, 2.25])
def test_figure_eight_rejects_partial_cycles(cycles):
    with pytest.raises(ValueError):
        _module().CircleTrajectoryConfig(mode="figure_eight", revolutions=cycles)


def test_explicit_crossing_is_copied_with_hover_height():
    module = _module()
    center = [2., -3.]
    config = module.CircleTrajectoryConfig.from_mapping(
        dict(mode="figure_eight", center_xy=center))
    center[:] = [99., 99.]
    reference = module.PayloadFigureEightReference([8., 9., .6], 10., config)
    np.testing.assert_array_equal(reference.entry_position, [2., -3., .6])
    np.testing.assert_array_equal(reference.lobe_centers, [[2.5, -3., .6], [1.5, -3., .6]])


@pytest.mark.parametrize("direction,sign", [("ccw", 1.), ("cw", -1.)])
def test_each_lobe_is_circular_with_body_x_aligned_to_motion(direction, sign):
    reference = _figure_eight(direction=direction)
    half_duration = reference.duration/2.
    for index in range(2):
        previous_radius = None
        for elapsed in np.linspace(.01, half_duration-.01, 301):
            sample = reference.evaluate(10.+index*half_duration+elapsed)
            radial = sample["position"]-reference.lobe_centers[index]
            assert np.linalg.norm(radial[:2]) == pytest.approx(.5, abs=1e-14)
            assert sample["position"][2] == .6
            assert sample["velocity"][2] == sample["acceleration"][2] == 0.
            assert sample["phase"] == "figure_eight"
            assert sample["lobe_index"] == index
            speed = np.linalg.norm(sample["velocity"])
            assert speed > 0.
            np.testing.assert_allclose(sample["rotation"][:, 0], sample["velocity"]/speed,
                                       atol=2e-14)
            np.testing.assert_allclose(sample["rotation"].T@sample["rotation"], np.eye(3), atol=1e-14)
            assert np.linalg.det(sample["rotation"]) == pytest.approx(1.)
            if previous_radius is not None:
                cross_z = previous_radius[0]*radial[1]-previous_radius[1]*radial[0]
                assert sign*(1. if index == 0 else -1.)*cross_z > 0.
            previous_radius = radial


@pytest.mark.parametrize("direction,initial", [("ccw", -np.pi/2.), ("cw", np.pi/2.)])
def test_stopped_crossings_keep_the_continuous_tangent_heading(direction, initial):
    reference = _figure_eight(direction=direction)
    assert reference.initial_yaw == pytest.approx(initial)
    np.testing.assert_allclose(reference.initial_rotation[:, 0], [0., np.sign(initial), 0.], atol=1e-14)
    times = [0., 10., 10.+reference.duration/2., reference.end_time, reference.end_time+1.]
    samples = [reference.evaluate(now) for now in times]
    assert [sample["phase"] for sample in samples] == ["waiting", "figure_eight", "figure_eight",
                                                      "complete", "complete"]
    assert [sample["progress"] for sample in samples] == [0., 0., .5, 1., 1.]
    for sample in samples:
        np.testing.assert_allclose(sample["position"], [0., 0., .6], atol=1e-14)
        np.testing.assert_array_equal(sample["velocity"], np.zeros(3))
        np.testing.assert_array_equal(sample["acceleration"], np.zeros(3))
        assert sample["yaw_rate"] == sample["yaw_acceleration"] == 0.
        np.testing.assert_allclose(sample["rotation"], reference.initial_rotation, atol=1e-14)
    assert samples[2]["yaw"] == pytest.approx(-3.*initial)
    assert samples[-1]["yaw"] == pytest.approx(initial)


@pytest.mark.parametrize("cycles", [1., 2., 3.])
def test_multiple_cycles_finish_at_crossing_and_respect_peak_speed(cycles):
    reference = _figure_eight(revolutions=cycles)
    samples = [reference.evaluate(now) for now in np.linspace(10., reference.end_time, 2001)]
    speeds = np.array([np.linalg.norm(sample["velocity"]) for sample in samples])
    assert np.max(speeds) == pytest.approx(.10)
    assert np.max(speeds) <= .10+1e-14
    assert reference.duration == pytest.approx(72.83185307179586*cycles)
    progress = [sample["progress"] for sample in samples]
    assert np.all(np.diff(progress) >= 0.)
    np.testing.assert_allclose(samples[-1]["position"], reference.entry_position, atol=1e-14)
    for cycle in range(1, int(cycles)):
        sample = reference.evaluate(10.+cycle*72.83185307179586)
        np.testing.assert_allclose(sample["position"], reference.entry_position, atol=1e-14)
        assert sample["yaw"] == pytest.approx(reference.initial_yaw)
        assert sample["progress"] == pytest.approx(cycle/cycles)


@pytest.mark.parametrize("direction", ["ccw", "cw"])
@pytest.mark.parametrize("elapsed", [0., 1.25, 5., 12., 31.41592653589793,
                                      36.41592653589793, 40., 41.41592653589793,
                                      67.83185307179586, 72.83185307179586])
def test_pva_and_heading_rates_match_derivatives_at_ramps_and_lobe_seams(direction, elapsed):
    reference = _figure_eight(direction=direction)
    now, step = 10.+elapsed, 1.e-4
    left, middle, right = [reference.evaluate(t) for t in (now-step, now, now+step)]
    np.testing.assert_allclose((right["position"]-left["position"])/(2.*step),
                               middle["velocity"], atol=2e-9)
    np.testing.assert_allclose((right["velocity"]-left["velocity"])/(2.*step),
                               middle["acceleration"], atol=2e-9)
    assert (right["yaw"]-left["yaw"])/(2.*step) == pytest.approx(middle["yaw_rate"], abs=1e-9)
    assert (right["yaw_rate"]-left["yaw_rate"])/(2.*step) == pytest.approx(
        middle["yaw_acceleration"], abs=1e-9)
    omega = np.array([[0., -middle["yaw_rate"], 0.],
                      [middle["yaw_rate"], 0., 0.], [0., 0., 0.]])
    np.testing.assert_allclose((right["rotation"]-left["rotation"])/(2.*step),
                               middle["rotation"]@omega, atol=2e-9)


@pytest.mark.parametrize("argument,value", [("radius_m", 0.), ("speed_mps", np.nan),
                                             ("ramp_s", np.inf), ("settle_s", -1.),
                                             ("center_xy", [0., np.nan]),
                                             ("revolutions", True)])
def test_figure_eight_keeps_finite_configuration_validation(argument, value):
    with pytest.raises(ValueError):
        _figure_eight(**{argument: value})


@pytest.mark.parametrize("position,start", [([1., 2.], 0.), ([1., np.nan, .6], 0.),
                                            ([1., 2., .6], np.inf)])
def test_figure_eight_rejects_invalid_initial_state(position, start):
    module = _module()
    config = module.CircleTrajectoryConfig(mode="figure_eight")
    with pytest.raises(ValueError):
        module.PayloadFigureEightReference(position, start, config)


def test_figure_eight_outputs_do_not_alias_state_and_invalid_times_are_rejected():
    reference = _figure_eight()
    with pytest.raises(ValueError):
        reference.evaluate(np.nan)
    for value in (reference.entry_position, reference.center_position, reference.lobe_centers,
                  reference.initial_rotation, reference.evaluate(10.)["position"]):
        value[:] = 99.
    np.testing.assert_array_equal(reference.entry_position, [0., 0., .6])
    np.testing.assert_array_equal(reference.lobe_centers, [[.5, 0., .6], [-.5, 0., .6]])
    np.testing.assert_allclose(reference.evaluate(10.)["rotation"], reference.initial_rotation)


def test_complete_endpoint_clamps_derivatives_exactly_with_large_absolute_times():
    module = _module()
    reference = module.PayloadFigureEightReference(
        [0., 0., .6], 1.e9, module.CircleTrajectoryConfig(mode="figure_eight"))
    for now in (reference.end_time, reference.end_time+100.):
        sample = reference.evaluate(now)
        np.testing.assert_array_equal(sample["velocity"], np.zeros(3))
        np.testing.assert_array_equal(sample["acceleration"], np.zeros(3))
        assert sample["yaw_rate"] == sample["yaw_acceleration"] == 0.
        assert sample["yaw"] == reference.initial_yaw
        assert sample["progress"] == 1.


def test_high_requested_speed_shortens_each_lobe_to_two_complete_ramps():
    reference = _figure_eight(speed_mps=100.)
    assert reference.duration == pytest.approx(20.)
    samples = [reference.evaluate(now) for now in np.linspace(10., reference.end_time, 1001)]
    assert max(np.linalg.norm(sample["velocity"]) for sample in samples) == pytest.approx(np.pi/5.)
    for now in (10., 20., 30.):
        sample = reference.evaluate(now)
        np.testing.assert_allclose(sample["position"], reference.entry_position, atol=1e-14)
        np.testing.assert_array_equal(sample["velocity"], np.zeros(3))
        np.testing.assert_array_equal(sample["acceleration"], np.zeros(3))


def test_circle_keeps_fractional_revolutions_and_its_original_initial_heading():
    module = _module()
    config = module.CircleTrajectoryConfig(mode="circle", revolutions=.25)
    reference = module.PayloadCircleReference([0., 0., .6], 10., config)
    assert reference.initial_yaw == 0.
    np.testing.assert_array_equal(reference.initial_rotation, np.eye(3))
    final = reference.evaluate(reference.end_time)
    np.testing.assert_allclose(final["position"], [.5, -.5, .6], atol=1e-14)
    assert final["yaw"] == pytest.approx(np.pi/2.)
