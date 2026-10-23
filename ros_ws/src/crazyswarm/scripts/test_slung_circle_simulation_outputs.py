"""Circle replay diagnostics must compare physical motion with its reference."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

import simulate_slung_handoff as replay
from slung_circle_reference import CircleTrajectoryConfig, PayloadCircleReference


def circle_manager():
    config = CircleTrajectoryConfig(mode="circle")
    reference = PayloadCircleReference([0., 0., .6], 20., config)
    return SimpleNamespace(payload_circle_config=config,
                           payload_circle_reference=reference,
                           payload_circle_state=reference.evaluate(30.))


def test_reference_trace_keeps_desired_pose_and_circle_geometry_separate():
    manager = circle_manager()
    target = dict(payload_target_position=np.array([.5, -.5, .6]),
                  payload_target_rotation=np.array([[0., -1., 0.],
                                                     [1., 0., 0.], [0., 0., 1.]]),
                  payload_trajectory_mode="circle", payload_trajectory_phase="circle",
                  payload_circle_progress=.25,
                  payload_circle_center=np.array([.5, 0., .6]),
                  payload_circle_radius_m=.5)
    fields = replay.payload_reference_trace_fields(manager, target)
    assert [fields["payload_target_position_" + axis] for axis in "xyz"] == [.5, -.5, .6]
    assert fields["payload_target_roll_rad"] == 0.
    assert fields["payload_target_pitch_rad"] == 0.
    assert fields["payload_target_yaw_rad"] == pytest.approx(math.pi / 2)
    assert fields["payload_target_height_m"] == .6
    assert fields["payload_circle_progress"] == .25
    assert fields["payload_trajectory_phase"] == "circle"
    assert [fields["payload_circle_center_" + axis] for axis in "xyz"] == [.5, 0., .6]
    assert fields["payload_circle_radius_m"] == .5
    assert "payload_position_x" not in fields


def test_unavailable_reference_pose_is_nan_instead_of_a_zero_target():
    manager = circle_manager()
    manager.payload_circle_reference = None
    manager.payload_circle_state = None
    fields = replay.payload_reference_trace_fields(manager, {})
    for name in ["payload_target_position_x", "payload_target_height_m",
                 "payload_target_roll_rad", "payload_target_yaw_rad",
                 "payload_circle_center_x", "payload_circle_progress"]:
        assert math.isnan(fields[name]), name
    assert fields["payload_trajectory_mode"] == "circle"
    assert fields["payload_trajectory_phase"] == "waiting"


def test_circle_trace_has_no_lobe_centers_even_if_shared_override_contains_placeholders():
    manager = circle_manager()
    fields = replay.payload_reference_trace_fields(
        manager, dict(payload_lobe_centers=np.zeros((2, 3))))
    for lobe in (1, 2):
        for axis in "xyz":
            assert math.isnan(fields["payload_lobe_center_{}_{}".format(lobe, axis)])


def trace_row(now, phase="circle", progress=.5, error=(0., 0., 0.),
              yaw=0., desired_yaw=0.):
    row = dict(control_time_s=now, vehicle_id=3, payload_trajectory_mode="circle",
               payload_trajectory_phase=phase, payload_circle_progress=progress,
               payload_yaw_rad=yaw, payload_target_yaw_rad=desired_yaw)
    row.update({"payload_position_" + axis: value
                for axis, value in zip("xyz", error)})
    row.update({"payload_target_position_" + axis: 0. for axis in "xyz"})
    return row


def test_circle_metrics_use_actual_tracking_error_once_per_control_tick():
    manager = circle_manager()
    first = trace_row(21., error=(.03, .04, 0.),
                      yaw=math.radians(-179), desired_yaw=math.radians(179))
    second = trace_row(22., error=(0., 0., .12),
                       yaw=math.radians(10), desired_yaw=math.radians(14))
    # The first sample has error .05 m, the second .12 m. Repeated aircraft
    # rows must not bias the payload metrics toward the first control tick.
    summary = replay.payload_circle_summary(manager, [first, dict(first, vehicle_id=4),
                                                      dict(first, vehicle_id=5), second])
    assert summary["payload_circle_tracking_sample_count"] == 2
    assert summary["payload_circle_position_rmse_m"] == pytest.approx(math.sqrt(.00845))
    assert summary["payload_circle_position_max_error_m"] == pytest.approx(.12)
    assert summary["payload_circle_yaw_rmse_deg"] == pytest.approx(math.sqrt(10))
    assert summary["payload_circle_yaw_max_error_deg"] == pytest.approx(4.)
    assert summary["payload_circle_center_m"] == [.5, 0., .6]
    assert summary["payload_circle_start_time_s"] == 20.
    assert summary["payload_circle_end_time_s"] == pytest.approx(56.41592653589793)


def test_reference_completion_does_not_imply_physical_tracking_accuracy():
    manager = circle_manager()
    # A stationary load can coexist with a completely generated reference.
    rows = [trace_row(30., progress=.25, error=(.5, .5, 0.),
                      desired_yaw=math.pi / 2),
            trace_row(57., phase="complete", progress=1.)]
    summary = replay.payload_circle_summary(manager, rows)
    assert summary["payload_circle_reference_completion_observed"]
    assert summary["payload_circle_position_rmse_m"] == pytest.approx(math.sqrt(.5))
    assert summary["payload_circle_yaw_rmse_deg"] == pytest.approx(90.)
    assert replay.circle_observation_failure(summary) is None


def test_incomplete_trace_fails_even_when_manager_reference_has_finished():
    manager = circle_manager()
    manager.payload_circle_state = manager.payload_circle_reference.evaluate(57.)
    summary = replay.payload_circle_summary(manager, [trace_row(30., progress=.25)])
    assert not summary["payload_circle_reference_completion_observed"]
    assert replay.circle_observation_failure(summary) == "circle_reference_incomplete"
    assert replay.circle_observation_failure(summary, "payload_dropped") == "payload_dropped"


def test_observed_reference_metadata_survives_manager_reset_after_landing():
    manager = circle_manager()
    observed_reference = manager.payload_circle_reference
    manager.payload_circle_reference = None
    manager.payload_circle_state = None
    summary = replay.payload_circle_summary(
        manager, [trace_row(57., phase="complete", progress=1.)],
        reference=observed_reference)
    assert summary["payload_circle_reference_completion_observed"]
    assert summary["payload_circle_start_time_s"] == 20.
    assert summary["payload_circle_end_time_s"] == pytest.approx(56.41592653589793)
    assert summary["payload_circle_center_m"] == [.5, 0., .6]


def test_post_landing_rows_do_not_turn_completed_circle_back_into_waiting():
    manager = circle_manager()
    complete = trace_row(57., phase="complete", progress=1.)
    landing = trace_row(60., phase="landing", progress=0.)
    cleared = trace_row(70., phase="waiting", progress=float("nan"))
    for axis in "xyz":
        cleared["payload_target_position_" + axis] = float("nan")
    cleared["payload_target_yaw_rad"] = float("nan")
    summary = replay.payload_circle_summary(manager, [complete, landing, cleared])
    assert summary["payload_circle_observed_final_phase"] == "complete"
    assert summary["payload_circle_observed_progress"] == 1.
    assert summary["payload_circle_reference_completion_observed"]


def test_no_circle_samples_reports_unknown_errors_and_does_not_fail_hover():
    manager = SimpleNamespace(payload_circle_config=CircleTrajectoryConfig())
    summary = replay.payload_circle_summary(manager, [])
    assert summary["payload_trajectory_mode"] == "hover"
    assert summary["payload_circle_tracking_sample_count"] == 0
    assert summary["payload_circle_position_rmse_m"] is None
    assert summary["payload_circle_yaw_rmse_deg"] is None
    assert replay.circle_observation_failure(summary) is None


def test_invalid_actual_circle_sample_is_reported_instead_of_dropped_silently():
    manager = circle_manager()
    row = trace_row(30., error=(float("nan"), 0., 0.))
    summary = replay.payload_circle_summary(manager, [row])
    assert summary["payload_circle_tracking_sample_count"] == 0
    assert summary["payload_circle_tracking_invalid_sample_count"] == 1
    assert summary["payload_circle_position_rmse_m"] is None


def figure_eight_manager():
    # The simulator consumes this reference protocol; its geometry is tested
    # independently by the real circle/figure-eight reference module.
    config = SimpleNamespace(mode="figure_eight", radius_m=.5, speed_mps=.1,
                             ramp_s=5., settle_s=3., direction="ccw", revolutions=1.)
    reference = SimpleNamespace(start_time=20., end_time=92.83185307179586,
                                duration=72.83185307179586,
                                center_position=np.array([0., 0., .6]),
                                entry_position=np.array([0., 0., .6]),
                                lobe_centers=np.array([[-.5, 0., .6], [.5, 0., .6]]))
    return SimpleNamespace(payload_circle_config=config,
                           payload_circle_reference=reference,
                           payload_circle_state=dict(phase="figure_eight", progress=.25))


def figure_eight_row(now, phase="figure_eight", progress=.25, **kwargs):
    row = trace_row(now, phase=phase, progress=progress, **kwargs)
    row.update(payload_trajectory_mode="figure_eight", payload_trajectory_progress=progress)
    return row


def test_figure_eight_trace_records_crossing_both_centers_and_whole_path_progress():
    manager = figure_eight_manager()
    target = dict(payload_target_position=np.array([-.5, -.5, .6]),
                  payload_target_rotation=np.eye(3), payload_trajectory_progress=.25)
    fields = replay.payload_reference_trace_fields(manager, target)
    assert fields["payload_trajectory_mode"] == "figure_eight"
    assert fields["payload_trajectory_phase"] == "figure_eight"
    assert fields["payload_trajectory_progress"] == .25
    assert fields["payload_circle_progress"] == .25
    assert [fields["payload_circle_center_" + axis] for axis in "xyz"] == [0., 0., .6]
    assert [fields["payload_lobe_center_1_" + axis] for axis in "xyz"] == [-.5, 0., .6]
    assert [fields["payload_lobe_center_2_" + axis] for axis in "xyz"] == [.5, 0., .6]


def test_unavailable_figure_eight_trace_does_not_invent_lobe_centers_or_started_phase():
    manager = figure_eight_manager()
    manager.payload_circle_reference = manager.payload_circle_state = None
    fields = replay.payload_reference_trace_fields(manager, {})
    assert fields["payload_trajectory_phase"] == "waiting"
    assert math.isnan(fields["payload_trajectory_progress"])
    assert math.isnan(fields["payload_lobe_center_1_x"])
    assert math.isnan(fields["payload_lobe_center_2_y"])


def test_figure_eight_metrics_measure_both_lobes_with_generic_and_legacy_keys():
    manager = figure_eight_manager()
    rows = [figure_eight_row(30., progress=.25, error=(.03, .04, 0.),
                             yaw=math.radians(-179), desired_yaw=math.radians(179)),
            figure_eight_row(60., progress=.75, error=(0., 0., .12),
                             yaw=math.radians(10), desired_yaw=math.radians(14))]
    summary = replay.payload_circle_summary(manager, rows)
    assert summary["payload_trajectory_tracking_sample_count"] == 2
    assert summary["payload_trajectory_position_rmse_m"] == pytest.approx(math.sqrt(.00845))
    assert summary["payload_trajectory_position_max_error_m"] == .12
    assert summary["payload_trajectory_yaw_rmse_deg"] == pytest.approx(math.sqrt(10))
    assert summary["payload_trajectory_yaw_max_error_deg"] == pytest.approx(4.)
    assert summary["payload_circle_position_rmse_m"] == summary["payload_trajectory_position_rmse_m"]
    assert summary["payload_trajectory_center_m"] == [0., 0., .6]
    assert summary["payload_trajectory_lobe_centers_m"] == [[-.5, 0., .6], [.5, 0., .6]]
    assert summary["payload_trajectory_end_time_s"] == pytest.approx(92.83185307179586)
    assert summary["payload_trajectory_observed_progress"] == .75
    assert not summary["payload_trajectory_reference_completion_observed"]
    assert replay.circle_observation_failure(summary) == "figure_eight_reference_incomplete"
    assert replay.circle_observation_failure(summary, "payload_dropped") == "payload_dropped"


def test_figure_eight_completion_is_observed_across_landing_reset():
    manager = figure_eight_manager()
    observed = manager.payload_circle_reference
    manager.payload_circle_reference = manager.payload_circle_state = None
    cleared = figure_eight_row(110., phase="waiting", progress=float("nan"))
    for axis in "xyz":
        cleared["payload_target_position_" + axis] = float("nan")
    summary = replay.payload_circle_summary(manager, [
        figure_eight_row(93., phase="complete", progress=1.),
        figure_eight_row(100., phase="landing", progress=0.), cleared], reference=observed)
    assert summary["payload_trajectory_reference_completion_observed"]
    assert summary["payload_trajectory_observed_final_phase"] == "complete"
    assert summary["payload_trajectory_observed_progress"] == 1.
    assert summary["payload_trajectory_duration_s"] == pytest.approx(72.83185307179586)
    assert replay.circle_observation_failure(summary) is None


def test_figure_eight_cannot_complete_after_only_one_lobe():
    manager = figure_eight_manager()
    summary = replay.payload_circle_summary(
        manager, [figure_eight_row(57., phase="complete", progress=.5)])
    assert not summary["payload_trajectory_reference_completion_observed"]
    assert replay.circle_observation_failure(summary) == "figure_eight_reference_incomplete"


def cone_manager():
    config = SimpleNamespace(mode="attitude_cone", cone_half_angle_deg=10.,
                             cone_period_s=30., cone_transition_s=5., revolutions=1.)
    reference = SimpleNamespace(start_time=20., end_time=60., duration=40.,
                                center_position=np.array([0., 0., .6]),
                                entry_position=np.array([0., 0., .6]))
    return SimpleNamespace(payload_circle_config=config,
                           payload_circle_reference=reference,
                           payload_circle_state=dict(phase="attitude_cone", progress=.5))


def y_rotation(degrees):
    angle = math.radians(degrees)
    return np.array([[math.cos(angle), 0., math.sin(angle)], [0., 1., 0.],
                     [-math.sin(angle), 0., math.cos(angle)]])


def cone_trace_row(manager, now, phase="attitude_cone", progress=.5,
                   actual_rotation=None, target_rotation=None, error=(0., 0., 0.)):
    actual_rotation = np.eye(3) if actual_rotation is None else actual_rotation
    target_rotation = y_rotation(10.) if target_rotation is None else target_rotation
    fields = replay.payload_reference_trace_fields(manager, dict(
        payload_target_position=np.array([0., 0., .6]),
        payload_target_rotation=target_rotation,
        payload_trajectory_mode="attitude_cone", payload_trajectory_phase=phase,
        payload_trajectory_progress=progress), payload_rotation=actual_rotation)
    row = dict(trace_row(now, phase=phase, progress=progress, error=error,
                          yaw=math.atan2(actual_rotation[1, 0], actual_rotation[0, 0])),
               **fields)
    row["payload_position_z"] += .6
    return row


def test_cone_trace_records_world_body_z_axes_and_physical_axis_error():
    manager = cone_manager()
    fields = replay.payload_reference_trace_fields(manager, dict(
        payload_target_position=np.array([0., 0., .6]),
        payload_target_rotation=y_rotation(10.)), payload_rotation=y_rotation(-10.))
    np.testing.assert_allclose([fields["payload_body_z_" + axis] for axis in "xyz"],
                               [-math.sin(math.radians(10.)), 0., math.cos(math.radians(10.))])
    np.testing.assert_allclose([fields["payload_target_body_z_" + axis] for axis in "xyz"],
                               [math.sin(math.radians(10.)), 0., math.cos(math.radians(10.))])
    assert fields["payload_cone_half_angle_deg"] == pytest.approx(10.)
    assert fields["payload_target_cone_half_angle_deg"] == pytest.approx(10.)
    assert fields["payload_cone_axis_error_deg"] == pytest.approx(20.)
    assert fields["payload_cone_half_angle_error_deg"] == pytest.approx(0.)
    assert fields["payload_attitude_error_deg"] == pytest.approx(20.)
    assert fields["payload_cone_config_half_angle_deg"] == 10.
    assert fields["payload_cone_period_s"] == 30.
    assert math.isnan(fields["payload_lobe_center_1_x"])


def test_cone_half_angle_error_follows_the_ramped_target_during_entry():
    manager = cone_manager()
    fields = replay.payload_reference_trace_fields(manager, dict(
        payload_target_rotation=y_rotation(5.), payload_target_position=np.array([0., 0., .6]),
        payload_trajectory_phase="cone_tilt_in"), payload_rotation=y_rotation(3.))
    assert fields["payload_cone_half_angle_deg"] == pytest.approx(3.)
    assert fields["payload_target_cone_half_angle_deg"] == pytest.approx(5.)
    assert fields["payload_cone_half_angle_error_deg"] == pytest.approx(-2.)


def test_cone_no_reference_keeps_actual_axis_but_does_not_invent_desired_attitude():
    manager = cone_manager()
    manager.payload_circle_reference = manager.payload_circle_state = None
    fields = replay.payload_reference_trace_fields(manager, {}, payload_rotation=np.eye(3))
    assert fields["payload_trajectory_phase"] == "waiting"
    assert fields["payload_body_z_z"] == 1.
    assert math.isnan(fields["payload_target_body_z_z"])
    assert math.isnan(fields["payload_target_cone_half_angle_deg"])
    assert math.isnan(fields["payload_cone_axis_error_deg"])
    assert math.isnan(fields["payload_attitude_error_deg"])


def test_cone_full_attitude_error_detects_twist_with_the_same_body_z_axis():
    manager = cone_manager()
    desired = y_rotation(10.)
    quarter_turn = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    fields = replay.payload_reference_trace_fields(manager, dict(
        payload_target_rotation=desired, payload_target_position=np.array([0., 0., .6])),
        payload_rotation=desired @ quarter_turn)
    assert fields["payload_cone_axis_error_deg"] == pytest.approx(0., abs=2e-6)
    assert fields["payload_attitude_error_deg"] == pytest.approx(90.)


def test_cone_summary_measures_constant_phase_and_excludes_tilt_transitions():
    manager = cone_manager()
    rows = [cone_trace_row(manager, 22., phase="cone_tilt_in", actual_rotation=y_rotation(80.)),
            cone_trace_row(manager, 30., actual_rotation=y_rotation(7.), error=(.03, .04, 0.)),
            cone_trace_row(manager, 40., actual_rotation=y_rotation(6.), error=(0., 0., .12)),
            cone_trace_row(manager, 58., phase="cone_tilt_out", actual_rotation=y_rotation(80.))]
    summary = replay.payload_circle_summary(manager, rows)
    assert summary["payload_trajectory_tracking_sample_count"] == 2
    assert summary["payload_trajectory_position_rmse_m"] == pytest.approx(math.sqrt(.00845))
    assert summary["payload_trajectory_attitude_rmse_deg"] == pytest.approx(math.sqrt(12.5))
    assert summary["payload_trajectory_attitude_max_error_deg"] == pytest.approx(4.)
    assert summary["payload_cone_body_z_rmse_deg"] == pytest.approx(math.sqrt(12.5))
    assert summary["payload_cone_body_z_max_error_deg"] == pytest.approx(4.)
    assert summary["payload_cone_half_angle_rmse_deg"] == pytest.approx(math.sqrt(12.5))
    assert summary["payload_cone_half_angle_max_error_deg"] == pytest.approx(4.)
    assert summary["payload_cone_config_half_angle_deg"] == 10.
    assert summary["payload_cone_period_s"] == 30.
    assert summary["payload_cone_transition_s"] == 5.


@pytest.mark.parametrize("phase,progress", [("cone_tilt_in", .1), ("attitude_cone", .5),
                                           ("cone_tilt_out", .95)])
def test_cone_reference_is_incomplete_until_after_tilt_out(phase, progress):
    manager = cone_manager()
    summary = replay.payload_circle_summary(manager, [cone_trace_row(manager, 60., phase, progress)])
    assert not summary["payload_trajectory_reference_completion_observed"]
    assert summary["payload_trajectory_observed_final_phase"] == phase
    assert replay.circle_observation_failure(summary) == "attitude_cone_reference_incomplete"


def test_cone_observed_completion_survives_manager_reset_without_implying_tracking():
    manager = cone_manager()
    reference = manager.payload_circle_reference
    rows = [cone_trace_row(manager, 30.),
            cone_trace_row(manager, 60., phase="complete", progress=1.)]
    manager.payload_circle_reference = manager.payload_circle_state = None
    summary = replay.payload_circle_summary(manager, rows, reference=reference)
    assert summary["payload_trajectory_reference_completion_observed"]
    assert summary["payload_trajectory_observed_final_phase"] == "complete"
    assert summary["payload_trajectory_duration_s"] == 40.
    assert summary["payload_trajectory_attitude_rmse_deg"] == pytest.approx(10.)
    assert summary["payload_cone_body_z_rmse_deg"] == pytest.approx(10.)
    assert replay.circle_observation_failure(summary) is None
    assert replay.circle_observation_failure(summary, "payload_dropped") == "payload_dropped"
