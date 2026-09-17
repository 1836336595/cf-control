#!/usr/bin/env python3
"""Tests for the whole-circle quintic smoothstep reference.

These tests intentionally exercise the trajectory module without ROS.  The
behavioral contract is deliberately strict: one ``circle`` phase, zero angular
velocity/acceleration at both endpoints, and a peak-speed interpretation for
``circle_angular_speed_radps``.
"""

import math
import sys
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ctbr_trajectory import (  # noqa: E402
    CircularFlightTrajectory,
    CircularTrajectoryConfig,
    _smoothstep5,
)


def _config(**overrides):
    """Build a small deterministic config across old/new field layouts."""
    values = {
        "reference_hold_s": 0.05,
        "takeoff_height_m": 1.0,
        "takeoff_duration_s": 0.05,
        "takeoff_settle_s": 0.05,
        "takeoff_altitude_tolerance_m": 0.05,
        "takeoff_vertical_velocity_tolerance_mps": 0.05,
        "circle_center_offset_xy": np.array([1.0, 0.0]),
        "circle_radius_m": 1.0,
        "circle_revolutions": 1.0,
        "circle_angular_speed_radps": 1.0,
        # Retained for launch compatibility; it must not affect the profile.
        "circle_ramp_duration_s": 0.2,
        "circle_start_angle_rad": math.pi,
        "entry_duration_s": 0.05,
        "final_hover_s": 0.05,
        "landing_duration_s": 0.05,
        "landing_max_speed_mps": 20.0,
        "landing_altitude_tolerance_m": 0.05,
        "landing_vertical_velocity_tolerance_mps": 0.05,
        "landing_settle_s": 0.05,
        "landing_condition_grace_s": 0.05,
        "takeoff_max_tilt_rad": math.radians(5.0),
        "circle_max_tilt_rad": math.radians(10.0),
        "landing_max_tilt_rad": math.radians(5.0),
        "takeoff_min_collective_thrust": 0.0,
        "airborne_min_collective_thrust": 0.0,
    }
    values.update(overrides)
    return CircularTrajectoryConfig(**values)


def _circle_target(trajectory, elapsed):
    """Call the circle reference directly, independent of stage transitions."""
    for name in ("_circle_smoothstep_target", "_circle_target"):
        method = getattr(trajectory, name, None)
        if method is not None:
            return method(elapsed)
    raise AssertionError("trajectory has no direct circle target helper")


def _circle_duration(trajectory):
    duration = getattr(trajectory, "circle_duration_s", None)
    if duration is None:
        duration = getattr(trajectory.config, "circle_duration_s", None)
    if duration is None:
        raise AssertionError("whole-circle trajectory must expose circle_duration_s")
    return float(duration)


def _angle_samples(trajectory, duration, count=201):
    center = np.asarray(trajectory.circle_center[:2], dtype=float)
    values = []
    for elapsed in np.linspace(0.0, duration, count):
        target = _circle_target(trajectory, float(elapsed))
        radial = np.asarray(target["position"][:2], dtype=float) - center
        values.append(math.atan2(float(radial[1]), float(radial[0])))
    return np.unwrap(np.asarray(values, dtype=float))


def _initialize_circle(trajectory):
    start = np.array([0.0, 0.0, 0.2])
    trajectory.reset(start, start_yaw=0.0, now=0.0)
    # Direct helper tests do not need to run the takeoff gates.
    trajectory.phase = "circle"
    trajectory.phase_start_time = 0.0
    return start


def test_circle_smoothstep_has_quintic_endpoint_derivatives_and_full_angle():
    trajectory = CircularFlightTrajectory(_config())
    _initialize_circle(trajectory)
    duration = _circle_duration(trajectory)
    total_angle = 2.0 * math.pi * trajectory.config.circle_revolutions

    start = _circle_target(trajectory, 0.0)
    midpoint = _circle_target(trajectory, duration * 0.5)
    end = _circle_target(trajectory, duration)

    assert np.allclose(start["velocity"], 0.0, atol=1e-10)
    assert np.allclose(start["acceleration"], 0.0, atol=1e-10)
    assert np.allclose(end["velocity"], 0.0, atol=1e-10)
    assert np.allclose(end["acceleration"], 0.0, atol=1e-10)

    angles = _angle_samples(trajectory, duration)
    assert math.isclose(angles[-1] - angles[0], total_angle, rel_tol=1e-9, abs_tol=1e-9)
    # s(0.5) = 0.5 for the quintic smoothstep.
    assert math.isclose(
        angles[len(angles) // 2] - angles[0],
        total_angle * 0.5,
        rel_tol=1e-6,
        abs_tol=1e-6,
    )

    radius = float(trajectory.config.circle_radius_m)
    center = np.asarray(trajectory.circle_center[:2], dtype=float)
    for elapsed in np.linspace(0.0, duration, 31):
        target = _circle_target(trajectory, float(elapsed))
        assert math.isclose(
            np.linalg.norm(np.asarray(target["position"][:2]) - center),
            radius,
            rel_tol=1e-9,
            abs_tol=1e-9,
        )

    # The peak of s'(u) is 1.875 at u=0.5.  This test fixes the meaning of
    # circle_angular_speed_radps as the requested peak angular speed.
    peak_angular_speed = np.linalg.norm(midpoint["velocity"][:2]) / radius
    assert math.isclose(
        peak_angular_speed,
        float(trajectory.config.circle_angular_speed_radps),
        rel_tol=1e-6,
        abs_tol=1e-6,
    )


def test_legacy_ramp_duration_does_not_change_circle_duration():
    first = CircularFlightTrajectory(_config(circle_ramp_duration_s=0.1))
    second = CircularFlightTrajectory(_config(circle_ramp_duration_s=3.0))

    assert math.isclose(first.circle_duration_s, second.circle_duration_s, rel_tol=1e-12)


def test_circle_smoothstep_acceleration_changes_tangential_sign():
    trajectory = CircularFlightTrajectory(_config())
    _initialize_circle(trajectory)
    duration = _circle_duration(trajectory)
    center = np.asarray(trajectory.circle_center[:2], dtype=float)

    tangential_components = []
    for fraction in (0.25, 0.75):
        target = _circle_target(trajectory, duration * fraction)
        radial = np.asarray(target["position"][:2], dtype=float) - center
        radial /= np.linalg.norm(radial)
        tangent = np.array([-radial[1], radial[0]])
        tangential_components.append(
            float(np.dot(np.asarray(target["acceleration"][:2]), tangent))
        )

    assert tangential_components[0] > 0.0
    assert tangential_components[1] < 0.0


def test_stage_machine_has_single_circle_phase_and_no_ramp_phases():
    trajectory = CircularFlightTrajectory(_config())
    start = np.array([0.0, 0.0, 0.2])
    trajectory.reset(start, start_yaw=0.0, now=0.0)
    state = {"position": start.copy(), "velocity": np.zeros(3)}
    phases = []
    now = 0.0

    for _ in range(2000):
        if trajectory.phase == "takeoff":
            if now - trajectory.phase_start_time >= trajectory.config.takeoff_duration_s:
                state["position"] = trajectory.takeoff_position.copy()
                state["velocity"] = np.zeros(3)
        elif trajectory.phase in ("landing", "landing_settle"):
            state["position"] = trajectory.landing_target_position.copy()
            state["velocity"] = np.zeros(3)

        target = trajectory.evaluate(state, now)
        phase = target["flight_phase"]
        if not phases or phases[-1] != phase:
            phases.append(phase)
        if trajectory.phase == "landed":
            break
        now += 0.01

    assert trajectory.phase == "landed"
    assert "circle" in phases
    assert phases.count("circle") == 1
    assert not any("circle_ramp" in phase for phase in phases)
    assert phases.index("circle") < phases.index("final_hover")
    assert phases.index("final_hover") < phases.index("landing")


def test_height_correction_tracks_target_altitude_without_waiting_for_gate():
    """圆周前不等待高度门限，z 精确跟踪且水平参考保持名义起飞点。"""
    trajectory = CircularFlightTrajectory(_config(
        takeoff_altitude_tolerance_m=0.03,
        takeoff_vertical_velocity_tolerance_mps=0.05,
    ))
    start = np.array([0.0, 0.0, 0.2])
    trajectory.reset(start, start_yaw=0.0, now=0.0)
    state = {
        # 故意保持在旧的严格门限之外，验证飞行中由 z 位置误差继续校正。
        "position": np.array([0.12, -0.08, 0.75]),
        "velocity": np.array([0.0, 0.0, 0.20]),
    }
    takeoff_done = (
        trajectory.config.reference_hold_s
        + trajectory.config.takeoff_duration_s
        + 1e-6  # account for the recursive pre-takeoff -> takeoff transition
    )

    # Advance the phase machine through the pre-takeoff hold first.  A
    # recursive transition resets the takeoff clock at that exact sample.
    trajectory.evaluate(state, trajectory.config.reference_hold_s)
    target = trajectory.evaluate(state, takeoff_done)
    assert target["flight_phase"] == "height_correction"
    assert target["position"].shape == (3,)
    assert np.allclose(target["position"][:2], start[:2], atol=1e-12)
    assert np.allclose(
        trajectory.height_correction_position[:2], trajectory.takeoff_position[:2],
        atol=1e-12,
    )
    assert np.allclose(
        trajectory.circle_center[:2], start[:2] + np.array([1.0, 0.0]),
        atol=1e-12,
    )
    assert math.isclose(target["position"][2], start[2] + 1.0, abs_tol=1e-12)

    target = trajectory.evaluate(
        state, takeoff_done + trajectory.config.takeoff_settle_s + 1e-6
    )
    assert target["flight_phase"] == "circle_entry"
    assert np.allclose(target["position"][:2], trajectory.takeoff_position[:2], atol=1e-12)
    assert np.allclose(
        trajectory.circle_entry_start_position[:2], trajectory.takeoff_position[:2],
        atol=1e-12,
    )
    assert math.isclose(target["position"][2], start[2] + 1.0, abs_tol=1e-12)

    circle_target = _circle_target(trajectory, 0.5 * _circle_duration(trajectory))
    assert circle_target["position"].shape == (3,)
    assert math.isclose(circle_target["position"][2], start[2] + 1.0, abs_tol=1e-12)
    assert math.isclose(circle_target["velocity"][2], 0.0, abs_tol=1e-12)
    assert math.isclose(circle_target["acceleration"][2], 0.0, abs_tol=1e-12)


def test_emergency_landing_starts_from_current_position_and_never_releases_thrust_high():
    trajectory = CircularFlightTrajectory(_config(airborne_min_collective_thrust=0.3))
    start = np.array([0.0, 0.0, 0.2])
    trajectory.reset(start, start_yaw=0.3, now=0.0)
    current = np.array([0.4, -0.2, 1.1])
    trajectory.begin_emergency_landing(
        now=2.0, position=current, yaw=0.6, reason="test"
    )
    state = {"position": current.copy(), "velocity": np.zeros(3)}
    target = trajectory.evaluate(state, now=2.0)

    assert target["flight_phase"] == "emergency_landing"
    assert np.allclose(target["position"], current)
    assert math.isclose(target["min_collective_thrust"], 0.3)
    assert not target.get("zero_output", False)

    end_time = 2.0 + trajectory.active_landing_duration_s + 0.01
    target = trajectory.evaluate(state, now=end_time)
    assert not target.get("zero_output", False)
    state["position"][2] = trajectory.landing_target_position[2]
    target = trajectory.evaluate(state, now=end_time)
    assert target["zero_output"] is True


def test_airborne_minimum_thrust_remains_active_after_takeoff_overshoot():
    trajectory = CircularFlightTrajectory(_config(airborne_min_collective_thrust=0.3))
    start = np.array([0.0, 0.0, 0.2])
    trajectory.reset(start, start_yaw=0.0, now=0.0)
    high_state = {"position": np.array([0.0, 0.0, 2.0]), "velocity": np.zeros(3)}

    takeoff = trajectory._takeoff_target(0.02, high_state)
    entry = trajectory._entry_target(0.02)
    circle = trajectory._circle_target(0.02)
    landing = trajectory._landing_target(0.02)

    for target in (takeoff, entry, circle, landing):
        assert math.isclose(target["min_collective_thrust"], 0.3)


def test_circle_entry_and_final_hover_boundaries_are_position_continuous():
    trajectory = CircularFlightTrajectory(_config())
    _initialize_circle(trajectory)
    duration = _circle_duration(trajectory)

    entry = trajectory._entry_target(float(trajectory.config.entry_duration_s))
    circle_start = _circle_target(trajectory, 0.0)
    circle_end = _circle_target(trajectory, duration)

    assert np.allclose(entry["position"], circle_start["position"], atol=1e-9)
    assert np.allclose(entry["velocity"], circle_start["velocity"], atol=1e-9)
    assert np.allclose(entry["acceleration"], circle_start["acceleration"], atol=1e-9)

    final_hover = trajectory._static_target(
        circle_end["position"], circle_end["yaw"], "final_hover"
    )
    assert np.allclose(circle_end["position"], final_hover["position"], atol=1e-9)
    assert np.allclose(circle_end["velocity"], final_hover["velocity"], atol=1e-9)
    assert np.allclose(circle_end["acceleration"], final_hover["acceleration"], atol=1e-9)


def test_figure_eight_triangle_keeps_side_length_and_follows_velocity_heading():
    """三台机保持等边间距，机头沿 MATLAB formation 的水平速度方向。"""
    phases = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    trajectories = []
    for phase in phases:
        trajectory = CircularFlightTrajectory(_config(
            trajectory_mode="figure_eight_triangle",
            circle_center_xy=np.array([0.0, 0.0]),
            formation_side_length_m=0.5,
            figure_eight_radius_m=0.8,
            figure_eight_angular_speed_radps=1.0,
            orbit_phase_rad=phase,
        ))
        trajectory.reset(np.array([0.4, -0.3, 0.2]), start_yaw=0.0, now=0.0)
        trajectories.append(trajectory)

    lobe_duration = trajectories[0].figure_eight_lobe_duration_s
    for elapsed in np.linspace(0.0, 2.0 * lobe_duration, 13):
        targets = [trajectory._figure_eight_target(float(elapsed))
                   for trajectory in trajectories]
        positions = np.asarray([target["position"] for target in targets])
        centroid = np.mean(positions, axis=0)

        for first, second in ((0, 1), (1, 2), (2, 0)):
            assert math.isclose(
                np.linalg.norm(positions[first] - positions[second]), 0.5,
                rel_tol=1e-10, abs_tol=1e-10,
            )
        for target in targets:
            velocity = target["velocity"]
            acceleration = target["acceleration"]
            speed_squared = float(velocity[0] ** 2 + velocity[1] ** 2)
            if speed_squared <= 1.0e-12:
                expected_yaw = 0.0
                expected_yaw_rate = 0.0
            else:
                expected_yaw = math.atan2(velocity[1], velocity[0])
                expected_yaw_rate = (
                    velocity[0] * acceleration[1]
                    - velocity[1] * acceleration[0]
                ) / speed_squared
            assert math.isclose(
                math.atan2(
                    math.sin(target["yaw"] - expected_yaw),
                    math.cos(target["yaw"] - expected_yaw),
                ),
                0.0,
                abs_tol=1e-10,
            )
            assert math.isclose(
                target["yaw_rate"], expected_yaw_rate, abs_tol=1e-10
            )

        # 刚性平移：三机的平动速度、加速度和 jerk 必须一致。
        for key in ("velocity", "acceleration", "jerk"):
            assert np.allclose(targets[0][key], targets[1][key], atol=1e-12)
            assert np.allclose(targets[0][key], targets[2][key], atol=1e-12)


def test_figure_eight_crossing_is_continuous_without_stopping():
    trajectory = CircularFlightTrajectory(_config(
        trajectory_mode="figure_eight_triangle",
        circle_center_xy=np.array([0.0, 0.0]),
        formation_side_length_m=0.5,
        figure_eight_radius_m=0.8,
        figure_eight_angular_speed_radps=1.0,
    ))
    trajectory.reset(np.array([0.0, 0.0, 0.2]), start_yaw=0.0, now=0.0)
    lobe_duration = trajectory.figure_eight_lobe_duration_s
    offset = trajectory.formation_offset

    start = trajectory._figure_eight_target(0.0)
    crossing = trajectory._figure_eight_target(lobe_duration)
    end = trajectory._figure_eight_target(2.0 * lobe_duration)
    before_crossing = trajectory._figure_eight_target(lobe_duration - 1.0e-4)
    after_crossing = trajectory._figure_eight_target(lobe_duration + 1.0e-4)

    assert np.allclose(start["position"], offset + np.array([0.0, 0.0, 1.2]))
    assert np.allclose(crossing["position"], start["position"], atol=1e-10)
    assert np.allclose(end["position"], start["position"], atol=1e-10)
    # 原点交点不再像两个圆叶硬拼时那样停住。
    assert np.linalg.norm(crossing["velocity"][:2]) > 0.01
    assert np.allclose(
        before_crossing["velocity"], after_crossing["velocity"], atol=1e-4
    )
    assert np.allclose(
        before_crossing["acceleration"], after_crossing["acceleration"], atol=1e-4
    )
    # 仍覆盖左右两个叶瓣，且起点、终点平滑静止。
    samples = [trajectory._figure_eight_target(float(elapsed)) for elapsed in np.linspace(
        0.0, 2.0 * lobe_duration, 1001
    )]
    x_relative = [target["position"][0] - offset[0] for target in samples]
    assert max(x_relative) > 0.79
    assert min(x_relative) < -0.79
    for target in (start, end):
        assert np.allclose(target["velocity"], 0.0, atol=1e-10)
        assert np.allclose(target["acceleration"], 0.0, atol=1e-10)

    entry = trajectory._entry_target(float(trajectory.config.entry_duration_s))
    assert np.allclose(entry["position"], start["position"], atol=1e-10)
    assert np.allclose(entry["velocity"], start["velocity"], atol=1e-10)
    assert np.allclose(entry["acceleration"], start["acceleration"], atol=1e-10)
    # The first Gerono reference velocity is +x, so the entry transition must
    # end at the same heading used by the MATLAB reference.
    assert math.isclose(entry["yaw"], 0.0, abs_tol=1e-10)
    assert math.isclose(start["yaw"], 0.0, abs_tol=1e-10)


def test_smoothstep_helper_still_returns_zero_endpoint_derivatives():
    position0, velocity0, acceleration0 = _smoothstep5(0.0, 2.0)
    position1, velocity1, acceleration1 = _smoothstep5(2.0, 2.0)
    assert (position0, velocity0, acceleration0) == (0.0, 0.0, 0.0)
    assert math.isclose(position1, 1.0)
    assert math.isclose(velocity1, 0.0, abs_tol=1e-12)
    assert math.isclose(acceleration1, 0.0, abs_tol=1e-12)


def _figure_eight_trajectory(phase, heading):
    trajectory = CircularFlightTrajectory(_config(
        trajectory_mode="figure_eight_triangle",
        circle_center_xy=np.array([0.0, 0.0]),
        figure_eight_heading=heading,
        orbit_phase_rad=phase,
    ))
    trajectory.reset(np.array([0.4, -0.3, 0.2]), start_yaw=0.0, now=0.0)
    return trajectory


def test_center_heading_is_constant_and_has_zero_yaw_rate():
    """A center-pointing nose must not rotate, and must be free of grazing."""
    phases = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    fleet = [_figure_eight_trajectory(p, "center") for p in phases]
    duration = float(fleet[0].main_duration_s)

    for elapsed in np.linspace(0.0, duration, 41):
        targets = [t._figure_eight_target(float(elapsed)) for t in fleet]
        # The three vehicles keep a rigid triangle, so their true centroid is
        # recoverable from the references themselves.
        centroid = np.mean([np.asarray(t["position"]) for t in targets], axis=0)

        for trajectory, target in zip(fleet, targets):
            heading = np.array([
                math.cos(target["yaw"]), math.sin(target["yaw"]), 0.0,
            ])
            to_center = centroid - np.asarray(target["position"])
            to_center[2] = 0.0
            assert np.linalg.norm(to_center) > 1e-6
            # Body-x must be antiparallel to the offset, i.e. point at the center.
            assert float(heading @ to_center) > 0.0
            assert float(np.linalg.norm(np.cross(heading, to_center))) < 1e-9
            assert math.isclose(target["yaw_rate"], 0.0, abs_tol=1e-15)

        # Constant heading: compare each vehicle against its own first sample.
        for trajectory, target in zip(fleet, targets):
            first = trajectory._figure_eight_target(0.0)["yaw"]
            assert math.isclose(target["yaw"], first, abs_tol=1e-12)


def test_center_heading_points_inward_for_every_vehicle():
    """Each vertex of the triangle must face the shared centroid."""
    expected = {
        0.0: -180.0,
        2.0 * math.pi / 3.0: -60.0,
        4.0 * math.pi / 3.0: 60.0,
    }
    for phase, degrees in expected.items():
        trajectory = _figure_eight_trajectory(phase, "center")
        target = trajectory._figure_eight_target(0.0)
        delta = math.degrees(target["yaw"]) - degrees
        assert abs(math.atan2(math.sin(math.radians(delta)),
                              math.cos(math.radians(delta)))) < 1e-9


def test_velocity_heading_remains_the_default_and_still_rotates():
    """The MATLAB-faithful tangent heading must stay selectable."""
    trajectory = _figure_eight_trajectory(0.0, "velocity")
    duration = float(trajectory.main_duration_s)
    yaws = [
        math.degrees(trajectory._figure_eight_target(float(e))["yaw"])
        for e in np.linspace(0.05 * duration, 0.95 * duration, 25)
    ]
    assert max(yaws) - min(yaws) > 45.0, "tangent heading must sweep the curve"


def test_center_heading_removes_the_entry_yaw_step():
    """Entry must end exactly at the constant formation heading."""
    trajectory = _figure_eight_trajectory(math.pi, "center")
    entry = trajectory._entry_target(float(trajectory.config.entry_duration_s))
    main = trajectory._figure_eight_target(0.0)

    assert math.isclose(entry["yaw"], main["yaw"], abs_tol=1e-9)
    assert math.isclose(entry["yaw_rate"], 0.0, abs_tol=1e-9)


def test_unknown_figure_eight_heading_is_rejected():
    try:
        CircularFlightTrajectory(_config(
            trajectory_mode="figure_eight_triangle",
            figure_eight_heading="sideways",
        ))
    except ValueError as error:
        assert "figure_eight_heading" in str(error)
    else:
        raise AssertionError("an unknown heading mode must be rejected")

