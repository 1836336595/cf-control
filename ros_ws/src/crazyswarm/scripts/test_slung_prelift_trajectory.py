"""Prelift references must produce sustained PID lift and bounded recovery."""

import importlib

import numpy as np
import pytest

from test_ctbr_controller_v2 import _config, _controller_module, _state, _target


def _trajectory(**overrides):
    module = importlib.import_module("slung_prelift_trajectory")
    parameters = dict(
        clearance_m=0.20, clearance_margin_m=0.01,
        speed_mps=0.05, acceleration_mps2=0.05,
        max_rise_m=1.8, max_lead_m=1.6,
        timeout_s=60.0, alignment_duration_s=2.0,
    )
    parameters.update(overrides)
    return module.SlungPreliftTrajectory(**parameters)


def _start(trajectory):
    positions = np.array([[0.3, 0.0, 0.7], [-0.2, 0.3, 0.7], [-0.2, -0.3, 0.7]])
    trajectory.start(positions, positions, now=0.0)
    return positions


def test_reference_continues_upwards_when_payload_does_not_move():
    trajectory = _trajectory()
    positions = _start(trajectory)
    samples = [trajectory.update(positions, clearance_m=0.0, now=t)
               for t in np.arange(0.0, 15.01, 0.01)]
    assert samples[-1]["positions"][0, 2] > 1.30
    assert samples[-1]["active"]
    assert np.allclose(samples[-1]["positions"][:, :2], positions[:, :2])
    assert max(np.max(np.abs(s["velocities"])) for s in samples) <= 0.05 + 1e-9
    assert max(np.max(np.abs(s["accelerations"])) for s in samples) <= 0.05 + 1e-9


def test_clearance_hold_preserves_the_commanded_height_bias():
    trajectory = _trajectory()
    positions = _start(trajectory)
    for t in np.arange(0.0, 10.0, 0.01):
        trajectory.update(positions, clearance_m=0.0, now=t)
    for t in np.arange(10.0, 12.0, 0.01):
        held = trajectory.update(positions, clearance_m=0.22, now=t)
    assert held["stage"] == "holding"
    assert held["positions"][0, 2] > positions[0, 2] + 0.3
    assert np.allclose(held["velocities"], 0.0)
    positions[:, 2] -= 0.01
    next_sample = trajectory.update(positions, clearance_m=0.22, now=12.0)
    assert np.allclose(next_sample["positions"], held["positions"])


@pytest.mark.parametrize("parameters,reason", [
    ({"max_rise_m": 0.3}, "max_rise"),
    ({"max_lead_m": 0.3}, "max_lead"),
    ({"timeout_s": 3.0}, "timeout"),
])
def test_unmoving_payload_stops_at_a_bound(parameters, reason):
    trajectory = _trajectory(**parameters)
    positions = _start(trajectory)
    for t in np.arange(0.0, 30.0, 0.01):
        sample = trajectory.update(positions, clearance_m=0.0, now=t)
        if sample["failed"]:
            break
    assert sample["failed"]
    assert sample["reason"] == reason
    if reason != "timeout":
        assert np.max(sample["positions"][:, 2] - positions[:, 2]) <= 0.3 + 1e-9


def test_payload_pose_gap_freezes_reference_and_resumes_without_a_step():
    trajectory = _trajectory()
    positions = _start(trajectory)
    for t in np.arange(0.0, 5.0, 0.01):
        previous = trajectory.update(positions, clearance_m=0.0, now=t)
    paused = trajectory.update(positions, clearance_m=float("nan"), now=8.0)
    assert np.allclose(paused["positions"], previous["positions"])
    recovered = trajectory.update(positions, clearance_m=0.0, now=8.01)
    assert np.max(np.abs(recovered["positions"] - paused["positions"])) < 0.001


def test_alignment_respects_reference_speed_and_acceleration_limits():
    trajectory = _trajectory()
    measured = np.tile([0.0, 0.0, 0.7], (3, 1))
    source = measured + [0.0, 0.0, 0.5]
    trajectory.start(measured, source, 0.0)
    for now in np.arange(0.0, 3.0, 0.01):
        sample = trajectory.update(measured, clearance_m=0.0, now=now)
        assert np.max(np.abs(sample["velocities"])) <= 0.05 + 1e-9
        assert np.max(np.abs(sample["accelerations"])) <= 0.05 + 1e-9


def test_reference_lead_bound_still_applies_after_clearance_is_met():
    trajectory = _trajectory(max_lead_m=0.8)
    measured = _start(trajectory)
    for now in np.arange(0.0, 10.0, 0.01):
        trajectory.update(measured, clearance_m=0.0, now=now)
    for now in np.arange(10.0, 12.0, 0.01):
        sample = trajectory.update(measured, clearance_m=0.22, now=now)
    assert sample["stage"] == "holding"
    measured[:, 2] -= 1.0
    trajectory.update(measured, clearance_m=0.22, now=23.0)
    sample = trajectory.update(measured, clearance_m=0.22, now=34.0)
    assert sample["failed"]
    assert sample["reason"] == "max_lead"


def test_lead_bound_holds_reference_before_failing():
    trajectory = _trajectory(max_lead_m=0.3, lead_hold_s=1.0)
    measured = _start(trajectory)
    for now in np.arange(0.0, 20.0, 0.01):
        sample = trajectory.update(measured, clearance_m=0.0, now=now)
        if sample["stage"] == "lead_hold":
            break
    assert sample["stage"] == "lead_hold"
    held = sample["positions"].copy()
    sample = trajectory.update(measured, clearance_m=0.0, now=now + 0.5)
    assert sample["stage"] == "lead_hold"
    assert np.allclose(sample["positions"], held)
    sample = trajectory.update(measured, clearance_m=0.0, now=now + 1.1)
    assert sample["failed"]
    assert sample["reason"] == "max_lead"


def test_lead_hold_resumes_rising_when_aircraft_tracking_recovers():
    trajectory = _trajectory(max_lead_m=0.30, lead_hold_s=1.0)
    measured = _start(trajectory)
    for now in np.arange(0.0, 20.0, 0.01):
        sample = trajectory.update(measured, clearance_m=0.0, now=now)
        if sample["stage"] == "lead_hold":
            break
    held = sample["positions"].copy()
    measured[:, 2] += 0.08
    for t in np.arange(now + 0.01, now + 0.21, 0.01):
        sample = trajectory.update(measured, clearance_m=0.08, now=t)
    assert sample["stage"] == "rising"
    assert np.all(sample["positions"][:, 2] > held[:, 2])
    assert np.max(sample["velocities"][:, 2]) <= trajectory.speed_mps


def test_lead_hold_no_progress_timer_expires_even_if_lead_dips_below_limit():
    trajectory = _trajectory(max_lead_m=0.30, lead_hold_s=1.0)
    measured = _start(trajectory)
    for now in np.arange(0.0, 20.0, 0.01):
        sample = trajectory.update(measured, clearance_m=0.0, now=now)
        if sample["stage"] == "lead_hold":
            break
    measured[:, 2] += 0.001
    for t in np.arange(now + 0.01, now + 1.2, 0.01):
        sample = trajectory.update(measured, clearance_m=0.0, now=t)
    assert sample["failed"]
    assert sample["reason"] == "max_lead"


def test_lead_hold_clears_internal_reference_speed():
    trajectory = _trajectory(max_lead_m=0.30)
    measured = _start(trajectory)
    for now in np.arange(0.0, 20.0, 0.01):
        sample = trajectory.update(measured, clearance_m=0.0, now=now)
        if sample["stage"] == "lead_hold":
            break
    assert trajectory.speed == 0.0


def test_original_pid_lifts_and_sustains_a_ground_supported_payload():
    """One-dimensional taut-cable dynamics includes total mass and ground support."""
    module = _controller_module()
    trajectory = _trajectory()
    anchors = _start(trajectory)
    masses = [0.0434, 0.0440, 0.0424]
    controllers = [module.GeometricCtbrController(_config(
        module, mass=mass,
        independent_position_gain=np.array([3.0, 3.0, 4.0]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.6]),
        independent_integral_gain=np.array([0.8, 0.8, 0.8]),
        independent_integral_limit=np.full(3, 0.20),
        independent_integral_gate=0.20,
        independent_max_feedback_acceleration=np.array([6.0, 6.0, 8.0]),
    )) for mass in masses]
    total_mass = sum(masses) + 0.054
    height = velocity = 0.0
    history = []
    for t in np.arange(0.0, 45.0, 0.01):
        measured = anchors + [0.0, 0.0, height]
        reference = trajectory.update(measured, clearance_m=height, now=t)
        assert not reference["failed"]
        force = 0.0
        for i, controller in enumerate(controllers):
            state = _state()
            state.update(position=measured[i], velocity=np.array([0.0, 0.0, velocity]))
            target = _target(acceleration=reference["accelerations"][i])
            target.update(position=reference["positions"][i],
                          velocity=reference["velocities"][i], independent_mode=True)
            force += controller.compute(state, target, 0.01)["collective_thrust"]
        velocity += 0.01 * (force / total_mass - 9.80665)
        height += 0.01 * velocity
        if height < 0.0:
            height = velocity = 0.0
        history.append((height, velocity, force))
    assert min(h[0] for h in history[-500:]) >= 0.20
    assert max(abs(h[1]) for h in history[-500:]) < 0.05
    assert abs(np.mean([h[2] for h in history[-500:]]) - total_mass * 9.80665) < 0.02


def test_prelift_pid_increases_lift_at_fixed_lead_with_reduced_thrust_effectiveness():
    """A 70% thrust-effectiveness stress case must not plateau at the lead limit."""
    from test_ctbr_controller_v2 import _prelift_integral_controller, _prelift_integral_target
    module = _controller_module()
    trajectory = _trajectory()
    anchors = _start(trajectory)
    # Start from the latest CSV's lead-hold condition: 1.6 m bias, I=-0.2.
    trajectory.stage = "lead_hold"
    trajectory.lead_hold_started = trajectory.lead_hold_clearance = 0.0
    trajectory.positions = anchors + [0.0, 0.0, 1.6]
    controllers = [_prelift_integral_controller(module) for _ in range(3)]
    for mass, controller in zip([0.0434, 0.0440, 0.0424], controllers):
        controller.config.mass = mass
        controller.position_integral[2] = -0.2
    total_mass = 0.0434 + 0.0440 + 0.0424 + 0.054
    height = velocity = 0.0
    history = []
    for now in np.arange(0.0, 35.0, 0.01):
        measured = anchors + [0.0, 0.0, height]
        ref = trajectory.update(measured, clearance_m=height, now=now)
        assert not ref["failed"]
        force = 0.0
        for i, controller in enumerate(controllers):
            state = _state()
            state.update(position=measured[i], velocity=np.array([0.0, 0.0, velocity]))
            target = _prelift_integral_target()
            target.update(position=ref["positions"][i], velocity=ref["velocities"][i],
                          acceleration=ref["accelerations"][i])
            force += controller.compute(state, target, 0.01)["collective_thrust"]
        velocity += 0.01 * (0.70 * force / total_mass - 9.80665)
        height = max(0.0, height + 0.01 * velocity)
        if height == 0.0:
            velocity = max(0.0, velocity)
        history.append((height, velocity, force))
    assert min(h[0] for h in history[-300:]) >= 0.20
    assert max(abs(h[1]) for h in history[-300:]) < 0.05
    assert history[500][2] > history[0][2] + 0.2


def test_handoff_reference_preserves_bias_then_fades_to_measured_anchors():
    module = importlib.import_module("slung_prelift_trajectory")
    anchors = np.zeros((3, 3))
    held = anchors + [0.0, 0.0, 1.0]
    for blend, expected in [(0.0, 1.0), (0.5, 0.5), (1.0, 0.0)]:
        positions, velocities, accelerations = module.handoff_references(
            anchors, held, blend, 0.0, 0.0,
        )
        assert np.allclose(positions[:, 2], expected)
        assert np.allclose(velocities, 0.0)
        assert np.allclose(accelerations, 0.0)
