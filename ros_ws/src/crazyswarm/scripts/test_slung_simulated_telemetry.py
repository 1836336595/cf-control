"""Sampled EKF transport must exercise the deployed subscriber and filter."""

import math

import numpy as np
import pytest

import simulate_slung_handoff as replay
from test_ctbr_controller_v2 import _controller_module, _slung_manager


def _link(*args, **kwargs):
    from slung_simulated_telemetry import SampledEkfTelemetry
    return SampledEkfTelemetry(*args, **kwargs)


def test_sample_rate_is_independent_of_pose_rate():
    link = _link(sample_rate_hz=50, delay_s=.02)
    state = {"position": np.zeros(3), "velocity": np.zeros(3)}
    for tick in range(101):
        link.sample_if_due(tick / 100, [state])
    assert link.sample_count == 51  # Includes t=0 and t=1.
    assert math.isclose(link.next_sample_time, 1.02)


def test_delayed_packets_use_actual_callback_and_sample_timestamp(tmp_path, monkeypatch):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    node = manager.vehicles[0]
    try:
        link = _link(sample_rate_hz=50, delay_s=.02)
        state = {"position": np.array([.1, .2, .3]), "velocity": np.zeros(3),
                 "acceleration": np.full(3, 999.0)}
        link.sample_if_due(0, [state])
        state["position"][:] = 9  # A queued packet must retain its sampled pose.
        monkeypatch.setattr(module.time, "monotonic", lambda: .019)
        assert link.deliver(.019, [node]) == 0
        assert node.latest_ekf_state is None
        monkeypatch.setattr(module.time, "monotonic", lambda: .02)
        assert link.deliver(.02, [node]) == 1
        np.testing.assert_allclose(node.latest_ekf_state["position"], [.1, .2, .3])
        assert node.latest_ekf_state["firmware_sample_time_s"] == 0
        assert node.latest_ekf_state["received_time"] == .02
        assert not node.latest_ekf_state["filter_derivatives_valid"]
        # A velocity step must be filtered by the existing host filter. The
        # simulation's truth acceleration must never be assigned as feedback.
        state["velocity"] = np.array([1., 0., 0.])
        link.sample_if_due(.02, [state])
        monkeypatch.setattr(module.time, "monotonic", lambda: .04)
        link.deliver(.04, [node])
        filtered = node.latest_ekf_state
        assert filtered["filter_derivatives_valid"]
        assert 0 < filtered["filtered_velocity"][0] < 1
        np.testing.assert_allclose(filtered["filtered_acceleration"],
                                   filtered["filtered_velocity"] / .02)
        assert filtered["firmware_sample_time_s"] == .02
        assert link.delivered_count == 2
    finally:
        manager.logger.close()


def test_real_callback_reseeds_after_firmware_sample_gap(tmp_path, monkeypatch):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    node = manager.vehicles[0]
    try:
        link = _link(50, 0)
        state = {"position": np.zeros(3), "velocity": np.ones(3)}
        for now in (0, .02, .2):
            monkeypatch.setattr(module.time, "monotonic", lambda t=now: t)
            link.sample_if_due(now, [state])
            link.deliver(now, [node])
        assert node.latest_ekf_state["firmware_sample_time_s"] == .2
        assert not node.latest_ekf_state["filter_derivatives_valid"]
        np.testing.assert_array_equal(node.latest_ekf_state["filtered_acceleration"], 0)
    finally:
        manager.logger.close()


@pytest.mark.parametrize("rate,delay", [(0, .02), (float("nan"), .02), (50, -.01)])
def test_invalid_transport_settings_are_rejected(rate, delay):
    with pytest.raises(ValueError):
        _link(rate, delay)


def test_simulation_delays_and_ekf_mode_resolve_from_parameter_file():
    settings = replay.resolve_simulation_settings({
        "ekf_feedback_mode": "sampled_filtered", "ekf_sample_rate_hz": 50,
        "ekf_delay_s": .03, "pose_delay_s": .02, "command_delay_s": .01,
        "time_limit_s": 180,
    })
    assert settings["ekf_feedback_mode"] == "sampled_filtered"
    assert settings["ekf_delay_s"] == .03
    assert settings["pose_delay_s"] == .02
    assert settings["command_delay_s"] == .01
    assert replay.resolve_simulation_settings({}, pose_delay_s=.04)["pose_delay_s"] == .04


def test_incomplete_post_handoff_window_cannot_report_success():
    assert replay.observation_window_failure(29, 84, 60) == "post_handoff_window_incomplete"
    assert replay.observation_window_failure(29, 89, 60) is None
    assert replay.observation_window_failure(None, 89, 60) == "handoff_timeout"
