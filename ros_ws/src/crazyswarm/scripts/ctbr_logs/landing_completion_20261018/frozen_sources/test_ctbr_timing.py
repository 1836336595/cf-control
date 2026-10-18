"""Control-loop caching and host timing regressions with ROS transport stubbed."""

import csv
import math
from types import SimpleNamespace

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_handoff_safety import prepare


def test_transport_and_payload_path_use_startup_parameters_without_rpc(tmp_path, monkeypatch):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path, {
        "ctbr_controller_cf3": {"transport_attitude_gain": [5., 6., 7.],
                                "transport_rate_gain": [.1, .2, .3]},
    })
    try:
        def forbidden(*_args, **_kwargs):
            raise AssertionError("parameter server read from a runtime path")
        monkeypatch.setattr(module.rospy, "get_param", forbidden)
        prepare(manager, anchors, 1.)
        overrides = manager._transport_overrides(1.)
        cf3 = manager.vehicles[0]
        np.testing.assert_array_equal(overrides[id(cf3)]["transport_attitude_gain"], [5, 6, 7])
        np.testing.assert_array_equal(overrides[id(cf3)]["transport_rate_gain"], [.1, .2, .3])
        # Exercise the throttled path too: the interval must already be cached.
        manager.payload_path_last_publish_time = module.time.monotonic()
        manager._publish_payload_path(manager.latest_payload_state)
    finally:
        manager.logger.close()


@pytest.mark.parametrize("name,value", [
    ("transport_attitude_gain", [-1, 6, 3]),
    ("transport_rate_gain", [0, float("nan"), 1]),
    ("transport_rate_gain", [1, 2]),
])
def test_invalid_transport_gain_is_rejected_at_startup(tmp_path, name, value):
    module = _controller_module()
    with pytest.raises((ValueError, module.rospy.ROSInitException)):
        _slung_manager(module, tmp_path, {"ctbr_controller_cf3": {name: value}})


class Stamp:
    def __init__(self, secs, nsecs):
        self.secs, self.nsecs = secs, nsecs

    def to_nsec(self):
        return self.secs * 1000000000 + self.nsecs

    def to_sec(self):
        return self.secs + self.nsecs / 1000000000


def test_bridge_snapshot_keeps_source_stamp_and_does_not_claim_current_latency(tmp_path, monkeypatch):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    node = manager.vehicles[0]
    try:
        monkeypatch.setattr(module.time, "monotonic", lambda: 10.)
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=Stamp(123, 456), seq=999), command_seq=7,
            command_age_s=.003, callback_duration_s=.005, send_duration_s=.002,
            raw_thrust=42000, command_accepted=True, unlock_packet=False,
        )
        node._ctbr_timing_callback(message)
        node.last_published_command_stamp_ns = Stamp(123, 457).to_nsec()
        row = node._timing_log_fields(10.1)
        assert row["bridge_latest_command_stamp_ns"] == Stamp(123, 456).to_nsec()
        assert row["bridge_latest_command_seq"] == 7  # Not diagnostic header.seq.
        assert row["bridge_latest_matches_published_stamp"] == 0
        assert row["bridge_latest_send_call_wall_s"] == .002
        assert row["bridge_latest_command_age_s"] == .003
        node.last_published_command_stamp_ns = Stamp(123, 456).to_nsec()
        assert node._timing_log_fields(10.1)["bridge_latest_matches_published_stamp"] == 1
        # Reject malformed timing as a whole, preserving the previous tuple.
        message.send_duration_s = -1
        node._ctbr_timing_callback(message)
        assert node._timing_log_fields(10.1)["bridge_latest_send_call_wall_s"] == .002
    finally:
        manager.logger.close()


def test_completed_cycle_timing_is_labeled_as_previous_even_for_zero_output(tmp_path, monkeypatch):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    node = manager.vehicles[0]
    try:
        counter = iter(np.arange(100., 101., .001))
        monkeypatch.setattr(module.time, "perf_counter", lambda: next(counter))
        node.latest_state = None
        node._publish_zero = lambda: None
        node._timer_callback(None, now=node.start_time + .01)
        duration = node.previous_vehicle_cycle_wall_s
        previous_time = node.previous_vehicle_cycle_control_time_s
        assert duration > 0
        node._timer_callback(None, now=node.start_time + .03)
        manager.logger.flush()
        rows = list(csv.DictReader(open(manager.logger.path)))
        assert math.isnan(float(rows[0]["previous_vehicle_cycle_wall_s"]))
        assert float(rows[1]["previous_vehicle_cycle_wall_s"]) == duration
        assert float(rows[1]["previous_vehicle_cycle_control_time_s"]) == previous_time
        assert math.isclose(float(rows[1]["controller_tick_interval_s"]), .02, abs_tol=1e-8)
    finally:
        manager.logger.close()


def test_wall_clock_profiling_does_not_change_control_dt_or_command(tmp_path, monkeypatch):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    node = manager.vehicles[0]
    try:
        now = 0.
        prepare(manager, anchors, now)
        node.preflight_voltage_ready = True
        node.set_fleet_gate(True)
        node.latest_state.update(filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
                                 filter_derivatives_valid=True)
        node.latest_ekf_state = dict(
            position=node.latest_state["position"].copy(), received_time=now,
            filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
            filter_derivatives_valid=True,
        )
        target = manager._transport_overrides(now)[id(node)]
        node.last_tick = now - 1/70
        node._publish_path = lambda _state: None
        node._publish_command = lambda _command: None
        measured_dt = []
        compute = node.controller.compute
        def measured(state, reference, dt):
            measured_dt.append(dt)
            return compute(state, reference, dt)
        node.controller.compute = measured
        counter = iter(np.arange(100., 101., .001))
        monkeypatch.setattr(module.time, "perf_counter", lambda: next(counter))
        node._timer_callback(None, now=now, transport_override=target)
        assert math.isclose(measured_dt[0], 1/70, abs_tol=1e-12)
        assert node.controller_compute_wall_s > 0
        np.testing.assert_allclose(node.last_control_command["desired_force"], [.08, 0, .8])
        assert node.last_tick == now
    finally:
        manager.logger.close()
