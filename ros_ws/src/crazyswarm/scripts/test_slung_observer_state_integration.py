"""ROS-boundary behavior for accepted payload pose and observer quality."""

import math
import sys
import threading
import types

import numpy as np
import pytest
import yaml

from test_ctbr_controller_v2 import (
    CONTROLLER_CONFIG_PATH, _configure_controller_node_ros,
    _controller_module, _controller_parameter_roots, _node_logger,
)
from slung_load_controller import PayloadStateObserver


def _message(timestamp, position=(0., 0., .53), yaw=0., valid=True):
    vector = lambda values: types.SimpleNamespace(x=values[0], y=values[1], z=values[2])
    return types.SimpleNamespace(
        valid=valid, derivatives_valid=True,
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: timestamp)),
        pose=types.SimpleNamespace(
            position=vector(position),
            orientation=types.SimpleNamespace(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
        twist=types.SimpleNamespace(linear=vector((99., 98., 97.)), angular=vector((96., 95., 94.))),
        acceleration=vector((93., 92., 91.)),
    )


def _manager():
    module = _controller_module()
    module.rospy.Time.now = lambda: types.SimpleNamespace(to_sec=lambda: 10.10)
    manager = module.MultiCtbrControllerNode.__new__(module.MultiCtbrControllerNode)
    manager.transport_mode = "slung_load"
    manager.payload_lock = threading.Lock()
    manager.payload_state_observer_lock = threading.Lock()
    manager.payload_state_observer = PayloadStateObserver(
        max_velocity_mps=1., max_acceleration_mps2=8., max_body_rate_rps=2.)
    manager.payload_height_m = .06
    manager.latest_payload_state = None
    manager.last_valid_payload_geometry = None
    manager.vehicles = [types.SimpleNamespace(state_timeout=.1)]
    manager._publish_payload_path = lambda _state: None
    return manager


def _ready_manager():
    manager = _manager()
    for index in range(10):
        manager._payload_state_callback(_message(10. + index * .01))
    assert manager._payload_ready(manager.latest_payload_state["received_time"])
    return manager


def test_newly_accepted_pose_keeps_raw_pose_delay_and_flattens_observer_metadata():
    manager = _ready_manager()
    manager._payload_state_callback(_message(10.10, (.001, 0., .53), yaw=.01))
    state = manager.latest_payload_state
    np.testing.assert_allclose(state["position"], [.001, 0., .50])
    np.testing.assert_allclose(state["position"], state["raw_position"])
    np.testing.assert_allclose(state["rotation"], state["raw_rotation"])
    assert not np.allclose(state["rotation"], manager.payload_state_observer.rotation)
    assert state["source_valid"]
    assert state["observer_updated"]
    assert state["observer_quality_valid"]
    assert state["source_stamp"] == state["sample_time"] == 10.10
    assert state["observer_status"] == "updated"
    assert state["input_dt_s"] == pytest.approx(.01)
    assert state["position_innovation"][0] == pytest.approx(.001)
    assert state["velocity"][0] < 1.
    assert state["raw_velocity"][0] == 99.


@pytest.mark.parametrize("timestamp,position,yaw", [
    (10.091, (.001, 0., .53), .001),
    (10.09, (.5, 0., .53), .2),
    (10.08, (.5, 0., .53), .2),
    (10.10, (.5, 0., .53), 0.),
    (10.10, (0., 0., .53), .2),
])
def test_nonaccepted_source_cannot_replace_control_pose_or_last_good_geometry(timestamp, position, yaw):
    manager = _ready_manager()
    before = manager.latest_payload_state
    geometry = manager.last_valid_payload_geometry
    manager._payload_state_callback(_message(timestamp, position, yaw))
    state = manager.latest_payload_state
    assert not state["derivatives_valid"]
    assert not state["observer_quality_valid"]
    assert not manager._payload_state_ready(state, state["received_time"])
    assert state["sample_time"] == before["sample_time"]
    assert state["source_stamp"] == timestamp
    for key in ("position", "rotation", "velocity", "acceleration", "body_rate"):
        np.testing.assert_allclose(state[key], before[key])
    np.testing.assert_allclose(manager.last_valid_payload_geometry["position"], geometry["position"])
    np.testing.assert_allclose(manager.last_valid_payload_geometry["rotation"], geometry["rotation"])
    assert np.linalg.norm(state["raw_position"] - state["position"]) > 0. or not np.allclose(state["raw_rotation"], state["rotation"])


@pytest.mark.parametrize("invalid_kind", ["valid_flag", "nan_position", "zero_quaternion", "nan_stamp", "malformed_message"])
def test_invalid_ros_message_holds_history_marks_source_bad_and_recovers_without_reset(invalid_kind):
    manager = _ready_manager()
    before = manager.latest_payload_state
    bad = _message(10.10)
    if invalid_kind == "valid_flag":
        bad.valid = False
    elif invalid_kind == "nan_position":
        bad.pose.position.x = float("nan")
    elif invalid_kind == "zero_quaternion":
        bad.pose.orientation.w = 0.
    elif invalid_kind == "nan_stamp":
        bad.header.stamp.to_sec = lambda: float("nan")
    else:
        del bad.pose
    manager._payload_state_callback(bad)
    state = manager.latest_payload_state
    assert state is not None
    assert not state["source_valid"]
    assert not state["valid"]
    assert not state["derivatives_valid"]
    assert not manager._payload_ready(state["received_time"])
    assert state["reset_count"] == before["reset_count"]
    assert state["sample_time"] == before["sample_time"]
    np.testing.assert_allclose(state["position"], before["position"])
    manager._payload_state_callback(_message(10.11))
    recovered = manager.latest_payload_state
    assert manager._payload_ready(recovered["received_time"])
    assert recovered["sample_count"] == before["sample_count"] + 1


def test_real_gap_starts_warmup_but_does_not_replace_last_quality_valid_geometry():
    manager = _ready_manager()
    sys.modules["rospy"].Time.now = lambda: types.SimpleNamespace(to_sec=lambda: 10.32)
    geometry = manager.last_valid_payload_geometry
    manager._payload_state_callback(_message(10.30, (.2, 0., .53)))
    state = manager.latest_payload_state
    assert state["observer_updated"]
    assert state["observer_status"] == "gap_reset"
    assert not manager._payload_ready(state["received_time"])
    np.testing.assert_allclose(state["position"], [.2, 0., .5])
    np.testing.assert_allclose(manager.last_valid_payload_geometry["position"], geometry["position"])
    manager._payload_state_callback(_message(10.31, (.2, 0., .53)))
    assert not manager._payload_ready(manager.latest_payload_state["received_time"])
    manager._payload_state_callback(_message(10.32, (.2, 0., .53)))
    assert manager._payload_ready(manager.latest_payload_state["received_time"])
    np.testing.assert_allclose(manager.last_valid_payload_geometry["position"], [.2, 0., .5])


def test_readiness_uses_supplied_snapshot_and_keeps_legacy_states_compatible():
    manager = _ready_manager()
    good = dict(manager.latest_payload_state)
    now = good["received_time"]
    manager.latest_payload_state = dict(good, observer_quality_valid=False)
    assert manager._payload_state_ready(good, now)
    assert not manager._payload_ready(now)
    assert not manager._payload_state_ready(good, now + .11)
    legacy = {"valid": True, "derivatives_valid": True, "received_time": now}
    assert manager._payload_state_ready(legacy, now)
    assert not manager._payload_state_ready(dict(legacy, received_time=float("nan")), now)


@pytest.mark.parametrize("timestamp", [9., 10.5])
def test_stale_or_future_source_stamp_cannot_advance_history_and_normal_time_recovers(timestamp):
    manager = _ready_manager()
    before = manager.latest_payload_state
    manager._payload_state_callback(_message(timestamp))
    state = manager.latest_payload_state
    assert not state["source_valid"]
    assert not state["derivatives_valid"]
    assert state["source_stamp"] == timestamp
    assert state["sample_time"] == before["sample_time"]
    assert state["sample_count"] == before["sample_count"]
    manager._payload_state_callback(_message(10.11))
    recovered = manager.latest_payload_state
    assert manager._payload_state_ready(recovered, recovered["received_time"])


def test_readiness_rechecks_source_age_and_tolerates_callback_arriving_after_tick_time():
    manager = _ready_manager()
    state = manager.latest_payload_state
    received = state["received_time"]
    assert manager._payload_state_ready(state, received - .01)
    assert not manager._payload_state_ready(state, received - .03)
    sys.modules["rospy"].Time.now = lambda: types.SimpleNamespace(to_sec=lambda: 10.25)
    assert not manager._payload_state_ready(state, received)


def test_ground_baseline_uses_the_same_snapshot_that_passed_quality_check():
    manager = _ready_manager()
    state = manager.latest_payload_state
    manager.payload_takeup_start_time = None
    manager.payload_transport_active = False
    manager.payload_ground_reference_samples = []
    manager.slung_controller = types.SimpleNamespace(config=types.SimpleNamespace(payload_size_m=np.array([.08, .06, .06])))
    manager.vehicles[0].trajectory = None
    original_check = manager._payload_state_ready

    def receive_rejected_geometry_after_check(selected, now):
        ready = original_check(selected, now)
        manager.latest_payload_state = dict(state, position=np.array([8., 0., 9.]), derivatives_valid=False)
        return ready

    manager._payload_state_ready = receive_rejected_geometry_after_check
    manager._update_payload_ground_reference(state["received_time"])
    assert manager.payload_ground_center_z == pytest.approx(.5)


def _initialize_parameters(payload_updates=None):
    module = _controller_module()
    roots = _controller_parameter_roots()
    roots.update(yaml.safe_load((CONTROLLER_CONFIG_PATH.parent / "slung_payload.yaml").read_text()))
    roots["slung_payload"].update(payload_updates or {})
    _configure_controller_node_ros(module, roots)
    manager = module.MultiCtbrControllerNode.__new__(module.MultiCtbrControllerNode)
    manager.transport_mode = "slung_load"
    manager.logger = _node_logger()
    manager._initialize_payload_parameters()
    return manager


def test_observer_parameters_pass_configured_sampling_and_innovation_values():
    manager = _initialize_parameters({
        "observer_min_dt_s": .004, "observer_nominal_dt_s": .008,
        "observer_position_innovation_margin_m": [.01, .02, .03],
        "observer_attitude_innovation_margin_deg": 3.,
    })
    observer = manager.payload_state_observer
    assert observer.min_dt_s == .004
    assert observer.nominal_dt_s == .008
    np.testing.assert_allclose(observer.position_innovation_margin, [.01, .02, .03])
    assert observer.attitude_innovation_margin == pytest.approx(math.radians(3.))


@pytest.mark.parametrize("updates", [
    {"observer_min_dt_s": 0.}, {"observer_min_dt_s": .02},
    {"observer_nominal_dt_s": float("nan")},
    {"observer_position_innovation_margin_m": -.01},
    {"observer_attitude_innovation_margin_deg": 0.},
    {"observer_min_dt_s": True}, {"state_observer_enabled": "false"},
])
def test_invalid_observer_sampling_and_innovation_parameters_fail_initialization(updates):
    with pytest.raises(RuntimeError, match="slung_payload"):
        _initialize_parameters(updates)
