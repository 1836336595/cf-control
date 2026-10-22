"""Preflight regressions using real nodes with only ROS transport stubbed."""

import numpy as np

from test_ctbr_controller_v2 import _controller_module, _slung_manager


def _sample(node, now, derivatives=True, error=0.001):
    node.latest_ekf_state = dict(
        position=np.array([error, 0.0, 0.0]),
        filtered_velocity=np.zeros(3), filtered_acceleration=np.zeros(3),
        filter_derivatives_valid=derivatives, received_time=now,
    )


def _node(tmp_path):
    module = _controller_module()
    module.rospy.logerr = lambda *_args: None
    manager, _ = _slung_manager(module, tmp_path)
    node = manager.vehicles[0]
    node.ekf_alignment_hold_s = 1.0
    node.ekf_alignment_timeout_s = 5.0
    state = dict(node.latest_state, position=np.zeros(3))
    return manager, node, state


def test_ekf_position_alignment_survives_derivative_warmup_but_gate_waits(tmp_path):
    manager, node, state = _node(tmp_path)
    try:
        # Fresh positions remain within 1 mm. A radio-log gap may cause the
        # velocity filter to reseed, which does not invalidate those positions.
        for index in range(50):
            now = index * .02
            _sample(node, now, derivatives=index % 10 != 0)
            assert not node._ekf_preflight_ready(state, now)
        _sample(node, 1.0, derivatives=False)
        assert not node._ekf_preflight_ready(state, 1.0)
        _sample(node, 1.02, derivatives=True)
        assert node._ekf_preflight_ready(state, 1.02)
        assert not node.ekf_alignment_failed
    finally:
        manager.logger.close()


def test_stale_ekf_resets_position_alignment_and_never_releases_gate(tmp_path):
    manager, node, state = _node(tmp_path)
    try:
        _sample(node, 0.0)
        assert not node._ekf_preflight_ready(state, 0.0)
        # This is older than ekf_state_timeout even if position remains close.
        assert not node._ekf_preflight_ready(state, .4)
        for now in (.42, .9, 1.3):
            _sample(node, now)
            assert not node._ekf_preflight_ready(state, now)
        _sample(node, 1.44)
        assert node._ekf_preflight_ready(state, 1.44)
    finally:
        manager.logger.close()


def test_position_mismatch_resets_alignment_with_valid_derivatives(tmp_path):
    manager, node, state = _node(tmp_path)
    try:
        for now, error in ((0.0, .001), (.8, .06), (1.0, .001), (1.8, .001)):
            _sample(node, now, error=error)
            assert not node._ekf_preflight_ready(state, now)
        _sample(node, 2.02)
        assert node._ekf_preflight_ready(state, 2.02)
    finally:
        manager.logger.close()


def test_prewarming_filter_cannot_release_or_hide_preflight_timeout(tmp_path):
    manager, node, state = _node(tmp_path)
    try:
        for now in (0, 1.0, 2.0, 5.01):
            _sample(node, now, derivatives=False)
            assert not node._ekf_preflight_ready(state, now)
        assert node.ekf_alignment_failed
        _sample(node, 5.03)
        assert not node._ekf_preflight_ready(state, 5.03)
    finally:
        manager.logger.close()


def test_repeated_battery_sample_recovers_missing_first_packet_and_scale_stays_frozen(tmp_path, monkeypatch):
    manager, node, state = _node(tmp_path)
    module = __import__(node.__class__.__module__)
    try:
        assert node._preflight_voltage_target(state, 0.0)["zero_output"]
        from types import SimpleNamespace
        monkeypatch.setattr(module.time, "monotonic", lambda: .5)
        node._battery_callback(SimpleNamespace(values=[4.0, 4000.0, 0.0, 80.0]))
        assert node._preflight_voltage_target(state, .55) is None
        assert node.preflight_voltage_ready
        assert np.isclose(node.preflight_voltage_v, 4.0)
        assert np.isclose(node.thrust_raw_scale, 1.05)
        # Later periodic telemetry must never change the frozen PWM scaling.
        monkeypatch.setattr(module.time, "monotonic", lambda: 1.0)
        node._battery_callback(SimpleNamespace(values=[3.9, 3900.0, 0.0, 70.0]))
        assert node._preflight_voltage_target(state, 1.05) is None
        assert np.isclose(node.thrust_raw_scale, 1.05)
    finally:
        manager.logger.close()


def test_missing_or_stale_battery_still_times_out_at_zero_thrust(tmp_path):
    manager, node, state = _node(tmp_path)
    try:
        node.preflight_voltage_samples.append((-2.0, 4.0))
        assert node._preflight_voltage_target(state, 0.0)["zero_output"]
        target = node._preflight_voltage_target(state, node.preflight_voltage_timeout_s + .01)
        assert target["zero_output"]
        assert node.preflight_voltage_failed
        assert not node.preflight_voltage_ready
    finally:
        manager.logger.close()
