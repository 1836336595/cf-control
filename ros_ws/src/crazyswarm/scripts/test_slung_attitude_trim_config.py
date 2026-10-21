"""New attitude reference adaptation parameters must be validated at startup."""
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager


@pytest.mark.parametrize("key,value", [
    ("transport_attitude_trim_enabled", "true"),
    ("transport_attitude_trim_gain_per_s", [-.1, .1, .1]),
    ("transport_attitude_trim_gain_per_s", [0., float("nan"), 0.]),
    ("transport_attitude_trim_gain_per_s", [1., 1.]),
    ("transport_attitude_trim_limit_m", 0.),
    ("transport_attitude_trim_rate_limit_mps", float("inf")),
    ("transport_attitude_trim_acceleration_limit_mps2", -.001),
    ("transport_attitude_trim_interval_s", 0.),
    ("transport_attitude_trim_limit_m", True),
    ("transport_attitude_trim_gain_per_s", [True, .1, .1]),
])
def test_bad_trim_parameters_are_rejected(tmp_path, key, value):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="trim"):
        _slung_manager(module, tmp_path, {"slung_payload": {key: value}})


def test_trim_rejects_unsupported_force_mode(tmp_path):
    module = _controller_module()
    with pytest.raises(RuntimeError, match="trim.*reference_cascade"):
        _slung_manager(module, tmp_path, {"slung_payload": {
            "transport_force_mode": "coupled", "transport_feedback_controller": "differential_integral",
            "transport_attitude_trim_enabled": True}})


def test_enabled_trim_configuration_is_read_by_manager(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path, {"slung_payload": {
        "transport_attitude_trim_enabled": True,
        "transport_attitude_trim_gain_per_s": [.05, .04, .02],
        "transport_attitude_trim_limit_m": .01,
        "transport_attitude_trim_rate_limit_mps": .001,
        "transport_attitude_trim_acceleration_limit_mps2": .001,
        "transport_attitude_trim_interval_s": 1.}})
    try:
        assert manager.payload_attitude_trim_enabled
        assert manager.payload_attitude_trim is not None
        assert manager.payload_attitude_trim_parameters["max_offset_m"] == .01
        assert manager.payload_attitude_trim_parameters["adaptation_interval_s"] == 1.
    finally:
        manager.logger.close()


def test_missing_switch_keeps_legacy_reference_behavior_disabled(tmp_path, monkeypatch):
    import test_ctbr_controller_v2 as fixtures
    configure = fixtures._configure_controller_node_ros

    def legacy_configuration(module, roots):
        roots["slung_payload"].pop("transport_attitude_trim_enabled", None)
        configure(module, roots)

    monkeypatch.setattr(fixtures, "_configure_controller_node_ros", legacy_configuration)
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    try:
        assert not manager.payload_attitude_trim_enabled
        assert not manager.payload_attitude_trim.enabled
    finally:
        manager.logger.close()
