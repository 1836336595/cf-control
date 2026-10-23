"""Replay must use the physical calibration and bridge settings from YAML."""

import sys
import inspect
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import simulate_slung_handoff as replay


def test_replay_has_no_unused_onboard_controller_switch_or_ideal_rate_bandwidth():
    assert "onboard_controller" not in replay.PLANT_SETTING_NAMES
    assert not any(name.startswith("onboard_pid_") for name in replay.PLANT_SETTING_NAMES)
    assert "rate_bandwidth" not in inspect.signature(replay.run_simulation).parameters
    assert "rate_bandwidth" not in inspect.signature(replay.build_physics_config).parameters


def test_retired_positional_bandwidth_cannot_be_misread_as_observation_duration():
    with pytest.raises(TypeError):
        inspect.signature(replay.run_simulation).bind("output",None,.01,.005,(35.,35.,20.))


@pytest.mark.parametrize("obsolete,value", [
    ("onboard_controller","pid"), ("onboard_controller","first_order"),
    ("onboard_controller","mellinger"), ("onboard_pid_rate_kp",[250.,250.,120.]),
    ("rate_loop_bandwidth",[35.,35.,20.]),
])
def test_obsolete_simulation_options_do_not_silently_mislabel_the_fixed_model(obsolete,value):
    with pytest.raises(ValueError,match=obsolete):
        replay.resolve_simulation_settings({obsolete:value})


def test_replay_builds_paper_plant_and_preserves_measured_masses(tmp_path):
    from test_ctbr_controller_v2 import _controller_module, _slung_manager
    sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "simulate_uav" / "udaan"))
    from udaan.crazyswarm_slung_parameters import default_config
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    try:
        config = replay.build_physics_config(default_config(), manager, {
            "motor_model": "graefe2026",
            "brushless_motor_time_constant_s": .05,
            "bridge_collective_pwm_reference": 65535,
            "bridge_collective_pwm_limit": 60000,
            "bridge_thrust_raw_scale": [1.02, 1.03, 1.04],
            "vehicle_inertia": [3.3e-5, 3.6e-5, 5.9e-5],
            "motor_available_command_fraction": [.9, .95, 1.0],
        })
        assert config.motor_model == "graefe2026"
        assert config.bridge_collective_pwm_reference == 65535
        assert config.bridge_collective_pwm_limit == 60000
        np.testing.assert_allclose(config.bridge_thrust_raw_scale, [1.02, 1.03, 1.04])
        np.testing.assert_allclose(config.vehicle_masses, [.0434, .0440, .0424])
        np.testing.assert_allclose(config.cable_lengths, manager.slung_controller.config.link_lengths_m)
        assert config.vehicles[0].max_total_thrust == manager.vehicles[0].controller.config.max_total_thrust
        assert config.payload_mass == manager.slung_controller.config.payload_mass_kg
    finally:
        manager.logger.close()


def test_trace_motor_diagnostics_distinguish_actual_force_and_command():
    actuator = SimpleNamespace(raw_collective=52000, motor_pwm=np.array([50000, 51000, 52000, 53000]),
                               motor_speeds=np.array([2100, 2200, 2300, 2400]),
                               motor_forces=np.array([.1, .2, .3, .4]),
                               motor_reaction_torques=np.array([.001]*4))
    model = SimpleNamespace(_onboard_actuators=[actuator], _actual_thrust=[1.0],
                            config=SimpleNamespace(motor_model="graefe2026"))
    result = replay.actuator_trace_fields(model, 0)
    assert result["actual_thrust_n"] == 1.0
    assert result["collective_raw_thrust"] == 52000
    assert result["motor_pwm_4"] == 53000
    assert result["motor_speed_radps_1"] == 2100
    assert result["motor_force_n_3"] == .3


def test_replay_can_hold_physical_cables_fixed_while_testing_nominal_length_error(tmp_path):
    from test_ctbr_controller_v2 import _controller_module, _slung_manager
    sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "simulate_uav" / "udaan"))
    from udaan.crazyswarm_slung_parameters import default_config
    module=_controller_module()
    manager,_=_slung_manager(module,tmp_path,{'slung_payload':{'link_lengths_m':[.709,.699,.654]}})
    try:
        config=replay.build_physics_config(default_config(),manager,
            {'physical_link_lengths_m':[.709,.699,.679]})
        np.testing.assert_allclose(config.cable_lengths,[.709,.699,.679])
        np.testing.assert_allclose(manager.slung_controller.config.link_lengths_m,[.709,.699,.654])
    finally:manager.logger.close()
