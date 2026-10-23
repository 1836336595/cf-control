"""Offline plant settings must stay separate from flight ROS parameters."""

from copy import deepcopy
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
from xmlrpc.client import dumps

import pytest
import yaml

import simulate_slung_handoff as replay
from slung_circle_reference import CircleTrajectoryConfig
from test_ctbr_controller_v2 import _controller_module, _slung_manager


PACKAGE = Path(__file__).resolve().parents[1]


def test_default_simulation_file_preserves_current_physics_sensors_and_window():
    settings = replay.load_simulation_config()
    assert settings["motor_model"] == "graefe2026"
    assert settings["brushless_motor_time_constant_s"] == .05
    assert settings["mellinger_rate_hz"] == 500.
    assert settings["mellinger_attitude_gains"] == [70000., 70000., 60000.]
    assert settings["pose_sample_rate_hz"] == 100.
    assert settings["pose_delay_s"] == .01
    assert settings["ekf_sample_rate_hz"] == 50.
    assert settings["ekf_delay_s"] == .02
    assert settings["command_delay_s"] == .005
    assert settings["time_limit_s"] == 180.
    assert settings["post_handoff_s"] == 100.


@pytest.mark.parametrize("launch_name", ["hover_swarm.launch", "ctbr_controller.launch"])
def test_launch_loaded_flight_parameters_exclude_simulation_and_remain_xmlrpc_safe(launch_name):
    payload = yaml.safe_load((PACKAGE / "config/slung_payload.yaml").read_text())["slung_payload"]
    assert "simulation" not in payload
    assert "post_handoff_s" not in payload
    # The live task is selected by the user; this serialization/isolation
    # check must accept every supported mission rather than pinning cone mode.
    configured = CircleTrajectoryConfig.from_mapping(payload["trajectory"])
    assert configured.mode in ("hover", "circle", "figure_eight", "attitude_cone")
    assert payload["flight_validation_transport_s"] > 0.
    values = []
    for element in ET.parse(PACKAGE / "launch" / launch_name).iter("rosparam"):
        filename = element.get("file")
        if filename:
            path = Path(filename.replace("$(find crazyswarm)", str(PACKAGE)))
            values.append(yaml.safe_load(path.read_text()))
        else:
            values.append(yaml.safe_load(element.text or "{}"))
    assert all("slung_simulation" not in value for value in values if isinstance(value, dict))
    dumps((values,), allow_none=False)


@pytest.mark.parametrize("live", [False, True])
def test_simulation_candidate_never_enters_flight_parameter_server(tmp_path, live):
    module = _controller_module()
    overlay = {"slung_simulation": {"post_handoff_s": 70., "pose_delay_s": .04},
               "slung_payload": {"mass_kg": .024}}
    original = deepcopy(overlay)
    manager, _ = _slung_manager(module, tmp_path, overlay, use_live_payload_trajectory=live)
    try:
        assert not module.rospy.has_param("/slung_simulation")
        payload = module.rospy.get_param("/slung_payload")
        assert "simulation" not in payload
        assert "post_handoff_s" not in payload
        assert payload["mass_kg"] == .024
        assert manager.payload_flight_validation_transport_s == (100. if live else 60.)
        assert overlay == original
    finally:
        manager.logger.close()


@pytest.mark.parametrize("legacy", [{"simulation": {"pose_delay_s": .03}}, {"post_handoff_s": 60.}])
def test_old_candidate_layout_fails_with_the_new_namespace_in_the_error(legacy):
    with pytest.raises(ValueError, match="slung_simulation"):
        replay.load_simulation_config(parameter_overrides={"slung_payload": legacy})


@pytest.mark.parametrize("invalid", [None, [], {"slung_simulation": []}, {"slung_payload": {}}])
def test_simulation_config_file_requires_its_own_mapping_root(tmp_path, invalid):
    path = tmp_path / "simulation.yaml"
    path.write_text(yaml.safe_dump(invalid))
    with pytest.raises(ValueError, match="slung_simulation"):
        replay.load_simulation_config(path=path)


def test_import_and_load_do_not_install_ros_or_import_the_physics_runtime():
    script = """
import sys
import simulate_slung_handoff as replay
settings = replay.load_simulation_config()
assert settings['post_handoff_s'] == 100.
assert not {'rospy', 'mujoco', 'udaan'} & set(sys.modules)
"""
    completed = subprocess.run([sys.executable, "-c", script],
                               cwd=Path(replay.__file__).parent,
                               text=True, capture_output=True)
    assert completed.returncode == 0, completed.stdout + completed.stderr


@pytest.mark.parametrize("duration", [0., -.1, float("nan"), float("inf")])
def test_simulation_observation_duration_must_be_finite_and_positive(duration):
    with pytest.raises(ValueError, match="post_handoff_s"):
        replay.resolve_simulation_settings({"post_handoff_s": duration})
