"""Replay initial geometry must stay independent of controller cable estimates."""

import csv
import json
import os
from pathlib import Path
import subprocess

import numpy as np
import pytest

import simulate_slung_handoff as replay


def test_default_initial_positions_preserve_nominal_without_aliasing():
    nominal = np.arange(9, dtype=float).reshape(3, 3)
    result = replay.resolve_initial_vehicle_positions(nominal)
    np.testing.assert_array_equal(result, nominal)
    result[0, 0] = 99
    assert nominal[0, 0] == 0
    assert replay.resolve_simulation_settings({})["initial_vehicle_positions_m"] is None


def test_explicit_initial_positions_are_world_positions_and_copied():
    configured = np.array([[.4, 0., .7], [-.2, .3, .8], [-.2, -.3, .9]])
    result = replay.resolve_initial_vehicle_positions(np.zeros((3, 3)), configured)
    np.testing.assert_array_equal(result, configured)
    configured[:] = 0
    assert result[0, 0] == .4


@pytest.mark.parametrize("positions", [[], [0., 0., 1.], [[0., 0., 1.]],
    [[0., 0., 1.]] * 4, np.full((3, 3), np.nan), np.full((3, 3), np.inf), "bad"])
def test_invalid_initial_positions_are_rejected(positions):
    with pytest.raises(ValueError, match="initial_vehicle_positions_m"):
        replay.resolve_initial_vehicle_positions(np.zeros((3, 3)), positions)


def test_shorter_physical_cables_accept_consistent_initial_positions(tmp_path):
    interpreter = Path("/home/csy/miniconda3/envs/udaan/bin/python")
    if not interpreter.exists():
        pytest.skip("Udaan MuJoCo interpreter unavailable")
    script = r'''
import json
from pathlib import Path
import numpy as np
import simulate_slung_handoff as replay
from test_ctbr_controller_v2 import _controller_module, _slung_manager
directory = Path(__import__('sys').argv[1])
module = _controller_module()
manager, _ = _slung_manager(module, directory/'geometry')
try:
    nominal = manager.slung_controller.config.link_lengths_m.copy()
    physical = nominal - np.array([.02, .01, .015])
    anchors = manager._payload_takeup_positions()
    payload = manager.latest_payload_state
    attachment = payload['position'] + (payload['rotation'] @ manager.slung_controller.config.attachment_points_m).T
    relative = anchors - attachment
    initial = attachment + relative * (physical / np.linalg.norm(relative,axis=1))[:,None]
finally:
    manager.logger.close()
overrides = {'slung_payload': {'flight_validation_enabled': False,
    'link_calibration_enabled': False}, 'slung_simulation': {'post_handoff_s': 60.,
    'physical_link_lengths_m': physical.tolist(), 'initial_vehicle_positions_m': initial.tolist()}}
summary = replay.run_simulation(directory/'smoke', overrides, time_limit_s=.1)
assert np.max(np.abs(summary['initial_physical_cable_errors_m'])) <= 1e-5
np.testing.assert_allclose(summary['initial_vehicle_positions_m'], initial, atol=1e-12)
snapshot = __import__('yaml').safe_load((directory/'smoke/input_parameters.yaml').read_text())
np.testing.assert_allclose(snapshot['simulation_runtime']['initial_vehicle_positions_m'], initial)
(directory/'expected.json').write_text(json.dumps({'positions': initial.tolist(), 'physical': physical.tolist()}))
'''
    environment = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    result = subprocess.run([str(interpreter), "-c", script, str(tmp_path)],
                            cwd=Path(replay.__file__).parent, env=environment,
                            text=True, capture_output=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    expected = json.loads((tmp_path / "expected.json").read_text())
    with (tmp_path / "smoke/simulation_trace.csv").open() as stream:
        first = list(csv.DictReader(stream))[:3]
    for index, row in enumerate(first):
        assert float(row["control_time_s"]) == 0
        np.testing.assert_allclose([float(row["position_" + axis]) for axis in "xyz"], expected["positions"][index])
        assert abs(float(row["physical_cable_length_m"]) - expected["physical"][index]) <= 1e-5
        assert float(row["physical_cable_limit_m"]) == expected["physical"][index]
        assert float(row["payload_position_z"]) == float(row["payload_height_m"])
