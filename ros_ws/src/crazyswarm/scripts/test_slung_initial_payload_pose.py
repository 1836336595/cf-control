"""Replay tilt must change the physical box and its cable attachment sites."""

import csv
import json
import os
from pathlib import Path
import subprocess

import numpy as np
import pytest

import simulate_slung_handoff as replay


def test_initial_payload_rpy_defaults_to_level():
    assert replay.resolve_simulation_settings({}).get("initial_payload_rpy_deg") == [0., 0., 0.]


@pytest.mark.parametrize("rpy", [None, [], [20.], [0., 20.], [0., 20., 0., 0.],
                                      [0., np.nan, 0.], [0., np.inf, 0.], "bad"])
def test_initial_payload_rpy_rejects_invalid_attitudes(rpy):
    with pytest.raises(ValueError, match="initial_payload_rpy_deg"):
        replay.resolve_simulation_settings({"simulation": {"initial_payload_rpy_deg": rpy}})


@pytest.mark.parametrize("custom_anchors", [False, True])
def test_payload_tilt_sets_physics_and_ground_clearance_without_moving_custom_anchors(tmp_path, custom_anchors):
    interpreter = Path("/home/csy/miniconda3/envs/udaan/bin/python")
    if not interpreter.exists():
        pytest.skip("Udaan MuJoCo interpreter unavailable")
    script = r'''
import json
from pathlib import Path
import sys
import simulate_slung_handoff as replay
directory = Path(sys.argv[1])
simulation = {'initial_payload_rpy_deg': [6., 20., -10.]}
if sys.argv[2] == 'True':
    simulation['initial_vehicle_positions_m'] = [[.4, 0., .78], [-.2, .3, .8], [-.2, -.3, .79]]
summary = replay.run_simulation(directory, {'slung_payload': {
    'flight_validation_enabled': False, 'link_calibration_enabled': False,
    'mass_kg': .023, 'link_lengths_m': [.85, .85, .85], 'simulation': simulation}},
    time_limit_s=.05)
'''
    environment = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    result = subprocess.run([str(interpreter), "-c", script, str(tmp_path), str(custom_anchors)],
                            cwd=Path(replay.__file__).parent, env=environment,
                            text=True, capture_output=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    summary = json.loads((tmp_path / "summary.json").read_text())
    with (tmp_path / "simulation_trace.csv").open() as stream:
        rows = list(csv.DictReader(stream))[:3]
    np.testing.assert_allclose([float(rows[0][name]) for name in
        ("payload_roll_rad", "payload_pitch_rad", "payload_yaw_rad")], np.deg2rad([6., 20., -10.]), atol=1e-12)

    # Independent Euler conversion and all eight actual box corners check both
    # quaternion convention and the tilted box's initial contact with ground.
    from scipy.spatial.transform import Rotation
    rotation = Rotation.from_euler("xyz", [6., 20., -10.], degrees=True).as_matrix()
    center = np.array([float(rows[0]["payload_position_" + axis]) for axis in "xyz"])
    corners = np.array([[x, y, z] for x in (-.04, .04) for y in (-.03, .03)
                        for z in (-.025, .025)]) @ rotation.T + center
    assert abs(np.min(corners[:, 2])) < 1e-12
    assert float(rows[0]["payload_clearance_m"]) == pytest.approx(0., abs=1e-12)
    top = center + rotation @ [0., 0., .025]
    np.testing.assert_allclose(summary["initial_payload_rpy_deg"], [6., 20., -10.])
    np.testing.assert_allclose(summary["initial_payload_rotation"], rotation, atol=1e-12)
    np.testing.assert_allclose(summary["initial_payload_center_m"], center, atol=1e-12)
    np.testing.assert_allclose(summary["initial_payload_top_m"], top, atol=1e-12)
    assert summary["initial_payload_ground_clearance_m"] == pytest.approx(0., abs=1e-12)
    attachment = np.array([[.04, 0., .025], [-.04, .03, .025], [-.04, -.03, .025]]) @ rotation.T + center
    positions = np.array([[float(row["position_" + axis]) for axis in "xyz"] for row in rows])
    np.testing.assert_allclose(summary["initial_payload_attachment_positions_m"], attachment, atol=1e-12)
    np.testing.assert_allclose(summary["initial_physical_cable_lengths_m"],
                               np.linalg.norm(positions - attachment, axis=1), atol=1e-12)
    if custom_anchors:
        np.testing.assert_allclose(positions, [[.4, 0., .78], [-.2, .3, .8], [-.2, -.3, .79]], atol=1e-12)
    else:
        np.testing.assert_allclose(summary["initial_physical_cable_errors_m"], 0., atol=1e-9)
    import yaml
    runtime = yaml.safe_load((tmp_path / "input_parameters.yaml").read_text())["simulation_runtime"]
    for name in ("initial_payload_rpy_deg", "initial_payload_rotation", "initial_payload_center_m",
                 "initial_payload_top_m", "initial_payload_attachment_positions_m"):
        np.testing.assert_allclose(runtime[name], summary[name], atol=1e-12)
