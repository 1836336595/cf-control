"""Compare the pure controller split with captured complete pre-split commands."""

import importlib
import gzip
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


def _arrays(value):
    if isinstance(value, list):
        return np.asarray(value, dtype=float)
    if isinstance(value, dict):
        return {key: _arrays(item) for key, item in value.items()}
    return value


def _equal(actual, expected, path):
    if isinstance(expected, dict):
        assert set(actual) == set(expected), path
        for name, value in expected.items():
            _equal(actual[name], value, path+"."+name)
    elif isinstance(expected, list):
        # The snapshot was captured with NumPy 1.x. NumPy 2.x changes the
        # final few rounding bits of norms/cross products (measured maximum
        # difference 3.6e-15), without changing the control law. Keep a machine
        # precision budget rather than requiring cross-version bit identity.
        epsilon = 32. * np.finfo(float).eps
        np.testing.assert_allclose(actual, expected, rtol=epsilon, atol=epsilon, err_msg=path)
    elif isinstance(expected, float):
        epsilon = 32. * np.finfo(float).eps
        np.testing.assert_allclose(actual, expected, rtol=epsilon, atol=epsilon, err_msg=path)
    else:
        assert actual == expected, path


@pytest.mark.parametrize("case_index", [0, 1])
def test_multiphase_commands_and_persistent_state_match_captured_baseline(case_index):
    module = importlib.import_module("ctbr_geometric_controller")
    cases = json.loads(gzip.decompress(Path(__file__).with_name("ctbr_pure_controller_golden.json.gz").read_bytes()))
    case = cases[case_index]
    controller = module.GeometricCtbrController(module.ControllerConfig(**_arrays(case["config"])))
    for tick, step in enumerate(case["steps"]):
        inputs = _arrays(step["inputs"])
        if inputs["reset"]:
            controller.reset()
        actual = controller.compute(inputs["state"], inputs["target"], inputs["dt"])
        _equal(actual, step["expected"], "tick%d.command" % tick)
        for name, expected in step["state"].items():
            _equal(getattr(controller, name), expected, "tick%d.%s" % (tick, name))


def test_pure_controller_imports_and_executes_without_ros_packages():
    script = """
import builtins
original_import = builtins.__import__
def import_without_ros(name, *args, **kwargs):
    if name.split('.')[0] in ('rospy', 'roslib', 'geometry_msgs', 'sensor_msgs', 'std_msgs'):
        raise ImportError('ROS is deliberately unavailable')
    return original_import(name, *args, **kwargs)
builtins.__import__ = import_without_ros
from test_ctbr_pure_controller_split import test_multiphase_commands_and_persistent_state_match_captured_baseline
test_multiphase_commands_and_persistent_state_match_captured_baseline(0)
"""
    subprocess.run([sys.executable, "-c", script], cwd=str(Path(__file__).parent), check=True)
