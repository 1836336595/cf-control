"""Explicit payload inertia must reach gains, rigid-body moments and physics."""

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

from test_ctbr_controller_v2 import _controller_module, _slung_manager
from test_slung_attitude_feedforward import _case


def test_yaml_inertia_drives_gains_independently_of_mass_and_size(tmp_path):
    module = _controller_module()
    j = np.array([2e-5, 3e-5, 4e-5])
    manager, _ = _slung_manager(module, tmp_path, {'slung_payload': {
        'inertia_diag_kgm2': j.tolist(), 'mass_kg': .04, 'size_m': [.1, .07, .06]}})
    try:
        np.testing.assert_array_equal(manager.slung_controller.config.payload_inertia_kgm2, j)
        omega = 2 * np.pi * np.array([1.5, 1.5, .7])
        np.testing.assert_allclose(manager.payload_attitude_gain, j * omega**2)
        np.testing.assert_allclose(manager.payload_rate_gain, 2 * .9 * omega * j)
    finally:
        manager.logger.close()


@pytest.mark.parametrize('bad', [None, [], [1e-5, 2e-5], [[1e-5,2e-5,3e-5]],
    [0., 2e-5, 3e-5], [-1e-5, 2e-5, 3e-5], [np.nan, 2e-5, 3e-5],
    [np.inf, 2e-5, 3e-5], [True, 2e-5, 3e-5], [1e-5,1e-5,9e-5]])
def test_invalid_inertia_fails_at_parameter_loading(tmp_path, bad):
    module = _controller_module()
    with pytest.raises(RuntimeError, match='inertia'):
        _slung_manager(module, tmp_path, {'slung_payload': {'inertia_diag_kgm2': bad}})


def test_feedforward_uses_supplied_inertia_and_does_not_recalculate_from_mass():
    controller, payload, vehicles, target = _case()
    j = np.array([2e-5, 3e-5, 4e-5])
    cfg = replace(controller.config, payload_inertia_kgm2=j,
                  payload_mass_kg=.04, payload_size_m=np.array([.2,.15,.1]))
    from slung_load_controller import SlungLoadController
    target['body_rate_dot'] = np.array([.3,-.2,.1])
    omega = payload['body_rate']
    result = SlungLoadController(cfg).compute(payload, vehicles, target, .01)
    np.testing.assert_allclose(result['desired_moment'],
                              np.cross(omega, j*omega)+j*target['body_rate_dot'], atol=1e-16)


def test_simulation_model_receives_exact_host_inertia(tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parents[5] / 'simulate_uav/udaan'))
    from udaan.crazyswarm_slung_parameters import default_config
    from simulate_slung_handoff import build_physics_config
    module = _controller_module()
    j = [2e-5,3e-5,4e-5]
    manager, _ = _slung_manager(module, tmp_path, {'slung_payload': {'inertia_diag_kgm2': j}})
    try:
        config = build_physics_config(default_config(), manager, {})
        np.testing.assert_array_equal(config.payload_inertia, j)
        np.testing.assert_array_equal(replace(config, payload_mass=.1, payload_size=(.3,.2,.1)).payload_inertia,j)
        import yaml
        assert yaml.safe_load(yaml.safe_dump(list(config.payload_inertia_kgm2))) == j
    finally:
        manager.logger.close()


def test_missing_yaml_inertia_is_not_silently_derived_from_dimensions(tmp_path):
    module = _controller_module()
    manager, _ = _slung_manager(module, tmp_path)
    try:
        original = module.rospy.get_param
        def missing_inertia(name, *args):
            value = original(name, *args)
            if isinstance(value, dict) and 'inertia_diag_kgm2' in value:
                value = dict(value)
                del value['inertia_diag_kgm2']
            return value
        module.rospy.get_param = missing_inertia
        with pytest.raises(RuntimeError, match='inertia_diag_kgm2'):
            manager._initialize_payload_parameters()
    finally:
        manager.logger.close()


def test_actual_mujoco_body_uses_the_configured_inertia(tmp_path):
    mujoco = pytest.importorskip('mujoco')
    sys.path.insert(0, str(Path(__file__).resolve().parents[5] / 'simulate_uav/udaan'))
    from udaan.crazyswarm_slung_parameters import default_config
    from udaan.models.mujoco.crazyswarm_slung_scene import build_mjcf
    from simulate_slung_handoff import build_physics_config
    module = _controller_module()
    j = [2e-5,3e-5,4e-5]
    manager, _ = _slung_manager(module, tmp_path, {'slung_payload': {'inertia_diag_kgm2': j}})
    try:
        config = build_physics_config(default_config(), manager, {})
        model = mujoco.MjModel.from_xml_string(build_mjcf(config))
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'payload')
        np.testing.assert_allclose(model.body_inertia[body], j, rtol=1e-9)
    finally:
        manager.logger.close()
