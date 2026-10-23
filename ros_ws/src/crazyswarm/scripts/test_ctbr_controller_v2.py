#!/usr/bin/env python3
"""Behavioral tests for the MATLAB V2 analytic Omega_c migration.

The production controller imports ROS message types at module import time.  The
tests replace only those external transport classes so the real, ROS-independent
geometric control law can be exercised without a ROS installation.
"""

import importlib.util
import math
import sys
import tempfile
import threading
import types
from collections import deque
from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
CONTROLLER_CONFIG_PATH = SCRIPT_DIR.parent / "config" / "ctbr_controller.yaml"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


def _install_ros_import_stubs():
    """Install only the message symbols needed to import the control module."""
    rospy = types.ModuleType("rospy")
    rospy.ROSInitException = RuntimeError
    rospy.logwarn_throttle = lambda *_args, **_kwargs: None
    rospy.Time = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_sec=lambda: 0.0)
    )
    sys.modules["rospy"] = rospy

    crazyswarm = types.ModuleType("crazyswarm")
    crazyswarm_msg = types.ModuleType("crazyswarm.msg")
    for name in ("CTBR", "CTBRTiming", "GenericLogData", "MocapState"):
        setattr(crazyswarm_msg, name, type(name, (), {}))
    crazyswarm.msg = crazyswarm_msg
    sys.modules["crazyswarm"] = crazyswarm
    sys.modules["crazyswarm.msg"] = crazyswarm_msg

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = type("PoseStamped", (), {})
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    class PathMessage:
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="", stamp=None)
            self.poses = []

    nav_msgs_msg.Path = PathMessage
    nav_msgs.msg = nav_msgs_msg
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.UInt16 = type("UInt16", (), {})
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg


def _controller_module():
    _install_ros_import_stubs()
    # Each fixture installs fresh ROS transport stubs; reload their consumers.
    for name in ("ctbr_fleet_node", "ctbr_vehicle_node", "ctbr_vehicle_logging",
                 "ctbr_vehicle_state", "ctbr_vehicle_preflight", "ctbr_payload_parameters",
                 "ctbr_payload_state", "ctbr_payload_takeup", "ctbr_payload_reference",
                 "ctbr_payload_control", "ctbr_payload_mission"):
        sys.modules.pop(name, None)
    module_name = "ctbr_controller_v2_under_test"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(
        module_name, SCRIPT_DIR / "ctbr_controller.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def test_vehicle_parameter_namespace_uses_cf_id_suffix():
    """Changing a CF ID must select only its matching calibration root."""
    module = _controller_module()

    assert module.vehicle_parameter_namespace(2) == "/ctbr_controller_cf2"
    assert module.vehicle_parameter_namespace(4) == "/ctbr_controller_cf4"


def test_parameter_resolver_keeps_global_trajectory_and_cf_calibration_separate():
    """A CF4 read must not silently fall back to CF2 or shared calibration."""
    module = _controller_module()
    values = {
        "/ctbr_controller_cf2": {},
        "/ctbr_controller_cf4": {},
        "/ctbr_controller/control_rate_hz": 60.0,
        "/ctbr_trajectory/figure_eight_radius_m": 0.8,
        "/ctbr_controller_cf2/mass_kg": 0.0434,
        "/ctbr_controller_cf4/mass_kg": 0.0460,
        "/ctbr_controller_cf4/position_gain": [0.8, 0.8, 0.5],
    }
    resolver = module.CtbrParameterResolver(
        lambda name, default=None: values.get(name, default),
        lambda name: name in values,
        vehicle_id=4,
    )

    assert resolver.global_param("control_rate_hz") == 60.0
    assert resolver.trajectory_param("figure_eight_radius_m") == 0.8
    assert resolver.vehicle_param("mass_kg") == 0.0460
    assert resolver.vehicle_param("position_gain") == [0.8, 0.8, 0.5]


def test_parameter_resolver_rejects_missing_vehicle_calibration_block():
    """A newly enabled CF may not fly with another vehicle's tuning."""
    module = _controller_module()

    try:
        module.CtbrParameterResolver(lambda *_args: None, lambda _name: False, 5)
    except ValueError as error:
        assert "CF5" in str(error)
        assert "/ctbr_controller_cf5" in str(error)
    else:
        raise AssertionError("missing CF5 parameter block must be rejected")


class _FakeParameterServer:
    """Minimal ROS parameter-server boundary with only absolute YAML paths."""

    _MISSING = object()

    def __init__(self, roots):
        self.values = {}
        for root, mapping in roots.items():
            root_path = "/" + root
            self.values[root_path] = mapping
            if isinstance(mapping, dict):
                for name, value in mapping.items():
                    self.values[root_path + "/" + name] = value

    def has_param(self, name):
        return name in self.values

    def get_param(self, name, default=_MISSING):
        if name in self.values:
            return self.values[name]
        if default is self._MISSING:
            raise KeyError(name)
        return default


def _configure_controller_node_ros(module, roots):
    """Keep production parameter routing real while replacing ROS transport only."""
    server = _FakeParameterServer(roots)
    module.rospy.get_param = server.get_param
    module.rospy.has_param = server.has_param
    module.rospy.Publisher = lambda *_args, **_kwargs: types.SimpleNamespace(
        publish=lambda *_publish_args, **_publish_kwargs: None
    )
    module.rospy.Subscriber = lambda *_args, **_kwargs: types.SimpleNamespace()
    module.rospy.loginfo = lambda *_args, **_kwargs: None
    module.rospy.logwarn = lambda *_args, **_kwargs: None
    module.rospy.on_shutdown = lambda *_args, **_kwargs: None
    return server


def _controller_parameter_roots():
    roots = yaml.safe_load(CONTROLLER_CONFIG_PATH.read_text())
    roots.update(yaml.safe_load(
        (CONTROLLER_CONFIG_PATH.parent / "ctbr_vehicle.yaml").read_text()
    ))
    roots["ctbr_controller"]["target_confirmed"] = True
    return roots


def _node_logger():
    return types.SimpleNamespace(path="/tmp/ctbr-test.csv", close=lambda: None)


def test_controller_node_reads_cf_specific_calibration_from_absolute_namespace():
    """Leaving one private ~mass read would reject the new YAML layout."""
    module = _controller_module()
    roots = _controller_parameter_roots()
    roots["ctbr_controller_cf4"]["mass_kg"] = 0.0460
    roots["ctbr_controller_cf4"]["position_gain"] = [0.8, 0.8, 0.5]
    _configure_controller_node_ros(module, roots)

    cf2 = module.CtbrControllerNode(
        vehicle_config={"id": 3, "uri": "radio://0/80/2M/E7E7E7E703"},
        logger=_node_logger(), auto_timer=False, register_shutdown=False,
    )
    cf4 = module.CtbrControllerNode(
        vehicle_config={"id": 4, "uri": "radio://0/80/2M/E7E7E7E704"},
        logger=_node_logger(), auto_timer=False, register_shutdown=False,
    )

    assert math.isclose(cf2.controller.config.mass, 0.0434)
    assert math.isclose(cf4.controller.config.mass, 0.0460)
    assert np.allclose(cf4.controller.config.position_gain, [0.8, 0.8, 0.5])
    expected_rate = float(roots["ctbr_controller"]["control_rate_hz"])
    assert cf2.rate_hz == cf4.rate_hz == expected_rate
    assert cf2.trajectory_config.trajectory_mode == "hover"
    assert cf4.trajectory_config.trajectory_mode == "hover"
    assert math.isclose(
        cf2.takeoff_position_gain_scale,
        float(roots["ctbr_controller"]["takeoff_position_gain_scale"]),
    )
    assert math.isclose(
        cf4.takeoff_velocity_gain_scale,
        float(roots["ctbr_controller"]["takeoff_velocity_gain_scale"]),
    )


def test_controller_node_rejects_missing_cf_specific_parameter_block():
    """An enabled CF5 must fail closed when its calibration map is absent."""
    module = _controller_module()
    roots = _controller_parameter_roots()
    del roots["ctbr_controller_cf5"]
    _configure_controller_node_ros(module, roots)

    try:
        module.CtbrControllerNode(
            vehicle_config={"id": 5, "uri": "radio://0/80/2M/E7E7E7E705"},
            logger=_node_logger(), auto_timer=False, register_shutdown=False,
        )
    except RuntimeError as error:
        assert "CF5" in str(error)
        assert "/ctbr_controller_cf5" in str(error)
    else:
        raise AssertionError("CF5 without a calibration block must be rejected")


def _config(module, **overrides):
    values = {
        "mass": 0.0434,
        "gravity": 9.80665,
        "max_total_thrust": 1.2,
        "max_command_thrust": 1.0,
        "max_tilt_rad": math.radians(80.0),
        "max_body_rate": np.array([10.0, 10.0, 10.0]),
        "position_gain": np.array([0.75, 0.75, 0.45]),
        "velocity_gain": np.array([0.45, 0.45, 0.35]),
        "integral_gain": np.array([0.05, 0.05, 0.02]),
        "integral_limit": np.array([0.8, 0.8, 0.8]),
        "attitude_gain": np.array([3.0, 3.0, 2.0]),
        "attitude_integral_gain": np.zeros(3),
        "attitude_integral_limit": np.array([0.8, 0.8, 0.8]),
        "position_integral_c1": 0.35,
        "use_body_rate_feedforward": True,
        "omega_c_method": "analytic",
        "omega_c_force_norm_epsilon": 1e-8,
        "omega_c_heading_projection_epsilon": 1e-6,
    }
    values.update(overrides)
    return module.ControllerConfig(**values)


def _state(acceleration=None, derivatives_valid=True, rotation=None):
    return {
        "position": np.zeros(3),
        "velocity": np.zeros(3),
        "acceleration": (
            np.zeros(3) if acceleration is None else np.asarray(acceleration, dtype=float)
        ),
        "derivatives_valid": derivatives_valid,
        "rotation": np.eye(3) if rotation is None else np.asarray(rotation, dtype=float),
    }


def _target(acceleration=None, jerk=None, yaw=0.0, yaw_rate=0.0):
    return {
        "position": np.zeros(3),
        "velocity": np.zeros(3),
        "acceleration": (
            np.zeros(3) if acceleration is None else np.asarray(acceleration, dtype=float)
        ),
        "jerk": np.zeros(3) if jerk is None else np.asarray(jerk, dtype=float),
        "yaw": float(yaw),
        "yaw_rate": float(yaw_rate),
    }


def test_normal_trajectory_phase_transitions_preserve_position_integral():
    module = _controller_module()

    for phase in (
            "takeoff", "height_correction", "circle_entry", "circle",
            "final_hover", "landing", "landing_settle"):
        assert not module.phase_requires_full_controller_reset(phase)
    for phase in ("emergency_landing", "landed"):
        assert module.phase_requires_full_controller_reset(phase)


def test_analytic_omega_c_forwards_target_yaw_rate_without_attitude_error():
    """A missing b1 reference derivative would incorrectly produce zero yaw rate."""
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))

    command = controller.compute(
        _state(), _target(yaw=0.0, yaw_rate=0.6), 0.01
    )

    assert np.allclose(command["computed_body_rate"], [0.0, 0.0, 0.6], atol=1e-10)
    assert np.allclose(command["body_rate_command"], [0.0, 0.0, 0.6], atol=1e-10)


def test_transport_force_override_bypasses_single_vehicle_position_pid():
    """MATLAB transport force must not be changed by the independent PID terms."""
    module = _controller_module()
    config = _config(module)
    controller = module.GeometricCtbrController(config)
    state = _state()
    state["position"] = np.array([1.0, -0.5, 0.2])
    state["filter_derivatives_valid"] = True
    state["filtered_velocity"] = np.zeros(3)
    state["filtered_acceleration"] = np.zeros(3)
    target = _target()
    target.update({
        "transport_mode": True,
        "desired_force_override": np.array([0.0, 0.0, config.mass * config.gravity]),
        "desired_force_dot_override": np.zeros(3),
        "transport_attitude_gain": np.array([240.0, 240.0, 120.0]),
        "transport_rate_gain": np.array([4.0, 4.0, 4.0]),
    })
    command = controller.compute(state, target, 0.01)
    assert np.allclose(command["desired_force"], [0.0, 0.0, config.mass * config.gravity])
    assert np.allclose(command["position_integral"], np.zeros(3))
    assert command["transport_mode"] is True


def test_transport_body_rate_filter_smooths_a_step_and_resets_at_handoff():
    """The optional transport command filter damps a step without filtering thrust."""
    module = _controller_module()
    config = _config(module, transport_body_rate_filter_tau_s=0.02)
    controller = module.GeometricCtbrController(config)
    state = _state()
    state["body_rate"] = np.zeros(3)
    target = _target()
    target.update(
        transport_mode=True,
        transport_blend=1.0,
        desired_force_override=np.array([0.0, 0.0, 0.8]),
        desired_force_dot_override=np.zeros(3),
        transport_attitude_gain=np.array([12.0, 12.0, 6.0]),
        transport_rate_gain=np.array([1.0, 1.0, 0.5]),
    )
    first = controller.compute(state, target, 1 / 70)
    angle = math.radians(20.0)
    state["rotation"] = np.array([
        [math.cos(angle), 0.0, math.sin(angle)],
        [0.0, 1.0, 0.0],
        [-math.sin(angle), 0.0, math.cos(angle)],
    ])
    second = controller.compute(state, target, 1 / 70)
    assert abs(second["body_rate_command"][1]) < abs(second["raw_body_rate_command"][1])
    assert np.allclose(second["desired_force"], [0.0, 0.0, 0.8])
    target["transport_filter_reset"] = True
    reset = controller.compute(state, target, 1 / 70)
    assert np.allclose(reset["body_rate_command"], reset["raw_body_rate_command"])


def test_transport_force_filter_keeps_force_and_attitude_direction_consistent():
    module = _controller_module()
    config = _config(module, transport_force_filter_tau_s=0.02)
    controller = module.GeometricCtbrController(config)
    state = _state()
    target = _target()
    target.update(
        transport_mode=True, transport_blend=1.0,
        desired_force_override=np.array([0.0, 0.0, 0.8]),
        desired_force_dot_override=np.zeros(3),
        transport_attitude_gain=np.array([6.0, 6.0, 3.0]),
        transport_rate_gain=np.array([2.0, 2.0, 1.0]),
        transport_filter_reset=True,
    )
    first = controller.compute(state, target, 1 / 70)
    target["transport_filter_reset"] = False
    target["desired_force_override"] = np.array([0.18, 0.0, 0.8])
    second = controller.compute(state, target, 1 / 70)
    assert 0.0 < second["desired_force"][0] < 0.18
    assert np.isclose(second["collective_thrust"], second["desired_force"][2], atol=1e-12)
    assert not np.allclose(first["desired_force"], second["desired_force"])


def test_tension_ramp_keeps_measured_vehicle_anchor_with_independent_hold_feedback():
    """Udaan-style ramp hold counters inward cable-force motion during handoff."""
    module = _controller_module()
    config = _config(
        module,
        independent_position_gain=np.array([3.0, 3.0, 4.0]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.60]),
        independent_max_feedback_acceleration=np.array([6.0, 6.0, 8.0]),
    )
    state = _state()
    state["position"] = np.array([0.10, 0.0, 0.0])
    state["filter_derivatives_valid"] = True
    state["filtered_velocity"] = np.zeros(3)
    state["filtered_acceleration"] = np.zeros(3)

    def run(hold_scale):
        controller = module.GeometricCtbrController(config)
        target = _target()
        target.update({
            "position": np.zeros(3),
            "transport_mode": True,
            "transport_blend": 0.5,
            "desired_force_override": np.array([0.0, 0.0, config.mass * config.gravity]),
            "desired_force_dot_override": np.zeros(3),
            "ramp_position_hold_gain_scale": hold_scale,
        })
        return controller.compute(state, target, 0.01)["desired_force"]

    without_hold = run(0.0)
    with_hold = run(2.0)
    expected_delta_x = 0.5 * config.mass * 2.0 * (-3.0 * 0.10)
    assert math.isclose(
        with_hold[0] - without_hold[0], expected_delta_x, rel_tol=0.0, abs_tol=1.0e-10
    )
    assert with_hold[0] < without_hold[0]


def test_transport_lift_keeps_horizontal_anchor_with_independent_hold_feedback():
    """The hold remains active after TENSION_RAMP during reference lift."""
    module = _controller_module()
    config = _config(
        module,
        independent_position_gain=np.array([3.0, 3.0, 4.0]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.60]),
        independent_max_feedback_acceleration=np.array([6.0, 6.0, 8.0]),
    )
    controller = module.GeometricCtbrController(config)
    state = _state()
    state["position"] = np.array([0.10, 0.0, 0.0])
    state["filter_derivatives_valid"] = True
    state["filtered_velocity"] = np.zeros(3)
    state["filtered_acceleration"] = np.zeros(3)
    target = _target()
    target.update({
        "transport_mode": True,
        "transport_blend": 1.0,
        "desired_force_override": np.array([0.0, 0.0, config.mass * config.gravity]),
        "desired_force_dot_override": np.zeros(3),
        "transport_position_hold_gain_scale": 2.0,
    })
    command = controller.compute(state, target, 0.01)
    expected_delta_x = config.mass * 2.0 * (-3.0 * 0.10)
    assert math.isclose(
        command["desired_force"][0], expected_delta_x,
        rel_tol=0.0, abs_tol=1.0e-10
    )


def test_position_hold_force_derivative_tracks_velocity_and_acceleration_errors():
    module = _controller_module()
    config = _config(
        module,
        independent_position_gain=np.array([3.0, 3.0, 4.0]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.60]),
        independent_max_feedback_acceleration=np.array([6.0, 6.0, 8.0]),
    )
    controller = module.GeometricCtbrController(config)
    state = _state(acceleration=np.array([0.3, 0.0, 0.0]))
    state["position"] = np.array([0.10, 0.0, 0.0])
    state["velocity"] = np.array([0.20, 0.0, 0.0])
    target = _target()
    target.update({
        "transport_mode": True,
        "transport_blend": 1.0,
        "desired_force_override": np.array([0.0, 0.0, config.mass * config.gravity]),
        "desired_force_dot_override": np.zeros(3),
        "transport_position_hold_gain_scale": 0.5,
    })

    command = controller.compute(state, target, 0.01)
    expected = config.mass * 0.5 * (-3.0 * 0.20 - 3.12 * 0.30)
    assert math.isclose(
        command["desired_force_dot"][0], expected,
        rel_tol=0.0, abs_tol=1.0e-10
    )


def test_position_hold_force_derivative_includes_the_gain_fade():
    module = _controller_module()
    config = _config(module, independent_position_gain=np.array([3.0, 3.0, 4.0]))
    state = _state()
    state["position"] = np.array([0.10, 0.0, 0.0])
    target = _target()
    target.update({
        "transport_mode": True, "transport_blend": 1.0,
        "desired_force_override": np.array([0.0, 0.0, config.mass * config.gravity]),
        "desired_force_dot_override": np.zeros(3),
        "transport_position_hold_gain_scale": 1.0,
        "transport_position_hold_gain_scale_dot": -0.5,
    })
    command = module.GeometricCtbrController(config).compute(state, target, 0.01)
    assert math.isclose(command["desired_force_dot"][0], 0.00651, abs_tol=1e-10)


def test_matlab_independent_phase_scales_acceleration_feedback_by_mass():
    """SLACK/TAKEUP must match crazyflie_slung_independent_controller's A law."""
    module = _controller_module()
    config = _config(
        module,
        independent_position_gain=np.array([3.0, 3.0, 4.0]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.60]),
        independent_integral_gain=np.array([0.8, 0.8, 0.8]),
        independent_integral_limit=np.array([0.20, 0.20, 0.20]),
        independent_integral_gate=0.05,
        independent_max_feedback_acceleration=np.array([6.0, 6.0, 8.0]),
        independent_max_body_rate=np.array([5.0, 5.0, 3.5]),
        independent_attitude_gain=np.array([240.0, 240.0, 120.0]),
    )
    controller = module.GeometricCtbrController(config)
    state = _state()
    state["position"] = np.array([0.0, 0.0, 0.0])
    target = _target()
    target.update({
        "position": np.array([0.10, 0.0, 0.20]),
        "independent_mode": True,
    })
    command = controller.compute(state, target, 0.01)
    expected = config.mass * np.array([
        config.independent_position_gain[0] * 0.10,
        0.0,
        config.gravity + config.independent_position_gain[2] * 0.20,
    ])
    assert np.allclose(command["desired_force"], expected, atol=1.0e-10)


def test_independent_feedback_scale_can_strengthen_prelift_xy_pid():
    module = _controller_module()
    config = _config(
        module,
        independent_position_gain=np.array([3.0, 3.0, 4.0]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.60]),
    )
    state = _state()
    state["position"] = np.array([0.20, -0.10, 0.0])
    target = _target()
    target.update({
        "independent_mode": True,
        "position": np.zeros(3),
        "independent_position_gain_scale": np.array([3.0, 3.0, 1.0]),
        "independent_velocity_gain_scale": np.array([1.5, 1.5, 1.0]),
    })
    command = module.GeometricCtbrController(config).compute(state, target, 0.01)
    assert np.isclose(command["desired_force"][0], -config.mass * 3.0 * 3.0 * 0.20)
    assert np.isclose(command["desired_force"][1], -config.mass * 3.0 * 3.0 * -0.10)


def _prelift_integral_target():
    target = _target()
    target.update({
        "position": np.array([0.0, 0.0, 1.6]),
        "independent_mode": True,
        "prelift_vertical_integral_active": True,
        "prelift_vertical_integral_limit": 8.0,
        "prelift_vertical_integral_gate_m": 1.65,
        "prelift_vertical_integral_rate_limit": 0.5,
        "prelift_vertical_feedback_limit_mps2": 12.0,
    })
    return target


def _prelift_integral_controller(module):
    return module.GeometricCtbrController(_config(
        module, independent_position_gain=np.array([3.0, 3.0, 4.0]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.6]),
        independent_integral_gain=np.full(3, 0.8),
        independent_integral_limit=np.full(3, 0.2), independent_integral_gate=0.2,
        independent_max_feedback_acceleration=np.array([6.0, 6.0, 8.0]),
    ))


def test_fixed_prelift_reference_keeps_increasing_thrust_with_large_z_error():
    module = _controller_module()
    controller = _prelift_integral_controller(module)
    target = _prelift_integral_target()
    state = _state()
    initial = controller.compute(state, target, 0.01)
    for _ in range(1000):
        command = controller.compute(state, target, 0.01)
    assert command["collective_thrust"] > initial["collective_thrust"] + 0.10
    assert np.allclose(command["position_integral"][:2], 0.0)
    assert command["position_integral"][2] < -0.2
    assert command["collective_thrust"] <= controller.config.max_command_thrust


def test_prelift_integral_stops_growing_at_acceleration_saturation():
    module = _controller_module()
    controller = _prelift_integral_controller(module)
    target = _prelift_integral_target()
    controller.position_integral[2] = -7.0
    initial = controller.position_integral.copy()
    for _ in range(100):
        command = controller.compute(_state(), target, 0.01)
    assert np.allclose(controller.position_integral, initial)
    assert command["collective_thrust"] <= controller.config.mass * (controller.config.gravity + 12.0)


def test_prelift_integral_stops_growing_when_projected_thrust_is_saturated():
    module = _controller_module()
    controller = _prelift_integral_controller(module)
    controller.config.max_command_thrust = 0.70
    target = _prelift_integral_target()
    command = controller.compute(_state(), target, 0.01)
    assert np.allclose(command["position_integral"], 0.0)


def test_prelift_integral_holds_when_velocity_feedback_is_invalid():
    module = _controller_module()
    controller = _prelift_integral_controller(module)
    controller.compute(_state(derivatives_valid=False), _prelift_integral_target(), 0.01)
    assert np.allclose(controller.position_integral, 0.0)


def test_prelift_integral_can_unwind_and_preserves_base_controller_parameters():
    module = _controller_module()
    controller = _prelift_integral_controller(module)
    limit = controller.config.independent_integral_limit.copy()
    acceleration_limit = controller.config.independent_max_feedback_acceleration.copy()
    controller.position_integral[2] = -7.0
    state = _state()
    state["position"] = np.array([0.0, 0.0, 0.1])
    target = _prelift_integral_target()
    target["position"] = np.zeros(3)
    for _ in range(100):
        controller.compute(state, target, 0.01)
    assert -7.0 < controller.position_integral[2] < -6.9
    assert np.array_equal(controller.config.independent_integral_limit, limit)
    assert np.array_equal(controller.config.independent_max_feedback_acceleration, acceleration_limit)


def test_standard_independent_mode_still_uses_its_small_error_integral_gate():
    module = _controller_module()
    controller = _prelift_integral_controller(module)
    target = _target()
    target.update(position=np.array([0.0, 0.0, 1.6]), independent_mode=True)
    for _ in range(100):
        controller.compute(_state(), target, 0.01)
    assert np.allclose(controller.position_integral, 0.0)


def test_prelift_handoff_retains_last_independent_force_at_blend_zero():
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))
    state = _state()
    target = _target()
    target.update({
        "transport_mode": True,
        "transport_blend": 0.0,
        "handoff_force": np.array([0.0, 0.0, 0.65]),
        "desired_force_override": np.array([0.0, 0.0, 0.65]),
        "desired_force_dot_override": np.zeros(3),
    })
    command = controller.compute(state, target, 0.01)
    assert np.allclose(command["desired_force"], [0.0, 0.0, 0.65])


def _merge_parameter_overrides(baseline, overlay):
    """Recursively overlay mappings; copy replaced scalars/lists for isolation."""
    result = deepcopy(baseline)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge_parameter_overrides(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _slung_manager(module, directory, parameter_overrides=None, use_live_payload_trajectory=False):
    """Instantiate real mission/controller objects; replace only ROS transport."""
    roots = _controller_parameter_roots()
    roots.update(yaml.safe_load((CONTROLLER_CONFIG_PATH.parent / "slung_payload.yaml").read_text()))
    # Unit cases declare the legacy stationary mission; simulation explicitly
    # opts into the live configured trajectory before applying its candidate.
    if not use_live_payload_trajectory:
        roots["slung_payload"]["trajectory"] = {"mode": "hover"}
        # Legacy unit cases keep the original post-reference adaptation gate;
        # live replay and explicit feature cases opt into the new early gate.
        roots["slung_payload"]["transport_attitude_trim_dynamic_enabled"] = False
        roots["slung_payload"]["transport_attitude_trim_during_lift_enabled"] = False
        roots["slung_payload"]["flight_validation_transport_s"] = 60.
        roots["ctbr_trajectory"]["hover_duration_s"] = 60.
    flight_overrides = {name: value for name, value in (parameter_overrides or {}).items()
                        if name != "slung_simulation"}
    roots = _merge_parameter_overrides(roots, flight_overrides)
    roots["ctbr_controller"].update(transport_mode="slung_load", log_directory=str(directory))
    roots["crazyflies"] = [
        {"id": i, "channel": 80, "uri": "radio://0/80/2M/E7E7E7E7%02X" % i,
         "ctbr_enabled": True, "initialPosition": [0.0, 0.0, 0.0],
         "orbit_phase_rad": (i - 3) * 2.0 * math.pi / 3.0}
        for i in (3, 4, 5)
    ]
    _configure_controller_node_ros(module, roots)
    module.rospy.Timer = lambda *_args: None
    module.rospy.Duration = lambda duration: duration
    module.rospy.Time.now = lambda: types.SimpleNamespace(to_sec=lambda: 100.0)
    manager = module.MultiCtbrControllerNode()
    payload = dict(_state(), valid=True, body_rate=np.zeros(3),
                   position=np.array([0.0, 0.0, 0.025]), received_time=0.0)
    manager.latest_payload_state = payload
    manager.last_valid_payload_geometry = payload
    positions = module.takeup_vehicle_targets(
        payload["position"], np.eye(3), manager.slung_controller.config.attachment_points_m,
        manager.slung_controller.config.link_lengths_m, 0.12,
    ).T
    for vehicle, position in zip(manager.vehicles, positions):
        vehicle.latest_state = dict(_state(), position=position, valid=True,
                                    body_rate=np.zeros(3), received_time=0.0,
                                    mocap_sample_time_s=100.0)
        vehicle.trajectory = module.CircularFlightTrajectory(vehicle.trajectory_config)
        vehicle.trajectory.reset(position, 0.0, 0.0)
        vehicle.trajectory.phase = "height_correction"
        vehicle.trajectory.phase_start_time = 0.0
    return manager, positions


def test_real_takeup_waits_for_taut_cables_and_then_keeps_pulling(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        # First wait with one slack cable, even after the nominal TAKEUP time.
        manager.vehicles[0].latest_state["position"] = anchors[0] - [0.0, 0.0, 0.2]
        for now in np.arange(0.0, 9.0, 0.02):
            manager.latest_payload_state["received_time"] = now
            for vehicle in manager.vehicles:
                vehicle.latest_state["received_time"] = now
            manager._hold_payload_mission_clock(now)
            manager._payload_takeup_overrides(now)
        assert manager.payload_prelift_reference is None
        manager.vehicles[0].latest_state["position"] = anchors[0].copy()
        for now in np.arange(9.0, 40.0, 0.02):
            manager.latest_payload_state["received_time"] = now
            for vehicle in manager.vehicles:
                vehicle.latest_state["received_time"] = now
            manager._hold_payload_mission_clock(now)
            overrides = manager._payload_takeup_overrides(now)
        assert manager.payload_prelift_reference is not None
        for i, vehicle in enumerate(manager.vehicles):
            assert overrides[id(vehicle)]["position"][2] > anchors[i, 2] + 0.50
            assert vehicle.trajectory.phase == "height_correction"
        assert not manager._payload_takeup_ready(now)
    finally:
        manager.logger.close()


def test_transport_overrides_keep_prelift_bias_and_force_at_handoff(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        manager.payload_transport_active = True
        manager.payload_transport_start_time = 0.0
        manager.payload_transport_start_position = manager.latest_payload_state["position"].copy()
        manager.payload_transport_vehicle_start_positions = anchors.copy()
        manager.payload_handoff_reference_positions = anchors + [0.0, 0.0, 1.0]
        manager.payload_handoff_forces = np.tile([0.0, 0.0, 0.65], (3, 1))
        overrides = manager._transport_overrides(0.0)
        assert len(overrides) == 3
        for i, vehicle in enumerate(manager.vehicles):
            target = overrides[id(vehicle)]
            assert target["independent_mode"] is False
            assert np.allclose(target["position"], anchors[i] + [0.0, 0.0, 1.0])
            assert np.allclose(target["handoff_force"], [0.0, 0.0, 0.65])
            assert np.allclose(target["position_hold_position"], anchors[i])
    finally:
        manager.logger.close()


def test_handoff_rechecks_cables_even_after_takeup_was_latched(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        raised = anchors + [0.0, 0.0, 0.22]
        manager.latest_payload_state["position"] += [0.0, 0.0, 0.22]
        manager.latest_payload_state["received_time"] = 2.2
        for vehicle, position in zip(manager.vehicles, raised):
            vehicle.latest_state["position"] = position.copy()
        manager.payload_takeup_start_time = 0.0
        manager.payload_takeup_target_positions = raised.copy()
        manager.payload_takeup_taut_latched = True
        manager.payload_prelift_confirm_s = manager.payload_takeup_confirm_s = 0.0
        manager.payload_prelift_reference = module.SlungPreliftTrajectory()
        manager.payload_prelift_reference.start(raised, raised, 0.0)
        for now in np.arange(0.0, 2.21, 0.02):
            manager.payload_prelift_reference.update(raised, clearance_m=0.22, now=now)
        assert manager._payload_takeup_ready(2.2)
        manager.vehicles[0].latest_state["position"] -= [0.0, 0.0, 0.2]
        assert not manager._payload_takeup_ready(2.2)
        assert not manager.payload_prelift_ready
    finally:
        manager.logger.close()


def test_prelift_limit_routes_all_vehicles_into_controlled_landing(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        manager.payload_prelift_parameters["max_lead_m"] = 0.10
        for now in np.arange(0.0, 40.0, 0.02):
            manager.latest_payload_state["received_time"] = now
            for vehicle in manager.vehicles:
                vehicle.latest_state["received_time"] = now
            manager._hold_payload_mission_clock(now)
            overrides = manager._payload_takeup_overrides(now)
            if manager.payload_last_takeup_reference["failed"]:
                break
        assert manager.payload_last_takeup_reference["reason"] == "max_lead"
        assert overrides == {}
        assert all(vehicle.trajectory.phase == "emergency_landing"
                   for vehicle in manager.vehicles)
        assert all(not vehicle._target_for(vehicle.latest_state, now).get("zero_output", False)
                   for vehicle in manager.vehicles)
    finally:
        manager.logger.close()


def test_takeup_csv_reports_actual_payload_health_and_landing_reason(tmp_path):
    module = _controller_module()
    manager, anchors = _slung_manager(module, tmp_path)
    try:
        vehicle = manager.vehicles[0]
        payload = dict(manager.latest_payload_state, received_time=0.99)
        vehicle.set_payload_diagnostics(payload)
        state = dict(vehicle.latest_state, filtered_velocity=np.zeros(3),
                     filtered_acceleration=np.zeros(3))
        target = _target()
        target.update(position=anchors[0], flight_phase="takeup", independent_mode=True)
        command = vehicle.controller.compute(state, target, 0.01)
        vehicle._write_log(1.0, state, target, command)
        vehicle.trajectory.begin_emergency_landing(
            1.01, anchors[0], 0.0, "payload_load: 预抬参考达到边界：max_lead"
        )
        vehicle.set_payload_diagnostics(dict(payload, received_time=-1.0))
        target = vehicle._target_for(state, 1.01)
        command = vehicle.controller.compute(state, target, 0.01)
        vehicle._write_log(1.01, state, target, command)
        manager.logger.flush()
        with open(manager.logger.path, newline="") as stream:
            import csv
            rows = list(csv.DictReader(stream))
        assert rows[0]["payload_state_valid"] == "1"
        assert math.isclose(float(rows[0]["payload_state_age_s"]), 0.01)
        assert rows[1]["payload_state_valid"] == "0"
        assert rows[1]["emergency_landing_reason"].endswith("max_lead")
    finally:
        manager.logger.close()


def test_independent_real_vehicle_attitude_gains_are_rate_loop_safe():
    """Real rate loops must not saturate on a small independent-phase error."""
    vehicle_config = yaml.safe_load(
        (CONTROLLER_CONFIG_PATH.parent / "ctbr_vehicle.yaml").read_text()
    )
    for name in ("ctbr_controller_cf3", "ctbr_controller_cf4", "ctbr_controller_cf5"):
        gain = np.asarray(vehicle_config[name]["independent_attitude_gain"], dtype=float)
        limit = np.asarray(
            vehicle_config[name]["independent_max_body_rate_radps"], dtype=float
        )
        # Keep MATLAB's 2:2:1 axis ratio while scaling kR for the real inner loop.
        assert np.allclose(gain, [12.0, 12.0, 6.0])
        assert np.allclose(limit, [3.0, 3.0, 2.0])
        assert np.all(np.abs(gain * 0.1) < limit)


def test_independent_takeup_integral_gate_covers_observed_tracking_error():
    """TAKEUP errors around 0.1-0.17 m must be allowed to remove thrust bias."""
    vehicle_config = yaml.safe_load(
        (CONTROLLER_CONFIG_PATH.parent / "ctbr_vehicle.yaml").read_text()
    )
    for name in ("ctbr_controller_cf3", "ctbr_controller_cf4", "ctbr_controller_cf5"):
        assert vehicle_config[name]["independent_integral_gate"] >= 0.20


def test_independent_takeoff_preserves_each_vehicle_start_yaw():
    """A fixed world heading must not spin a vehicle during vertical takeoff.

    The MATLAB simulation starts every vehicle at yaw zero, but a real fleet
    may be armed with different initial yaws.  The independent geometric PID
    should use the trajectory's start yaw in that case; otherwise the yaw
    rate saturates and couples into roll/pitch before the vehicle has lifted.
    """
    module = _controller_module()
    yaw = -0.85
    config = _config(
        module,
        independent_position_gain=np.array([3.0, 3.0, 4.0]),
        independent_velocity_gain=np.array([3.12, 3.12, 3.60]),
        independent_integral_gain=np.array([0.8, 0.8, 0.8]),
        independent_integral_limit=np.array([0.20, 0.20, 0.20]),
        independent_integral_gate=0.05,
        independent_max_feedback_acceleration=np.array([6.0, 6.0, 8.0]),
        independent_max_body_rate=np.array([5.0, 5.0, 3.5]),
        independent_attitude_gain=np.array([240.0, 240.0, 120.0]),
        independent_heading=np.array([1.0, 0.0, 0.0]),
    )
    rotation = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    controller = module.GeometricCtbrController(config)
    state = _state(rotation=rotation)
    target = _target(yaw=yaw)
    target["independent_mode"] = True

    command = controller.compute(state, target, 0.01)

    assert np.allclose(command["attitude_error"], np.zeros(3), atol=1.0e-10)
    assert np.allclose(command["body_rate_command"], np.zeros(3), atol=1.0e-10)


def _computed_rotation_at(module, acceleration, jerk, yaw):
    controller = module.GeometricCtbrController(_config(module))
    command = controller.compute(
        _state(acceleration=acceleration),
        _target(acceleration=acceleration, jerk=jerk, yaw=yaw),
        0.01,
    )
    return command["computed_rotation"]


def test_analytic_omega_c_matches_numerical_derivative_of_computed_rotation():
    """A wrong z-up sign in b3_dot disagrees with the actual Rc trajectory."""
    module = _controller_module()
    acceleration = np.array([0.40, -0.25, 0.10])
    jerk = np.array([0.20, 0.15, -0.04])
    yaw = 0.30
    epsilon = 1e-5

    controller = module.GeometricCtbrController(_config(module))
    command = controller.compute(
        _state(acceleration=acceleration),
        _target(acceleration=acceleration, jerk=jerk, yaw=yaw),
        0.01,
    )
    rotation_before = _computed_rotation_at(
        module, acceleration - jerk * epsilon, jerk, yaw
    )
    rotation_after = _computed_rotation_at(
        module, acceleration + jerk * epsilon, jerk, yaw
    )
    numerical_body_rate = module.so3_log(
        rotation_before.T @ rotation_after
    ) / (2.0 * epsilon)

    assert np.allclose(
        command["computed_body_rate"], numerical_body_rate, rtol=2e-4, atol=2e-6
    )


def test_analytic_omega_c_ignores_invalid_mocap_acceleration():
    """Invalid derivative frames must use a_d, not a stale acceleration spike."""
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))

    command = controller.compute(
        _state(acceleration=[8.0, -7.0, 3.0], derivatives_valid=False),
        _target(),
        0.01,
    )

    assert np.allclose(command["computed_body_rate"], 0.0, atol=1e-12)


def test_log_difference_remains_an_explicit_debug_fallback():
    """Removing the selectable legacy method would break V2 comparison runs."""
    module = _controller_module()
    controller = module.GeometricCtbrController(
        _config(module, omega_c_method="log_difference")
    )

    first = controller.compute(_state(), _target(yaw=0.0), 0.01)
    second = controller.compute(_state(), _target(yaw=0.02), 0.01)

    assert np.allclose(first["computed_body_rate"], 0.0, atol=1e-12)
    assert math.isclose(second["computed_body_rate"][2], 2.0, rel_tol=2e-3)


def test_circle_reference_exposes_jerk_and_yaw_rate_for_analytic_omega_c():
    """Dropping trajectory derivatives makes the V2 feedforward incomplete."""
    from test_ctbr_trajectory_smoothstep import _config as trajectory_config
    from ctbr_trajectory import CircularFlightTrajectory

    trajectory = CircularFlightTrajectory(trajectory_config())
    trajectory.reset(np.array([0.0, 0.0, 0.2]), start_yaw=0.0, now=0.0)
    duration = float(trajectory.circle_duration_s)
    elapsed = 0.37 * duration
    epsilon = 1e-4

    target = trajectory._circle_target(elapsed)
    before = trajectory._circle_target(elapsed - epsilon)
    after = trajectory._circle_target(elapsed + epsilon)
    numerical_jerk = (after["acceleration"] - before["acceleration"]) / (2.0 * epsilon)

    assert np.allclose(target["jerk"], numerical_jerk, rtol=2e-4, atol=2e-5)
    assert math.isclose(
        target["yaw_rate"],
        np.linalg.norm(target["velocity"][:2]) / trajectory.config.circle_radius_m,
        rel_tol=1e-9,
        abs_tol=1e-9,
    )


def test_second_order_velocity_filter_smooths_velocity_and_derives_acceleration():
    module = _controller_module()
    velocity_filter = module.SecondOrderVelocityFilter(cutoff_hz=5.0, max_dt=0.05)
    time_s = np.arange(160, dtype=float) * 0.01
    raw_velocity = np.sin(2.0 * np.pi * 1.0 * time_s)

    filtered = []
    accelerations = []
    readiness = []
    for timestamp, value in zip(time_s, raw_velocity):
        output, acceleration, ready = velocity_filter.update(
            np.array([value, 0.0, 0.0]), timestamp
        )
        filtered.append(output)
        accelerations.append(acceleration)
        readiness.append(ready)
    filtered = np.asarray(filtered).T
    accelerations = np.asarray(accelerations).T

    noisy_filter = module.SecondOrderVelocityFilter(cutoff_hz=5.0, max_dt=0.05)
    noisy_velocity = np.sin(2.0 * np.pi * 20.0 * time_s)
    filtered_noise = np.asarray([
        noisy_filter.update(np.array([value, 0.0, 0.0]), timestamp)[0]
        for timestamp, value in zip(time_s, noisy_velocity)
    ]).T
    assert np.std(filtered_noise[0, 30:-30]) < 0.1 * np.std(noisy_velocity[30:-30])
    assert readiness[0] is False
    assert readiness[-1] is True
    assert np.allclose(accelerations[:, 1:], np.diff(filtered, axis=1) / 0.01)


def test_second_order_velocity_filter_resets_on_invalid_or_long_gap():
    module = _controller_module()
    velocity_filter = module.SecondOrderVelocityFilter(cutoff_hz=5.0, max_dt=0.05)

    first = velocity_filter.update(np.array([0.0, 0.0, 0.0]), 0.0)
    second = velocity_filter.update(np.array([1.0, 0.0, 0.0]), 0.01)
    invalid = velocity_filter.update(np.array([np.nan, 0.0, 0.0]), 0.02)
    after_invalid = velocity_filter.update(np.array([2.0, 0.0, 0.0]), 0.03)
    after_gap = velocity_filter.update(np.array([3.0, 0.0, 0.0]), 0.20)

    assert first[2] is False
    assert second[2] is True
    assert invalid == (None, None, False)
    assert after_invalid[2] is False
    assert np.allclose(after_invalid[0], [2.0, 0.0, 0.0])
    assert after_gap[2] is False
    assert np.allclose(after_gap[0], [3.0, 0.0, 0.0])


def test_controller_uses_filtered_velocity_when_present():
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))
    state = _state()
    state["velocity"] = np.array([1.0, 0.0, 0.0])
    state["filtered_velocity"] = np.zeros(3)
    state["filtered_acceleration"] = np.zeros(3)
    target = _target()

    command = controller.compute(state, target, 0.01)

    assert np.allclose(command["velocity_error"], np.zeros(3))
    assert np.isclose(command["desired_force"][0], 0.0, atol=1e-12)


def test_controller_prefers_blended_control_velocity_over_mocap_filter():
    """The configured EKF weight must reach the velocity feedback term."""
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))
    state = _state()
    state["velocity"] = np.zeros(3)
    state["filtered_velocity"] = np.zeros(3)
    state["control_velocity"] = np.array([0.4, 0.0, 0.0])
    state["filtered_acceleration"] = np.zeros(3)
    state["filter_derivatives_valid"] = True

    command = controller.compute(state, _target(), 0.01)

    assert np.allclose(command["velocity_error"], [0.4, 0.0, 0.0])
    assert np.isclose(command["desired_force"][0], -0.1802, atol=1e-12)


def test_controller_uses_filtered_acceleration_when_present():
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))
    state = _state(acceleration=[9.0, 0.0, 0.0])
    state["filtered_velocity"] = np.zeros(3)
    state["filtered_acceleration"] = np.zeros(3)
    target = _target()

    command = controller.compute(state, target, 0.01)

    assert np.allclose(command["computed_body_rate"], np.zeros(3), atol=1e-12)


def test_kinematic_feedback_keeps_nokov_pose_and_uses_pure_ekf_kinematics():
    """NOKOV pose must not be mixed with EKF velocity/acceleration."""
    module = _controller_module()
    mocap_rotation = module.quaternion_to_rotation(0.0, 0.0, 0.2, math.sqrt(0.96))
    mocap = {
        "position": np.array([1.0, -2.0, 0.4]),
        "rotation": mocap_rotation,
        "velocity": np.zeros(3),
        "filtered_velocity": np.array([1.0, 2.0, 3.0]),
        "filtered_acceleration": np.array([2.0, 4.0, 6.0]),
        "derivatives_valid": True,
        "filter_derivatives_valid": True,
    }
    ekf = {
        "position": np.array([1.03, -2.02, 0.41]),
        "filtered_velocity": np.array([5.0, 6.0, 7.0]),
        "filtered_acceleration": np.array([10.0, 12.0, 14.0]),
        "filter_derivatives_valid": True,
        "received_time": 9.98,
    }

    mixed = module.blend_kinematic_feedback(
        mocap, ekf, now=10.0, ekf_weight=1.0,
        ekf_state_timeout=0.1, max_acceleration=20.0,
        max_position_delta=0.2,
    )

    assert np.allclose(mixed["position"], [1.0, -2.0, 0.4])
    assert np.allclose(mixed["rotation"], mocap_rotation)
    assert np.allclose(mixed["control_velocity"], [5.0, 6.0, 7.0])
    assert np.allclose(mixed["mixed_acceleration"], [10.0, 12.0, 14.0])
    assert np.allclose(mixed["control_acceleration"], [10.0, 12.0, 14.0])
    assert math.isclose(mixed["ekf_kinematics_weight_effective"], 1.0)
    assert mixed["ekf_position_consistent"] is True


def test_kinematic_feedback_never_falls_back_to_mocap_when_ekf_is_stale():
    """A delayed EKF log must not inject NOKOV derivative noise into CTBR."""
    module = _controller_module()
    mocap = {
        "position": np.array([0.0, 0.0, 0.5]),
        "rotation": np.eye(3),
        "velocity": np.zeros(3),
        "filtered_velocity": np.array([0.2, -0.1, 0.3]),
        "filtered_acceleration": np.array([0.4, -0.2, 0.6]),
        "derivatives_valid": True,
        "filter_derivatives_valid": True,
    }
    stale_ekf = {
        "position": np.zeros(3),
        "filtered_velocity": np.array([9.0, 9.0, 9.0]),
        "filtered_acceleration": np.array([9.0, 9.0, 9.0]),
        "filter_derivatives_valid": True,
        "received_time": 9.0,
    }

    mixed = module.blend_kinematic_feedback(
        mocap, stale_ekf, now=10.0, ekf_weight=0.8,
        ekf_state_timeout=0.1, max_acceleration=5.0,
        max_position_delta=20.0,
    )

    assert np.allclose(mixed["control_velocity"], 0.0)
    assert np.allclose(mixed["mixed_acceleration"], 0.0)
    assert np.allclose(mixed["control_acceleration"], 0.0)
    assert math.isclose(mixed["ekf_kinematics_weight_effective"], 0.0)
    assert mixed["ekf_status"] == "stale_or_unready"


def test_kinematic_feedback_uses_zero_kinematics_when_ekf_is_missing():
    """No EKF sample must remain finite without using NOKOV derivatives."""
    module = _controller_module()
    mocap = {
        "position": np.array([0.0, 0.0, 0.5]),
        "rotation": np.eye(3),
        "filtered_velocity": np.array([0.2, -0.1, 0.3]),
        "filtered_acceleration": np.array([0.4, -0.2, 0.6]),
        "derivatives_valid": True,
        "filter_derivatives_valid": True,
    }

    mixed = module.blend_kinematic_feedback(
        mocap, None, now=10.0, ekf_weight=0.8,
        ekf_state_timeout=0.1, max_acceleration=5.0,
        max_position_delta=0.2,
    )

    assert np.all(np.isfinite(mixed["control_velocity"]))
    assert np.all(np.isfinite(mixed["mixed_acceleration"]))
    assert np.allclose(mixed["control_velocity"], 0.0)
    assert np.allclose(mixed["mixed_acceleration"], 0.0)
    assert math.isclose(mixed["ekf_kinematics_weight_effective"], 0.0)


def test_kinematic_feedback_holds_last_valid_ekf_without_using_mocap_derivatives():
    module = _controller_module()
    mocap = {
        "position": np.array([0.0, 0.0, 0.5]),
        "rotation": np.eye(3),
        "filtered_velocity": np.array([9.0, 9.0, 9.0]),
        "filtered_acceleration": np.array([9.0, 9.0, 9.0]),
    }
    held = {
        "velocity": np.array([0.2, -0.1, 0.3]),
        "acceleration": np.array([0.4, -0.2, 0.6]),
        "valid_time": 9.95,
    }
    mixed = module.blend_kinematic_feedback(
        mocap, None, now=10.0, ekf_weight=1.0,
        ekf_state_timeout=0.1, max_acceleration=5.0,
        max_position_delta=0.2, last_valid_ekf=held, hold_timeout_s=0.1,
    )

    assert np.allclose(mixed["control_velocity"], held["velocity"])
    assert np.allclose(mixed["mixed_acceleration"], held["acceleration"])
    assert mixed["ekf_kinematics_held"] is True
    assert mixed["ekf_status"] == "held_last_valid"


def test_ekf_callback_filters_firmware_velocity_with_firmware_sample_time():
    """Using host callback spacing for EKF differentiation would distort a delayed log."""
    module = _controller_module()

    class FixedFilter:
        def __init__(self):
            self.timestamp = None

        def reset(self):
            pass

        def update(self, velocity, timestamp):
            self.timestamp = timestamp
            assert np.allclose(velocity, [0.4, -0.5, 0.6])
            return np.array([0.1, -0.2, 0.3]), np.array([1.0, 2.0, 3.0]), True

    velocity_filter = FixedFilter()
    node = types.SimpleNamespace(
        max_abs_position_m=10.0,
        ekf_velocity_filter=velocity_filter,
        ekf_velocity_filter_lock=threading.Lock(),
        lock=threading.Lock(),
        latest_ekf_state=None,
    )
    message = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: 42.25)),
        values=[1.0, -2.0, 0.5, 0.4, -0.5, 0.6],
    )
    original_monotonic = module.time.monotonic
    module.time.monotonic = lambda: 7.0
    try:
        module.CtbrControllerNode._ekf_state_callback(node, message)
    finally:
        module.time.monotonic = original_monotonic

    assert math.isclose(velocity_filter.timestamp, 42.25)
    assert np.allclose(node.latest_ekf_state["position"], [1.0, -2.0, 0.5])
    assert np.allclose(node.latest_ekf_state["velocity"], [0.4, -0.5, 0.6])
    assert np.allclose(node.latest_ekf_state["filtered_velocity"], [0.1, -0.2, 0.3])
    assert np.allclose(node.latest_ekf_state["filtered_acceleration"], [1.0, 2.0, 3.0])
    assert node.latest_ekf_state["filter_derivatives_valid"] is True
    assert math.isclose(node.latest_ekf_state["received_time"], 7.0)


def test_preflight_keeps_initial_one_shot_battery_sample():
    """A latched one-shot battery log must not be discarded when preflight starts."""
    module = _controller_module()
    module.rospy.loginfo = lambda *_args, **_kwargs: None
    node = types.SimpleNamespace(
        require_preflight_voltage=True,
        preflight_voltage_ready=False,
        preflight_voltage_failed=False,
        preflight_voltage_started_time=None,
        preflight_voltage_samples_required=1,
        preflight_voltage_timeout_s=5.0,
        preflight_battery_max_age_s=0.5,
        preflight_min_voltage_v=3.8,
        preflight_voltage_reference_v=4.2,
        preflight_raw_scale_exponent=0.0,
        preflight_raw_scale_min=0.9,
        preflight_raw_scale_max=1.12,
        preflight_voltage_samples=deque([(10.0, 4.0)], maxlen=10),
        preflight_voltage_sample_count=0,
        preflight_voltage_v=math.nan,
        thrust_raw_scale=1.0,
        controller=types.SimpleNamespace(reset=lambda: None),
        lock=threading.Lock(),
    )
    state = {
        "position": np.array([0.0, 0.0, 0.0]),
        "rotation": np.eye(3),
    }

    target = module.CtbrControllerNode._preflight_voltage_target(node, state, 10.1)

    assert target is None
    assert node.preflight_voltage_ready is True
    assert math.isclose(node.preflight_voltage_v, 4.0)


def test_state_callback_caches_velocity_filter_output_not_raw_acceleration():
    module = _controller_module()
    node = types.SimpleNamespace(
        max_abs_position_m=10.0,
        max_mocap_velocity_mps=1.0,
        max_mocap_acceleration_mps2=5.0,
        velocity_filter=module.SecondOrderVelocityFilter(cutoff_hz=5.0, max_dt=0.05),
        velocity_filter_lock=threading.Lock(),
        lock=threading.Lock(),
        latest_state=None,
    )

    def message(velocity_x, acceleration_x, sample_time):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: sample_time)
            ),
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=velocity_x, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
            acceleration=types.SimpleNamespace(x=acceleration_x, y=0.0, z=0.0),
            valid=True,
            derivatives_valid=True,
        )

    timestamps = iter((1.0, 1.01))
    original_monotonic = module.time.monotonic
    module.time.monotonic = lambda: next(timestamps)
    try:
        module.CtbrControllerNode._state_callback(node, message(0.0, 999.0, 10.0))
        module.CtbrControllerNode._state_callback(node, message(0.4, 999.0, 10.01))
    finally:
        module.time.monotonic = original_monotonic

    assert np.isclose(node.latest_state["acceleration"][0], 999.0)
    assert node.latest_state["derivatives_valid"] is True
    assert node.latest_state["filtered_acceleration"][0] < 999.0


def test_state_callback_uses_mocap_stamp_instead_of_callback_arrival_time():
    """Callback scheduling gaps must not reset a continuous mocap sample stream."""
    module = _controller_module()
    node = types.SimpleNamespace(
        max_abs_position_m=10.0,
        max_mocap_velocity_mps=1.0,
        max_mocap_acceleration_mps2=5.0,
        velocity_filter=module.SecondOrderVelocityFilter(cutoff_hz=5.0, max_dt=0.05),
        velocity_filter_lock=threading.Lock(),
        lock=threading.Lock(),
        latest_state=None,
    )

    def message(velocity_x, sample_time):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: sample_time)
            ),
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=velocity_x, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
            acceleration=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            valid=True,
            derivatives_valid=True,
        )

    arrival_times = iter((1.0, 1.5))
    original_monotonic = module.time.monotonic
    module.time.monotonic = lambda: next(arrival_times)
    try:
        module.CtbrControllerNode._state_callback(node, message(0.0, 10.0))
        module.CtbrControllerNode._state_callback(node, message(0.4, 10.01))
    finally:
        module.time.monotonic = original_monotonic

    assert node.latest_state["filter_derivatives_valid"] is True
    assert node.latest_state["filtered_acceleration"][0] > 0.0
    assert np.isclose(node.latest_state["mocap_sample_time_s"], 10.01)


def test_state_callback_resets_filter_when_mocap_derivatives_are_invalid():
    """A source differentiator reset must not connect to prior local filter state."""
    module = _controller_module()
    node = types.SimpleNamespace(
        max_abs_position_m=10.0,
        max_mocap_velocity_mps=1.0,
        max_mocap_acceleration_mps2=5.0,
        velocity_filter=module.SecondOrderVelocityFilter(cutoff_hz=5.0, max_dt=0.05),
        velocity_filter_lock=threading.Lock(),
        lock=threading.Lock(),
        latest_state=None,
    )

    def message(velocity_x, sample_time, derivatives_valid):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: sample_time)
            ),
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=velocity_x, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
            acceleration=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            valid=True,
            derivatives_valid=derivatives_valid,
        )

    original_monotonic = module.time.monotonic
    module.time.monotonic = lambda: 1.0
    try:
        module.CtbrControllerNode._state_callback(node, message(0.0, 10.0, True))
        module.CtbrControllerNode._state_callback(node, message(0.4, 10.01, True))
        assert node.latest_state["filter_derivatives_valid"] is True

        module.CtbrControllerNode._state_callback(node, message(0.9, 10.02, False))
    finally:
        module.time.monotonic = original_monotonic

    assert node.latest_state["derivatives_valid"] is False
    assert node.latest_state["filter_derivatives_valid"] is False
    assert np.allclose(node.latest_state["filtered_velocity"], np.zeros(3))
    assert np.allclose(node.latest_state["filtered_acceleration"], np.zeros(3))


def test_state_callback_keeps_raw_acceleration_for_csv_and_clips_control_copy():
    """CSV diagnostics retain the pre-limit derivative while control uses the cap."""
    module = _controller_module()

    class FixedFilter:
        def reset(self):
            pass

        def update(self, _velocity, _timestamp):
            return np.zeros(3), np.array([12.0, -8.0, 2.0]), True

    node = types.SimpleNamespace(
        max_abs_position_m=10.0,
        max_mocap_velocity_mps=1.0,
        max_mocap_acceleration_mps2=5.0,
        velocity_filter=FixedFilter(),
        velocity_filter_lock=threading.Lock(),
        lock=threading.Lock(),
        latest_state=None,
    )
    message = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: 10.0)),
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        acceleration=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        valid=True,
        derivatives_valid=True,
    )
    module.CtbrControllerNode._state_callback(node, message)

    assert np.allclose(node.latest_state["filtered_acceleration"], [12.0, -8.0, 2.0])
    assert np.allclose(node.latest_state["control_acceleration"], [5.0, -5.0, 2.0])


def test_controller_uses_clipped_acceleration_copy_not_csv_diagnostic():
    """The analytic Omega_c path must consume only the internal clipped value."""
    module = _controller_module()
    clipped_state = _state()
    clipped_state["filtered_velocity"] = np.zeros(3)
    clipped_state["filtered_acceleration"] = np.array([100.0, 0.0, 0.0])
    clipped_state["control_acceleration"] = np.array([5.0, 0.0, 0.0])
    clipped_state["filter_derivatives_valid"] = True
    reference_state = _state()
    reference_state["filtered_velocity"] = np.zeros(3)
    reference_state["filtered_acceleration"] = np.array([5.0, 0.0, 0.0])
    reference_state["filter_derivatives_valid"] = True

    clipped_command = module.GeometricCtbrController(_config(module)).compute(
        clipped_state, _target(), 0.01
    )
    reference_command = module.GeometricCtbrController(_config(module)).compute(
        reference_state, _target(), 0.01
    )

    assert np.allclose(
        clipped_command["computed_body_rate"],
        reference_command["computed_body_rate"],
        atol=1e-12,
    )


def test_controller_rejects_filtered_acceleration_before_filter_warmup():
    """The analytic Omega_c path must wait for a locally derived acceleration."""
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))
    state = _state(acceleration=np.zeros(3), derivatives_valid=True)
    state["filtered_velocity"] = np.zeros(3)
    state["filtered_acceleration"] = np.array([9.0, 0.0, 0.0])
    state["filter_derivatives_valid"] = False

    command = controller.compute(state, _target(), 0.01)

    assert np.allclose(command["computed_body_rate"], np.zeros(3), atol=1e-12)


def test_controller_disables_velocity_feedback_before_filter_warmup():
    """Unknown velocity must not be interpreted as zero during a moving target."""
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))
    state = _state()
    state["filtered_velocity"] = np.zeros(3)
    state["filtered_acceleration"] = np.zeros(3)
    state["filter_derivatives_valid"] = False
    target = _target()
    target["velocity"] = np.array([0.5, 0.0, 0.0])

    command = controller.compute(state, target, 0.01)

    assert np.allclose(command["velocity_error"], np.zeros(3))
    assert np.isclose(command["desired_force"][0], 0.0, atol=1e-12)
    assert np.isclose(controller.position_integral[0], 0.0, atol=1e-12)


def test_analytic_omega_c_uses_saturated_integral_rate():
    """An integral state pinned at its limit must not create a fictitious F_dot."""
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(
        module,
        position_gain=np.zeros(3),
        velocity_gain=np.zeros(3),
        integral_gain=np.array([1.0, 0.0, 0.0]),
        integral_limit=np.array([0.01, 0.1, 0.1]),
        position_integral_c1=1.0,
    ))
    state = _state()
    state["position"] = np.array([1.0, 0.0, 0.0])

    command = controller.compute(state, _target(), 1.0)

    assert np.allclose(controller.position_integral, [0.01, 0.0, 0.0])
    assert np.allclose(command["computed_body_rate"], np.zeros(3), atol=1e-12)


def test_filter_update_and_timeout_reset_do_not_run_concurrently():
    """Timer safety resets cannot mutate filter history during a state callback."""
    module = _controller_module()

    class BlockingFilter:
        def __init__(self):
            self.update_entered = threading.Event()
            self.allow_update = threading.Event()
            self.in_update = False
            self.concurrent_reset = False

        def update(self, _velocity, _timestamp):
            self.in_update = True
            self.update_entered.set()
            assert self.allow_update.wait(timeout=1.0)
            self.in_update = False
            return np.zeros(3), np.zeros(3), False

        def reset(self):
            self.concurrent_reset = self.concurrent_reset or self.in_update

    velocity_filter = BlockingFilter()
    node = types.SimpleNamespace(
        max_abs_position_m=10.0,
        max_mocap_velocity_mps=1.0,
        max_mocap_acceleration_mps2=5.0,
        velocity_filter=velocity_filter,
        velocity_filter_lock=threading.Lock(),
        lock=threading.Lock(),
        latest_state=None,
        last_tick=0.0,
        state_timeout=0.1,
        controller=types.SimpleNamespace(reset=lambda: None),
        trajectory=None,
        flight_phase="test",
        _publish_zero=lambda: None,
        _write_invalid_log=lambda _now, _state, _reason="": None,
    )
    message = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: 10.0)),
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        acceleration=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        valid=True,
        derivatives_valid=True,
    )
    original_monotonic = module.time.monotonic
    module.time.monotonic = lambda: 1.0
    callback_thread = threading.Thread(
        target=module.CtbrControllerNode._state_callback, args=(node, message)
    )
    timeout_thread = threading.Thread(
        target=module.CtbrControllerNode._timer_callback, args=(node, None)
    )
    try:
        callback_thread.start()
        assert velocity_filter.update_entered.wait(timeout=1.0)
        timeout_thread.start()
        velocity_filter.allow_update.set()
        callback_thread.join(timeout=1.0)
        timeout_thread.join(timeout=1.0)
    finally:
        module.time.monotonic = original_monotonic

    assert not callback_thread.is_alive()
    assert not timeout_thread.is_alive()
    assert velocity_filter.concurrent_reset is False


def test_timer_rejects_stale_mocap_stamp_after_a_fresh_callback_arrival():
    """A queued old pose must not be treated as fresh merely on receipt."""
    module = _controller_module()
    module.rospy.Time = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_sec=lambda: 100.0)
    )
    events = []
    node = types.SimpleNamespace(
        last_tick=0.99,
        lock=threading.Lock(),
        latest_state={
            "valid": True,
            "received_time": 1.0,
            "mocap_sample_time_s": 99.0,
        },
        state_timeout=0.1,
        controller=types.SimpleNamespace(reset=lambda: events.append("controller_reset")),
        velocity_filter=types.SimpleNamespace(reset=lambda: events.append("filter_reset")),
        velocity_filter_lock=threading.Lock(),
        trajectory=None,
        flight_phase="test",
        _publish_zero=lambda: events.append("zero"),
        _write_invalid_log=lambda _now, _state, _reason="": events.append("invalid_log"),
        _publish_path=lambda _state: (_ for _ in ()).throw(
            AssertionError("stale state reached the control path")
        ),
    )
    original_monotonic = module.time.monotonic
    module.time.monotonic = lambda: 1.01
    try:
        module.CtbrControllerNode._timer_callback(node, None)
    finally:
        module.time.monotonic = original_monotonic

    assert events == ["controller_reset", "filter_reset", "zero", "invalid_log"]


def test_state_ready_accepts_a_small_future_mocap_stamp_from_callback_scheduling():
    """A newly queued callback may have a stamp milliseconds after the timer."""
    module = _controller_module()
    module.rospy.Time = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_sec=lambda: 100.0)
    )
    node = types.SimpleNamespace(
        lock=threading.Lock(),
        latest_state={
            "valid": True,
            "received_time": 1.0,
            "mocap_sample_time_s": 100.008,
        },
        state_timeout=0.1,
        mocap_timestamp_future_tolerance_s=0.02,
    )

    assert module.CtbrControllerNode._state_ready(node, now=1.01) is True


def test_state_ready_rejects_a_mocap_stamp_far_in_the_future():
    """The scheduling tolerance must not hide a real ROS-clock fault."""
    module = _controller_module()
    module.rospy.Time = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_sec=lambda: 100.0)
    )
    node = types.SimpleNamespace(
        lock=threading.Lock(),
        latest_state={
            "valid": True,
            "received_time": 1.0,
            "mocap_sample_time_s": 100.021,
        },
        state_timeout=0.1,
        mocap_timestamp_future_tolerance_s=0.02,
    )

    assert module.CtbrControllerNode._state_ready(node, now=1.01) is False


def test_filter_columns_are_appended_after_legacy_csv_columns():
    """Column-index readers retain the legacy CSV layout after adding diagnostics."""
    module = _controller_module()
    with tempfile.TemporaryDirectory() as directory:
        logger = module.FlightCsvLogger(directory, "test")
        logger.close()
        header = Path(logger.path).read_text().splitlines()[0].split(",")

    assert header[header.index("command_thrust_newton")] == "command_thrust_newton"
    assert header[header.index("mocap_sample_age_s"):header.index("controller_tick_interval_s")] == [
        "mocap_sample_age_s",
        "ekf_position_x", "ekf_position_y", "ekf_position_z",
        "ekf_velocity_x", "ekf_velocity_y", "ekf_velocity_z",
        "ekf_acceleration_x", "ekf_acceleration_y", "ekf_acceleration_z",
        "ekf_state_valid", "ekf_state_age_s", "ekf_kinematics_weight_effective",
        "control_velocity_x", "control_velocity_y", "control_velocity_z",
        "mixed_acceleration_x", "mixed_acceleration_y", "mixed_acceleration_z",
        "control_acceleration_x", "control_acceleration_y", "control_acceleration_z",
        "mission_time_s", "vehicle_id", "radio_uri", "orbit_phase_rad",
        "formation_state", "global_abort_reason",
        "position_integral_x", "position_integral_y", "position_integral_z",
        "command_raw_thrust", "command_raw_thrust_age_s", "invalid_reason",
        "ekf_position_error_m", "ekf_position_consistent",
        "ekf_kinematics_held", "ekf_fault_age_s", "ekf_status",
        "payload_state_valid",
        "payload_position_error_x", "payload_position_error_y", "payload_position_error_z",
        "payload_velocity_error_x", "payload_velocity_error_y", "payload_velocity_error_z",
        "payload_attitude_error_x", "payload_attitude_error_y", "payload_attitude_error_z",
        "link_direction_x", "link_direction_y", "link_direction_z",
        "desired_link_direction_x", "desired_link_direction_y", "desired_link_direction_z",
            "link_direction_error_x", "link_direction_error_y", "link_direction_error_z",
            "desired_tension_n",
            "payload_position_x", "payload_position_y", "payload_position_z",
            "payload_raw_velocity_x", "payload_raw_velocity_y", "payload_raw_velocity_z",
            "payload_raw_acceleration_x", "payload_raw_acceleration_y", "payload_raw_acceleration_z",
            "payload_velocity_x", "payload_velocity_y", "payload_velocity_z",
            "payload_acceleration_x", "payload_acceleration_y", "payload_acceleration_z",
            "payload_body_rate_x", "payload_body_rate_y", "payload_body_rate_z",
            "payload_filter_derivatives_valid",
            "payload_target_x", "payload_target_y", "payload_target_z",
            "payload_desired_force_x", "payload_desired_force_y", "payload_desired_force_z",
            "payload_desired_moment_x", "payload_desired_moment_y", "payload_desired_moment_z",
            "raw_desired_link_direction_x", "raw_desired_link_direction_y",
            "raw_desired_link_direction_z",
            "desired_tension_signed_n", "payload_yaw_enabled", "transport_blend",
            "transport_blend_dot", "transport_link_gain_scale",
            "ramp_position_hold_gain_scale", "transport_position_hold_gain_scale",
            "payload_landing_rope_preserving", "payload_landing_rope_distance_m",
            "desired_force_dot_x", "desired_force_dot_y", "desired_force_dot_z",
            "payload_ground_clearance_m", "payload_prelift_clearance_target_m",
            "payload_prelift_ready", "payload_prelift_active",
            "payload_prelift_stage", "payload_prelift_reference_rise_m",
            "payload_prelift_reference_lead_m", "payload_prelift_failure_reason",
            "target_velocity_x", "target_velocity_y", "target_velocity_z",
            "target_acceleration_x", "target_acceleration_y", "target_acceleration_z",
            "payload_state_age_s", "emergency_landing_reason",
            "prelift_vertical_integral_active", "prelift_vertical_integral_force_n",
            "transport_support_trim_n", "payload_minimum_vehicle_distance_m",
            "payload_predicted_vehicle_distance_m", "payload_safety_reason",
            "transport_max_tilt_rad",
        ]
