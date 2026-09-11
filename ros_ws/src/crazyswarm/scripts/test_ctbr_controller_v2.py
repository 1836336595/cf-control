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
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
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
    for name in ("CTBR", "GenericLogData", "MocapState"):
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
    nav_msgs_msg.Path = type("Path", (), {})
    nav_msgs.msg = nav_msgs_msg
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msgs_msg


def _controller_module():
    _install_ros_import_stubs()
    module_name = "ctbr_controller_v2_under_test"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(
        module_name, SCRIPT_DIR / "ctbr_controller.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


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


def test_analytic_omega_c_forwards_target_yaw_rate_without_attitude_error():
    """A missing b1 reference derivative would incorrectly produce zero yaw rate."""
    module = _controller_module()
    controller = module.GeometricCtbrController(_config(module))

    command = controller.compute(
        _state(), _target(yaw=0.0, yaw_rate=0.6), 0.01
    )

    assert np.allclose(command["computed_body_rate"], [0.0, 0.0, 0.6], atol=1e-10)
    assert np.allclose(command["body_rate_command"], [0.0, 0.0, 0.6], atol=1e-10)


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


def test_kinematic_blend_keeps_nokov_position_and_rotation():
    """Replacing the mocap pose while blending feedback would add radio latency."""
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
        mocap, ekf, now=10.0, ekf_weight=0.25,
        ekf_state_timeout=0.1, max_acceleration=20.0,
        max_position_delta=0.2,
    )

    assert np.allclose(mixed["position"], [1.0, -2.0, 0.4])
    assert np.allclose(mixed["rotation"], mocap_rotation)
    assert np.allclose(mixed["control_velocity"], [2.0, 3.0, 4.0])
    assert np.allclose(mixed["mixed_acceleration"], [4.0, 6.0, 8.0])
    assert np.allclose(mixed["control_acceleration"], [4.0, 6.0, 8.0])
    assert math.isclose(mixed["ekf_kinematics_weight_effective"], 0.25)


def test_kinematic_blend_falls_back_to_mocap_when_ekf_is_stale():
    """A delayed radio log must not affect CTBR feedback even when k is nonzero."""
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

    assert np.allclose(mixed["control_velocity"], [0.2, -0.1, 0.3])
    assert np.allclose(mixed["mixed_acceleration"], [0.4, -0.2, 0.6])
    assert np.allclose(mixed["control_acceleration"], [0.4, -0.2, 0.6])
    assert math.isclose(mixed["ekf_kinematics_weight_effective"], 0.0)


def test_kinematic_blend_uses_mocap_when_ekf_is_missing():
    """No EKF sample must not turn the pure-NOKOV fallback into NaN."""
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
    assert np.allclose(mixed["control_velocity"], mocap["filtered_velocity"])
    assert np.allclose(mixed["mixed_acceleration"], mocap["filtered_acceleration"])
    assert math.isclose(mixed["ekf_kinematics_weight_effective"], 0.0)


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
        _write_invalid_log=lambda _now, _state: None,
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
        _write_invalid_log=lambda _now, _state: events.append("invalid_log"),
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


def test_filter_columns_are_appended_after_legacy_csv_columns():
    """Column-index readers retain the legacy CSV layout after adding diagnostics."""
    module = _controller_module()
    with tempfile.TemporaryDirectory() as directory:
        logger = module.FlightCsvLogger(directory, "test")
        logger.close()
        header = Path(logger.path).read_text().splitlines()[0].split(",")

    assert header[header.index("command_thrust_newton")] == "command_thrust_newton"
    assert header[header.index("mocap_sample_age_s"):] == [
        "mocap_sample_age_s",
        "ekf_position_x", "ekf_position_y", "ekf_position_z",
        "ekf_velocity_x", "ekf_velocity_y", "ekf_velocity_z",
        "ekf_acceleration_x", "ekf_acceleration_y", "ekf_acceleration_z",
        "ekf_state_valid", "ekf_state_age_s", "ekf_kinematics_weight_effective",
        "control_velocity_x", "control_velocity_y", "control_velocity_z",
        "mixed_acceleration_x", "mixed_acceleration_y", "mixed_acceleration_z",
        "control_acceleration_x", "control_acceleration_y", "control_acceleration_z",
    ]
