#!/usr/bin/python3
"""基于 Nokov 位姿与固件 EKF 运动学的 Crazyflie 主机端几何 CTBR 外环控制器。

控制分工
========

本文件只运行在主机。Nokov 位姿由 ``crazyswarm_server`` 转换为
``/cf<ID>/mocap_state``；本节点计算总推力 ``T``（N）和机体系角速度
``[p, q, r]``（rad/s），连续发布到 ``/cf<ID>/cmd_ctbr``。服务器再将它转换为
legacy RPYT CRTP 包，机载固件负责角速度内环、电机混控和姿态估计。
Nokov 始终只提供位置和 ``R_WB``。速度和由速度导出的加速度只来自固件 EKF；EKF
位置与 Nokov 的差值仅用作健康检查，绝不把 Nokov 差分速度/加速度切入 CTBR 反馈。

坐标与单位
==========

world 坐标系采用 ROS 约定，z 轴向上。刚体姿态矩阵 ``R`` 将机体系向量转换到 world
系；机体 z 轴 ``R[:, 2]`` 指向螺旋桨产生正推力的方向。位置单位为 m，速度为 m/s，
角速度为 rad/s，推力为整机总推力 N。

运行模式与安全边界
==================

本节点只运行实际 CTBR 控制流程。启动前必须设置 ``~target_confirmed:=true``；状态失效、
状态超时和进程退出时均发送零 CTBR。起飞、轨迹和降落均在同一个低层流式控制器内完成，
不能在发送 CTBR 后再调用高层 ``takeoff()`` 或 ``land()``。
"""

import csv
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime

import numpy as np
from std_msgs.msg import UInt16

from vehicle_config import (
    select_vehicle_entries,
    select_vehicle_entry,
    validate_vehicle_entries,
    vehicle_uri,
)
import rospy

from crazyswarm.msg import CTBR, CTBRTiming, GenericLogData, MocapState
from slung_prelift_trajectory import SlungPreliftTrajectory, handoff_references
from slung_transport_guard import SlungTransportGuard, SlungSupportTrim
from slung_transport_integral import SlungDifferentialHoldIntegral
from native_position_pid import (position_integral_step, position_pid_feedback,
                                 validate_integral_velocity_weight)
from slung_landing import SlungLandingProfile, PayloadTouchdownDetector
from slung_link_calibration import SlungLinkCalibrator
from slung_reference import attachment_rotation_reference, cable_direction_reference
from slung_attitude_trim import SlungAttitudeReferenceTrim
from ctbr_trajectory import (
    CircularFlightTrajectory,
    CircularTrajectoryConfig,
)
from slung_load_controller import (
    SlungLoadConfig,
    SlungLoadController,
    PayloadStateObserver,
    SecondOrderVelocityFilter as PayloadVelocityFilter,
    attachment_points_from_yaml,
    cable_distances,
    link_lengths_vector,
    mocap_top_surface_to_center,
    ordered_vehicle_ids,
    payload_ground_clearance,
    smoothstep5_profile,
    takeup_distance_ready,
    takeup_target_distance,
    takeup_vehicle_targets,
    translate_vehicle_references,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


GLOBAL_CTBR_PARAMETER_ROOT = "/ctbr_controller"
TRAJECTORY_PARAMETER_ROOT = "/ctbr_trajectory"
PAYLOAD_PARAMETER_ROOT = "/slung_payload"
_PARAMETER_MISSING = object()


def vehicle_parameter_namespace(vehicle_id):
    """Return the dedicated absolute controller-parameter root for one CF."""
    try:
        normalized_id = int(vehicle_id)
    except (TypeError, ValueError) as error:
        raise ValueError("Crazyflie ID 必须是正整数: %r" % (vehicle_id,)) from error
    if isinstance(vehicle_id, bool) or normalized_id <= 0:
        raise ValueError("Crazyflie ID 必须是正整数: %r" % (vehicle_id,))
    return "/ctbr_controller_cf%d" % normalized_id


class CtbrParameterResolver:
    """Read CTBR parameters from global, trajectory, and per-CF namespaces."""

    def __init__(self, get_param, has_param, vehicle_id):
        if not callable(get_param) or not callable(has_param):
            raise TypeError("get_param 和 has_param 必须可调用")
        self._get_param = get_param
        self._has_param = has_param
        self.vehicle_id = int(vehicle_id)
        self.vehicle_root = vehicle_parameter_namespace(self.vehicle_id)
        if not self._has_param(self.vehicle_root):
            raise ValueError(
                "CF%d 缺少专属参数块：%s" % (self.vehicle_id, self.vehicle_root)
            )
        root_value = self._get_param(self.vehicle_root)
        if not isinstance(root_value, dict):
            raise ValueError(
                "CF%d 专属参数块必须是字典：%s" % (
                    self.vehicle_id, self.vehicle_root
                )
            )

    @staticmethod
    def _path(root, name):
        name = str(name).strip("/")
        if not name:
            raise ValueError("参数名不能为空")
        return root + "/" + name

    def _read(self, root, name, default=_PARAMETER_MISSING):
        path = self._path(root, name)
        if default is _PARAMETER_MISSING:
            return self._get_param(path)
        return self._get_param(path, default)

    def global_param(self, name, default=_PARAMETER_MISSING):
        return self._read(GLOBAL_CTBR_PARAMETER_ROOT, name, default)

    def trajectory_param(self, name, default=_PARAMETER_MISSING):
        return self._read(TRAJECTORY_PARAMETER_ROOT, name, default)

    def vehicle_param(self, name, default=_PARAMETER_MISSING):
        return self._read(self.vehicle_root, name, default)


def as_vector(value, name):
    """把 ROS 参数或消息字段转成长度为 3 的有限浮点向量。"""
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.size != 3 or not np.all(np.isfinite(vector)):
        raise ValueError("%s 必须是 3 个有限数值" % name)
    return vector


def clamp_vector(vector, lower, upper):
    """逐元素限幅；所有积分器、角速度和推力相关向量均通过此函数防止发散。"""
    return np.minimum(np.maximum(vector, lower), upper)


class SecondOrderVelocityFilter:
    """Causal second-order Butterworth filter for three-axis velocity samples.

    The filter is updated once per new mocap callback.  Its derivative is the
    backward difference of the filtered output, so the controller receives a
    velocity and acceleration derived from the same signal.  Invalid samples
    and long gaps reset the state instead of connecting unrelated segments.
    """

    def __init__(self, cutoff_hz=5.0, max_dt=0.1):
        self.cutoff_hz = float(cutoff_hz)
        self.max_dt = float(max_dt)
        if (not math.isfinite(self.cutoff_hz) or self.cutoff_hz <= 0.0 or
                not math.isfinite(self.max_dt) or self.max_dt <= 0.0):
            raise ValueError("velocity filter cutoff_hz and max_dt must be positive")
        if 2.0 * self.cutoff_hz * self.max_dt > 0.8:
            raise ValueError(
                "velocity filter cutoff_hz is too high for max_dt; "
                "require cutoff_hz <= 0.4 / max_dt"
            )
        self.reset()

    def reset(self):
        self._previous_time = None
        self._previous_input = None
        self._previous_input_2 = None
        self._previous_output = None
        self._previous_output_2 = None
        self.ready = False

    @staticmethod
    def _coefficients(cutoff_hz, dt):
        # Recompute the bilinear-transform coefficients for the mocap sample
        # interval.  __init__ rejects a cutoff that could approach Nyquist at
        # max_dt, so no runtime coefficient clamp is necessary.
        normalized_cutoff = 2.0 * cutoff_hz * dt
        omega = math.pi * normalized_cutoff
        cosine = math.cos(omega)
        alpha = math.sin(omega) / (2.0 * math.sqrt(0.5))
        a0 = 1.0 + alpha
        return (
            (1.0 - cosine) / (2.0 * a0),
            (1.0 - cosine) / a0,
            (1.0 - cosine) / (2.0 * a0),
            -2.0 * cosine / a0,
            (1.0 - alpha) / a0,
        )

    def _seed(self, value, timestamp):
        value = value.copy()
        self._previous_time = float(timestamp)
        self._previous_input = value
        self._previous_input_2 = value.copy()
        self._previous_output = value.copy()
        self._previous_output_2 = value.copy()
        self.ready = False
        return value, np.zeros(3), False

    def update(self, velocity, timestamp):
        """Return filtered velocity, its derivative, and derivative readiness."""
        try:
            value = np.asarray(velocity, dtype=float).reshape(3)
        except (TypeError, ValueError) as error:
            raise ValueError("velocity filter input must contain 3 values") from error
        timestamp = float(timestamp)
        if (not math.isfinite(timestamp) or not np.all(np.isfinite(value))):
            self.reset()
            return None, None, False

        if self._previous_time is None:
            return self._seed(value, timestamp)

        dt = timestamp - self._previous_time
        if not math.isfinite(dt) or dt <= 0.0 or dt > self.max_dt:
            self.reset()
            return self._seed(value, timestamp)

        b0, b1, b2, a1, a2 = self._coefficients(self.cutoff_hz, dt)
        filtered = (
            b0 * value
            + b1 * self._previous_input
            + b2 * self._previous_input_2
            - a1 * self._previous_output
            - a2 * self._previous_output_2
        )
        acceleration = (filtered - self._previous_output) / dt
        self._previous_input_2 = self._previous_input
        self._previous_input = value.copy()
        self._previous_output_2 = self._previous_output
        self._previous_output = filtered.copy()
        self._previous_time = timestamp
        self.ready = True
        return filtered, acceleration, True


def blend_kinematic_feedback(
        mocap_state, ekf_state, now, ekf_weight, ekf_state_timeout,
        max_acceleration, max_position_delta, last_valid_ekf=None,
        hold_timeout_s=0.0, timestamp_future_tolerance_s=0.0):
    """Return a NOKOV-pose / EKF-kinematics control-state view.

    ``ekf_weight`` is retained for launch compatibility: zero disables EKF
    kinematics for bench diagnostics; any positive value enables *pure* EKF
    velocity/acceleration.  It no longer blends in NOKOV derivatives.
    """
    weight = float(ekf_weight)
    timeout = float(ekf_state_timeout)
    acceleration_limit = float(max_acceleration)
    position_delta_limit = float(max_position_delta)
    hold_timeout_s = float(hold_timeout_s)
    timestamp_future_tolerance_s = float(timestamp_future_tolerance_s)
    now = float(now)
    if (not 0.0 <= weight <= 1.0 or timeout <= 0.0 or
            acceleration_limit <= 0.0 or position_delta_limit <= 0.0 or
            not all(math.isfinite(value) for value in (
            weight, timeout, acceleration_limit, position_delta_limit, now,
            hold_timeout_s, timestamp_future_tolerance_s)) or
            hold_timeout_s < 0.0 or timestamp_future_tolerance_s < 0.0):
        raise ValueError("EKF 运动学混合参数无效")

    result = dict(mocap_state)
    effective_weight = 0.0
    ekf_age = math.inf
    ekf_valid = False
    ekf_sample_fresh = False
    ekf_position_consistent = False
    ekf_position_error_m = math.inf
    ekf_kinematics_held = False
    ekf_status = "disabled" if weight <= 0.0 else "missing"
    ekf_position = np.full(3, math.nan)
    ekf_velocity = np.full(3, math.nan)
    ekf_acceleration = np.full(3, math.nan)

    if ekf_state is not None:
        try:
            received_time = float(ekf_state["received_time"])
            ekf_age = now - received_time
            ekf_position = as_vector(ekf_state["position"], "EKF position")
            ekf_velocity = as_vector(
                ekf_state["filtered_velocity"], "EKF filtered velocity"
            )
            ekf_acceleration = as_vector(
                ekf_state["filtered_acceleration"], "EKF filtered acceleration"
            )
            ekf_position_error_m = float(np.linalg.norm(
                ekf_position - mocap_state["position"]
            ))
            ekf_sample_fresh = (
                bool(ekf_state.get("filter_derivatives_valid", False)) and
                -timestamp_future_tolerance_s <= ekf_age <= timeout
            )
            ekf_position_consistent = (
                ekf_position_error_m <= position_delta_limit
            )
            ekf_valid = ekf_sample_fresh and ekf_position_consistent
            if not ekf_sample_fresh:
                ekf_status = "stale_or_unready"
            elif not ekf_position_consistent:
                ekf_status = "position_mismatch"
            else:
                ekf_status = "fresh"
        except (KeyError, TypeError, ValueError):
            ekf_valid = False

    if weight > 0.0 and ekf_valid:
        effective_weight = 1.0
        control_velocity = ekf_velocity.copy()
        mixed_acceleration = ekf_acceleration.copy()
    elif weight > 0.0 and last_valid_ekf is not None:
        try:
            held_age = now - float(last_valid_ekf["valid_time"])
            held_velocity = as_vector(last_valid_ekf["velocity"], "last EKF velocity")
            held_acceleration = as_vector(
                last_valid_ekf["acceleration"], "last EKF acceleration"
            )
            if 0.0 <= held_age <= hold_timeout_s:
                effective_weight = 1.0
                control_velocity = held_velocity
                mixed_acceleration = held_acceleration
                ekf_kinematics_held = True
                ekf_status = "held_last_valid"
            else:
                control_velocity = np.zeros(3)
                mixed_acceleration = np.zeros(3)
        except (KeyError, TypeError, ValueError):
            control_velocity = np.zeros(3)
            mixed_acceleration = np.zeros(3)
    else:
        # NOKOV 差分只用于记录和离线诊断，绝不作为 EKF 失效时的控制替代品。
        control_velocity = np.zeros(3)
        mixed_acceleration = np.zeros(3)
    result["control_velocity"] = control_velocity
    result["mixed_acceleration"] = mixed_acceleration
    result["control_acceleration"] = np.clip(
        mixed_acceleration, -acceleration_limit, acceleration_limit
    )
    result["ekf_position"] = ekf_position
    result["ekf_velocity"] = ekf_velocity
    result["ekf_acceleration"] = ekf_acceleration
    result["ekf_state_age_s"] = ekf_age
    result["ekf_state_valid"] = ekf_valid
    result["ekf_sample_fresh"] = ekf_sample_fresh
    result["ekf_position_consistent"] = ekf_position_consistent
    result["ekf_position_error_m"] = ekf_position_error_m
    result["ekf_kinematics_held"] = ekf_kinematics_held
    result["ekf_status"] = ekf_status
    result["ekf_kinematics_weight_effective"] = effective_weight
    return result


def vee(matrix):
    """反对称矩阵到三维向量的 vee 映射。"""
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def project_so3(rotation):
    """用 SVD 将数值误差造成的近似旋转矩阵投影回右手 SO(3)。"""
    u_matrix, _, v_matrix = np.linalg.svd(rotation)
    result = u_matrix @ v_matrix
    if np.linalg.det(result) < 0.0:
        u_matrix[:, 2] *= -1.0
        result = u_matrix @ v_matrix
    return result


def rope_preserving_vehicle_references(
        payload_position, payload_velocity, payload_acceleration,
        payload_rotation, attachment_points_m, link_directions,
        link_distances_m):
    """Translate vehicle references while preserving measured rope geometry.

    ``link_directions`` point from each vehicle toward its payload attachment.
    Keeping those directions and distances fixed prevents the payload landing
    profile from pulling aircraft toward the load or shrinking their spacing.
    Returned arrays use one row per vehicle.
    """
    position = np.asarray(payload_position, dtype=float).reshape(3)
    velocity = np.asarray(payload_velocity, dtype=float).reshape(3)
    acceleration = np.asarray(payload_acceleration, dtype=float).reshape(3)
    rotation = project_so3(np.asarray(payload_rotation, dtype=float).reshape(3, 3))
    attachments = np.asarray(attachment_points_m, dtype=float)
    directions = np.asarray(link_directions, dtype=float)
    distances = np.asarray(link_distances_m, dtype=float).reshape(-1)
    if (attachments.shape != (3, 3) or directions.shape != (3, 3)
            or distances.shape != (3,)
            or not all(np.all(np.isfinite(value)) for value in
                       (position, velocity, acceleration, attachments,
                        directions, distances))
            or np.any(distances <= 0.0)):
        raise ValueError("invalid rope-preserving landing geometry")
    norms = np.linalg.norm(directions, axis=0)
    if np.any(norms <= 1.0e-9):
        raise ValueError("link directions must be nonzero")
    directions = directions / norms[None, :]
    attachment_world = position[:, None] + rotation @ attachments
    positions = (attachment_world - directions * distances[None, :]).T
    return positions, np.tile(velocity, (3, 1)), np.tile(acceleration, (3, 1))


def so3_log(rotation):
    """SO(3) 对数映射，返回可用于有限差分的旋转向量（rad）。"""
    cos_theta = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    theta = math.acos(cos_theta)
    if theta < 1e-7:
        return 0.5 * vee(rotation - rotation.T)
    if abs(math.pi - theta) < 1e-5:
        values, vectors = np.linalg.eigh((rotation + np.eye(3)) * 0.5)
        axis = vectors[:, int(np.argmax(values))]
        return theta * axis / max(float(np.linalg.norm(axis)), 1e-12)
    return theta * vee(rotation - rotation.T) / (2.0 * math.sin(theta))


def _limit_force_with_derivative(force, force_dot, max_command_thrust, max_tilt_rad):
    """Apply the force safety limits and differentiate the active branches.

    The analytic V2 feedforward must be differentiated from the same force that
    determines ``b3``.  Treating a clipped component as locally constant avoids
    asking the attitude loop to follow a direction that the safety limiter has
    already removed.
    """
    force = np.asarray(force, dtype=float).reshape(3).copy()
    force_dot = np.asarray(force_dot, dtype=float).reshape(3).copy()
    limited = force.copy()
    limited_dot = force_dot.copy()

    raw_vertical = float(force[2])
    limited[2] = float(np.clip(raw_vertical, 0.0, max_command_thrust))
    if raw_vertical <= 0.0 or raw_vertical >= max_command_thrust:
        limited_dot[2] = 0.0

    horizontal = force[:2].copy()
    horizontal_dot = force_dot[:2].copy()
    horizontal_norm = float(np.linalg.norm(horizontal))
    horizontal_limit = limited[2] * math.tan(float(max_tilt_rad))
    if horizontal_norm <= horizontal_limit:
        limited[:2] = horizontal
        limited_dot[:2] = horizontal_dot
    elif horizontal_norm <= 1e-12 or horizontal_limit <= 0.0:
        limited[:2] = 0.0
        limited_dot[:2] = 0.0
    else:
        norm_dot = float(horizontal @ horizontal_dot) / horizontal_norm
        scale = horizontal_limit / horizontal_norm
        scale_dot = -horizontal_limit * norm_dot / (horizontal_norm ** 2)
        limited[:2] = scale * horizontal
        limited_dot[:2] = scale_dot * horizontal + scale * horizontal_dot
    return limited, limited_dot


def analytic_omega_c(computed_rotation, desired_force, desired_force_dot, target, config):
    """Calculate V2's continuous-time ``Omega_c = vee(R_c.T @ R_c_dot)``.

    This implementation follows the MATLAB V2 column-derivative construction,
    with the sign changed for the ROS world-z-up convention (``b3 = F / ||F||``).
    The returned vector is expressed in the desired body frame.
    """
    force = np.asarray(desired_force, dtype=float).reshape(3)
    force_dot = np.asarray(desired_force_dot, dtype=float).reshape(3)
    if not np.all(np.isfinite(force)) or not np.all(np.isfinite(force_dot)):
        return np.zeros(3)
    force_norm = float(np.linalg.norm(force))
    force_epsilon = float(getattr(config, "omega_c_force_norm_epsilon", 1e-8))
    if force_norm < force_epsilon:
        return np.zeros(3)

    rotation = np.asarray(computed_rotation, dtype=float).reshape(3, 3)
    b3_desired = rotation[:, 2]
    b3_dot = (
        (np.eye(3) - np.outer(b3_desired, b3_desired)) @ force_dot
        / force_norm
    )

    yaw = float(target.get("yaw", 0.0))
    yaw_rate = float(target.get("yaw_rate", 0.0))
    b1_reference = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    b1_reference_dot = yaw_rate * np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    projection_matrix = np.eye(3) - np.outer(b3_desired, b3_desired)
    projection = projection_matrix @ b1_reference
    projection_norm = float(np.linalg.norm(projection))
    projection_epsilon = float(
        getattr(config, "omega_c_heading_projection_epsilon", 1e-6)
    )
    b1_desired = rotation[:, 0]
    if projection_norm < projection_epsilon:
        b1_dot = np.zeros(3)
    else:
        projection_dot = (
            -(np.outer(b3_dot, b3_desired) + np.outer(b3_desired, b3_dot))
            @ b1_reference
            + projection_matrix @ b1_reference_dot
        )
        b1_dot = (
            (np.eye(3) - np.outer(b1_desired, b1_desired))
            @ projection_dot
            / projection_norm
        )

    b2_desired = rotation[:, 1]
    b2_dot = np.cross(b3_dot, b1_desired) + np.cross(b3_desired, b1_dot)
    rotation_dot = np.column_stack((b1_dot, b2_dot, b3_dot))
    omega_hat = rotation.T @ rotation_dot
    omega_hat = 0.5 * (omega_hat - omega_hat.T)
    return vee(omega_hat)


def quaternion_to_rotation(x, y, z, w):
    """将 ROS ``[x,y,z,w]`` 四元数转换为机体系到 world 系的旋转矩阵。"""
    quaternion = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-8 or not np.all(np.isfinite(quaternion)):
        raise ValueError("收到无效姿态四元数")
    x, y, z, w = quaternion / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def rotation_to_rpy(rotation):
    """将机体系到 world 系旋转矩阵转为 ZYX ``[roll,pitch,yaw]``（rad）。"""
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return np.array([roll, pitch, yaw])


def phase_requires_full_controller_reset(phase):
    """Return whether a trajectory phase has a discontinuous/safe-state reset.

    Normal references meet continuously at takeoff correction, circle entry,
    circle, final hover and landing.  Their position integral represents the
    per-vehicle trim required to hover and must remain intact.  Only phases
    that intentionally leave normal closed-loop flight may clear it.
    """
    return str(phase) in ("emergency_landing", "landed")


@dataclass
class ControllerConfig:
    """几何控制律的只读配置。

    ``position_gain``、``velocity_gain`` 和 ``integral_gain`` 均按 world xyz 逐轴
    作用；``attitude_gain`` 和 ``max_body_rate`` 按机体 pqr 逐轴作用。所有推力上限
    使用整机总推力 N，而不是单电机推力或 firmware 的 raw PWM 值。
    """
    mass: float
    gravity: float
    max_total_thrust: float
    max_command_thrust: float
    max_tilt_rad: float
    max_body_rate: np.ndarray
    position_gain: np.ndarray
    velocity_gain: np.ndarray
    integral_gain: np.ndarray
    integral_limit: np.ndarray
    attitude_gain: np.ndarray
    attitude_integral_gain: np.ndarray
    attitude_integral_limit: np.ndarray
    position_integral_c1: float
    use_body_rate_feedforward: bool
    omega_c_method: str = "analytic"
    omega_c_force_norm_epsilon: float = 1e-8
    omega_c_heading_projection_epsilon: float = 1e-6
    # Optional first-order filter for the external transport rate setpoint.
    # Zero preserves the historical unfiltered command path.
    transport_body_rate_filter_tau_s: float = 0.0
    transport_force_filter_tau_s: float = 0.0
    # MATLAB crazyflie_slung_independent_controller parameters.  They are
    # acceleration gains, so the independent force law multiplies the whole
    # feedback acceleration by ``mass``.
    independent_position_gain: object = None
    independent_velocity_gain: object = None
    independent_integral_gain: object = None
    independent_integral_limit: object = None
    independent_integral_gate: float = 0.05
    independent_max_feedback_acceleration: object = None
    independent_max_body_rate: object = None
    independent_attitude_gain: object = None
    independent_heading: object = None


class GeometricCtbrController:
    """MATLAB 几何控制器的 ROS world-z-up 版本。

    输入 ``state`` 至少包含 ``position``、``velocity``、``rotation``；输入 ``target``
    包含世界系期望位置、速度、加速度和 yaw，可选 ``jerk``、``yaw_rate``。输出保留
    所有中间量，既用于发布 CTBR，也用于 CSV 和离线调参。该类不直接使用 ROS，便于
    离线单元测试。
    """

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        """清除位置/姿态积分与期望姿态差分历史。

        仅在状态失效、紧急降落或新任务等不连续场景调用。正常轨迹阶段连续衔接时，
        位置积分保留为各飞机自身的悬停/推力偏差补偿。
        """
        self.position_integral = np.zeros(3)
        self.reset_transport_position_pid()
        self.attitude_integral = np.zeros(3)
        self.previous_computed_rotation = None
        self.previous_heading = None
        self.previous_transport_body_rate_command = None
        self.previous_transport_force = None

    def reset_transport_position_pid(self):
        """Clear the transport-only native I without losing independent trim."""
        self.transport_position_integral = np.zeros(3)
        self.transport_full_pid_force_previous = np.zeros(3)
        self._transport_position_pid_tick = None
        self._transport_position_pid_signature = None
        self._transport_position_pid_cached = None
        self._transport_position_pid_reset_tick = None

    def _native_transport_feedback(self, state, target, dt, derivatives_ready,
                                   measured_velocity, position_gain,
                                   velocity_gain, integral_gain):
        """Evaluate shared native feedback once per integral/budget time step."""
        tick = target.get("transport_full_pid_tick")
        integral_velocity_weight = validate_integral_velocity_weight(
            target.get("transport_full_pid_integral_velocity_weight", 1.0))
        if (target.get("transport_full_pid_reset", False) and
                (tick is None or tick != self._transport_position_pid_reset_tick)):
            self.reset_transport_position_pid()
            self._transport_position_pid_reset_tick = tick
        same_tick = tick is not None and tick == self._transport_position_pid_tick
        ep = state["position"] - np.asarray(
            target.get("position_hold_position", target["position"]), dtype=float)
        ev = (measured_velocity - np.asarray(
            target.get("position_hold_velocity", target["velocity"]), dtype=float)
              if derivatives_ready else np.zeros(3))
        reference_acceleration = np.asarray(target.get("position_hold_acceleration",
            target.get("acceleration", np.zeros(3))), dtype=float)
        acceleration = np.asarray(state.get("control_acceleration",
            state.get("filtered_acceleration", state.get("acceleration", reference_acceleration))), dtype=float)
        if not derivatives_ready or not np.all(np.isfinite(acceleration)):
            acceleration = reference_acceleration.copy()
        beta = float(np.clip(target.get("transport_blend", 1.), 0., 1.))
        scale = float(target.get("ramp_position_hold_gain_scale", 0.) if beta < 1.
                      else target.get("transport_position_hold_gain_scale", 0.))
        if not math.isfinite(scale) or scale < 0.:
            scale = 0.
        scale_dot = float(target.get("transport_position_hold_gain_scale_dot", 0.)) if beta >= 1. else 0.
        if not math.isfinite(scale_dot):
            scale_dot = 0.
        # Match the offline matched-scale derivative at a zero hold gain.
        if scale <= 0.:
            scale_dot = 0.
        gate = float(target.get("transport_full_pid_integral_gate_m", .23))
        limit_value = target.get("transport_full_pid_integral_limit")
        if limit_value is None:
            limit_value = getattr(self.config, "independent_integral_limit", None)
            if limit_value is None:
                limit_value = self.config.integral_limit
            limit_value = np.asarray(limit_value, dtype=float) * (10. / 3.)
        integral_limit = np.asarray(limit_value, dtype=float).reshape(3)
        ki = integral_gain * float(target.get("transport_full_pid_integral_gain_scale", 3.))
        feedback_limit = getattr(self.config, "independent_max_feedback_acceleration", None)
        feedback_limit = (np.full(3, np.inf) if feedback_limit is None
                          else np.asarray(feedback_limit, dtype=float).reshape(3))
        force_limit = float(target.get("transport_full_pid_force_limit_n", .06))
        force_rate = float(target.get("transport_full_pid_force_rate_limit_nps", .004))
        force_hold = bool(target.get("transport_full_pid_force_hold", False))
        reason = str(target.get("transport_full_pid_freeze_reason", ""))
        learning = (bool(target.get("transport_full_pid_learning", False))
                    and derivatives_ready and not reason and not force_hold)
        signature = (learning, reason, force_hold, scale, scale_dot, gate,
            integral_velocity_weight, force_limit, force_rate,
            *(tuple(np.asarray(value).reshape(-1)) for value in
              (ep, ev, acceleration, reference_acceleration, position_gain, velocity_gain,
               ki, integral_limit, feedback_limit)))
        if same_tick and signature == self._transport_position_pid_signature:
            result = {key: (value.copy() if isinstance(value, np.ndarray) else value)
                      for key, value in self._transport_position_pid_cached.items()}
            result.update(learning=False, duplicate_tick=True,
                          freeze_reason=reason or "duplicate_tick")
            return result
        previous_integral = self.transport_position_integral.copy()
        self.transport_position_integral, integral_rate, gate_open = position_integral_step(
            previous_integral, ep, ev, dt, gate, integral_limit,
            learning=learning and not same_tick, velocity_weight=integral_velocity_weight)
        pd, pd_dot, _ = position_pid_feedback(ep, ev, position_gain, velocity_gain,
            np.zeros(3), np.zeros(3), feedback_limit, acceleration-reference_acceleration,
            gain_scale=scale, gain_scale_dot=scale_dot)
        pid, pid_dot, unclamped = position_pid_feedback(ep, ev, position_gain, velocity_gain,
            ki, self.transport_position_integral, feedback_limit,
            acceleration-reference_acceleration, integral_rate,
            gain_scale=scale, gain_scale_dot=scale_dot)
        native_i = self.config.mass * (pid-pd)
        previous_force = self.transport_full_pid_force_previous.copy()
        force_norm = float(np.linalg.norm(native_i))
        limited = native_i * min(1., force_limit/max(force_norm, 1.e-30))
        force_limited = not np.allclose(limited, native_i, rtol=0., atol=1.e-14)
        delta = limited-previous_force
        max_step = 0. if same_tick or force_hold else force_rate*dt
        bias = previous_force + delta * min(1., max_step/max(float(np.linalg.norm(delta)), 1.e-30))
        rate_limited = not np.allclose(bias, limited, rtol=0., atol=1.e-14)
        # fixed_force offline budgeting always uses the actual bounded force step.
        # Only the explicitly unbounded native law uses its analytic I derivative.
        bias_dot = (np.zeros(3) if force_hold else
                    (bias-previous_force)/dt if math.isfinite(force_limit) or math.isfinite(force_rate)
                    else self.config.mass * (pid_dot-pd_dot))
        self.transport_full_pid_force_previous = bias.copy()
        result = dict(pd_force=self.config.mass*pd, pd_force_dot=self.config.mass*pd_dot,
            feedback_force=self.config.mass*pd+bias,
            feedback_force_dot=self.config.mass*pd_dot+bias_dot,
            native_pid_force=self.config.mass*pid, native_pid_force_dot=self.config.mass*pid_dot,
            native_extra_i=native_i, integral_force=bias, integral_force_dot=bias_dot,
            integral_state=self.transport_position_integral.copy(),
            integral_state_delta=self.transport_position_integral-previous_integral,
            integral_state_limit=integral_limit.copy(), integral_gain=ki.copy(),
            integral_gate_m=gate, integral_velocity_weight=integral_velocity_weight,
            native_gate_open=gate_open,
            integral_state_limit_active=bool(np.any(np.abs(self.transport_position_integral)>=integral_limit-1.e-12)),
            feedback_clamp_active=bool(np.any(np.abs(unclamped)>=feedback_limit)),
            force_budget_limited=force_limited, rate_budget_limited=rate_limited,
            learning=learning and not same_tick, duplicate_tick=same_tick, force_hold=force_hold,
            freeze_reason=reason or ("vehicle_derivatives" if not derivatives_ready else
                "force_hold" if force_hold else "duplicate_tick" if same_tick else
                "learning" if learning else "external_freeze"),
            hold_gain_scale=scale)
        self._transport_position_pid_tick = tick
        self._transport_position_pid_signature = signature
        self._transport_position_pid_cached = result
        return result

    def compute(self, state, target, dt):
        """计算一个 CTBR 命令。

        步骤为：根据位置误差得到期望 world 合力；将合力方向和 yaw 组装为期望姿态；
        再用 SO(3) 姿态误差得到机体系角速度。总推力投影到当前机体 z 轴后发送，
        因而姿态尚未完全跟踪时不会错误地把 world 合力模长直接当作电机推力。
        """
        dt = float(np.clip(dt, 0.001, 0.05))
        position_error = state["position"] - target["position"]
        source_derivatives_valid = bool(state.get("derivatives_valid", True))
        filter_derivatives_valid = bool(
            state.get("filter_derivatives_valid", source_derivatives_valid)
        )
        derivatives_ready = source_derivatives_valid and filter_derivatives_valid
        if derivatives_ready:
            measured_velocity = np.asarray(
                state.get(
                    "control_velocity",
                    state.get("filtered_velocity", state["velocity"]),
                ), dtype=float
            ).reshape(3)
            if np.all(np.isfinite(measured_velocity)):
                velocity_error = measured_velocity - target["velocity"]
            else:
                derivatives_ready = False
                velocity_error = np.zeros(3)
        else:
            # 滤波器预热或 Nokov 导数无效时，速度是未知量而不是零速度。暂时
            # 关闭速度反馈，避免圆周阶段把 -v_d 误当成真实速度误差并积分进去。
            velocity_error = np.zeros(3)

        independent_mode = bool(target.get("independent_mode", False))
        transport_mode = bool(target.get("transport_mode", False))
        native_transport_pid = (transport_mode and not independent_mode and
            target.get("transport_feedback_controller", "differential_integral") == "native_position_pid")
        if not native_transport_pid:
            self.reset_transport_position_pid()
        prelift_vertical_integral = independent_mode and bool(
            target.get("prelift_vertical_integral_active", False)
        )
        landing_vertical_integral = independent_mode and bool(
            target.get("landing_vertical_integral_active", False)
        )
        vertical_integral_active = prelift_vertical_integral or landing_vertical_integral
        transport_blend = float(np.clip(target.get("transport_blend", 1.0), 0.0, 1.0))
        if transport_mode:
            # MATLAB slung-load mode supplies a complete per-vehicle force.
            # The independent position PID integral is held during handoff.
            position_integral_rate = np.zeros(3)
            saturated_integral_rate = np.zeros(3)
        elif independent_mode:
            saved_vertical_integral = float(self.position_integral[2])
            independent_integral_gate = float(
                getattr(self.config, "independent_integral_gate", 0.05)
            )
            independent_integral_rate = velocity_error + 0.5 * position_error
            independent_integral_limit = np.asarray(
                getattr(self.config, "independent_integral_limit", None)
                if getattr(self.config, "independent_integral_limit", None) is not None
                else self.config.integral_limit,
                dtype=float,
            ).reshape(3)
            self.position_integral, _, gate_open = position_integral_step(
                self.position_integral, position_error, velocity_error, dt,
                independent_integral_gate, independent_integral_limit)
            if gate_open:
                position_integral_rate = independent_integral_rate
            else:
                # Match MATLAB's anti-windup gate during the large SLACK/TAKEUP move.
                position_integral_rate = np.zeros(3)
            if vertical_integral_active:
                # Z has its own bounded update during prelift/post-release
                # landing. XY keeps the original MATLAB error gate/limits.
                self.position_integral[2] = saved_vertical_integral
                position_integral_rate[2] = 0.0
            saturated_integral_rate = position_integral_rate.copy()
        else:
            # C1 形式积分同时吸收位置静差与速度静差；逐轴限幅避免低电、饱和或丢帧时
            # 长期累积，恢复后出现突然的大推力。保留积分器的有效导数，供 A_dot 使用。
            position_integral_rate = (
                velocity_error + self.config.position_integral_c1 * position_error
            )
            self.position_integral += dt * position_integral_rate
            self.position_integral = clamp_vector(
                self.position_integral,
                -self.config.integral_limit,
                self.config.integral_limit,
            )
            saturated_integral_rate = position_integral_rate.copy()
            upper_active = np.logical_and(
                self.position_integral >= self.config.integral_limit - 1e-12,
                position_integral_rate > 0.0,
            )
            lower_active = np.logical_and(
                self.position_integral <= -self.config.integral_limit + 1e-12,
                position_integral_rate < 0.0,
            )
            saturated_integral_rate[upper_active | lower_active] = 0.0

        target_acceleration = np.asarray(
            target.get("acceleration", np.zeros(3)), dtype=float
        ).reshape(3).copy()
        target_jerk = np.asarray(
            target.get("jerk", np.zeros(3)), dtype=float
        ).reshape(3).copy()
        current_acceleration = np.asarray(
            state.get(
                "control_acceleration",
                state.get("filtered_acceleration", state.get("acceleration", target_acceleration)),
            ),
            dtype=float,
        ).reshape(3)
        if (not derivatives_ready or
                not np.all(np.isfinite(current_acceleration))):
            # V2 falls back to a_d until both differentiators have enough history.
            current_acceleration = target_acceleration.copy()
        if not np.all(np.isfinite(target_jerk)):
            target_jerk = np.zeros(3)
        position_gain = self.config.position_gain * float(
            target.get("position_gain_scale", 1.0)
        )
        velocity_gain = self.config.velocity_gain * float(
            target.get("velocity_gain_scale", 1.0)
        )

        independent_position_gain = np.asarray(
            getattr(self.config, "independent_position_gain", None)
            if getattr(self.config, "independent_position_gain", None) is not None
            else self.config.position_gain,
            dtype=float,
        ).reshape(3).copy()
        independent_position_gain *= np.asarray(
            target.get("independent_position_gain_scale", 1.0), dtype=float
        )
        independent_velocity_gain = np.asarray(
            getattr(self.config, "independent_velocity_gain", None)
            if getattr(self.config, "independent_velocity_gain", None) is not None
            else self.config.velocity_gain,
            dtype=float,
        ).reshape(3).copy()
        independent_velocity_gain *= np.asarray(
            target.get("independent_velocity_gain_scale", 1.0), dtype=float
        )
        independent_integral_gain = np.asarray(
            getattr(self.config, "independent_integral_gain", None)
            if getattr(self.config, "independent_integral_gain", None) is not None
            else self.config.integral_gain / max(self.config.mass, 1.0e-12),
            dtype=float,
        ).reshape(3)
        if independent_mode:
            max_feedback = getattr(self.config, "independent_max_feedback_acceleration", None)
            max_feedback = (np.full(3, np.inf) if max_feedback is None
                            else np.asarray(max_feedback, dtype=float).reshape(3).copy())
            if vertical_integral_active:
                vertical_prefix = "prelift" if prelift_vertical_integral else "landing"
                max_feedback[2] = float(target[vertical_prefix + "_vertical_feedback_limit_mps2"])
                if derivatives_ready and abs(position_error[2]) <= float(
                        target.get(vertical_prefix + "_vertical_integral_gate_m", math.inf)):
                    integral_rate_z = float(np.clip(
                        velocity_error[2] + 0.5 * position_error[2],
                        -float(target[vertical_prefix + "_vertical_integral_rate_limit"]),
                        float(target[vertical_prefix + "_vertical_integral_rate_limit"]),
                    ))
                    candidate = self.position_integral.copy()
                    limit = float(target[vertical_prefix + "_vertical_integral_limit"])
                    candidate[2] = np.clip(candidate[2] + dt * integral_rate_z, -limit, limit)
                    candidate_feedback = (
                        -independent_position_gain * position_error
                        -independent_velocity_gain * velocity_error
                        -independent_integral_gain * candidate
                    )
                    delta_feedback_z = -independent_integral_gain[2] * (
                        candidate[2] - self.position_integral[2]
                    )
                    candidate_force = self.config.mass * (
                        candidate_feedback + target_acceleration
                        + np.array([0.0, 0.0, self.config.gravity])
                    )
                    thrust_axis_z = float(state["rotation"][2, 2])
                    projected_force = float(candidate_force @ state["rotation"][:, 2])
                    delta_thrust = self.config.mass * delta_feedback_z * thrust_axis_z
                    lower_thrust_limit = (max(0.0, float(target.get("min_collective_thrust", 0.0)))
                                          if landing_vertical_integral else 0.0)
                    winds_up = (
                        (candidate_feedback[2] >= max_feedback[2] and delta_feedback_z > 0.0)
                        or (candidate_feedback[2] <= -max_feedback[2] and delta_feedback_z < 0.0)
                        or (projected_force >= self.config.max_command_thrust and delta_thrust > 0.0)
                        or (projected_force <= lower_thrust_limit and delta_thrust < 0.0)
                    )
                    if not winds_up:
                        self.position_integral[2] = candidate[2]
            feedback_acceleration, _, _ = position_pid_feedback(
                position_error, velocity_error, independent_position_gain,
                independent_velocity_gain, independent_integral_gain,
                self.position_integral, max_feedback)
            raw_desired_force = self.config.mass * (
                feedback_acceleration + target_acceleration +
                np.array([0.0, 0.0, self.config.gravity])
            )
            raw_desired_force[2] += float(target.get("landing_support_trim_n", 0.0))
            transport_blend = 0.0
        # ROS world 系按 z 向上处理：F = m(g e3 + a_d) - Kp ep - Kv ev - Ki ei。
        hold_acceleration = np.zeros(3)
        hold_acceleration_dot = np.zeros(3)
        hold_feedback_limit_active = False
        transport_integral_force = np.zeros(3)
        transport_integral_force_dot = np.zeros(3)
        transport_full_pid_force = np.zeros(3)
        transport_full_pid_force_dot = np.zeros(3)
        transport_full_pid_pd_force = np.zeros(3)
        transport_full_pid_pd_force_dot = np.zeros(3)
        native_pid_info = None
        blend_force_delta = np.zeros(3)
        force_override = target.get("desired_force_override")
        if independent_mode:
            pass
        elif force_override is not None:
            transport_force = np.asarray(force_override, dtype=float).reshape(3)
            if native_transport_pid:
                native_pid_info = self._native_transport_feedback(
                    state, target, dt, derivatives_ready,
                    measured_velocity if derivatives_ready else np.zeros(3),
                    independent_position_gain, independent_velocity_gain,
                    independent_integral_gain)
                transport_full_pid_force = native_pid_info["integral_force"]
                transport_full_pid_force_dot = native_pid_info["integral_force_dot"]
                weight = float(np.clip(target.get("transport_full_pid_weight", 1.), 0., 1.))
                weight_dot = float(target.get("transport_full_pid_weight_dot", 0.))
                if not math.isfinite(weight_dot):
                    weight_dot = 0.
                transport_full_pid_force_dot = (
                    weight * transport_full_pid_force_dot + weight_dot * transport_full_pid_force)
                transport_full_pid_force = weight * transport_full_pid_force
                transport_full_pid_pd_force = native_pid_info["pd_force"]
                transport_full_pid_pd_force_dot = native_pid_info["pd_force_dot"]
                transport_force = transport_force + transport_full_pid_pd_force + transport_full_pid_force
                hold_feedback_limit_active = native_pid_info["feedback_clamp_active"]
            elif transport_mode:
                transport_integral_force = as_vector(
                    target.get("transport_hold_integral_force", np.zeros(3)),
                    "transport_hold_integral_force")
                transport_integral_force_dot = as_vector(
                    target.get("transport_hold_integral_force_dot", np.zeros(3)),
                    "transport_hold_integral_force_dot")
                transport_force = transport_force + transport_integral_force
            independent_force = (
                self.config.mass * (
                    target_acceleration + np.array([0.0, 0.0, self.config.gravity])
                )
                - position_gain * position_error
                - velocity_gain * velocity_error
                - self.config.integral_gain * self.position_integral
            )
            if "handoff_force" in target:
                # Reuse the PID output that was supporting the prelifted load.
                independent_force = np.asarray(target["handoff_force"], dtype=float).reshape(3)
            raw_desired_force = (
                (1.0 - transport_blend) * independent_force
                + transport_blend * transport_force
            )
            blend_force_delta = transport_force - independent_force
            # Match Udaan's TENSION_RAMP handoff: the cooperative force is
            # blended in, but an independent position-hold term remains active
            # so the aircraft stay at their measured TAKEUP anchors.  Without
            # this term, horizontal cable forces can pull all three aircraft
            # toward the payload center while tension is being established.
            # During the ramp use the Udaan handoff gain.  After the ramp,
            # keep the same independent hold around the translated TAKEUP
            # anchor while the payload reference is lifted; otherwise a cable
            # direction/model error can pull CF4/CF5 inward during hover.
            if transport_blend < 1.0:
                hold_scale = float(target.get(
                    "ramp_position_hold_gain_scale", 0.0
                ))
            else:
                hold_scale = float(target.get(
                    "transport_position_hold_gain_scale", 0.0
                ))
            if (not native_transport_pid and transport_blend > 0.0 and math.isfinite(hold_scale) and
                    hold_scale > 0.0):
                hold_position_error = state["position"] - np.asarray(
                    target.get("position_hold_position", target["position"]), dtype=float
                )
                hold_velocity_error = measured_velocity - np.asarray(
                    target.get("position_hold_velocity", target["velocity"]), dtype=float
                ) if derivatives_ready else np.zeros(3)
                hold_acceleration = hold_scale * (
                    -independent_position_gain * hold_position_error
                    -independent_velocity_gain * hold_velocity_error
                )
                hold_acceleration_dot = hold_scale * (
                    -independent_position_gain * hold_velocity_error
                    -independent_velocity_gain * (current_acceleration - np.asarray(
                        target.get("position_hold_acceleration", target_acceleration)
                    ))
                )
                hold_scale_dot = (float(target.get(
                    "transport_position_hold_gain_scale_dot", 0.0
                )) if transport_blend >= 1.0 else 0.0)
                hold_acceleration_dot += hold_scale_dot * (
                    -independent_position_gain * hold_position_error
                    -independent_velocity_gain * hold_velocity_error
                )
                max_feedback = getattr(
                    self.config, "independent_max_feedback_acceleration", None
                )
                if max_feedback is not None:
                    max_feedback = np.asarray(max_feedback, dtype=float).reshape(3)
                    unsaturated = np.abs(hold_acceleration) < max_feedback
                    hold_feedback_limit_active = bool(np.any(np.abs(hold_acceleration) >= max_feedback))
                    hold_acceleration = clamp_vector(hold_acceleration, -max_feedback, max_feedback)
                    hold_acceleration_dot = np.where(
                        unsaturated, hold_acceleration_dot, 0.0
                    )
                raw_desired_force += (
                    transport_blend * self.config.mass * hold_acceleration
                )
        else:
            transport_blend = 0.0
            raw_desired_force = (
                self.config.mass * (
                    target_acceleration + np.array([0.0, 0.0, self.config.gravity])
                )
                - position_gain * position_error
                - velocity_gain * velocity_error
                - self.config.integral_gain * self.position_integral
            )
        force_dot_override = target.get("desired_force_dot_override")
        if independent_mode:
            raw_desired_force_dot = self.config.mass * target_jerk
            raw_desired_force_dot[2] += float(target.get("landing_support_trim_dot_nps", 0.0))
        elif force_dot_override is not None or native_pid_info is not None:
            transport_force_dot = np.asarray(
                np.zeros(3) if force_dot_override is None else force_dot_override, dtype=float
            ).reshape(3) + transport_integral_force_dot
            if native_pid_info is not None:
                transport_force_dot = (transport_force_dot + transport_full_pid_pd_force_dot
                                       + transport_full_pid_force_dot)
            independent_force_dot = (
                self.config.mass * target_jerk
                - position_gain * velocity_error
                - velocity_gain * (current_acceleration - target_acceleration)
                - self.config.integral_gain * saturated_integral_rate
            )
            if "handoff_force" in target:
                independent_force_dot = np.zeros(3)
            raw_desired_force_dot = (
                (1.0 - transport_blend) * independent_force_dot
                + transport_blend * transport_force_dot
            )
        else:
            raw_desired_force_dot = (
                self.config.mass * target_jerk
                - position_gain * velocity_error
                - velocity_gain * (current_acceleration - target_acceleration)
                - self.config.integral_gain * saturated_integral_rate
            )
        transport_blend_dot = float(target.get("transport_blend_dot", 0.0))
        if math.isfinite(transport_blend_dot):
            raw_desired_force_dot += (
                transport_blend_dot * blend_force_delta
                + self.config.mass * (
                    transport_blend * hold_acceleration_dot
                    + transport_blend_dot * hold_acceleration
                )
            )

        # 先限制竖直推力，再依照最大倾角限制水平合力。水平力上限依赖当前可用的
        # 竖直力，确保大位置误差不会要求接近 90 deg 的危险倾斜；同时求该限幅的导数，
        # 使解析 Omega_c 与实际用于构造姿态的合力方向一致。
        max_tilt_rad = float(target.get("max_tilt_rad", self.config.max_tilt_rad))
        desired_force, desired_force_dot = _limit_force_with_derivative(
            raw_desired_force,
            raw_desired_force_dot,
            self.config.max_command_thrust,
            max_tilt_rad,
        )
        # Check the limiter itself, before the intentional force filter lag.
        force_limit_active = not np.allclose(
            raw_desired_force, desired_force, rtol=0.0, atol=1.0e-10)
        force_filter_tau = float(getattr(self.config, "transport_force_filter_tau_s", 0.0))
        if transport_mode and force_filter_tau > 0.0:
            force_filter_reset = bool(target.get("transport_filter_reset", False))
            filter_was_reset = force_filter_reset or self.previous_transport_force is None
            if filter_was_reset:
                self.previous_transport_force = desired_force.copy()
            else:
                previous_filtered_force = self.previous_transport_force.copy()
                alpha = float(np.clip(dt / (force_filter_tau + dt), 0.0, 1.0))
                self.previous_transport_force += alpha * (
                    desired_force - self.previous_transport_force
                )
            desired_force = self.previous_transport_force.copy()
            # Keep force direction, thrust projection and Omega_c consistent.
            # The force filter is intentionally separate from collective thrust.
            desired_force_dot = (
                np.zeros(3) if filter_was_reset else
                (desired_force - previous_filtered_force) / dt
            )
        elif not transport_mode:
            self.previous_transport_force = None

        force_norm = float(np.linalg.norm(desired_force))
        if force_norm < float(self.config.omega_c_force_norm_epsilon):
            b3_desired = (
                np.array([0.0, 0.0, 1.0])
                if self.previous_computed_rotation is None
                else self.previous_computed_rotation[:, 2]
            )
        else:
            b3_desired = desired_force / force_norm

        # 期望机体 z 轴与期望合力同向。以 yaw 方向为 x 轴参考，正交化后构造完整
        # 右手期望姿态；当两者近似共线时回退到上一帧航向，避免数值奇异。
        # MATLAB's independent controller uses a fixed [1, 0, 0] heading
        # because its simulation initializes every vehicle at yaw zero.  On a
        # real fleet the three vehicles can have different start yaws.  The
        # trajectory already stores that yaw, so use it for the independent
        # takeoff/TAKEUP reference; otherwise a large yaw correction saturates
        # the rate command and couples into roll/pitch before lift-off.  An
        # explicit per-target heading still has priority, followed by the
        # configured MATLAB fallback for callers without a yaw target.
        independent_heading = target.get("independent_heading")
        if independent_mode and independent_heading is None and "yaw" not in target:
            independent_heading = getattr(self.config, "independent_heading", None)
        if independent_mode and independent_heading is not None:
            b1_reference = np.asarray(independent_heading, dtype=float).reshape(3)
            b1_reference = b1_reference / max(float(np.linalg.norm(b1_reference)), 1.0e-12)
        else:
            desired_yaw = float(target.get("yaw", 0.0))
            b1_reference = np.array([math.cos(desired_yaw), math.sin(desired_yaw), 0.0])
        b1_projection = b1_reference - b3_desired * float(b3_desired @ b1_reference)
        if (np.linalg.norm(b1_projection) <
                float(self.config.omega_c_heading_projection_epsilon)):
            b1_projection = (
                np.array([1.0, 0.0, 0.0])
                if self.previous_heading is None
                else self.previous_heading
            )
        b1_desired = b1_projection / max(float(np.linalg.norm(b1_projection)), 1e-12)
        b2_desired = np.cross(b3_desired, b1_desired)
        b2_desired /= max(float(np.linalg.norm(b2_desired)), 1e-12)
        b1_desired = np.cross(b2_desired, b3_desired)
        b1_desired /= max(float(np.linalg.norm(b1_desired)), 1e-12)
        computed_rotation = project_so3(np.column_stack((b1_desired, b2_desired, b3_desired)))

        # V2 的 analytic 方法沿 A_dot -> b3_dot -> Rc_dot 链直接计算 Omega_c；
        # log_difference 保留用于与旧 MATLAB/Python 日志做离散对比。
        omega_c_method = str(self.config.omega_c_method).strip().lower()
        if independent_mode:
            computed_body_rate = np.zeros(3)
            omega_c_method = "independent_matlab"
        elif transport_mode:
            # The MATLAB transport implementation sets Omega_ic=0 and sends
            # -kR*eR-kOmega*Omega_i to the Crazyflie rate loop.
            computed_body_rate = np.zeros(3)
            omega_c_method = "slung_load_matlab"
        elif omega_c_method in ("log_difference", "log", "so3_log"):
            if self.previous_computed_rotation is None:
                computed_body_rate = np.zeros(3)
            else:
                relative_rotation = self.previous_computed_rotation.T @ computed_rotation
                computed_body_rate = so3_log(relative_rotation) / dt
            omega_c_method = "log_difference"
        elif omega_c_method in ("analytic", "derivative"):
            computed_body_rate = analytic_omega_c(
                computed_rotation,
                desired_force,
                desired_force_dot,
                target,
                self.config,
            )
            omega_c_method = "analytic"
        else:
            raise ValueError(
                "未知 omega_c_method：%s；可选 analytic 或 log_difference" %
                self.config.omega_c_method
            )
        self.previous_computed_rotation = computed_rotation
        self.previous_heading = computed_rotation[:, 0]

        rotation = state["rotation"]
        attitude_error = 0.5 * vee(
            computed_rotation.T @ rotation - rotation.T @ computed_rotation
        )
        if independent_mode:
            independent_attitude_gain = np.asarray(
                getattr(self.config, "independent_attitude_gain", None)
                if getattr(self.config, "independent_attitude_gain", None) is not None
                else self.config.attitude_gain,
                dtype=float,
            ).reshape(3)
            body_rate_command = -independent_attitude_gain * attitude_error
        elif transport_mode:
            transport_attitude_gain = np.asarray(
                target.get("transport_attitude_gain", self.config.attitude_gain),
                dtype=float,
            ).reshape(3)
            transport_rate_gain = np.asarray(
                target.get("transport_rate_gain", np.zeros(3)),
                dtype=float,
            ).reshape(3)
            transport_body_rate_command = (
                -transport_attitude_gain * attitude_error
                -transport_rate_gain * np.asarray(
                    state.get("body_rate", np.zeros(3)), dtype=float
                ).reshape(3)
            )
            if transport_blend < 1.0:
                if self.config.use_body_rate_feedforward:
                    computed_body_rate_current_frame = (
                        rotation.T @ computed_rotation @ computed_body_rate
                    )
                else:
                    computed_body_rate_current_frame = np.zeros(3)
                independent_body_rate_command = (
                    computed_body_rate_current_frame
                    - self.config.attitude_gain * attitude_error
                    - self.config.attitude_integral_gain * self.attitude_integral
                )
                if "handoff_force" in target:
                    gain = (self.config.independent_attitude_gain
                            if self.config.independent_attitude_gain is not None
                            else self.config.attitude_gain)
                    independent_body_rate_command = -np.asarray(gain) * attitude_error
                body_rate_command = (
                    (1.0 - transport_blend) * independent_body_rate_command
                    + transport_blend * transport_body_rate_command
                )
            else:
                body_rate_command = transport_body_rate_command
        else:
            self.attitude_integral += dt * attitude_error
            self.attitude_integral = clamp_vector(
                self.attitude_integral,
                -self.config.attitude_integral_limit,
                self.config.attitude_integral_limit,
            )
            # 该开关只控制是否把 Omega_c 前馈送入 CTBR；解析结果仍会保留在返回值和日志中。
            if self.config.use_body_rate_feedforward:
                computed_body_rate_current_frame = rotation.T @ computed_rotation @ computed_body_rate
            else:
                computed_body_rate_current_frame = np.zeros(3)
            body_rate_command = (
                computed_body_rate_current_frame
                - self.config.attitude_gain * attitude_error
                - self.config.attitude_integral_gain * self.attitude_integral
            )
        body_rate_limit = self.config.max_body_rate
        if independent_mode and getattr(self.config, "independent_max_body_rate", None) is not None:
            body_rate_limit = np.asarray(
                self.config.independent_max_body_rate, dtype=float
            ).reshape(3)
        raw_body_rate_command = np.asarray(body_rate_command, dtype=float).copy()
        body_rate_limit_active = bool(np.any(np.abs(raw_body_rate_command) >= body_rate_limit))
        filter_tau = float(getattr(self.config, "transport_body_rate_filter_tau_s", 0.0))
        filter_reset = bool(target.get("transport_filter_reset", False))
        if transport_mode and filter_tau > 0.0:
            if filter_reset or self.previous_transport_body_rate_command is None:
                self.previous_transport_body_rate_command = raw_body_rate_command.copy()
            else:
                alpha = float(np.clip(dt / (filter_tau + dt), 0.0, 1.0))
                self.previous_transport_body_rate_command += alpha * (
                    raw_body_rate_command - self.previous_transport_body_rate_command
                )
            body_rate_command = self.previous_transport_body_rate_command.copy()
        else:
            body_rate_command = raw_body_rate_command
            if not transport_mode:
                self.previous_transport_body_rate_command = None
        body_rate_limit_active |= bool(np.any(np.abs(body_rate_command) >= body_rate_limit))
        body_rate_command = clamp_vector(
            body_rate_command, -body_rate_limit, body_rate_limit
        )

        # 在起飞阶段可施加最小总推力，克服地面静摩擦和小幅估计误差；其他阶段不使用
        # 该下限，特别是降落阶段必须允许推力降低。
        minimum_thrust = float(target.get("min_collective_thrust", 0.0))
        projected_thrust = float(desired_force @ rotation[:, 2])
        collective_limit_active = (
            projected_thrust <= max(0.0, minimum_thrust)
            or projected_thrust >= self.config.max_command_thrust)
        collective_thrust = float(np.clip(
            max(projected_thrust, minimum_thrust),
            0.0, self.config.max_command_thrust
        ))
        return {
            "position_error": position_error,
            "velocity_error": velocity_error,
            "desired_force": desired_force,
            "raw_desired_force": raw_desired_force,
            "desired_force_dot": desired_force_dot,
            "computed_rotation": computed_rotation,
            "computed_body_rate": computed_body_rate,
            "omega_c_method": omega_c_method,
            "attitude_error": attitude_error,
            "body_rate_command": body_rate_command,
            "raw_body_rate_command": raw_body_rate_command,
            "collective_thrust": collective_thrust,
            # 诊断用：记录本周期积分器状态，便于区分推力标定误差和积分器被重置。
            "position_integral": self.position_integral.copy(),
            "transport_mode": transport_mode,
            "independent_mode": independent_mode,
            "transport_feedback_controller": target.get(
                "transport_feedback_controller", "differential_integral"),
            "force_limit_active": force_limit_active,
            "hold_feedback_limit_active": hold_feedback_limit_active,
            "body_rate_limit_active": body_rate_limit_active,
            "collective_limit_active": collective_limit_active,
            "transport_hold_integral_force": transport_blend * transport_integral_force,
            "transport_hold_integral_force_dot": (
                transport_blend * transport_integral_force_dot
                + (transport_blend_dot if math.isfinite(transport_blend_dot) else 0.0)
                * transport_integral_force),
            "transport_full_pid_force": transport_blend * transport_full_pid_force,
            "transport_full_pid_force_dot": (
                transport_blend * transport_full_pid_force_dot
                + (transport_blend_dot if math.isfinite(transport_blend_dot) else 0.) * transport_full_pid_force),
            "transport_full_pid_pd_force": transport_blend * transport_full_pid_pd_force,
            "transport_full_pid_pd_force_dot": (
                transport_blend * transport_full_pid_pd_force_dot
                + (transport_blend_dot if math.isfinite(transport_blend_dot) else 0.) * transport_full_pid_pd_force),
            "transport_full_pid_feedback_force": (
                transport_blend * (transport_full_pid_pd_force + transport_full_pid_force)),
            "transport_full_pid_feedback_force_dot": (
                transport_blend * (transport_full_pid_pd_force_dot + transport_full_pid_force_dot)
                + (transport_blend_dot if math.isfinite(transport_blend_dot) else 0.)
                * (transport_full_pid_pd_force + transport_full_pid_force)),
            "transport_full_pid_native_force": (
                transport_blend * native_pid_info["native_pid_force"]
                if native_pid_info is not None else np.zeros(3)),
            "transport_full_pid_info": native_pid_info,
            "transport_full_pid_integral_state": self.transport_position_integral.copy(),
            "transport_full_pid_integral_state_delta": (
                native_pid_info["integral_state_delta"] if native_pid_info is not None else np.zeros(3)),
            "transport_full_pid_integral_state_limit": (
                native_pid_info["integral_state_limit"] if native_pid_info is not None else np.zeros(3)),
            "transport_full_pid_integral_gain": (
                native_pid_info["integral_gain"] if native_pid_info is not None else np.zeros(3)),
            "transport_full_pid_integral_velocity_weight": (
                native_pid_info["integral_velocity_weight"] if native_pid_info is not None else 0.),
            "transport_full_pid_learning": bool(native_pid_info and native_pid_info["learning"]),
            "transport_full_pid_freeze_reason": (
                native_pid_info["freeze_reason"] if native_pid_info is not None else "inactive"),
            "transport_full_pid_duplicate_tick": bool(native_pid_info and native_pid_info["duplicate_tick"]),
            "transport_full_pid_integral_gate_m": (
                native_pid_info["integral_gate_m"] if native_pid_info is not None else 0.),
            "transport_full_pid_gate_open": bool(native_pid_info and native_pid_info["native_gate_open"]),
            "transport_full_pid_integral_limit_active": bool(native_pid_info and native_pid_info["integral_state_limit_active"]),
            "transport_full_pid_feedback_limit_active": bool(native_pid_info and native_pid_info["feedback_clamp_active"]),
            "transport_full_pid_force_budget_limited": bool(native_pid_info and native_pid_info["force_budget_limited"]),
            "transport_full_pid_rate_budget_limited": bool(native_pid_info and native_pid_info["rate_budget_limited"]),
            "transport_position_hold_force": (
                transport_blend * (transport_full_pid_pd_force
                                   if native_pid_info is not None else self.config.mass * hold_acceleration)
            ),
            "prelift_vertical_integral_active": prelift_vertical_integral and derivatives_ready,
            "prelift_vertical_integral_force_n": (
                -self.config.mass * independent_integral_gain[2] * self.position_integral[2]
                if prelift_vertical_integral else 0.0
            ),
            "landing_vertical_integral_active": landing_vertical_integral and derivatives_ready,
            "landing_vertical_integral_force_n": (
                -self.config.mass * independent_integral_gain[2] * self.position_integral[2]
                if landing_vertical_integral else 0.0
            ),
        }


class FlightCsvLogger:
    """以固定列、线程安全方式写入一次飞行的 CSV。

    控制定时器和 ROS 回调可能并发运行，因此文件写入使用独立锁。每 25 行 flush 一次，
    既减少磁盘开销，也尽量保留异常中断前的数据。字段名是可视化脚本与后续 MATLAB/Python
    分析的稳定接口，新字段只能追加而不应重命名旧字段。
    """

    FIELDS = [
        "control_time_s", "ros_time_s", "mode", "flight_phase", "state_valid",
        "derivatives_valid", "state_age_s",
        # 来自 /cf<ID>/battery（GenericLogData）的固件主电池遥测；无样本时写 NaN/-1。
        "battery_voltage_v", "battery_voltage_mv", "battery_state", "battery_level_percent",
        "battery_age_s",
        # 起飞前冻结的电压标定结果。raw scale 只影响 C++ 桥接的 PWM 映射，不改变
        # command_thrust_newton 的物理单位含义。
        "preflight_voltage_v", "preflight_voltage_sample_count",
        "preflight_voltage_ready", "preflight_voltage_failed", "thrust_raw_scale",
        "position_x", "position_y", "position_z",
        "velocity_x", "velocity_y", "velocity_z",
        "acceleration_x", "acceleration_y", "acceleration_z",
        "body_rate_x", "body_rate_y", "body_rate_z",
        "roll_rad", "pitch_rad", "yaw_rad",
        "target_x", "target_y", "target_z", "target_yaw_rad",
        "target_jerk_x", "target_jerk_y", "target_jerk_z",
        "position_error_x", "position_error_y", "position_error_z",
        "velocity_error_x", "velocity_error_y", "velocity_error_z",
        "desired_force_x", "desired_force_y", "desired_force_z",
        "desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad",
        "attitude_error_x", "attitude_error_y", "attitude_error_z",
        "computed_rate_x", "computed_rate_y", "computed_rate_z", "omega_c_method",
        "command_rate_x", "command_rate_y", "command_rate_z",
        "command_thrust_newton",
        # 仅追加诊断列，保持既有 MATLAB/CSV 按列号读取的布局不变。
        "filtered_velocity_x", "filtered_velocity_y", "filtered_velocity_z",
        "filtered_acceleration_x", "filtered_acceleration_y", "filtered_acceleration_z",
        "filter_derivatives_valid",
        "mocap_sample_age_s",
        # EKF 仅参与平移反馈；position/rotation 列仍来自 NOKOV。
        "ekf_position_x", "ekf_position_y", "ekf_position_z",
        "ekf_velocity_x", "ekf_velocity_y", "ekf_velocity_z",
        "ekf_acceleration_x", "ekf_acceleration_y", "ekf_acceleration_z",
        "ekf_state_valid", "ekf_state_age_s", "ekf_kinematics_weight_effective",
        "control_velocity_x", "control_velocity_y", "control_velocity_z",
        "mixed_acceleration_x", "mixed_acceleration_y", "mixed_acceleration_z",
        "control_acceleration_x", "control_acceleration_y", "control_acceleration_z",
        # Multi-vehicle task metadata.  Appended to preserve the legacy prefix.
        "mission_time_s", "vehicle_id", "radio_uri", "orbit_phase_rad",
        "formation_state", "global_abort_reason",
        # 诊断列只追加，避免破坏旧 MATLAB/CSV 的列号布局。
        "position_integral_x", "position_integral_y", "position_integral_z",
        "command_raw_thrust", "command_raw_thrust_age_s", "invalid_reason",
        # EKF 健康状态追加在末尾；不改变旧 CSV 的列号布局。
        "ekf_position_error_m", "ekf_position_consistent",
        "ekf_kinematics_held", "ekf_fault_age_s", "ekf_status",
        # Slung-load diagnostics appended after the legacy CSV schema.
        "payload_state_valid",
        "payload_position_error_x", "payload_position_error_y", "payload_position_error_z",
        "payload_velocity_error_x", "payload_velocity_error_y", "payload_velocity_error_z",
        "payload_attitude_error_x", "payload_attitude_error_y", "payload_attitude_error_z",
        "link_direction_x", "link_direction_y", "link_direction_z",
        "desired_link_direction_x", "desired_link_direction_y", "desired_link_direction_z",
        "link_direction_error_x", "link_direction_error_y", "link_direction_error_z",
        "desired_tension_n",
        # 负载原始/滤波状态；这些列在每架飞机行中重复，便于合并日志离线绘图。
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
        "transport_blend_dot",
        "transport_link_gain_scale", "ramp_position_hold_gain_scale",
        "transport_position_hold_gain_scale",
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
        # Wall-clock diagnostics only; previous-cycle fields refer to the
        # completed callback named by their control-time column.
        "controller_tick_interval_s", "controller_compute_wall_s",
        "previous_vehicle_cycle_wall_s", "previous_vehicle_cycle_control_time_s",
        "fleet_tick_interval_s", "previous_fleet_cycle_wall_s",
        "previous_fleet_cycle_control_time_s", "published_command_stamp_ns",
        "bridge_latest_command_stamp_ns", "bridge_latest_command_seq",
        "bridge_latest_command_age_s", "bridge_latest_callback_wall_s",
        "bridge_latest_send_call_wall_s", "bridge_latest_raw_thrust",
        "bridge_latest_accepted", "bridge_latest_unlock_packet",
        "bridge_latest_receipt_age_s", "bridge_latest_matches_published_stamp",
        "flight_validation_enabled", "flight_validation_elapsed_s",
        "flight_validation_landing_started",
        "flight_validation_landing_stage",
        "payload_ground_reference_z_m", "payload_landing_duration_s",
        "payload_landing_touchdown_confirmed", "payload_landing_unloading",
        "payload_landing_planning_failure_count", "payload_landing_planning_retry_count",
        "payload_landing_planning_failure_reason", "payload_landing_planning_fallback",
        "transport_support_requested_trim_n", "transport_support_trim_limit_n",
        "transport_support_initialization_clipped", "effective_transport_link_gain_scale",
        "raw_link_force_x", "raw_link_force_y", "raw_link_force_z",
        "limited_link_force_x", "limited_link_force_y", "limited_link_force_z",
        "kinematic_perpendicular_force_x", "kinematic_perpendicular_force_y",
        "kinematic_perpendicular_force_z", "link_force_rate_limited",
        "link_force_magnitude_limited", "payload_control_dt_s",
        "link_length_nominal_m", "link_length_effective_m", "link_length_measured_m",
        "link_calibration_enabled", "link_calibration_applied", "link_calibration_status",
        "link_calibration_reason", "link_calibration_samples", "link_calibration_window_s",
        "link_calibration_spread_m",
        "payload_roll_rad", "payload_pitch_rad", "payload_yaw_rad",
        "payload_target_roll_rad", "payload_target_pitch_rad", "payload_target_yaw_rad",
        "payload_target_body_rate_x", "payload_target_body_rate_y", "payload_target_body_rate_z",
        "payload_target_body_rate_dot_x", "payload_target_body_rate_dot_y", "payload_target_body_rate_dot_z",
        "position_hold_position_x", "position_hold_position_y", "position_hold_position_z",
        "position_hold_velocity_x", "position_hold_velocity_y", "position_hold_velocity_z",
        "position_hold_acceleration_x", "position_hold_acceleration_y", "position_hold_acceleration_z",
        "transport_position_hold_force_x", "transport_position_hold_force_y", "transport_position_hold_force_z",
        "payload_attitude_reference_progress",
        "payload_transport_force_mode",
        "kinematic_specific_acceleration_x", "kinematic_specific_acceleration_y", "kinematic_specific_acceleration_z",
        "landing_vertical_integral_active", "landing_vertical_integral_force_n",
        "transport_hold_integral_enabled", "transport_hold_integral_learning",
        "transport_hold_integral_freeze_reason",
        "transport_hold_integral_bias_x", "transport_hold_integral_bias_y", "transport_hold_integral_bias_z",
        "transport_hold_integral_error_x", "transport_hold_integral_error_y", "transport_hold_integral_error_z",
        "transport_hold_integral_force_x", "transport_hold_integral_force_y", "transport_hold_integral_force_z",
        "transport_hold_integral_force_dot_x", "transport_hold_integral_force_dot_y", "transport_hold_integral_force_dot_z",
        "transport_feedback_controller", "transport_full_pid_learning", "transport_full_pid_freeze_reason",
        "transport_full_pid_duplicate_tick", "transport_full_pid_integral_gate_m", "transport_full_pid_gate_open",
        "transport_full_pid_integral_limit_active", "transport_full_pid_feedback_limit_active",
        "transport_full_pid_force_budget_limited", "transport_full_pid_rate_budget_limited",
        "transport_full_pid_integral_state_x", "transport_full_pid_integral_state_y", "transport_full_pid_integral_state_z",
        "transport_full_pid_integral_state_delta_x", "transport_full_pid_integral_state_delta_y", "transport_full_pid_integral_state_delta_z",
        "transport_full_pid_integral_state_limit_x", "transport_full_pid_integral_state_limit_y", "transport_full_pid_integral_state_limit_z",
        "transport_full_pid_integral_gain_x", "transport_full_pid_integral_gain_y", "transport_full_pid_integral_gain_z",
        "transport_full_pid_force_x", "transport_full_pid_force_y", "transport_full_pid_force_z",
        "transport_full_pid_force_dot_x", "transport_full_pid_force_dot_y", "transport_full_pid_force_dot_z",
        "transport_full_pid_pd_force_x", "transport_full_pid_pd_force_y", "transport_full_pid_pd_force_z",
        "transport_full_pid_pd_force_dot_x", "transport_full_pid_pd_force_dot_y", "transport_full_pid_pd_force_dot_z",
        "transport_full_pid_feedback_force_x", "transport_full_pid_feedback_force_y", "transport_full_pid_feedback_force_z",
        "transport_full_pid_feedback_force_dot_x", "transport_full_pid_feedback_force_dot_y", "transport_full_pid_feedback_force_dot_z",
        "force_limit_active", "hold_feedback_limit_active", "body_rate_limit_active", "collective_limit_active",
        "independent_mode", "transport_mode",
        "transport_full_pid_integral_velocity_weight",
        "transport_attitude_trim_enabled", "transport_attitude_trim_learning",
        "transport_attitude_trim_freeze_reason", "transport_attitude_trim_limit_active",
        "transport_attitude_trim_rate_limit_active", "transport_attitude_trim_acceleration_limit_active",
        "transport_attitude_trim_error_x", "transport_attitude_trim_error_y", "transport_attitude_trim_error_z",
        "transport_attitude_trim_integral_x", "transport_attitude_trim_integral_y", "transport_attitude_trim_integral_z",
        "transport_attitude_trim_position_x", "transport_attitude_trim_position_y", "transport_attitude_trim_position_z",
        "transport_attitude_trim_velocity_x", "transport_attitude_trim_velocity_y", "transport_attitude_trim_velocity_z",
        "transport_attitude_trim_acceleration_x", "transport_attitude_trim_acceleration_y", "transport_attitude_trim_acceleration_z",
        "transport_attitude_trim_bridge_active",
        "transport_attitude_trim_bridge_position_x", "transport_attitude_trim_bridge_position_y", "transport_attitude_trim_bridge_position_z",
        "transport_attitude_trim_bridge_velocity_x", "transport_attitude_trim_bridge_velocity_y", "transport_attitude_trim_bridge_velocity_z",
        "transport_attitude_trim_bridge_acceleration_x", "transport_attitude_trim_bridge_acceleration_y", "transport_attitude_trim_bridge_acceleration_z",
        "transport_attitude_trim_bridge_hold_position_x", "transport_attitude_trim_bridge_hold_position_y", "transport_attitude_trim_bridge_hold_position_z",
        "transport_attitude_trim_bridge_hold_velocity_x", "transport_attitude_trim_bridge_hold_velocity_y", "transport_attitude_trim_bridge_hold_velocity_z",
        "transport_attitude_trim_bridge_hold_acceleration_x", "transport_attitude_trim_bridge_hold_acceleration_y", "transport_attitude_trim_bridge_hold_acceleration_z",
    ]

    def __init__(self, directory, prefix):
        directory = os.path.expanduser(directory)
        os.makedirs(directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(directory, "%s_%s.csv" % (prefix, timestamp))
        self.file = open(self.path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self._writes_since_flush = 0
        self._lock = threading.Lock()

    def write(self, row):
        with self._lock:
            if self.file.closed:
                return
            self.writer.writerow(row)
            self._writes_since_flush += 1
            if self._writes_since_flush >= 25:
                self.file.flush()
                self._writes_since_flush = 0

    def flush(self):
        """Flush a task-level log after a synchronized multi-vehicle tick."""
        with self._lock:
            if not self.file.closed:
                self.file.flush()
                self._writes_since_flush = 0

    def close(self):
        with self._lock:
            if not self.file.closed:
                self.file.flush()
                self.file.close()


class CtbrControllerNode:
    """ROS 封装层：状态接收、CTBR 发布、日志和轨迹模块适配。

    控制输出模式下的阶段顺序为：

    ``pre_takeoff_hold -> takeoff -> height_correction -> circle_entry
    -> circle -> final_hover -> landing
    -> landing_settle -> landed``。动捕长时间失效会转为 ``aborted`` 并保持零输出。

    具体阶段机和解析圆周参考位于 ``ctbr_trajectory.py``。本类不再实现起飞、圆周、
    悬停或降落的轨迹公式，只向该模块传入测量状态并把输出交给几何控制器。
    """

    def __init__(self, vehicle_config=None, logger=None, auto_timer=True,
                 register_shutdown=True, start_time=None):
        # crazyflies.yaml 是车辆身份的唯一来源。多机管理器会显式传入每架配置；
        # 直接实例化时保留旧的 ~cf_id/唯一 ctbr_enabled 选择行为。
        if vehicle_config is None:
            vehicle_entries = rospy.get_param("/crazyflies", [])
            requested_id = rospy.get_param("~cf_id", None)
            try:
                vehicle_config = select_vehicle_entry(
                    vehicle_entries, cf_id=requested_id
                )
            except (KeyError, TypeError, ValueError) as error:
                raise rospy.ROSInitException("飞机参数无效：%s" % error)
        try:
            self.vehicle_config = dict(vehicle_config)
            self.vehicle_id = int(self.vehicle_config["id"])
            self.radio_uri = vehicle_uri(self.vehicle_config)
            self.orbit_phase_rad = float(self.vehicle_config.get("orbit_phase_rad", 0.0))
            self.params = CtbrParameterResolver(
                rospy.get_param, rospy.has_param, self.vehicle_id
            )
        except (KeyError, TypeError, ValueError) as error:
            raise rospy.ROSInitException("飞机参数无效：%s" % error)
        self.auto_timer = bool(auto_timer)
        self.register_shutdown = bool(register_shutdown)
        self.logger = logger
        self.fleet_gate_open = bool(auto_timer)
        self.global_abort_reason = ""
        self.shared_start_time = None if start_time is None else float(start_time)
        self.mission_start_time = self.shared_start_time

        # 保留 ~cf_prefix 作为旧 launch 的兼容覆盖；新 launch 不再需要它。
        configured_prefix = str(rospy.get_param("~cf_prefix", "")).strip()
        self.prefix = (
            configured_prefix.rstrip("/")
            if configured_prefix
            else "/cf%d" % self.vehicle_id
        )
        if not bool(self.params.global_param("target_confirmed")):
            raise rospy.ROSInitException(
                "拒绝启动控制输出：请设置 ~target_confirmed:=true"
            )

        # 状态时效性与工作空间检查在控制发布之前执行；max_mocap_velocity_mps 同时
        # 限制 Nokov 差分尖峰进入速度反馈项。
        self.rate_hz = float(self.params.global_param("control_rate_hz"))
        self.state_timeout = float(self.params.global_param("state_timeout"))
        # These multipliers are consumed by this per-vehicle timer callback,
        # so they must be resolved on every CtbrControllerNode.  The fleet
        # manager also validates the same global values for early startup
        # feedback, but it is not the owner of a vehicle's callback state.
        self.takeoff_position_gain_scale = float(self.params.global_param(
            "takeoff_position_gain_scale", 1.0
        ))
        self.takeoff_velocity_gain_scale = float(self.params.global_param(
            "takeoff_velocity_gain_scale", 1.0
        ))
        if (not math.isfinite(self.takeoff_position_gain_scale) or
                self.takeoff_position_gain_scale < 1.0 or
                not math.isfinite(self.takeoff_velocity_gain_scale) or
                self.takeoff_velocity_gain_scale < 1.0):
            raise rospy.ROSInitException("起飞阶段增益倍率必须是不小于 1 的有限数")
        # ROS 的 mocap 回调和控制定时器在不同线程运行。多机模式中，若在整个
        # 车队周期开始时读取一次 ROS 时间，后续刚收到的一帧 header.stamp 可能比该
        # 旧时间晚几毫秒；这不是时间回退，不能据此切断推力。只容忍这种很短的
        # 调度差，超过该值仍按异常时间戳保护。
        self.mocap_timestamp_future_tolerance_s = float(self.params.global_param(
            "mocap_timestamp_future_tolerance_s", 0.02
        ))
        self.max_abs_position_m = float(self.params.global_param("max_abs_position_m"))
        self.max_mocap_velocity_mps = float(
            self.params.global_param("max_mocap_velocity_mps")
        )
        self.max_mocap_acceleration_mps2 = float(
            self.params.global_param("max_mocap_acceleration_mps2", 5.0)
        )
        if (self.rate_hz <= 0.0 or self.state_timeout <= 0.0 or
                not math.isfinite(self.mocap_timestamp_future_tolerance_s) or
                self.mocap_timestamp_future_tolerance_s < 0.0 or
                self.mocap_timestamp_future_tolerance_s > self.state_timeout or
                self.max_abs_position_m <= 0.0 or self.max_mocap_velocity_mps <= 0.0 or
                not math.isfinite(self.max_mocap_acceleration_mps2) or
                self.max_mocap_acceleration_mps2 <= 0.0):
            raise rospy.ROSInitException(
                "控制频率、动捕超时/时间戳容差、动捕边界、速度和加速度上限必须有效"
            )
        try:
            filter_cutoff_hz = float(self.params.global_param(
                "mocap_velocity_filter_cutoff_hz", 5.0
            ))
            filter_max_dt = float(self.params.global_param(
                "mocap_velocity_filter_max_dt", 0.05
            ))
            if filter_max_dt > self.state_timeout:
                raise ValueError(
                    "mocap_velocity_filter_max_dt 不能大于 state_timeout"
                )
            self.velocity_filter = SecondOrderVelocityFilter(
                cutoff_hz=filter_cutoff_hz,
                max_dt=filter_max_dt,
            )
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException("速度低通滤波参数无效：%s" % error)
        self.velocity_filter_lock = threading.Lock()

        # EKF 日志提供的平移状态独立滤波；NOKOV 仍是位置和姿态的唯一来源。
        self.ekf_kinematics_weight = float(self.params.global_param(
            "ekf_kinematics_weight", 0.0
        ))
        self.ekf_state_timeout = float(self.params.global_param(
            "ekf_state_timeout", 0.15
        ))
        self.ekf_max_position_delta_m = float(self.params.global_param(
            "ekf_max_position_delta_m", 0.30
        ))
        self.ekf_alignment_position_delta_m = float(self.params.global_param(
            "ekf_alignment_position_delta_m", 0.05
        ))
        self.ekf_alignment_hold_s = float(self.params.global_param(
            "ekf_alignment_hold_s", 1.0
        ))
        self.ekf_alignment_timeout_s = float(self.params.global_param(
            "ekf_alignment_timeout_s", 5.0
        ))
        self.ekf_kinematics_hold_s = float(self.params.global_param(
            "ekf_kinematics_hold_s", 0.10
        ))
        self.ekf_fault_emergency_land_s = float(self.params.global_param(
            "ekf_fault_emergency_land_s", 0.35
        ))
        self.ekf_timestamp_future_tolerance_s = float(self.params.global_param(
            "ekf_timestamp_future_tolerance_s", 0.02
        ))
        try:
            ekf_filter_cutoff_hz = float(self.params.global_param(
                "ekf_velocity_filter_cutoff_hz", filter_cutoff_hz
            ))
            ekf_filter_max_dt = float(self.params.global_param(
                "ekf_velocity_filter_max_dt", filter_max_dt
            ))
            if (not math.isfinite(self.ekf_kinematics_weight) or
                    not 0.0 <= self.ekf_kinematics_weight <= 1.0 or
                    not math.isfinite(self.ekf_state_timeout) or
                    self.ekf_state_timeout <= 0.0 or
                    self.ekf_state_timeout > self.state_timeout or
                    not math.isfinite(self.ekf_max_position_delta_m) or
                    self.ekf_max_position_delta_m <= 0.0 or
                    not math.isfinite(self.ekf_alignment_position_delta_m) or
                    self.ekf_alignment_position_delta_m <= 0.0 or
                    self.ekf_alignment_position_delta_m > self.ekf_max_position_delta_m or
                    not math.isfinite(self.ekf_alignment_hold_s) or
                    self.ekf_alignment_hold_s < 0.0 or
                    not math.isfinite(self.ekf_alignment_timeout_s) or
                    self.ekf_alignment_timeout_s <= 0.0 or
                    self.ekf_alignment_timeout_s < self.ekf_alignment_hold_s or
                    not math.isfinite(self.ekf_kinematics_hold_s) or
                    self.ekf_kinematics_hold_s < 0.0 or
                    not math.isfinite(self.ekf_fault_emergency_land_s) or
                    self.ekf_fault_emergency_land_s < self.ekf_kinematics_hold_s or
                    not math.isfinite(self.ekf_timestamp_future_tolerance_s) or
                    self.ekf_timestamp_future_tolerance_s < 0.0 or
                    self.ekf_timestamp_future_tolerance_s > self.ekf_state_timeout or
                    ekf_filter_max_dt > self.ekf_state_timeout):
                raise ValueError(
                    "EKF 权重、对齐/故障时间、位置差和滤波时间参数无效"
                )
            self.ekf_velocity_filter = SecondOrderVelocityFilter(
                cutoff_hz=ekf_filter_cutoff_hz,
                max_dt=ekf_filter_max_dt,
            )
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException("EKF 运动学参数无效：%s" % error)
        self.ekf_velocity_filter_lock = threading.Lock()
        # 短暂丢帧时暂停参考计时；超过此时长后不尝试在未知位置继续任务，而是锁定
        # aborted 并持续发送零 CTBR。该阈值必须大于 state_timeout，才允许一次短暂
        # 网络/动捕抖动恢复。
        self.trajectory_pause_abort_s = float(
            self.params.global_param("trajectory_pause_abort_s")
        )
        if self.trajectory_pause_abort_s <= self.state_timeout:
            raise rospy.ROSInitException(
                "trajectory_pause_abort_s 必须大于 state_timeout"
            )
        mass_kg = float(self.params.vehicle_param("mass_kg"))
        rospy.loginfo(
            "CTBR 参数：mass=%.4f kg，state_timeout=%.3f s，轨迹丢帧中止阈值=%.3f s",
            mass_kg,
            self.state_timeout,
            self.trajectory_pause_abort_s,
        )

        try:
            trajectory_param = self.params.trajectory_param
            vehicle_param = self.params.vehicle_param
            offset = np.array([
                float(trajectory_param("circle_center_offset_x")),
                float(trajectory_param("circle_center_offset_y")),
            ], dtype=float)
            if not np.all(np.isfinite(offset)):
                raise ValueError("circle_center_offset_x/y 必须是有限数值")
            configured_center = trajectory_param("circle_center_xy", None)
            circle_center_xy = None
            if configured_center is not None:
                circle_center_xy = np.asarray(configured_center, dtype=float).reshape(-1)
                if circle_center_xy.size != 2 or not np.all(np.isfinite(circle_center_xy)):
                    raise ValueError("circle_center_xy 必须是 2 个有限数值")
            takeoff_height_m = float(trajectory_param("takeoff_height_m"))
            transport_mode = str(self.params.global_param(
                "transport_mode", "formation"
            )).strip().lower().replace("-", "_")
            independent_takeoff_height_m = None
            independent_takeoff_duration_s = None
            takeup_duration_s = None
            if transport_mode == "slung_load":
                payload_param = rospy.get_param(PAYLOAD_PARAMETER_ROOT, {})
                # The trajectory position belongs to the aircraft.  For the
                # MATLAB cable geometry, the aircraft must be one link length
                # above the desired payload center before transport starts.
                configured_lengths = payload_param.get(
                    "link_lengths_m", payload_param.get("link_length_m")
                )
                if configured_lengths is not None:
                    takeoff_height_m += float(np.max(link_lengths_vector(
                        configured_lengths, "link_lengths_m"
                    )))
                independent_takeoff_height_m = float(
                    payload_param.get("independent_hover_height_m", 0.30)
                )
                independent_takeoff_duration_s = float(
                    payload_param.get("independent_takeoff_duration_s", 2.5)
                )
                takeup_duration_s = float(
                    payload_param.get("takeup_duration_s", 2.5)
                )
            self.trajectory_config = CircularTrajectoryConfig(
                reference_hold_s=float(trajectory_param("reference_hold_s")),
                takeoff_height_m=takeoff_height_m,
                takeoff_duration_s=float(trajectory_param("takeoff_duration_s")),
                takeoff_settle_s=float(trajectory_param("takeoff_settle_s")),
                takeoff_altitude_tolerance_m=float(
                    trajectory_param("takeoff_altitude_tolerance_m")
                ),
                takeoff_vertical_velocity_tolerance_mps=float(
                    trajectory_param("takeoff_vertical_velocity_tolerance_mps")
                ),
                circle_center_offset_xy=offset,
                circle_radius_m=float(trajectory_param("circle_radius_m")),
                circle_revolutions=float(trajectory_param("circle_revolutions")),
                # 该参数现在表示整圈五次角度轨迹的峰值角速度。
                circle_angular_speed_radps=float(
                    trajectory_param("circle_angular_speed_radps")
                ),
                circle_start_angle_rad=float(
                    trajectory_param("circle_start_angle_rad")
                ),
                entry_duration_s=float(trajectory_param("circle_entry_duration_s")),
                # 旧参数保留用于兼容已有 YAML，但整圈 smoothstep 不使用它。
                circle_ramp_duration_s=float(
                    trajectory_param("circle_ramp_duration_s")
                ),
                hover_duration_s=float(trajectory_param("hover_duration_s", 30.0)),
                independent_takeoff_height_m=independent_takeoff_height_m,
                independent_takeoff_duration_s=independent_takeoff_duration_s,
                takeup_duration_s=takeup_duration_s,
                final_hover_s=float(trajectory_param("final_hover_s")),
                landing_duration_s=float(trajectory_param("landing_duration_s")),
                landing_max_speed_mps=float(
                    trajectory_param("landing_max_speed_mps")
                ),
                landing_altitude_tolerance_m=float(
                    trajectory_param("landing_altitude_tolerance_m")
                ),
                landing_vertical_velocity_tolerance_mps=float(
                    trajectory_param("landing_vertical_velocity_tolerance_mps")
                ),
                landing_settle_s=float(trajectory_param("landing_settle_s")),
                landing_condition_grace_s=float(
                    trajectory_param("landing_condition_grace_s")
                ),
                takeoff_max_tilt_rad=math.radians(float(
                    trajectory_param("takeoff_max_tilt_deg")
                )),
                circle_max_tilt_rad=math.radians(float(
                    trajectory_param("circle_max_tilt_deg")
                )),
                landing_max_tilt_rad=math.radians(float(
                    trajectory_param("landing_max_tilt_deg")
                )),
                takeoff_min_collective_thrust=float(
                    vehicle_param("takeoff_min_thrust_newton")
                ),
                airborne_min_collective_thrust=float(
                    vehicle_param("airborne_min_thrust_newton")
                ),
                circle_center_xy=circle_center_xy,
                orbit_phase_rad=self.orbit_phase_rad,
                trajectory_mode=str(trajectory_param("trajectory_mode", "circle")),
                formation_side_length_m=float(
                    trajectory_param("formation_side_length_m", 0.5)
                ),
                figure_eight_radius_m=float(
                    trajectory_param("figure_eight_radius_m", 0.8)
                ),
                figure_eight_angular_speed_radps=float(
                    trajectory_param("figure_eight_angular_speed_radps", 0.55)
                ),
            )
            # 这里先做一次无起点构造，尽早报告参数错误；实际圆心/编队质心到第一帧后才解析。
            CircularFlightTrajectory(self.trajectory_config)
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException("任务轨迹参数无效：%s" % error)

        # 预起飞电压补偿不是飞行中的闭环：它只用零推力阶段采集到的一段电压中位数
        # 计算一次 raw PWM 缩放值。飞行中电压因负载和 ADC 噪声快速变化，逐包补偿会
        # 直接制造推力抖动，因此明确禁止那种做法。
        self.require_preflight_voltage = bool(
            self.params.global_param("require_preflight_voltage")
        )
        self.preflight_voltage_samples_required = int(
            self.params.global_param("preflight_voltage_samples")
        )
        self.preflight_voltage_timeout_s = float(
            self.params.global_param("preflight_voltage_timeout_s")
        )
        self.preflight_battery_max_age_s = float(
            self.params.global_param("preflight_battery_max_age_s")
        )
        self.preflight_min_voltage_v = float(
            self.params.global_param("preflight_min_voltage_v")
        )
        # 这个参考电压必须与 max_total_thrust_newton 的静态标定电压一致。4.20 V
        # 目前只是单节满电的暂定值，后续 F(raw, V) 标定可直接替换它。
        self.preflight_voltage_reference_v = float(
            self.params.vehicle_param("preflight_voltage_reference_v")
        )
        # 对理想关系 F ∝ (raw * V)^2，raw 缩放为 V_ref / V_preflight，即指数为 1。
        # 保留独立参数是为了后续实验标定电压-推力幂次，而不修改接口。
        self.preflight_raw_scale_exponent = float(
            self.params.vehicle_param("preflight_raw_scale_exponent")
        )
        self.preflight_raw_scale_min = float(
            self.params.vehicle_param("preflight_raw_scale_min")
        )
        self.preflight_raw_scale_max = float(
            self.params.vehicle_param("preflight_raw_scale_max")
        )
        if (self.preflight_voltage_samples_required < 1 or
                self.preflight_voltage_timeout_s <= 0.0 or
                self.preflight_battery_max_age_s <= 0.0 or
                self.preflight_min_voltage_v <= 0.0 or
                self.preflight_voltage_reference_v <= 0.0 or
                self.preflight_raw_scale_exponent <= 0.0 or
                self.preflight_raw_scale_min <= 0.0 or
                self.preflight_raw_scale_max < self.preflight_raw_scale_min):
            raise rospy.ROSInitException(
                "预起飞电压采样、阈值和 raw 推力缩放参数必须有效"
            )

        # max_total_thrust 是物理标定上限（120 gf）；max_command_thrust 是本次控制器
        # 使用的更保守上限。两者均是四个电机的总推力。
        max_total_thrust = float(self.params.vehicle_param("max_total_thrust_newton"))
        max_command_thrust = float(
            self.params.vehicle_param("max_command_thrust_newton")
        )
        if max_total_thrust <= 0.0 or max_command_thrust <= 0.0:
            raise rospy.ROSInitException("推力上限必须为正数")
        max_command_thrust = min(max_command_thrust, max_total_thrust)
        omega_c_method = str(
            self.params.vehicle_param("omega_c_method", "analytic")
        )
        omega_c_force_norm_epsilon = float(
            self.params.vehicle_param("omega_c_force_norm_epsilon", 1e-8)
        )
        omega_c_heading_projection_epsilon = float(
            self.params.vehicle_param("omega_c_heading_projection_epsilon", 1e-6)
        )
        if (omega_c_method.strip().lower() not in
                ("analytic", "derivative", "log_difference", "log", "so3_log")):
            raise rospy.ROSInitException(
                "omega_c_method 必须为 analytic 或 log_difference"
            )
        if omega_c_method.strip().lower() in ("log_difference", "log", "so3_log"):
            rospy.logwarn(
                "omega_c_method=%s：滤波加速度不会进入 Omega_c；"
                "设为 analytic 才会启用 V2 解析角速度前馈",
                omega_c_method,
            )
        if (not math.isfinite(omega_c_force_norm_epsilon) or
                omega_c_force_norm_epsilon <= 0.0 or
                not math.isfinite(omega_c_heading_projection_epsilon) or
                omega_c_heading_projection_epsilon <= 0.0):
            raise rospy.ROSInitException("Omega_c 数值保护参数必须为正数")
        self.controller = GeometricCtbrController(ControllerConfig(
            mass=mass_kg,
            gravity=float(self.params.global_param("gravity_mps2")),
            max_total_thrust=max_total_thrust,
            max_command_thrust=max_command_thrust,
            max_tilt_rad=math.radians(float(
                self.params.vehicle_param("max_tilt_deg")
            )),
            max_body_rate=self._vector_param(self.params.vehicle_param, "max_body_rate_radps"),
            position_gain=self._vector_param(self.params.vehicle_param, "position_gain"),
            velocity_gain=self._vector_param(self.params.vehicle_param, "velocity_gain"),
            integral_gain=self._vector_param(self.params.vehicle_param, "integral_gain"),
            integral_limit=self._vector_param(self.params.vehicle_param, "integral_limit"),
            independent_position_gain=self._vector_param(
                self.params.vehicle_param, "independent_position_gain"
            ),
            independent_velocity_gain=self._vector_param(
                self.params.vehicle_param, "independent_velocity_gain"
            ),
            independent_integral_gain=self._vector_param(
                self.params.vehicle_param, "independent_integral_gain"
            ),
            independent_integral_limit=self._vector_param(
                self.params.vehicle_param, "independent_integral_limit"
            ),
            independent_integral_gate=float(
                self.params.vehicle_param("independent_integral_gate")
            ),
            independent_max_feedback_acceleration=self._vector_param(
                self.params.vehicle_param, "independent_max_feedback_acceleration"
            ),
            independent_max_body_rate=self._vector_param(
                self.params.vehicle_param, "independent_max_body_rate_radps"
            ),
            independent_attitude_gain=self._vector_param(
                self.params.vehicle_param, "independent_attitude_gain"
            ),
            independent_heading=self._vector_param(
                self.params.vehicle_param, "independent_heading"
            ),
            attitude_gain=self._vector_param(self.params.vehicle_param, "attitude_gain"),
            attitude_integral_gain=self._vector_param(
                self.params.vehicle_param, "attitude_integral_gain"
            ),
            attitude_integral_limit=self._vector_param(
                self.params.vehicle_param, "attitude_integral_limit"
            ),
            position_integral_c1=float(
                self.params.vehicle_param("position_integral_c1")
            ),
            use_body_rate_feedforward=bool(
                self.params.vehicle_param("use_body_rate_feedforward")
            ),
            omega_c_method=omega_c_method,
            omega_c_force_norm_epsilon=omega_c_force_norm_epsilon,
            omega_c_heading_projection_epsilon=omega_c_heading_projection_epsilon,
            transport_body_rate_filter_tau_s=float(self.params.vehicle_param(
                "transport_body_rate_filter_tau_s", 0.0
            )),
            transport_force_filter_tau_s=float(self.params.vehicle_param(
                "transport_force_filter_tau_s", 0.0
            )),
        ))

        # These static gains must not query the ROS master in the 70 Hz loop.
        for name, default in (("transport_attitude_gain", [240., 240., 120.]),
                              ("transport_rate_gain", [4., 4., 4.])):
            try:
                gain = as_vector(self.params.vehicle_param(name, default), name).copy()
            except ValueError as error:
                raise rospy.ROSInitException(str(error))
            if np.any(gain < 0):
                raise rospy.ROSInitException(name + " 必须为非负有限三维向量")
            gain.setflags(write=False)
            setattr(self, name, gain)

        # 每次启动创建一份独立 CSV，避免覆盖上一次实飞；状态和电池回调共享同一把锁。
        log_directory = self.params.global_param("log_directory")
        log_prefix = rospy.get_param(
            "~log_prefix", "cf%d_ctbr" % self.vehicle_id
        )
        if self.logger is None:
            self.logger = FlightCsvLogger(log_directory, log_prefix)
        self.path_frame_id = str(self.params.global_param("path_frame_id", "world"))
        self.path_publish_interval_s = float(
            self.params.global_param("path_publish_interval_s", 0.1)
        )
        self.path_max_poses = int(self.params.global_param("path_max_poses", 1000))
        if not self.path_frame_id:
            raise rospy.ROSInitException("path_frame_id 不能为空")
        if self.path_publish_interval_s <= 0.0:
            raise rospy.ROSInitException("path_publish_interval_s 必须为正数")
        if self.path_max_poses <= 0:
            raise rospy.ROSInitException("path_max_poses 必须为正整数")
        self.path_publisher = rospy.Publisher(
            rospy.get_param("~path_topic", self.prefix + "/path"),
            Path,
            queue_size=1,
            latch=True,
        )
        self.path_message = Path()
        self.path_message.header.frame_id = self.path_frame_id
        self.last_path_publish_time = 0.0
        self.lock = threading.Lock()
        self.latest_state = None
        self.latest_ekf_state = None
        self.latest_battery = None
        self.payload_diagnostics = None
        self.latest_raw_thrust = None
        self.latest_raw_thrust_received_time = None
        self.latest_bridge_timing = None
        self.last_published_command_stamp_ns = 0
        self.controller_compute_wall_s = math.nan
        self.previous_vehicle_cycle_wall_s = math.nan
        self.previous_vehicle_cycle_control_time_s = math.nan
        self.previous_fleet_cycle_wall_s = math.nan
        self.previous_fleet_cycle_control_time_s = math.nan
        # 最后一帧经过位置一致性检查的 EKF 运动学。飞行中只允许在很短的时间窗口
        # 内保持它，绝不改用 NOKOV 的差分速度/加速度。
        self.last_valid_ekf_kinematics = None
        self.ekf_alignment_started_time = None
        self.ekf_alignment_since = None
        self.ekf_alignment_failed = False
        self.ekf_alignment_failure_reason = ""
        self.ekf_fault_since = None
        # 回调只在收到一条新的 /battery 消息时追加样本，避免控制定时器把同一条
        # 10 Hz 电压消息误计为 100 个独立样本。
        self.preflight_voltage_samples = deque(
            maxlen=max(2 * self.preflight_voltage_samples_required, 50)
        )
        self.last_tick = time.monotonic()
        self.start_time = self.last_tick if self.shared_start_time is None else self.shared_start_time
        # 单机直接运行时也记录从控制器创建开始的任务时间；多机模式则沿用
        # 管理器传入的共享时钟，保证所有飞机的 CSV 时间轴一致。
        if self.mission_start_time is None:
            self.mission_start_time = self.start_time
        # 轨迹模块在第一帧有效状态后创建；latest_* 由 ROS 回调线程写入，必须受锁保护。
        self.trajectory = None
        self.last_trajectory_phase = None
        self.flight_phase = "waiting_for_state"
        self.preflight_voltage_started_time = None
        self.preflight_voltage_v = math.nan
        self.preflight_voltage_sample_count = 0
        self.thrust_raw_scale = 1.0
        self.preflight_voltage_ready = not self.require_preflight_voltage
        self.preflight_voltage_failed = False
        self.preflight_voltage_failure_reason = ""

        self.command_publisher = rospy.Publisher(
            self.prefix + "/cmd_ctbr", CTBR, queue_size=1
        )
        # C++ 桥接层发布实际送入 sendSetpoint() 的 raw PWM，供 CSV 对齐诊断。
        self.raw_thrust_subscriber = rospy.Subscriber(
            self.prefix + "/ctbr_raw_thrust", UInt16,
            self._raw_thrust_callback, queue_size=1,
        )
        self.timing_subscriber = rospy.Subscriber(
            self.prefix + "/ctbr_timing", CTBRTiming,
            self._ctbr_timing_callback, queue_size=1,
        )
        self.state_subscriber = rospy.Subscriber(
            self.prefix + "/mocap_state", MocapState, self._state_callback, queue_size=1
        )
        self.ekf_subscriber = rospy.Subscriber(
            self.prefix + "/ekf_kinematics", GenericLogData,
            self._ekf_state_callback, queue_size=1,
        )
        # hover_swarm.launch 配置 GenericLogData 的 values 顺序为
        # [pm.vbat(V), pm.vbatMV(mV), pm.state, pm.batteryLevel]；电池日志只在
        # 低频重复提供样本，控制器完成预检后不再依赖后续电池消息。
        # 未启用固件日志时该订阅者保持空闲，控制器仍可运行，只会在 CSV 中写 NaN。
        self.battery_subscriber = rospy.Subscriber(
            self.prefix + "/battery", GenericLogData, self._battery_callback, queue_size=1
        )
        self.timer = None
        if self.auto_timer:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)
        if self.register_shutdown:
            rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "CTBR 控制器已启动：cf%d，话题前缀 %s，日志：%s",
            self.vehicle_id, self.prefix, self.logger.path,
        )

    def _vector_param(self, reader, name):
        """读取必需的三维参数，并统一转换错误为 ROS 启动异常。"""
        try:
            return as_vector(reader(name), name)
        except ValueError as error:
            raise rospy.ROSInitException(str(error))

    def _state_callback(self, message):
        """验证 Nokov 状态并缓存最新一帧，不在回调内执行控制计算。

        ``MocapState.valid`` 表示刚体被看到；控制器使用收到的速度经过因果二阶低通
        后的结果，并由该结果求加速度。9999.999 m 等 Nokov 丢失刚体哨兵值会先被工作
        空间检查拒绝。
        """
        try:
            state = {
                "position": np.array([
                    message.pose.position.x, message.pose.position.y, message.pose.position.z
                ], dtype=float),
                "velocity": np.array([
                    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z
                ], dtype=float),
                "acceleration": np.array([
                    message.acceleration.x,
                    message.acceleration.y,
                    message.acceleration.z,
                ], dtype=float),
                "body_rate": np.array([
                    message.twist.angular.x, message.twist.angular.y, message.twist.angular.z
                ], dtype=float),
                "rotation": quaternion_to_rotation(
                    message.pose.orientation.x, message.pose.orientation.y,
                    message.pose.orientation.z, message.pose.orientation.w,
                ),
                "valid": bool(message.valid),
                # 保留原字段的语义：这是服务器端 Nokov 微分器的有效标志。
                "derivatives_valid": bool(message.derivatives_valid),
            }
            if not all(
                    np.all(np.isfinite(state[key]))
                    for key in ("position", "velocity", "body_rate")):
                raise ValueError("状态中存在非有限数值")
            if np.max(np.abs(state["position"])) > self.max_abs_position_m:
                raise ValueError(
                    "位置超过工作空间边界 %.3f m" % self.max_abs_position_m
                )
            state["velocity"] = np.clip(
                state["velocity"], -self.max_mocap_velocity_mps,
                self.max_mocap_velocity_mps,
            )
            received_time = time.monotonic()
            try:
                sample_time = float(message.header.stamp.to_sec())
            except (AttributeError, TypeError, ValueError) as error:
                raise ValueError("mocap_state 缺少有效 header.stamp") from error
            if not math.isfinite(sample_time) or sample_time <= 0.0:
                raise ValueError("mocap_state header.stamp 必须为正的有限时间")
            if not state["valid"]:
                with self.velocity_filter_lock:
                    self.velocity_filter.reset()
                with self.lock:
                    self.latest_state = None
                return
            if not state["derivatives_valid"]:
                # Nokov 侧在丢帧或时间跳变后会重置微分历史。此时不能把新样本
                # 和本地旧滤波器状态相连，也不能把不可靠速度送入速度反馈。
                with self.velocity_filter_lock:
                    self.velocity_filter.reset()
                filtered_velocity = np.zeros(3)
                filtered_acceleration = np.zeros(3)
                control_acceleration = np.zeros(3)
                filter_derivatives_valid = False
            else:
                with self.velocity_filter_lock:
                    filtered_velocity, filtered_acceleration, filter_derivatives_valid = (
                        self.velocity_filter.update(state["velocity"], sample_time)
                    )
                if filtered_velocity is None:
                    raise ValueError("速度低通滤波器拒绝当前样本")
                control_acceleration = np.clip(
                    filtered_acceleration,
                    -self.max_mocap_acceleration_mps2,
                    self.max_mocap_acceleration_mps2,
                )
            state["filtered_velocity"] = filtered_velocity
            state["filtered_acceleration"] = filtered_acceleration
            # CSV 保留限幅前的滤波差分；控制器使用独立的限幅副本。
            state["control_acceleration"] = control_acceleration
            state["filter_derivatives_valid"] = filter_derivatives_valid
            state["mocap_sample_time_s"] = sample_time
            state["received_time"] = received_time
        except ValueError as error:
            rospy.logwarn_throttle(1.0, "忽略无效 mocap 状态：%s", error)
            with self.velocity_filter_lock:
                self.velocity_filter.reset()
            with self.lock:
                self.latest_state = None
            return
        with self.lock:
            self.latest_state = state

    def _ekf_state_callback(self, message):
        """缓存固件 EKF 的位置和速度，不让它覆盖 NOKOV 位姿。"""
        try:
            values = np.asarray(message.values, dtype=float).reshape(-1)
            if values.size != 6:
                raise ValueError("EKF 日志应包含 x/y/z 与 vx/vy/vz 共 6 个值")
            position = values[:3]
            velocity = values[3:]
            if (not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)) or
                    np.max(np.abs(position)) > self.max_abs_position_m):
                raise ValueError("EKF 状态包含无效或越界数值")
            sample_time = float(message.header.stamp.to_sec())
            if not math.isfinite(sample_time) or sample_time < 0.0:
                raise ValueError("EKF 固件日志时间戳无效")
            received_time = time.monotonic()
            with self.ekf_velocity_filter_lock:
                filtered_velocity, filtered_acceleration, derivatives_valid = (
                    self.ekf_velocity_filter.update(velocity, sample_time)
                )
            if filtered_velocity is None:
                raise ValueError("EKF 速度滤波器拒绝当前样本")
            state = {
                "position": position.copy(),
                "velocity": velocity.copy(),
                "filtered_velocity": filtered_velocity,
                "filtered_acceleration": filtered_acceleration,
                "filter_derivatives_valid": bool(derivatives_valid),
                "received_time": received_time,
                "firmware_sample_time_s": sample_time,
            }
        except (AttributeError, IndexError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(1.0, "忽略无效 EKF 状态：%s", error)
            with self.ekf_velocity_filter_lock:
                self.ekf_velocity_filter.reset()
            with self.lock:
                self.latest_ekf_state = None
            return
        with self.lock:
            self.latest_ekf_state = state

    def _raw_thrust_callback(self, message):
        """缓存 C++ 桥接层最终送入 sendSetpoint() 的 raw PWM。"""
        try:
            raw_thrust = int(message.data)
        except (AttributeError, TypeError, ValueError):
            return
        if raw_thrust < 0 or raw_thrust > 60000:
            return
        with self.lock:
            self.latest_raw_thrust = raw_thrust
            self.latest_raw_thrust_received_time = time.monotonic()

    @staticmethod
    def _stamp_nanoseconds(stamp):
        if hasattr(stamp, "to_nsec"):
            return int(stamp.to_nsec())
        # SimpleNamespace clocks used by the offline replay have only to_sec.
        return int(round(float(stamp.to_sec()) * 1000000000))

    def _ctbr_timing_callback(self, message):
        """Cache one complete bridge report; its header belongs to the source command."""
        try:
            stamp_ns = self._stamp_nanoseconds(message.header.stamp)
            command_seq = int(message.command_seq)
            callback_wall = float(message.callback_duration_s)
            send_wall = float(message.send_duration_s)
            command_age = float(message.command_age_s)
            raw = int(message.raw_thrust)
            if (stamp_ns < 0 or command_seq < 0 or not 0 <= raw <= 60000
                    or not math.isfinite(callback_wall) or callback_wall < 0
                    or not math.isfinite(send_wall) or send_wall < 0
                    or (stamp_ns > 0 and not math.isfinite(command_age))):
                return
            timing = dict(
                command_stamp_ns=stamp_ns, command_seq=command_seq,
                command_age_s=command_age, callback_wall_s=callback_wall,
                send_call_wall_s=send_wall, raw_thrust=raw,
                accepted=int(bool(message.command_accepted)),
                unlock_packet=int(bool(message.unlock_packet)),
                received_time=time.monotonic(),
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            return
        with self.lock:
            self.latest_bridge_timing = timing

    def _timing_log_fields(self, now):
        fields = {name: float(getattr(self, name, math.nan)) for name in (
            "controller_tick_interval_s", "controller_compute_wall_s",
            "previous_vehicle_cycle_wall_s", "previous_vehicle_cycle_control_time_s",
            "fleet_tick_interval_s", "previous_fleet_cycle_wall_s",
            "previous_fleet_cycle_control_time_s",
        )}
        with self.lock:
            published = int(getattr(self, "last_published_command_stamp_ns", 0))
            cached = getattr(self, "latest_bridge_timing", None)
            timing = None if cached is None else dict(cached)
        fields["published_command_stamp_ns"] = published
        for name in ("command_stamp_ns", "command_seq", "command_age_s", "callback_wall_s",
                     "send_call_wall_s", "raw_thrust", "accepted", "unlock_packet"):
            fields["bridge_latest_" + name] = math.nan if timing is None else timing[name]
        fields["bridge_latest_receipt_age_s"] = (math.nan if timing is None else
            max(0.0, time.monotonic() - timing["received_time"]))
        # A stamp match is a correlation hint. ROS serialization owns header.seq,
        # and paused/coarse ROS clocks can give different commands the same stamp.
        fields["bridge_latest_matches_published_stamp"] = int(
            timing is not None and published > 0 and published == timing["command_stamp_ns"])
        return fields

    def _battery_callback(self, message):
        """缓存固件主电池遥测，时间使用主机接收时刻以便与控制日志对齐。"""
        try:
            # GenericLogData 没有变量名；这里的下标必须与 hover_swarm.launch 中的
            # genericLogTopic_battery_Variables 顺序一致。
            if len(message.values) < 4:
                raise ValueError("battery 日志字段数量不足")
            voltage_v, voltage_mv, battery_state, battery_level = [
                float(value) for value in message.values[:4]
            ]
            if not all(math.isfinite(value) for value in (
                    voltage_v, voltage_mv, battery_state, battery_level)):
                raise ValueError("battery 日志包含非有限数值")
            # 本项目的 CF21BL 使用单节主电池。此检查只拒绝明显错误的数据，不承担
            # 低电保护功能；低电是否继续飞行由后续标定后的安全策略决定。
            if not 0.0 < voltage_v < 5.5:
                raise ValueError("主电池电压 %.3f V 超出单节电池范围" % voltage_v)
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(1.0, "忽略无效电池遥测：%s", error)
            return

        received_time = time.monotonic()
        with self.lock:
            self.latest_battery = {
                "voltage_v": voltage_v,
                "voltage_mv": voltage_mv,
                "state": int(round(battery_state)),
                "level_percent": battery_level,
                "received_time": received_time,
            }
            self.preflight_voltage_samples.append((received_time, voltage_v))

    @staticmethod
    def _preflight_hold_target(state, phase):
        """构造预检阶段的零输出参考，仅用于日志和状态机可视化。"""
        return {
            "position": state["position"].copy(),
            "velocity": np.zeros(3),
            "acceleration": np.zeros(3),
            "jerk": np.zeros(3),
            "yaw": rotation_to_rpy(state["rotation"])[2],
            "yaw_rate": 0.0,
            "flight_phase": phase,
            "zero_output": True,
        }

    def _fail_preflight_voltage(self, reason):
        """使预检失败保持粘滞：只有人工检查后重启节点才能再次尝试起飞。"""
        if not self.preflight_voltage_failed:
            self.preflight_voltage_failed = True
            self.preflight_voltage_failure_reason = reason
            rospy.logerr("起飞前电压预检失败：%s；保持零 CTBR 推力", reason)

    def _preflight_voltage_target(self, state, now):
        """在实际控制前冻结电压补偿系数，失败时不允许进入起飞状态机。

        电池日志低频重复发送；控制器在预检阶段使用新鲜样本。成功后
        ``thrust_raw_scale`` 在本次节点生命周期内保持不变，不再依赖飞行中电池日志。
        """
        if not self.require_preflight_voltage:
            return None
        if self.preflight_voltage_ready:
            return None
        if self.preflight_voltage_failed:
            return self._preflight_hold_target(state, "preflight_voltage_failed")

        if self.preflight_voltage_started_time is None:
            self.preflight_voltage_started_time = now
            rospy.loginfo(
                "开始起飞前电压预检：等待 %d 个 pm.vbat 样本，期间保持零推力",
                self.preflight_voltage_samples_required,
            )

        with self.lock:
            samples = [
                (received_time, voltage_v)
                for received_time, voltage_v in self.preflight_voltage_samples
            ]
        self.preflight_voltage_sample_count = len(samples)
        latest_sample_age = (
            math.inf if not samples else now - samples[-1][0]
        )
        if (len(samples) >= self.preflight_voltage_samples_required and
                latest_sample_age <= self.preflight_battery_max_age_s):
            values = np.asarray(
                [sample[1] for sample in samples[-self.preflight_voltage_samples_required:]],
                dtype=float,
            )
            preflight_voltage_v = float(np.median(values))
            if preflight_voltage_v < self.preflight_min_voltage_v:
                self._fail_preflight_voltage(
                    "电压中位数 %.3f V 低于 %.3f V" % (
                        preflight_voltage_v, self.preflight_min_voltage_v
                    )
                )
                return self._preflight_hold_target(state, "preflight_voltage_failed")

            raw_scale = (self.preflight_voltage_reference_v / preflight_voltage_v) ** (
                self.preflight_raw_scale_exponent
            )
            if (raw_scale < self.preflight_raw_scale_min or
                    raw_scale > self.preflight_raw_scale_max):
                self._fail_preflight_voltage(
                    "raw 缩放 %.3f 超出 [%.3f, %.3f]" % (
                        raw_scale, self.preflight_raw_scale_min,
                        self.preflight_raw_scale_max,
                    )
                )
                return self._preflight_hold_target(state, "preflight_voltage_failed")

            self.preflight_voltage_v = preflight_voltage_v
            self.thrust_raw_scale = raw_scale
            self.preflight_voltage_ready = True
            self.controller.reset()
            rospy.loginfo(
                "电压预检完成：中位数 %.3f V，冻结 raw 推力缩放 %.3f（参考 %.3f V）",
                self.preflight_voltage_v, self.thrust_raw_scale,
                self.preflight_voltage_reference_v,
            )
            return None

        if now - self.preflight_voltage_started_time >= self.preflight_voltage_timeout_s:
            self._fail_preflight_voltage(
                "%.1f s 内未获得 %d 个新鲜 pm.vbat 样本（当前 %d 个，最后样本年龄 %.2f s）" % (
                    self.preflight_voltage_timeout_s,
                    self.preflight_voltage_samples_required,
                    len(samples), latest_sample_age,
                )
            )
            return self._preflight_hold_target(state, "preflight_voltage_failed")
        return self._preflight_hold_target(state, "preflight_voltage")

    @property
    def _ekf_kinematics_enabled(self):
        """Whether this run requires EKF velocity/acceleration for CTBR."""
        return self.ekf_kinematics_weight > 0.0

    def _build_ekf_control_state(self, state, now):
        """Build the control state without ever substituting NOKOV derivatives.

        A fresh, position-consistent EKF sample replaces the retained sample.
        During a short radio/log gap the last sample is held; after that the
        returned velocity/acceleration are zero and the caller starts the
        controlled emergency landing path instead of injecting mocap
        differentiation noise into the feedback loop.
        """
        with self.lock:
            ekf_state = self.latest_ekf_state
        control_state = blend_kinematic_feedback(
            state,
            ekf_state,
            now=now,
            ekf_weight=self.ekf_kinematics_weight,
            ekf_state_timeout=self.ekf_state_timeout,
            max_acceleration=self.max_mocap_acceleration_mps2,
            max_position_delta=self.ekf_max_position_delta_m,
            last_valid_ekf=self.last_valid_ekf_kinematics,
            hold_timeout_s=self.ekf_kinematics_hold_s,
            timestamp_future_tolerance_s=self.ekf_timestamp_future_tolerance_s,
        )
        if control_state["ekf_state_valid"]:
            self.last_valid_ekf_kinematics = {
                "velocity": control_state["control_velocity"].copy(),
                "acceleration": control_state["mixed_acceleration"].copy(),
                "valid_time": float(now),
            }
            self.ekf_fault_since = None
        elif self._ekf_kinematics_enabled:
            if self.ekf_fault_since is None:
                self.ekf_fault_since = float(now)
        fault_age = (
            0.0 if self.ekf_fault_since is None
            else max(0.0, float(now) - self.ekf_fault_since)
        )
        control_state["ekf_fault_age_s"] = fault_age
        return control_state

    def _fail_ekf_alignment(self, reason):
        """Latch an on-ground EKF preflight failure; it must not release CTBR."""
        if not self.ekf_alignment_failed:
            self.ekf_alignment_failed = True
            self.ekf_alignment_failure_reason = str(reason)
            rospy.logerr("EKF 起飞前对齐失败：%s；保持零 CTBR 推力", reason)

    def _ekf_preflight_ready(self, state, now):
        """Require continuous position alignment and ready kinematics at release.

        Velocity-filter warmup does not invalidate a finite, fresh position.
        Takeoff still requires fresh, valid filtered EKF derivatives.
        """
        if not self._ekf_kinematics_enabled:
            return True
        if self.ekf_alignment_failed:
            return False
        if self.ekf_alignment_started_time is None:
            self.ekf_alignment_started_time = float(now)

        control_state = self._build_ekf_control_state(state, now)
        position_age = float(control_state["ekf_state_age_s"])
        aligned = (
            math.isfinite(position_age) and
            -self.ekf_timestamp_future_tolerance_s <= position_age <= self.ekf_state_timeout and
            control_state["ekf_position_error_m"] <=
            self.ekf_alignment_position_delta_m
        )
        if aligned:
            if self.ekf_alignment_since is None:
                self.ekf_alignment_since = float(now)
            if (now - self.ekf_alignment_since >= self.ekf_alignment_hold_s
                    and control_state["ekf_state_valid"]):
                return True
        else:
            self.ekf_alignment_since = None

        if now - self.ekf_alignment_started_time >= self.ekf_alignment_timeout_s:
            self._fail_ekf_alignment(
                "CF%d: %.1f s 内未完成位置连续 %.1f s 对齐到 %.3f m 和 EKF 导数就绪"
                "（状态=%s，当前差=%.3f m，位置连续=%.3f s，导数就绪=%d）" % (
                    self.vehicle_id,
                    self.ekf_alignment_timeout_s,
                    self.ekf_alignment_hold_s,
                    self.ekf_alignment_position_delta_m,
                    control_state.get("ekf_status", "unknown"),
                    control_state.get("ekf_position_error_m", math.inf),
                    (0.0 if self.ekf_alignment_since is None else
                     max(0.0, now - self.ekf_alignment_since)),
                    int(control_state["ekf_state_valid"]),
                )
            )
        return False

    def _target_for(self, state, now):
        """取得轨迹模块的当前参考目标。"""
        if self.trajectory is None:
            self.trajectory = CircularFlightTrajectory(self.trajectory_config)
            self.trajectory.reset(
                state["position"], rotation_to_rpy(state["rotation"])[2], now
            )

        target = self.trajectory.evaluate(state, now)
        for event in self.trajectory.pop_events():
            rospy.loginfo("%s", event)
        return target

    def _resume_or_abort_trajectory(self, state, now):
        """恢复短暂暂停的轨迹，或在长时间动捕失效后锁定零输出中止状态。"""
        if self.trajectory is None or not self.trajectory.is_paused:
            return
        pause_duration = self.trajectory.resume(now)
        # 状态失效路径已经 reset 过积分器；这里再次 reset 使恢复后的第一帧不会带入
        # 丢帧前的姿态差分历史。
        self.controller.reset()
        if pause_duration > self.trajectory_pause_abort_s:
            self.trajectory.abort(
                now,
                "Nokov 状态连续失效 %.3f s，超过 %.3f s" % (
                    pause_duration, self.trajectory_pause_abort_s
                ),
                position=state["position"],
            )

    def _mocap_state_ages(self, state, now):
        """Return callback and NOKOV-header ages for one immutable state snapshot.

        ``received_time`` is the primary freshness signal: it measures when this
        process actually received the valid mocap state.  The header timestamp is
        retained to reject queued old frames and a genuinely faulty future clock.
        ROS callbacks may publish a new frame between two control operations, so
        take ``ros_now`` only *after* the state snapshot is selected.
        """
        state_age = math.inf if state is None else float(now) - float(
            state.get("received_time", math.inf)
        )
        try:
            ros_now = rospy.Time.now().to_sec()
            mocap_sample_age = float(ros_now) - float(state["mocap_sample_time_s"])
        except (AttributeError, KeyError, TypeError, ValueError):
            mocap_sample_age = math.nan
        return state_age, mocap_sample_age

    def _state_ready(self, now):
        """Return whether this vehicle has a fresh, valid NOKOV state."""
        with self.lock:
            state = self.latest_state
        if state is None or not state.get("valid", False):
            return False
        state_age, mocap_sample_age = CtbrControllerNode._mocap_state_ages(
            self, state, now
        )
        future_tolerance = float(getattr(
            self, "mocap_timestamp_future_tolerance_s", 0.0
        ))
        return (
            math.isfinite(state_age)
            and state_age <= self.state_timeout
            and math.isfinite(mocap_sample_age)
            and -future_tolerance <= mocap_sample_age <= self.state_timeout
        )

    def set_fleet_gate(self, open_gate):
        """Allow the multi-vehicle manager to release this vehicle together."""
        self.fleet_gate_open = bool(open_gate)

    def set_global_abort(self, reason):
        """Latch a fleet-wide abort reason; subsequent cycles only send zero CTBR."""
        self.global_abort_reason = str(reason or "")

    def set_payload_diagnostics(self, state):
        """Attach the latest load sample to this vehicle's next CSV row."""
        if state is None:
            self.payload_diagnostics = None
            return
        self.payload_diagnostics = dict(state)

    def _timer_callback(self, _event, now=None, transport_override=None):
        control_now = time.monotonic() if now is None else float(now)
        wall_start = time.perf_counter()
        self.controller_tick_interval_s = control_now - self.last_tick
        self.controller_compute_wall_s = math.nan
        try:
            return CtbrControllerNode._run_control_tick(
                self, _event, now=control_now, transport_override=transport_override)
        finally:
            self.previous_vehicle_cycle_wall_s = time.perf_counter() - wall_start
            self.previous_vehicle_cycle_control_time_s = control_now - getattr(self, "start_time", control_now)

    def _run_control_tick(self, _event, now=None, transport_override=None):
        """控制周期入口：先执行状态失效保护，再计算、发布并记录同一份命令。"""
        now = time.monotonic() if now is None else float(now)
        dt = now - self.last_tick
        self.last_tick = now
        with self.lock:
            state = self.latest_state

        if getattr(self, "global_abort_reason", ""):
            self.flight_phase = "aborted"
            self._publish_zero()
            if state is None:
                self._write_invalid_log(now, state, "global_abort_without_state")
            else:
                target = self._preflight_hold_target(state, "aborted")
                command = self._zero_command(state, target)
                self._write_log(
                    now, state, target, command,
                    invalid_reason="global_abort:" + self.global_abort_reason,
                )
            return

        state_age, mocap_sample_age = CtbrControllerNode._mocap_state_ages(
            self, state, now
        )
        future_tolerance = float(getattr(
            self, "mocap_timestamp_future_tolerance_s", 0.0
        ))
        state_ready = (
            state is not None
            and state["valid"]
            and math.isfinite(state_age)
            and state_age <= self.state_timeout
            and math.isfinite(mocap_sample_age)
            and -future_tolerance <= mocap_sample_age <= self.state_timeout
        )
        if not state_ready:
            # 不使用旧 mocap 继续飞行。服务器端还有 0.1 s CTBR watchdog；这里主动
            # 发送零包可比等待 watchdog 更快地切断推力。
            reasons = []
            if state is None:
                reasons.append("尚未收到动捕状态")
            else:
                if not state["valid"]:
                    reasons.append("mocap_state.valid=false")
                if state_age > self.state_timeout:
                    reasons.append("状态超时")
            if not math.isfinite(mocap_sample_age):
                reasons.append("mocap 时间戳无效或回退")
            elif mocap_sample_age < -future_tolerance:
                reasons.append(
                    "mocap 时间戳超前 %.3f s（容差 %.3f s）" % (
                        -mocap_sample_age,
                        future_tolerance,
                    )
                )
            elif mocap_sample_age > self.state_timeout:
                reasons.append("mocap 样本超时")
            rospy.logwarn_throttle(
                1.0,
                "CTBR 安全保护：%s；发送零推力 "
                "(phase=%s, state_age=%.3f s, timeout=%.3f s)",
                "、".join(reasons) if reasons else "状态检查失败",
                self.flight_phase,
                state_age,
                self.state_timeout,
            )
            self.controller.reset()
            with self.velocity_filter_lock:
                self.velocity_filter.reset()
            if self.trajectory is not None:
                self.trajectory.pause(now)
            self._publish_zero()
            self._write_invalid_log(
                now, state, ";".join(reasons) or "state_check_failed"
            )
            return

        # 路径发布与控制阶段无关：只要收到有效动捕，就把实际位置提供给 RViz。
        self._publish_path(state)
        self._resume_or_abort_trajectory(state, now)
        preflight_target = self._preflight_voltage_target(state, now)
        if preflight_target is not None:
            # 预检阶段绝不调用几何控制器：即使刚体处于地面，重力补偿也会形成非零
            # 悬停推力。持续零包同时满足 legacy commander 的首包解锁要求。
            self.controller.reset()
            command = self._zero_command(state, preflight_target)
            self._publish_zero()
            self._write_log(now, state, preflight_target, command)
            return

        if self.trajectory is None and not self._ekf_preflight_ready(state, now):
            # The EKF may be reset/reinitialized only while all motors remain
            # at zero.  Do not enter the trajectory merely because NOKOV is
            # fresh: this controller deliberately requires EKF v/a feedback.
            target = self._preflight_hold_target(state, "waiting_for_ekf")
            self.controller.reset()
            command = self._zero_command(state, target)
            self._publish_zero()
            self._write_log(
                now, state, target, command,
                control_state=self._build_ekf_control_state(state, now),
            )
            return

        if not getattr(self, "fleet_gate_open", True):
            # In multi-vehicle mode, an individually ready vehicle must remain
            # on zero output until every selected vehicle passes preflight and
            # has a fresh NOKOV state.
            target = self._preflight_hold_target(state, "waiting_for_fleet")
            self.controller.reset()
            command = self._zero_command(state, target)
            self._publish_zero()
            self._write_log(now, state, target, command)
            return

        target = self._target_for(state, now)
        if transport_override is not None:
            target = dict(target)
            target.update(transport_override)
        elif target.get("flight_phase") in ("takeoff", "height_correction"):
            target = dict(target)
            target["position_gain_scale"] = self.takeoff_position_gain_scale
            target["velocity_gain_scale"] = self.takeoff_velocity_gain_scale
        control_state = self._build_ekf_control_state(state, now)
        if (self._ekf_kinematics_enabled and
                not control_state["ekf_state_valid"] and
                not control_state["ekf_kinematics_held"] and
                control_state["ekf_fault_age_s"] >=
                self.ekf_fault_emergency_land_s and
                target["flight_phase"] not in (
                    "emergency_landing", "landing", "landing_settle", "landed"
                )):
            # Never reset a flying EKF or fall back to NOKOV derivatives.  A
            # prolonged failure latches a slow position-reference landing;
            # control_state contains zero v/a after the short hold window.
            self.controller.reset()
            self.trajectory.begin_emergency_landing(
                now,
                state["position"],
                rotation_to_rpy(state["rotation"])[2],
                "EKF 运动学持续失效：" + control_state.get("ekf_status", "unknown"),
            )
            target = self._target_for(state, now)
        # 普通阶段的参考位置、速度和加速度连续衔接。保留位置积分可维持每架飞机的
        # 稳态推力补偿，避免入圆或降落开始时因积分清零而产生突跳。只有安全/终止
        # 阶段才执行完整 reset。
        if (target["flight_phase"] != self.last_trajectory_phase and
                phase_requires_full_controller_reset(target["flight_phase"])):
            self.controller.reset()
        self.last_trajectory_phase = target["flight_phase"]
        if target["flight_phase"] == "landed" or target.get("zero_output", False):
            # landed、起飞前保持和安全中止必须绕开正常 compute()，否则目标地面高度的
            # 重力补偿会再次产生悬停推力，使飞机在地面附近反复弹起或提前离地。
            command = self._zero_command(control_state, target)
        else:
            compute_start = time.perf_counter()
            command = self.controller.compute(control_state, target, dt)
            self.controller_compute_wall_s = time.perf_counter() - compute_start
        if target["flight_phase"] == "landed" or target.get("zero_output", False):
            self._publish_zero()
        else:
            self._publish_command(command)
        self.last_control_command = command
        self._write_log(now, state, target, command, control_state=control_state)

    def _publish_path(self, state):
        """发布动捕实际轨迹，RViz 可直接订阅 nav_msgs/Path。"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.path_frame_id
        pose.pose.position.x = float(state["position"][0])
        pose.pose.position.y = float(state["position"][1])
        pose.pose.position.z = float(state["position"][2])
        pose.pose.orientation.w = 1.0
        self.path_message.header.stamp = pose.header.stamp
        self.path_message.poses.append(pose)
        if len(self.path_message.poses) > self.path_max_poses:
            del self.path_message.poses[:-self.path_max_poses]
        now = time.monotonic()
        if now - self.last_path_publish_time >= self.path_publish_interval_s:
            self.last_path_publish_time = now
            self.path_publisher.publish(self.path_message)

    def _publish_command(self, command):
        """把 SI 单位的控制输出封装为 CTBR 消息；raw PWM 转换在 C++ 桥接层完成。"""
        message = CTBR()
        message.header.stamp = rospy.Time.now()
        message.body_rates.x = float(command["body_rate_command"][0])
        message.body_rates.y = float(command["body_rate_command"][1])
        message.body_rates.z = float(command["body_rate_command"][2])
        message.collective_thrust = float(command["collective_thrust"])
        # 物理推力仍由 collective_thrust（N）表达；冻结电压只修正 C++ 中的 raw PWM。
        message.thrust_raw_scale = float(self.thrust_raw_scale)
        with self.lock:
            self.last_published_command_stamp_ns = self._stamp_nanoseconds(message.header.stamp)
        self.command_publisher.publish(message)

    def _publish_zero(self):
        """发送一个显式零 RPYT/推力包，用于状态失效、落地和节点退出。"""
        message = CTBR()
        message.header.stamp = rospy.Time.now()
        with self.lock:
            self.last_published_command_stamp_ns = self._stamp_nanoseconds(message.header.stamp)
        self.command_publisher.publish(message)

    @staticmethod
    def _zero_command(state, target):
        """记录落地后的真实误差，但不再请求任何推力或角速度。"""
        measured_velocity = state.get(
            "control_velocity",
            state.get("filtered_velocity", state["velocity"]),
        )
        return {
            "position_error": state["position"] - target["position"],
            "velocity_error": measured_velocity - target["velocity"],
            "desired_force": np.zeros(3),
            "computed_rotation": state["rotation"],
            "computed_body_rate": np.zeros(3),
            "omega_c_method": "zero",
            "attitude_error": np.zeros(3),
            "body_rate_command": np.zeros(3),
            "collective_thrust": 0.0,
        }

    @staticmethod
    def _with_xyz(row, prefix, vector):
        row[prefix + "_x"] = float(vector[0])
        row[prefix + "_y"] = float(vector[1])
        row[prefix + "_z"] = float(vector[2])

    def _base_log_row(self, now, state):
        state_age = math.inf if state is None else now - state["received_time"]
        ros_time_s = rospy.Time.now().to_sec()
        try:
            mocap_sample_age = (
                math.inf if state is None
                else ros_time_s - float(state["mocap_sample_time_s"])
            )
        except (KeyError, TypeError, ValueError):
            mocap_sample_age = math.nan
        row = {
            "control_time_s": now - self.start_time,
            "ros_time_s": ros_time_s,
            "mode": "control",
            "flight_phase": self.flight_phase,
            "state_valid": int(state is not None and state["valid"]),
            "derivatives_valid": int(state is not None and state["derivatives_valid"]),
            "filter_derivatives_valid": int(
                state is not None and state.get("filter_derivatives_valid", False)
            ),
            "state_age_s": state_age,
            "mocap_sample_age_s": mocap_sample_age,
            "mission_time_s": (
                math.nan if self.mission_start_time is None
                else now - self.mission_start_time
            ),
            "vehicle_id": self.vehicle_id,
            "radio_uri": self.radio_uri,
            "orbit_phase_rad": self.orbit_phase_rad,
            "formation_state": self.flight_phase,
            "global_abort_reason": self.global_abort_reason,
            "invalid_reason": "",
        }
        row.update(self._battery_log_fields(now))
        row.update(self._preflight_voltage_log_fields())
        row.update(self._timing_log_fields(now))
        payload = self.payload_diagnostics
        payload_age = (math.inf if payload is None
                       else now - float(payload.get("received_time", -math.inf)))
        row["payload_state_age_s"] = payload_age
        row["payload_state_valid"] = int(
            payload is not None and payload.get("valid", False)
            and math.isfinite(payload_age) and payload_age <= self.state_timeout
        )
        row["emergency_landing_reason"] = (
            getattr(self.trajectory, "emergency_landing_reason", "")
            if self.trajectory is not None else ""
        )
        if payload is not None:
            for name, key in (
                    ("payload_position", "position"),
                    ("payload_raw_velocity", "raw_velocity"),
                    ("payload_raw_acceleration", "raw_acceleration"),
                    ("payload_velocity", "velocity"),
                    ("payload_acceleration", "acceleration"),
                    ("payload_body_rate", "body_rate")):
                value = payload.get(key)
                if value is not None:
                    self._with_xyz(row, name, value)
            row["payload_filter_derivatives_valid"] = int(
                bool(payload.get("derivatives_valid", False))
            )
            if "rotation" in payload:
                rpy = rotation_to_rpy(payload["rotation"])
                for name, value in zip(("roll", "pitch", "yaw"), rpy):
                    row["payload_" + name + "_rad"] = float(value)
        return row

    def _preflight_voltage_log_fields(self):
        """返回冻结补偿的诊断字段，便于离线区分电压和控制器问题。"""
        return {
            "preflight_voltage_v": self.preflight_voltage_v,
            "preflight_voltage_sample_count": self.preflight_voltage_sample_count,
            "preflight_voltage_ready": int(self.preflight_voltage_ready),
            "preflight_voltage_failed": int(self.preflight_voltage_failed),
            "thrust_raw_scale": self.thrust_raw_scale,
        }

    def _battery_log_fields(self, now):
        """返回一组可直接写入 CSV 的电池字段，不把旧样本伪装成当前测量。"""
        with self.lock:
            battery = None if self.latest_battery is None else dict(self.latest_battery)
        if battery is None:
            return {
                "battery_voltage_v": math.nan,
                "battery_voltage_mv": math.nan,
                "battery_state": -1,
                "battery_level_percent": math.nan,
                "battery_age_s": math.inf,
            }
        return {
            "battery_voltage_v": battery["voltage_v"],
            "battery_voltage_mv": battery["voltage_mv"],
            "battery_state": battery["state"],
            "battery_level_percent": battery["level_percent"],
            "battery_age_s": now - battery["received_time"],
        }

    def _write_invalid_log(self, now, state, invalid_reason=""):
        """记录失效样本；保留可用原始状态，控制量字段留空以便离线识别。"""
        row = self._base_log_row(now, state)
        row["invalid_reason"] = str(invalid_reason or "state_check_failed")
        if state is not None:
            self._with_xyz(row, "position", state["position"])
            self._with_xyz(row, "velocity", state["velocity"])
            self._with_xyz(row, "acceleration", state["acceleration"])
            if "filtered_velocity" in state:
                self._with_xyz(row, "filtered_velocity", state["filtered_velocity"])
            if "filtered_acceleration" in state:
                self._with_xyz(
                    row, "filtered_acceleration", state["filtered_acceleration"]
                )
            self._with_xyz(row, "body_rate", state["body_rate"])
            rpy = rotation_to_rpy(state["rotation"])
            row.update(dict(zip(("roll_rad", "pitch_rad", "yaw_rad"), rpy)))
        self.logger.write(row)

    def _write_log(
            self, now, state, target, command, control_state=None,
            invalid_reason=""):
        """记录一次有效控制周期，所有误差均使用与实际发布相同的参考和命令。"""
        row = self._base_log_row(now, state)
        self.flight_phase = target["flight_phase"]
        row["flight_phase"] = self.flight_phase
        row["formation_state"] = self.flight_phase
        row["invalid_reason"] = str(invalid_reason or "")
        self._with_xyz(row, "position", state["position"])
        self._with_xyz(row, "velocity", state["velocity"])
        self._with_xyz(row, "acceleration", state["acceleration"])
        self._with_xyz(row, "filtered_velocity", state["filtered_velocity"])
        self._with_xyz(row, "filtered_acceleration", state["filtered_acceleration"])
        self._with_xyz(row, "body_rate", state["body_rate"])
        self._with_xyz(row, "target", target["position"])
        self._with_xyz(row, "target_jerk", target.get("jerk", np.zeros(3)))
        self._with_xyz(row, "target_velocity", target.get("velocity", np.zeros(3)))
        self._with_xyz(row, "target_acceleration", target.get("acceleration", np.zeros(3)))
        self._with_xyz(row, "position_error", command["position_error"])
        self._with_xyz(row, "velocity_error", command["velocity_error"])
        self._with_xyz(row, "desired_force", command["desired_force"])
        self._with_xyz(
            row, "desired_force_dot", command.get("desired_force_dot", np.zeros(3))
        )
        row["prelift_vertical_integral_active"] = int(bool(
            command.get("prelift_vertical_integral_active", False)
        ))
        row["prelift_vertical_integral_force_n"] = float(
            command.get("prelift_vertical_integral_force_n", 0.0)
        )
        row["landing_vertical_integral_active"] = int(bool(
            command.get("landing_vertical_integral_active", False)))
        row["landing_vertical_integral_force_n"] = float(
            command.get("landing_vertical_integral_force_n", 0.0))
        for field in ("transport_support_trim_n", "payload_minimum_vehicle_distance_m",
                      "payload_predicted_vehicle_distance_m", "transport_max_tilt_rad"):
            row[field] = float(target.get(field, math.nan))
        row["payload_safety_reason"] = target.get("payload_safety_reason", "")
        row["payload_landing_rope_preserving"] = int(bool(
            target.get("payload_landing_rope_preserving", False)
        ))
        row["payload_landing_rope_distance_m"] = float(
            target.get("payload_landing_rope_distance_m", math.nan)
        )
        row["flight_validation_enabled"] = int(bool(target.get("flight_validation_enabled", False)))
        row["flight_validation_elapsed_s"] = float(target.get("flight_validation_elapsed_s", math.nan))
        row["flight_validation_landing_started"] = int(bool(target.get("flight_validation_landing_started", False)))
        row["flight_validation_landing_stage"] = str(target.get("flight_validation_landing_stage", ""))
        for field in ("link_length_nominal_m", "link_length_effective_m", "link_length_measured_m",
                      "link_calibration_window_s", "link_calibration_spread_m"):
            row[field] = float(target.get(field, math.nan))
        for field in ("link_calibration_enabled", "link_calibration_applied"):
            row[field] = int(bool(target.get(field, False)))
        row["link_calibration_samples"] = int(target.get("link_calibration_samples", 0))
        for field in ("link_calibration_status", "link_calibration_reason"):
            row[field] = str(target.get(field, ""))
        for field in ("payload_ground_reference_z_m", "payload_landing_duration_s",
                      "transport_support_requested_trim_n", "transport_support_trim_limit_n",
                      "effective_transport_link_gain_scale"):
            row[field] = float(target.get(field, math.nan))
        for field in ("payload_landing_touchdown_confirmed", "payload_landing_unloading",
                      "transport_support_initialization_clipped", "payload_landing_planning_fallback"):
            row[field] = int(bool(target.get(field, False)))
        for field in ("payload_landing_planning_failure_count", "payload_landing_planning_retry_count"):
            row[field] = int(target.get(field, 0))
        row["payload_landing_planning_failure_reason"] = str(
            target.get("payload_landing_planning_failure_reason", ""))
        row["payload_ground_clearance_m"] = float(
            target.get("payload_ground_clearance_m", math.nan)
        )
        row["payload_prelift_clearance_target_m"] = float(
            target.get("payload_prelift_clearance_target_m", math.nan)
        )
        row["payload_prelift_ready"] = int(bool(
            target.get("payload_prelift_ready", False)
        ))
        row["payload_prelift_active"] = int(bool(
            target.get("payload_prelift_active", False)
        ))
        row["payload_prelift_stage"] = target.get("payload_prelift_stage", "")
        row["payload_prelift_failure_reason"] = target.get("payload_prelift_failure_reason", "")
        for field in ("payload_prelift_reference_rise_m", "payload_prelift_reference_lead_m"):
            row[field] = float(target.get(field, math.nan))
        for prefix in (
                "payload_position_error", "payload_velocity_error",
                "payload_attitude_error", "link_direction",
                "desired_link_direction", "raw_desired_link_direction",
                "link_direction_error", "raw_link_force",
                "kinematic_perpendicular_force",
                "payload_desired_force", "payload_desired_moment"):
            if prefix in target:
                self._with_xyz(row, prefix, target[prefix])
            elif prefix in ("raw_link_force", "kinematic_perpendicular_force"):
                for axis in "xyz":
                    row[prefix + "_" + axis] = math.nan
        if "link_force" in target:
            self._with_xyz(row, "limited_link_force", target["link_force"])
        else:
            for axis in "xyz":
                row["limited_link_force_" + axis] = math.nan
        for field in ("link_force_rate_limited", "link_force_magnitude_limited"):
            row[field] = int(bool(target[field])) if field in target else math.nan
        row["payload_control_dt_s"] = float(target.get("payload_control_dt_s", math.nan))
        if "desired_tension" in target:
            row["desired_tension_n"] = float(target["desired_tension"])
        if "desired_tension_signed" in target:
            row["desired_tension_signed_n"] = float(target["desired_tension_signed"])
        if "payload_yaw_enabled" in target:
            row["payload_yaw_enabled"] = int(bool(target["payload_yaw_enabled"]))
        for field in (
                "transport_blend", "transport_blend_dot", "transport_link_gain_scale",
                "ramp_position_hold_gain_scale",
                "transport_position_hold_gain_scale"):
            if field in target:
                row[field] = float(target[field])
        if "payload_target_position" in target:
            self._with_xyz(row, "payload_target", target["payload_target_position"])
        if "payload_target_rotation" in target:
            rpy = rotation_to_rpy(target["payload_target_rotation"])
            for name, value in zip(("roll", "pitch", "yaw"), rpy):
                row["payload_target_" + name + "_rad"] = float(value)
        for prefix in ("position_hold_position", "position_hold_velocity",
                       "position_hold_acceleration", "payload_target_body_rate",
                       "payload_target_body_rate_dot"):
            if prefix in target:
                self._with_xyz(row, prefix, target[prefix])
        self._with_xyz(row, "transport_position_hold_force",
                       command.get("transport_position_hold_force", np.zeros(3)))
        row["payload_attitude_reference_progress"] = float(
            target.get("payload_attitude_reference_progress", math.nan))
        row["payload_transport_force_mode"] = str(target.get("payload_transport_force_mode", ""))
        for field in ("transport_hold_integral_enabled", "transport_hold_integral_learning"):
            row[field] = int(bool(target.get(field, False)))
        row["transport_hold_integral_freeze_reason"] = str(
            target.get("transport_hold_integral_freeze_reason", "inactive"))
        for prefix in ("transport_hold_integral_bias", "transport_hold_integral_error"):
            self._with_xyz(row, prefix, target.get(prefix, np.zeros(3)))
        for prefix in ("transport_hold_integral_force", "transport_hold_integral_force_dot"):
            self._with_xyz(row, prefix, command.get(prefix, np.zeros(3)))
        row["transport_feedback_controller"] = str(command.get(
            "transport_feedback_controller", "differential_integral"))
        row["transport_full_pid_freeze_reason"] = str(command.get(
            "transport_full_pid_freeze_reason", "inactive_mode"))
        row["transport_full_pid_integral_gate_m"] = float(command.get(
            "transport_full_pid_integral_gate_m", math.nan))
        row["transport_full_pid_integral_velocity_weight"] = float(command.get(
            "transport_full_pid_integral_velocity_weight", math.nan))
        for field in ("transport_attitude_trim_enabled", "transport_attitude_trim_learning",
                      "transport_attitude_trim_limit_active", "transport_attitude_trim_rate_limit_active",
                      "transport_attitude_trim_acceleration_limit_active", "transport_attitude_trim_bridge_active"):
            row[field] = int(bool(target.get(field, False)))
        row["transport_attitude_trim_freeze_reason"] = str(
            target.get("transport_attitude_trim_freeze_reason", "inactive"))
        for prefix in ("transport_attitude_trim_error", "transport_attitude_trim_integral",
                       "transport_attitude_trim_position", "transport_attitude_trim_velocity",
                       "transport_attitude_trim_acceleration", "transport_attitude_trim_bridge_position",
                       "transport_attitude_trim_bridge_velocity", "transport_attitude_trim_bridge_acceleration",
                       "transport_attitude_trim_bridge_hold_position", "transport_attitude_trim_bridge_hold_velocity",
                       "transport_attitude_trim_bridge_hold_acceleration"):
            self._with_xyz(row, prefix, target.get(prefix, np.zeros(3)))
        for field in ("transport_full_pid_learning", "transport_full_pid_duplicate_tick",
                      "transport_full_pid_gate_open", "transport_full_pid_integral_limit_active",
                      "transport_full_pid_feedback_limit_active", "transport_full_pid_force_budget_limited",
                      "transport_full_pid_rate_budget_limited"):
            row[field] = int(bool(command.get(field, False)))
        for prefix in ("transport_full_pid_integral_state", "transport_full_pid_integral_state_delta",
                       "transport_full_pid_integral_state_limit", "transport_full_pid_integral_gain",
                       "transport_full_pid_force", "transport_full_pid_force_dot",
                       "transport_full_pid_pd_force", "transport_full_pid_pd_force_dot",
                       "transport_full_pid_feedback_force", "transport_full_pid_feedback_force_dot"):
            self._with_xyz(row, prefix, command.get(prefix, np.zeros(3)))
        for field in ("force_limit_active", "hold_feedback_limit_active", "body_rate_limit_active",
                      "collective_limit_active", "independent_mode", "transport_mode"):
            row[field] = int(bool(command.get(field, False)))
        if "kinematic_specific_acceleration" in target:
            self._with_xyz(row, "kinematic_specific_acceleration", target["kinematic_specific_acceleration"])
        self._with_xyz(
            row, "position_integral", command.get("position_integral", np.zeros(3))
        )
        self._with_xyz(row, "attitude_error", command["attitude_error"])
        self._with_xyz(row, "computed_rate", command.get("computed_body_rate", np.zeros(3)))
        self._with_xyz(row, "command_rate", command["body_rate_command"])
        actual_rpy = rotation_to_rpy(state["rotation"])
        desired_rpy = rotation_to_rpy(command["computed_rotation"])
        row.update(dict(zip(("roll_rad", "pitch_rad", "yaw_rad"), actual_rpy)))
        row.update(dict(zip(("desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad"), desired_rpy)))
        row["target_yaw_rad"] = float(target["yaw"])
        row["omega_c_method"] = str(command.get("omega_c_method", "zero"))
        row["command_thrust_newton"] = float(command["collective_thrust"])
        if command["collective_thrust"] <= 0.0:
            row["command_raw_thrust"] = 0
            row["command_raw_thrust_age_s"] = 0.0
        else:
            with self.lock:
                raw_thrust = self.latest_raw_thrust
                raw_received_time = self.latest_raw_thrust_received_time
            if raw_thrust is not None and raw_received_time is not None:
                row["command_raw_thrust"] = int(raw_thrust)
                row["command_raw_thrust_age_s"] = max(
                    0.0, now - float(raw_received_time)
                )
        if control_state is not None:
            for prefix in (
                    "ekf_position", "ekf_velocity", "ekf_acceleration",
                    "control_velocity", "mixed_acceleration", "control_acceleration"):
                if prefix in control_state:
                    self._with_xyz(row, prefix, control_state[prefix])
            row["ekf_state_valid"] = int(
                bool(control_state.get("ekf_state_valid", False))
            )
            row["ekf_state_age_s"] = float(
                control_state.get("ekf_state_age_s", math.inf)
            )
            row["ekf_kinematics_weight_effective"] = float(
                control_state.get("ekf_kinematics_weight_effective", 0.0)
            )
            row["ekf_position_error_m"] = float(
                control_state.get("ekf_position_error_m", math.inf)
            )
            row["ekf_position_consistent"] = int(
                bool(control_state.get("ekf_position_consistent", False))
            )
            row["ekf_kinematics_held"] = int(
                bool(control_state.get("ekf_kinematics_held", False))
            )
            row["ekf_fault_age_s"] = float(
                control_state.get("ekf_fault_age_s", math.inf)
            )
            row["ekf_status"] = str(control_state.get("ekf_status", ""))
        self.logger.write(row)

    def _shutdown(self):
        """ROS 退出钩子：先保证零推力包离开主机，再关闭日志文件。"""
        self._publish_zero()
        self.logger.close()


class MultiCtbrControllerNode:
    """Synchronize independent per-vehicle CTBR controllers on one Radio."""

    def __init__(self):
        entries = rospy.get_param("/crazyflies", [])
        requested_id = rospy.get_param("~cf_id", None)
        try:
            if requested_id is None:
                selected_entries = select_vehicle_entries(entries, ctbr_only=True)
                multi_mode = len(selected_entries) > 1
                self.vehicle_configs = validate_vehicle_entries(
                    selected_entries,
                    require_ctbr=True,
                    min_ctbr=1,
                    require_shared_radio=multi_mode,
                    require_phase=multi_mode,
                )
            else:
                self.vehicle_configs = select_vehicle_entries(
                    entries, cf_id=requested_id, ctbr_only=False
                )
        except (KeyError, TypeError, ValueError) as error:
            raise rospy.ROSInitException("多机飞机参数无效：%s" % error)

        self.transport_mode = str(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/transport_mode", "formation"
        )).strip().lower().replace("-", "_")
        if self.transport_mode == "slung_load":
            try:
                self.vehicle_configs = sorted(
                    self.vehicle_configs, key=lambda item: int(item["id"])
                )
                ordered_vehicle_ids(self.vehicle_configs)
            except (KeyError, TypeError, ValueError) as error:
                raise rospy.ROSInitException(
                    "slung_load 模式必须启用 CF3、CF4、CF5：%s" % error
                )

        try:
            self.takeoff_vehicle_count = int(
                rospy.get_param(
                    GLOBAL_CTBR_PARAMETER_ROOT + "/takeoff_vehicle_count",
                    len(self.vehicle_configs),
                )
            )
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException("takeoff_vehicle_count 无效：%s" % error)
        if self.takeoff_vehicle_count < 1:
            raise rospy.ROSInitException("takeoff_vehicle_count 必须为正整数")
        # ``~cf_id`` is an explicit single-vehicle compatibility mode used for
        # bench tests and recovery.  Do not reject it merely because the normal
        # formation parameter still contains the full fleet count.
        if requested_id is None and self.takeoff_vehicle_count != len(self.vehicle_configs):
            raise rospy.ROSInitException(
                "takeoff_vehicle_count=%d，但实际启用 %d 架 ctbr_enabled 飞机；"
                "请同步修改参数和 crazyflies.yaml" %
                (self.takeoff_vehicle_count, len(self.vehicle_configs))
            )

        if len(self.vehicle_configs) > 1 and rospy.get_param("~cf_prefix", ""):
            raise rospy.ROSInitException("多机模式不能使用统一的 ~cf_prefix")

        log_directory = rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/log_directory"
        )
        # Keep a legacy-looking filename for explicit/single-aircraft runs so
        # the realtime viewer does not wait for a second vehicle forever.
        self.multi_mode = requested_id is None and len(self.vehicle_configs) > 1
        log_prefix = (
            "multi_ctbr"
            if self.multi_mode else
            "cf%d_ctbr" % int(self.vehicle_configs[0]["id"])
        )
        self.logger = FlightCsvLogger(log_directory, log_prefix)
        self.rate_hz = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/control_rate_hz"
        ))
        self.trajectory_pause_abort_s = float(
            rospy.get_param(GLOBAL_CTBR_PARAMETER_ROOT + "/trajectory_pause_abort_s")
        )
        self.takeoff_position_gain_scale = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/takeoff_position_gain_scale", 1.0
        ))
        self.takeoff_velocity_gain_scale = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/takeoff_velocity_gain_scale", 1.0
        ))
        if (not math.isfinite(self.takeoff_position_gain_scale) or
                self.takeoff_position_gain_scale < 1.0 or
                not math.isfinite(self.takeoff_velocity_gain_scale) or
                self.takeoff_velocity_gain_scale < 1.0):
            self.logger.close()
            raise rospy.ROSInitException("起飞阶段增益倍率必须是不小于 1 的有限数")
        if self.rate_hz <= 0.0 or self.trajectory_pause_abort_s <= 0.0:
            self.logger.close()
            raise rospy.ROSInitException("多机控制频率和中止阈值必须为正数")

        self.payload_lock = threading.Lock()
        self.latest_payload_state = None
        self.last_valid_payload_geometry = None
        self.payload_topic = str(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/payload_mocap_topic",
            "/load/mocap_state",
        ))
        self.slung_controller = None
        self.payload_height_m = None
        self.payload_hold_s = 0.20
        self.payload_emergency_land_after_s = 0.30
        self.payload_activation_hold_s = 1.0
        self.payload_tension_ramp_s = 2.0
        self.payload_reference_lift_s = 4.0
        self.payload_target_height_m = math.nan
        self.payload_flight_validation_enabled = False
        self.payload_flight_validation_transport_s = 10.0
        # One-shot mission state: intentionally survives _reset_payload_handoff.
        self.payload_validation_handoff_time = None
        self.payload_validation_landing_started_time = None
        self.payload_validation_landing_stage = ""
        self.payload_validation_ground_since = None
        self.payload_controlled_exit_reason = ""
        self.payload_ground_reference_samples = deque(maxlen=100)
        self.payload_ground_center_z = math.nan
        self.payload_ground_clearance_reference_m = 0.0
        self.payload_landing_profile = None
        self.payload_landing_failure_count = 0
        self.payload_landing_failure_reason = ""
        self.payload_landing_failure_snapshot = None
        self.payload_landing_retry_since = None
        self.payload_landing_retry_count = 0
        self.payload_landing_retry_tick = None
        self.fleet_numerical_fault_reason = ""
        self.fleet_numerical_fault_since = None
        self.fleet_tick_completed_vehicles = set()
        self.fleet_tick_initial_commands = {}
        self.payload_last_reference = None
        self.payload_landing_gain_snapshot = None
        self.payload_landing_rope_directions = None
        self.payload_landing_rope_distances = None
        self.payload_payload_aware_emergency = False
        self.payload_landing_max_speed_mps = .12
        self.payload_landing_max_acceleration_mps2 = .15
        self.payload_landing_gain_transition_s = 2.0
        self.payload_landing_unload_s = 2.0
        self.payload_touchdown_detector = None
        self.payload_unload_start_time = None
        self.payload_landing_trim_at_unload = None
        self.payload_emergency_support = {}
        # Session-level effective geometry survives per-handoff resets.
        self.payload_nominal_link_lengths = np.full(3, np.nan)
        self.payload_link_calibrator = None
        self.payload_link_calibration_enabled = False
        self.payload_link_calibration_applied = False
        self.payload_link_calibration_pending = None
        self.payload_link_calibration_max_relative_speed_mps = .08
        self.payload_link_calibration_hold_reference = None
        self.payload_link_measured_distances = np.full(3, np.nan)
        self.payload_takeup_outward_offset_m = 0.12
        self.payload_takeup_pre_tension_slack_m = 0.05
        self.payload_takeup_distance_tolerance_m = 0.03
        self.payload_takeup_confirm_s = 0.30
        self.payload_takeup_position_tolerance_m = 0.03
        self.payload_takeup_link_angle_tolerance_rad = math.radians(10.0)
        self.payload_transport_link_gain_scale = 0.0
        self.payload_transport_steady_link_gain_scale = 1.0
        self.payload_ramp_position_hold_gain_scale = 2.0
        self.payload_transport_position_hold_gain_scale = 0.0
        self.payload_transport_feedback_controller = "differential_integral"
        self.payload_transport_pid_integral_gain_scale = 1.0
        self.payload_transport_pid_integral_limit_scale = 1.0
        self.payload_transport_pid_integral_gate_m = 0.20
        self.payload_transport_pid_integral_velocity_weight = 1.0
        self.payload_transport_pid_force_limit_n = 0.06
        self.payload_transport_pid_force_rate_limit_nps = 0.004
        self.payload_attitude_trim_enabled = False
        self.payload_attitude_trim_parameters = {}
        self.payload_attitude_trim = None
        self.payload_attitude_trim_landing_bridge = None
        self.payload_yaw_enabled = True
        self.payload_velocity_limit_mps = np.ones(3)
        self.payload_velocity_jump_limit_mps = np.ones(3)
        self.payload_landing_approach_fraction = 0.55
        self.payload_velocity_filter = None
        self.payload_velocity_filter_lock = threading.Lock()
        self.payload_state_observer = None
        self.payload_state_observer_lock = threading.Lock()
        self.payload_fault_since = None
        self.last_transport_overrides = {}
        self.payload_transport_active = False
        self.payload_ready_since = None
        self.payload_transport_start_time = None
        self.payload_transport_start_position = None
        self.payload_transport_start_rotation = None
        self.payload_transport_goal_rotation = None
        self.payload_attitude_reference_enabled = False
        self.payload_attitude_reference_s = 12.0
        self.payload_equilibrium_reference_enabled = False
        self.payload_transport_initial_link_units = None
        self.payload_transport_initial_link_distances = None
        self.payload_transport_goal_link_units = None
        self.payload_transport_vehicle_start_positions = None
        self.payload_takeup_start_time = None
        self.payload_takeup_start_positions = None
        self.payload_takeup_target_positions = None
        self.payload_takeup_ready_since = None
        self.payload_prelift_ready_since = None
        self.payload_prelift_clearance_m = 0.10
        self.payload_prelift_parameters = {}
        self.payload_prelift_integral_parameters = {}
        self.payload_landing_integral_parameters = {}
        self.payload_prelift_reference = None
        self.payload_last_takeup_reference = None
        self.payload_geometry_ready_since = None
        self.payload_mission_hold_time = None
        self.payload_handoff_reference_positions = None
        self.payload_handoff_forces = None
        self.payload_transport_guard = SlungTransportGuard()
        self.payload_support_trim = SlungSupportTrim()
        self.payload_transport_integral = SlungDifferentialHoldIntegral()
        self.payload_transport_integral_enabled = False
        self.payload_transport_integral_last_time = None
        self.payload_safety_reason = ""
        self.payload_minimum_vehicle_distance_m = math.nan
        self.payload_predicted_vehicle_distance_m = math.nan
        self.payload_prelift_confirm_s = 0.50
        self.payload_prelift_vertical_velocity_tolerance_mps = 0.05
        self.payload_ground_clearance_m = math.nan
        self.payload_prelift_ready = False
        self.payload_takeup_taut_latched = False
        self.payload_takeup_diagnostic_interval_s = 1.0
        self.payload_takeup_diagnostic_last_time = None
        self.payload_landing_start_time = None
        self.payload_landing_start_position = None
        self.payload_path_publisher = rospy.Publisher(
            "/load/path", Path, queue_size=1, latch=True
        )
        self.payload_path_message = Path()
        self.payload_path_message.header.frame_id = str(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/path_frame_id", "world"
        ))
        self.payload_path_last_publish_time = 0.0
        self.payload_path_publish_interval_s = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/path_publish_interval_s", 0.1))
        self.payload_path_max_poses = int(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/path_max_poses", 1000))
        if (not math.isfinite(self.payload_path_publish_interval_s)
                or self.payload_path_publish_interval_s <= 0):
            raise rospy.ROSInitException("path_publish_interval_s 必须为正的有限数")
        if self.payload_path_max_poses <= 0:
            raise rospy.ROSInitException("path_max_poses 必须为正整数")
        if self.transport_mode == "slung_load":
            try:
                payload = rospy.get_param(PAYLOAD_PARAMETER_ROOT)
                size = np.asarray(payload["size_m"], dtype=float).reshape(3)
                attachment_points = attachment_points_from_yaml(
                    payload["attachment_points_m"]
                )
                attitude_bandwidth = np.asarray(
                    payload["attitude_bandwidth_hz"], dtype=float
                ).reshape(3)
                inertia_diag = np.array([
                    float(payload["mass_kg"]) * (size[1] ** 2 + size[2] ** 2) / 12.0,
                    float(payload["mass_kg"]) * (size[0] ** 2 + size[2] ** 2) / 12.0,
                    float(payload["mass_kg"]) * (size[0] ** 2 + size[1] ** 2) / 12.0,
                ])
                link_lengths = link_lengths_vector(
                    payload.get("link_lengths_m", payload.get("link_length_m")),
                    "link_lengths_m",
                )
                self.payload_nominal_link_lengths = link_lengths.copy()
                enabled = payload.get("link_calibration_enabled", False)
                if not isinstance(enabled, bool):
                    raise ValueError("link_calibration_enabled 必须为布尔值")
                self.payload_link_calibration_enabled = enabled
                try:
                    self.payload_link_calibrator = SlungLinkCalibrator(
                        link_lengths,
                        window_s=float(payload.get("link_calibration_window_s", 5.)),
                        max_adjustment_m=float(payload.get("link_calibration_max_adjustment_m", .03)),
                        max_spread_m=float(payload.get("link_calibration_max_spread_m", .006)),
                        max_drift_mps=float(payload.get("link_calibration_max_drift_mps", .003)),
                        max_sample_gap_s=float(payload.get("link_calibration_max_sample_gap_s", .10)),
                        min_samples=payload.get("link_calibration_min_samples", 100),
                    )
                    self.payload_link_calibration_max_relative_speed_mps = float(
                        payload.get("link_calibration_max_relative_speed_mps", .08))
                    if (not math.isfinite(self.payload_link_calibration_max_relative_speed_mps)
                            or self.payload_link_calibration_max_relative_speed_mps <= 0):
                        raise ValueError("max_relative_speed_mps 必须为正的有限数")
                except (TypeError, ValueError) as error:
                    raise ValueError("link_calibration 参数无效：%s" % error)
                load_cfg = SlungLoadConfig(
                    payload_mass_kg=float(payload["mass_kg"]),
                    gravity_mps2=float(rospy.get_param(
                        GLOBAL_CTBR_PARAMETER_ROOT + "/gravity_mps2", 9.80665
                    )),
                    payload_size_m=size,
                    attachment_points_m=attachment_points,
                    link_length_m=float(np.mean(link_lengths)),
                    link_lengths_m=link_lengths,
                    position_gain=np.asarray(payload["position_gain"], dtype=float),
                    velocity_gain=np.asarray(payload["velocity_gain"], dtype=float),
                    integral_gain=np.asarray(payload["integral_gain"], dtype=float),
                    integral_limit=np.asarray(payload["integral_limit"], dtype=float),
                    c1=float(payload["c1"]),
                    force_norm_epsilon=float(payload["force_norm_epsilon"]),
                    tension_pinv_tolerance=float(payload["tension_pinv_tolerance"]),
                    link_kq=float(payload["link_kq"]),
                    link_komega=float(payload["link_komega"]),
                    link_integral_gain=float(payload["link_integral_gain"]),
                    link_integral_limit=np.asarray(
                        payload["link_integral_limit"], dtype=float
                    ),
                    outward_bias_fraction=float(payload.get("outward_bias_fraction", 0.0)),
                    outward_bias_max_n=float(payload.get("outward_bias_max_n", 0.0)),
                    force_dot_limit_nps=float(payload.get("force_dot_limit_nps", 20.0)),
                    link_force_correction_limit_n=float(payload.get(
                        "link_force_correction_limit_n", 0.12
                    )),
                    link_force_correction_rate_limit_nps=float(payload.get(
                        "link_force_correction_rate_limit_nps", 5.0
                    )),
                    transport_force_mode=str(payload.get("transport_force_mode", "coupled")),
                )
                self.slung_controller = SlungLoadController(load_cfg)
                self.payload_height_m = float(size[2])
                self.payload_hold_s = float(payload.get("mocap_hold_s", 0.20))
                self.payload_emergency_land_after_s = float(
                    payload.get("emergency_land_after_s", 0.30)
                )
                payload_filter_cutoff_hz = float(
                    payload.get("velocity_filter_cutoff_hz", 5.0)
                )
                payload_filter_max_dt = float(
                    payload.get("velocity_filter_max_dt", 0.05)
                )
                self.payload_velocity_limit_mps = np.asarray(
                    payload.get("velocity_limit_mps", [1.0, 1.0, 1.0]),
                    dtype=float,
                ).reshape(-1)
                self.payload_velocity_jump_limit_mps = np.asarray(
                    payload.get("velocity_jump_limit_mps", [0.8, 0.8, 0.8]),
                    dtype=float,
                ).reshape(-1)
                if self.payload_velocity_limit_mps.size == 1:
                    self.payload_velocity_limit_mps = np.full(
                        3, float(self.payload_velocity_limit_mps[0])
                    )
                if self.payload_velocity_jump_limit_mps.size == 1:
                    self.payload_velocity_jump_limit_mps = np.full(
                        3, float(self.payload_velocity_jump_limit_mps[0])
                    )
                if (self.payload_velocity_limit_mps.size != 3 or
                        self.payload_velocity_jump_limit_mps.size != 3 or
                        not np.all(np.isfinite(self.payload_velocity_limit_mps)) or
                        not np.all(np.isfinite(self.payload_velocity_jump_limit_mps)) or
                        np.any(self.payload_velocity_limit_mps <= 0.0) or
                        np.any(self.payload_velocity_jump_limit_mps <= 0.0)):
                    raise ValueError(
                        "负载速度限幅和跳变限幅必须是三个正的有限数值"
                    )
                self.payload_velocity_filter = PayloadVelocityFilter(
                    cutoff_hz=payload_filter_cutoff_hz,
                    max_dt=payload_filter_max_dt,
                    max_speed_mps=self.payload_velocity_limit_mps,
                    max_jump_mps=self.payload_velocity_jump_limit_mps,
                )
                observer_enabled = bool(payload.get("state_observer_enabled", True))
                if observer_enabled:
                    self.payload_state_observer = PayloadStateObserver(
                        position_gain=float(payload.get("observer_position_gain", 0.35)),
                        velocity_gain=float(payload.get("observer_velocity_gain", 0.08)),
                        acceleration_gain=float(payload.get("observer_acceleration_gain", 0.02)),
                        attitude_gain=float(payload.get("observer_attitude_gain", 0.35)),
                        angular_rate_gain=float(payload.get("observer_angular_rate_gain", 0.08)),
                        max_dt=float(payload.get("observer_max_dt", 0.05)),
                        max_velocity_mps=payload.get(
                            "observer_max_velocity_mps", [1.0, 1.0, 1.0]
                        ),
                        max_acceleration_mps2=payload.get(
                            "observer_max_acceleration_mps2", [8.0, 8.0, 8.0]
                        ),
                        max_body_rate_rps=payload.get(
                            "observer_max_body_rate_rps", [8.0, 8.0, 8.0]
                        ),
                        min_samples=int(payload.get("observer_min_samples", 3)),
                    )
                    rospy.loginfo(
                        "负载状态观测器已启用：仅使用 NOKOV 位姿/姿态估计速度、加速度和角速度；"
                        "至少等待 %d 个样本。",
                        self.payload_state_observer.min_samples,
                    )
                self.payload_activation_hold_s = float(
                    payload.get("activation_hold_s", 1.0)
                )
                self.payload_tension_ramp_s = float(
                    payload.get("tension_ramp_s", 2.0)
                )
                self.payload_reference_lift_s = float(
                    payload.get("reference_lift_s", 4.0)
                )
                target_height = payload.get("target_payload_height_m", None)
                self.payload_target_height_m = (
                    math.nan if target_height is None else float(target_height)
                )
                enabled = payload.get("flight_validation_enabled", False)
                if not isinstance(enabled, bool):
                    raise ValueError("flight_validation_enabled 必须为布尔值")
                self.payload_flight_validation_enabled = enabled
                self.payload_flight_validation_transport_s = float(
                    payload.get("flight_validation_transport_s", 10.0))
                if (not math.isfinite(self.payload_flight_validation_transport_s)
                        or self.payload_flight_validation_transport_s <= 0):
                    raise ValueError("flight_validation_transport_s 必须为正的有限数")
                self.payload_takeup_outward_offset_m = float(
                    payload.get("takeup_outward_offset_m", 0.12)
                )
                self.payload_takeup_pre_tension_slack_m = float(
                    payload.get("takeup_pre_tension_slack_m", 0.05)
                )
                self.payload_takeup_distance_tolerance_m = float(
                    payload.get("takeup_distance_tolerance_m", 0.03)
                )
                self.payload_takeup_confirm_s = float(
                    payload.get("takeup_confirm_s", 0.30)
                )
                self.payload_takeup_diagnostic_interval_s = float(
                    payload.get("takeup_diagnostic_interval_s", 1.0)
                )
                self.payload_takeup_position_tolerance_m = float(
                    payload.get("takeup_position_tolerance_m", 0.03)
                )
                self.payload_takeup_link_angle_tolerance_rad = math.radians(float(
                    payload.get("takeup_link_angle_tolerance_deg", 10.0)
                ))
                self.payload_prelift_clearance_m = float(
                    payload.get("prelift_clearance_m", 0.10)
                )
                self.payload_prelift_parameters = {
                    "clearance_m": self.payload_prelift_clearance_m,
                    "clearance_margin_m": float(payload.get("prelift_clearance_margin_m", 0.02)),
                    "speed_mps": float(payload.get("prelift_speed_mps", 0.05)),
                    "acceleration_mps2": float(payload.get("prelift_acceleration_mps2", 0.05)),
                    "max_rise_m": float(payload.get("prelift_max_reference_rise_m", 1.8)),
                    "max_lead_m": float(payload.get("prelift_max_reference_lead_m", 1.6)),
                    "timeout_s": float(payload.get("prelift_timeout_s", 60.0)),
                    "alignment_duration_s": float(payload.get("prelift_alignment_duration_s", 2.0)),
                    "lead_hold_s": float(payload.get("prelift_lead_hold_s", 10.0)),
                    "position_gain_scale": payload.get(
                        "prelift_independent_position_gain_scale", [3.0, 3.0, 1.0]
                    ),
                    "velocity_gain_scale": payload.get(
                        "prelift_independent_velocity_gain_scale", [1.5, 1.5, 1.0]
                    ),
                    "max_tilt_rad": math.radians(float(
                        payload.get("prelift_max_tilt_deg", 12.0)
                    )),
                }
                SlungPreliftTrajectory(**self.payload_prelift_parameters)
                self.payload_prelift_integral_parameters = {
                    "prelift_vertical_integral_limit": float(payload.get(
                        "prelift_vertical_integral_limit", 8.0)),
                    "prelift_vertical_integral_rate_limit": float(payload.get(
                        "prelift_vertical_integral_rate_limit", 0.5)),
                    "prelift_vertical_feedback_limit_mps2": float(payload.get(
                        "prelift_vertical_feedback_limit_mps2", 12.0)),
                    "prelift_vertical_integral_gate_m": (
                        self.payload_prelift_parameters["max_lead_m"] + 0.05),
                }
                if any(not math.isfinite(v) or v <= 0.0
                       for v in self.payload_prelift_integral_parameters.values()):
                    raise ValueError("预抬竖直积分/加速度限幅必须为正的有限数")
                self.payload_landing_integral_parameters = {
                    "landing_vertical_integral_limit": float(payload.get(
                        "landing_vertical_integral_limit", 2.0)),
                    "landing_vertical_integral_rate_limit": float(payload.get(
                        "landing_vertical_integral_rate_limit", 0.5)),
                    "landing_vertical_feedback_limit_mps2": float(payload.get(
                        "landing_vertical_feedback_limit_mps2", 8.0)),
                }
                if any(not math.isfinite(v) or v <= 0.0
                       for v in self.payload_landing_integral_parameters.values()):
                    raise ValueError("landing_vertical 积分/加速度限幅必须为正的有限数")
                self.payload_prelift_confirm_s = float(
                    payload.get("prelift_confirm_s", 0.50)
                )
                self.payload_prelift_vertical_velocity_tolerance_mps = float(
                    payload.get("prelift_vertical_velocity_tolerance_mps", 0.05)
                )
                self.payload_transport_link_gain_scale = float(
                    payload.get("transport_link_gain_scale", 0.0)
                )
                self.payload_transport_steady_link_gain_scale = float(
                    payload.get("transport_link_steady_gain_scale", 1.0)
                )
                self.payload_ramp_position_hold_gain_scale = float(
                    payload.get("ramp_position_hold_gain_scale", 2.0)
                )
                self.payload_transport_position_hold_gain_scale = float(
                    payload.get("transport_position_hold_gain_scale", 0.0)
                )
                self.payload_transport_feedback_controller = str(payload.get(
                    "transport_feedback_controller", "differential_integral"))
                if self.payload_transport_feedback_controller not in (
                        "differential_integral", "native_position_pid"):
                    raise ValueError(
                        "transport_feedback_controller 必须为 differential_integral 或 native_position_pid")
                if (self.payload_transport_feedback_controller == "native_position_pid"
                        and load_cfg.transport_force_mode != "reference_cascade"):
                    raise ValueError("native_position_pid 仅支持 reference_cascade")
                self.payload_transport_pid_integral_gain_scale = float(payload.get(
                    "transport_position_pid_integral_gain_scale", 1.0))
                self.payload_transport_pid_integral_limit_scale = float(payload.get(
                    "transport_position_pid_integral_limit_scale", 1.0))
                self.payload_transport_pid_integral_gate_m = float(payload.get(
                    "transport_position_pid_integral_gate_m", 0.20))
                integral_velocity_weight = payload.get(
                    "transport_position_pid_integral_velocity_weight", 1.0)
                if (isinstance(integral_velocity_weight, (bool, np.bool_))
                        or not isinstance(integral_velocity_weight, (int, float, np.integer, np.floating))
                        or not math.isfinite(float(integral_velocity_weight))
                        or integral_velocity_weight < 0.0):
                    raise ValueError("transport_position_pid_integral_velocity_weight 必须为非负的有限数")
                self.payload_transport_pid_integral_velocity_weight = float(integral_velocity_weight)
                self.payload_transport_pid_force_limit_n = float(payload.get(
                    "transport_position_pid_force_limit_n", 0.06))
                self.payload_transport_pid_force_rate_limit_nps = float(payload.get(
                    "transport_position_pid_force_rate_limit_nps", 0.004))
                if any(not math.isfinite(v) or v <= 0.0 for v in (
                        self.payload_transport_pid_integral_gain_scale,
                        self.payload_transport_pid_integral_limit_scale,
                        self.payload_transport_pid_integral_gate_m,
                        self.payload_transport_pid_force_limit_n,
                        self.payload_transport_pid_force_rate_limit_nps)):
                    raise ValueError("transport native position PID 参数必须为正的有限数")
                self.payload_attitude_reference_enabled = payload.get(
                    "transport_attitude_reference_enabled", False)
                self.payload_equilibrium_reference_enabled = payload.get(
                    "transport_equilibrium_reference_enabled", False)
                self.payload_attitude_reference_s = float(payload.get(
                    "transport_attitude_reference_s", self.payload_reference_lift_s))
                if (not isinstance(self.payload_attitude_reference_enabled, bool)
                        or not isinstance(self.payload_equilibrium_reference_enabled, bool)
                        or not math.isfinite(self.payload_attitude_reference_s)
                        or self.payload_attitude_reference_s <= 0.0):
                    raise ValueError("transport_attitude_reference_enabled/s 必须为布尔值/正有限秒数")
                if (load_cfg.transport_force_mode == "reference_cascade"
                        and (not self.payload_attitude_reference_enabled
                             or not self.payload_equilibrium_reference_enabled
                             or self.payload_ramp_position_hold_gain_scale <= 0.0
                             or self.payload_transport_position_hold_gain_scale <= 0.0)):
                    raise ValueError("reference_cascade 需要启用姿态/平衡构型参考以及正的位置保持增益")
                self.payload_attitude_trim_enabled = payload.get("transport_attitude_trim_enabled", False)
                if not isinstance(self.payload_attitude_trim_enabled, bool):
                    raise ValueError("transport_attitude_trim_enabled 必须为布尔值")
                if (self.payload_attitude_trim_enabled
                        and load_cfg.transport_force_mode != "reference_cascade"):
                    raise ValueError("transport_attitude_trim 仅支持 reference_cascade")
                self.payload_attitude_trim_parameters = dict(
                    gain_per_s=payload.get("transport_attitude_trim_gain_per_s", [.05, .05, .02]),
                    max_offset_m=payload.get("transport_attitude_trim_limit_m", .01),
                    max_rate_mps=payload.get("transport_attitude_trim_rate_limit_mps", .001),
                    max_acceleration_mps2=payload.get("transport_attitude_trim_acceleration_limit_mps2", .001),
                    adaptation_interval_s=payload.get("transport_attitude_trim_interval_s", 1.),
                )
                try:
                    self.payload_attitude_trim = SlungAttitudeReferenceTrim(
                        load_cfg.attachment_points_m, enabled=self.payload_attitude_trim_enabled,
                        **self.payload_attitude_trim_parameters)
                except (TypeError, ValueError) as error:
                    raise ValueError("transport_attitude_trim 参数无效：%s" % error) from error
                self.payload_transport_guard = SlungTransportGuard(
                    min_distance_m=float(payload.get("minimum_vehicle_distance_m", 0.30)),
                    prediction_s=float(payload.get("separation_prediction_s", 0.35)),
                    height_error_m=float(payload.get("transport_height_error_limit_m", 0.08)),
                    confirm_s=float(payload.get("transport_fault_confirm_s", 0.20)),
                )
                self.payload_support_trim = SlungSupportTrim(
                    limit_n=float(payload.get("transport_support_trim_limit_n", 0.25)),
                    integral_gain=float(payload.get("transport_support_integral_gain_npm_s", 0.20)),
                    rate_limit_nps=float(payload.get("transport_support_trim_rate_nps", 0.03)),
                )
                self.payload_transport_integral_enabled = payload.get("transport_hold_integral_enabled", False)
                if not isinstance(self.payload_transport_integral_enabled, bool):
                    raise ValueError("transport_hold_integral_enabled must be boolean")
                try:
                    self.payload_transport_integral = SlungDifferentialHoldIntegral(
                        gain_npm_s=payload.get("transport_hold_integral_gain_npm_s", .02),
                        limit_n=payload.get("transport_hold_integral_force_limit_n", .03),
                        rate_limit_nps=payload.get("transport_hold_integral_rate_limit_nps", .002),
                    )
                except ValueError as error:
                    raise ValueError("transport_hold_integral: %s" % error) from error
                self.payload_landing_max_speed_mps = float(payload.get("landing_payload_max_speed_mps", .12))
                self.payload_landing_max_acceleration_mps2 = float(payload.get("landing_payload_max_acceleration_mps2", .15))
                self.payload_landing_gain_transition_s = float(payload.get("landing_gain_transition_s", 2.0))
                self.payload_landing_unload_s = float(payload.get("landing_unload_s", 2.0))
                if any(not math.isfinite(v) or v <= 0 for v in (
                        self.payload_landing_max_speed_mps, self.payload_landing_max_acceleration_mps2,
                        self.payload_landing_gain_transition_s, self.payload_landing_unload_s)):
                    raise ValueError("负载降落速度/加速度/交接时间必须为正的有限数")
                self.payload_touchdown_parameters = dict(
                    clearance_tolerance_m=float(payload.get("landing_touchdown_tolerance_m", .015)),
                    contact_margin_m=float(payload.get("landing_touchdown_contact_margin_m", .006)),
                    speed_limit_mps=float(payload.get("landing_touchdown_speed_mps", .05)),
                    confirm_s=float(payload.get("landing_touchdown_confirm_s", .30)),
                    window_s=float(payload.get("landing_touchdown_window_s", .40)),
                    max_height_span_m=float(payload.get("landing_touchdown_height_span_m", .012)),
                    timeout_s=float(payload.get("landing_touchdown_timeout_s", 4.0)),
                )
                self.payload_touchdown_detector = PayloadTouchdownDetector(**self.payload_touchdown_parameters)
                self.payload_yaw_enabled = bool(
                    payload.get("yaw_channel_enabled", True)
                )
                self.payload_landing_approach_fraction = float(
                    payload.get("landing_approach_fraction", 0.55)
                )
                if self.payload_hold_s < 0.0 or self.payload_emergency_land_after_s < self.payload_hold_s:
                    raise ValueError(
                        "mocap_hold_s 和 emergency_land_after_s 时间参数无效"
                    )
                if (self.payload_activation_hold_s < 0.0 or
                        self.payload_tension_ramp_s <= 0.0 or
                        self.payload_reference_lift_s <= 0.0):
                    raise ValueError("负载接管/张力/抬升时间参数无效")
                if (math.isfinite(self.payload_target_height_m)
                        and self.payload_target_height_m <= 0.0):
                    raise ValueError("target_payload_height_m 必须为正的有限数")
                if (self.payload_takeup_outward_offset_m < 0.0 or
                        self.payload_takeup_outward_offset_m >= float(np.min(load_cfg.link_lengths_m)) or
                        self.payload_takeup_pre_tension_slack_m < 0.0 or
                        self.payload_takeup_pre_tension_slack_m >= float(np.min(load_cfg.link_lengths_m)) or
                        self.payload_takeup_distance_tolerance_m <= 0.0 or
                        self.payload_takeup_confirm_s < 0.0 or
                        not math.isfinite(self.payload_takeup_diagnostic_interval_s) or
                        self.payload_takeup_diagnostic_interval_s <= 0.0 or
                        self.payload_takeup_position_tolerance_m <= 0.0 or
                        not math.isfinite(self.payload_takeup_link_angle_tolerance_rad) or
                        self.payload_takeup_link_angle_tolerance_rad <= 0.0 or
                        not math.isfinite(self.payload_prelift_clearance_m) or
                        self.payload_prelift_clearance_m < 0.0 or
                        not math.isfinite(self.payload_prelift_confirm_s) or
                        self.payload_prelift_confirm_s < 0.0 or
                        not math.isfinite(
                            self.payload_prelift_vertical_velocity_tolerance_mps
                        ) or
                        self.payload_prelift_vertical_velocity_tolerance_mps <= 0.0 or
                        not math.isfinite(self.payload_transport_link_gain_scale) or
                        not 0.0 <= self.payload_transport_link_gain_scale <= 1.0 or
                        not math.isfinite(self.payload_transport_steady_link_gain_scale) or
                        not 0.0 <= self.payload_transport_steady_link_gain_scale <= 1.0 or
                        not math.isfinite(self.payload_ramp_position_hold_gain_scale) or
                        self.payload_ramp_position_hold_gain_scale < 0.0 or
                        not math.isfinite(self.payload_transport_position_hold_gain_scale) or
                        self.payload_transport_position_hold_gain_scale < 0.0 or
                        not 0.0 < self.payload_landing_approach_fraction <= 1.0):
                    raise ValueError("负载 TAKEUP 几何参数无效")
                self.payload_takeup_target_distance_m = takeup_target_distance(
                    load_cfg.link_lengths_m, self.payload_takeup_pre_tension_slack_m
                )
                self.payload_attitude_gain = inertia_diag * (
                    2.0 * math.pi * attitude_bandwidth
                ) ** 2
                self.payload_rate_gain = (
                    2.0 * float(payload["attitude_damping_ratio"])
                    * 2.0 * math.pi * attitude_bandwidth * inertia_diag
                )
            except (KeyError, TypeError, ValueError) as error:
                self.logger.close()
                raise rospy.ROSInitException("slung_payload 参数无效：%s" % error)

        mission_start = time.monotonic()
        self.vehicles = [
            CtbrControllerNode(
                vehicle_config=vehicle_config,
                logger=self.logger,
                auto_timer=False,
                register_shutdown=False,
                start_time=mission_start,
            )
            for vehicle_config in self.vehicle_configs
        ]
        if self.transport_mode == "slung_load":
            rospy.loginfo("吊运名义绳长 CF3/CF4/CF5=%s；本次运行有效绳长校准=%s。",
                          self.payload_nominal_link_lengths.tolist(), self.payload_link_calibration_enabled)
            self.payload_subscriber = rospy.Subscriber(
                self.payload_topic, MocapState, self._payload_state_callback,
                queue_size=1,
            )
        self.mission_start_time = mission_start
        self.fleet_gate_open = False
        self.global_abort_reason = ""
        # Terminal-only mission stage monitor.  The CSV keeps the exact
        # per-vehicle ``flight_phase``; this compact view makes the shared
        # multi-vehicle state machine easy to follow during a flight.
        self.last_terminal_stage = None
        self.last_tick = mission_start
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "多机 CTBR 控制器已启动：%d 架飞机，共用一份日志 %s",
            len(self.vehicles), self.logger.path,
        )
        if self.transport_mode == "slung_load":
            rospy.loginfo(
                "吊运飞机反馈：%s；native I 增益倍率 %.3f、状态上限倍率 %.3f、"
                "门限 %.3f m、积分速度权重 %.3f、附加 I 力上限 %.3f N、变化率 %.3f N/s。",
                self.payload_transport_feedback_controller,
                self.payload_transport_pid_integral_gain_scale,
                self.payload_transport_pid_integral_limit_scale,
                self.payload_transport_pid_integral_gate_m,
                self.payload_transport_pid_integral_velocity_weight,
                self.payload_transport_pid_force_limit_n,
                self.payload_transport_pid_force_rate_limit_nps,
            )
            if self.payload_flight_validation_enabled:
                rospy.loginfo(
                    "[实飞验证] 已启用：从进入 TAUT_RAMP 开始计时 %.2f s 后受控降落"
                    "（包含张力渐增和参考抬升，不等待完整轨迹结束）。",
                    self.payload_flight_validation_transport_s,
                )
            rospy.loginfo(
                "吊运主流程共 5 个阶段：1 起飞前保持，2 独立起飞，"
                "3 TAKEUP 收绳，4 负载接管/轨迹跟踪，5 降落。"
                "等待 EKF/全机就绪属于起飞前准备。"
            )

    def _terminal_mission_stage(self):
        """Return the compact five-stage view and exact per-vehicle phases."""
        phases = [str(vehicle.flight_phase) for vehicle in self.vehicles]
        phase_text = ", ".join(
            "CF%d=%s" % (vehicle.vehicle_id, phase)
            for vehicle, phase in zip(self.vehicles, phases)
        )

        if self.global_abort_reason or "aborted" in phases:
            return 5, "安全中止", phase_text
        if (any(p in ("landing", "landing_settle", "landed") for p in phases) and
                self.payload_validation_landing_started_time is not None):
            return 5, "实飞验证结束/受控降落", phase_text
        if "emergency_landing" in phases:
            return 5, "紧急降落", phase_text
        if any(phase in ("landing", "landing_settle", "landed")
               for phase in phases):
            return 5, "降落/落地确认", phase_text
        if (self.payload_transport_active or "tension_ramp" in phases or
                any(phase in (
                    "height_correction", "circle_entry", "circle",
                    "figure_eight", "hover", "final_hover"
                ) for phase in phases)):
            if self.payload_transport_active or "tension_ramp" in phases:
                return 4, "负载接管/轨迹跟踪", phase_text
            return 4, "轨迹跟踪（负载尚未接管）", phase_text
        if any(phase == "takeup" for phase in phases):
            return 3, "TAKEUP 收绳", phase_text
        if any(phase == "takeoff" for phase in phases):
            return 2, "独立起飞", phase_text
        if any(phase == "pre_takeoff_hold" for phase in phases):
            return 1, "起飞前保持", phase_text
        return 1, "起飞前准备/等待全机就绪", phase_text

    def _log_terminal_mission_stage(self):
        """Print one shared stage line whenever the fleet stage changes."""
        stage, label, phase_text = self._terminal_mission_stage()
        key = (stage, label, phase_text)
        if key == self.last_terminal_stage:
            return
        self.last_terminal_stage = key
        log_prefix = "吊运阶段" if self.transport_mode == "slung_load" else "任务阶段"
        rospy.loginfo(
            "[%s] %d/5 %s；内部状态：%s",
            log_prefix, stage, label, phase_text,
        )

    def _payload_state_callback(self, message):
        """Cache NOKOV load state after converting top-surface origin to center."""
        try:
            position = np.array([
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ], dtype=float)
            rotation = quaternion_to_rotation(
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            )
            if not bool(message.valid) or not np.all(np.isfinite(position)):
                observer = getattr(self, "payload_state_observer", None)
                if observer is not None:
                    with self.payload_state_observer_lock:
                        observer.reset()
                with self.payload_lock:
                    self.latest_payload_state = None
                return
            center_position = mocap_top_surface_to_center(
                position, rotation, self.payload_height_m
            )
            velocity = np.array([
                message.twist.linear.x,
                message.twist.linear.y,
                message.twist.linear.z,
            ], dtype=float)
            acceleration = np.array([
                message.acceleration.x,
                message.acceleration.y,
                message.acceleration.z,
            ], dtype=float)
            raw_velocity = velocity.copy()
            body_rate = np.array([
                message.twist.angular.x,
                message.twist.angular.y,
                message.twist.angular.z,
            ], dtype=float)
            if not np.all(np.isfinite(velocity)):
                velocity = np.zeros(3)
                raw_velocity = velocity.copy()
            if not np.all(np.isfinite(acceleration)):
                acceleration = np.zeros(3)
            if not np.all(np.isfinite(body_rate)):
                body_rate = np.zeros(3)
            raw_body_rate = body_rate.copy()
            sample_time = float(message.header.stamp.to_sec())
            if not math.isfinite(sample_time) or sample_time <= 0.0:
                raise ValueError("负载 header.stamp 无效")
            observer = getattr(self, "payload_state_observer", None)
            if observer is not None:
                # The observer consumes only pose and attitude.  The twist and
                # acceleration fields remain raw diagnostics and are never fed
                # into the load controller.
                observer_lock = getattr(
                    self, "payload_state_observer_lock", threading.Lock()
                )
                with observer_lock:
                    observed = observer.update(
                        center_position, rotation, sample_time
                    )
                if observed is None:
                    raise ValueError("负载位姿观测器拒绝当前样本")
                velocity = np.asarray(observed["velocity"], dtype=float)
                acceleration = np.asarray(observed["acceleration"], dtype=float)
                body_rate = np.asarray(observed["body_rate"], dtype=float)
                filtered_derivatives_valid = bool(
                    observed.get("derivatives_valid", False)
                )
            else:
                # Compatibility path for unit tests and legacy callers that
                # construct a node without the new pose observer.
                if not bool(message.derivatives_valid):
                    with self.payload_velocity_filter_lock:
                        self.payload_velocity_filter.reset()
                    velocity = np.zeros(3)
                    acceleration = np.zeros(3)
                    filtered_derivatives_valid = False
                else:
                    with self.payload_velocity_filter_lock:
                        velocity, acceleration, filtered_derivatives_valid = (
                            self.payload_velocity_filter.update(raw_velocity, sample_time)
                        )
                    if velocity is None:
                        raise ValueError("负载速度低通滤波器拒绝当前样本")
            state = {
                "valid": True,
                "position": center_position,
                "velocity": velocity,
                "acceleration": acceleration,
                "raw_velocity": raw_velocity,
                "raw_acceleration": np.array([
                    message.acceleration.x,
                    message.acceleration.y,
                    message.acceleration.z,
                ], dtype=float),
                "raw_body_rate": raw_body_rate,
                "rotation": rotation,
                "body_rate": body_rate,
                "received_time": time.monotonic(),
                "sample_time": sample_time,
                "derivatives_valid": bool(
                    filtered_derivatives_valid
                ),
            }
        except (AttributeError, TypeError, ValueError):
            with self.payload_lock:
                self.latest_payload_state = None
            return
        with self.payload_lock:
            self.latest_payload_state = state
            self.last_valid_payload_geometry = {
                "position": state["position"].copy(),
                "rotation": state["rotation"].copy(),
            }
        self._publish_payload_path(state)

    def _publish_payload_path(self, state):
        """Publish the measured load-center path for RViz."""
        now = time.monotonic()
        interval = self.payload_path_publish_interval_s
        if now - self.payload_path_last_publish_time < interval:
            return
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.payload_path_message.header.frame_id
        pose.pose.position.x = float(state["position"][0])
        pose.pose.position.y = float(state["position"][1])
        pose.pose.position.z = float(state["position"][2])
        rotation = state.get("rotation")
        if rotation is not None:
            # A Path pose needs a quaternion.  Keep this conversion local so
            # the plotted/path position remains the geometric load center.
            pose.pose.orientation.w = 1.0
        self.payload_path_message.header.stamp = pose.header.stamp
        self.payload_path_message.poses.append(pose)
        if len(self.payload_path_message.poses) > self.payload_path_max_poses:
            del self.payload_path_message.poses[:-self.payload_path_max_poses]
        self.payload_path_last_publish_time = now
        self.payload_path_publisher.publish(self.payload_path_message)

    def _payload_ready(self, now):
        if self.transport_mode != "slung_load":
            return True
        with self.payload_lock:
            state = self.latest_payload_state
        if state is None:
            return False
        return (
            state.get("valid", False)
            and state.get("derivatives_valid", False)
            and math.isfinite(float(now) - float(state["received_time"]))
            and float(now) - float(state["received_time"]) <= self.vehicles[0].state_timeout
        )

    def _payload_takeup_positions(self):
        """Return TAKEUP positions at the current load height and cable distance."""
        with self.payload_lock:
            geometry = self.last_valid_payload_geometry
        if geometry is None or self.slung_controller is None:
            return None
        try:
            equilibrium_links = self.slung_controller.equilibrium_link_units(
                geometry["rotation"]
            )
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            # Keep the MATLAB fallback behavior if the allocation matrix is
            # temporarily unusable; do not let one bad load pose kill the
            # multi-vehicle timer thread.
            rospy.logwarn_throttle(
                1.0,
                "负载静态平衡绳向求解失败，TAKEUP 使用外张回退几何：%s",
                error,
            )
            equilibrium_links = None
        takeup_payload_position = np.asarray(
            geometry["position"], dtype=float
        ).copy()
        # TAKEUP first establishes cable geometry at the current load height.
        # The separate pre-lift segment raises all three aircraft afterwards.
        return takeup_vehicle_targets(
            takeup_payload_position, geometry["rotation"],
            self.slung_controller.config.attachment_points_m,
            self.payload_takeup_target_distance_m,
            self.payload_takeup_outward_offset_m,
            link_units=equilibrium_links,
        ).T

    def _payload_takeup_overrides(self, now):
        """Confirm measured taut cables, then keep pulling until measured clearance."""
        if self.transport_mode != "slung_load" or self.slung_controller is None:
            return {}
        states = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None:
                return {}
            states.append(state)
        measured = np.asarray([state["position"] for state in states], dtype=float)
        targets = (self._payload_takeup_positions()
                   if self.payload_takeup_start_positions is None else None)
        if self.payload_takeup_start_positions is None:
            if targets is None:
                return {}
            self.payload_takeup_start_positions = measured.copy()
            self.payload_takeup_target_positions = targets.copy()
            self.payload_takeup_start_time = float(now)
            self.payload_mission_hold_time = float(now)
        duration = max(float(self.vehicles[0].trajectory_config.takeup_duration_s
                             or self.vehicles[0].trajectory_config.takeoff_settle_s), 1e-6)
        elapsed = float(now) - self.payload_takeup_start_time
        with self.payload_lock:
            payload_state = self.latest_payload_state
        payload_ready = self._payload_ready(now)
        clearance = (payload_ground_clearance(
            payload_state["position"], payload_state["rotation"],
            self.slung_controller.config.payload_size_m,
        ) if payload_ready else math.nan)
        self.payload_ground_clearance_m = clearance
        if self.payload_prelift_reference is not None:
            reference = self.payload_prelift_reference.update(measured, clearance, now)
        else:
            profile = smoothstep5_profile(elapsed, duration)
            displacement = self.payload_takeup_target_positions - self.payload_takeup_start_positions
            reference = dict(
                positions=self.payload_takeup_start_positions + profile[0] * displacement,
                velocities=profile[1] * displacement, accelerations=profile[2] * displacement,
                stage="takeup", active=False, failed=False, reason="", rise_m=0.0,
            )
            if self.payload_link_calibration_hold_reference is not None:
                reference.update(positions=self.payload_link_calibration_hold_reference.copy(),
                                 velocities=np.zeros((3, 3)), accelerations=np.zeros((3, 3)))
            if not payload_ready and self.payload_last_takeup_reference is not None:
                reference = dict(self.payload_last_takeup_reference,
                                 velocities=np.zeros((3, 3)), accelerations=np.zeros((3, 3)))
            if payload_ready and elapsed >= duration:
                distances = cable_distances(
                    payload_state["position"], payload_state["rotation"],
                    self.slung_controller.config.attachment_points_m, measured,
                )
                taut = takeup_distance_ready(
                    distances, self.slung_controller.config.link_lengths_m,
                    self.payload_takeup_target_distance_m, self.payload_takeup_distance_tolerance_m,
                )
                if taut:
                    if self.payload_geometry_ready_since is None:
                        self.payload_geometry_ready_since = float(now)
                    if now - self.payload_geometry_ready_since >= self.payload_takeup_confirm_s:
                        self.payload_takeup_taut_latched = True
                        self.payload_prelift_reference = SlungPreliftTrajectory(
                            **self.payload_prelift_parameters
                        )
                        self.payload_prelift_reference.start(measured, reference["positions"], now)
                        self.payload_link_calibration_hold_reference = None
                        reference = self.payload_prelift_reference.snapshot()
                else:
                    self.payload_geometry_ready_since = None
            if elapsed > duration + self.payload_prelift_parameters["timeout_s"]:
                reference.update(failed=True, reason="takeup_timeout", stage="failed")
        self.payload_last_takeup_reference = reference
        self._update_payload_link_calibration(now, measured, payload_state, reference, duration, states)
        if reference["failed"]:
            self._begin_payload_emergency_landing(now, "预抬参考达到边界：" + reference["reason"])
            return {}
        overrides = {}
        for index, (vehicle, state) in enumerate(zip(self.vehicles, states)):
            target = dict(vehicle._target_for(state, now))
            target.update({
                "position": reference["positions"][index].copy(),
                "velocity": reference["velocities"][index].copy(),
                "acceleration": reference["accelerations"][index].copy(),
                "jerk": np.zeros(3),
                "independent_mode": True, "transport_mode": False,
                "takeup_active": True, "flight_phase": "takeup",
                "payload_prelift_active": reference["active"],
                "payload_prelift_stage": reference["stage"],
                "payload_prelift_reference_rise_m": reference["rise_m"],
                "payload_prelift_reference_lead_m": float(
                    reference["positions"][index, 2] - measured[index, 2]
                ),
                "prelift_vertical_integral_active": (
                    payload_ready and reference["stage"] in (
                        "rising", "braking", "holding", "lead_hold"
                    )
                ),
                "independent_position_gain_scale": (
                    self.payload_prelift_reference.position_gain_scale
                    if self.payload_prelift_reference is not None
                    else np.ones(3)
                ),
                "independent_velocity_gain_scale": (
                    self.payload_prelift_reference.velocity_gain_scale
                    if self.payload_prelift_reference is not None
                    else np.ones(3)
                ),
                "max_tilt_rad": (
                    self.payload_prelift_reference.max_tilt_rad
                    if self.payload_prelift_reference is not None
                    else vehicle.controller.config.max_tilt_rad
                ),
            })
            target.update(self.payload_prelift_integral_parameters)
            overrides[id(vehicle)] = target
        return overrides

    def _update_payload_link_calibration(self, now, measured, payload, reference, takeup_duration, states):
        """Use fresh, steady endpoint geometry as a bounded tautness proxy."""
        calibrator = self.payload_link_calibrator
        if not self.payload_link_calibration_enabled or self.payload_link_calibration_applied:
            return False
        if (self.payload_transport_active or self.payload_controlled_exit_reason
                or reference.get("failed", False) or reference.get("stage") not in (
                    "takeup", "rising", "braking", "holding", "lead_hold")
                or now - self.payload_takeup_start_time < takeup_duration
                or not self._payload_ready(now)):
            self._reset_payload_link_calibration_window("phase_or_payload_not_ready")
            return False
        relative_velocities = []
        payload_age = (float(now)-float(payload.get("received_time", math.nan))
                       if payload is not None else math.nan)
        if (payload is None or not payload.get("valid", False) or not math.isfinite(payload_age)
                or payload_age > self.vehicles[0].state_timeout or payload_age < -.02):
            self._reset_payload_link_calibration_window("payload_snapshot_not_ready")
            return False
        for index, (vehicle, state) in enumerate(zip(self.vehicles, states)):
            state_age, sample_age = vehicle._mocap_state_ages(state, now)
            if (not state.get("valid", False) or not math.isfinite(state_age)
                    or state_age > vehicle.state_timeout or not math.isfinite(sample_age)
                    or sample_age > vehicle.state_timeout
                    or sample_age < -vehicle.mocap_timestamp_future_tolerance_s):
                self._reset_payload_link_calibration_window("vehicle_state_not_ready")
                return False
            control = vehicle._build_ekf_control_state(state, now)
            if not control.get("ekf_state_valid", False):
                self._reset_payload_link_calibration_window("vehicle_kinematics_not_ready")
                return False
            rho = payload["rotation"] @ self.slung_controller.config.attachment_points_m[:, index]
            attachment_velocity = (payload["velocity"]
                + np.cross(payload["rotation"] @ payload.get("body_rate", np.zeros(3)), rho))
            relative_velocities.append(attachment_velocity-control["control_velocity"])
        minimum_distance = min(float(np.linalg.norm(measured[a]-measured[b]))
                               for a,b in ((0,1),(0,2),(1,2)))
        eligible = (minimum_distance >= self.payload_transport_guard.min_distance_m
                    and np.max(np.linalg.norm(relative_velocities,axis=1))
                    <= self.payload_link_calibration_max_relative_speed_mps)
        if not eligible:
            self._reset_payload_link_calibration_window("motion_or_spacing_not_ready")
            return False
        distances = cable_distances(payload["position"], payload["rotation"],
                                   self.slung_controller.config.attachment_points_m, measured)
        self.payload_link_measured_distances = distances.copy()
        proposal = calibrator.update(payload.get("sample_time", payload["received_time"]), distances,
                                     eligible=eligible,
                                     mismatch_tolerance_m=self.payload_takeup_distance_tolerance_m)
        if proposal is not None:
            self.payload_link_calibration_pending = proposal.copy()
        proposal = self.payload_link_calibration_pending
        if proposal is None:
            if calibrator.status in ("out_of_bounds", "unstable"):
                rospy.logwarn_throttle(2., "有效绳长校准暂不采用：%s，距离波动=%s。",
                                       calibrator.reason, calibrator.spread_m.tolist())
            return False
        target_distances = takeup_target_distance(proposal, self.payload_takeup_pre_tension_slack_m)
        # A robust window may end in an outlier. Wait for an instantaneous
        # consistent frame before applying; never bypass the original gate.
        if not eligible or not takeup_distance_ready(distances, proposal, proposal,
                                                     self.payload_takeup_distance_tolerance_m):
            return False
        cfg = self.slung_controller.config
        try:
            links = self.slung_controller.equilibrium_link_units(payload["rotation"])
            new_targets = takeup_vehicle_targets(
                payload["position"], payload["rotation"], cfg.attachment_points_m,
                target_distances, self.payload_takeup_outward_offset_m, link_units=links).T
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            self._reset_payload_link_calibration_window("geometry_update_failed")
            rospy.logwarn_throttle(2., "有效绳长几何更新未采用：%s", error)
            return False
        cfg.link_lengths_m = proposal.copy()
        cfg.link_length_m = float(np.mean(proposal))
        self.payload_takeup_target_distance_m = np.asarray(target_distances).copy()
        self.payload_takeup_target_positions = new_targets
        if self.payload_prelift_reference is None:
            # The nominal TAKEUP has finished, so this reference has zero
            # derivatives. Hold it until the existing taut-confirmation then
            # align smoothly to measured anchors via SlungPreliftTrajectory.
            self.payload_link_calibration_hold_reference = reference["positions"].copy()
        self.payload_geometry_ready_since = None
        self.payload_takeup_ready_since = self.payload_prelift_ready_since = None
        self.payload_link_calibration_applied = True
        self.payload_link_calibration_pending = None
        rospy.loginfo("有效绳长校准完成：名义=%s，运行有效=%s，稳定窗口 %.3f s/%d 样本；YAML 未改写。",
                      self.payload_nominal_link_lengths.tolist(), proposal.tolist(),
                      calibrator.window_elapsed_s, calibrator.sample_count)
        return True

    def _reset_payload_link_calibration_window(self, reason):
        calibrator = self.payload_link_calibrator
        if self.payload_link_calibration_pending is not None and not self.payload_link_calibration_applied:
            # The module's accepted proposal has not reached the model yet.
            # Invalid input requires a new full window, not a stale proposal.
            settings = {name: getattr(calibrator, name) for name in (
                "window_s", "max_adjustment_m", "max_spread_m", "max_drift_mps",
                "max_sample_gap_s", "min_samples", "trim_fraction")}
            self.payload_link_calibrator = SlungLinkCalibrator(self.payload_nominal_link_lengths, **settings)
            self.payload_link_calibration_pending = None
        self.payload_link_calibrator.reset_window(reason)

    def _hold_payload_mission_clock(self, now):
        """Defer the task's hover/path time until prelift and reference lift finish."""
        if self.transport_mode != "slung_load" or self.payload_mission_hold_time is None:
            return
        release = (self.payload_transport_active and now >= self.payload_transport_start_time
                   + self.payload_tension_ramp_s + self.payload_reference_lift_s)
        dt = max(0.0, float(now) - self.payload_mission_hold_time)
        for vehicle in self.vehicles:
            trajectory = vehicle.trajectory
            if trajectory is None or trajectory.phase not in ("height_correction", "hover"):
                continue
            if release and trajectory.phase == "height_correction":
                duration = (trajectory.config.takeup_duration_s
                            or trajectory.config.takeoff_settle_s)
                trajectory.phase_start_time = now - duration
            else:
                trajectory.phase_start_time += dt
        self.payload_mission_hold_time = None if release else float(now)

    def _log_payload_takeup_status(self, now, reason, distances=None):
        """Explain why TAKEUP is not yet allowed to enter TAUT_RAMP."""
        interval = float(self.payload_takeup_diagnostic_interval_s)
        if (self.payload_takeup_diagnostic_last_time is not None and
                float(now) - self.payload_takeup_diagnostic_last_time < interval):
            return
        self.payload_takeup_diagnostic_last_time = float(now)

        with self.payload_lock:
            payload_state = None if self.latest_payload_state is None else dict(
                self.latest_payload_state
            )
        if payload_state is None:
            state_text = "负载状态=无"
        else:
            try:
                age = float(now) - float(payload_state["received_time"])
            except (KeyError, TypeError, ValueError):
                age = math.nan
            state_text = (
                "负载状态 valid=%d derivatives=%d age=%.3f s" % (
                    int(bool(payload_state.get("valid", False))),
                    int(bool(payload_state.get("derivatives_valid", False))),
                    age,
                )
            )

        length_text = "绳长不可计算"
        if distances is not None and self.slung_controller is not None:
            distances = np.asarray(distances, dtype=float).reshape(3)
            lengths = np.asarray(
                self.slung_controller.config.link_lengths_m, dtype=float
            ).reshape(3)
            targets = link_lengths_vector(
                self.payload_takeup_target_distance_m,
                "target_distance_m",
            )
            details = []
            for vehicle, distance, length, target in zip(
                    self.vehicles, distances, lengths, targets):
                error = float(distance - target)
                within = (
                    abs(error) <= self.payload_takeup_distance_tolerance_m and
                    distance <= length + self.payload_takeup_distance_tolerance_m
                )
                details.append(
                    "CF%d %.3f/目标%.3f m(配置%.3f,误差%+.3f,%s)" % (
                        vehicle.vehicle_id,
                        float(distance),
                        float(target),
                        float(length),
                        error,
                        "满足" if within else "不满足",
                    )
                )
            length_text = "；".join(details)

        rospy.logwarn(
            "TAKEUP 尚未进入 TAUT_RAMP：%s；%s；%s",
            reason,
            state_text,
            length_text,
        )

    def _payload_takeup_ready(self, now):
        """Confirm current cable geometry, held clearance and settled load velocity."""
        if self.transport_mode != "slung_load" or self.slung_controller is None:
            return True
        self.payload_prelift_ready = False
        if not self._payload_ready(now):
            self.payload_prelift_ready_since = self.payload_takeup_ready_since = None
            self._log_payload_takeup_status(now, "等待新鲜有效负载状态")
            return False
        with self.payload_lock:
            payload = dict(self.latest_payload_state)
        positions = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None:
                self.payload_prelift_ready_since = self.payload_takeup_ready_since = None
                return False
            positions.append(np.asarray(state["position"], dtype=float))
        distances = cable_distances(
            payload["position"], payload["rotation"],
            self.slung_controller.config.attachment_points_m, positions,
        )
        self.payload_link_measured_distances = distances.copy()
        ready = takeup_distance_ready(
            distances, self.slung_controller.config.link_lengths_m,
            self.payload_takeup_target_distance_m, self.payload_takeup_distance_tolerance_m,
        )
        clearance = payload_ground_clearance(
            payload["position"], payload["rotation"], self.slung_controller.config.payload_size_m,
        )
        self.payload_ground_clearance_m = clearance
        settled = (math.isfinite(float(payload["velocity"][2]))
                   and abs(float(payload["velocity"][2]))
                   <= self.payload_prelift_vertical_velocity_tolerance_mps)
        holding = (self.payload_prelift_reference is not None
                   and self.payload_prelift_reference.stage == "holding")
        if not (ready and holding and clearance >= self.payload_prelift_clearance_m and settled):
            self.payload_prelift_ready_since = self.payload_takeup_ready_since = None
            stage = (self.payload_prelift_reference.stage
                     if self.payload_prelift_reference is not None else "takeup")
            self._log_payload_takeup_status(
                now, "阶段=%s，间隙 %.3f/%.3f m，vz=%.3f m/s，当前绳长有效=%d" % (
                    stage, clearance, self.payload_prelift_clearance_m,
                    payload["velocity"][2], int(ready),
                ), distances,
            )
            return False
        if self.payload_prelift_ready_since is None:
            self.payload_prelift_ready_since = float(now)
        self.payload_prelift_ready = now - self.payload_prelift_ready_since >= self.payload_prelift_confirm_s
        if not self.payload_prelift_ready:
            return False
        if self.payload_takeup_ready_since is None:
            self.payload_takeup_ready_since = float(now)
        return now - self.payload_takeup_ready_since >= self.payload_takeup_confirm_s

    def _payload_reference_profile(self, now, desired_position):
        """Build MATLAB-style frozen-ground, tension-ramp, then lift reference."""
        if self.payload_transport_start_position is None:
            with self.payload_lock:
                state = self.latest_payload_state
            if state is None:
                return None
            self.payload_transport_start_position = np.asarray(
                state["position"], dtype=float
            ).copy()
        elapsed = max(
            0.0,
            float(now) - float(self.payload_transport_start_time),
        )
        ramp_s = float(self.payload_tension_ramp_s)
        lift_s = float(self.payload_reference_lift_s)
        if elapsed < ramp_s:
            profile = smoothstep5_profile(elapsed, ramp_s)
            return (
                self.payload_transport_start_position.copy(),
                np.zeros(3),
                np.zeros(3),
                float(profile[0]),
                float(profile[1]),
            )
        profile = smoothstep5_profile(elapsed - ramp_s, lift_s)
        displacement = np.asarray(desired_position, dtype=float) - self.payload_transport_start_position
        return (
            self.payload_transport_start_position + profile[0] * displacement,
            profile[1] * displacement,
            profile[2] * displacement,
            1.0,
            0.0,
        )

    @staticmethod
    def _finite_supported_command(command):
        if not command:
            return False
        try:
            force = np.asarray(command["desired_force"], dtype=float)
            rates = np.asarray(command.get("body_rate_command", np.zeros(3)), dtype=float)
            thrust = float(command["collective_thrust"])
            return (force.shape == (3,) and rates.shape == (3,)
                    and np.all(np.isfinite(force)) and np.all(np.isfinite(rates))
                    and math.isfinite(thrust) and thrust > 0. and force[2] > 0.)
        except (KeyError, TypeError, ValueError):
            return False

    @staticmethod
    def _finite_transport_override(target):
        vectors = ("position", "velocity", "acceleration", "position_hold_position",
                   "position_hold_velocity", "position_hold_acceleration",
                   "desired_force_override", "desired_force_dot_override",
                   "kinematic_specific_acceleration")
        scalars = ("transport_blend", "transport_blend_dot", "max_tilt_rad",
                   "transport_position_hold_gain_scale")
        try:
            return (target.get("transport_mode", False)
                    and all(np.asarray(target[key]).shape == (3,)
                            and np.all(np.isfinite(target[key])) for key in vectors)
                    and all(math.isfinite(float(target[key])) for key in scalars)
                    and all(np.all(np.isfinite(value)) for value in target.values()
                            if isinstance(value, np.ndarray)))
        except (KeyError, TypeError, ValueError):
            return False

    def _payload_planning_hold(self, now):
        """Freeze a complete supported reference for the bounded retry window."""
        overrides = {}
        for index, vehicle in enumerate(self.vehicles):
            previous = self.last_transport_overrides.get(id(vehicle), {})
            if self._finite_transport_override(previous):
                target = dict(previous)
            else:
                command = getattr(vehicle, "last_control_command", None)
                references = self.payload_transport_vehicle_start_positions
                if (not self._finite_supported_command(command) or references is None
                        or not np.all(np.isfinite(references[index]))):
                    return {}
                # A last valid actuator command is the support source when a
                # complete cooperative target has not yet been cached.
                target = dict(transport_mode=True, independent_mode=False,
                    transport_blend=1., position=references[index].copy(),
                    position_hold_position=references[index].copy(),
                    desired_force_override=command["desired_force"].copy(),
                    transport_position_hold_gain_scale=0.,
                    max_tilt_rad=vehicle.controller.config.max_tilt_rad,
                    kinematic_specific_acceleration=np.zeros(3))
            for key in ("velocity", "acceleration", "position_hold_velocity",
                        "position_hold_acceleration", "desired_force_dot_override",
                        "transport_hold_integral_force_dot"):
                target[key] = np.zeros(3)
            target.update(transport_blend_dot=0., transport_position_hold_gain_scale_dot=0.,
                transport_hold_integral_learning=False,
                transport_hold_integral_freeze_reason="landing_planner_retry",
                transport_full_pid_learning=False,
                transport_full_pid_freeze_reason="landing_planner_retry",
                transport_full_pid_tick=float(now), transport_full_pid_force_hold=True,
                transport_full_pid_weight_dot=0., transport_filter_reset=False)
            if not self._finite_transport_override(target):
                return {}
            overrides[id(vehicle)] = target
        self._refresh_cached_attitude_trim(now, overrides, "landing_planner_retry", freeze_base_derivatives=True)
        return overrides

    def _exit_payload_planning_fault(self, now, reason, source="landing_planner"):
        """Finish a failed retry window using the existing supported descent."""
        reason = source + ": " + str(reason)
        self.payload_controlled_exit_reason = "payload_load: " + reason
        if not all(self._finite_supported_command(getattr(v, "last_control_command", None))
                   for v in self.vehicles):
            self._set_abort(reason + "; no finite supported fleet command")
            return
        # Disable the cooperative planner before selecting the independent
        # emergency path, so recovery cannot immediately invoke it again.
        self.payload_transport_active = False
        self._begin_payload_emergency_landing(now, reason)
        self._reset_payload_handoff()
        self.payload_validation_landing_stage = "planning_fault_descent"

    def _payload_landing_planning_failed(self, now, error, payload_state):
        if self.payload_landing_retry_tick != float(now):
            self.payload_landing_retry_tick = float(now)
            self.payload_landing_failure_count += 1
            self.payload_landing_retry_count += 1
            if self.payload_landing_retry_since is None:
                self.payload_landing_retry_since = float(now)
            self.payload_landing_failure_reason = "%s: %s" % (type(error).__name__, error)
            if self.payload_landing_failure_snapshot is None:
                self.payload_landing_failure_snapshot = {
                    "start_position": np.asarray(payload_state["position"]).tolist(),
                    "start_velocity": np.asarray(payload_state.get("velocity", np.zeros(3))).tolist(),
                    "start_acceleration": np.asarray(payload_state.get("acceleration", np.zeros(3))).tolist(),
                    "ground_z": (self.payload_ground_center_z if math.isfinite(self.payload_ground_center_z)
                                 else .5 * float(self.payload_height_m)),
                    "now": float(now),
                    "min_duration_s": float(self.vehicles[0].trajectory_config.landing_duration_s),
                    "max_speed_mps": self.payload_landing_max_speed_mps,
                    "max_accel_mps2": self.payload_landing_max_acceleration_mps2,
                    "reason": self.payload_landing_failure_reason,
                }
                rospy.logerr("负载下降规划失败，暂时保留协同支撑并有界重试：%s", self.payload_landing_failure_snapshot)
        self.payload_landing_profile = None
        self.payload_landing_start_time = None
        held = self._payload_planning_hold(now)
        if (self.payload_landing_retry_count >= 3
                or now - self.payload_landing_retry_since >= .3 or not held):
            self._exit_payload_planning_fault(now, self.payload_landing_failure_reason)
            return {}
        return held

    def _transport_overrides(self, now):
        if self.transport_mode != "slung_load" or not self.slung_controller:
            return {}
        if not self._payload_ready(now):
            return {}
        with self.payload_lock:
            payload_state = dict(self.latest_payload_state)
        targets = []
        vehicle_states = []
        control_states = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._state_ready(now):
                return {}
            target = vehicle._target_for(state, now)
            flight_phase = target.get("flight_phase")
            if flight_phase not in (
                    "height_correction", "hover", "figure_eight", "landing",
                    "landing_settle"):
                return {}
            control_state = vehicle._build_ekf_control_state(state, now)
            control_states.append(control_state)
            vehicle_states.append({
                "position": state["position"],
                "velocity": control_state.get("control_velocity", np.zeros(3)),
                "acceleration": control_state.get("control_acceleration", np.zeros(3)),
                "body_rate": state.get("body_rate", np.zeros(3)),
                "mass": vehicle.controller.config.mass,
            })
            targets.append(target)

        target_velocity = np.mean([item["velocity"] for item in targets], axis=0)
        target_yaw = math.atan2(float(target_velocity[1]), float(target_velocity[0])) \
            if np.linalg.norm(target_velocity[:2]) > 1.0e-9 else 0.0
        target_rotation = np.array([
            [math.cos(target_yaw), -math.sin(target_yaw), 0.0],
            [math.sin(target_yaw), math.cos(target_yaw), 0.0],
            [0.0, 0.0, 1.0],
        ])
        landing = any(target.get("flight_phase") in ("landing", "landing_settle")
                      for target in targets)
        if landing:
            if self.payload_landing_retry_tick == float(now):
                return self._payload_planning_hold(now)
            # A normal trajectory can enter landing before the validation
            # deadline.  Validation then clears the start timestamp while an
            # old profile may still exist; rebuild both pieces together so a
            # landing tick can never subtract ``None`` or reuse a pre-deadline
            # reference.
            if (self.payload_landing_profile is None
                    or self.payload_landing_start_time is None):
                # The landing profile must start at the current measured load
                # state.  Reusing payload_last_reference can point above the
                # load after the validation timer has elapsed.
                initial = (
                    payload_state["position"],
                    payload_state.get("velocity", np.zeros(3)),
                    payload_state.get("acceleration", np.zeros(3)),
                    1., 0.,
                )
                ground_z = (self.payload_ground_center_z if math.isfinite(self.payload_ground_center_z)
                            else .5 * float(self.payload_height_m))
                try:
                    profile = SlungLandingProfile(
                        initial[0], initial[1], initial[2], ground_z, now,
                        float(self.vehicles[0].trajectory_config.landing_duration_s),
                        self.payload_landing_max_speed_mps, self.payload_landing_max_acceleration_mps2)
                except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                    return self._payload_landing_planning_failed(now, error, payload_state)
                previous = next(iter(self.last_transport_overrides.values()), {})
                gains = (
                    float(previous.get("transport_position_hold_gain_scale", self.payload_ramp_position_hold_gain_scale)),
                    float(previous.get("effective_transport_link_gain_scale", self.payload_transport_link_gain_scale)),
                    float(previous.get("transport_max_tilt_rad", self.vehicles[0].controller.config.max_tilt_rad)),
                    float(previous.get("transport_blend", initial[3])),
                )
                detector = PayloadTouchdownDetector(
                    ground_clearance_m=self.payload_ground_clearance_reference_m,
                    **self.payload_touchdown_parameters)
                attachment_points = self.slung_controller.config.attachment_points_m
                measured_positions = np.asarray(
                    [item["position"] for item in vehicle_states], dtype=float
                )
                measured_attachments = (
                    np.asarray(payload_state["position"], dtype=float)[:, None]
                    + np.asarray(payload_state["rotation"], dtype=float)
                    @ attachment_points
                )
                relative = measured_attachments - measured_positions.T
                measured_distances = np.linalg.norm(relative, axis=0)
                directions = distances = None
                if (np.all(np.isfinite(relative)) and
                        np.all(measured_distances > 1.0e-6)):
                    directions = (
                        relative / measured_distances[None, :]
                    )
                    # Preserve a measured overrun rather than pulling an
                    # aircraft inward to the shorter nominal length.
                    distances = np.maximum(
                        measured_distances,
                        np.asarray(self.slung_controller.config.link_lengths_m, dtype=float),
                    )
                self.payload_landing_profile = profile
                self.payload_landing_start_time = float(now)
                self.payload_landing_start_position = np.asarray(initial[0]).copy()
                self.payload_landing_gain_snapshot = gains
                self.payload_touchdown_detector = detector
                self.payload_landing_rope_directions = directions
                self.payload_landing_rope_distances = distances
                rospy.loginfo("负载受控下降参考 %.2f s，速度上限 %.3f m/s，加速度上限 %.3f m/s²。",
                              self.payload_landing_profile.duration, self.payload_landing_max_speed_mps,
                              self.payload_landing_max_acceleration_mps2)
            blend_profile = smoothstep5_profile(now - self.payload_landing_start_time,
                                               self.payload_landing_gain_transition_s)
            initial_blend = self.payload_landing_gain_snapshot[3]
            try:
                payload_reference = (*self.payload_landing_profile.evaluate(now),
                                     initial_blend + (1-initial_blend)*blend_profile[0],
                                     (1-initial_blend)*blend_profile[1])
                if not all(np.all(np.isfinite(value)) for value in payload_reference):
                    raise ValueError("landing evaluation returned nonfinite references")
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                return self._payload_landing_planning_failed(now, error, payload_state)
            self.payload_landing_retry_since = None
            self.payload_landing_retry_count = 0
        else:
            desired_payload_position = np.mean(
                [vehicle.trajectory.takeoff_position for vehicle in self.vehicles],
                axis=0,
            )
            desired_payload_position[2] -= float(np.mean(
                self.slung_controller.config.link_lengths_m
            ))
            if math.isfinite(self.payload_target_height_m):
                desired_payload_position[2] = self.payload_target_height_m
            payload_reference = self._payload_reference_profile(
                now, desired_payload_position
            )
        if payload_reference is None:
            return {}
        (
            payload_target_position,
            payload_target_velocity,
            payload_target_acceleration,
            transport_blend,
            transport_blend_dot,
        ) = payload_reference
        self.payload_last_reference = tuple(np.asarray(v).copy() if i < 3 else float(v)
                                            for i, v in enumerate(payload_reference))
        if self.payload_transport_vehicle_start_positions is None:
            # TAKEUP has already produced the exact per-vehicle handoff
            # anchors.  Keep those anchors and translate them with the load's
            # quintic reference instead of returning to the aircraft-only
            # trajectory when TAUT_RAMP reaches blend=1.
            if self.payload_takeup_target_positions is None:
                return {}
            self.payload_transport_vehicle_start_positions = (
                self.payload_takeup_target_positions.copy()
            )
        vehicle_reference_positions, vehicle_reference_velocities, vehicle_reference_accelerations = (
            translate_vehicle_references(
                self.payload_transport_vehicle_start_positions,
                self.payload_transport_start_position,
                payload_target_position,
                payload_target_velocity,
                payload_target_acceleration,
            )
        )
        physical_positions = vehicle_reference_positions.copy()
        hold_velocities = vehicle_reference_velocities.copy()
        hold_accelerations = vehicle_reference_accelerations.copy()
        attitude_reference = None
        attitude_progress = math.nan
        if not landing and self.payload_attitude_reference_enabled:
            if self.payload_transport_start_rotation is None:
                self.payload_transport_start_rotation = project_so3(payload_state["rotation"])
            if self.payload_transport_goal_rotation is None:
                # Freeze the hover heading as well as the measured start pose:
                # changing either endpoint mid-profile invalidates its rates.
                self.payload_transport_goal_rotation = target_rotation.copy()
            attitude_profile = smoothstep5_profile(
                max(0.0, now - self.payload_transport_start_time),
                self.payload_attitude_reference_s,
            )
            # Polynomial cancellation can put a quintic endpoint a few ULPs
            # outside [0,1]; never let roundoff terminate the control timer.
            attitude_profile[0] = np.clip(attitude_profile[0], 0.0, 1.0)
            attitude_progress = float(attitude_profile[0])
            attitude_reference = attachment_rotation_reference(
                self.payload_transport_start_rotation,
                self.payload_transport_goal_rotation,
                self.slung_controller.config.attachment_points_m,
                *attitude_profile,
            )
            if self.payload_equilibrium_reference_enabled:
                if self.payload_transport_initial_link_units is None:
                    initial_cables = (
                        self.payload_transport_start_position[:, None]
                        + self.payload_transport_start_rotation
                        @ self.slung_controller.config.attachment_points_m
                        - self.payload_transport_vehicle_start_positions.T
                    )
                    distances = np.linalg.norm(initial_cables, axis=0)
                    if np.any(distances <= 1.0e-6):
                        return {}
                    self.payload_transport_initial_link_distances = distances
                    self.payload_transport_initial_link_units = initial_cables / distances[None, :]
                    self.payload_transport_goal_link_units = self.slung_controller.equilibrium_link_units(
                        self.payload_transport_goal_rotation)
                link_reference = cable_direction_reference(
                    self.payload_transport_initial_link_units,
                    self.payload_transport_goal_link_units,
                    *attitude_profile,
                )
                distances = self.payload_transport_initial_link_distances[None, :]
                # A level payload alone does not make a consistent hover:
                # its cable directions must also agree with static allocation.
                # Preserve measured length throughout the direction transition.
                attitude_reference["position_offsets"] -= distances * (
                    link_reference["directions"] - self.payload_transport_initial_link_units)
                attitude_reference["velocity_offsets"] -= distances * link_reference["direction_rates"]
                attitude_reference["acceleration_offsets"] -= distances * link_reference["direction_accelerations"]
            target_rotation = attitude_reference["rotation"]
            physical_positions += attitude_reference["position_offsets"].T
            vehicle_reference_positions = physical_positions.copy()
            vehicle_reference_velocities += attitude_reference["velocity_offsets"].T
            vehicle_reference_accelerations += attitude_reference["acceleration_offsets"].T
            hold_velocities = vehicle_reference_velocities.copy()
            hold_accelerations = vehicle_reference_accelerations.copy()
        transport_hold_scale = self.payload_transport_position_hold_gain_scale
        transport_hold_scale_dot = 0.0
        if not landing and self.payload_handoff_reference_positions is not None:
            lift_profile = smoothstep5_profile(
                max(0.0, now - self.payload_transport_start_time - self.payload_tension_ramp_s),
                self.payload_reference_lift_s,
            )
            lift_progress = float(lift_profile[0])
            # Keep the measured anchors through reference lift, then fade to
            # the configured steady transport hold without a force step.
            hold_delta = self.payload_ramp_position_hold_gain_scale - transport_hold_scale
            transport_hold_scale += (1.0 - lift_progress) * hold_delta
            transport_hold_scale_dot = -float(lift_profile[1]) * hold_delta
        if self.payload_handoff_reference_positions is not None:
            blend_ddot = (float(smoothstep5_profile(
                float(now) - self.payload_transport_start_time, self.payload_tension_ramp_s
            )[2]) if transport_blend < 1.0 else 0.0)
            held_positions = (self.payload_handoff_reference_positions
                              + (payload_target_position - self.payload_transport_start_position))
            if attitude_reference is not None:
                # Rotate both sides of the old PID bias blend together. Its
                # difference then remains constant, so handoff_references'
                # analytic first/second derivatives stay valid.
                held_positions = held_positions + attitude_reference["position_offsets"].T
            vehicle_reference_positions, bias_velocities, bias_accelerations = handoff_references(
                physical_positions,
                held_positions,
                transport_blend, transport_blend_dot, blend_ddot,
            )
            vehicle_reference_velocities += bias_velocities
            vehicle_reference_accelerations += bias_accelerations
        link_gain_scale = transport_blend * self.payload_transport_link_gain_scale
        if (not landing and transport_blend >= 1.0
                and self.payload_transport_steady_link_gain_scale < 1.0):
            steady_profile = smoothstep5_profile(
                max(0.0, now - self.payload_transport_start_time - self.payload_tension_ramp_s),
                self.payload_reference_lift_s,
            )
            link_gain_scale *= (
                1.0 - (1.0 - self.payload_transport_steady_link_gain_scale)
                * float(steady_profile[0])
            )
        if landing:
            progress, progress_dot, _ = smoothstep5_profile(
                now - self.payload_landing_start_time, self.payload_landing_gain_transition_s)
            initial_hold, initial_link, _, _ = self.payload_landing_gain_snapshot
            # Keep horizontal support at least as strong through touchdown;
            # transition any necessary increase continuously from its value.
            final_hold = max(initial_hold, self.payload_ramp_position_hold_gain_scale)
            transport_hold_scale = initial_hold + progress * (final_hold - initial_hold)
            transport_hold_scale_dot = progress_dot * (final_hold - initial_hold)
            link_gain_scale = initial_link
            if (self.payload_landing_rope_directions is not None
                    and self.payload_landing_rope_distances is not None):
                vehicle_reference_positions, vehicle_reference_velocities, vehicle_reference_accelerations = (
                    rope_preserving_vehicle_references(
                        payload_target_position,
                        payload_target_velocity,
                        payload_target_acceleration,
                        target_rotation,
                        self.slung_controller.config.attachment_points_m,
                        self.payload_landing_rope_directions,
                        self.payload_landing_rope_distances,
                    )
                )
                physical_positions = vehicle_reference_positions.copy()
                hold_velocities = vehicle_reference_velocities.copy()
                hold_accelerations = vehicle_reference_accelerations.copy()
        # Apply the mode after every phase's gain selection, including an
        # existing landing snapshot. The core bypass and logged gain agree.
        if self.slung_controller.config.transport_force_mode == "reference_cascade":
            link_gain_scale = 0.0
        target = {
            "position": payload_target_position,
            "velocity": payload_target_velocity,
            "acceleration": payload_target_acceleration,
            "rotation": target_rotation,
            "body_rate": (np.zeros(3) if attitude_reference is None
                          else attitude_reference["omega_body"]),
            "body_rate_dot": (np.zeros(3) if attitude_reference is None
                              else attitude_reference["alpha_body"]),
            "payload_attitude_gain": self.payload_attitude_gain,
            "payload_rate_gain": self.payload_rate_gain,
            "payload_yaw_enabled": self.payload_yaw_enabled,
            "link_gain_scale": link_gain_scale,
            # During TENSION_RAMP the target position is the measured TAKEUP
            # anchor.  Keep an independent hold term while the load force is
            # blended in, as in Udaan's ramp_position_hold_commands().
            "ramp_position_hold_gain_scale": self.payload_ramp_position_hold_gain_scale,
            "transport_position_hold_gain_scale": (
                transport_hold_scale
            ),
            "transport_blend_dot": transport_blend_dot,
        }
        payload_control_dt = getattr(self, "payload_control_dt", now - self.last_tick)
        result = self.slung_controller.compute(
            payload_state, vehicle_states, target,
            payload_control_dt,
        )
        if result is None:
            return {}
        trim_dot = np.zeros(3)
        if not self.payload_support_trim.initialized and self.payload_handoff_forces is not None:
            self.payload_support_trim.initialize(
                self.payload_handoff_forces[:, 2],
                [output["force"][2] for output in result["vehicles"]],
                available_z=[v.controller.config.max_command_thrust for v in self.vehicles],
            )
        if self.payload_support_trim.initialized:
            saturated = [
                bool(getattr(vehicle, "last_control_command", None) is not None and
                     vehicle.last_control_command["collective_thrust"] >=
                     vehicle.controller.config.max_command_thrust - 0.01)
                for vehicle in self.vehicles
            ]
            trim, trim_dot = self.payload_support_trim.update(
                float(payload_target_position[2] - payload_state["position"][2]),
                getattr(self, "payload_control_dt", 1.0 / self.rate_hz), saturated,
            )
        else:
            trim = np.zeros(3)
        if landing and self.payload_unload_start_time is not None:
            unload = smoothstep5_profile(now - self.payload_unload_start_time, self.payload_landing_unload_s)
            trim = (1 - unload[0]) * self.payload_landing_trim_at_unload
            trim_dot = -unload[1] * self.payload_landing_trim_at_unload
        overrides = {}
        for index, vehicle in enumerate(self.vehicles):
            output = result["vehicles"][index]
            vehicle_cfg = vehicle.vehicle_config
            prelift_tilt = self.payload_prelift_parameters.get(
                "max_tilt_rad", vehicle.controller.config.max_tilt_rad)
            tilt_progress = float(smoothstep5_profile(
                max(0.0, now - self.payload_transport_start_time - self.payload_tension_ramp_s),
                self.payload_reference_lift_s,
            )[0])
            transport_tilt = (max(vehicle.controller.config.max_tilt_rad, self.payload_landing_gain_snapshot[2]) if landing else
                              (1.0 - tilt_progress) * max(prelift_tilt, vehicle.controller.config.max_tilt_rad)
                              + tilt_progress * vehicle.controller.config.max_tilt_rad)
            force = output["force"].copy()
            force[2] += trim[index]
            force_dot = output["force_dot"].copy()
            force_dot[2] += trim_dot[index]
            overrides[id(vehicle)] = {
                "transport_mode": True,
                "independent_mode": False,
                "transport_blend": transport_blend,
                "transport_blend_dot": transport_blend_dot,
                "transport_filter_reset": bool(transport_blend <= 1.0e-9),
                "position": vehicle_reference_positions[index].copy(),
                "velocity": vehicle_reference_velocities[index].copy(),
                "acceleration": vehicle_reference_accelerations[index].copy(),
                "position_hold_position": physical_positions[index].copy(),
                "position_hold_velocity": hold_velocities[index].copy(),
                "position_hold_acceleration": hold_accelerations[index].copy(),
                "payload_target_rotation": target_rotation.copy(),
                "payload_target_body_rate": target["body_rate"].copy(),
                "payload_target_body_rate_dot": target["body_rate_dot"].copy(),
                "payload_attitude_reference_progress": attitude_progress,
                "payload_transport_force_mode": result["force_mode"],
                "kinematic_specific_acceleration": output["kinematic_specific_acceleration"].copy(),
                "transport_position_hold_gain_scale_dot": transport_hold_scale_dot,
                "desired_force_override": force,
                "desired_force_dot_override": force_dot,
                "transport_support_trim_n": trim[index],
                "max_tilt_rad": transport_tilt,
                "transport_max_tilt_rad": transport_tilt,
                "transport_attitude_gain": vehicle.transport_attitude_gain.copy(),
                "transport_rate_gain": vehicle.transport_rate_gain.copy(),
                "payload_position_error": result["payload_position_error"],
                "payload_velocity_error": result["payload_velocity_error"],
                "payload_attitude_error": result["payload_attitude_error"],
                "payload_state_valid": True,
                "payload_target_position": payload_target_position,
                "link_direction": output["link_direction"],
                "desired_link_direction": output["desired_link_direction"],
                "raw_desired_link_direction": output["raw_desired_link_direction"],
                "link_direction_error": output["link_direction_error"],
                "desired_tension": output["desired_tension"],
                "desired_tension_signed": output["desired_tension_signed"],
                "payload_yaw_enabled": self.payload_yaw_enabled,
                "payload_desired_force": result["desired_force"],
                "payload_desired_moment": result["desired_moment"],
                "transport_link_gain_scale": self.payload_transport_link_gain_scale,
                "effective_transport_link_gain_scale": link_gain_scale,
                "payload_target_velocity": payload_target_velocity.copy(),
                "payload_target_acceleration": payload_target_acceleration.copy(),
                "transport_support_requested_trim_n": self.payload_support_trim.requested_bias[index],
                "transport_support_trim_limit_n": self.payload_support_trim.limits[index],
                "transport_support_initialization_clipped": bool(self.payload_support_trim.initialization_clipped[index]),
                "raw_link_force": output["raw_link_force"].copy(),
                "link_force": output["link_force"].copy(),
                "link_force_rate_limited": bool(output["link_force_rate_limited"]),
                "link_force_magnitude_limited": bool(output["link_force_magnitude_limited"]),
                "kinematic_perpendicular_force": output["kinematic_perpendicular_force"].copy(),
                "payload_control_dt_s": float(payload_control_dt),
                "ramp_position_hold_gain_scale": (
                    self.payload_ramp_position_hold_gain_scale
                ),
                "transport_position_hold_gain_scale": (
                    transport_hold_scale
                ),
                "transport_feedback_controller": self.payload_transport_feedback_controller,
                "transport_full_pid_integral_gate_m": self.payload_transport_pid_integral_gate_m,
                "transport_full_pid_integral_velocity_weight": self.payload_transport_pid_integral_velocity_weight,
                "transport_full_pid_integral_limit": (
                    np.asarray(vehicle.controller.config.independent_integral_limit, dtype=float)
                    * self.payload_transport_pid_integral_limit_scale
                ),
                "transport_full_pid_integral_gain_scale": self.payload_transport_pid_integral_gain_scale,
                "transport_full_pid_force_limit_n": self.payload_transport_pid_force_limit_n,
                "transport_full_pid_force_rate_limit_nps": self.payload_transport_pid_force_rate_limit_nps,
                "payload_landing_rope_preserving": bool(
                    landing and self.payload_landing_rope_directions is not None
                ),
                "payload_landing_rope_distance_m": (
                    float(self.payload_landing_rope_distances[index])
                    if landing and self.payload_landing_rope_distances is not None
                    else math.nan
                ),
                "transport_blend": transport_blend,
                "payload_position": payload_state["position"],
                "payload_raw_velocity": payload_state.get(
                    "raw_velocity", payload_state["velocity"]
                ),
                "payload_velocity": payload_state["velocity"],
                "payload_acceleration": payload_state["acceleration"],
                "payload_body_rate": payload_state["body_rate"],
                "payload_filter_derivatives_valid": payload_state.get(
                    "derivatives_valid", False
                ),
            }
            if self.payload_handoff_forces is not None:
                overrides[id(vehicle)]["handoff_force"] = self.payload_handoff_forces[index].copy()
            if transport_blend < 1.0:
                overrides[id(vehicle)]["flight_phase"] = "tension_ramp"
        try:
            self._apply_attitude_reference_trim(now, payload_state, control_states,
                                               overrides, landing)
        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
            if landing:
                return self._payload_landing_planning_failed(now, error, payload_state)
            raise
        self._apply_transport_hold_integral(now, payload_state, vehicle_states,
                                            control_states, overrides, landing)
        if not all(self._finite_transport_override(target) for target in overrides.values()):
            raise ValueError("cooperative controller returned nonfinite command references")
        return overrides

    def _transport_learning_reason(self, now, payload_state, control_states,
                                   overrides, landing, enabled, supported):
        """Use one fleet health gate for the slow reference and force learners."""
        elapsed = float(now) - float(self.payload_transport_start_time)
        reference_end = max(
            self.payload_attitude_reference_s if self.payload_attitude_reference_enabled else 0.,
            self.payload_tension_ramp_s + self.payload_reference_lift_s)
        reason = ""
        if not enabled:
            reason = "disabled"
        elif not supported:
            reason = "unsupported_force_mode"
        elif self.global_abort_reason:
            reason = "global_abort"
        elif landing or self.payload_validation_landing_stage or self.payload_controlled_exit_reason:
            reason = "landing"
        elif not self.payload_transport_active:
            reason = "inactive"
        elif elapsed < reference_end:
            reason = "reference_moving"
        elif (self.payload_flight_validation_enabled
              and elapsed >= self.payload_flight_validation_transport_s):
            reason = "validation_due"
        if not reason:
            try:
                source_age = float(rospy.Time.now().to_sec()) - float(payload_state["sample_time"])
            except (AttributeError, KeyError, TypeError, ValueError):
                source_age = math.nan
            future_tolerance = float(getattr(self.vehicles[0], "mocap_timestamp_future_tolerance_s", 0.))
            if (not self._payload_ready(now) or not math.isfinite(source_age)
                    or not -future_tolerance <= source_age <= self.vehicles[0].state_timeout):
                reason = "payload_feedback"
            elif any(not state.get("ekf_state_valid", False)
                     or state.get("ekf_kinematics_held", False)
                     or not math.isfinite(float(state.get("ekf_kinematics_weight_effective", 0.)))
                     or state.get("ekf_kinematics_weight_effective", 0.) <= 0.
                     or not state.get("derivatives_valid", False)
                     or not state.get("filter_derivatives_valid", state.get("derivatives_valid", False))
                     for state in control_states):
                reason = "vehicle_feedback"
            elif any(not math.isfinite(float(overrides[id(vehicle)].get("desired_tension_signed", 0.)))
                     or overrides[id(vehicle)].get("desired_tension_signed", 0.) <= 0.
                     for vehicle in self.vehicles):
                reason = "nonpositive_tension"
            elif any(any(bool((getattr(vehicle, "last_control_command", None) or {}).get(key, False))
                         for key in ("force_limit_active", "hold_feedback_limit_active",
                                     "body_rate_limit_active", "collective_limit_active"))
                     for vehicle in self.vehicles):
                reason = "host_command_limited"
        return reason

    def _attitude_trim_metadata(self, overrides, offsets, reason):
        trim = getattr(self, "payload_attitude_trim", None)
        enabled = bool(getattr(self, "payload_attitude_trim_enabled", False)
                       and trim is not None
                       and self.slung_controller.config.transport_force_mode == "reference_cascade")
        for index, vehicle in enumerate(self.vehicles):
            overrides[id(vehicle)].update(
                transport_attitude_trim_enabled=enabled,
                transport_attitude_trim_learning=enabled and not reason and bool(trim.learning),
                transport_attitude_trim_freeze_reason=reason,
                transport_attitude_trim_error=(trim.error if enabled else np.zeros(3)),
                transport_attitude_trim_integral=(trim.integral if enabled else np.zeros(3)),
                transport_attitude_trim_position=offsets[0][index].copy(),
                transport_attitude_trim_velocity=offsets[1][index].copy(),
                transport_attitude_trim_acceleration=offsets[2][index].copy(),
                transport_attitude_trim_limit_active=enabled and bool(trim.saturation),
                transport_attitude_trim_rate_limit_active=enabled and bool(trim.rate_limited),
                transport_attitude_trim_acceleration_limit_active=enabled and bool(trim.acceleration_limited))

    @staticmethod
    def _evaluate_attitude_trim_bridge(bridge, now):
        elapsed = max(0., float(now) - bridge["start_time"])
        shape = bridge["segments"][0][1].shape[1:]
        for duration, coefficients in bridge["segments"]:
            if elapsed < duration:
                u = elapsed / duration
                p = np.polynomial.polynomial.polyval(u, coefficients)
                v = np.polynomial.polynomial.polyval(u,
                    coefficients[1:] * np.arange(1, len(coefficients))[:, None, None]) / duration
                a = np.polynomial.polynomial.polyval(u,
                    coefficients[2:] * (np.arange(2, len(coefficients))
                    * np.arange(1, len(coefficients)-1))[:, None, None]) / duration**2
                return p, v, a
            elapsed -= duration
        return tuple(np.zeros(shape) for _ in range(3))

    def _make_attitude_trim_landing_bridge(self, now, overrides):
        """Bridge the old command into measured rope geometry exactly once.

        The measured rope snapshot already includes achieved trim. A residual
        from the previous complete command preserves its p/v/a without adding
        that trim again. A normalized quintic removes the residual; difficult
        initial derivatives first use a C2 acceleration ramp and velocity brake.
        """
        previous = self.last_transport_overrides
        if not all(self._finite_transport_override(previous.get(id(v), {})) for v in self.vehicles):
            return None
        old = {key: dict(value) for key, value in previous.items()}
        self._refresh_cached_attitude_trim(now, old, "landing")
        names = (("position", "velocity", "acceleration"),
                 ("position_hold_position", "position_hold_velocity", "position_hold_acceleration"))
        p, v, a = [np.asarray([old[id(vehicle)][fields[order]]
                    - overrides[id(vehicle)][fields[order]]
                    for fields in names for vehicle in self.vehicles]) for order in range(3)]
        distance = float(np.max(np.linalg.norm(p, axis=1)))
        speed_limit = max(self.payload_landing_max_speed_mps,
                          float(np.max(np.linalg.norm(v, axis=1))))
        acceleration_limit = max(self.payload_landing_max_acceleration_mps2,
                                 float(np.max(np.linalg.norm(a, axis=1))))
        minimum = max(float(self.payload_landing_gain_transition_s), 1.e-6)
        duration = max(minimum, 1.875 * distance / speed_limit,
                       math.sqrt((10. / math.sqrt(3.)) * distance / acceleration_limit))
        segments = []
        for _ in range(3):
            coefficients = SlungLandingProfile._quintic_coefficients(p, v, a, np.zeros_like(p), duration)
            velocity = coefficients[1:] * np.arange(1, 6)[:, None, None] / duration
            acceleration = velocity[1:] * np.arange(1, 5)[:, None, None] / duration
            if all(SlungLandingProfile._peak_norm(velocity[:, i]) <= speed_limit + 1.e-9
                   and SlungLandingProfile._peak_norm(acceleration[:, i]) <= acceleration_limit + 1.e-9
                   for i in range(len(p))):
                segments = [(duration, coefficients)]
                break
            duration *= 1.15
        if not segments:
            # Keep every aircraft's inherited speed ceiling while ramping its
            # acceleration to zero. An outward acceleration at that ceiling
            # has no C2 feasible ramp and must use the supported retry path.
            ramp = .1
            if np.any(a):
                for initial_v, initial_a in zip(v, a):
                    anorm2 = float(initial_a @ initial_a)
                    if anorm2 == 0.:
                        continue
                    dot = float(initial_v @ initial_a)
                    margin = max(0., speed_limit**2 - float(initial_v @ initial_v))
                    discriminant = math.sqrt(dot**2 + anorm2*margin)
                    maximum = (2.*margin/(discriminant+dot) if dot >= 0. and discriminant+dot > 0.
                               else 2.*(discriminant-dot)/anorm2)
                    if maximum <= 0.:
                        raise ValueError("attitude trim landing bridge has outward acceleration at its speed ceiling")
                    ramp = min(ramp, .5*maximum)
                coefficients = np.asarray([p, v*ramp, .5*a*ramp**2,
                                           -a*ramp**2/6.])
                segments.append((ramp, coefficients))
                p = p + v*ramp + a*ramp**2/3.
                v = v + .5*a*ramp
            else:
                ramp = 0.
            brake = max(1.e-9, 1.5 * float(np.max(np.linalg.norm(v, axis=1))) / acceleration_limit)
            segments.append((brake, np.asarray([p, v*brake, np.zeros_like(p),
                                                -v*brake, .5*v*brake])))
            p = p + .5*brake*v
            distance = float(np.max(np.linalg.norm(p, axis=1)))
            duration = max(1.e-9, minimum-ramp-brake, 1.875*distance/speed_limit,
                           math.sqrt((10./math.sqrt(3.))*distance/acceleration_limit))
            segments.append((duration, SlungLandingProfile._quintic_coefficients(
                p, np.zeros_like(p), np.zeros_like(p), np.zeros_like(p), duration)))
        return {"start_time": float(now), "duration": sum(segment[0] for segment in segments),
                "segments": segments}

    def _apply_attitude_reference_trim(self, now, payload_state, control_states,
                                       overrides, landing):
        trim = getattr(self, "payload_attitude_trim", None)
        enabled = bool(getattr(self, "payload_attitude_trim_enabled", False) and trim is not None)
        supported = self.slung_controller.config.transport_force_mode == "reference_cascade"
        reason = self._transport_learning_reason(now, payload_state, control_states,
                                                overrides, landing, enabled, supported)
        offsets = tuple(np.zeros((len(self.vehicles), 3)) for _ in range(3))
        if enabled and supported:
            goal = self.payload_transport_goal_rotation
            if goal is None:
                goal = next(iter(overrides.values()))["payload_target_rotation"]
            if landing:
                trim.freeze()
            offsets = trim.update(payload_state["rotation"], goal, now, learning=not reason)
        self._attitude_trim_metadata(overrides, offsets, reason)
        if landing and enabled and supported and trim.initialized:
            if self.payload_attitude_trim_landing_bridge is None:
                self.payload_attitude_trim_landing_bridge = self._make_attitude_trim_landing_bridge(now, overrides)
            bridge = self.payload_attitude_trim_landing_bridge
            applied = (self._evaluate_attitude_trim_bridge(bridge, now) if bridge is not None else
                       tuple(np.zeros((2*len(self.vehicles), 3)) for _ in range(3)))
        else:
            applied = tuple(np.concatenate((value, value), axis=0) for value in offsets)
        count = len(self.vehicles)
        for index, vehicle in enumerate(self.vehicles):
            item = overrides[id(vehicle)]
            for hold, fields in enumerate((("position", "velocity", "acceleration"),
                    ("position_hold_position", "position_hold_velocity", "position_hold_acceleration"))):
                row = index + hold*count
                for derivative, field in enumerate(fields):
                    item[field] = item[field] + applied[derivative][row]
            item["_attitude_trim_applied"] = np.asarray([value[index] for value in applied])
            item["_attitude_trim_hold_applied"] = np.asarray([value[index+count] for value in applied])
            item["_attitude_trim_landing"] = bool(landing)
            item["transport_attitude_trim_bridge_active"] = bool(landing and enabled and supported and trim.initialized
                and self.payload_attitude_trim_landing_bridge is not None
                and now < self.payload_attitude_trim_landing_bridge["start_time"] + self.payload_attitude_trim_landing_bridge["duration"])
            for hold, prefix in enumerate(("transport_attitude_trim_bridge_", "transport_attitude_trim_bridge_hold_")):
                for derivative, name in enumerate(("position", "velocity", "acceleration")):
                    item[prefix+name] = applied[derivative][index+hold*count].copy() if landing else np.zeros(3)

    def _refresh_cached_attitude_trim(self, now, overrides, reason, freeze_base_derivatives=False):
        """Finish bounded motion across a feedback gap without learning stale pose."""
        trim = getattr(self, "payload_attitude_trim", None)
        enabled = bool(getattr(self, "payload_attitude_trim_enabled", False)
                       and trim is not None
                       and self.slung_controller.config.transport_force_mode == "reference_cascade")
        if not enabled:
            return
        offsets = trim.update(None, None, now, learning=False)
        bridge = getattr(self, "payload_attitude_trim_landing_bridge", None)
        if bridge is not None and any(t.get("_attitude_trim_landing", False) for t in overrides.values()):
            applied = self._evaluate_attitude_trim_bridge(bridge, now)
        else:
            applied = tuple(np.concatenate((value, value), axis=0) for value in offsets)
        count = len(self.vehicles)
        for index, vehicle in enumerate(self.vehicles):
            item = overrides[id(vehicle)]
            for hold, (fields, key) in enumerate(((("position", "velocity", "acceleration"), "_attitude_trim_applied"),
                    (("position_hold_position", "position_hold_velocity", "position_hold_acceleration"), "_attitude_trim_hold_applied"))):
                old = np.asarray(item.get(key, np.zeros((3, 3))))
                row = index+hold*count
                for derivative, field in enumerate(fields):
                    base = (np.zeros(3) if freeze_base_derivatives and derivative else item[field]-old[derivative])
                    item[field] = base + applied[derivative][row]
                item[key] = np.asarray([value[row] for value in applied])
                prefix = "transport_attitude_trim_bridge_hold_" if hold else "transport_attitude_trim_bridge_"
                for derivative, name in enumerate(("position", "velocity", "acceleration")):
                    item[prefix+name] = applied[derivative][row].copy() if item.get("_attitude_trim_landing", False) else np.zeros(3)
            item["transport_attitude_trim_bridge_active"] = bool(bridge is not None and item.get("_attitude_trim_landing", False)
                and now < bridge["start_time"]+bridge["duration"])
        self._attitude_trim_metadata(overrides, offsets, reason)

    def _apply_transport_hold_integral(self, now, payload_state, vehicle_states,
                                       control_states, overrides, landing):
        """Learn differential world forces only from steady, healthy transport.

        Keep learned forces through temporary feedback gaps and load lowering.
        Touchdown releases them with the same profile as the support trim.
        All limits freeze the fleet together, preserving its zero force sum.
        """
        native_pid = self.payload_transport_feedback_controller == "native_position_pid"
        enabled = native_pid or self.payload_transport_integral_enabled
        supported = self.slung_controller.config.transport_force_mode == "reference_cascade"
        reason = self._transport_learning_reason(now, payload_state, control_states,
                                                overrides, landing, enabled, supported)
        learning = not reason
        duplicate = self.payload_transport_integral_last_time == float(now)
        errors = np.asarray([
            state["position"] - overrides[id(vehicle)]["position_hold_position"]
            for vehicle, state in zip(self.vehicles, vehicle_states)])
        dt = float(np.clip(getattr(self, "payload_control_dt", 1. / self.rate_hz), .001, .05))
        if native_pid:
            bias = np.zeros((3, 3))
            rate = np.zeros((3, 3))
        else:
            bias, rate = self.payload_transport_integral.update(
                errors, dt, learning=learning and not duplicate)
            self.payload_transport_integral_last_time = float(now)
        # Disabled/legacy branches cannot apply a previous mode's state.
        force = bias.copy() if enabled and supported and not native_pid else np.zeros_like(bias)
        force_dot = rate.copy() if enabled and supported and not native_pid else np.zeros_like(rate)
        integral_weight, integral_weight_dot = 1., 0.
        if landing and self.payload_unload_start_time is not None:
            unload = smoothstep5_profile(now - self.payload_unload_start_time, self.payload_landing_unload_s)
            integral_weight, integral_weight_dot = 1. - unload[0], -unload[1]
            force = integral_weight * force
            force_dot = integral_weight_dot * bias if enabled and supported else np.zeros_like(rate)
        for index, vehicle in enumerate(self.vehicles):
            overrides[id(vehicle)].update(
                transport_hold_integral_enabled=enabled and not native_pid,
                transport_hold_integral_learning=learning and not duplicate and not native_pid,
                transport_hold_integral_freeze_reason=("replaced_by_native_position_pid" if native_pid
                    else reason or ("duplicate_tick" if duplicate else "learning")),
                transport_hold_integral_bias=bias[index].copy(),
                transport_hold_integral_error=self.payload_transport_integral.differential_error_m[index].copy(),
                transport_hold_integral_force=force[index].copy(),
                transport_hold_integral_force_dot=force_dot[index].copy(),
                transport_feedback_controller=self.payload_transport_feedback_controller,
                transport_full_pid_learning=native_pid and learning,
                transport_full_pid_freeze_reason=reason if native_pid else "inactive_mode",
                transport_full_pid_tick=float(now),
                transport_full_pid_weight=integral_weight,
                transport_full_pid_weight_dot=integral_weight_dot)

    def _transport_phase_active(self, now):
        if self.transport_mode != "slung_load":
            return False
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._state_ready(now):
                return False
            target = vehicle._target_for(state, now)
            phase = target.get("flight_phase")
            if phase in ("landing", "landing_settle") and not self.payload_transport_active:
                return False
            if phase not in (
                    "height_correction", "hover", "figure_eight", "landing",
                    "landing_settle"):
                return False
        return True

    def _payload_landing_release_ready(self, now):
        """Match MATLAB LANDING_TAUT: release only after load reaches ground."""
        if not self.payload_transport_active or self.slung_controller is None:
            return False
        phases = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None:
                return False
            phase = vehicle._target_for(state, now).get("flight_phase")
            phases.append(phase)
        if not phases or not all(phase == "landing_settle" for phase in phases):
            return False
        with self.payload_lock:
            state = None if self.latest_payload_state is None else dict(self.latest_payload_state)
        if state is None or not state.get("valid", False):
            return False
        try:
            position = np.asarray(state.get("position"), dtype=float).reshape(3)
            velocity = np.asarray(state.get("velocity"), dtype=float).reshape(3)
        except (TypeError, ValueError):
            return False
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return False
        ground_z = 0.5 * float(self.payload_height_m)
        altitude_tolerance = float(self.vehicles[0].trajectory_config.landing_altitude_tolerance_m)
        velocity_tolerance = float(
            self.vehicles[0].trajectory_config.landing_vertical_velocity_tolerance_mps
        )
        return (
            abs(float(position[2]) - ground_z) <= altitude_tolerance and
            abs(float(velocity[2])) <= velocity_tolerance
        )

    def _begin_payload_emergency_landing(self, now, reason):
        """Switch each vehicle to its existing controlled landing path.

        A transient load-mocap loss must never be handled by the fleet-wide
        zero-thrust abort while aircraft are airborne.  The vehicle state
        machines already have a slow emergency landing reference; reuse it.
        """
        if (self.payload_transport_active
                and self.payload_validation_landing_stage == "payload_lowering"
                and self._payload_ready(now)
                and not str(reason).startswith("touchdown_timeout:")):
            # Keep the payload-aware landing active.  Switching all aircraft
            # to independent ground references makes them descend beside an
            # airborne/attached load and destroys the rope geometry.
            self.payload_payload_aware_emergency = True
            self.payload_controlled_exit_reason = "payload_load: " + str(reason)
            rospy.logwarn(
                "距离/高度保护转入绳向保持下降：%s；等待负载触地确认后再释放绳向。",
                reason,
            )
            return
        if (str(reason).startswith("height_drop:") and self.payload_transport_active
                and self._payload_ready(now) and not self.payload_controlled_exit_reason):
            # A valid, separated loaded fleet can still lower the load. Do not
            # remove its support by switching straight to aircraft-only gravity.
            self.payload_controlled_exit_reason = "payload_load: " + str(reason)
            self.payload_validation_landing_stage = "payload_lowering"
            self.payload_landing_profile = None
            self.payload_landing_start_time = None
            self.payload_unload_start_time = None
            self._begin_validation_vehicle_descent(now)
            return
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if (state is None or not state.get("valid", False)
                    or vehicle.trajectory is None):
                continue
            command = getattr(vehicle, "last_control_command", None)
            if command is not None and vehicle.trajectory.phase not in ("emergency_landing", "landed", "aborted"):
                control = vehicle._build_ekf_control_state(state, now)
                velocity_z = float(control.get("control_velocity", np.zeros(3))[2])
                baseline = vehicle.controller.config.mass * (
                    vehicle.controller.config.gravity - vehicle.controller.config.independent_velocity_gain[2] * velocity_z)
                residual = float(command["desired_force"][2]) - baseline
                self.payload_emergency_support[id(vehicle)] = (float(now), residual)
            vehicle.trajectory.begin_emergency_landing(
                now,
                state["position"],
                rotation_to_rpy(state["rotation"])[2],
                "payload_load: " + str(reason),
            )

    def _check_payload_flight_validation(self, now):
        """End a short flight once, using the existing measured-position descent."""
        if (not self.payload_flight_validation_enabled
                or self.payload_validation_landing_started_time is not None
                or not self.payload_transport_active
                or self.payload_transport_start_time is None
                or self.global_abort_reason):
            return False
        if any(v.trajectory is None or v.trajectory.phase in (
                "emergency_landing", "landing_settle", "landed", "aborted")
               for v in self.vehicles):
            return False
        elapsed = max(0.0, float(now) - self.payload_transport_start_time)
        if elapsed < self.payload_flight_validation_transport_s:
            return False
        self.payload_validation_handoff_time = self.payload_transport_start_time
        self.payload_validation_landing_started_time = float(now)
        rospy.loginfo(
            "[实飞验证] flight_validation_complete：接管已运行 %.3f s（上限 %.3f s）；"
            "三机从当前实测位置开始受控降落。",
            elapsed, self.payload_flight_validation_transport_s,
        )
        self.payload_validation_landing_stage = "payload_lowering"
        self.payload_controlled_exit_reason = "payload_load: flight_validation_complete: %.3f s" % elapsed
        self.payload_validation_ground_since = None
        self.payload_landing_start_time = None
        self.payload_landing_start_position = None
        self._begin_validation_vehicle_descent(now)
        return True

    def _begin_validation_vehicle_descent(self, now):
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if (state is not None and state.get("valid", False)
                    and vehicle.trajectory is not None):
                vehicle.trajectory.begin_controlled_landing(
                    now, state["position"], rotation_to_rpy(state["rotation"])[2],
                    self.payload_controlled_exit_reason,
                )

    def _release_validation_payload(self, now):
        """Release taut support after fresh measured payload touchdown, then land aircraft."""
        if (self.payload_validation_landing_stage != "payload_lowering"
                or not self.payload_transport_active or not self._payload_ready(now)):
            return False
        with self.payload_lock:
            payload = dict(self.latest_payload_state)
        clearance = payload_ground_clearance(
            payload["position"], payload["rotation"], self.slung_controller.config.payload_size_m)
        if self.payload_touchdown_detector is None:
            return False
        detected = self.payload_touchdown_detector.update(
            payload.get("received_time", now), clearance, float(payload["velocity"][2]))
        if not detected:
            if (self.payload_landing_profile is not None and
                    self.payload_touchdown_detector.timed_out(
                        now, self.payload_landing_profile.start_time + self.payload_landing_profile.duration)):
                self._begin_payload_emergency_landing(now, "touchdown_timeout: 未确认负载稳定接地")
                rospy.logwarn_throttle(1., "负载接地未确认（间隙 %.3f m），转入保留支撑残差的独立下降；不标记为接地。",
                                       clearance)
                self._reset_payload_handoff()
            return False
        if self.payload_unload_start_time is None:
            self.payload_unload_start_time = float(now)
            self.payload_landing_trim_at_unload = self.payload_support_trim.bias.copy()
            rospy.loginfo("负载稳定接地已确认，开始 %.2f s 有界卸载。", self.payload_landing_unload_s)
            return False
        if now - self.payload_unload_start_time < self.payload_landing_unload_s:
            return False
        self._reset_payload_handoff()
        self.payload_validation_landing_stage = "vehicle_descent"
        for vehicle in self.vehicles:
            # A negative prelift integral supported the airborne load. Do not
            # revive that extra lift under the larger landing limit. Preserve
            # only an already learned, ordinary bounded downward correction.
            ordinary_limit = float(vehicle.controller.config.independent_integral_limit[2])
            vehicle.controller.position_integral[2] = np.clip(
                vehicle.controller.position_integral[2], 0.0, ordinary_limit)
        self._begin_validation_vehicle_descent(now)
        rospy.loginfo("[实飞验证] 负载已落地稳定，释放协同拉力；三机保持各自当前位置 XY 继续降落。")
        return True

    def _update_payload_ground_reference(self, now):
        """Measure the resting load origin before TAKEUP, then freeze that floor."""
        if self.payload_takeup_start_time is not None or self.payload_transport_active or not self._payload_ready(now):
            return
        phases = [v.trajectory.phase if v.trajectory is not None else "waiting_for_ekf" for v in self.vehicles]
        if self.payload_ground_reference_samples and not all(p in (
                "waiting_for_ekf", "waiting_for_fleet", "pre_takeoff_hold", "waiting_for_reset") for p in phases):
            return
        with self.payload_lock:
            payload = dict(self.latest_payload_state)
        clearance = payload_ground_clearance(payload["position"], payload["rotation"], self.slung_controller.config.payload_size_m)
        self.payload_ground_reference_samples.append((float(payload["position"][2]), max(0., clearance)))
        self.payload_ground_center_z = float(np.median([v[0] for v in self.payload_ground_reference_samples]))
        self.payload_ground_clearance_reference_m = float(np.median([v[1] for v in self.payload_ground_reference_samples]))

    def _check_payload_physical_safety(self, now):
        """Check measured geometry before publishing another cooperative tick."""
        if self.transport_mode != "slung_load":
            return False
        if self.payload_payload_aware_emergency:
            return False
        with self.payload_lock:
            payload = None if self.latest_payload_state is None else dict(self.latest_payload_state)
        if payload is not None and self._payload_ready(now):
            self.payload_ground_clearance_m = payload_ground_clearance(
                payload["position"], payload["rotation"], self.slung_controller.config.payload_size_m,
            )
            if self.payload_transport_active:
                self.payload_prelift_ready = bool(
                    self.payload_ground_clearance_m >= self.payload_prelift_clearance_m
                    and abs(float(payload.get("velocity", np.zeros(3))[2]))
                    <= self.payload_prelift_vertical_velocity_tolerance_mps
                )
        else:
            self.payload_ground_clearance_m = math.nan
            self.payload_prelift_ready = False
        # Check after the TAKEUP motion has begun, including independent
        # prelift, so a close formation never reaches the handoff gate.
        if not (self.payload_transport_active or self.payload_takeup_start_time is not None
                or self.payload_validation_landing_stage == "vehicle_descent"
                or self.payload_emergency_support):
            return False
        positions, velocities, phases = [], [], []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._state_ready(now):
                return False  # Existing state-loss handling owns this path.
            positions.append(state["position"])
            control = vehicle._build_ekf_control_state(state, now)
            velocities.append(control.get("control_velocity", np.zeros(3)))
            phases.append(vehicle.trajectory.phase if vehicle.trajectory is not None else "")
        emergency_active = "emergency_landing" in phases
        if "landed" in phases and not (
                self.payload_validation_landing_stage == "vehicle_descent"
                and any(phase in ("landing", "landing_settle") for phase in phases)):
            return False
        height_error = None
        rate_saturated = False
        if self.payload_transport_active and self._payload_ready(now) and not any(
                phase in ("landing", "landing_settle") for phase in phases):
            # During the ramp, the frozen payload height is the actual target.
            # During lift, read the previous complete reference (one tick old).
            previous = next(iter(self.last_transport_overrides.values()), {})
            target_position = previous.get("payload_target_position", self.payload_transport_start_position)
            if target_position is not None:
                height_error = float(target_position[2] - payload["position"][2])
            rate_saturated = any(
                getattr(vehicle, "last_control_command", None) is not None and np.any(
                    np.abs(vehicle.last_control_command["body_rate_command"][:2]) >=
                    0.98 * vehicle.controller.config.max_body_rate[:2])
                for vehicle in self.vehicles
            )
        result = self.payload_transport_guard.check(
            now, positions, velocities, height_error, rate_saturated,
        )
        self.payload_minimum_vehicle_distance_m = result["minimum_distance_m"]
        self.payload_predicted_vehicle_distance_m = result["predicted_distance_m"]
        if emergency_active:
            # Keep measured proximity visible during the latched descent;
            # repeatedly restarting its reference would prevent landing.
            if result["reason"]:
                rospy.logwarn_throttle(1., "受控故障下降中机间距 %.3f m，预测 %.3f m。",
                                       self.payload_minimum_vehicle_distance_m,
                                       self.payload_predicted_vehicle_distance_m)
            return False
        if not result["reason"]:
            return False
        self.payload_safety_reason = result["reason"]
        self._begin_payload_emergency_landing(
            now, "%s: distance=%.3f m predicted=%.3f m height_error=%s" % (
                result["reason"], result["minimum_distance_m"],
                result["predicted_distance_m"], height_error),
        )
        if (self.payload_controlled_exit_reason
                and all(v.trajectory is not None and v.trajectory.phase == "landing"
                        for v in self.vehicles)):
            # Height fault transitioned to supported descent. Keep its caches
            # and rebuild the descent override on this same control tick.
            return True
        clearance = self.payload_ground_clearance_m
        self._reset_payload_handoff()
        self.payload_ground_clearance_m = clearance
        return True

    def _reset_payload_handoff(self):
        if getattr(self, "payload_attitude_trim", None) is not None:
            self.payload_attitude_trim.reset()
        self.payload_attitude_trim_landing_bridge = None
        self.payload_transport_active = False
        self.payload_ready_since = None
        self.payload_transport_start_time = None
        self.payload_transport_start_position = None
        self.payload_transport_start_rotation = None
        self.payload_transport_goal_rotation = None
        self.payload_transport_initial_link_units = None
        self.payload_transport_initial_link_distances = None
        self.payload_transport_goal_link_units = None
        self.payload_transport_vehicle_start_positions = None
        self.payload_takeup_start_time = None
        self.payload_takeup_start_positions = None
        self.payload_takeup_target_positions = None
        self.payload_takeup_ready_since = None
        self.payload_prelift_ready_since = None
        self.payload_ground_clearance_m = math.nan
        self.payload_prelift_ready = False
        self.payload_prelift_reference = None
        self.payload_link_calibration_hold_reference = None
        self.payload_last_takeup_reference = None
        self.payload_geometry_ready_since = None
        self.payload_mission_hold_time = None
        self.payload_handoff_reference_positions = None
        self.payload_handoff_forces = None
        self.payload_support_trim.reset()
        self.payload_transport_integral.reset()
        self.payload_transport_integral_last_time = None
        for vehicle in self.vehicles:
            vehicle.controller.reset_transport_position_pid()
        self.payload_transport_guard.reset()
        self.payload_takeup_taut_latched = False
        self.payload_takeup_diagnostic_last_time = None
        self.payload_landing_start_time = None
        self.payload_landing_start_position = None
        self.payload_landing_profile = None
        self.payload_last_reference = None
        self.payload_landing_gain_snapshot = None
        self.payload_landing_rope_directions = None
        self.payload_landing_rope_distances = None
        self.payload_payload_aware_emergency = False
        self.payload_unload_start_time = None
        self.payload_landing_trim_at_unload = None
        self.payload_fault_since = None
        self.last_transport_overrides = {}
        if self.slung_controller is not None:
            self.slung_controller.reset()

    def _all_states_ready(self, now):
        return all(vehicle._state_ready(now) for vehicle in self.vehicles)

    def _all_preflight_ready(self, now):
        if not self._payload_ready(now):
            return False
        for vehicle in self.vehicles:
            if not vehicle._state_ready(now):
                return False
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._ekf_preflight_ready(state, now):
                return False
        return True

    def _set_abort(self, reason):
        if self.global_abort_reason:
            return
        self.global_abort_reason = str(reason)
        if getattr(self, "payload_attitude_trim", None) is not None:
            self.payload_attitude_trim.reset()
        self.payload_attitude_trim_landing_bridge = None
        self.payload_transport_integral.reset()
        self.payload_transport_integral_last_time = None
        for vehicle in self.vehicles:
            vehicle.controller.reset_transport_position_pid()
            vehicle.set_global_abort(self.global_abort_reason)
        rospy.logerr("多机 CTBR 全局中止：%s；所有飞机发送零推力", reason)

    def _payload_emergency_override(self, vehicle, now):
        if (vehicle.trajectory is None or vehicle.trajectory.phase != "emergency_landing"
                or id(vehicle) not in self.payload_emergency_support):
            return {}
        started, initial_trim = self.payload_emergency_support[id(vehicle)]
        duration = max(2., float(vehicle.trajectory.active_landing_duration_s))
        fade = smoothstep5_profile(now - started, duration)
        return dict(independent_mode=True, transport_mode=False,
            independent_position_gain_scale=self.payload_prelift_parameters["position_gain_scale"],
            independent_velocity_gain_scale=self.payload_prelift_parameters["velocity_gain_scale"],
            max_tilt_rad=self.payload_prelift_parameters["max_tilt_rad"],
            landing_support_trim_n=(1 - fade[0]) * initial_trim,
            landing_support_trim_dot_nps=-fade[1] * initial_trim)

    @staticmethod
    def _finite_recovery_state(vehicle, now):
        try:
            if not vehicle._state_ready(now):
                return False
            with vehicle.lock:
                state = vehicle.latest_state
            control = vehicle._build_ekf_control_state(state, now)
            return (np.asarray(state["position"]).shape == (3,)
                    and np.asarray(state["rotation"]).shape == (3, 3)
                    and all(np.all(np.isfinite(value)) for value in (
                        state["position"], state["rotation"], state.get("body_rate", np.zeros(3)),
                        control.get("control_velocity", np.zeros(3)),
                        control.get("control_acceleration", np.zeros(3)))))
        except (ValueError, ArithmeticError, np.linalg.LinAlgError):
            return False

    def _recover_fleet_numerical_fault(self, now, error):
        """Contain numerical faults without bypassing arming or state freshness."""
        if not self.fleet_numerical_fault_reason:
            self.fleet_numerical_fault_reason = "%s: %s" % (type(error).__name__, error)
            self.fleet_numerical_fault_since = float(now)
            rospy.logerr("多机控制数值故障，转入有支撑受控退出：%s", self.fleet_numerical_fault_reason)
        now = max(float(now), float(time.monotonic()))
        self.last_tick = now
        healthy = all(self._finite_recovery_state(v, now) for v in self.vehicles)
        if self.fleet_gate_open and not healthy:
            self._set_abort("fleet_numerical: invalid or stale vehicle state")
        armed = (self.fleet_gate_open and healthy and not self.global_abort_reason
                 and all(v.preflight_voltage_ready and not v.preflight_voltage_failed
                         and v.trajectory is not None for v in self.vehicles))
        if armed:
            try:
                if self.payload_validation_landing_stage != "planning_fault_descent":
                    self._exit_payload_planning_fault(now, self.fleet_numerical_fault_reason,
                                                     source="fleet_numerical")
                # Fresh measured separation and height diagnostics still run;
                # the failed cooperative planner is never called here.
                self._check_payload_physical_safety(now)
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as guard_error:
                self._set_abort("fleet_numerical: fault descent/guard failed: %s" % guard_error)
        for vehicle in self.vehicles:
            # The vehicle latches last_control_command after publication and
            # before CSV conversion. A logging failure must not actuate the
            # same vehicle again merely because its callback did not return.
            already_published = (id(vehicle) in self.fleet_tick_initial_commands
                and getattr(vehicle, "last_control_command", None)
                is not self.fleet_tick_initial_commands[id(vehicle)])
            if id(vehicle) in self.fleet_tick_completed_vehicles or already_published:
                continue
            command_now = max(now, float(time.monotonic()))
            if not armed or self.global_abort_reason:
                vehicle.last_tick = command_now
                vehicle._publish_zero()
                continue
            try:
                vehicle._timer_callback(None, now=command_now,
                    transport_override=self._payload_emergency_override(vehicle, command_now))
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as control_error:
                if (id(vehicle) in self.fleet_tick_initial_commands
                        and getattr(vehicle, "last_control_command", None)
                        is not self.fleet_tick_initial_commands[id(vehicle)]):
                    # Publication completed before a diagnostic conversion
                    # failed. Do not compute or publish its command twice.
                    self.fleet_tick_completed_vehicles.add(id(vehicle))
                    continue
                # A finite previously published command may bridge only the
                # short watchdog window if the independent core also fails.
                # It cannot become an indefinite open-loop force command.
                supported = self._finite_supported_command(getattr(vehicle, "last_control_command", None))
                if (supported and command_now - self.fleet_numerical_fault_since <= .3
                        and self._finite_recovery_state(vehicle, command_now)):
                    vehicle.last_tick = command_now
                    vehicle._publish_command(dict(vehicle.last_control_command))
                else:
                    self._set_abort("fleet_numerical: vehicle control failed: %s" % control_error)
                    vehicle._publish_zero()
            self.fleet_tick_completed_vehicles.add(id(vehicle))
        try:
            self.logger.flush()
        except (ValueError, OSError) as logging_error:
            rospy.logwarn_throttle(1., "控制退出命令已发布，CSV 刷新失败：%s", logging_error)

    def _timer_callback(self, _event):
        now = time.monotonic()
        wall_start = time.perf_counter()
        previous_start = getattr(self, "previous_fleet_cycle_started_time", math.nan)
        for vehicle in self.vehicles:
            vehicle.fleet_tick_interval_s = now - self.last_tick
            vehicle.previous_fleet_cycle_wall_s = getattr(self, "previous_fleet_cycle_wall_s", math.nan)
            vehicle.previous_fleet_cycle_control_time_s = previous_start - vehicle.start_time
        self.fleet_tick_completed_vehicles = set()
        self.fleet_tick_initial_commands = {
            id(vehicle): getattr(vehicle, "last_control_command", None)
            for vehicle in self.vehicles}
        try:
            return MultiCtbrControllerNode._run_fleet_tick(self, _event, now)
        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
            return self._recover_fleet_numerical_fault(now, error)
        finally:
            self.previous_fleet_cycle_wall_s = time.perf_counter() - wall_start
            self.previous_fleet_cycle_started_time = now

    def _run_fleet_tick(self, _event, now):
        self.payload_control_dt = float(np.clip(now - self.last_tick, 0.001, 0.05))
        self.last_tick = now
        if self.transport_mode == "slung_load":
            self._update_payload_ground_reference(now)

        if self.global_abort_reason:
            for vehicle in self.vehicles:
                vehicle._timer_callback(None, now=now)
                self.fleet_tick_completed_vehicles.add(id(vehicle))
            self._log_terminal_mission_stage()
            self.logger.flush()
            return

        if not self.fleet_gate_open:
            for vehicle in self.vehicles:
                vehicle.set_fleet_gate(False)
                vehicle._timer_callback(None, now=now)
                self.fleet_tick_completed_vehicles.add(id(vehicle))
            failed = [
                vehicle for vehicle in self.vehicles
                if vehicle.preflight_voltage_failed or vehicle.ekf_alignment_failed
            ]
            if failed:
                ids = ",".join("cf%d" % vehicle.vehicle_id for vehicle in failed)
                self._set_abort("%s 起飞前预检失败（电压或 EKF 对齐）" % ids)
                return
            if all(
                    vehicle.preflight_voltage_ready
                    for vehicle in self.vehicles
            ) and self._all_preflight_ready(now):
                self.fleet_gate_open = True
                for vehicle in self.vehicles:
                    vehicle.set_fleet_gate(True)
                    vehicle.last_tick = now
                rospy.loginfo(
                    "全部 %d 架飞机完成预检和 NOKOV 状态检查，同时开始 CTBR 轨迹",
                    len(self.vehicles),
                )
            self._log_terminal_mission_stage()
            self.logger.flush()
            return

        if not self._all_states_ready(now):
            invalid_ids = [
                "cf%d" % vehicle.vehicle_id
                for vehicle in self.vehicles
                if not vehicle._state_ready(now)
            ]
            self._set_abort("NOKOV 状态失效：%s" % ",".join(invalid_ids))

        self._hold_payload_mission_clock(now)
        self._release_validation_payload(now)
        transport_phase_active = self._transport_phase_active(now)
        if (transport_phase_active and self.payload_transport_active and
                self._payload_landing_release_ready(now)):
            rospy.loginfo("负载已达到地面高度并稳定，结束 MATLAB LANDING_TAUT，释放绷紧控制。")
            self._reset_payload_handoff()
            transport_phase_active = False
        transport_overrides = {}
        takeup_overrides = {}
        if not transport_phase_active:
            self._reset_payload_handoff()
        elif not self.payload_transport_active:
            takeup_overrides = self._payload_takeup_overrides(now)
            if self._payload_ready(now):
                self.payload_fault_since = None
            else:
                if self.payload_fault_since is None:
                    self.payload_fault_since = now
                if now - self.payload_fault_since > self.payload_emergency_land_after_s:
                    self._begin_payload_emergency_landing(now, "预抬期间负载状态持续失效")
                    takeup_overrides = {}
            takeup_ready = self._payload_takeup_ready(now)
            if self._payload_ready(now) and takeup_ready:
                if self.payload_ready_since is None:
                    self.payload_ready_since = now
                if now - self.payload_ready_since >= self.payload_activation_hold_s:
                    with self.payload_lock:
                        payload_state = dict(self.latest_payload_state)
                    current_vehicle_positions = []
                    for vehicle in self.vehicles:
                        with vehicle.lock:
                            state = vehicle.latest_state
                        if state is None:
                            current_vehicle_positions = []
                            break
                        current_vehicle_positions.append(
                            np.asarray(state["position"], dtype=float).copy()
                        )
                    self.payload_handoff_reference_positions = (
                        self.payload_last_takeup_reference["positions"].copy()
                    )
                    self.payload_handoff_forces = np.asarray([
                        vehicle.last_control_command["desired_force"].copy()
                        for vehicle in self.vehicles
                    ])
                    self.payload_transport_active = True
                    self.payload_transport_start_time = now
                    self.payload_transport_start_position = np.asarray(
                        payload_state["position"], dtype=float
                    ).copy()
                    self.payload_transport_start_rotation = project_so3(payload_state["rotation"])
                    self.payload_transport_goal_rotation = None
                    if len(current_vehicle_positions) == len(self.vehicles):
                        # Preserve continuity at the TAKEUP -> TAUT_RAMP handoff.
                        # The cable geometry is measured from these actual
                        # positions; jumping back to nominal anchors can create
                        # a large position step even though the cables are taut.
                        self.payload_transport_vehicle_start_positions = np.asarray(
                            current_vehicle_positions, dtype=float
                        )
                    elif self.payload_takeup_target_positions is not None:
                        self.payload_transport_vehicle_start_positions = (
                            self.payload_takeup_target_positions.copy()
                        )
                    else:
                        self.payload_transport_vehicle_start_positions = None
                    self.payload_fault_since = None
                    rospy.loginfo(
                        "TAKEUP 已确认，负载状态连续有效 %.2f s，开始 TAUT_RAMP；"
                        "先保持地面参考 %.2f s，再抬升 %.2f s。",
                        self.payload_activation_hold_s,
                        self.payload_tension_ramp_s,
                        self.payload_reference_lift_s,
                    )
                else:
                    rospy.logwarn_throttle(
                        self.payload_takeup_diagnostic_interval_s,
                        "TAKEUP 绳长条件已满足，等待负载状态连续有效 %.2f/%.2f s 后进入 TAUT_RAMP",
                        now - self.payload_ready_since,
                        self.payload_activation_hold_s,
                    )
            else:
                self.payload_ready_since = None
        if transport_phase_active and self.payload_transport_active:
            payload_ready = self._payload_ready(now)
            if payload_ready:
                transport_overrides = self._transport_overrides(now)
                if transport_overrides:
                    self.payload_fault_since = None
                    self.last_transport_overrides = transport_overrides
                elif self.payload_fault_since is None:
                    self.payload_fault_since = now
            elif self.payload_fault_since is None:
                self.payload_fault_since = now

            if (not transport_overrides and self.last_transport_overrides and
                    self.payload_fault_since is not None and
                    now - self.payload_fault_since <= self.payload_hold_s):
                # Keep the last complete MATLAB force command across a short
                # load-mocap gap; this avoids a thrust step at the exact moment
                # a single rigid-body frame is lost.
                transport_overrides = {
                    key: dict(value, transport_hold_integral_learning=False,
                              transport_hold_integral_freeze_reason="payload_cached_hold",
                              transport_hold_integral_force_dot=np.zeros(3),
                              transport_full_pid_learning=False,
                              transport_full_pid_freeze_reason="payload_cached_hold",
                              transport_full_pid_tick=float(now),
                              transport_full_pid_force_hold=True,
                              transport_full_pid_weight_dot=0.)
                    for key, value in self.last_transport_overrides.items()}
                self._refresh_cached_attitude_trim(now, transport_overrides, "payload_cached_hold")
            elif (not transport_overrides and self.payload_fault_since is not None and
                  now - self.payload_fault_since > self.payload_emergency_land_after_s):
                self._begin_payload_emergency_landing(
                    now, "load 状态持续失效 %.3f s" %
                    (now - self.payload_fault_since)
                )
                self.last_transport_overrides = {}
                self.payload_fault_since = None
        if self._check_payload_physical_safety(now):
            transport_overrides = (self._transport_overrides(now)
                                   if self.payload_transport_active and self.payload_controlled_exit_reason else {})
            takeup_overrides = {}
        # Fault guards own this tick first. A planned validation stop must not
        # replace a fault reason, and no cached cooperative output may survive.
        if self._check_payload_flight_validation(now):
            # Keep the taut-load controller while lowering the payload. Its
            # reference is rebuilt for landing, never reuse the old lift tick.
            transport_overrides = self._transport_overrides(now)
            self.last_transport_overrides = transport_overrides
            takeup_overrides = {}
        # The fleet tick's ``now`` is captured before path publication and
        # before the per-vehicle callbacks.  Use one refreshed timestamp for
        # all callbacks so a freshly received EKF sample cannot appear to be
        # from the future merely because an earlier callback spent time
        # publishing a growing Path message.
        vehicle_control_now = max(float(now), float(time.monotonic()))
        for vehicle_index, vehicle in enumerate(self.vehicles):
            with self.payload_lock:
                payload_diagnostics = (
                    None if self.latest_payload_state is None
                    else dict(self.latest_payload_state)
                )
            vehicle.set_payload_diagnostics(payload_diagnostics)
            vehicle.set_fleet_gate(True)
            target_override = transport_overrides.get(id(vehicle))
            if target_override is None:
                target_override = takeup_overrides.get(id(vehicle))
            emergency_override = self._payload_emergency_override(vehicle, now)
            if emergency_override:
                target_override = dict(target_override or {})
                target_override.update(emergency_override)
            if (self.payload_validation_landing_stage == "vehicle_descent"
                    and vehicle.trajectory is not None
                    and vehicle.trajectory.phase in ("landing", "landing_settle", "landed")):
                # The ropes can remain taut briefly after payload touchdown.
                # Keep the demonstrated TAKEUP XY/attitude loop while each
                # aircraft descends; weak ordinary gains/5-degree tilt let
                # rope tension collapse the formation during this transition.
                target_override = dict(target_override or {})
                target_override.update(
                    independent_mode=True, transport_mode=False,
                    independent_position_gain_scale=self.payload_prelift_parameters["position_gain_scale"],
                    independent_velocity_gain_scale=self.payload_prelift_parameters["velocity_gain_scale"],
                    max_tilt_rad=self.payload_prelift_parameters["max_tilt_rad"],
                )
                target_override.update(self.payload_landing_integral_parameters)
                target_override["landing_vertical_integral_active"] = True
            if self.transport_mode == "slung_load":
                target_override = dict(target_override or {})
                target_override.update({
                    "link_length_nominal_m": self.payload_nominal_link_lengths[vehicle_index],
                    "link_length_effective_m": self.slung_controller.config.link_lengths_m[vehicle_index],
                    "link_length_measured_m": self.payload_link_measured_distances[vehicle_index],
                    "link_calibration_enabled": self.payload_link_calibration_enabled,
                    "link_calibration_applied": self.payload_link_calibration_applied,
                    "link_calibration_status": (
                        "disabled" if not self.payload_link_calibration_enabled else
                        "awaiting_current_frame" if self.payload_link_calibration_pending is not None else
                        self.payload_link_calibrator.status),
                    "link_calibration_reason": self.payload_link_calibrator.reason,
                    "link_calibration_samples": self.payload_link_calibrator.sample_count,
                    "link_calibration_window_s": self.payload_link_calibrator.window_elapsed_s,
                    "link_calibration_spread_m": self.payload_link_calibrator.spread_m[vehicle_index],
                    "payload_ground_clearance_m": self.payload_ground_clearance_m,
                    "payload_prelift_clearance_target_m": self.payload_prelift_clearance_m,
                    "payload_prelift_ready": self.payload_prelift_ready,
                    "payload_minimum_vehicle_distance_m": self.payload_minimum_vehicle_distance_m,
                    "payload_predicted_vehicle_distance_m": self.payload_predicted_vehicle_distance_m,
                    "payload_safety_reason": self.payload_safety_reason,
                    "flight_validation_enabled": self.payload_flight_validation_enabled,
                    "flight_validation_landing_started": self.payload_validation_landing_started_time is not None,
                    "flight_validation_landing_stage": self.payload_validation_landing_stage,
                    "payload_ground_reference_z_m": self.payload_ground_center_z,
                    "payload_landing_duration_s": (self.payload_landing_profile.duration
                                                  if self.payload_landing_profile is not None else math.nan),
                    "payload_landing_touchdown_confirmed": (self.payload_touchdown_detector is not None
                                                            and self.payload_touchdown_detector.detected),
                    "payload_landing_unloading": self.payload_unload_start_time is not None,
                    "payload_landing_planning_failure_count": self.payload_landing_failure_count,
                    "payload_landing_planning_retry_count": self.payload_landing_retry_count,
                    "payload_landing_planning_failure_reason": self.payload_landing_failure_reason,
                    "payload_landing_planning_fallback": (self.payload_landing_retry_since is not None
                        or self.payload_validation_landing_stage == "planning_fault_descent"),
                    "flight_validation_elapsed_s": (
                        max(0.0, (self.payload_validation_landing_started_time
                                  if self.payload_validation_landing_started_time is not None else now)
                            - (self.payload_validation_handoff_time
                               if self.payload_validation_handoff_time is not None
                               else self.payload_transport_start_time))
                        if (self.payload_validation_handoff_time is not None
                            or self.payload_transport_start_time is not None) else math.nan),
                })
            if self.transport_mode == "slung_load" and self.payload_last_takeup_reference is not None:
                target_override = dict(target_override or {})
                target_override["payload_prelift_failure_reason"] = (
                    self.payload_last_takeup_reference["reason"]
                )
                target_override["payload_prelift_stage"] = (
                    self.payload_last_takeup_reference["stage"]
                    if not self.payload_transport_active else "transport"
                )
            vehicle._timer_callback(
                None,
                now=vehicle_control_now,
                transport_override=target_override,
            )
            self.fleet_tick_completed_vehicles.add(id(vehicle))
        self._log_terminal_mission_stage()
        self.logger.flush()

    def _shutdown(self):
        for vehicle in self.vehicles:
            vehicle.set_global_abort("节点关闭")
            vehicle._publish_zero()
        self.logger.close()


if __name__ == "__main__":
    rospy.init_node("ctbr_controller")
    MultiCtbrControllerNode()
    rospy.spin()
