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

from crazyswarm.msg import CTBR, GenericLogData, MocapState
from ctbr_trajectory import (
    CircularFlightTrajectory,
    CircularTrajectoryConfig,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


GLOBAL_CTBR_PARAMETER_ROOT = "/ctbr_controller"
TRAJECTORY_PARAMETER_ROOT = "/ctbr_trajectory"
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
        self.attitude_integral = np.zeros(3)
        self.previous_computed_rotation = None
        self.previous_heading = None

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
        ).reshape(3)
        target_jerk = np.asarray(
            target.get("jerk", np.zeros(3)), dtype=float
        ).reshape(3)
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

        # ROS world 系按 z 向上处理：F = m(g e3 + a_d + a_form)
        # - Kp ep - Kv ev - Ki ei。a_form 是 MATLAB formation 控制律
        # 在 z-up 坐标下的附加编队加速度；没有编队控制时为零。
        formation_acceleration = np.asarray(
            target.get("formation_acceleration", np.zeros(3)), dtype=float
        ).reshape(3)
        formation_acceleration_dot = np.asarray(
            target.get("formation_acceleration_dot", np.zeros(3)), dtype=float
        ).reshape(3)
        if not np.all(np.isfinite(formation_acceleration)):
            formation_acceleration = np.zeros(3)
        if not np.all(np.isfinite(formation_acceleration_dot)):
            formation_acceleration_dot = np.zeros(3)
        raw_desired_force = (
            self.config.mass * (
                target_acceleration + formation_acceleration
                + np.array([0.0, 0.0, self.config.gravity])
            )
            - self.config.position_gain * position_error
            - self.config.velocity_gain * velocity_error
            - self.config.integral_gain * self.position_integral
        )
        # The V2/formation derivative uses the *full* desired acceleration
        # u_i = a_d + a_form in the velocity-feedback term.  Comparing the
        # measured acceleration only with a_d would omit the formation law
        # from A_dot and make analytic Omega_c inconsistent with the force
        # direction actually sent to the vehicle.
        desired_acceleration = target_acceleration + formation_acceleration
        raw_desired_force_dot = (
            self.config.mass * (target_jerk + formation_acceleration_dot)
            - self.config.position_gain * velocity_error
            - self.config.velocity_gain * (current_acceleration - desired_acceleration)
            - self.config.integral_gain * saturated_integral_rate
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
        desired_yaw = target["yaw"]
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
        if omega_c_method in ("log_difference", "log", "so3_log"):
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
        body_rate_command = clamp_vector(
            body_rate_command, -self.config.max_body_rate, self.config.max_body_rate
        )

        # 在起飞阶段可施加最小总推力，克服地面静摩擦和小幅估计误差；其他阶段不使用
        # 该下限，特别是降落阶段必须允许推力降低。
        minimum_thrust = float(target.get("min_collective_thrust", 0.0))
        collective_thrust = float(np.clip(
            max(desired_force @ rotation[:, 2], minimum_thrust),
            0.0, self.config.max_command_thrust
        ))
        return {
            "position_error": position_error,
            "velocity_error": velocity_error,
            "desired_force": desired_force,
            "formation_acceleration": formation_acceleration,
            "formation_acceleration_dot": formation_acceleration_dot,
            "computed_rotation": computed_rotation,
            "computed_body_rate": computed_body_rate,
            "omega_c_method": omega_c_method,
            "attitude_error": attitude_error,
            "body_rate_command": body_rate_command,
            "collective_thrust": collective_thrust,
            # 诊断用：记录本周期积分器状态，便于区分推力标定误差和积分器被重置。
            "position_integral": self.position_integral.copy(),
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
        # Formation terms are new diagnostics and must remain at the end so
        # legacy CSV consumers keep their original column positions.
        "formation_acceleration_x", "formation_acceleration_y", "formation_acceleration_z",
        "formation_acceleration_dot_x", "formation_acceleration_dot_y", "formation_acceleration_dot_z",
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
            self.trajectory_config = CircularTrajectoryConfig(
                reference_hold_s=float(trajectory_param("reference_hold_s")),
                takeoff_height_m=float(trajectory_param("takeoff_height_m")),
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
        ))

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
        if not self.path_frame_id:
            raise rospy.ROSInitException("path_frame_id 不能为空")
        if self.path_publish_interval_s <= 0.0:
            raise rospy.ROSInitException("path_publish_interval_s 必须为正数")
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
        self.latest_raw_thrust = None
        self.latest_raw_thrust_received_time = None
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
        self.state_subscriber = rospy.Subscriber(
            self.prefix + "/mocap_state", MocapState, self._state_callback, queue_size=1
        )
        self.ekf_subscriber = rospy.Subscriber(
            self.prefix + "/ekf_kinematics", GenericLogData,
            self._ekf_state_callback, queue_size=1,
        )
        # hover_swarm.launch 配置 GenericLogData 的 values 顺序为
        # [pm.vbat(V), pm.vbatMV(mV), pm.state, pm.batteryLevel]；电池日志只在
        # 起飞前提供一次样本，控制器完成预检后不再依赖后续电池消息。
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

        电池日志只产生一次样本；控制器保留该样本并在预检阶段使用它。成功后
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
        """Require a continuously aligned EKF before the fleet may take off."""
        if not self._ekf_kinematics_enabled:
            return True
        if self.ekf_alignment_failed:
            return False
        if self.ekf_alignment_started_time is None:
            self.ekf_alignment_started_time = float(now)

        control_state = self._build_ekf_control_state(state, now)
        aligned = (
            control_state["ekf_state_valid"] and
            control_state["ekf_position_error_m"] <=
            self.ekf_alignment_position_delta_m
        )
        if aligned:
            if self.ekf_alignment_since is None:
                self.ekf_alignment_since = float(now)
            if now - self.ekf_alignment_since >= self.ekf_alignment_hold_s:
                return True
        else:
            self.ekf_alignment_since = None

        if now - self.ekf_alignment_started_time >= self.ekf_alignment_timeout_s:
            self._fail_ekf_alignment(
                "%.1f s 内未连续 %.1f s 对齐到 %.3f m（状态=%s，当前差=%.3f m）" % (
                    self.ekf_alignment_timeout_s,
                    self.ekf_alignment_hold_s,
                    self.ekf_alignment_position_delta_m,
                    control_state.get("ekf_status", "unknown"),
                    control_state.get("ekf_position_error_m", math.inf),
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

    def _timer_callback(
            self, _event, now=None, target_override=None,
            control_state_override=None, formation_acceleration=None,
            formation_acceleration_dot=None):
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

        target = (
            self._target_for(state, now)
            if target_override is None else target_override
        )
        control_state = (
            self._build_ekf_control_state(state, now)
            if control_state_override is None else control_state_override
        )
        if formation_acceleration is not None:
            target = dict(target)
            target["formation_acceleration"] = np.asarray(
                formation_acceleration, dtype=float
            ).reshape(3)
            target["formation_acceleration_dot"] = np.zeros(3) if (
                formation_acceleration_dot is None
            ) else np.asarray(formation_acceleration_dot, dtype=float).reshape(3)
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
                control_state.get("ekf_status", "unknown"),
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
            command = self.controller.compute(control_state, target, dt)
        if target["flight_phase"] == "landed" or target.get("zero_output", False):
            self._publish_zero()
        else:
            self._publish_command(command)
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
        self.command_publisher.publish(message)

    def _publish_zero(self):
        """发送一个显式零 RPYT/推力包，用于状态失效、落地和节点退出。"""
        message = CTBR()
        message.header.stamp = rospy.Time.now()
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
            "formation_acceleration": np.zeros(3),
            "formation_acceleration_dot": np.zeros(3),
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
        self._with_xyz(row, "position_error", command["position_error"])
        self._with_xyz(row, "velocity_error", command["velocity_error"])
        self._with_xyz(row, "desired_force", command["desired_force"])
        self._with_xyz(
            row, "formation_acceleration",
            command.get("formation_acceleration", np.zeros(3)),
        )
        self._with_xyz(
            row, "formation_acceleration_dot",
            command.get("formation_acceleration_dot", np.zeros(3)),
        )
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
        if self.rate_hz <= 0.0 or self.trajectory_pause_abort_s <= 0.0:
            self.logger.close()
            raise rospy.ROSInitException("多机控制频率和中止阈值必须为正数")

        # MATLAB formation/leader-follower gains are acceleration gains.  They
        # are applied only during the figure-eight phase; circle and single-CF
        # runs retain the existing independent geometric CTBR controller.
        self.formation_control_mode = str(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/formation_control_mode",
            "leader_follower",
        )).strip().lower().replace("-", "_")
        if self.formation_control_mode != "leader_follower":
            self.logger.close()
            raise rospy.ROSInitException(
                "formation_control_mode 当前仅支持 leader_follower"
            )
        self.formation_kf = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/formation_kf", 2.8
        ))
        self.formation_kvf = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/formation_kvf", 2.1
        ))
        self.formation_kbl = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/formation_kbl", 2.0
        ))
        self.formation_kvl = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/formation_kvl", 1.6
        ))
        self.formation_bl = np.asarray(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/formation_bl", [1.0, 1.0, 1.0]
        ), dtype=float).reshape(-1)
        self.formation_adjacency = np.asarray(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/formation_adjacency",
            [[0.0, 1.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 0.0]],
        ), dtype=float)
        if (not all(math.isfinite(value) and value >= 0.0 for value in (
                self.formation_kf, self.formation_kvf,
                self.formation_kbl, self.formation_kvl)) or
                self.formation_bl.size != len(self.vehicle_configs) or
                self.formation_adjacency.shape != (
                    len(self.vehicle_configs), len(self.vehicle_configs)) or
                not np.all(np.isfinite(self.formation_adjacency)) or
                np.any(self.formation_adjacency < 0.0)):
            self.logger.close()
            raise rospy.ROSInitException(
                "formation 参数维度、增益和邻接矩阵必须有效"
            )
        np.fill_diagonal(self.formation_adjacency, 0.0)

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
        self.mission_start_time = mission_start
        self.fleet_gate_open = False
        self.global_abort_reason = ""
        self.last_tick = mission_start
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "多机 CTBR 控制器已启动：%d 架飞机，共用一份日志 %s",
            len(self.vehicles), self.logger.path,
        )

    def _all_states_ready(self, now):
        return all(vehicle._state_ready(now) for vehicle in self.vehicles)

    def _all_preflight_ready(self, now):
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
        for vehicle in self.vehicles:
            vehicle.set_global_abort(self.global_abort_reason)
        rospy.logerr("多机 CTBR 全局中止：%s；所有飞机发送零推力", reason)

    def _formation_overrides(self, now):
        """Compute MATLAB formation acceleration corrections in ROS z-up.

        MATLAB uses z-down and computes ``u_i`` as a desired acceleration.  A
        sign flip on the z component maps it to ROS z-up; since the current
        reference has constant altitude, this is equivalent to using the
        already z-up center derivatives below.  The returned correction is
        ``u_i - a_c`` so the existing CTBR position PID remains available as a
        local stabilizing loop.
        """
        if (not self.multi_mode or len(self.vehicles) < 2 or
                self.formation_control_mode != "leader_follower"):
            return {}

        states = []
        controls = []
        targets = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._state_ready(now):
                return {}
            control_state = vehicle._build_ekf_control_state(state, now)
            if (vehicle._ekf_kinematics_enabled and
                    not control_state["ekf_state_valid"] and
                    not control_state["ekf_kinematics_held"]):
                return {}
            target = vehicle._target_for(state, now)
            if target.get("flight_phase") != "figure_eight":
                return {}
            states.append(state)
            controls.append(control_state)
            targets.append(target)

        count = len(self.vehicles)
        positions = np.vstack([state["position"] for state in states])
        velocities = np.vstack([
            control["control_velocity"] for control in controls
        ])
        accelerations = np.vstack([
            control["control_acceleration"] for control in controls
        ])
        references = np.vstack([target["position"] for target in targets])
        reference_velocity = np.vstack([target["velocity"] for target in targets])
        reference_acceleration = np.vstack([
            target["acceleration"] for target in targets
        ])
        reference_jerk = np.vstack([
            target.get("jerk", np.zeros(3)) for target in targets
        ])
        offsets = references - np.mean(references, axis=0)
        center_position = np.mean(references, axis=0)
        center_velocity = np.mean(reference_velocity, axis=0)
        center_acceleration = np.mean(reference_acceleration, axis=0)
        center_jerk = np.mean(reference_jerk, axis=0)
        formation_acceleration = np.zeros((count, 3))
        formation_acceleration_dot = np.zeros((count, 3))

        for i in range(count):
            relative_position_sum = np.zeros(3)
            relative_velocity_sum = np.zeros(3)
            acceleration_difference_sum = np.zeros(3)
            for j in range(count):
                weight = self.formation_adjacency[i, j]
                if j == i or weight == 0.0:
                    continue
                relative_position_sum += weight * (
                    (positions[i] - positions[j]) - (offsets[i] - offsets[j])
                )
                relative_velocity_sum += weight * (
                    velocities[i] - velocities[j]
                )
                acceleration_difference_sum += weight * (
                    accelerations[i] - accelerations[j]
                )

            error_to_reference = positions[i] - references[i]
            velocity_to_reference = velocities[i] - center_velocity
            desired_acceleration = (
                center_acceleration
                - self.formation_kf * relative_position_sum
                - self.formation_kvf * relative_velocity_sum
                - self.formation_kbl * self.formation_bl[i] * error_to_reference
                - self.formation_kvl * self.formation_bl[i] * velocity_to_reference
            )
            desired_acceleration_dot = (
                center_jerk
                - self.formation_kf * relative_velocity_sum
                - self.formation_kvf * acceleration_difference_sum
                - self.formation_kbl * self.formation_bl[i] * velocity_to_reference
                - self.formation_kvl * self.formation_bl[i] * (
                    accelerations[i] - center_acceleration
                )
            )

            formation_acceleration[i] = desired_acceleration - targets[i]["acceleration"]
            formation_acceleration_dot[i] = (
                desired_acceleration_dot - targets[i].get("jerk", np.zeros(3))
            )

        return {
            id(vehicle): {
                "target": targets[i],
                "control_state": controls[i],
                "formation_acceleration": formation_acceleration[i],
                "formation_acceleration_dot": formation_acceleration_dot[i],
            }
            for i, vehicle in enumerate(self.vehicles)
        }

    def _timer_callback(self, _event):
        now = time.monotonic()

        if self.global_abort_reason:
            for vehicle in self.vehicles:
                vehicle._timer_callback(None, now=now)
            self.logger.flush()
            return

        if not self.fleet_gate_open:
            for vehicle in self.vehicles:
                vehicle.set_fleet_gate(False)
                vehicle._timer_callback(None, now=now)
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
            self.logger.flush()
            return

        if not self._all_states_ready(now):
            invalid_ids = [
                "cf%d" % vehicle.vehicle_id
                for vehicle in self.vehicles
                if not vehicle._state_ready(now)
            ]
            self._set_abort("NOKOV 状态失效：%s" % ",".join(invalid_ids))

        formation_overrides = self._formation_overrides(now)
        for vehicle in self.vehicles:
            vehicle.set_fleet_gate(True)
            override = formation_overrides.get(id(vehicle), {})
            vehicle._timer_callback(
                None,
                now=now,
                target_override=override.get("target"),
                control_state_override=override.get("control_state"),
                formation_acceleration=override.get("formation_acceleration"),
                formation_acceleration_dot=override.get("formation_acceleration_dot"),
            )
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
