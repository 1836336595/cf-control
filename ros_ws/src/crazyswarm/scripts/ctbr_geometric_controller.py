"""ROS-independent geometric CTBR facade with independent/transport formulas."""

import math
from dataclasses import dataclass

import numpy as np

from ctbr_control_math import (clamp_vector, vee, project_so3, so3_log,
                               _limit_force_with_derivative, analytic_omega_c)
from ctbr_independent_controller import IndependentVehicleController
from ctbr_transport_vehicle_controller import TransportVehicleController


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


class GeometricCtbrController(IndependentVehicleController, TransportVehicleController):
    """Preserve the public controller API while composing pure aircraft laws."""

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
        position_integral_rate, saturated_integral_rate = self._update_position_integral(
            position_error, velocity_error, dt, transport_mode, independent_mode,
            vertical_integral_active)

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

        independent_position_gain, independent_velocity_gain, independent_integral_gain = (
            self._independent_gains(target))
        raw_desired_force = None
        if independent_mode:
            raw_desired_force = self._independent_force(
                state, target, dt, derivatives_ready, position_error, velocity_error,
                target_acceleration, independent_position_gain, independent_velocity_gain,
                independent_integral_gain, prelift_vertical_integral,
                landing_vertical_integral, vertical_integral_active)
            transport_blend = 0.0
        force = self._compose_vehicle_force(
            state, target, dt, derivatives_ready,
            measured_velocity if derivatives_ready else np.zeros(3), current_acceleration,
            position_error, velocity_error, target_acceleration, target_jerk,
            position_gain, velocity_gain, independent_position_gain,
            independent_velocity_gain, independent_integral_gain, saturated_integral_rate,
            independent_mode, transport_mode, native_transport_pid, transport_blend,
            raw_desired_force)
        # 先限制竖直推力，再依照最大倾角限制水平合力。水平力上限依赖当前可用的
        # 竖直力，确保大位置误差不会要求接近 90 deg 的危险倾斜；同时求该限幅的导数，
        # 使解析 Omega_c 与实际用于构造姿态的合力方向一致。
        max_tilt_rad = float(target.get("max_tilt_rad", self.config.max_tilt_rad))
        desired_force, desired_force_dot = _limit_force_with_derivative(
            force.raw_desired_force,
            force.raw_desired_force_dot,
            self.config.max_command_thrust,
            max_tilt_rad,
        )
        # Check the limiter itself, before the intentional force filter lag.
        force_limit_active = not np.allclose(
            force.raw_desired_force, desired_force, rtol=0.0, atol=1.0e-10)
        desired_force, desired_force_dot = self._filter_transport_force(
            desired_force, desired_force_dot, target, transport_mode, dt)

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
            body_rate_command = self._independent_body_rate(attitude_error)
        elif transport_mode:
            body_rate_command = self._transport_body_rate(
                state, target, rotation, computed_rotation, computed_body_rate,
                attitude_error, force.transport_blend)
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
        raw_body_rate_command, body_rate_command, body_rate_limit_active = self._limit_body_rate(
            body_rate_command, target, independent_mode, transport_mode, dt)

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
            "raw_desired_force": force.raw_desired_force,
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
            "hold_feedback_limit_active": force.hold_feedback_limit_active,
            "body_rate_limit_active": body_rate_limit_active,
            "collective_limit_active": collective_limit_active,
            "transport_hold_integral_force": force.transport_blend * force.transport_integral_force,
            "transport_hold_integral_force_dot": (
                force.transport_blend * force.transport_integral_force_dot
                + (force.transport_blend_dot if math.isfinite(force.transport_blend_dot) else 0.0)
                * force.transport_integral_force),
            "transport_full_pid_force": force.transport_blend * force.transport_full_pid_force,
            "transport_full_pid_force_dot": (
                force.transport_blend * force.transport_full_pid_force_dot
                + (force.transport_blend_dot if math.isfinite(force.transport_blend_dot) else 0.) * force.transport_full_pid_force),
            "transport_full_pid_pd_force": force.transport_blend * force.transport_full_pid_pd_force,
            "transport_full_pid_pd_force_dot": (
                force.transport_blend * force.transport_full_pid_pd_force_dot
                + (force.transport_blend_dot if math.isfinite(force.transport_blend_dot) else 0.) * force.transport_full_pid_pd_force),
            "transport_full_pid_feedback_force": (
                force.transport_blend * (force.transport_full_pid_pd_force + force.transport_full_pid_force)),
            "transport_full_pid_feedback_force_dot": (
                force.transport_blend * (force.transport_full_pid_pd_force_dot + force.transport_full_pid_force_dot)
                + (force.transport_blend_dot if math.isfinite(force.transport_blend_dot) else 0.)
                * (force.transport_full_pid_pd_force + force.transport_full_pid_force)),
            "transport_full_pid_native_force": (
                force.transport_blend * force.native_pid_info["native_pid_force"]
                if force.native_pid_info is not None else np.zeros(3)),
            "transport_full_pid_info": force.native_pid_info,
            "transport_full_pid_integral_state": self.transport_position_integral.copy(),
            "transport_full_pid_integral_state_delta": (
                force.native_pid_info["integral_state_delta"] if force.native_pid_info is not None else np.zeros(3)),
            "transport_full_pid_integral_state_limit": (
                force.native_pid_info["integral_state_limit"] if force.native_pid_info is not None else np.zeros(3)),
            "transport_full_pid_integral_gain": (
                force.native_pid_info["integral_gain"] if force.native_pid_info is not None else np.zeros(3)),
            "transport_full_pid_integral_velocity_weight": (
                force.native_pid_info["integral_velocity_weight"] if force.native_pid_info is not None else 0.),
            "transport_full_pid_learning": bool(force.native_pid_info and force.native_pid_info["learning"]),
            "transport_full_pid_freeze_reason": (
                force.native_pid_info["freeze_reason"] if force.native_pid_info is not None else "inactive"),
            "transport_full_pid_duplicate_tick": bool(force.native_pid_info and force.native_pid_info["duplicate_tick"]),
            "transport_full_pid_integral_gate_m": (
                force.native_pid_info["integral_gate_m"] if force.native_pid_info is not None else 0.),
            "transport_full_pid_gate_open": bool(force.native_pid_info and force.native_pid_info["native_gate_open"]),
            "transport_full_pid_integral_limit_active": bool(force.native_pid_info and force.native_pid_info["integral_state_limit_active"]),
            "transport_full_pid_feedback_limit_active": bool(force.native_pid_info and force.native_pid_info["feedback_clamp_active"]),
            "transport_full_pid_force_budget_limited": bool(force.native_pid_info and force.native_pid_info["force_budget_limited"]),
            "transport_full_pid_rate_budget_limited": bool(force.native_pid_info and force.native_pid_info["rate_budget_limited"]),
            "transport_position_hold_force": (
                force.transport_blend * (force.transport_full_pid_pd_force
                                   if force.native_pid_info is not None else self.config.mass * force.hold_acceleration)
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

