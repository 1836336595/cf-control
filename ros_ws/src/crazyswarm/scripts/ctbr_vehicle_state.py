"""Per-aircraft mocap, EKF, battery and bridge state reception."""

from ctbr_control_math import quaternion_to_rotation
from ctbr_state_feedback import blend_kinematic_feedback
import math
import numpy as np
import rospy
import time


class VehicleStateMixin:
    """Per-aircraft mocap, EKF, battery and bridge state reception."""

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
