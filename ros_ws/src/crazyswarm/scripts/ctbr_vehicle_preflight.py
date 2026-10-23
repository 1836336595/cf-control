"""Per-aircraft zero-thrust voltage and EKF preflight checks."""

from ctbr_control_math import rotation_to_rpy
import math
import numpy as np
import rospy


class VehiclePreflightMixin:
    """Per-aircraft zero-thrust voltage and EKF preflight checks."""

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
