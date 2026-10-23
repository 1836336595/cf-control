"""Independent aircraft position/integral and attitude control.

Methods share the facade's integral state so startup, prelift and landing keep
exactly the same trim ownership through cooperative handoffs.
"""

import math

import numpy as np

from ctbr_control_math import clamp_vector
from native_position_pid import position_integral_step, position_pid_feedback


class IndependentVehicleController:
    """Pure independent-flight formulas mixed into the geometric facade."""

    def _update_position_integral(self, position_error, velocity_error, dt,
                                  transport_mode, independent_mode,
                                  vertical_integral_active):
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
        return position_integral_rate, saturated_integral_rate

    def _independent_gains(self, target):
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
        return independent_position_gain, independent_velocity_gain, independent_integral_gain

    def _independent_force(self, state, target, dt, derivatives_ready,
                           position_error, velocity_error, target_acceleration,
                           independent_position_gain, independent_velocity_gain,
                           independent_integral_gain, prelift_vertical_integral,
                           landing_vertical_integral, vertical_integral_active):
        max_feedback = getattr(self.config, "independent_max_feedback_acceleration", None)
        max_feedback = (np.full(3, np.inf) if max_feedback is None
                        else np.asarray(max_feedback, dtype=float).reshape(3).copy())
        if vertical_integral_active:
            vertical_prefix = "prelift" if prelift_vertical_integral else "landing"
            max_feedback[2] = float(target[vertical_prefix + "_vertical_feedback_limit_mps2"])
            learning = (not prelift_vertical_integral or bool(
                target.get("prelift_vertical_integral_learning", True)))
            if derivatives_ready and learning and abs(position_error[2]) <= float(
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
        return raw_desired_force

    def _independent_body_rate(self, attitude_error):
        independent_attitude_gain = np.asarray(
            getattr(self.config, "independent_attitude_gain", None)
            if getattr(self.config, "independent_attitude_gain", None) is not None
            else self.config.attitude_gain,
            dtype=float,
        ).reshape(3)
        body_rate_command = -independent_attitude_gain * attitude_error
        return body_rate_command

