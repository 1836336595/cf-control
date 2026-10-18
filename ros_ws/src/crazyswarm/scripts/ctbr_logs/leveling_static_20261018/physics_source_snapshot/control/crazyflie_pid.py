"""Bounded Crazyflie legacy RPYT PID, X mixer, and selectable rotor dynamics.

The PID arithmetic and coordinate conventions follow the checked-in Crazyflie
firmware (controller_pid.c, attitude_pid_controller.c, pid.c, and
power_distribution_stock.c). The motor force curve, reaction-torque coefficient,
and force lag belong to the legacy model. The graefe2026 plant uses the paper's
identified per-rotor speed, thrust, aerodynamic torque and rotor inertia.
Inputs and returned torque use physical, right-handed body axes in radians.
"""

from __future__ import annotations

import numpy as np


class CrazyfliePidActuators:
    """One vehicle's fixed-rate onboard PID and four independent motors."""

    def __init__(self, config, vehicle, vehicle_index=0):
        self.config = config
        self.vehicle = vehicle
        self.vehicle_index = vehicle_index
        if config.motor_model not in ("graefe2026", "legacy_quadratic"):
            raise ValueError("motor_model must be graefe2026 or legacy_quadratic")
        for name in ("bridge_collective_pwm_reference", "bridge_collective_pwm_limit",
                     "bridge_thrust_curve_exponent", "motor_pwm_max"):
            value = float(getattr(config, name))
            if not np.isfinite(value) or value <= 0:
                raise ValueError(name + " must be positive and finite")
        if max(config.bridge_collective_pwm_reference, config.bridge_collective_pwm_limit) > config.motor_pwm_max:
            raise ValueError("collective PWM reference/limit must not exceed motor_pwm_max")
        for name in ("bridge_thrust_raw_scale", "motor_available_command_fraction"):
            value = np.asarray(getattr(config, name), dtype=float)
            if value.shape != (len(config.vehicles),) or not np.all(np.isfinite(value)) or np.any(value <= 0):
                raise ValueError(name + " must contain a positive value per vehicle")
        if np.any(np.asarray(config.motor_available_command_fraction) > 1):
            raise ValueError("motor_available_command_fraction must not exceed one")
        self.raw_scale = float(config.bridge_thrust_raw_scale[vehicle_index])
        self.motor_pwm_limit = int(np.floor(config.motor_pwm_max * config.motor_available_command_fraction[vehicle_index]))
        self.rotor_model = None
        if config.motor_model == "graefe2026":
            from .crazyflie_brushless import Graefe2026MotorModel
            self.rotor_model = Graefe2026MotorModel(
                time_constant=config.brushless_motor_time_constant_s,
                speed_gain=config.brushless_motor_speed_gain_radps,
                speed_normalization=config.brushless_speed_normalization_radps,
                rotor_inertia=config.brushless_rotor_inertia_kgm2,
                pwm_max=config.motor_pwm_max,
                thrust_coefficients=config.brushless_thrust_coefficients,
                torque_coefficients=config.brushless_torque_coefficients,
            )
        for name in ("onboard_pid_rate_hz", "thrust_time_constant", "arm_length"):
            value = float(getattr(config, name))
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        for name in (
            "onboard_pid_rate_kp", "onboard_pid_rate_ki", "onboard_pid_rate_kd",
            "onboard_pid_rate_integral_limit", "onboard_pid_yaw_gains",
        ):
            value = np.asarray(getattr(config, name), dtype=float)
            if value.shape != (3,) or not np.all(np.isfinite(value)) or np.any(value < 0):
                raise ValueError(f"{name} must contain three finite nonnegative values")
        efficiency = np.asarray(config.motor_thrust_efficiency, dtype=float)
        if (efficiency.shape != (len(config.vehicles),) or
                not np.all(np.isfinite(efficiency)) or np.any(efficiency <= 0)):
            raise ValueError("motor_thrust_efficiency must contain a positive value per vehicle")
        for name in ("motor_yaw_torque_coefficient", "onboard_pid_yaw_integral_limit"):
            value = float(getattr(config, name))
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be nonnegative and finite")
        self.pid_dt = 1.0 / config.onboard_pid_rate_hz
        self.efficiency = float(efficiency[vehicle_index])
        offset = config.arm_length / np.sqrt(2.0)
        # Firmware motor order, in physical right-handed body coordinates.
        self.motor_positions = offset * np.array([[1, -1], [-1, -1], [-1, 1], [1, 1]])
        self.reset()

    @staticmethod
    def _wrap_degrees(angle):
        return (float(angle) + 180.0) % 360.0 - 180.0

    def _reset_pid(self, yaw_rad):
        self.rate_integral = np.zeros(3)
        self._previous_rate_error = np.zeros(3)
        self._yaw_integral = 0.0
        self._previous_yaw_error = 0.0
        self.yaw_target_deg = self._wrap_degrees(np.rad2deg(yaw_rad))
        self.control_pwm = np.zeros(3, dtype=np.int64)

    def reset(self, yaw_rad=0.0, thrust=0.0):
        """Clear PID history and initialize the physical motor-force state."""
        if not np.isfinite(thrust) or thrust < 0 or not np.isfinite(yaw_rad):
            raise ValueError("initial thrust and yaw must be finite; thrust must be nonnegative")
        self._reset_pid(yaw_rad)
        self.time_until_update = 0.0
        self.applied_thrust_command = float(thrust)
        self.raw_collective = self._collective_pwm(thrust)
        self.motor_pwm = np.full(4, min(self.raw_collective, self.motor_pwm_limit), dtype=np.int64)
        if self.rotor_model is not None:
            self.rotor_model.reset(self.motor_pwm)
            self.motor_forces = self.efficiency * self.rotor_model.steady_thrust(self.motor_pwm)
            self.motor_reaction_torques = self.rotor_model.steady_reactive_torque(self.motor_pwm)
        else:
            self.motor_forces = self._target_motor_forces()
            self.motor_reaction_torques = self.config.motor_yaw_torque_coefficient * self.motor_forces

    def _collective_pwm(self, thrust):
        fraction = np.clip(thrust / self.vehicle.max_total_thrust, 0.0, 1.0)
        raw = int(np.floor(min(self.config.bridge_collective_pwm_limit,
                              self.config.bridge_collective_pwm_reference
                              * fraction ** (1 / self.config.bridge_thrust_curve_exponent)
                              * self.raw_scale) + 0.5))
        # crtp_commander_rpyt.c's decoded thrust is zero below MIN_THRUST.
        return raw if raw >= 1000 else 0

    def _target_motor_forces(self):
        return self.efficiency * self.vehicle.max_total_thrust / 4.0 * (
            self.motor_pwm.astype(float) / self.config.bridge_collective_pwm_reference
        ) ** 2

    @property
    def motor_speeds(self):
        return self.rotor_model.omega.copy() if self.rotor_model is not None else np.full(4, np.nan)

    def _advance_motors(self, interval):
        if self.rotor_model is not None:
            forces, reaction = self.rotor_model.advance(self.motor_pwm, interval)
            self.motor_forces = self.efficiency * forces
            self.motor_reaction_torques = reaction
        else:
            alpha = -np.expm1(-interval / self.config.thrust_time_constant)
            self.motor_forces += alpha * (self._target_motor_forces() - self.motor_forces)
            self.motor_reaction_torques = self.config.motor_yaw_torque_coefficient * self.motor_forces

    def _body_torque(self):
        return np.array([
            self.motor_positions[:, 1] @ self.motor_forces,
            -self.motor_positions[:, 0] @ self.motor_forces,
            -np.array([1.0, -1.0, 1.0, -1.0]) @ self.motor_reaction_torques,
        ])

    def _update_controller(self, thrust, commanded_rate, actual_rate, yaw_rad):
        self.applied_thrust_command = float(thrust)
        self.raw_collective = self._collective_pwm(thrust)
        if self.raw_collective == 0:
            self._reset_pid(yaw_rad)
            self.motor_pwm[:] = 0
            return
        # Host bridge: wire=(p,-q,-r). The legacy decoder negates yaw, so
        # attitudeRate=(p,-q,r); rate PID feedback=(gyro.x,-gyro.y,gyro.z).
        desired_rate = np.rad2deg(commanded_rate) * [1.0, -1.0, 1.0]
        actual_rate = np.rad2deg(actual_rate) * [1.0, -1.0, 1.0]
        self.yaw_target_deg = self._wrap_degrees(
            self.yaw_target_deg + desired_rate[2] * self.pid_dt
        )
        yaw_error = self._wrap_degrees(self.yaw_target_deg - np.rad2deg(yaw_rad))
        yaw_limit = self.config.onboard_pid_yaw_integral_limit
        self._yaw_integral += yaw_error * self.pid_dt
        if yaw_limit:
            self._yaw_integral = float(np.clip(self._yaw_integral, -yaw_limit, yaw_limit))
        yaw_derivative = (yaw_error - self._previous_yaw_error) / self.pid_dt
        self._previous_yaw_error = yaw_error
        yaw_kp, yaw_ki, yaw_kd = self.config.onboard_pid_yaw_gains
        desired_rate[2] = (yaw_kp * yaw_error + yaw_ki * self._yaw_integral +
                           yaw_kd * yaw_derivative)
        error = desired_rate - actual_rate
        derivative = (error - self._previous_rate_error) / self.pid_dt
        self._previous_rate_error = error.copy()
        self.rate_integral += error * self.pid_dt
        limits = np.asarray(self.config.onboard_pid_rate_integral_limit)
        self.rate_integral = np.where(
            limits > 0, np.clip(self.rate_integral, -limits, limits), self.rate_integral
        )
        output = (np.asarray(self.config.onboard_pid_rate_kp) * error +
                  np.asarray(self.config.onboard_pid_rate_ki) * self.rate_integral +
                  np.asarray(self.config.onboard_pid_rate_kd) * derivative)
        # saturateSignedInt16 uses +/-32767 before truncation to int16.
        self.control_pwm = np.trunc(np.clip(output, -32767, 32767)).astype(np.int64)
        roll = int(self.control_pwm[0] / 2)
        pitch = int(self.control_pwm[1] / 2)
        yaw = -self.control_pwm[2]
        self.motor_pwm = np.clip(self.raw_collective + np.array([
            -roll + pitch + yaw, -roll - pitch - yaw,
            roll - pitch + yaw, roll + pitch - yaw,
        ]), 0, self.motor_pwm_limit)

    def step(self, thrust, commanded_rate, actual_rate, yaw_rad, dt):
        """Advance motors while sampling PID at its fixed onboard clock.

        Physics callers should also stop at ``time_until_update`` to refresh the
        measured body rate at each PID deadline. Held measurements are supported
        for callers advancing a larger interval.
        """
        commanded_rate = np.asarray(commanded_rate, dtype=float).reshape(3)
        actual_rate = np.asarray(actual_rate, dtype=float).reshape(3)
        if (not np.isfinite(dt) or dt <= 0 or not np.isfinite(thrust) or
                not np.isfinite(yaw_rad) or not np.all(np.isfinite(commanded_rate)) or
                not np.all(np.isfinite(actual_rate))):
            raise ValueError("actuator inputs must be finite and dt must be positive")
        remaining = float(dt)
        while remaining > 1.0e-12:
            if self.time_until_update <= 1.0e-12:
                self._update_controller(max(float(thrust), 0.0), commanded_rate, actual_rate, yaw_rad)
                self.time_until_update = self.pid_dt
            interval = min(remaining, self.time_until_update)
            self._advance_motors(interval)
            self.time_until_update = max(0.0, self.time_until_update - interval)
            remaining -= interval
        return float(np.sum(self.motor_forces)), self._body_torque()


class CrazyflieMellingerActuators(CrazyfliePidActuators):
    """Firmware Mellinger attitude/rate path followed by the X motor mixer.

    The incoming CTBR rate is interpreted in physical body axes. Mellinger's
    RATE decoder still levels roll and pitch, while its rate error contributes
    damping/feed-forward. The large firmware gains are in the signed mixer
    command units and are therefore clipped before motor mixing.
    """

    def __init__(self, config, vehicle, vehicle_index=0):
        super().__init__(config, vehicle, vehicle_index)
        value = float(config.mellinger_rate_hz)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError("mellinger_rate_hz must be positive and finite")
        self.mellinger_dt = 1.0 / value
        self._mellinger_time = 0.0
        self._previous_rate_error = np.zeros(3)
        self._mellinger_yaw_integral = 0.0

    def reset(self, yaw_rad=0.0, thrust=0.0):
        super().reset(yaw_rad=yaw_rad, thrust=thrust)
        self._mellinger_time = 0.0
        self._previous_rate_error = np.zeros(3)
        self._mellinger_yaw_integral = 0.0

    @staticmethod
    def _rotation_error(rotation, yaw_target):
        desired = np.array([
            [np.cos(yaw_target), -np.sin(yaw_target), 0.0],
            [np.sin(yaw_target), np.cos(yaw_target), 0.0],
            [0.0, 0.0, 1.0],
        ])
        skew = desired.T @ rotation - rotation.T @ desired
        # controller_mellinger.c's fast formula uses the full skew error.
        # The host geometric controller's additional 1/2 is not used here.
        result = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
        # Firmware's legacy pitch coordinate is inverted before moment output.
        result[1] = -result[1]
        return result

    def _mellinger_control(self, thrust, commanded_rate, actual_rate, yaw_rad, rotation):
        self.applied_thrust_command = float(thrust)
        self.raw_collective = self._collective_pwm(thrust)
        if self.raw_collective == 0:
            self._reset_pid(yaw_rad)
            self.motor_pwm[:] = 0
            self.control_pwm[:] = 0
            self._mellinger_yaw_integral = 0.0
            return
        dt = self.mellinger_dt
        commanded_rate = np.asarray(commanded_rate, dtype=float)
        actual_rate = np.asarray(actual_rate, dtype=float)
        self.yaw_target_deg = self._wrap_degrees(
            np.rad2deg(yaw_rad + commanded_rate[2] * dt)
        )
        # Mellinger bridge wire=(p,q,-r), decoder attitudeRate=(p,q,r).
        # Firmware ew=(p_cmd-p, -q_cmd+q, r_cmd-r); control.pitch is a
        # legacy mixer coordinate, so the physical pitch sign reverses twice.
        rate_error = (commanded_rate - actual_rate) * [1.0, -1.0, 1.0]
        rate_derivative = (rate_error - self._previous_rate_error) / dt
        self._previous_rate_error = rate_error.copy()
        attitude_error = self._rotation_error(rotation, np.deg2rad(self.yaw_target_deg))
        self._mellinger_yaw_integral += -attitude_error[2] * dt
        self._mellinger_yaw_integral = float(np.clip(
            self._mellinger_yaw_integral,
            -self.config.mellinger_yaw_integral_limit,
            self.config.mellinger_yaw_integral_limit,
        ))
        k_r = np.asarray(self.config.mellinger_attitude_gains)
        k_w = np.asarray(self.config.mellinger_rate_gains)
        moment = -k_r * attitude_error + k_w * rate_error
        moment[:2] += self.config.mellinger_rate_derivative_gain * rate_derivative[:2]
        moment[2] += self.config.mellinger_yaw_integral_gain * self._mellinger_yaw_integral
        self.control_pwm = np.trunc(np.clip(
            np.array([moment[0], moment[1], -moment[2]]), -32000, 32000
        )).astype(np.int64)
        roll = int(self.control_pwm[0] / 2)
        pitch = int(self.control_pwm[1] / 2)
        yaw = int(self.control_pwm[2])
        self.motor_pwm = np.clip(self.raw_collective + np.array([
            -roll + pitch + yaw, -roll - pitch - yaw,
            roll - pitch + yaw, roll + pitch - yaw,
        ]), 0, self.motor_pwm_limit)

    def step(self, thrust, commanded_rate, actual_rate, yaw_rad, rotation, dt):
        rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
        commanded_rate = np.asarray(commanded_rate, dtype=float).reshape(3)
        actual_rate = np.asarray(actual_rate, dtype=float).reshape(3)
        if (not np.isfinite(dt) or dt <= 0 or not np.isfinite(thrust)
                or not np.isfinite(yaw_rad) or not np.all(np.isfinite(rotation))
                or not np.all(np.isfinite(commanded_rate))
                or not np.all(np.isfinite(actual_rate))):
            raise ValueError("actuator inputs must be finite and dt must be positive")
        remaining = float(dt)
        while remaining > 1.0e-12:
            if self.time_until_update <= 1.0e-12:
                self._mellinger_control(
                    max(float(thrust), 0.0), commanded_rate, actual_rate, yaw_rad, rotation
                )
                self.time_until_update = self.mellinger_dt
            interval = min(remaining, self.time_until_update)
            self._advance_motors(interval)
            self.time_until_update = max(0.0, self.time_until_update - interval)
            remaining -= interval
        return float(np.sum(self.motor_forces)), self._body_torque()
