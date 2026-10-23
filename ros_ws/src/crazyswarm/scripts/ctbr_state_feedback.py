"""Causal velocity filtering and EKF feedback selection without ROS I/O."""

from ctbr_control_math import as_vector
import math
import numpy as np


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
