"""Shared native position feedback; all gains are acceleration gains."""
from numbers import Real

import numpy as np


def validate_integral_velocity_weight(velocity_weight):
    """Return a finite nonnegative real scalar, including zero."""
    message = "integral velocity_weight must be a finite nonnegative scalar"
    if isinstance(velocity_weight, (bool, np.bool_)) or not isinstance(velocity_weight, Real):
        raise ValueError(message)
    try:
        velocity_weight = float(velocity_weight)
    except (OverflowError, TypeError, ValueError) as error:
        raise ValueError(message) from error
    if not np.isfinite(velocity_weight) or velocity_weight < 0.:
        raise ValueError(message)
    return velocity_weight


def position_integral_step(integral, position_error, velocity_error, dt,
                           gate, limit, learning=True, velocity_weight=1.0):
    """Integrate weighted velocity plus 0.5 position error within gate/limits."""
    velocity_weight = validate_integral_velocity_weight(velocity_weight)
    previous = np.asarray(integral, dtype=float)
    gate_open = bool(np.linalg.norm(position_error) < gate)
    updated = previous.copy()
    if learning and gate_open:
        updated = np.clip(previous + dt * (velocity_weight * velocity_error + .5 * position_error),
                          -np.asarray(limit), np.asarray(limit))
    return updated, (updated - previous) / dt, gate_open


def position_pid_feedback(position_error, velocity_error, position_gain,
                          velocity_gain, integral_gain, integral, limit,
                          acceleration_error=None, integral_rate=None,
                          gain_scale=1., gain_scale_dot=0.):
    """Return clamped native PID acceleration and its consistent derivative."""
    base_pd = -position_gain * position_error - velocity_gain * velocity_error
    unclamped = gain_scale * base_pd - integral_gain * integral
    feedback = np.clip(unclamped, -limit, limit)
    acceleration_error = (np.zeros(3) if acceleration_error is None
                          else acceleration_error)
    integral_rate = np.zeros(3) if integral_rate is None else integral_rate
    derivative = gain_scale * (-position_gain * velocity_error
                              -velocity_gain * acceleration_error)
    derivative += gain_scale_dot * base_pd - integral_gain * integral_rate
    derivative = np.where(np.abs(unclamped) < limit, derivative, 0.)
    return feedback, derivative, unclamped
