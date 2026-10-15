"""ROS-independent physical handoff checks and bounded support integration."""

import math

import numpy as np


class SlungTransportGuard:
    def __init__(self, min_distance_m=0.30, prediction_s=0.35,
                 height_error_m=0.08, confirm_s=0.20):
        values = (min_distance_m, prediction_s, height_error_m, confirm_s)
        if any(not math.isfinite(float(v)) or float(v) <= 0 for v in values):
            raise ValueError("transport guard limits must be positive and finite")
        self.min_distance_m = float(min_distance_m)
        self.prediction_s = float(prediction_s)
        self.height_error_m = float(height_error_m)
        self.confirm_s = float(confirm_s)
        self.reset()

    def reset(self):
        self._since = {}

    def _persistent(self, name, active, now):
        if not active:
            self._since.pop(name, None)
            return False
        self._since.setdefault(name, float(now))
        return float(now) - self._since[name] >= self.confirm_s - 1e-12

    def check(self, now, positions, velocities, height_error=None,
              rate_saturated=False):
        positions = np.asarray(positions, dtype=float).reshape(3, 3)
        velocities = np.asarray(velocities, dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities)):
            return dict(reason="nonfinite_geometry", minimum_distance_m=float("nan"),
                        predicted_distance_m=float("nan"))
        minimum, predicted = float("inf"), float("inf")
        for a, b in ((0, 1), (0, 2), (1, 2)):
            relative = positions[a] - positions[b]
            rate = velocities[a] - velocities[b]
            minimum = min(minimum, float(np.linalg.norm(relative)))
            speed_sq = float(rate @ rate)
            closest_time = (float(np.clip(-relative @ rate / speed_sq, 0, self.prediction_s))
                            if speed_sq > 1e-12 else 0.0)
            predicted = min(predicted, float(np.linalg.norm(relative + closest_time * rate)))
        # An actual boundary violation is immediate. Prediction and noisy
        # height/rate signals require persistence, each with its own timer.
        predicted_fault = self._persistent("distance", predicted < self.min_distance_m, now)
        height_fault = self._persistent("height_drop", height_error is not None
                                        and float(height_error) > self.height_error_m, now)
        rate_fault = self._persistent("rate_saturation", bool(rate_saturated), now)
        reason = ("distance" if minimum < self.min_distance_m or predicted_fault else
                  "height_drop" if height_fault else "rate_saturation" if rate_fault else "")
        return dict(reason=reason, minimum_distance_m=minimum, predicted_distance_m=predicted)


class SlungSupportTrim:
    """Carry only the prelift/model force residual, then adjust via bounded I."""
    def __init__(self, limit_n=0.25, integral_gain=0.20, rate_limit_nps=0.03):
        if any(not math.isfinite(float(v)) or float(v) <= 0
               for v in (limit_n, integral_gain, rate_limit_nps)):
            raise ValueError("support trim limits must be positive and finite")
        self.limit_n = float(limit_n)
        self.integral_gain = float(integral_gain)
        self.rate_limit_nps = float(rate_limit_nps)
        self.reset()

    def reset(self):
        self.bias = np.zeros(3)
        self.initialized = False

    def initialize(self, handoff_z, model_z):
        difference = np.asarray(handoff_z, dtype=float) - np.asarray(model_z, dtype=float)
        difference = difference.reshape(3)
        if not np.all(np.isfinite(difference)):
            raise ValueError("support trim forces must be finite")
        self.bias = np.clip(difference, 0.0, self.limit_n)
        self.initialized = True
        return self.bias.copy()

    def update(self, height_error, dt, thrust_saturated=None):
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("support trim dt must be positive and finite")
        if not self.initialized:
            return self.bias.copy(), np.zeros(3)
        if not math.isfinite(float(height_error)):
            raise ValueError("support trim height error must be finite")
        derivative = np.full(3, np.clip(self.integral_gain * float(height_error),
                                       -self.rate_limit_nps, self.rate_limit_nps))
        if thrust_saturated is not None:
            # Do not wind up while an actuator cannot supply more force.
            derivative[(np.asarray(thrust_saturated, dtype=bool)) & (derivative > 0)] = 0.0
        previous = self.bias.copy()
        self.bias = np.clip(previous + derivative * dt, 0.0, self.limit_n)
        return self.bias.copy(), (self.bias - previous) / dt
