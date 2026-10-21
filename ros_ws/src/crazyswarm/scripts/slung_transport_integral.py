"""ROS-independent differential force integral for three-vehicle transport."""

import math

import numpy as np


class SlungDifferentialHoldIntegral:
    """Integrate relative position error without adding common fleet force.

    Rows are CF3, CF4 and CF5; columns are world-frame XYZ. Position error
    uses measured minus reference position, so positive error learns a
    negative force bias. Callers decide when transport learning is safe.
    """

    def __init__(self, gain_npm_s=0.02, limit_n=0.03, rate_limit_nps=0.002):
        self.gain_npm_s = self._positive_scalar(gain_npm_s, "gain_npm_s")
        self.limit_n = self._positive_scalar(limit_n, "limit_n")
        self.rate_limit_nps = self._positive_scalar(rate_limit_nps, "rate_limit_nps")
        self.reset()

    @staticmethod
    def _positive_scalar(value, name):
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError):
            raise ValueError(name + " must be positive and finite") from None
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(name + " must be positive and finite")
        return value

    def reset(self):
        self.bias = np.zeros((3, 3))
        self.differential_error_m = np.zeros((3, 3))

    @staticmethod
    def _bound_fleet_norm(values, limit):
        maximum_norm = float(np.max(np.hypot.reduce(values, axis=1)))
        if maximum_norm > limit:
            values *= limit / maximum_norm
        return values

    def update(self, position_errors, dt, learning=True):
        """Return independent (bias N, actual bias rate N/s) fleet arrays.

        Freezing retains the learned bias and returns a zero rate. A frozen
        call may omit errors with ``None``; the last error snapshot is then
        retained. Any provided errors and every dt are still validated.
        """
        dt = self._positive_scalar(dt, "dt")
        if position_errors is not None:
            try:
                errors = np.asarray(position_errors, dtype=float)
            except (TypeError, ValueError, OverflowError):
                raise ValueError("position_errors must be a finite 3-by-3 array") from None
            if errors.shape != (3, 3) or not np.all(np.isfinite(errors)):
                raise ValueError("position_errors must be a finite 3-by-3 array")
            self.differential_error_m = errors - np.mean(errors, axis=0)
        elif learning:
            raise ValueError("position_errors are required while learning")
        if not learning:
            return self.bias.copy(), np.zeros((3, 3))
        rate = self._bound_fleet_norm(
            -self.gain_npm_s * self.differential_error_m, self.rate_limit_nps)
        proposed = self._bound_fleet_norm(self.bias + dt * rate, self.limit_n)
        # Joint projection can move a row faster than the requested rate.
        # A common scale of the increment keeps the zero sum and, by
        # convexity, stays inside the force bound as well as the rate bound.
        increment = self._bound_fleet_norm(proposed - self.bias, dt * self.rate_limit_nps)
        self.bias = self.bias + increment
        return self.bias.copy(), increment / dt
