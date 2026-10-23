"""ROS-independent, bounded payload attitude integral in physical moment units."""

import numpy as np

from slung_reference import _rotation_matrix, _rotation_vector


class SlungAttitudeMomentIntegral:
    """Store a world-frame disturbance moment without rotating accumulated bias.

    The logarithmic attitude error and diagonal gain use desired-body axes.
    ``Bdot_W = -Rd @ (Ki * log(Rd.T @ R))`` and the applied body moment is
    ``weight * R.T @ B_W``. The integral stores the accepted, norm-bounded
    moment itself, so saturation has no hidden accumulator to unwind.

    A gate holds learned state. Disabling or ``reset()`` clears it. Turning
    yaw off also removes its stored component along the current measured
    body z axis, preventing hidden yaw bias from returning on re-enable.
    ``tick`` makes repeated fleet calls idempotent. An optional ``accept``
    callback receives candidate and previous world moments and can return
    a rejection reason, or False, to reject only that learning increment.
    """

    PREFIX = "payload_attitude_integral_"

    def __init__(self, gain_nm_per_rad_s=(5.e-5, 5.e-5, 2.e-5),
                 limit_nm=.0015, rate_limit_nmps=.00015):
        self.configure(gain_nm_per_rad_s, limit_nm, rate_limit_nmps)
        self.reset()

    @staticmethod
    def _numeric(value, name):
        try:
            original = np.asarray(value, dtype=object)
            if any(isinstance(item, (bool, np.bool_, str, bytes))
                   for item in original.flat):
                raise ValueError(name + " must contain numbers")
            numeric = np.asarray(value, dtype=float)
            if not np.all(np.isfinite(numeric)):
                raise ValueError(name + " must contain finite numbers")
            return numeric
        except (TypeError, OverflowError):
            raise ValueError(name + " must contain finite numbers") from None

    def configure(self, gain_nm_per_rad_s, limit_nm, rate_limit_nmps):
        """Validate physical limits and preserve the bounded learned moment."""
        gains = self._numeric(gain_nm_per_rad_s, "gain_nm_per_rad_s")
        limit = self._numeric(limit_nm, "limit_nm")
        rate_limit = self._numeric(rate_limit_nmps, "rate_limit_nmps")
        if gains.shape != (3,) or np.any(gains < 0.):
            raise ValueError("gain_nm_per_rad_s must be three nonnegative values")
        if limit.shape != () or float(limit) <= 0.:
            raise ValueError("limit_nm must be a positive finite scalar")
        if rate_limit.shape != () or float(rate_limit) <= 0.:
            raise ValueError("rate_limit_nmps must be a positive finite scalar")
        self.gain_nm_per_rad_s = gains.copy()
        self.limit_nm = float(limit)
        self.rate_limit_nmps = float(rate_limit)
        if hasattr(self, "_world_moment"):
            magnitude = float(np.hypot.reduce(self._world_moment))
            if magnitude > self.limit_nm:
                self._world_moment *= self.limit_nm / magnitude

    def reset(self):
        self._world_moment = np.zeros(3)
        self._error_goal = np.zeros(3)
        self._last_tick = None
        self._cached = None

    @property
    def world_moment(self):
        return self._world_moment.copy()

    @staticmethod
    def _copy(snapshot):
        return {key: value.copy() if isinstance(value, np.ndarray) else value
                for key, value in snapshot.items()}

    def _snapshot(self, rotation, enabled, learning, reason, yaw_enabled,
                  weight, weight_dot, world_rate, limit_active=False,
                  rate_active=False, allocation_limited=False):
        body = weight * (rotation.T @ self._world_moment)
        if not yaw_enabled:
            body[2] = 0.
        values = dict(enabled=bool(enabled), learning=bool(learning),
                      freeze_reason=str(reason), world_moment=self.world_moment,
                      body_moment=body, world_moment_dot=(
                          weight * world_rate + weight_dot * self._world_moment),
                      error_goal=self._error_goal.copy(),
                      limit_active=bool(limit_active),
                      rate_limit_active=bool(rate_active),
                      allocation_limited=bool(allocation_limited),
                      yaw_enabled=bool(yaw_enabled))
        return {self.PREFIX + key: value for key, value in values.items()}

    def update(self, measured_rotation, desired_rotation, dt, *, enabled=True,
               learning=True, freeze_reason="", tick=None, yaw_enabled=True,
               weight=1., weight_dot=0., accept=None):
        """Advance at most once per tick and return copies of diagnostics.

        Reported world derivative includes the actual accepted increment and
        unload weight derivative. Vehicle force derivatives are obtained from
        the controller's history of final forces, which includes this moment.
        Invalid measurements freeze without changing the finite stored state.
        """
        if not enabled:
            self.reset()
            return self._snapshot(np.eye(3), False, False, "disabled",
                                  yaw_enabled, 0., 0., np.zeros(3))
        duplicate = tick is not None and tick == self._last_tick
        if duplicate and learning:
            learning, freeze_reason = False, "duplicate_tick"
        valid = True
        try:
            rotation = _rotation_matrix(measured_rotation, "measured_rotation")
            desired = _rotation_matrix(desired_rotation, "desired_rotation")
            dt = float(dt)
            weight = float(weight)
            weight_dot = float(weight_dot)
            if (not np.isfinite(dt) or dt <= 0. or not np.isfinite(weight)
                    or not 0. <= weight <= 1. or not np.isfinite(weight_dot)):
                raise ValueError("invalid integral update")
        except (TypeError, ValueError, OverflowError):
            valid = False
            rotation = np.eye(3)
            weight, weight_dot, dt = 0., 0., 1.
            learning, freeze_reason = False, "invalid_state"

        previous = self._world_moment.copy()
        if valid:
            self._error_goal = _rotation_vector(desired.T @ rotation)
            if not yaw_enabled:
                # Remove stored yaw immediately, even with learning frozen.
                self._world_moment -= rotation[:, 2] * float(
                    rotation[:, 2] @ self._world_moment)
        baseline = self._world_moment.copy()
        limit_active = rate_active = allocation_limited = False
        eligible = bool(learning) and valid
        reason = "" if eligible else (str(freeze_reason) or "gated")
        if eligible:
            error = self._error_goal.copy()
            if not yaw_enabled:
                error[2] = 0.
            with np.errstate(over="ignore", invalid="ignore"):
                increment = -desired @ (self.gain_nm_per_rad_s * error) * dt
            if not yaw_enabled:
                increment -= rotation[:, 2] * float(rotation[:, 2] @ increment)
            if not np.all(np.isfinite(increment)):
                eligible, reason = False, "invalid_update"
            else:
                step = float(np.hypot.reduce(increment))
                maximum_step = self.rate_limit_nmps * dt
                rate_active = step > maximum_step
                if rate_active:
                    increment *= maximum_step / step
                candidate = baseline + increment
                magnitude = float(np.hypot.reduce(candidate))
                limit_active = magnitude > self.limit_nm
                if limit_active:
                    candidate *= self.limit_nm / magnitude
                accepted = True if accept is None else accept(candidate.copy(), baseline.copy())
                if accepted is True or isinstance(accepted, np.bool_) and bool(accepted):
                    self._world_moment = candidate
                else:
                    allocation_limited = True
                    eligible = False
                    reason = accepted if isinstance(accepted, str) else "allocation_limited"
        world_rate = (self._world_moment - previous) / dt
        self._cached = self._snapshot(rotation, True, eligible, reason, yaw_enabled,
                                      weight, weight_dot, world_rate, limit_active,
                                      rate_active, allocation_limited)
        self._last_tick = tick
        return self._copy(self._cached)
