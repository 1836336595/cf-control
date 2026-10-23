"""ROS-independent, bounded attitude feedback into aircraft references."""

import math

import numpy as np

from slung_reference import _rotation_matrix, _rotation_vector


class SlungAttitudeReferenceTrim:
    """Learn a small formation correction without changing payload attitude goals.

    Attachments are body-frame columns ``(3, N)``; returned position, velocity,
    and acceleration offsets are world-frame rows ``(N, 3)``. The integral is
    a small rotation in a fixed desired payload frame. With centered attachment
    ``rho_i``, its linear reference correction is ``-Rd @ (eta x rho_i)``.
    Positive measured pitch therefore raises a forward aircraft. Centering
    preserves zero common translation, including for yaw corrections.

    Every learning step starts a rest-to-rest quintic segment. Its increment
    is jointly limited using the exact quintic speed and acceleration maxima;
    convex endpoint bounds guarantee the position limit for the whole segment.
    At the defaults the acceleration bound restricts each 1 s endpoint step to
    0.1732 mm, with a peak speed of 0.3248 mm/s. These conservative pauses avoid
    discontinuous reference acceleration when a health gate freezes learning.

    A frozen gate holds the integral while the current bounded segment finishes.
    ``freeze()`` latches that behavior until ``reset()``. The desired rotation
    can be arbitrary but must remain fixed between resets; moving frame rates
    are deliberately not inferred from sampled rotations. Clock gaps never
    catch up missed integral updates. Duplicate update times are idempotent.
    """

    _QUINTIC_MAX_SPEED = 1.875
    _QUINTIC_MAX_ACCELERATION = 10.0 / math.sqrt(3.0)

    def __init__(self, attachment_points_m, gain_per_s=.1, max_offset_m=.01,
                 max_rate_mps=.001, max_acceleration_mps2=.001,
                 adaptation_interval_s=1., enabled=True):
        attachments = self._finite_array(attachment_points_m, "attachment_points_m")
        if (attachments.ndim != 2 or attachments.shape[0] != 3
                or attachments.shape[1] < 3):
            raise ValueError("attachment_points_m must have shape (3, N), N >= 3")
        self._attachments = (attachments - np.mean(attachments, axis=1, keepdims=True)).T
        if np.linalg.matrix_rank(self._attachments) < 2:
            raise ValueError("attachment_points_m must contain noncollinear attachments")
        gains = self._numeric_parameter(gain_per_s, "gain_per_s")
        if gains.shape == ():
            gains = np.full(3, float(gains))
        if gains.shape != (3,) or np.any(gains < 0.):
            raise ValueError("gain_per_s must be a nonnegative scalar or three-vector")
        self.gain_per_s = gains.copy()
        self.max_offset_m = self._positive_scalar(max_offset_m, "max_offset_m")
        self.max_rate_mps = self._positive_scalar(max_rate_mps, "max_rate_mps")
        self.max_acceleration_mps2 = self._positive_scalar(
            max_acceleration_mps2, "max_acceleration_mps2")
        self.adaptation_interval_s = self._positive_scalar(
            adaptation_interval_s, "adaptation_interval_s")
        if not isinstance(enabled, (bool, np.bool_)):
            raise ValueError("enabled must be a boolean")
        self.enabled = bool(enabled)
        self.reset()

    @staticmethod
    def _finite_array(value, name):
        try:
            array = np.asarray(value, dtype=float)
        except (TypeError, ValueError, OverflowError):
            raise ValueError(name + " must contain finite numbers") from None
        if not np.all(np.isfinite(array)):
            raise ValueError(name + " must contain finite numbers")
        return array

    @classmethod
    def _scalar(cls, value, name):
        array = cls._finite_array(value, name)
        if array.shape != ():
            raise ValueError(name + " must be a finite scalar")
        return float(array)

    @classmethod
    def _positive_scalar(cls, value, name):
        array = cls._numeric_parameter(value, name)
        if array.shape != ():
            raise ValueError(name + " must be a finite scalar")
        scalar = float(array)
        if scalar <= 0.:
            raise ValueError(name + " must be positive")
        return scalar

    @classmethod
    def _numeric_parameter(cls, value, name):
        # Inspect before float coercion: a mixed list would otherwise turn
        # True into 1.0 and silently change a conservative physical limit.
        try:
            original = np.asarray(value, dtype=object)
        except (TypeError, ValueError, OverflowError):
            raise ValueError(name + " must contain finite numbers") from None
        if any(isinstance(item, (bool, np.bool_, str, bytes)) for item in original.flat):
            raise ValueError(name + " must contain numbers, without booleans or strings")
        return cls._finite_array(value, name)

    @staticmethod
    def _rotation(value, name):
        try:
            return _rotation_matrix(value, name).copy()
        except OverflowError:
            raise ValueError(name + " must contain finite numbers") from None

    @staticmethod
    def _max_norm(rows):
        return float(np.max(np.hypot.reduce(rows, axis=1)))

    def _offset(self, integral, rotation):
        return -(rotation @ np.cross(integral, self._attachments).T).T

    @property
    def integral(self):
        return self._integral.copy()

    @property
    def attitude_error(self):
        return self._attitude_error.copy()

    @property
    def desired_rotation(self):
        return None if self._desired_rotation is None else self._desired_rotation.copy()

    @property
    def initialized(self):
        return self._desired_rotation is not None

    # Alternate diagnostic names keep manager snapshots descriptive.
    error = attitude_error
    goal_rotation = desired_rotation

    @property
    def saturation(self):
        return self.saturated

    def reset(self):
        """Clear learning and the reference segment, including the fixed frame."""
        self._integral = np.zeros(3)
        self._attitude_error = np.zeros(3)
        self._desired_rotation = None
        self._segment_start = None
        self._start_position = np.zeros_like(self._attachments)
        self._end_position = np.zeros_like(self._attachments)
        self._last_update_time = None
        self._cached = tuple(np.zeros_like(self._attachments) for _ in range(3))
        self.learning = False
        self.frozen = False
        self.saturated = False
        self.rate_limited = False
        self.acceleration_limited = False
        self.progress = 1.

    def freeze(self):
        """Latch the integral hold; the current segment still finishes smoothly."""
        self.frozen = True
        self.learning = False

    def evaluate(self, now):
        """Evaluate the active segment without changing learning or clock state."""
        now = self._scalar(now, "now")
        if self._segment_start is None:
            return tuple(np.zeros_like(self._attachments) for _ in range(3))
        duration = self.adaptation_interval_s
        u = (now - self._segment_start) / duration
        if u <= 0.:
            return (self._start_position.copy(), np.zeros_like(self._attachments),
                    np.zeros_like(self._attachments))
        if u >= 1.:
            return (self._end_position.copy(), np.zeros_like(self._attachments),
                    np.zeros_like(self._attachments))
        s = u**3 * (10. + u * (-15. + 6. * u))
        sd = 30. * u**2 * (1. - u)**2 / duration
        sdd = 60. * u * (1. - u) * (1. - 2. * u) / duration**2
        increment = self._end_position - self._start_position
        return (self._start_position + s * increment, sd * increment, sdd * increment)

    def _next_segment(self, error, desired, now):
        duration = self.adaptation_interval_s
        with np.errstate(over="ignore", invalid="ignore"):
            proposed = self._integral + self.gain_per_s * error * duration
            proposed_position = self._offset(proposed, desired)
        if not np.all(np.isfinite(proposed_position)):
            raise ValueError("trim integral update must remain finite")
        proposed_norm = self._max_norm(proposed_position)
        saturated = proposed_norm > self.max_offset_m
        if saturated:
            proposed *= self.max_offset_m / proposed_norm
        increment = proposed - self._integral
        displacement = self._max_norm(self._offset(increment, desired))
        speed_step = self.max_rate_mps * duration / self._QUINTIC_MAX_SPEED
        acceleration_step = (self.max_acceleration_mps2 * duration**2
                             / self._QUINTIC_MAX_ACCELERATION)
        scale = min(1., speed_step / displacement, acceleration_step / displacement) \
            if displacement > 0. else 1.
        integral = self._integral + scale * increment
        self._start_position = self._end_position.copy()
        self._end_position = self._offset(integral, desired)
        self._integral = integral
        self._segment_start = now
        self.saturated = saturated
        self.rate_limited = displacement > speed_step
        self.acceleration_limited = displacement > acceleration_step

    def update(self, measured_rotation, desired_rotation, now, learning=True):
        """Return bounded reference offsets, adapting at most once per segment.

        ``None`` measurements are accepted while learning is gated. A gated
        call before first learning does not lock the desired rotation. All
        supplied rotations and timestamps are validated before state changes.
        """
        now = self._scalar(now, "now")
        if now < 0. or (self._last_update_time is not None and now < self._last_update_time):
            raise ValueError("now must be nonnegative and nondecreasing")
        measured = None if measured_rotation is None else self._rotation(
            measured_rotation, "measured_rotation")
        desired = None if desired_rotation is None else self._rotation(
            desired_rotation, "desired_rotation")
        if (desired is not None and self._desired_rotation is not None
                and not np.allclose(desired, self._desired_rotation, rtol=0., atol=1e-8)):
            raise ValueError("desired_rotation must remain fixed until reset")
        if self._last_update_time == now:
            return tuple(values.copy() for values in self._cached)
        if desired is None:
            desired = self._desired_rotation
        eligible = self.enabled and bool(learning) and not self.frozen
        if eligible and (measured is None or desired is None):
            raise ValueError("measured_rotation and desired_rotation are required while learning")
        error = self._attitude_error
        if measured is not None and desired is not None:
            error = _rotation_vector(desired.T @ measured)
        if eligible and (self._segment_start is None
                         or now >= self._segment_start + self.adaptation_interval_s):
            self._next_segment(error, desired, now)
        if eligible and self._desired_rotation is None:
            self._desired_rotation = desired.copy()
        self._attitude_error = error.copy()
        self.learning = eligible
        self._last_update_time = now
        self.progress = (1. if self._segment_start is None else min(
            1., max(0., (now - self._segment_start) / self.adaptation_interval_s)))
        self._cached = self.evaluate(now)
        return tuple(values.copy() for values in self._cached)


class SlungDynamicAttitudeReferenceTrim(SlungAttitudeReferenceTrim):
    """Learn bounded world offsets while the full desired attitude changes.

    The actual learned state is the world-frame aircraft offset endpoint,
    with zero common translation. At each segment start, the instantaneous
    desired-frame error ``e = Log(Rd.T @ Rmeasured)`` requests the increment
    ``-Rd @ ((gain_per_s * e) x centered_rho_i) * duration``. The new endpoint
    is projected jointly into the per-aircraft position balls, then approached
    with the same bounded rest-to-rest quintic as the static helper. Desired
    rotations during a segment never rotate its endpoints, so returned world
    velocity and acceleration are complete derivatives even for moving goals.

    ``attitude_error`` remains in instantaneous desired-frame coordinates.
    For API compatibility, ``integral`` is a world-frame small-angle diagnostic:
    if endpoint projection uses beta and the segment limiter uses alpha, it is
    updated as ``eta_new = (1-alpha)*eta + alpha*beta*(eta + Rd @ (gain*e)*T)``.
    This is a weighted sum of historical world angular increments; because
    their attachment orientations differ, it is NOT a single rotation that
    reconstructs the learned offsets. It is neither feedback state nor
    independently bounded by the position limit.

    Gating or ``freeze()`` finishes the active segment and holds its world
    endpoint. Duplicate timestamps are idempotent; gaps never catch up missed
    learning steps. ``desired_rotation`` reports the latest accepted goal
    after initialization, including while learning is gated. A gated call
    before first learning does not initialize the helper.
    """

    def _next_segment(self, error, desired, now):
        duration = self.adaptation_interval_s
        with np.errstate(over="ignore", invalid="ignore"):
            angular_increment = self.gain_per_s * error * duration
            proposed_position = self._end_position + self._offset(angular_increment, desired)
            proposed_integral = self._integral + desired @ angular_increment
        if (not np.all(np.isfinite(proposed_position))
                or not np.all(np.isfinite(proposed_integral))):
            raise ValueError("dynamic trim update must remain finite")

        proposed_norm = self._max_norm(proposed_position)
        saturated = proposed_norm > self.max_offset_m
        projection = self.max_offset_m / proposed_norm if saturated else 1.
        with np.errstate(over="ignore", invalid="ignore"):
            increment = projection * proposed_position - self._end_position
        displacement = self._max_norm(increment)
        speed_step = self.max_rate_mps * duration / self._QUINTIC_MAX_SPEED
        acceleration_step = (self.max_acceleration_mps2 * duration**2
                             / self._QUINTIC_MAX_ACCELERATION)
        scale = min(1., speed_step / displacement, acceleration_step / displacement) \
            if displacement > 0. else 1.
        with np.errstate(over="ignore", invalid="ignore"):
            endpoint = self._end_position + scale * increment
            integral = ((1. - scale) * self._integral
                        + scale * projection * proposed_integral)
        if not np.all(np.isfinite(endpoint)) or not np.all(np.isfinite(integral)):
            raise ValueError("dynamic trim update must remain finite")

        self._start_position = self._end_position.copy()
        self._end_position = endpoint
        self._integral = integral
        self._segment_start = now
        self.saturated = saturated
        self.rate_limited = displacement > speed_step
        self.acceleration_limited = displacement > acceleration_step

    def update(self, measured_rotation, desired_rotation, now, learning=True):
        """Return world P/V/A, sampling the moving goal once per learning step.

        All supplied rotations and timestamps are validated before mutation,
        including duplicate ticks. Missing attitudes are allowed while gated;
        a missing desired rotation uses the most recently accepted goal.
        """
        now = self._scalar(now, "now")
        if now < 0. or (self._last_update_time is not None and now < self._last_update_time):
            raise ValueError("now must be nonnegative and nondecreasing")
        measured = None if measured_rotation is None else self._rotation(
            measured_rotation, "measured_rotation")
        desired = None if desired_rotation is None else self._rotation(
            desired_rotation, "desired_rotation")
        if self._last_update_time == now:
            return tuple(values.copy() for values in self._cached)
        if desired is None:
            desired = self._desired_rotation
        eligible = self.enabled and bool(learning) and not self.frozen
        if eligible and (measured is None or desired is None):
            raise ValueError("measured_rotation and desired_rotation are required while learning")

        error = self._attitude_error
        if measured is not None and desired is not None:
            error = _rotation_vector(desired.T @ measured)
        if eligible and (self._segment_start is None
                         or now >= self._segment_start + self.adaptation_interval_s):
            self._next_segment(error, desired, now)
        if desired is not None and (eligible or self.initialized):
            self._desired_rotation = desired.copy()
        self._attitude_error = error.copy()
        self.learning = eligible
        self._last_update_time = now
        self.progress = (1. if self._segment_start is None else min(
            1., max(0., (now - self._segment_start) / self.adaptation_interval_s)))
        self._cached = self.evaluate(now)
        return tuple(values.copy() for values in self._cached)
