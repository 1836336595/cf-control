"""Continuous payload landing references and measured touchdown confirmation."""

from collections import deque
import math

import numpy as np


def _vector3(value, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(name + ' must be a finite three-vector')
    return vector.copy()


class SlungLandingProfile:
    """Bounded C2 reference preserving initial p/v/a and ending on ground.

    The endpoint holds the initial x/y position. A short, bounded search retains
    a single quintic when feasible. Otherwise acceleration ramps continuously to
    zero, velocity brakes to rest, and a rest-to-rest quintic returns from that
    intermediate position to the endpoint. Full-vector limits and ground are
    checked at polynomial extrema, rather than only at sampled points.

    As before, an initial derivative exceeding a configured limit becomes its
    ceiling, with no additional allowance. Some initial conditions cannot admit
    a C2 bounded reference (for example outward acceleration at maximum speed).
    Those and insufficient clearance for the chosen brake raise ValueError.
    """

    def __init__(self, start_position, start_velocity, start_acceleration,
                 ground_z, now, min_duration_s, max_speed_mps=.12,
                 max_accel_mps2=.15):
        self.start_position = _vector3(start_position, 'start_position')
        self.start_velocity = _vector3(start_velocity, 'start_velocity')
        self.start_acceleration = _vector3(start_acceleration, 'start_acceleration')
        self.start_time = float(now)
        self.ground_z = float(ground_z)
        for name, value in [('min_duration_s', min_duration_s),
                            ('max_speed_mps', max_speed_mps),
                            ('max_accel_mps2', max_accel_mps2)]:
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(name + ' must be positive and finite')
        if not math.isfinite(self.start_time) or not math.isfinite(self.ground_z):
            raise ValueError('start time and ground_z must be finite')
        if self.start_position[2] < self.ground_z - 1e-9:
            raise ValueError('landing reference must start at or above ground')
        self.end_position = self.start_position.copy()
        self.end_position[2] = self.ground_z
        distance = float(np.linalg.norm(self.end_position - self.start_position))
        speed = float(np.linalg.norm(self.start_velocity))
        acceleration = float(np.linalg.norm(self.start_acceleration))
        if not math.isfinite(speed) or not math.isfinite(acceleration):
            raise ValueError('initial derivative magnitudes must be finite')
        speed_limit = max(float(max_speed_mps), speed)
        accel_limit = max(float(max_accel_mps2), acceleration)
        if speed >= speed_limit and self.start_velocity @ self.start_acceleration > 0.:
            raise ValueError('initial speed is at its ceiling with outward acceleration')
        if self.start_position[2] <= self.ground_z:
            if (self.start_velocity[2] < 0.
                    or (self.start_velocity[2] == 0. and self.start_acceleration[2] < 0.)):
                raise ValueError('initial ground boundary has downward velocity or acceleration')
        duration = max(float(min_duration_s), 1.875 * distance / speed_limit,
                       math.sqrt((10. / math.sqrt(3.)) * distance / accel_limit))
        # Duration is not monotone in feasibility when a0 != 0: the a0*T
        # velocity contribution eventually grows. Never stretch indefinitely.
        for _ in range(3):
            coefficients = self._quintic_coefficients(
                self.start_position, self.start_velocity, self.start_acceleration,
                self.end_position, duration)
            if self._within_limits(coefficients, duration, speed_limit, accel_limit):
                self._set_segments([(duration, coefficients)])
                return
            duration *= 1.15
        self._build_braking_descent(float(min_duration_s), speed_limit, accel_limit)

    @staticmethod
    def _quintic_coefficients(position, velocity, acceleration, endpoint, duration):
        # Normalized segment time keeps both short brakes and long descents
        # conditioned; the endpoint velocity and acceleration are zero.
        delta = endpoint - position
        vt, at2 = velocity * duration, acceleration * duration**2
        return np.array([position, vt, .5 * at2,
                         10. * delta - 6. * vt - 1.5 * at2,
                         -15. * delta + 8. * vt + 1.5 * at2,
                         6. * delta - 3. * vt - .5 * at2])

    @staticmethod
    def _critical_points(coefficients):
        """Endpoints and all real roots of a polynomial on normalized [0, 1]."""
        coefficients = np.trim_zeros(coefficients, 'b')
        if len(coefficients) <= 1:
            return np.array([0., 1.])
        roots = np.polynomial.polynomial.polyroots(coefficients)
        real = roots.real[(np.abs(roots.imag) <= 1e-7)
                          & (roots.real >= -1e-9) & (roots.real <= 1. + 1e-9)]
        return np.concatenate(([0., 1.], np.clip(real, 0., 1.)))

    @classmethod
    def _peak_norm(cls, coefficients):
        squared = sum(np.convolve(axis, axis) for axis in coefficients.T)
        critical = cls._critical_points(np.polynomial.polynomial.polyder(squared))
        values = np.polynomial.polynomial.polyval(critical, coefficients)
        return float(np.max(np.linalg.norm(values, axis=0)))

    def _within_limits(self, coefficients, duration, speed_limit, accel_limit):
        velocity = coefficients[1:] * np.arange(1, len(coefficients))[:, None] / duration
        acceleration = velocity[1:] * np.arange(1, len(velocity))[:, None] / duration
        if (self._peak_norm(velocity) > speed_limit + 1e-9
                or self._peak_norm(acceleration) > accel_limit + 1e-9):
            return False
        heights = np.polynomial.polynomial.polyval(
            self._critical_points(velocity[:, 2]), coefficients[:, 2])
        return bool(np.min(heights) >= self.ground_z - 1e-9)

    def _build_braking_descent(self, min_duration, speed_limit, accel_limit):
        segments = []
        position = self.start_position.copy()
        velocity = self.start_velocity.copy()
        acceleration = self.start_acceleration
        anorm2 = float(acceleration @ acceleration)
        if anorm2 > 0.:
            # a(t)=a0*(1-t/T). Its velocity traces the line from v0 to
            # v1=v0+a0*T/2, so the speed norm is bounded by those endpoints.
            # Solve ||v1||<=V for the largest positive T, then keep margin.
            dot = float(velocity @ acceleration)
            margin = max(0., speed_limit**2 - float(velocity @ velocity))
            discriminant = math.sqrt(dot**2 + anorm2 * margin)
            if dot >= 0. and discriminant + dot > 0.:
                max_ramp = 2. * margin / (discriminant + dot)
            else:
                max_ramp = 2. * (discriminant - dot) / anorm2
            if max_ramp <= 0.:
                raise ValueError('no bounded acceleration-ramp brake at the initial speed ceiling')
            duration = min(.1, .5 * max_ramp)
            coefficients = np.array([position, velocity * duration,
                                     .5 * acceleration * duration**2,
                                     -acceleration * duration**2 / 6.])
            segments.append((duration, coefficients))
            position = position + velocity * duration + acceleration * duration**2 / 3.
            velocity = velocity + acceleration * duration / 2.
        speed = float(np.linalg.norm(velocity))
        if speed > 0.:
            # v(u)=v1*(1-3u^2+2u^3). Speed decreases monotonically;
            # peak acceleration is exactly 1.5*||v1||/T at u=1/2.
            duration = 1.5 * speed / accel_limit
            coefficients = np.array([position, velocity * duration, np.zeros(3),
                                     -velocity * duration, .5 * velocity * duration])
            segments.append((duration, coefficients))
            position = position + .5 * velocity * duration
        # The brake is allowed to drift; return from rest to the original XY.
        # Along this straight segment, distance and the scalar quintic maxima
        # give exact full-vector bounds, independent of initial derivatives.
        distance = float(np.linalg.norm(self.end_position - position))
        elapsed = sum(segment[0] for segment in segments)
        duration = max(min_duration - elapsed, 1.875 * distance / speed_limit,
                       math.sqrt((10. / math.sqrt(3.)) * distance / accel_limit), 1e-9)
        segments.append((duration, self._quintic_coefficients(
            position, np.zeros(3), np.zeros(3), self.end_position, duration)))
        for duration, coefficients in segments:
            if not self._within_limits(coefficients, duration, speed_limit, accel_limit):
                raise ValueError('bounded braking transition cannot satisfy speed, acceleration and ground clearance')
        self._set_segments(segments)

    def _set_segments(self, segments):
        self._segments = segments
        self._segment_ends = np.cumsum([segment[0] for segment in segments])
        self.duration = float(self._segment_ends[-1])

    @staticmethod
    def _sample_segment(progress, duration, coefficients):
        progress = np.asarray(progress, dtype=float).reshape(-1)
        powers = progress[:, None] ** np.arange(len(coefficients))[None, :]
        position = powers @ coefficients
        velocity = (powers[:, :-1] @ (
            coefficients[1:] * np.arange(1, len(coefficients))[:, None])) / duration
        acceleration = (powers[:, :-2] @ (
            coefficients[2:] * (np.arange(2, len(coefficients))
                                * np.arange(1, len(coefficients) - 1))[:, None])) / duration**2
        return position, velocity, acceleration

    def _sample(self, progress):
        progress = np.asarray(progress, dtype=float).reshape(-1)
        elapsed = progress * self.duration
        indices = np.minimum(np.searchsorted(self._segment_ends, elapsed, side='right'),
                             len(self._segments) - 1)
        result = tuple(np.empty((len(progress), 3)) for _ in range(3))
        start = 0.
        for index, (duration, coefficients) in enumerate(self._segments):
            mask = indices == index
            if np.any(mask):
                values = self._sample_segment((elapsed[mask] - start) / duration,
                                              duration, coefficients)
                for destination, value in zip(result, values):
                    destination[mask] = value
            start = self._segment_ends[index]
        return result

    def evaluate(self, now):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('evaluation time must be finite')
        if now <= self.start_time:
            return (self.start_position.copy(), self.start_velocity.copy(),
                    self.start_acceleration.copy())
        if now >= self.start_time + self.duration:
            return self.end_position.copy(), np.zeros(3), np.zeros(3)
        progress = (now - self.start_time) / self.duration
        return tuple(value[0] for value in self._sample([progress]))


class PayloadTouchdownDetector:
    """Confirm a quiet near-ground height window despite isolated v_z spikes.

    Confirmation uses a robust 90th-minus-10th percentile height span, a
    least-squares height slope, and absolute median vertical velocity. The
    signed median tolerates alternating derivative noise when height remains
    still, while sustained descent fails both the velocity and height tests.
    Timeout is exposed as a diagnostic only and
    never turns an uncertain landing into a confirmed release.
    """

    def __init__(self, ground_clearance_m=.009, clearance_tolerance_m=.015,
                 speed_limit_mps=.05, confirm_s=.3, window_s=.4,
                 max_height_span_m=.012, timeout_s=4, contact_margin_m=.006):
        for name, value in [
                ('ground_clearance_m', ground_clearance_m),
                ('clearance_tolerance_m', clearance_tolerance_m),
                ('speed_limit_mps', speed_limit_mps), ('confirm_s', confirm_s),
                ('window_s', window_s), ('max_height_span_m', max_height_span_m),
                ('timeout_s', timeout_s), ('contact_margin_m', contact_margin_m)]:
            value = float(value)
            if not math.isfinite(value) or value < 0 or (name != 'ground_clearance_m' and value == 0):
                raise ValueError(name + ' must be finite and positive')
            setattr(self, name, value)
        if self.window_s < self.confirm_s:
            raise ValueError('window_s must cover confirm_s')
        if self.contact_margin_m > self.clearance_tolerance_m:
            raise ValueError('contact_margin_m must not exceed clearance_tolerance_m')
        self.reset()

    def reset(self):
        self._samples = deque()
        self.ready_since = None
        self.detected = False

    def timed_out(self, now, start):
        return bool(math.isfinite(float(now)) and math.isfinite(float(start))
                    and float(now) - float(start) >= self.timeout_s)

    def update(self, now, clearance_m, vertical_velocity):
        try:
            now, height, velocity = map(float, (now, clearance_m, vertical_velocity))
        except (TypeError, ValueError):
            self.reset()
            return False
        if not all(map(math.isfinite, (now, height, velocity))):
            self.reset()
            return False
        if self.detected:
            return True
        # Reusing one received mocap frame at several control ticks adds no
        # independent evidence and must neither extend nor erase the window.
        if self._samples and now == self._samples[-1][0]:
            return False
        if self._samples and (now < self._samples[-1][0]
                              or now - self._samples[-1][0] > self.confirm_s):
            self.reset()
        if abs(height - self.ground_clearance_m) > self.clearance_tolerance_m:
            self.reset()
            return False
        self._samples.append((now, height, velocity))
        while self._samples and now - self._samples[0][0] > self.window_s + 1e-9:
            self._samples.popleft()
        samples = np.asarray(self._samples)
        if len(samples) < 3:
            self.ready_since = None
            return False
        times = samples[:, 0] - samples[0, 0]
        heights = samples[:, 1]
        centered_time = times - np.mean(times)
        slope = float(centered_time @ (heights - np.mean(heights)) /
                      max(float(centered_time @ centered_time), 1e-12))
        stable = bool(
            np.percentile(heights, 90) - np.percentile(heights, 10) <= self.max_height_span_m
            # A wide collection band admits noisy frames, while the tighter
            # median contact band rejects a quiet hover above the floor.
            and np.median(heights) <= self.ground_clearance_m + self.contact_margin_m
            and abs(slope) <= self.speed_limit_mps
            and abs(float(np.median(samples[:, 2]))) <= self.speed_limit_mps
        )
        self.ready_since = float(samples[0, 0]) if stable else None
        self.detected = bool(stable and times[-1] >= self.confirm_s - 1e-9)
        return self.detected
