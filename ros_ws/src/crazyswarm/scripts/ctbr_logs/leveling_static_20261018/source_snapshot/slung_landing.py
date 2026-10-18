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
    """Quintic reference preserving initial p/v/a and ending at rest on ground.

    The endpoint holds the initial x/y position. Duration grows until sampled
    full-vector speed/acceleration bounds hold and the vertical reference does
    not cross below ground. If the initial reference already exceeds a bound,
    that initial magnitude is the allowable ceiling; no instantaneous reset is
    introduced to disguise it.
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
        speed_limit = max(float(max_speed_mps), float(np.linalg.norm(self.start_velocity)))
        accel_limit = max(float(max_accel_mps2), float(np.linalg.norm(self.start_acceleration)))
        duration = max(float(min_duration_s), 1.875 * distance / speed_limit,
                       math.sqrt(5.774 * distance / accel_limit))
        for _ in range(80):
            self.duration = duration
            self._solve_coefficients()
            samples = self._sample(np.linspace(0., 1., 1001))
            position, velocity, acceleration = samples
            if (np.max(np.linalg.norm(velocity, axis=1)) <= speed_limit + 1e-8
                    and np.max(np.linalg.norm(acceleration, axis=1)) <= accel_limit + 1e-8
                    and np.min(position[:, 2]) >= self.ground_z - 1e-9):
                return
            duration *= 1.15
        raise ValueError('no bounded quintic landing satisfies the initial derivatives and ground constraint')

    def _solve_coefficients(self):
        duration = self.duration
        # Solve in normalized time to retain conditioning for long descents.
        self._coefficients = np.zeros((6, 3))
        self._coefficients[0] = self.start_position
        self._coefficients[1] = self.start_velocity * duration
        self._coefficients[2] = .5 * self.start_acceleration * duration * duration
        a0, a1, a2 = self._coefficients[:3]
        rhs = np.array([self.end_position - a0 - a1 - a2,
                        -a1 - 2 * a2, -2 * a2])
        self._coefficients[3:] = np.linalg.solve(
            np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]), rhs)

    def _sample(self, progress):
        progress = np.asarray(progress, dtype=float).reshape(-1)
        powers = progress[:, None] ** np.arange(6)[None, :]
        position = powers @ self._coefficients
        velocity = (powers[:, :5] @ (
            self._coefficients[1:] * np.arange(1, 6)[:, None])) / self.duration
        acceleration = (powers[:, :4] @ (
            self._coefficients[2:] * np.array([2., 6., 12., 20.])[:, None])) / self.duration**2
        return position, velocity, acceleration

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
