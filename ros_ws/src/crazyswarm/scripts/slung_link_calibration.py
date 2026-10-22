"""Bounded, one-shot effective cable length estimates from session geometry.

These estimates use a stable upper geometry plateau as a proxy for tautness.
The caller owns state validity, flight phase, and eligibility; this module
neither proves cable tension nor bypasses those gates or writes parameters.
"""

from collections import deque
import math

import numpy as np


class SlungLinkCalibrator:
    """Freeze one positive correction after a full, stable unique-sample window."""

    def __init__(self, nominal_lengths, window_s=5., max_adjustment_m=.03,
                 max_spread_m=.006, max_drift_mps=.003, max_sample_gap_s=.10,
                 min_samples=100, trim_fraction=.10):
        nominal = np.asarray(nominal_lengths, dtype=float)
        if nominal.shape != (3,) or not np.all(np.isfinite(nominal)) or np.any(nominal <= 0):
            raise ValueError('nominal_lengths must be three finite positive lengths')
        self._nominal = nominal.copy()
        self._effective = nominal.copy()
        for name, value in [('window_s', window_s), ('max_adjustment_m', max_adjustment_m),
                            ('max_spread_m', max_spread_m), ('max_drift_mps', max_drift_mps),
                            ('max_sample_gap_s', max_sample_gap_s)]:
            value = float(value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be positive and finite')
            setattr(self, name, value)
        if isinstance(min_samples, bool) or int(min_samples) != min_samples or min_samples < 2:
            raise ValueError('min_samples must be an integer of at least two')
        self.min_samples = int(min_samples)
        self.trim_fraction = float(trim_fraction)
        if not math.isfinite(self.trim_fraction) or not 0 <= self.trim_fraction < .5:
            raise ValueError('trim_fraction must be in [0, 0.5)')
        self.applied_time = None
        self._samples = deque()
        self.status = 'collecting'
        self.reason = ''
        self.sample_count = 0
        self.window_elapsed_s = 0.
        self.spread_m = np.full(3, np.nan)
        self.candidate = None

    @property
    def nominal(self):
        return self._nominal.copy()

    @property
    def effective(self):
        return self._effective.copy()

    def reset_window(self, reason):
        """Clear untrusted samples while preserving an already applied estimate."""
        self._samples.clear()
        self.sample_count = 0
        self.window_elapsed_s = 0.
        self.spread_m = np.full(3, np.nan)
        if self.applied_time is not None:
            self.status = 'applied'
            return
        self.candidate = None
        self.status = 'collecting'
        self.reason = str(reason)

    def update(self, sample_time, distances, eligible=True, mismatch_tolerance_m=.015):
        """Return the one accepted effective-length vector, otherwise ``None``.

        Duplicate timestamps add no evidence. The window retains at most one
        sample immediately before its cutoff so a five-second observation can
        qualify even when the sensor period does not divide five seconds.
        """
        if self.applied_time is not None:
            return None
        if not eligible:
            self.reset_window('ineligible')
            return None
        try:
            sample_time = float(sample_time)
            tolerance = float(mismatch_tolerance_m)
            values = np.asarray(distances, dtype=float)
        except (TypeError, ValueError):
            self.reset_window('invalid_geometry')
            return None
        if (not math.isfinite(sample_time) or not math.isfinite(tolerance) or tolerance < 0
                or values.shape != (3,) or not np.all(np.isfinite(values))
                or np.any(values <= 0)):
            self.reset_window('invalid_geometry')
            return None
        if self._samples:
            previous = self._samples[-1][0]
            if sample_time == previous:
                return None
            if sample_time < previous:
                self.reset_window('timestamp_reversal')
            elif sample_time - previous > self.max_sample_gap_s + 1e-9:
                self.reset_window('sample_gap')
        self._samples.append((sample_time, values.copy()))
        cutoff = sample_time - self.window_s
        while len(self._samples) > 1 and self._samples[1][0] <= cutoff + 1e-9:
            self._samples.popleft()
        self.sample_count = len(self._samples)
        self.window_elapsed_s = sample_time - self._samples[0][0]
        self.status, self.reason = 'collecting', ''
        if self.window_elapsed_s < self.window_s - 1e-9:
            return None
        if self.sample_count < self.min_samples:
            self.status, self.reason = 'insufficient_samples', 'not_enough_unique_samples'
            return None
        times = np.array([item[0] for item in self._samples])
        measured = np.array([item[1] for item in self._samples])
        self.spread_m = np.percentile(measured, 90, axis=0) - np.percentile(measured, 10, axis=0)
        ordered = np.sort(measured, axis=0)
        trim_count = int(self.sample_count * self.trim_fraction)
        central = ordered[trim_count:self.sample_count - trim_count] if trim_count else ordered
        estimate = np.mean(central, axis=0)
        self.candidate = np.maximum(self._nominal, estimate)
        # Use the interior quantile band for drift, preventing one distance
        # spike from dominating a least-squares slope of otherwise quiet data.
        lower, upper = np.percentile(measured, [10, 90], axis=0)
        clipped = np.clip(measured, lower, upper)
        centered_time = times - np.mean(times)
        drift = centered_time @ (clipped - np.mean(clipped, axis=0)) / max(
            float(centered_time @ centered_time), 1e-12)
        if np.any(self.spread_m > self.max_spread_m + 1e-9) or np.any(
                np.abs(drift) > self.max_drift_mps + 1e-9):
            self.status, self.reason = 'unstable', 'distance_spread_or_drift'
            return None
        if np.any(np.median(measured, axis=0) < self._nominal - tolerance - 1e-9):
            self.status, self.reason = 'unstable', 'slack_geometry'
            return None
        if np.any(self.candidate - self._nominal > self.max_adjustment_m + 1e-9):
            self.status, self.reason = 'out_of_bounds', 'positive_adjustment_exceeds_limit'
            return None
        if not np.any(estimate - self._nominal > tolerance + 1e-9):
            self.status, self.reason = 'within_tolerance', 'no_positive_mismatch'
            return None
        self._effective = self.candidate.copy()
        self.applied_time = sample_time
        self.status, self.reason = 'applied', 'stable_positive_mismatch'
        return self._effective.copy()
