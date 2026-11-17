"""Report stable airborne endpoint distances without modifying cable models."""

from collections import deque
import math
import numpy as np


class SlungLinkLengthReporter:
    """Pose-window evidence for a human-readable effective length measurement.

    Mount points are ROWS in payload coordinates. The load position must
    already be its geometric center, not the top-surface mocap origin.
    Neither nominal lengths nor estimated tensions enter the measurement.
    """

    def __init__(self, attachment_points, window_s=3., level_tolerance_deg=3.):
        self.points = np.asarray(attachment_points, dtype=float).copy()
        self.window_s = float(window_s)
        self.level_tolerance_deg = float(level_tolerance_deg)
        if (self.points.shape != (3, 3) or not np.all(np.isfinite(self.points))
                or not math.isfinite(self.window_s) or self.window_s <= 0.
                or not math.isfinite(self.level_tolerance_deg)
                or not 0. < self.level_tolerance_deg <= 10.):
            raise ValueError('invalid link length report geometry/window/level tolerance')
        self.sequence = 0
        self.last_report = None
        self.reset_window()

    def reset_window(self):
        self.samples = deque()
        self.last_report_time = None
        self.status = 'collecting'

    @property
    def sample_count(self):
        return len(self.samples)

    def update(self, stamp, position, rotation, vehicles, eligible=True):
        if not eligible:
            self.reset_window()
            self.status = 'ineligible'
            return None
        try:
            stamp = float(stamp)
            position = np.asarray(position, dtype=float).reshape(3)
            rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
            vehicles = np.asarray(vehicles, dtype=float).reshape(3, 3)
            if not all(np.all(np.isfinite(v)) for v in (stamp, position, rotation, vehicles)):
                raise ValueError('nonfinite sample')
            tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.))))
            if tilt > self.level_tolerance_deg + 2.:
                self.reset_window()
                self.status = 'not_level'
                return None
        except (ValueError, TypeError):
            self.reset_window()
            self.status = 'invalid_geometry'
            return None
        if self.samples:
            if stamp == self.samples[-1][0]:
                return None
            if stamp < self.samples[-1][0] or stamp-self.samples[-1][0] > .10+1e-9:
                self.reset_window()
        mounts = position + self.points @ rotation.T
        lengths = np.linalg.norm(vehicles-mounts, axis=1)
        if np.any(lengths <= 0.):
            self.reset_window()
            self.status = 'invalid_geometry'
            return None
        self.samples.append((stamp, lengths, np.vstack([position, vehicles]), tilt))
        while len(self.samples) > 1 and self.samples[1][0] <= stamp-self.window_s+1e-9:
            self.samples.popleft()
        if len(self.samples) < 30 or stamp-self.samples[0][0] < self.window_s-1e-9:
            self.status = 'collecting'
            return None
        if self.last_report_time is not None and stamp-self.last_report_time < 10.:
            self.status = 'waiting_interval'
            return None
        lengths = np.array([v[1] for v in self.samples])
        positions = np.array([v[2] for v in self.samples])
        tilts = np.array([v[3] for v in self.samples])
        p10, p90 = np.percentile(lengths, [10, 90], axis=0)
        motion = np.linalg.norm(np.ptp(positions, axis=0), axis=1)
        if np.median(tilts) > self.level_tolerance_deg:
            self.status = 'not_level'
            return None
        if np.max(motion) > .03:
            self.status = 'pose_motion'
            return None
        if np.any(p90-p10 > .005):
            self.status = 'length_variation'
            return None
        self.sequence += 1
        result = dict(sequence=self.sequence, sample_time=stamp,
                      window_s=stamp-self.samples[0][0], samples=len(self.samples),
                      median_m=np.median(lengths, axis=0), p10_m=p10, p90_m=p90,
                      median_tilt_deg=float(np.median(tilts)), max_tilt_deg=float(np.max(tilts)))
        self.last_report_time = stamp
        self.last_report = result
        self.status = 'reported'
        return result
