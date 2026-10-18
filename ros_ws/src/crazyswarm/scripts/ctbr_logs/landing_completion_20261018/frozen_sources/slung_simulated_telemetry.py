"""Sampled firmware-log transport for the offline slung replay (no ROS I/O).

The producer supplies ideal position/velocity. Delivery uses the real node's
EKF callback; this models telemetry timing and host filtering, not firmware EKF.
"""

from collections import deque
import math
from types import SimpleNamespace

import numpy as np


class SampledEkfTelemetry:
    def __init__(self, sample_rate_hz=50.0, delay_s=.02):
        sample_rate_hz, delay_s = float(sample_rate_hz), float(delay_s)
        if (not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0
                or not math.isfinite(delay_s) or delay_s < 0):
            raise ValueError("EKF sample rate must be positive and delay nonnegative")
        self.period = 1.0 / sample_rate_hz
        self.delay_s = delay_s
        self.next_sample_time = 0.0
        self.sample_count = 0
        self.delivered_count = 0  # One frame contains one packet per vehicle.
        self._queue = deque()

    def sample_if_due(self, now, states):
        now = float(now)
        if now + 1e-10 < self.next_sample_time:
            return False
        packets = [np.concatenate((state["position"], state["velocity"])).tolist()
                   for state in states]
        self._queue.append((now, packets))
        self.next_sample_time = now + self.period
        self.sample_count += 1
        return True

    def deliver(self, now, vehicles):
        count = 0
        while self._queue and self._queue[0][0] + self.delay_s <= now + 1e-10:
            sampled, packets = self._queue.popleft()
            if len(packets) != len(vehicles):
                raise ValueError("EKF packet count must match vehicle count")
            for vehicle, values in zip(vehicles, packets):
                vehicle._ekf_state_callback(SimpleNamespace(
                    values=values,
                    header=SimpleNamespace(stamp=SimpleNamespace(
                        to_sec=lambda stamp=sampled: stamp)),
                ))
            count += 1
        self.delivered_count += count
        return count
