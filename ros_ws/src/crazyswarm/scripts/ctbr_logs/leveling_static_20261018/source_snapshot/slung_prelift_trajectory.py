"""Bounded payload prelift references; no force or payload-mass feedforward."""

import math

import numpy as np

from slung_load_controller import smoothstep5_profile


def _positions(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError("prelift positions must be finite arrays with shape (3, 3)")
    return result.copy()


def handoff_references(anchors, held, blend, blend_dot, blend_ddot):
    """Fade the PID height bias while the cooperative force takes over."""
    anchors = _positions(anchors)
    bias = _positions(held) - anchors
    return (anchors + (1.0 - blend) * bias,
            -blend_dot * bias, -blend_ddot * bias)


class SlungPreliftTrajectory:
    """Align to measured taut geometry, pull slowly, and retain the height bias."""

    def __init__(self, clearance_m=0.20, clearance_margin_m=0.02,
                 speed_mps=0.05, acceleration_mps2=0.05,
                 max_rise_m=1.8, max_lead_m=1.6, timeout_s=60.0,
                 alignment_duration_s=2.0, lead_hold_s=10.0,
                 position_gain_scale=(5.0, 5.0, 1.0),
                 velocity_gain_scale=(2.0, 2.0, 1.0), max_tilt_rad=math.radians(12.0)):
        values = (speed_mps, acceleration_mps2, max_rise_m, max_lead_m,
                  timeout_s, alignment_duration_s, lead_hold_s)
        if any(not math.isfinite(v) or v <= 0.0 for v in values):
            raise ValueError("prelift speed, acceleration, bounds and durations must be positive")
        if any(not math.isfinite(v) or v < 0.0
               for v in (clearance_m, clearance_margin_m)):
            raise ValueError("prelift clearance and margin must be non-negative")
        self.clearance_m = float(clearance_m)
        self.goal = float(clearance_m + clearance_margin_m)
        self.speed_mps = float(speed_mps)
        self.acceleration_mps2 = float(acceleration_mps2)
        self.max_rise_m = float(max_rise_m)
        self.max_lead_m = float(max_lead_m)
        self.timeout_s = float(timeout_s)
        self.alignment_duration_s = float(alignment_duration_s)
        self.lead_hold_s = float(lead_hold_s)
        self.position_gain_scale = np.asarray(position_gain_scale, dtype=float)
        self.velocity_gain_scale = np.asarray(velocity_gain_scale, dtype=float)
        self.max_tilt_rad = float(max_tilt_rad)
        if (self.position_gain_scale.shape != (3,) or
                self.velocity_gain_scale.shape != (3,) or
                not np.all(np.isfinite(self.position_gain_scale)) or
                not np.all(np.isfinite(self.velocity_gain_scale)) or
                np.any(self.position_gain_scale <= 0.0) or
                np.any(self.velocity_gain_scale <= 0.0) or
                not math.isfinite(self.max_tilt_rad) or self.max_tilt_rad <= 0.0):
            raise ValueError("prelift PID gain scales must be three positive finite values")
        self.stage = "waiting"
        self.reason = ""
        self.lead_hold_started = None
        self.lead_hold_clearance = None

    def start(self, measured_positions, reference_positions, now):
        self.anchors = _positions(measured_positions)
        self.source = _positions(reference_positions)
        self.positions = self.source.copy()
        self.velocities = np.zeros((3, 3))
        self.accelerations = np.zeros((3, 3))
        self.started = self.last_time = float(now)
        self.alignment_elapsed = 0.0
        distance = float(np.max(np.abs(self.anchors - self.source)))
        self.alignment_duration = max(
            self.alignment_duration_s,
            1.875 * distance / self.speed_mps,
            math.sqrt((10.0 / math.sqrt(3.0)) * distance / self.acceleration_mps2),
        )
        self.speed = 0.0
        self.stage = "aligning"
        self.reason = ""
        self.lead_hold_started = self.lead_hold_clearance = None

    def snapshot(self, stage=None):
        return dict(positions=self.positions.copy(), velocities=self.velocities.copy(),
                    accelerations=self.accelerations.copy(), stage=stage or self.stage,
                    active=bool(np.any(np.abs(self.velocities) > 1e-9)),
                    failed=self.stage == "failed", reason=self.reason,
                    rise_m=float(np.max(self.positions[:, 2] - self.anchors[:, 2])))

    def update(self, measured_positions, clearance_m, now):
        if self.stage == "waiting":
            raise RuntimeError("start the prelift reference after taut-cable confirmation")
        measured = _positions(measured_positions)
        now = float(now)
        dt = float(np.clip(now - self.last_time, 0.0, 0.05))
        self.last_time = now
        self.velocities[:] = self.accelerations[:] = 0.0
        if self.stage == "failed":
            return self.snapshot()
        if now - self.started >= self.timeout_s:
            self.stage, self.reason = "failed", "timeout"
            return self.snapshot()
        if not math.isfinite(clearance_m):
            self.speed = 0.0
            return self.snapshot(stage="paused")
        if self.stage == "aligning":
            self.alignment_elapsed += dt
            profile = smoothstep5_profile(self.alignment_elapsed, self.alignment_duration)
            displacement = self.anchors - self.source
            self.positions = self.source + profile[0] * displacement
            self.velocities = profile[1] * displacement
            self.accelerations = profile[2] * displacement
            if self.alignment_elapsed >= self.alignment_duration:
                self.stage = "rising"
            return self.snapshot()

        # All anchors rise by the same offset, even when cable lengths differ.
        remaining_rise = self.max_rise_m - float(np.max(self.positions[:, 2] - self.anchors[:, 2]))
        remaining_lead = self.max_lead_m - float(np.max(self.positions[:, 2] - measured[:, 2]))
        remaining = min(remaining_rise, remaining_lead)
        if remaining_rise < -1e-8 or (clearance_m < self.clearance_m and remaining_rise <= 1e-8):
            self.stage, self.reason = "failed", "max_rise"
            self.speed = 0.0
            return self.snapshot()
        if self.stage == "lead_hold":
            # Measure elapsed time on every held update, even when noise moves
            # the lead just below the boundary. Resume only with useful headroom.
            if clearance_m - self.lead_hold_clearance >= 0.02:
                self.lead_hold_started = now
                self.lead_hold_clearance = float(clearance_m)
            if remaining_lead >= -1e-8 and clearance_m >= self.goal:
                self.stage = "holding"
                self.lead_hold_started = self.lead_hold_clearance = None
                return self.snapshot()
            resume_margin = min(0.02, 0.25 * self.max_lead_m)
            if remaining_lead >= resume_margin:
                self.stage = "rising"
                self.lead_hold_started = self.lead_hold_clearance = None
            elif now - self.lead_hold_started >= self.lead_hold_s:
                self.stage, self.reason = "failed", "max_lead"
                return self.snapshot()
            else:
                return self.snapshot()
        if remaining_lead < -1e-8 or (clearance_m < self.clearance_m and remaining_lead <= 1e-8):
            self.stage = "lead_hold"
            self.speed = 0.0
            self.lead_hold_started = now
            self.lead_hold_clearance = float(clearance_m)
            return self.snapshot()
        if self.stage == "holding" and clearance_m >= self.clearance_m:
            return self.snapshot()
        self.stage = "rising" if clearance_m < self.goal else "braking"
        goal_speed = (min(self.speed_mps,
                          math.sqrt(2.0 * self.acceleration_mps2 * max(remaining, 0.0)))
                      if self.stage == "rising" else 0.0)
        next_speed = self.speed + float(np.clip(
            goal_speed - self.speed, -self.acceleration_mps2 * dt,
            self.acceleration_mps2 * dt,
        ))
        increment = min(0.5 * (self.speed + next_speed) * dt, max(remaining, 0.0))
        self.positions[:, 2] += increment
        self.velocities[:, 2] = next_speed
        self.accelerations[:, 2] = (next_speed - self.speed) / dt if dt > 0.0 else 0.0
        self.speed = next_speed
        if self.stage == "braking" and self.speed < 1e-9:
            self.stage = "holding"
        return self.snapshot()
