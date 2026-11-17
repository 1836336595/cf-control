"""Pose-only SE(3) auxiliary system; beta = [body omega, body velocity].

This is an observation/feedback substitution, not the note's full wrench law.
No measured twist, pose difference divided by dt, or acceleration enters beta.
The inherited snapshot contract keeps the flight manager's freshness gates.
"""

import math
import numpy as np

from slung_load_controller import (
    PayloadStateObserver, _as_vector, _hat, _vee, _rotation_exp,
    _rotation_log, _project_so3,
)


def se3_step(rotation, position, twist, dt):
    """Right multiply T(R,p) by exp(dt * hat([omega, v]))."""
    phi = np.asarray(twist[:3]) * dt
    vdt = np.asarray(twist[3:]) * dt
    theta = float(np.linalg.norm(phi))
    h = _hat(phi)
    if theta < 1.e-5:
        jacobian = np.eye(3) + .5 * h + h @ h / 6.
    else:
        jacobian = (np.eye(3) + (1.-math.cos(theta))/theta**2 * h
                    + (theta-math.sin(theta))/theta**3 * (h @ h))
    return (_project_so3(rotation @ _rotation_exp(phi)),
            position + rotation @ jacobian @ vdt)


class VelocityFreePayloadObserver(PayloadStateObserver):
    """Nonhybrid auxiliary pose with sampled, causal reference feedforward.

    Auxiliary propagation uses the preceding accepted sample's twist. Quality
    rejection holds state and its timestamp; only a genuine source gap resets.
    The measured pose remains the control pose (no extra pose filtering).
    """

    def __init__(self, kappa_position=12., kappa_attitude=20.,
                 attitude_weights=(1., 1.1, 1.2), **quality_options):
        self.kappa_position = float(kappa_position)
        self.kappa_attitude = float(kappa_attitude)
        self.weights = _as_vector(attitude_weights, "velocity_free_attitude_weights")
        if (not all(math.isfinite(v) and v > 0. for v in
                    (self.kappa_position, self.kappa_attitude))
                or np.any(self.weights <= 0.)
                or len(np.unique(self.weights)) != 3):
            raise ValueError("velocity-free kappa must be positive; W needs distinct positive weights")
        self.wbar = .5 * (np.sum(self.weights) - self.weights)
        super().__init__(**quality_options)
        # This is a numerical bound, not a closed-loop stability guarantee.
        if max(self.kappa_position, self.kappa_attitude) * self.max_dt > 1.:
            raise ValueError("velocity-free kappa * observer_max_dt must be <= 1")

    def reset(self):
        super().reset()
        self.beta = np.zeros(6)
        self.beta_reference = np.zeros(6)
        self.auxiliary_twist = np.zeros(6)
        self.auxiliary_position = None
        self.auxiliary_rotation = np.eye(3)
        self.reference = None
        self.reference_age_s = math.nan
        self.reference_active = False

    def set_reference(self, target, sample_time):
        """Cache a validated target at its pose-source epoch, under caller lock.

        The next pose callback extrapolates this reference over at most max_dt.
        Clearing a stale reference avoids keeping a moving target after exit.
        """
        stamp = float(sample_time)
        if not math.isfinite(stamp):
            raise ValueError("velocity-free reference needs a finite timestamp")
        reference = dict(
            stamp=stamp, position=_as_vector(target["position"], "reference position"),
            rotation=_project_so3(target.get("rotation", np.eye(3))),
            velocity=_as_vector(target.get("velocity", np.zeros(3)), "reference velocity"),
            acceleration=_as_vector(target.get("acceleration", np.zeros(3)), "reference acceleration"),
            body_rate=_as_vector(target.get("body_rate", np.zeros(3)), "reference body rate"),
            body_rate_dot=_as_vector(target.get("body_rate_dot", np.zeros(3)), "reference angular acceleration"),
        )
        self.reference = {key: value.copy() if isinstance(value, np.ndarray) else value
                          for key, value in reference.items()}

    def _reference_twist(self, position, rotation, timestamp):
        ref = self.reference
        age = math.nan if ref is None else timestamp - ref["stamp"]
        self.reference_age_s = age
        self.reference_active = bool(ref is not None and -1.e-9 <= age <= self.max_dt)
        if not self.reference_active:
            return np.zeros(6)
        pd = ref["position"] + age * ref["velocity"] + .5 * age**2 * ref["acceleration"]
        vd = ref["velocity"] + age * ref["acceleration"]
        rd = ref["rotation"] @ _rotation_exp(
            age * ref["body_rate"] + .5 * age**2 * ref["body_rate_dot"])
        wd = ref["body_rate"] + age * ref["body_rate_dot"]
        # Ad_(Xe^-1) xi_r, with xi_r = [wd, Rd.T vd]. The translational
        # cross term must not be dropped when the desired attitude moves.
        omega_world = rd @ wd
        return np.r_[rotation.T @ omega_world,
                     rotation.T @ (vd + np.cross(omega_world, position-pd))]

    def _snapshot(self):
        state = super()._snapshot()
        state.update(
            feedback_mode="velocity_free_beta",
            beta_body=self.beta.copy(), beta_reference_body=self.beta_reference.copy(),
            auxiliary_position=(None if self.auxiliary_position is None
                                else self.auxiliary_position.copy()),
            auxiliary_rotation=self.auxiliary_rotation.copy(),
            beta_reference_active=self.reference_active,
            beta_reference_age_s=self.reference_age_s,
            acceleration_available=False,
        )
        return state

    def _initialize(self, position, rotation, timestamp, status):
        self.auxiliary_position = position.copy()
        self.auxiliary_rotation = rotation.copy()
        # Start without an invented initial speed; TAKEUP already has a warmup.
        self.beta[:] = 0.
        self.auxiliary_twist[:] = 0.
        return super()._initialize(position, rotation, timestamp, status)

    def update(self, position, rotation, timestamp):
        timestamp = float(timestamp)
        self.source_stamp = timestamp
        self.input_dt_s = None if self._previous_source_time is None else timestamp-self._previous_source_time
        self.update_dt_s = None if self._previous_time is None else timestamp-self._previous_time
        self.observer_updated = False
        self.velocity_clipped = self.acceleration_clipped = self.body_rate_clipped = False
        if not math.isfinite(timestamp):
            return self._reject("invalid_input")
        if self.input_dt_s is not None and self.input_dt_s <= 0.:
            return self._reject("nonmonotonic_time")
        self._previous_source_time = timestamp
        try:
            p = _as_vector(position, "payload position")
            raw_r = np.asarray(rotation, dtype=float).reshape(3, 3)
            if not np.all(np.isfinite(raw_r)):
                return self._reject("invalid_input")
            r = _project_so3(raw_r)
        except (ValueError, np.linalg.LinAlgError):
            return self._reject("invalid_input")
        if self._previous_time is None:
            return self._initialize(p, r, timestamp, "initializing")
        if self.input_dt_s > self.max_dt:
            raw_dt, accepted_dt = self.input_dt_s, self.update_dt_s
            self.reset()
            self.source_stamp = timestamp
            self.input_dt_s, self.update_dt_s = raw_dt, accepted_dt
            return self._initialize(p, r, timestamp, "gap_reset")
        dt = self.update_dt_s
        if dt < self.min_dt_s - 1.e-12:
            self.observer_status = "pending_interval"
            self.derivatives_valid = False
            return self._snapshot()
        # No velocity estimator participates in these pose-jump gates.
        self.position_innovation = p - self.position
        self.attitude_innovation = _rotation_log(self.rotation.T @ r)
        gate_dt = min(self.input_dt_s, self.max_dt)
        if np.any(np.abs(self.position_innovation) >
                  self.position_innovation_margin + self.max_velocity * gate_dt):
            return self._reject("position_innovation")
        if np.linalg.norm(self.attitude_innovation) > (
                self.attitude_innovation_margin + np.max(self.max_body_rate)*gate_dt):
            return self._reject("attitude_innovation")
        # Repeated rejected samples cannot eventually reset/admit a new pose.
        if dt > self.max_dt + 1.e-12:
            return self._reject("auxiliary_stale")
        rh, ph = se3_step(self.auxiliary_rotation, self.auxiliary_position,
                         self.auxiliary_twist, dt)
        rt = rh.T @ r
        pt = rh.T @ (p-ph)
        psi_r = .5 * _vee(np.diag(self.weights) @ rt - rt.T @ np.diag(self.weights))
        beta_ref = self._reference_twist(p, r, timestamp)
        beta = beta_ref + np.r_[self.kappa_attitude * psi_r / self.wbar,
                                self.kappa_position * rt.T @ pt]
        velocity = r @ beta[3:]
        self.velocity_clipped = bool(np.any(np.abs(velocity) >= self.max_velocity))
        self.body_rate_clipped = bool(np.any(np.abs(beta[:3]) >= self.max_body_rate))
        if not np.all(np.isfinite(beta)) or self.velocity_clipped or self.body_rate_clipped:
            return self._reject("beta_limit")
        # Ad_Xtilde beta is in Xhat's body frame, including ptilde x Rtilde*omega.
        self.auxiliary_twist = np.r_[rt @ beta[:3],
            np.cross(pt, rt @ beta[:3]) + rt @ beta[3:]]
        self.auxiliary_rotation, self.auxiliary_position = rh, ph
        self.position, self.rotation = p.copy(), r.copy()
        self.velocity, self.body_rate = velocity, beta[:3].copy()
        self.acceleration = np.zeros(3)  # No differentiated beta; landing starts at a=0.
        self.beta, self.beta_reference = beta, beta_ref
        self._previous_time = timestamp
        self.sample_count += 1
        self.derivatives_valid = self.sample_count >= self.min_samples
        self.consecutive_rejected_count = 0
        self.observer_updated = True
        self.observer_status = "updated" if self.derivatives_valid else "initializing"
        return self._snapshot()
