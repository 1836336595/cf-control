#!/usr/bin/env python3
"""MATLAB-compatible geometric controller for a three-Crazyflie slung load.

The implementation keeps the MATLAB controller's control stages in one place:
payload position/attitude feedback, wrench allocation, link-direction control,
and per-vehicle force construction.  Inputs and outputs use the project's
right-handed world-z-up convention.  The load rigid body itself is measured at
the top-surface origin; :func:`mocap_top_surface_to_center` converts it to the
geometric center before this controller is called.
"""

from dataclasses import dataclass
import math

import numpy as np


def smoothstep5_profile(elapsed, duration):
    """Return quintic blend position, velocity and acceleration coefficients."""
    duration = float(duration)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("smoothstep duration must be positive")
    progress = float(np.clip(float(elapsed) / duration, 0.0, 1.0))
    p2 = progress * progress
    p3 = p2 * progress
    return np.array([
        10.0 * p3 - 15.0 * p3 * progress + 6.0 * p3 * p2,
        (30.0 * p2 - 60.0 * p3 + 30.0 * p3 * progress) / duration,
        (60.0 * progress - 180.0 * p2 + 120.0 * p3) / (duration * duration),
    ])


class SecondOrderVelocityFilter:
    """Causal second-order low-pass for a sampled three-axis velocity."""

    def __init__(self, cutoff_hz=5.0, max_dt=0.1):
        self.cutoff_hz = float(cutoff_hz)
        self.max_dt = float(max_dt)
        if (not math.isfinite(self.cutoff_hz) or self.cutoff_hz <= 0.0 or
                not math.isfinite(self.max_dt) or self.max_dt <= 0.0 or
                2.0 * self.cutoff_hz * self.max_dt > 0.8):
            raise ValueError("invalid second-order velocity filter parameters")
        self.reset()

    def reset(self):
        self._previous_time = None
        self._previous_input = None
        self._previous_input_2 = None
        self._previous_output = None
        self._previous_output_2 = None
        self.ready = False

    @staticmethod
    def _coefficients(cutoff_hz, dt):
        normalized_cutoff = 2.0 * cutoff_hz * dt
        omega = math.pi * normalized_cutoff
        cosine = math.cos(omega)
        alpha = math.sin(omega) / (2.0 * math.sqrt(0.5))
        a0 = 1.0 + alpha
        return (
            (1.0 - cosine) / (2.0 * a0),
            (1.0 - cosine) / a0,
            (1.0 - cosine) / (2.0 * a0),
            -2.0 * cosine / a0,
            (1.0 - alpha) / a0,
        )

    def update(self, velocity, timestamp):
        value = np.asarray(velocity, dtype=float).reshape(3)
        timestamp = float(timestamp)
        if not math.isfinite(timestamp) or not np.all(np.isfinite(value)):
            self.reset()
            return None, None, False
        if self._previous_time is None:
            self._previous_time = timestamp
            self._previous_input = value.copy()
            self._previous_input_2 = value.copy()
            self._previous_output = value.copy()
            self._previous_output_2 = value.copy()
            self.ready = False
            return value.copy(), np.zeros(3), False
        dt = timestamp - self._previous_time
        if not math.isfinite(dt) or dt <= 0.0 or dt > self.max_dt:
            self.reset()
            return self.update(value, timestamp)
        b0, b1, b2, a1, a2 = self._coefficients(self.cutoff_hz, dt)
        filtered = (
            b0 * value + b1 * self._previous_input + b2 * self._previous_input_2
            - a1 * self._previous_output - a2 * self._previous_output_2
        )
        acceleration = (filtered - self._previous_output) / dt
        self._previous_input_2 = self._previous_input
        self._previous_input = value.copy()
        self._previous_output_2 = self._previous_output
        self._previous_output = filtered.copy()
        self._previous_time = timestamp
        self.ready = True
        return filtered, acceleration, True


def _as_vector(value, name):
    vector = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(vector)):
        raise ValueError("%s must be finite" % name)
    return vector


def _hat(vector):
    x, y, z = _as_vector(vector, "vector")
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _vee(matrix):
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]], dtype=float)


def _normalize(vector, fallback=None):
    vector = _as_vector(vector, "vector")
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-12:
        if fallback is None:
            raise ValueError("cannot normalize zero vector")
        return _as_vector(fallback, "fallback")
    return vector / norm


def _project_so3(rotation):
    u, _, vh = np.linalg.svd(np.asarray(rotation, dtype=float).reshape(3, 3))
    result = u @ vh
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vh
    return result


def mocap_top_surface_to_center(position, rotation, height_m):
    """Convert a top-surface-center rigid-body origin to box center.

    The object body z-axis is upward in the ROS/NOKOV representation, so the
    geometric center is ``height/2`` below the measured top surface.
    """
    position = _as_vector(position, "payload position")
    rotation = _project_so3(rotation)
    height_m = float(height_m)
    if not math.isfinite(height_m) or height_m <= 0.0:
        raise ValueError("payload height must be positive")
    return position + rotation @ np.array([0.0, 0.0, -0.5 * height_m])


def attachment_points_ros_z_up(size_m):
    """Return MATLAB's [CF3, CF4, CF5] attachment points in ROS z-up.

    The returned array is the controller's internal ``3 x 3`` layout:
    columns are vehicles and rows are ``[x, y, z]``. MATLAB uses geometric
    center coordinates with z-down positive and puts all three attachments on
    the upper face. Reflecting z gives positive ROS z coordinates.
    """
    size = _as_vector(size_m, "payload size")
    if np.any(size <= 0.0):
        raise ValueError("payload size must be positive")
    length, width, height = size
    return np.array([
        [0.5 * length, -0.5 * length, -0.5 * length],
        [0.0, 0.5 * width, -0.5 * width],
        [0.5 * height, 0.5 * height, 0.5 * height],
    ], dtype=float)


def attachment_points_from_yaml(value):
    """Convert YAML point rows ``[[x, y, z], ...]`` to controller columns."""
    points = np.asarray(value, dtype=float)
    if points.shape != (3, 3) or not np.all(np.isfinite(points)):
        raise ValueError(
            "attachment_points_m must contain three finite [x, y, z] points"
        )
    return points.T.copy()


def takeup_vehicle_targets(payload_position, payload_rotation,
                          attachment_points_m, link_length_m,
                          outward_offset_m):
    """Compute spaced aircraft targets one cable length from load anchors."""
    position = _as_vector(payload_position, "payload_position")
    rotation = _project_so3(payload_rotation)
    attachment_points = np.asarray(attachment_points_m, dtype=float)
    if attachment_points.shape != (3, 3) or not np.all(np.isfinite(attachment_points)):
        raise ValueError("attachment_points_m must have shape (3, 3)")
    link_length = float(link_length_m)
    outward_offset = float(outward_offset_m)
    if (not math.isfinite(link_length) or link_length <= 0.0 or
            not math.isfinite(outward_offset) or outward_offset < 0.0 or
            outward_offset >= link_length):
        raise ValueError("invalid TAKEUP cable geometry")
    horizontal = min(outward_offset, 0.85 * link_length)
    vertical = math.sqrt(max(link_length * link_length - horizontal * horizontal, 0.0))
    targets = np.zeros((3, 3), dtype=float)
    for index in range(3):
        rho = attachment_points[:, index]
        attachment = position + rotation @ rho
        radial = rotation @ np.array([rho[0], rho[1], 0.0])
        radial[2] = 0.0
        radial_norm = float(np.linalg.norm(radial))
        if radial_norm < 1.0e-9:
            radial = np.array([
                math.cos(2.0 * math.pi * index / 3.0),
                math.sin(2.0 * math.pi * index / 3.0),
                0.0,
            ])
        else:
            radial /= radial_norm
        targets[:, index] = (
            attachment + horizontal * radial + np.array([0.0, 0.0, vertical])
        )
    return targets


def cable_distances(payload_position, payload_rotation, attachment_points_m,
                    vehicle_positions):
    """Measure all three load-anchor to aircraft-center distances."""
    position = _as_vector(payload_position, "payload_position")
    rotation = _project_so3(payload_rotation)
    attachment_points = np.asarray(attachment_points_m, dtype=float)
    vehicle_positions = np.asarray(vehicle_positions, dtype=float)
    if attachment_points.shape != (3, 3) or vehicle_positions.shape != (3, 3):
        raise ValueError("TAKEUP geometry requires three attachment and vehicle positions")
    if not np.all(np.isfinite(attachment_points)) or not np.all(np.isfinite(vehicle_positions)):
        raise ValueError("TAKEUP geometry must be finite")
    distances = np.zeros(3, dtype=float)
    for index in range(3):
        attachment = position + rotation @ attachment_points[:, index]
        distances[index] = np.linalg.norm(attachment - vehicle_positions[index])
    return distances


def ordered_vehicle_ids(entries):
    """Return the required transport order CF3, CF4, CF5."""
    ids = {int(entry["id"]) for entry in entries}
    required = [3, 4, 5]
    if ids != set(required):
        raise ValueError("slung-load transport requires exactly CF3, CF4, CF5")
    return required


@dataclass
class SlungLoadConfig:
    payload_mass_kg: float
    gravity_mps2: float
    payload_size_m: np.ndarray
    attachment_points_m: np.ndarray
    link_length_m: float
    position_gain: np.ndarray
    velocity_gain: np.ndarray
    integral_gain: np.ndarray
    integral_limit: np.ndarray
    c1: float
    force_norm_epsilon: float
    tension_pinv_tolerance: float
    link_kq: float
    link_komega: float
    link_integral_gain: float
    link_integral_limit: np.ndarray
    outward_bias_fraction: float = 0.20
    outward_bias_max_n: float = 0.12


class SlungLoadController:
    """Stateful MATLAB slung-load controller in ROS world-z-up coordinates."""

    def __init__(self, config):
        self.config = config
        cfg = config
        self.position_integral = np.zeros(3)
        self.link_integrals = np.zeros((3, 3))
        self.previous_link_units = None
        self.previous_payload_rotation = None
        self._validate()

    def _validate(self):
        cfg = self.config
        if cfg.payload_mass_kg <= 0.0 or cfg.gravity_mps2 <= 0.0:
            raise ValueError("payload mass and gravity must be positive")
        if np.asarray(cfg.attachment_points_m).shape != (3, 3):
            raise ValueError("attachment_points_m must have shape (3, 3)")
        if cfg.link_length_m <= 0.0 or cfg.force_norm_epsilon <= 0.0:
            raise ValueError("link length and force epsilon must be positive")
        if cfg.tension_pinv_tolerance <= 0.0:
            raise ValueError("tension pseudoinverse tolerance must be positive")

    def reset(self):
        self.position_integral[:] = 0.0
        self.link_integrals[:] = 0.0
        self.previous_link_units = None
        self.previous_payload_rotation = None

    @staticmethod
    def _state_vector(state, name, fallback=None):
        if name in state:
            return _as_vector(state[name], name)
        if fallback is not None:
            return _as_vector(fallback, name)
        raise KeyError(name)

    def _allocation_matrix(self):
        rho = np.asarray(self.config.attachment_points_m, dtype=float)
        matrix = np.zeros((6, 9), dtype=float)
        for index in range(3):
            matrix[:3, 3 * index:3 * index + 3] = np.eye(3)
            matrix[3:, 3 * index:3 * index + 3] = _hat(rho[:, index])
        return matrix

    def _internal_bias(self, matrix, payload_mass, gravity):
        desired = np.zeros((3, 3), dtype=float)
        rho = self.config.attachment_points_m
        for index in range(3):
            radial = np.array([rho[0, index], rho[1, index], 0.0])
            if np.linalg.norm(radial) < 1.0e-12:
                angle = 2.0 * math.pi * index / 3.0
                radial = np.array([math.cos(angle), math.sin(angle), 0.0])
            else:
                radial /= np.linalg.norm(radial)
            desired[:, index] = (
                self.config.outward_bias_fraction * payload_mass * gravity /
                math.sqrt(3.0) * radial
            )
        u, singular, vh = np.linalg.svd(matrix)
        rank = int(np.sum(singular > self.config.tension_pinv_tolerance))
        if rank >= matrix.shape[1]:
            return np.zeros(9)
        null_basis = vh[rank:, :].T
        bias = null_basis @ (null_basis.T @ desired.reshape(-1))
        blocks = bias.reshape(3, 3)
        rms = math.sqrt(float(np.mean(np.sum(blocks * blocks, axis=0))))
        target_rms = min(
            self.config.outward_bias_fraction * payload_mass * gravity / math.sqrt(3.0),
            self.config.outward_bias_max_n,
        )
        if rms > 1.0e-12 and target_rms > 0.0:
            bias *= target_rms / rms
        return bias

    def compute(self, payload_state, vehicle_states, target, dt):
        """Return per-vehicle force commands or ``None`` for invalid payload state."""
        if not payload_state or not bool(payload_state.get("valid", False)):
            return None
        if len(vehicle_states) != 3:
            raise ValueError("slung-load controller requires three vehicle states")
        dt = float(np.clip(dt, 0.001, 0.05))
        try:
            p0 = self._state_vector(payload_state, "position")
            v0 = self._state_vector(payload_state, "velocity", np.zeros(3))
            a0 = self._state_vector(payload_state, "acceleration", np.zeros(3))
            r0 = _project_so3(payload_state["rotation"])
            omega0 = self._state_vector(payload_state, "body_rate", np.zeros(3))
            pd = self._state_vector(target, "position")
            vd = self._state_vector(target, "velocity", np.zeros(3))
            ad = self._state_vector(target, "acceleration", np.zeros(3))
            rd = _project_so3(target.get("rotation", np.eye(3)))
            omegad = self._state_vector(target, "body_rate", np.zeros(3))
            omegadd = self._state_vector(target, "body_rate_dot", np.zeros(3))
        except (KeyError, TypeError, ValueError):
            return None

        ep = p0 - pd
        ev = v0 - vd
        integral_rate = ev + self.config.c1 * ep
        self.position_integral = np.clip(
            self.position_integral + dt * integral_rate,
            -self.config.integral_limit,
            self.config.integral_limit,
        )
        desired_load_acceleration = (
            ad - self.config.position_gain * ep
            - self.config.velocity_gain * ev
            - self.config.integral_gain * self.position_integral
        )
        gravity = np.array([0.0, 0.0, self.config.gravity_mps2])
        desired_force = self.config.payload_mass_kg * (
            desired_load_acceleration + gravity
        )

        e_r = 0.5 * _vee(rd.T @ r0 - r0.T @ rd)
        e_omega = omega0 - r0.T @ rd @ omegad
        inertia = np.diag([
            self.config.payload_mass_kg * (
                self.config.payload_size_m[1] ** 2 + self.config.payload_size_m[2] ** 2
            ) / 12.0,
            self.config.payload_mass_kg * (
                self.config.payload_size_m[0] ** 2 + self.config.payload_size_m[2] ** 2
            ) / 12.0,
            self.config.payload_mass_kg * (
                self.config.payload_size_m[0] ** 2 + self.config.payload_size_m[1] ** 2
            ) / 12.0,
        ])
        feedforward = r0.T @ rd @ omegad
        desired_moment = (
            -np.array([0.0, 0.0, 0.0]) * e_r
            - np.array([0.0, 0.0, 0.0]) * e_omega
            + _hat(feedforward) @ inertia @ feedforward
            + inertia @ r0.T @ rd @ omegadd
        )
        # Payload attitude gains are supplied by the caller in target.  This
        # avoids duplicating MATLAB's size-derived gains in the vehicle file.
        desired_moment += (
            -_as_vector(target["payload_attitude_gain"], "payload_attitude_gain") * e_r
            -_as_vector(target["payload_rate_gain"], "payload_rate_gain") * e_omega
        )
        if not bool(target.get("payload_yaw_enabled", True)):
            desired_moment[2] = 0.0
        omega0_dot_cmd = np.linalg.solve(
            inertia, desired_moment - np.cross(omega0, inertia @ omega0)
        )

        matrix = self._allocation_matrix()
        rhs = np.concatenate([r0.T @ desired_force, desired_moment])
        gram = matrix @ matrix.T
        if np.linalg.cond(gram) > 1.0 / self.config.tension_pinv_tolerance:
            raise ValueError("slung-load tension allocation matrix is singular")
        mu_body = matrix.T @ np.linalg.solve(gram, rhs)
        mu_body += self._internal_bias(
            matrix, self.config.payload_mass_kg, self.config.gravity_mps2
        )
        mu_desired = (r0 @ mu_body.reshape(3, 3).T).T

        link_units = np.zeros((3, 3))
        link_rates = np.zeros((3, 3))
        for index, vehicle in enumerate(vehicle_states):
            pv = self._state_vector(vehicle, "position")
            vv = self._state_vector(vehicle, "velocity", vehicle.get("control_velocity", np.zeros(3)))
            rho_world = r0 @ self.config.attachment_points_m[:, index]
            attachment = p0 + rho_world
            relative = attachment - pv
            link_units[:, index] = _normalize(relative, [0.0, 0.0, -1.0])
            payload_omega_world = r0 @ omega0
            relative_velocity = v0 + np.cross(payload_omega_world, rho_world) - vv
            link_rates[:, index] = (
                (np.eye(3) - np.outer(link_units[:, index], link_units[:, index]))
                @ relative_velocity / max(float(np.linalg.norm(relative)), 1.0e-6)
            )

        if self.previous_link_units is None:
            self.previous_link_units = link_units.copy()
        outputs = []
        for index, vehicle in enumerate(vehicle_states):
            q = link_units[:, index]
            qdot = link_rates[:, index]
            omega_link = np.cross(q, qdot)
            rho = self.config.attachment_points_m[:, index]
            ai = (
                desired_load_acceleration + gravity
                + r0 @ (_hat(omega0) @ (_hat(omega0) @ rho))
                - r0 @ _hat(rho) @ omega0_dot_cmd
            )
            mu_id = mu_desired[:, index]
            mu_i = q * float(q @ mu_id)
            qid = _normalize(-mu_id, self.previous_link_units[:, index])
            eq = np.cross(qid, q)
            self.link_integrals[:, index] = np.clip(
                self.link_integrals[:, index] + dt * eq,
                -self.config.link_integral_limit,
                self.config.link_integral_limit,
            )
            correction = (
                -self.config.link_kq * eq
                - self.config.link_komega * omega_link
                - self.config.link_integral_gain * self.link_integrals[:, index]
            )
            q_hat_sq = _hat(q) @ _hat(q)
            u_parallel = (
                mu_i
                + vehicle.get("mass", 0.043) * self.config.link_length_m
                * float(omega_link @ omega_link) * q
                + vehicle.get("mass", 0.043) * q * float(q @ ai)
            )
            u_perp = (
                vehicle.get("mass", 0.043) * self.config.link_length_m
                * np.cross(q, correction)
                - vehicle.get("mass", 0.043) * q_hat_sq @ ai
            )
            total_force = u_parallel + u_perp
            outputs.append({
                "force": total_force,
                "force_dot": np.zeros(3),
                "payload_position_error": ep.copy(),
                "payload_velocity_error": ev.copy(),
                "payload_attitude_error": e_r.copy(),
                "link_direction": q.copy(),
                "desired_link_direction": qid.copy(),
                "link_direction_error": eq.copy(),
                "desired_tension": float(np.linalg.norm(mu_i)),
                "desired_tension_vector": mu_id.copy(),
            })
        self.previous_link_units = link_units.copy()
        return {
            "vehicles": outputs,
            "payload_position_error": ep,
            "payload_velocity_error": ev,
            "payload_attitude_error": e_r,
            "desired_force": desired_force,
            "desired_moment": desired_moment,
            "link_units": link_units,
        }
