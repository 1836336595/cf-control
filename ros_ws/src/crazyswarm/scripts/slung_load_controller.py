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
    """Causal second-order low-pass for a sampled three-axis velocity.

    Optional speed and sample-jump limits reject impossible mocap derivatives
    before they enter the IIR state.  A rejected sample holds the last valid
    output for one cycle and marks the derivative as not ready.
    """

    def __init__(self, cutoff_hz=5.0, max_dt=0.1,
                 max_speed_mps=None, max_jump_mps=None):
        self.cutoff_hz = float(cutoff_hz)
        self.max_dt = float(max_dt)
        if (not math.isfinite(self.cutoff_hz) or self.cutoff_hz <= 0.0 or
                not math.isfinite(self.max_dt) or self.max_dt <= 0.0 or
                2.0 * self.cutoff_hz * self.max_dt > 0.8):
            raise ValueError("invalid second-order velocity filter parameters")
        self.max_speed_mps = self._limit_vector(max_speed_mps, "max_speed_mps")
        self.max_jump_mps = self._limit_vector(max_jump_mps, "max_jump_mps")
        self.reset()

    @staticmethod
    def _limit_vector(value, name):
        if value is None:
            return None
        limits = np.asarray(value, dtype=float).reshape(-1)
        if limits.size == 1:
            limits = np.full(3, float(limits[0]))
        if (limits.size != 3 or not np.all(np.isfinite(limits)) or
                np.any(limits <= 0.0)):
            raise ValueError("%s must contain three positive finite values" % name)
        return limits

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

    def _reject_sample(self, timestamp):
        """Hold the last output while keeping the next sample time continuous."""
        if self._previous_output is None:
            self.reset()
            return None, None, False
        held = self._previous_output.copy()
        self._previous_time = timestamp
        self._previous_input = held.copy()
        self._previous_input_2 = held.copy()
        self._previous_output = held.copy()
        self._previous_output_2 = held.copy()
        self.ready = False
        return held, np.zeros(3), False

    def update(self, velocity, timestamp):
        value = np.asarray(velocity, dtype=float).reshape(3)
        timestamp = float(timestamp)
        if not math.isfinite(timestamp) or not np.all(np.isfinite(value)):
            self.reset()
            return None, None, False
        if (self.max_speed_mps is not None and
                np.any(np.abs(value) > self.max_speed_mps)):
            return self._reject_sample(timestamp)
        if (self.max_jump_mps is not None and self._previous_input is not None and
                np.any(np.abs(value - self._previous_input) > self.max_jump_mps)):
            return self._reject_sample(timestamp)
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


def _rotation_exp(vector):
    """SO(3) exponential map for a rotation vector in radians."""
    vector = _as_vector(vector, "rotation vector")
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-8:
        return np.eye(3) + _hat(vector)
    axis = vector / angle
    axis_hat = _hat(axis)
    return (
        np.eye(3)
        + math.sin(angle) * axis_hat
        + (1.0 - math.cos(angle)) * (axis_hat @ axis_hat)
    )


def _rotation_log(rotation):
    """Return the principal SO(3) rotation vector."""
    rotation = _project_so3(rotation)
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1.0e-7:
        return _vee(0.5 * (rotation - rotation.T))
    sine = math.sin(angle)
    if abs(sine) < 1.0e-6:
        # The load is not expected to jump by pi between mocap samples.  This
        # fallback keeps the observer finite if a rigid-body relabeling does.
        diagonal = np.diag(rotation)
        axis = np.sqrt(np.maximum(0.0, (diagonal + 1.0) * 0.5))
        index = int(np.argmax(axis))
        if axis[index] < 1.0e-7:
            return np.zeros(3)
        if index == 0:
            axis[1] = rotation[0, 1] / max(2.0 * axis[0], 1.0e-7)
            axis[2] = rotation[0, 2] / max(2.0 * axis[0], 1.0e-7)
        elif index == 1:
            axis[0] = rotation[0, 1] / max(2.0 * axis[1], 1.0e-7)
            axis[2] = rotation[1, 2] / max(2.0 * axis[1], 1.0e-7)
        else:
            axis[0] = rotation[0, 2] / max(2.0 * axis[2], 1.0e-7)
            axis[1] = rotation[1, 2] / max(2.0 * axis[2], 1.0e-7)
        return angle * _normalize(axis, [1.0, 0.0, 0.0])
    return angle * _vee((rotation - rotation.T) / (2.0 * sine))


class PayloadStateObserver:
    """Estimate load derivatives from NOKOV pose and attitude only.

    Translation uses a causal alpha-beta-gamma observer.  Attitude uses the
    same alpha-beta idea on SO(3): propagate with the estimated body rate,
    correct the rotation with the measured rigid-body orientation, then use
    that orientation innovation to correct angular velocity.  NOKOV twist and
    acceleration fields are intentionally not consumed by this observer.
    """

    def __init__(
            self, position_gain=0.35, velocity_gain=0.08,
            acceleration_gain=0.02, attitude_gain=0.35,
            angular_rate_gain=0.08, max_dt=0.05,
            max_velocity_mps=None, max_acceleration_mps2=None,
            max_body_rate_rps=None, min_samples=3):
        self.position_gain = float(position_gain)
        self.velocity_gain = float(velocity_gain)
        self.acceleration_gain = float(acceleration_gain)
        self.attitude_gain = float(attitude_gain)
        self.angular_rate_gain = float(angular_rate_gain)
        self.max_dt = float(max_dt)
        self.max_velocity = self._limit_vector(
            max_velocity_mps, "max_velocity_mps", default=np.full(3, np.inf)
        )
        self.max_acceleration = self._limit_vector(
            max_acceleration_mps2, "max_acceleration_mps2", default=np.full(3, np.inf)
        )
        self.max_body_rate = self._limit_vector(
            max_body_rate_rps, "max_body_rate_rps", default=np.full(3, np.inf)
        )
        self.min_samples = int(min_samples)
        if (
                not all(math.isfinite(value) and value > 0.0 for value in (
                    self.position_gain, self.velocity_gain,
                    self.acceleration_gain, self.attitude_gain,
                    self.angular_rate_gain, self.max_dt
                )) or
                self.position_gain > 1.0 or self.velocity_gain > 1.0 or
                self.acceleration_gain > 1.0 or self.attitude_gain > 1.0 or
                self.angular_rate_gain > 1.0 or self.min_samples < 2):
            raise ValueError("invalid payload pose observer parameters")
        self.reset()

    @staticmethod
    def _limit_vector(value, name, default):
        if value is None:
            return np.asarray(default, dtype=float).copy()
        vector = np.asarray(value, dtype=float).reshape(-1)
        if vector.size == 1:
            vector = np.full(3, float(vector[0]))
        if (vector.size != 3 or not np.all(np.isfinite(vector)) or
                np.any(vector <= 0.0)):
            raise ValueError("%s must contain three positive finite values" % name)
        return vector

    def reset(self):
        self._previous_time = None
        self.position = None
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.rotation = np.eye(3)
        self.body_rate = np.zeros(3)
        self.sample_count = 0
        self.derivatives_valid = False

    def _snapshot(self):
        return {
            "velocity": self.velocity.copy(),
            "acceleration": self.acceleration.copy(),
            "rotation": self.rotation.copy(),
            "body_rate": self.body_rate.copy(),
            "derivatives_valid": bool(self.derivatives_valid),
            "sample_count": int(self.sample_count),
        }

    def update(self, position, rotation, timestamp):
        position = _as_vector(position, "observer position")
        rotation = _project_so3(rotation)
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            self.reset()
            return None
        if self._previous_time is None or self.position is None:
            self.position = position.copy()
            self.rotation = rotation.copy()
            self._previous_time = timestamp
            self.sample_count = 1
            self.derivatives_valid = False
            return self._snapshot()

        dt = timestamp - self._previous_time
        if not math.isfinite(dt) or dt <= 0.0 or dt > self.max_dt:
            self.reset()
            self.position = position.copy()
            self.rotation = rotation.copy()
            self._previous_time = timestamp
            self.sample_count = 1
            return self._snapshot()

        dt2 = dt * dt
        predicted_position = (
            self.position + dt * self.velocity + 0.5 * dt2 * self.acceleration
        )
        predicted_velocity = self.velocity + dt * self.acceleration
        residual = position - predicted_position
        self.position = predicted_position + self.position_gain * residual
        self.velocity = predicted_velocity + self.velocity_gain * residual / dt
        self.acceleration = self.acceleration + (
            self.acceleration_gain * residual / dt2
        )
        self.velocity = np.clip(self.velocity, -self.max_velocity, self.max_velocity)
        self.acceleration = np.clip(
            self.acceleration, -self.max_acceleration, self.max_acceleration
        )

        predicted_rotation = self.rotation @ _rotation_exp(self.body_rate * dt)
        attitude_residual = _rotation_log(predicted_rotation.T @ rotation)
        self.rotation = _project_so3(
            predicted_rotation @ _rotation_exp(self.attitude_gain * attitude_residual)
        )
        self.body_rate = self.body_rate + (
            self.angular_rate_gain * attitude_residual / dt
        )
        self.body_rate = np.clip(
            self.body_rate, -self.max_body_rate, self.max_body_rate
        )
        self._previous_time = timestamp
        self.sample_count += 1
        self.derivatives_valid = self.sample_count >= self.min_samples
        return self._snapshot()


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


def link_lengths_vector(value, name="link_lengths_m"):
    """Normalize a scalar or CF3/CF4/CF5 link-length list to three values."""
    values = np.asarray(value, dtype=float).reshape(-1)
    if values.size == 1:
        values = np.full(3, float(values[0]))
    elif values.size != 3:
        raise ValueError("%s must be a scalar or contain three values" % name)
    if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
        raise ValueError("%s must contain positive finite values" % name)
    return values


def translate_vehicle_references(anchor_positions, payload_start_position,
                                 payload_position, payload_velocity,
                                 payload_acceleration):
    """Translate fixed TAKEUP vehicle anchors along a payload reference.

    The anchors encode the measured cable geometry at handoff.  Translating
    them by the payload's quintic reference keeps the aircraft position,
    velocity, and acceleration continuous when TAUT_RAMP ends.
    """
    anchors = np.asarray(anchor_positions, dtype=float)
    if anchors.shape != (3, 3) or not np.all(np.isfinite(anchors)):
        raise ValueError("anchor_positions must have shape (3, 3)")
    start = _as_vector(payload_start_position, "payload_start_position")
    position = _as_vector(payload_position, "payload_position")
    velocity = _as_vector(payload_velocity, "payload_velocity")
    acceleration = _as_vector(payload_acceleration, "payload_acceleration")
    return (
        anchors + (position - start),
        np.tile(velocity, (3, 1)),
        np.tile(acceleration, (3, 1)),
    )


def takeup_target_distance(link_length_m, pre_tension_slack_m):
    """Return MATLAB's TAKEUP distance ``link length - pre-tension slack``."""
    scalar_input = np.asarray(link_length_m).size == 1
    link_length = link_lengths_vector(link_length_m, "link_length_m")
    slack = np.asarray(pre_tension_slack_m, dtype=float).reshape(-1)
    if slack.size == 1:
        slack = np.full(3, float(slack[0]))
    elif slack.size != 3:
        raise ValueError("pre_tension_slack_m must be a scalar or contain three values")
    if (not np.all(np.isfinite(slack)) or np.any(slack < 0.0) or
            np.any(slack >= link_length)):
        raise ValueError("invalid TAKEUP pre-tension geometry")
    target = link_length - slack
    return float(target[0]) if scalar_input else target


def takeup_distance_ready(distances, link_length_m, target_distance_m,
                          tolerance_m):
    """Check cables are near target without rejecting small tracking overruns.

    The measured rigid-body positions and the independent vehicle controllers
    have millimetre-to-centimetre error.  A hard ``distance <= link_length``
    gate can therefore reject a physically taut cable even when it is within
    the configured TAKEUP distance tolerance.
    """
    distances = np.asarray(distances, dtype=float).reshape(-1)
    try:
        link_length = link_lengths_vector(link_length_m, "link_length_m")
        target_distance = link_lengths_vector(target_distance_m, "target_distance_m")
    except (TypeError, ValueError):
        return False
    tolerance = float(tolerance_m)
    if (distances.size != 3 or not np.all(np.isfinite(distances)) or
            not np.all(np.isfinite(target_distance)) or
            not np.all((target_distance > 0.0) & (target_distance <= link_length)) or
            not math.isfinite(tolerance) or tolerance <= 0.0):
        return False
    return bool(np.all(distances <= link_length + tolerance) and
                np.all(np.abs(distances - target_distance) <= tolerance))


def takeup_tracking_ready(observed_positions, target_positions, payload_position,
                          payload_rotation, attachment_points_m,
                          position_tolerance_m, angle_tolerance_rad):
    """Check cable direction against the TAKEUP target geometry.

    ``position_tolerance_m`` is optional.  The measured cable length and
    direction are the physical handoff conditions; a nominal aircraft target
    can retain centimetre-level error even when the cable is already taut.
    Callers may therefore pass ``None`` to keep the position mismatch as a
    diagnostic only while retaining the direction check.
    """
    observed = np.asarray(observed_positions, dtype=float)
    targets = np.asarray(target_positions, dtype=float)
    payload = _as_vector(payload_position, "payload_position")
    rotation = _project_so3(payload_rotation)
    attachments = np.asarray(attachment_points_m, dtype=float)
    position_tolerance = (
        None if position_tolerance_m is None else float(position_tolerance_m)
    )
    angle_tolerance = float(angle_tolerance_rad)
    if (observed.shape != (3, 3) or targets.shape != (3, 3) or
            attachments.shape != (3, 3) or
            not np.all(np.isfinite(observed)) or
            not np.all(np.isfinite(targets)) or
            not np.all(np.isfinite(attachments)) or
            (position_tolerance is not None and
             (not math.isfinite(position_tolerance) or position_tolerance <= 0.0)) or
            not math.isfinite(angle_tolerance) or angle_tolerance <= 0.0):
        return False
    if (position_tolerance is not None and
            np.any(np.linalg.norm(observed - targets, axis=1) > position_tolerance)):
        return False
    for index in range(3):
        attachment = payload + rotation @ attachments[:, index]
        actual_vector = attachment - observed[index]
        target_vector = attachment - targets[index]
        actual_norm = float(np.linalg.norm(actual_vector))
        target_norm = float(np.linalg.norm(target_vector))
        if actual_norm < 1.0e-9 or target_norm < 1.0e-9:
            return False
        cosine = float(np.clip(
            actual_vector @ target_vector / (actual_norm * target_norm), -1.0, 1.0
        ))
        if math.acos(cosine) > angle_tolerance:
            return False
    return True


def takeup_vehicle_targets(payload_position, payload_rotation,
                          attachment_points_m, link_length_m,
                          outward_offset_m, link_units=None):
    """Compute spaced aircraft targets at the requested cable distance."""
    position = _as_vector(payload_position, "payload_position")
    rotation = _project_so3(payload_rotation)
    attachment_points = np.asarray(attachment_points_m, dtype=float)
    if attachment_points.shape != (3, 3) or not np.all(np.isfinite(attachment_points)):
        raise ValueError("attachment_points_m must have shape (3, 3)")
    link_lengths = link_lengths_vector(link_length_m, "link_length_m")
    outward_offset = float(outward_offset_m)
    if (not math.isfinite(outward_offset) or outward_offset < 0.0 or
            outward_offset >= float(np.min(link_lengths))):
        raise ValueError("invalid TAKEUP cable geometry")
    if link_units is not None:
        link_units = np.asarray(link_units, dtype=float)
        if link_units.shape != (3, 3) or not np.all(np.isfinite(link_units)):
            raise ValueError("link_units must have shape (3, 3)")
        link_units = np.column_stack([
            _normalize(link_units[:, index], [0.0, 0.0, -1.0])
            for index in range(3)
        ])
    targets = np.zeros((3, 3), dtype=float)
    for index in range(3):
        link_length = float(link_lengths[index])
        horizontal = min(outward_offset, 0.85 * link_length)
        vertical = math.sqrt(max(link_length * link_length - horizontal * horizontal, 0.0))
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
        if link_units is not None:
            # MATLAB: x_i = attachment_i - d_i q_i, q_i vehicle -> load.
            target = attachment - link_length * link_units[:, index]
            # MATLAB's ideal simulation has no vehicle collision risk.  On the
            # real platform, independent position tracking can pull the rear
            # vehicles inward before TAKEUP is confirmed.  Add the configured
            # horizontal outward margin while preserving the requested cable
            # distance, so the margin changes only direction, not geometry.
            outward = target[:2] - position[:2]
            outward_norm = float(np.linalg.norm(outward))
            if outward_norm < 1.0e-9:
                outward = radial[:2]
                outward_norm = float(np.linalg.norm(outward))
            outward = outward / max(outward_norm, 1.0e-12)
            target_xy = target[:2] + outward_offset * outward
            horizontal_from_attachment = target_xy - attachment[:2]
            horizontal_norm = float(np.linalg.norm(horizontal_from_attachment))
            if horizontal_norm >= link_length:
                target_xy = attachment[:2] + (
                    horizontal_from_attachment * (0.95 * link_length / horizontal_norm)
                )
                horizontal_from_attachment = target_xy - attachment[:2]
                horizontal_norm = float(np.linalg.norm(horizontal_from_attachment))
            target[:2] = target_xy
            target[2] = attachment[2] + math.sqrt(
                max(link_length * link_length - horizontal_norm * horizontal_norm, 0.0)
            )
            targets[:, index] = target
        else:
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
    link_lengths_m: object = None

    def __post_init__(self):
        source = self.link_length_m if self.link_lengths_m is None else self.link_lengths_m
        self.link_lengths_m = link_lengths_vector(source)
        self.link_length_m = float(np.mean(self.link_lengths_m))


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
        if (cfg.link_length_m <= 0.0 or
                not np.all(np.isfinite(cfg.link_lengths_m)) or
                np.any(cfg.link_lengths_m <= 0.0) or
                cfg.force_norm_epsilon <= 0.0):
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
        # P uses a stacked [mu_1; mu_2; mu_3] vector.  ``desired`` is stored
        # as 3-by-3 columns, so flatten in Fortran order to preserve MATLAB's
        # column-major block layout.
        desired_stacked = desired.reshape(-1, order="F")
        bias = null_basis @ (null_basis.T @ desired_stacked)
        blocks = bias.reshape(3, 3)
        rms = math.sqrt(float(np.mean(np.sum(blocks * blocks, axis=0))))
        target_rms = min(
            self.config.outward_bias_fraction * payload_mass * gravity / math.sqrt(3.0),
            self.config.outward_bias_max_n,
        )
        if rms > 1.0e-12 and target_rms > 0.0:
            bias *= target_rms / rms
        return bias

    def equilibrium_link_units(self, payload_rotation):
        """Return MATLAB's static-hover desired link directions in world axes."""
        rotation = _project_so3(payload_rotation)
        matrix = self._allocation_matrix()
        desired_force = np.array([
            0.0, 0.0,
            self.config.payload_mass_kg * self.config.gravity_mps2,
        ])
        rhs = np.concatenate([rotation.T @ desired_force, np.zeros(3)])
        gram = matrix @ matrix.T
        if np.linalg.cond(gram) > 1.0 / self.config.tension_pinv_tolerance:
            raise ValueError("slung-load tension allocation matrix is singular")
        mu_body = matrix.T @ np.linalg.solve(gram, rhs)
        mu_body += self._internal_bias(
            matrix, self.config.payload_mass_kg, self.config.gravity_mps2
        )
        mu_blocks = mu_body.reshape(3, 3)
        # Rows are stacked vehicle blocks; columns of the controller layout
        # are vehicle force vectors, matching MATLAB reshape(..., 3, n).
        mu_world = rotation @ mu_blocks.T
        directions = np.zeros((3, 3), dtype=float)
        for index in range(3):
            magnitude = float(np.linalg.norm(mu_world[:, index]))
            if magnitude < self.config.force_norm_epsilon:
                raise ValueError("equilibrium link tension is too small")
            # q points from vehicle to load; with ROS z-up the hover direction
            # therefore has a negative z component.
            directions[:, index] = -mu_world[:, index] / magnitude
        return directions

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
        mu_desired = r0 @ mu_body.reshape(3, 3).T

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
        # Establish link-direction correction continuously with TAUT_RAMP.
        # Full-strength correction at handoff can pull an aircraft inward when
        # its measured cable direction still differs from static allocation.
        link_gain_scale = float(np.clip(target.get("link_gain_scale", 1.0), 0.0, 1.0))
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
                -link_gain_scale * self.config.link_kq * eq
                -link_gain_scale * self.config.link_komega * omega_link
                -link_gain_scale * self.config.link_integral_gain * self.link_integrals[:, index]
            )
            q_hat_sq = _hat(q) @ _hat(q)
            u_parallel = (
                mu_i
                + vehicle.get("mass", 0.043) * self.config.link_lengths_m[index]
                * float(omega_link @ omega_link) * q
                + vehicle.get("mass", 0.043) * q * float(q @ ai)
            )
            u_perp = (
                vehicle.get("mass", 0.043) * self.config.link_lengths_m[index]
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
