"""Three CrazySwarm Crazyflies lifting a rigid payload with unilateral cables."""

from __future__ import annotations

import time
from enum import Enum

import mujoco
import numpy as np

from ...crazyswarm_slung_parameters import (
    CrazySwarmSlungConfig,
    CrazyflieParameters,
    default_config,
)
from ...control.crazyswarm_slung import (
    CrazySwarmSlungController,
    z_down_to_up,
    z_up_to_down,
)
from ...control.payload_state_observer import PayloadStateObserver
from ...control.payload_state_filter import PayloadPoseLowPassFilter
from ...control.crazyflie_pid import CrazyflieMellingerActuators, CrazyfliePidActuators
from .crazyswarm_slung_logging import (
    CrazySwarmSlungCsvLogger,
    rotation_euler_xyz as _rotation_euler_xyz,
)
from .crazyswarm_slung_scene import build_mjcf, initial_vehicle_positions as _initial_vehicle_positions
from ..mujoco import MujocoModel

__all__ = [
    "CrazySwarmSlungConfig",
    "CrazySwarmSlungCsvLogger",
    "CrazySwarmSlungModel",
    "CrazySwarmSlungPhase",
    "CrazyflieParameters",
    "build_mjcf",
    "default_config",
    "quintic_profile",
]


class CrazySwarmSlungPhase(str, Enum):
    GROUND_SLACK = "GROUND_SLACK"
    INDEPENDENT_TAKEOFF = "INDEPENDENT_TAKEOFF"
    TAKEUP = "TAKEUP"
    TENSION_RAMP = "TENSION_RAMP"
    REFERENCE_LIFT = "REFERENCE_LIFT"
    HOVER = "HOVER"
    LANDING_TAUT = "LANDING_TAUT"
    LANDING_RELEASE = "LANDING_RELEASE"
    LANDED = "LANDED"
    EMERGENCY_LANDING = "EMERGENCY_LANDING"


def quintic_profile(elapsed, duration):
    """Return a C2 smoothstep position, velocity, and acceleration scale."""
    duration = max(float(duration), 1.0e-9)
    u = float(np.clip(elapsed / duration, 0.0, 1.0))
    position = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
    velocity = (30.0 * u**2 * (1.0 - u) ** 2) / duration
    acceleration = (60.0 * u * (1.0 - u) * (1.0 - 2.0 * u)) / duration**2
    if elapsed <= 0.0 or elapsed >= duration:
        velocity = 0.0
        acceleration = 0.0
    return np.array([position, velocity, acceleration])


class CrazySwarmSlungModel:
    """Headless-capable MuJoCo simulation of CrazySwarm's three-CF slung load."""

    def __init__(self, render=False, record=None, config=None, csv_path=None):
        self.config = default_config() if config is None else config
        if not np.isfinite(self.config.control_rate_hz) or self.config.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive and finite")
        if not np.isfinite(self.config.timestep) or self.config.timestep <= 0.0:
            raise ValueError("timestep must be positive and finite")
        if self.config.onboard_controller not in ("first_order", "pid", "mellinger"):
            raise ValueError("onboard_controller must be 'first_order', 'pid', or 'mellinger'")
        if self.config.motor_model not in ("graefe2026", "legacy_quadratic"):
            raise ValueError("motor_model must be graefe2026 or legacy_quadratic")
        if self.config.motor_model == "graefe2026" and self.config.onboard_controller == "first_order":
            raise ValueError("graefe2026 requires pid or mellinger; first_order bypasses the motor mixer")
        self.control_dt = 1.0 / self.config.control_rate_hz
        if self.config.onboard_controller == "pid":
            actuator_type = CrazyfliePidActuators
        elif self.config.onboard_controller == "mellinger":
            actuator_type = CrazyflieMellingerActuators
        else:
            actuator_type = None
        self._onboard_actuators = (
            [actuator_type(self.config, vehicle, index)
             for index, vehicle in enumerate(self.config.vehicles)]
            if actuator_type is not None else []
        )
        self.render = bool(render)
        self.controller = CrazySwarmSlungController(self.config)
        if self.config.payload_state_estimator not in (
            "observer", "second_order_low_pass", "mujoco_truth"
        ):
            raise ValueError(
                "payload_state_estimator must be 'observer', 'second_order_low_pass', "
                "or 'mujoco_truth'"
            )
        self.payload_state_observer = PayloadStateObserver(
            position_gain=self.config.payload_observer_position_gain,
            velocity_gain=self.config.payload_observer_velocity_gain,
            acceleration_gain=self.config.payload_observer_acceleration_gain,
            attitude_gain=self.config.payload_observer_attitude_gain,
            angular_rate_gain=self.config.payload_observer_angular_rate_gain,
            max_dt=self.config.payload_observer_max_dt,
            max_velocity_mps=self.config.payload_observer_max_velocity_mps,
            max_acceleration_mps2=self.config.payload_observer_max_acceleration_mps2,
            max_body_rate_rps=self.config.payload_observer_max_body_rate_rps,
            min_samples=self.config.payload_observer_min_samples,
        )
        self.payload_pose_filter = PayloadPoseLowPassFilter(
            cutoff_hz=self.config.payload_velocity_filter_cutoff_hz,
            max_dt=self.config.payload_observer_max_dt,
            min_samples=self.config.payload_observer_min_samples,
        )
        pose_sample_rate = float(self.config.payload_pose_sample_rate_hz)
        if not np.isfinite(pose_sample_rate) or pose_sample_rate <= 0.0:
            raise ValueError("payload_pose_sample_rate_hz must be positive")
        self._payload_pose_sample_period = 1.0 / pose_sample_rate
        if self._payload_pose_sample_period > self.config.payload_observer_max_dt:
            raise ValueError("payload pose sample period must not exceed observer max_dt")
        self._last_payload_pose_sample_time = None
        self._payload_controller_state = None
        self._payload_truth_sample_count = 0
        self._mjMdl = MujocoModel(
            model_path=None,
            model_xml=build_mjcf(self.config),
            render=self.render,
            record=record,
        )
        self.model = self._mjMdl.model
        self.data = self._mjMdl.data
        self._csv_logger = CrazySwarmSlungCsvLogger(csv_path) if csv_path is not None else None
        self.t = 0.0
        self.phase = CrazySwarmSlungPhase.GROUND_SLACK
        self._phase_started = 0.0
        self._geometry_ready_since = None
        self._activation_ready_since = None
        self._landing_settle_since = None
        self._release_source_positions = None
        self._ramp_vehicle_hold_positions = None
        self._ramp_payload_hold_position = None
        self._phase_names = []
        self._phase_history = []
        self._actual_thrust = np.zeros(3)
        self._max_payload_height = 0.0
        self._fault_since = None

        self._load_body_id = self._name_id(mujoco.mjtObj.mjOBJ_BODY, "payload")
        self._load_joint_id = self._name_id(mujoco.mjtObj.mjOBJ_JOINT, "payload_free")
        self._load_qpos = int(self.model.jnt_qposadr[self._load_joint_id])
        self._load_dof = int(self.model.jnt_dofadr[self._load_joint_id])
        self._ground_geom_id = self._name_id(mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self._vehicle_body_ids = [
            self._name_id(mujoco.mjtObj.mjOBJ_BODY, item.name) for item in self.config.vehicles
        ]
        self._vehicle_geom_ids = [
            self._name_id(mujoco.mjtObj.mjOBJ_GEOM, f"{item.name}_body")
            for item in self.config.vehicles
        ]
        self._vehicle_joint_ids = [
            self._name_id(mujoco.mjtObj.mjOBJ_JOINT, f"{item.name}_free")
            for item in self.config.vehicles
        ]
        self._vehicle_qpos = [int(self.model.jnt_qposadr[item]) for item in self._vehicle_joint_ids]
        self._vehicle_dof = [int(self.model.jnt_dofadr[item]) for item in self._vehicle_joint_ids]
        self._anchor_site_ids = [
            self._name_id(mujoco.mjtObj.mjOBJ_SITE, f"{item.name}_cable_anchor")
            for item in self.config.vehicles
        ]
        self._attachment_site_ids = [
            self._name_id(mujoco.mjtObj.mjOBJ_SITE, f"payload_attach_{index}")
            for index in range(3)
        ]
        self._tendon_ids = [
            self._name_id(mujoco.mjtObj.mjOBJ_TENDON, f"cable_{item.name}")
            for item in self.config.vehicles
        ]
        self._load_initial_position = self.data.qpos[self._load_qpos : self._load_qpos + 3].copy()
        self._initial_vehicle_positions = np.array(
            [self.data.qpos[qpos : qpos + 3].copy() for qpos in self._vehicle_qpos]
        )
        self._takeoff_targets = self._initial_vehicle_positions.copy()
        self._takeoff_targets[:, 2] = self.config.independent_hover_height
        self._takeup_targets = self._make_takeup_targets()
        self._landing_targets = self._initial_vehicle_positions.copy()
        self._phase_start_payload = self._load_initial_position.copy()
        self._hover_target_payload = self._load_initial_position.copy()
        self._landing_duration = self.config.landing_minimum_duration
        self._landing_release_earliest = 0.0
        self.reset()

    def _name_id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"MuJoCo model is missing named object {name!r}")
        return object_id

    def _make_takeup_targets(self):
        points = self.config.attachment_array
        centroid = np.mean(points[:, :2], axis=0)
        result = np.zeros((3, 3))
        payload_center = np.array([0.0, 0.0, self.config.payload_size[2] / 2.0])
        for index, point in enumerate(points):
            radial = np.array([point[0] - centroid[0], point[1] - centroid[1], 0.0])
            radial = radial / max(float(np.linalg.norm(radial)), 1.0e-12)
            horizontal = self.config.takeup_outward_offset * radial[:2]
            vertical = np.sqrt(max(self.config.cable_lengths[index] ** 2 - float(horizontal @ horizontal), 0.0))
            result[index] = payload_center + point + np.array(
                [horizontal[0], horizontal[1], vertical]
            )
        return result

    def reset(self, position=None):
        mujoco.mj_resetData(self.model, self.data)
        self.model.tendon_limited[self._tendon_ids] = 1
        if position is not None:
            requested = np.asarray(position, dtype=float).reshape(3)
            offset = requested - self._load_initial_position
            self.data.qpos[self._load_qpos : self._load_qpos + 3] = requested
            for qpos in self._vehicle_qpos:
                self.data.qpos[qpos : qpos + 3] += offset
            self._takeoff_targets += offset
            self._takeup_targets += offset
            self._landing_targets += offset
            self._load_initial_position = requested.copy()
        else:
            self._load_initial_position = self.data.qpos[self._load_qpos : self._load_qpos + 3].copy()
            self._initial_vehicle_positions = np.array(
                [self.data.qpos[qpos : qpos + 3].copy() for qpos in self._vehicle_qpos]
            )
            self._takeoff_targets = self._initial_vehicle_positions.copy()
            self._takeoff_targets[:, 2] = self.config.independent_hover_height
            self._takeup_targets = self._make_takeup_targets()
            self._landing_targets = self._initial_vehicle_positions.copy()
        mujoco.mj_forward(self.model, self.data)
        self.controller.reset()
        self.t = 0.0
        self.phase = CrazySwarmSlungPhase.GROUND_SLACK
        self._phase_started = 0.0
        self._geometry_ready_since = None
        self._activation_ready_since = None
        self._landing_settle_since = None
        self._release_source_positions = None
        self._phase_names = [self.phase.value]
        self._phase_history = [self._phase_snapshot()]
        self.reset_actuators()
        self._max_payload_height = float(self.data.xpos[self._load_body_id, 2])
        self._fault_since = None
        self._phase_start_payload = self._load_initial_position.copy()
        self._hover_target_payload = self._load_initial_position.copy()
        self._landing_duration = self.config.landing_minimum_duration
        self._landing_release_earliest = 0.0
        self.payload_state_observer.reset()
        self.payload_pose_filter.reset()
        self._last_payload_pose_sample_time = None
        self._payload_truth_sample_count = 0
        self._update_payload_controller_state(force=True)
        return self

    def reset_actuators(self, thrusts=None):
        """Reset onboard controller/motor state, optionally at collective thrust."""
        thrusts = np.zeros(len(self.config.vehicles)) if thrusts is None else np.asarray(thrusts, dtype=float)
        if (thrusts.shape != (len(self.config.vehicles),) or
                not np.all(np.isfinite(thrusts)) or np.any(thrusts < 0.0)):
            raise ValueError("thrusts must contain one finite nonnegative value per vehicle")
        self._actual_thrust[:] = thrusts
        for index, actuator in enumerate(self._onboard_actuators):
            rotation = self._rotation(self._vehicle_body_ids[index])
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            actuator.reset(yaw_rad=yaw, thrust=thrusts[index])
            self._actual_thrust[index] = float(np.sum(actuator.motor_forces))

    def close(self):
        viewer = getattr(self._mjMdl, "_viewer", None)
        try:
            if viewer is not None:
                viewer.close()
        finally:
            if self._csv_logger is not None:
                self._csv_logger.close()

    def _rotation(self, body_id):
        return self.data.xmat[body_id].reshape(3, 3).copy()

    def _body_state(self, body_id, dof, anchor_site=None):
        position = self.data.xpos[body_id].copy()
        rotation = self._rotation(body_id)
        velocity = self.data.qvel[dof : dof + 3].copy()
        body_rate = self.data.qvel[dof + 3 : dof + 6].copy()
        result = {
            "position": position,
            "velocity": velocity,
            "acceleration": self.data.qacc[dof : dof + 3].copy(),
            "rotation": rotation,
            "body_rate": body_rate,
        }
        if anchor_site is not None:
            anchor_position = self.data.site_xpos[anchor_site].copy()
            offset = anchor_position - position
            anchor_velocity = velocity + np.cross(rotation @ body_rate, offset)
            result["anchor_position"] = anchor_position
            result["anchor_velocity"] = anchor_velocity
        return result

    def payload_state(self):
        return self._body_state(self._load_body_id, self._load_dof)

    def _update_payload_controller_state(self, force=False):
        """Sample payload pose at the mocap rate and estimate its derivatives."""
        timestamp = float(self.data.time)
        estimator_mode = self.config.payload_state_estimator
        if (
            force
            or estimator_mode == "mujoco_truth"
            or self._last_payload_pose_sample_time is None
            or timestamp - self._last_payload_pose_sample_time
            >= self._payload_pose_sample_period - 1.0e-12
        ):
            measured = self.payload_state()
            if estimator_mode == "mujoco_truth":
                self._payload_truth_sample_count += 1
                observed = {
                    "position": measured["position"],
                    "velocity": measured["velocity"],
                    "acceleration": measured["acceleration"],
                    "body_rate": measured["body_rate"],
                    "derivatives_valid": True,
                    "sample_count": self._payload_truth_sample_count,
                }
            elif estimator_mode == "second_order_low_pass":
                observed = self.payload_pose_filter.update(
                    measured["position"], measured["rotation"], timestamp
                )
                if observed is None:
                    raise RuntimeError("payload low-pass filter rejected the MuJoCo sample")
            else:
                observed = self.payload_state_observer.update(
                    measured["position"], measured["rotation"], timestamp
                )
                if observed is None:
                    raise RuntimeError("payload state observer rejected the MuJoCo sample")
            self._payload_controller_state = {
                "valid": True,
                "position": observed.get("position", measured["position"]).copy(),
                "rotation": measured["rotation"].copy(),
                "velocity": observed["velocity"],
                "acceleration": observed["acceleration"],
                "body_rate": observed["body_rate"],
                "derivatives_valid": observed["derivatives_valid"],
                "sample_count": observed["sample_count"],
                "sample_time": timestamp,
            }
            self._last_payload_pose_sample_time = timestamp
        return {
            key: value.copy() if isinstance(value, np.ndarray) else value
            for key, value in self._payload_controller_state.items()
        }

    def vehicle_states(self):
        return [
            self._body_state(body, dof, site)
            for body, dof, site in zip(
                self._vehicle_body_ids,
                self._vehicle_dof,
                self._anchor_site_ids,
                strict=True,
            )
        ]

    def _current_cable_geometry(self):
        anchors = self.data.site_xpos[self._anchor_site_ids]
        attachments = self.data.site_xpos[self._attachment_site_ids]
        vectors = attachments - anchors
        lengths = np.linalg.norm(vectors, axis=1)
        units = vectors / np.maximum(lengths[:, None], 1.0e-12)
        targets = self._takeup_targets
        target_vectors = attachments - targets
        target_units = target_vectors / np.maximum(
            np.linalg.norm(target_vectors, axis=1)[:, None], 1.0e-12
        )
        cosine = np.sum(units * target_units, axis=1)
        angles = np.arccos(np.clip(cosine, -1.0, 1.0))
        return lengths, angles

    def _set_phase(self, phase):
        if phase is self.phase:
            return
        self.phase = phase
        self._phase_started = self.t
        self._phase_names.append(phase.value)
        self._geometry_ready_since = None
        self._activation_ready_since = None
        if phase is CrazySwarmSlungPhase.TENSION_RAMP:
            states = self.vehicle_states()
            self._ramp_vehicle_hold_positions = np.asarray(
                [state["position"] for state in states], dtype=float
            )
            self._ramp_payload_hold_position = self.payload_state()["position"].copy()
            self.controller.independent_integrals[:] = 0.0
        elif phase is CrazySwarmSlungPhase.REFERENCE_LIFT:
            self._phase_start_payload = self.payload_state()["position"].copy()
            self._hover_target_payload = self._phase_start_payload.copy()
            self._hover_target_payload[2] = self.config.hover_height
        elif phase is CrazySwarmSlungPhase.HOVER:
            self._phase_start_payload = self._hover_target_payload.copy()
        elif phase is CrazySwarmSlungPhase.LANDING_TAUT:
            self._phase_start_payload = self.payload_state()["position"].copy()
            ground_center = self.config.payload_size[2] / 2.0
            descent = abs(self._phase_start_payload[2] - ground_center)
            self._landing_duration = max(
                self.config.landing_minimum_duration,
                1.875 * descent / self.config.landing_max_speed,
            )
            self._landing_release_earliest = (
                self.t
                + self.config.landing_minimum_duration
                * self.config.landing_approach_fraction
            )
        elif phase is CrazySwarmSlungPhase.LANDING_RELEASE:
            self.model.tendon_limited[self._tendon_ids] = 0
            self.controller.independent_integrals[:] = 0.0
            self._release_source_positions = np.array(
                [self.data.qpos[qpos : qpos + 3].copy() for qpos in self._vehicle_qpos]
            )
            self._landing_targets = self._initial_vehicle_positions.copy()
            self._landing_targets[:, 2] = self.config.body_size[2] / 2.0
            max_distance = float(
                np.max(np.linalg.norm(self._landing_targets - self._release_source_positions, axis=1))
            )
            self._landing_duration = max(
                self.config.landing_minimum_duration,
                1.875 * max_distance / self.config.landing_max_speed,
            )
        self._phase_history.append(self._phase_snapshot())

    def _phase_snapshot(self):
        if not hasattr(self, "_load_body_id"):
            return {"phase": self.phase.value, "time": self.t}
        return {
            "phase": self.phase.value,
            "time": self.t,
            "payload_position": self.data.xpos[self._load_body_id].copy(),
            "cable_lengths": self.data.ten_length.copy(),
        }

    def _independent_reference(self):
        elapsed = self.t - self._phase_started
        if self.phase is CrazySwarmSlungPhase.INDEPENDENT_TAKEOFF:
            source = self._initial_vehicle_positions
            target = self._takeoff_targets
            duration = self.config.independent_takeoff_duration
        elif self.phase is CrazySwarmSlungPhase.TAKEUP:
            source = self._takeoff_targets
            target = self._takeup_targets
            duration = self.config.takeup_duration
        elif self.phase is CrazySwarmSlungPhase.LANDING_RELEASE:
            source = self._release_source_positions
            target = self._landing_targets
            duration = self._landing_duration
        else:
            source = self._initial_vehicle_positions
            target = self._landing_targets
            duration = self._landing_duration
        profile = quintic_profile(elapsed, duration)
        displacement = target - source
        positions = source + profile[0] * displacement
        velocities = profile[1] * displacement
        return positions, velocities

    def _payload_reference(self):
        if self.phase is CrazySwarmSlungPhase.REFERENCE_LIFT:
            elapsed = self.t - self._phase_started
            target = self._phase_start_payload.copy()
            target[2] = self.config.hover_height
            profile = quintic_profile(elapsed, self.config.reference_lift_duration)
            displacement = target - self._phase_start_payload
            return {
                "position": self._phase_start_payload + profile[0] * displacement,
                "velocity": profile[1] * displacement,
                "acceleration": profile[2] * displacement,
                "rotation": np.eye(3),
            }
        if self.phase is CrazySwarmSlungPhase.LANDING_TAUT:
            elapsed = self.t - self._phase_started
            target = self._phase_start_payload.copy()
            target[2] = self.config.payload_size[2] / 2.0
            profile = quintic_profile(elapsed, self._landing_duration)
            displacement = target - self._phase_start_payload
            return {
                "position": self._phase_start_payload + profile[0] * displacement,
                "velocity": profile[1] * displacement,
                "acceleration": profile[2] * displacement,
                "rotation": np.eye(3),
            }
        position = self._phase_start_payload.copy()
        if self.phase is CrazySwarmSlungPhase.TENSION_RAMP:
            position = (
                self._ramp_payload_hold_position.copy()
                if self._ramp_payload_hold_position is not None
                else self._load_initial_position.copy()
            )
        elif self.phase is CrazySwarmSlungPhase.HOVER:
            position = self._hover_target_payload.copy()
        return {
            "position": position,
            "velocity": np.zeros(3),
            "acceleration": np.zeros(3),
            "rotation": np.eye(3),
        }

    def _contact(self, geom_id):
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if {int(contact.geom1), int(contact.geom2)} == {self._ground_geom_id, geom_id}:
                return True
        return False

    def _all_states_finite(self):
        return bool(np.all(np.isfinite(self.data.qpos)) and np.all(np.isfinite(self.data.qvel)))

    def _cable_tensions(self):
        tensions = np.zeros(len(self._tendon_ids))
        tendon_limit_type = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)
        for index, tendon_id in enumerate(self._tendon_ids):
            rows = np.flatnonzero(
                (self.data.efc_type[: self.data.nefc] == tendon_limit_type)
                & (self.data.efc_id[: self.data.nefc] == tendon_id)
            )
            if rows.size:
                tensions[index] = float(np.maximum(self.data.efc_force[rows], 0.0).sum())
        return tensions

    def _write_csv_sample(
        self,
        control_mode,
        commands,
        vehicle_targets,
        vehicle_target_velocities,
        payload_target,
        cooperative,
        cooperative_blend,
        payload_controller_state,
    ):
        if self._csv_logger is None:
            return

        payload = self.payload_state()
        payload_observed = payload_controller_state
        vehicles = self.vehicle_states()
        cable_lengths = self.data.ten_length[self._tendon_ids]
        cable_velocities = self.data.ten_velocity[self._tendon_ids]
        cable_tensions = self._cable_tensions()
        attachment_positions = self.data.site_xpos[self._attachment_site_ids]
        anchor_positions = self.data.site_xpos[self._anchor_site_ids]
        actual_links = attachment_positions - anchor_positions
        actual_links /= np.maximum(np.linalg.norm(actual_links, axis=1)[:, None], 1.0e-12)

        desired_links = np.full((3, 3), np.nan)
        if cooperative is not None:
            desired_links = np.column_stack(
                [z_down_to_up(cooperative["desired_link_units"][:, index]) for index in range(3)]
            ).T
        elif vehicle_targets is not None:
            desired_rotation = payload_target.get("rotation", np.eye(3))
            attachments = (
                payload_target["position"]
                + (desired_rotation @ self.config.attachment_array.T).T
            )
            desired_links = attachments - vehicle_targets
            desired_links /= np.maximum(np.linalg.norm(desired_links, axis=1)[:, None], 1.0e-12)

        payload_target_position = np.asarray(payload_target["position"], dtype=float)
        payload_target_velocity = np.asarray(payload_target.get("velocity", np.zeros(3)), dtype=float)
        payload_target_acceleration = np.asarray(
            payload_target.get("acceleration", np.zeros(3)), dtype=float
        )
        payload_position_error = payload["position"] - payload_target_position
        payload_velocity_error = payload["velocity"] - payload_target_velocity
        payload_observed_position_error = (
            payload_observed["position"] - payload_target_position
        )
        payload_observed_velocity_error = (
            payload_observed["velocity"] - payload_target_velocity
        )
        if cooperative is None:
            payload_attitude_error = np.full(3, np.nan)
            payload_desired_force = np.full(3, np.nan)
            payload_desired_moment = np.full(3, np.nan)
            desired_tensions = np.full(3, np.nan)
        else:
            payload_attitude_error = cooperative["attitude_error_world"]
            payload_desired_force = cooperative["desired_force_world"]
            payload_desired_moment = cooperative["desired_moment_world"]
            desired_tensions = cooperative["desired_tensions"]

        config = self.config
        for index, (state, params) in enumerate(zip(vehicles, config.vehicles, strict=True)):
            command = commands[index] if commands is not None else None
            row = {
                "control_time_s": self.t,
                "phase_time_s": self.t - self._phase_started,
                "flight_phase": self.phase.value,
                "control_mode": control_mode,
                "cooperative_blend": cooperative_blend,
                "vehicle_id": params.name,
                "roll_rad": _rotation_euler_xyz(state["rotation"])[0],
                "pitch_rad": _rotation_euler_xyz(state["rotation"])[1],
                "yaw_rad": _rotation_euler_xyz(state["rotation"])[2],
                "command_thrust_newton": command.thrust if command is not None else 0.0,
                "actual_thrust_newton": self._actual_thrust[index],
                "cable_length_m": cable_lengths[index],
                "cable_length_error_m": cable_lengths[index] - config.cable_lengths[index],
                "cable_velocity_mps": cable_velocities[index],
                "cable_tension_n": cable_tensions[index],
                "desired_tension_n": desired_tensions[index],
                "payload_roll_rad": _rotation_euler_xyz(payload["rotation"])[0],
                "payload_pitch_rad": _rotation_euler_xyz(payload["rotation"])[1],
                "payload_yaw_rad": _rotation_euler_xyz(payload["rotation"])[2],
                "payload_observed_roll_rad": _rotation_euler_xyz(
                    payload_observed["rotation"]
                )[0],
                "payload_observed_pitch_rad": _rotation_euler_xyz(
                    payload_observed["rotation"]
                )[1],
                "payload_observed_yaw_rad": _rotation_euler_xyz(
                    payload_observed["rotation"]
                )[2],
                "payload_observation_derivatives_valid": int(
                    payload_observed["derivatives_valid"]
                ),
                "payload_observation_sample_count": payload_observed["sample_count"],
                "payload_observation_sample_time_s": payload_observed["sample_time"],
                "payload_pose_sample_rate_hz": config.payload_pose_sample_rate_hz,
                "payload_state_estimator": config.payload_state_estimator,
                "payload_velocity_filter_cutoff_hz": config.payload_velocity_filter_cutoff_hz,
                "payload_observer_position_gain": config.payload_observer_position_gain,
                "payload_observer_velocity_gain": config.payload_observer_velocity_gain,
                "payload_observer_acceleration_gain": config.payload_observer_acceleration_gain,
                "payload_observer_attitude_gain": config.payload_observer_attitude_gain,
                "payload_observer_angular_rate_gain": config.payload_observer_angular_rate_gain,
                "payload_observer_max_dt_s": config.payload_observer_max_dt,
                "vehicle_mass_kg": params.mass,
                "vehicle_max_total_thrust_n": params.max_total_thrust,
                "vehicle_max_command_thrust_n": params.max_command_thrust,
                "vehicle_min_flight_thrust_n": params.min_flight_thrust,
                "vehicle_max_tilt_rad": params.max_tilt_rad,
                "independent_integral_gate_m": params.independent_integral_gate,
                "payload_mass_kg": config.payload_mass,
                "payload_attitude_damping": config.payload_attitude_damping,
                "payload_integral_c1": config.payload_integral_c1,
                "payload_yaw_enabled": config.payload_yaw_enabled,
                "link_kq": config.link_kq,
                "link_komega": config.link_komega,
                "link_ki": config.link_integral_gain,
                "tension_pinv_tolerance": config.tension_pinv_tolerance,
                "transport_link_gain_scale": config.transport_link_gain_scale,
                "prop_guard_outer_diameter_m": config.prop_guard_outer_diameter,
                "prop_guard_bar_diameter_m": 2.0 * config.prop_guard_bar_radius,
                "prop_guard_z_offset_m": config.prop_guard_z_offset,
                "prop_guard_segments": config.prop_guard_segments,
                "ramp_position_hold_gain_scale": config.ramp_position_hold_gain_scale,
                "outward_bias_fraction": config.outward_bias_fraction,
                "outward_bias_max_n": config.outward_bias_max_n,
                "thrust_time_constant_s": config.thrust_time_constant,
                "control_rate_hz": config.control_rate_hz,
                "physics_timestep_s": config.timestep,
                "onboard_controller": config.onboard_controller,
                "motor_model": config.motor_model,
                "brushless_motor_time_constant_s": config.brushless_motor_time_constant_s,
                "brushless_motor_speed_gain_radps": config.brushless_motor_speed_gain_radps,
                "brushless_rotor_inertia_kgm2": config.brushless_rotor_inertia_kgm2,
                "bridge_collective_pwm_reference": config.bridge_collective_pwm_reference,
                "bridge_collective_pwm_limit": config.bridge_collective_pwm_limit,
                "motor_pwm_max": config.motor_pwm_max,
                "brushless_speed_normalization_radps": config.brushless_speed_normalization_radps,
                "bridge_thrust_curve_exponent": config.bridge_thrust_curve_exponent,
                "bridge_thrust_raw_scale": config.bridge_thrust_raw_scale[index],
                "motor_available_command_fraction": config.motor_available_command_fraction[index],
                "motor_thrust_efficiency": config.motor_thrust_efficiency[index],
                "vehicle_arm_length_m": config.arm_length,
                "actuator_state_time_s": float(self.data.time),
            }
            for power, thrust_coefficient, torque_coefficient in zip(
                    (3, 2, 1), config.brushless_thrust_coefficients, config.brushless_torque_coefficients):
                row[f"brushless_thrust_coefficient_{power}"] = thrust_coefficient
                row[f"brushless_torque_coefficient_{power}"] = torque_coefficient
            row.update({f"vehicle_inertia_{axis}": value for axis, value in zip("xyz", config.vehicle_inertia)})
            actuator = self._onboard_actuators[index] if self._onboard_actuators else None
            row["collective_raw_thrust"] = actuator.raw_collective if actuator is not None else np.nan
            row["applied_thrust_command_n"] = actuator.applied_thrust_command if actuator is not None else np.nan
            row["motor_at_upper_limit_count"] = int(np.sum(
                actuator.motor_pwm >= actuator.motor_pwm_limit)) if actuator is not None else 0
            for prefix, attribute in (("motor_pwm", "motor_pwm"),
                                      ("motor_speed_radps", "motor_speeds"),
                                      ("motor_force_n", "motor_forces"),
                                      ("motor_reaction_torque_nm", "motor_reaction_torques")):
                values = getattr(actuator, attribute) if actuator is not None else np.full(4, np.nan)
                for motor_index, value in enumerate(values, 1):
                    row[f"{prefix}_{motor_index}"] = value
            for axis, value in zip(("x", "y", "z"), config.rate_loop_bandwidth, strict=True):
                row[f"rate_loop_bandwidth_{axis}"] = value

            def add_vector(prefix, vector):
                if vector is None:
                    return
                for axis, value in zip(("x", "y", "z"), vector, strict=True):
                    row[f"{prefix}_{axis}"] = value

            add_vector("position", state["position"])
            add_vector("velocity", state["velocity"])
            add_vector("acceleration", state["acceleration"])
            add_vector("body_rate", state["body_rate"])
            add_vector("payload_position", payload["position"])
            add_vector("payload_velocity", payload["velocity"])
            add_vector("payload_acceleration", payload["acceleration"])
            add_vector("payload_body_rate", payload["body_rate"])
            add_vector("payload_observed_position", payload_observed["position"])
            add_vector("payload_observed_velocity", payload_observed["velocity"])
            add_vector("payload_observed_acceleration", payload_observed["acceleration"])
            add_vector("payload_observed_body_rate", payload_observed["body_rate"])
            add_vector("payload_observer_max_velocity", config.payload_observer_max_velocity_mps)
            add_vector(
                "payload_observer_max_acceleration",
                config.payload_observer_max_acceleration_mps2,
            )
            add_vector("payload_observer_max_body_rate", config.payload_observer_max_body_rate_rps)
            add_vector("payload_target", payload_target_position)
            add_vector("payload_target_velocity", payload_target_velocity)
            add_vector("payload_target_acceleration", payload_target_acceleration)
            add_vector("payload_position_error", payload_position_error)
            add_vector("payload_velocity_error", payload_velocity_error)
            add_vector("payload_observed_position_error", payload_observed_position_error)
            add_vector("payload_observed_velocity_error", payload_observed_velocity_error)
            add_vector("payload_attitude_error", payload_attitude_error)
            add_vector("payload_desired_force", payload_desired_force)
            add_vector("payload_desired_moment", payload_desired_moment)
            add_vector(
                "payload_position_integral",
                z_down_to_up(self.controller.payload_position_integral),
            )
            add_vector("link_direction", actual_links[index])
            add_vector("desired_link_direction", desired_links[index])
            add_vector("link_direction_error", desired_links[index] - actual_links[index])
            if vehicle_targets is not None:
                add_vector("vehicle_target", vehicle_targets[index])
                add_vector("position_error", state["position"] - vehicle_targets[index])
                add_vector("vehicle_target_velocity", vehicle_target_velocities[index])
                add_vector(
                    "velocity_error", state["velocity"] - vehicle_target_velocities[index]
                )
            if command is not None:
                add_vector("command_rate", command.body_rate)
                add_vector("desired_force", command.desired_force_world)
            add_vector("actual_force", self._actual_thrust[index] * state["rotation"][:, 2])
            for axis, value in zip(("x", "y", "z"), config.payload_position_gain, strict=True):
                row[f"payload_kp_{axis}"] = value
            for axis, value in zip(("x", "y", "z"), config.payload_velocity_gain, strict=True):
                row[f"payload_kd_{axis}"] = value
            for axis, value in zip(("x", "y", "z"), config.payload_integral_gain, strict=True):
                row[f"payload_ki_{axis}"] = value
            for axis, value in zip(
                ("x", "y", "z"), config.payload_attitude_bandwidth_hz, strict=True
            ):
                row[f"payload_attitude_bandwidth_hz_{axis}"] = value
            for prefix, vector in (
                ("vehicle_kp", params.position_gain),
                ("vehicle_kd", params.velocity_gain),
                ("vehicle_ki", params.integral_gain),
                ("vehicle_integral_limit", params.integral_limit),
                ("independent_kp", params.independent_position_gain),
                ("independent_kd", params.independent_velocity_gain),
                ("independent_ki", params.independent_integral_gain),
                ("independent_integral_limit", params.independent_integral_limit),
                (
                    "independent_max_feedback_acceleration",
                    params.independent_max_feedback_acceleration,
                ),
                ("independent_attitude_gain", params.independent_attitude_gain),
                ("transport_attitude_gain", params.transport_attitude_gain),
                ("transport_rate_gain", params.transport_rate_gain),
                ("max_body_rate", params.max_body_rate),
                ("payload_integral_limit", config.payload_integral_limit),
                ("link_integral_limit", config.link_integral_limit),
            ):
                add_vector(prefix, vector)
            self._csv_logger.write(row)

    def _update_phase(self):
        elapsed = self.t - self._phase_started
        if self.phase is CrazySwarmSlungPhase.GROUND_SLACK:
            if elapsed >= self.config.ground_slack_duration:
                self._set_phase(CrazySwarmSlungPhase.INDEPENDENT_TAKEOFF)
        elif self.phase is CrazySwarmSlungPhase.INDEPENDENT_TAKEOFF:
            if elapsed >= self.config.independent_takeoff_duration:
                self._set_phase(CrazySwarmSlungPhase.TAKEUP)
        elif self.phase is CrazySwarmSlungPhase.TAKEUP:
            lengths, angles = self._current_cable_geometry()
            distance_ok = np.all(
                np.abs(lengths - np.asarray(self.config.cable_lengths))
                <= self.config.takeup_distance_tolerance
            )
            angle_ok = np.all(angles <= self.config.takeup_angle_tolerance_rad)
            ready = elapsed >= self.config.takeup_duration and distance_ok and angle_ok
            if ready:
                if self._geometry_ready_since is None:
                    self._geometry_ready_since = self.t
                elif self.t - self._geometry_ready_since >= self.config.takeup_confirm_duration:
                    if self._activation_ready_since is None:
                        self._activation_ready_since = self.t
                    elif self.t - self._activation_ready_since >= self.config.activation_hold_duration:
                        self._set_phase(CrazySwarmSlungPhase.TENSION_RAMP)
            else:
                self._geometry_ready_since = None
                self._activation_ready_since = None
        elif self.phase is CrazySwarmSlungPhase.TENSION_RAMP:
            if elapsed >= self.config.tension_ramp_duration:
                self._set_phase(CrazySwarmSlungPhase.REFERENCE_LIFT)
        elif self.phase is CrazySwarmSlungPhase.REFERENCE_LIFT:
            if elapsed >= self.config.reference_lift_duration:
                self._set_phase(CrazySwarmSlungPhase.HOVER)
        elif self.phase is CrazySwarmSlungPhase.HOVER:
            if elapsed >= self.config.hover_duration:
                self._set_phase(CrazySwarmSlungPhase.LANDING_TAUT)
        elif self.phase is CrazySwarmSlungPhase.LANDING_TAUT:
            ground_contact = self._contact(self._name_id(mujoco.mjtObj.mjOBJ_GEOM, "payload_geom"))
            if self.t >= self._landing_release_earliest and ground_contact:
                self._set_phase(CrazySwarmSlungPhase.LANDING_RELEASE)
        elif self.phase is CrazySwarmSlungPhase.LANDING_RELEASE:
            contacts = [self._contact(geom) for geom in self._vehicle_geom_ids]
            speeds = [np.linalg.norm(state["velocity"]) for state in self.vehicle_states()]
            if all(contacts) and max(speeds) <= self.config.landing_velocity_tolerance:
                if self._landing_settle_since is None:
                    self._landing_settle_since = self.t
                elif self.t - self._landing_settle_since >= self.config.landing_settle_duration:
                    self._set_phase(CrazySwarmSlungPhase.LANDED)
            else:
                self._landing_settle_since = None

    def step(self):
        """Update the external controller once, then hold its command for one tick."""
        dt = self.control_dt
        self.data.xfrc_applied[:] = 0.0
        self.data.ctrl[:] = 0.0
        control_mode = "none"
        commands = None
        vehicle_targets = None
        vehicle_target_velocities = None
        cooperative = None
        cooperative_blend = float("nan")
        payload_controller_state = self._update_payload_controller_state()
        payload_target = {
            "position": self._load_initial_position.copy(),
            "velocity": np.zeros(3),
            "acceleration": np.zeros(3),
            "rotation": np.eye(3),
        }

        if self.phase is CrazySwarmSlungPhase.GROUND_SLACK:
            control_mode = "idle"
            self._actual_thrust[:] = 0.0
        elif self.phase in (
            CrazySwarmSlungPhase.INDEPENDENT_TAKEOFF,
            CrazySwarmSlungPhase.TAKEUP,
            CrazySwarmSlungPhase.LANDING_RELEASE,
        ):
            control_mode = "independent"
            vehicle_targets, vehicle_target_velocities = self._independent_reference()
            commands = self.controller.independent_commands(
                self.vehicle_states(), vehicle_targets, vehicle_target_velocities, dt
            )
        elif self.phase in (
            CrazySwarmSlungPhase.TENSION_RAMP,
            CrazySwarmSlungPhase.REFERENCE_LIFT,
            CrazySwarmSlungPhase.HOVER,
            CrazySwarmSlungPhase.LANDING_TAUT,
        ):
            control_mode = "cooperative"
            payload_target = self._payload_reference()
            vehicle_states = self.vehicle_states()
            cooperative = self.controller.cooperative_commands(
                payload_controller_state,
                vehicle_states,
                payload_target,
                dt,
                link_gain_scale=self.config.transport_link_gain_scale,
            )
            if self.phase is CrazySwarmSlungPhase.TENSION_RAMP:
                control_mode = "blended"
                mix = float(
                    quintic_profile(
                        self.t - self._phase_started,
                        self.config.tension_ramp_duration,
                    )[0]
                )
                independent, independent_velocities = self._ramp_independent_reference()
                independent_commands = self.controller.independent_commands(
                    vehicle_states, independent, independent_velocities, dt
                )
                commands = self._blend_commands(
                    independent_commands, cooperative["commands"], mix
                )
                commands = self._ramp_position_hold_commands(
                    commands, vehicle_states, independent, mix
                )
                vehicle_targets = independent
                vehicle_target_velocities = independent_velocities
                cooperative_blend = mix
            else:
                commands = cooperative["commands"]
        elif self.phase is CrazySwarmSlungPhase.EMERGENCY_LANDING:
            commands = self._emergency_land_step(dt)
        self._write_csv_sample(
            control_mode,
            commands,
            vehicle_targets,
            vehicle_target_velocities,
            payload_target,
            cooperative,
            cooperative_blend,
            payload_controller_state,
        )
        self.advance_physics(commands, dt)
        self.t = float(self.data.time)
        self._max_payload_height = max(
            self._max_payload_height, float(self.data.xpos[self._load_body_id, 2])
        )
        if not self._all_states_finite() and self.phase not in (
            CrazySwarmSlungPhase.LANDED,
            CrazySwarmSlungPhase.EMERGENCY_LANDING,
        ):
            self._set_phase(CrazySwarmSlungPhase.EMERGENCY_LANDING)
            self._fault_since = self.t
        elif self.phase is not CrazySwarmSlungPhase.EMERGENCY_LANDING:
            self._update_phase()
        if self._mjMdl._viewer is not None:
            self._mjMdl._viewer._overlay_text = f"{self.phase.value} | 1-4 VIEW F FOLLOW"
        return self.phase

    def _ramp_independent_reference(self):
        if self._ramp_vehicle_hold_positions is None:
            states = self.vehicle_states()
            self._ramp_vehicle_hold_positions = np.asarray(
                [state["position"] for state in states], dtype=float
            )
        return self._ramp_vehicle_hold_positions.copy(), np.zeros((3, 3))

    def _ramp_position_hold_commands(self, commands, vehicle_states, targets, blend):
        if blend <= 0.0:
            return commands
        held_commands = []
        for command, state, target, params in zip(
            commands, vehicle_states, targets, self.config.vehicles, strict=True
        ):
            if command.desired_force_world is None:
                held_commands.append(command)
                continue
            position_error = target - state["position"]
            feedback_acceleration = self.config.ramp_position_hold_gain_scale * (
                np.asarray(params.independent_position_gain) * position_error
                - np.asarray(params.independent_velocity_gain) * state["velocity"]
            )
            feedback_acceleration = np.clip(
                feedback_acceleration,
                -np.asarray(params.independent_max_feedback_acceleration),
                np.asarray(params.independent_max_feedback_acceleration),
            )
            # Keep the position-hold term active as the independent command fades out.
            desired_force_world = (
                command.desired_force_world
                + blend * params.mass * feedback_acceleration
            )
            held_commands.append(
                self.controller._force_to_command(
                    state,
                    params,
                    z_up_to_down(desired_force_world),
                    params.transport_attitude_gain,
                    params.transport_rate_gain,
                    minimum_thrust=params.min_flight_thrust,
                )
            )
        return held_commands

    @staticmethod
    def _blend_commands(independent, cooperative, blend):
        return [
            type(left)(
                thrust=(1.0 - blend) * left.thrust + blend * right.thrust,
                body_rate=(1.0 - blend) * left.body_rate + blend * right.body_rate,
                desired_force_world=(
                    (1.0 - blend) * left.desired_force_world
                    + blend * right.desired_force_world
                    if left.desired_force_world is not None
                    and right.desired_force_world is not None
                    else None
                ),
            )
            for left, right in zip(independent, cooperative, strict=True)
        ]

    def advance_physics(self, commands, duration=None):
        """Hold external commands while integrating actuators and 100 Hz observations.

        Shorten the last substep at a control/measurement deadline instead of
        rounding 1/70 s to seven 2 ms steps (which would run at 71.43 Hz).
        This also supports the ROS controller replay without invoking this
        model's separate mission/controller implementation.
        """
        duration = self.control_dt if duration is None else float(duration)
        if not np.isfinite(duration) or duration <= 0.0:
            raise ValueError("duration must be positive and finite")
        deadline = float(self.data.time) + duration
        original_dt = self.model.opt.timestep
        try:
            while deadline - self.data.time > 1.0e-12:
                dt = min(self.config.timestep, deadline - self.data.time)
                if self._onboard_actuators:
                    until_update = self._onboard_actuators[0].time_until_update
                    if until_update > 1.0e-12:
                        dt = min(dt, until_update)
                # Observe at the mocap clock, independently of the outer loop.
                next_sample = self._last_payload_pose_sample_time + self._payload_pose_sample_period
                if next_sample - self.data.time > 1.0e-12:
                    dt = min(dt, next_sample - self.data.time)
                self.model.opt.timestep = dt
                self.data.xfrc_applied[:] = 0.0
                self.data.ctrl[:] = 0.0
                if commands is not None:
                    self._apply_commands(commands, dt)
                payload_rate = self.data.qvel[self._load_dof + 3 : self._load_dof + 6]
                self.data.xfrc_applied[self._load_body_id, 3:] = self._rotation(
                    self._load_body_id
                ) @ (-self.config.payload_rotational_damping * payload_rate)
                self._mjMdl._step_mujoco_simulation(1)
                mujoco.mj_forward(self.model, self.data)
                self._update_payload_controller_state()
        finally:
            self.model.opt.timestep = original_dt

    def _apply_commands(self, commands, dt):
        for index, (command, body_id, dof, params) in enumerate(
            zip(commands, self._vehicle_body_ids, self._vehicle_dof, self.config.vehicles, strict=True)
        ):
            if self.config.onboard_controller == "pid":
                rotation = self._rotation(body_id)
                body_rate = self.data.qvel[dof + 3 : dof + 6].copy()
                yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
                thrust, torque_body = self._onboard_actuators[index].step(
                    command.thrust, command.body_rate, body_rate, yaw, dt
                )
                self._actual_thrust[index] = thrust
                self.data.xfrc_applied[body_id, :3] = thrust * rotation[:, 2]
                self.data.xfrc_applied[body_id, 3:] = rotation @ torque_body
                continue
            if self.config.onboard_controller == "mellinger":
                rotation = self._rotation(body_id)
                body_rate = self.data.qvel[dof + 3 : dof + 6].copy()
                yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
                thrust, torque_body = self._onboard_actuators[index].step(
                    command.thrust, command.body_rate, body_rate, yaw, rotation, dt
                )
                self._actual_thrust[index] = thrust
                self.data.xfrc_applied[body_id, :3] = thrust * rotation[:, 2]
                self.data.xfrc_applied[body_id, 3:] = rotation @ torque_body
                continue
            alpha = min(1.0, dt / self.config.thrust_time_constant)
            self._actual_thrust[index] += alpha * (command.thrust - self._actual_thrust[index])
            self._actual_thrust[index] = float(
                np.clip(self._actual_thrust[index], 0.0, params.max_total_thrust)
            )
            rotation = self._rotation(body_id)
            body_rate = self.data.qvel[dof + 3 : dof + 6].copy()
            rate_error = command.body_rate - body_rate
            bandwidth = np.asarray(self.config.rate_loop_bandwidth)
            body_rate_dot = bandwidth * rate_error
            inertia = np.diag(self.config.vehicle_inertia)
            torque_body = inertia @ body_rate_dot + np.cross(body_rate, inertia @ body_rate)
            force_world = self._actual_thrust[index] * rotation[:, 2]
            torque_world = rotation @ torque_body
            self.data.xfrc_applied[body_id, :3] = force_world
            self.data.xfrc_applied[body_id, 3:] = torque_world

    def _emergency_land_step(self, dt):
        elapsed = self.t - (self._fault_since if self._fault_since is not None else self.t)
        if elapsed <= self.config.payload_state_hold_duration:
            return
        if all(self._contact(geom) for geom in self._vehicle_geom_ids):
            if self._landing_settle_since is None:
                self._landing_settle_since = self.t
            elif self.t - self._landing_settle_since >= self.config.landing_settle_duration:
                self._set_phase(CrazySwarmSlungPhase.LANDED)
            return
        # The internal MuJoCo state is normally always valid; this bounded path
        # is retained for controller/reference faults and avoids continuing a
        # cooperative-force command after a detected invalid state.
        self.model.tendon_limited[self._tendon_ids] = 0
        states = self.vehicle_states()
        desired = self._landing_targets.copy()
        desired[:, 2] = self.config.body_size[2] / 2.0
        return self.controller.independent_commands(
            states, desired, np.zeros((3, 3)), dt
        )

    def simulate(self, tf=None, position=None):
        """Run until LANDED or a caller-specified time limit and return metrics."""
        self.reset(position=position)
        planned_duration = (
            self.config.ground_slack_duration
            + self.config.independent_takeoff_duration
            + self.config.takeup_duration
            + self.config.takeup_confirm_duration
            + self.config.activation_hold_duration
            + self.config.tension_ramp_duration
            + self.config.reference_lift_duration
            + self.config.hover_duration
            + max(self.config.landing_minimum_duration, 1.875 * self.config.hover_height / self.config.landing_max_speed)
            + max(self.config.landing_minimum_duration, 1.875 * 0.8 / self.config.landing_max_speed)
            + self.config.landing_settle_duration
            + 5.0
        )
        time_limit = planned_duration if tf is None else float(tf)
        start = time.perf_counter()
        while self.t < time_limit and self.phase is not CrazySwarmSlungPhase.LANDED:
            self.step()
        elapsed = time.perf_counter() - start
        if self._csv_logger is not None:
            self._csv_logger.flush()
        payload_geom = self._name_id(mujoco.mjtObj.mjOBJ_GEOM, "payload_geom")
        return {
            "phase": self.phase.value,
            "time": self.t,
            "wall_time": elapsed,
            "all_states_finite": self._all_states_finite(),
            "max_payload_height": self._max_payload_height,
            "payload_height": float(self.data.xpos[self._load_body_id, 2]),
            "payload_ground_contact": self._contact(payload_geom),
            "all_vehicles_ground_contact": all(self._contact(geom) for geom in self._vehicle_geom_ids),
            "cable_lengths": self.data.ten_length[self._tendon_ids].copy(),
            "csv_path": str(self._csv_logger.path) if self._csv_logger is not None else None,
            "control_rate_hz": self.config.control_rate_hz,
            "physics_timestep_s": self.config.timestep,
            "phases_seen": list(self._phase_names),
            "phase_history": self._phase_history,
        }
