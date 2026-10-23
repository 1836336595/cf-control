"""Payload handoff, supported landing, safety and lifecycle state."""

from copy import deepcopy
from ctbr_control_math import rotation_to_rpy
from slung_load_controller import payload_ground_clearance
from slung_load_controller import smoothstep5_profile
import math
import numpy as np
import rospy


class PayloadMissionMixin:
    """Payload handoff, supported landing, safety and lifecycle state."""

    @staticmethod
    def _finite_supported_command(command):
        if not command:
            return False
        try:
            force = np.asarray(command["desired_force"], dtype=float)
            rates = np.asarray(command.get("body_rate_command", np.zeros(3)), dtype=float)
            thrust = float(command["collective_thrust"])
            return (force.shape == (3,) and rates.shape == (3,)
                    and np.all(np.isfinite(force)) and np.all(np.isfinite(rates))
                    and math.isfinite(thrust) and thrust > 0. and force[2] > 0.)
        except (KeyError, TypeError, ValueError):
            return False

    @staticmethod
    def _finite_transport_override(target):
        vectors = ("position", "velocity", "acceleration", "position_hold_position",
                   "position_hold_velocity", "position_hold_acceleration",
                   "desired_force_override", "desired_force_dot_override",
                   "kinematic_specific_acceleration")
        scalars = ("transport_blend", "transport_blend_dot", "max_tilt_rad",
                   "transport_position_hold_gain_scale")
        try:
            return (target.get("transport_mode", False)
                    and all(np.asarray(target[key]).shape == (3,)
                            and np.all(np.isfinite(target[key])) for key in vectors)
                    and all(math.isfinite(float(target[key])) for key in scalars)
                    and all(np.all(np.isfinite(value)) for value in target.values()
                            if isinstance(value, np.ndarray)))
        except (KeyError, TypeError, ValueError):
            return False

    def _payload_planning_hold(self, now):
        """Freeze a complete supported reference for the bounded retry window."""
        overrides = {}
        for index, vehicle in enumerate(self.vehicles):
            previous = self.last_transport_overrides.get(id(vehicle), {})
            if self._finite_transport_override(previous):
                target = dict(previous)
            else:
                command = getattr(vehicle, "last_control_command", None)
                references = self.payload_transport_vehicle_start_positions
                if (not self._finite_supported_command(command) or references is None
                        or not np.all(np.isfinite(references[index]))):
                    return {}
                # A last valid actuator command is the support source when a
                # complete cooperative target has not yet been cached.
                target = dict(transport_mode=True, independent_mode=False,
                    transport_blend=1., position=references[index].copy(),
                    position_hold_position=references[index].copy(),
                    desired_force_override=command["desired_force"].copy(),
                    transport_position_hold_gain_scale=0.,
                    max_tilt_rad=vehicle.controller.config.max_tilt_rad,
                    kinematic_specific_acceleration=np.zeros(3))
            for key in ("velocity", "acceleration", "position_hold_velocity",
                        "position_hold_acceleration", "desired_force_dot_override",
                        "transport_hold_integral_force_dot"):
                target[key] = np.zeros(3)
            target.update(transport_blend_dot=0., transport_position_hold_gain_scale_dot=0.,
                transport_hold_integral_learning=False,
                transport_hold_integral_freeze_reason="landing_planner_retry",
                transport_full_pid_learning=False,
                transport_full_pid_freeze_reason="landing_planner_retry",
                transport_full_pid_tick=float(now), transport_full_pid_force_hold=True,
                transport_full_pid_weight_dot=0., transport_filter_reset=False)
            if not self._finite_transport_override(target):
                return {}
            overrides[id(vehicle)] = target
        self._refresh_cached_attitude_trim(now, overrides, "landing_planner_retry", freeze_base_derivatives=True)
        return overrides

    def _exit_payload_planning_fault(self, now, reason, source="landing_planner"):
        """Finish a failed retry window using the existing supported descent."""
        reason = source + ": " + str(reason)
        self.payload_controlled_exit_reason = "payload_load: " + reason
        if not all(self._finite_supported_command(getattr(v, "last_control_command", None))
                   for v in self.vehicles):
            self._set_abort(reason + "; no finite supported fleet command")
            return
        # Disable the cooperative planner before selecting the independent
        # emergency path, so recovery cannot immediately invoke it again.
        self.payload_transport_active = False
        self._begin_payload_emergency_landing(now, reason)
        self._reset_payload_handoff()
        self.payload_validation_landing_stage = "planning_fault_descent"

    def _payload_landing_planning_failed(self, now, error, payload_state):
        if self.payload_landing_retry_tick != float(now):
            self.payload_landing_retry_tick = float(now)
            self.payload_landing_failure_count += 1
            self.payload_landing_retry_count += 1
            if self.payload_landing_retry_since is None:
                self.payload_landing_retry_since = float(now)
            self.payload_landing_failure_reason = "%s: %s" % (type(error).__name__, error)
            if self.payload_landing_failure_snapshot is None:
                self.payload_landing_failure_snapshot = {
                    "start_position": np.asarray(payload_state["position"]).tolist(),
                    "start_velocity": np.asarray(payload_state.get("velocity", np.zeros(3))).tolist(),
                    "start_acceleration": np.asarray(payload_state.get("acceleration", np.zeros(3))).tolist(),
                    "ground_z": (self.payload_ground_center_z if math.isfinite(self.payload_ground_center_z)
                                 else .5 * float(self.payload_height_m)),
                    "now": float(now),
                    "min_duration_s": float(self.vehicles[0].trajectory_config.landing_duration_s),
                    "max_speed_mps": self.payload_landing_max_speed_mps,
                    "max_accel_mps2": self.payload_landing_max_acceleration_mps2,
                    "reason": self.payload_landing_failure_reason,
                }
                rospy.logerr("负载下降规划失败，暂时保留协同支撑并有界重试：%s", self.payload_landing_failure_snapshot)
        self.payload_landing_profile = None
        self.payload_landing_start_time = None
        held = self._payload_planning_hold(now)
        if (self.payload_landing_retry_count >= 3
                or now - self.payload_landing_retry_since >= .3 or not held):
            self._exit_payload_planning_fault(now, self.payload_landing_failure_reason)
            return {}
        return held

    def _transport_phase_active(self, now):
        if self.transport_mode != "slung_load":
            return False
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._state_ready(now):
                return False
            target = vehicle._target_for(state, now)
            phase = target.get("flight_phase")
            if phase in ("landing", "landing_settle") and not self.payload_transport_active:
                return False
            if phase not in (
                    "height_correction", "hover", "figure_eight", "landing",
                    "landing_settle"):
                return False
        return True

    def _payload_landing_release_ready(self, now):
        """Match MATLAB LANDING_TAUT: release only after load reaches ground."""
        if not self.payload_transport_active or self.slung_controller is None:
            return False
        phases = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None:
                return False
            phase = vehicle._target_for(state, now).get("flight_phase")
            phases.append(phase)
        if not phases or not all(phase == "landing_settle" for phase in phases):
            return False
        with self.payload_lock:
            state = deepcopy(self.latest_payload_state)
        if not self._payload_state_ready(state, now):
            return False
        try:
            position = np.asarray(state.get("position"), dtype=float).reshape(3)
            velocity = np.asarray(state.get("velocity"), dtype=float).reshape(3)
        except (TypeError, ValueError):
            return False
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return False
        ground_z = 0.5 * float(self.payload_height_m)
        altitude_tolerance = float(self.vehicles[0].trajectory_config.landing_altitude_tolerance_m)
        velocity_tolerance = float(
            self.vehicles[0].trajectory_config.landing_vertical_velocity_tolerance_mps
        )
        return (
            abs(float(position[2]) - ground_z) <= altitude_tolerance and
            abs(float(velocity[2])) <= velocity_tolerance
        )

    def _begin_payload_emergency_landing(self, now, reason):
        """Switch each vehicle to its existing controlled landing path.

        A transient load-mocap loss must never be handled by the fleet-wide
        zero-thrust abort while aircraft are airborne.  The vehicle state
        machines already have a slow emergency landing reference; reuse it.
        """
        if (self.payload_transport_active
                and self.payload_validation_landing_stage == "payload_lowering"
                and self._payload_ready(now)
                and not str(reason).startswith("touchdown_timeout:")):
            # Keep the payload-aware landing active.  Switching all aircraft
            # to independent ground references makes them descend beside an
            # airborne/attached load and destroys the rope geometry.
            self.payload_payload_aware_emergency = True
            self.payload_controlled_exit_reason = "payload_load: " + str(reason)
            rospy.logwarn(
                "距离/高度保护转入绳向保持下降：%s；等待负载触地确认后再释放绳向。",
                reason,
            )
            return
        if (str(reason).startswith("height_drop:") and self.payload_transport_active
                and self._payload_ready(now) and not self.payload_controlled_exit_reason):
            # A valid, separated loaded fleet can still lower the load. Do not
            # remove its support by switching straight to aircraft-only gravity.
            self.payload_controlled_exit_reason = "payload_load: " + str(reason)
            self.payload_validation_landing_stage = "payload_lowering"
            self.payload_landing_profile = None
            self.payload_landing_start_time = None
            self.payload_unload_start_time = None
            self._begin_validation_vehicle_descent(now)
            return
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if (state is None or not state.get("valid", False)
                    or vehicle.trajectory is None):
                continue
            command = getattr(vehicle, "last_control_command", None)
            if command is not None and vehicle.trajectory.phase not in ("emergency_landing", "landed", "aborted"):
                control = vehicle._build_ekf_control_state(state, now)
                velocity_z = float(control.get("control_velocity", np.zeros(3))[2])
                baseline = vehicle.controller.config.mass * (
                    vehicle.controller.config.gravity - vehicle.controller.config.independent_velocity_gain[2] * velocity_z)
                residual = float(command["desired_force"][2]) - baseline
                self.payload_emergency_support[id(vehicle)] = (float(now), residual)
            vehicle.trajectory.begin_emergency_landing(
                now,
                state["position"],
                rotation_to_rpy(state["rotation"])[2],
                "payload_load: " + str(reason),
            )

    def _check_payload_flight_validation(self, now):
        """End a short flight once, using the existing measured-position descent."""
        if (not self.payload_flight_validation_enabled
                or self.payload_validation_landing_started_time is not None
                or not self.payload_transport_active
                or self.payload_transport_start_time is None
                or self.global_abort_reason):
            return False
        if any(v.trajectory is None or v.trajectory.phase in (
                "emergency_landing", "landing_settle", "landed", "aborted")
               for v in self.vehicles):
            return False
        elapsed = max(0.0, float(now) - self.payload_transport_start_time)
        if elapsed < self.payload_flight_validation_transport_s:
            return False
        self.payload_validation_handoff_time = self.payload_transport_start_time
        self.payload_validation_landing_started_time = float(now)
        rospy.loginfo(
            "[实飞验证] flight_validation_complete：接管已运行 %.3f s（上限 %.3f s）；"
            "三机从当前实测位置开始受控降落。",
            elapsed, self.payload_flight_validation_transport_s,
        )
        self.payload_validation_landing_stage = "payload_lowering"
        self.payload_controlled_exit_reason = "payload_load: flight_validation_complete: %.3f s" % elapsed
        self.payload_validation_ground_since = None
        self.payload_landing_start_time = None
        self.payload_landing_start_position = None
        self._begin_validation_vehicle_descent(now)
        return True

    def _begin_validation_vehicle_descent(self, now):
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if (state is not None and state.get("valid", False)
                    and vehicle.trajectory is not None):
                vehicle.trajectory.begin_controlled_landing(
                    now, state["position"], rotation_to_rpy(state["rotation"])[2],
                    self.payload_controlled_exit_reason,
                )

    def _release_validation_payload(self, now):
        """Release taut support after fresh measured payload touchdown, then land aircraft."""
        if (self.payload_validation_landing_stage != "payload_lowering"
                or not self.payload_transport_active):
            return False
        with self.payload_lock:
            payload = deepcopy(self.latest_payload_state)
        if not self._payload_state_ready(payload, now):
            return False
        clearance = payload_ground_clearance(
            payload["position"], payload["rotation"], self.slung_controller.config.payload_size_m)
        if self.payload_touchdown_detector is None:
            return False
        detected = self.payload_touchdown_detector.update(
            payload.get("received_time", now), clearance, float(payload["velocity"][2]))
        if not detected:
            if (self.payload_landing_profile is not None and
                    self.payload_touchdown_detector.timed_out(
                        now, self.payload_landing_profile.start_time + self.payload_landing_profile.duration)):
                self._begin_payload_emergency_landing(now, "touchdown_timeout: 未确认负载稳定接地")
                rospy.logwarn_throttle(1., "负载接地未确认（间隙 %.3f m），转入保留支撑残差的独立下降；不标记为接地。",
                                       clearance)
                self._reset_payload_handoff()
            return False
        if self.payload_unload_start_time is None:
            self.payload_unload_start_time = float(now)
            self.payload_landing_trim_at_unload = self.payload_support_trim.bias.copy()
            rospy.loginfo("负载稳定接地已确认，开始 %.2f s 有界卸载。", self.payload_landing_unload_s)
            return False
        if now - self.payload_unload_start_time < self.payload_landing_unload_s:
            return False
        self._reset_payload_handoff()
        self.payload_validation_landing_stage = "vehicle_descent"
        for vehicle in self.vehicles:
            # A negative prelift integral supported the airborne load. Do not
            # revive that extra lift under the larger landing limit. Preserve
            # only an already learned, ordinary bounded downward correction.
            ordinary_limit = float(vehicle.controller.config.independent_integral_limit[2])
            vehicle.controller.position_integral[2] = np.clip(
                vehicle.controller.position_integral[2], 0.0, ordinary_limit)
        self._begin_validation_vehicle_descent(now)
        rospy.loginfo("[实飞验证] 负载已落地稳定，释放协同拉力；三机保持各自当前位置 XY 继续降落。")
        return True

    def _check_payload_physical_safety(self, now):
        """Check measured geometry before publishing another cooperative tick."""
        if self.transport_mode != "slung_load":
            return False
        if self.payload_payload_aware_emergency:
            return False
        with self.payload_lock:
            payload = deepcopy(self.latest_payload_state)
        payload_ready = self._payload_state_ready(payload, now)
        if payload_ready:
            self.payload_ground_clearance_m = payload_ground_clearance(
                payload["position"], payload["rotation"], self.slung_controller.config.payload_size_m,
            )
            if self.payload_transport_active:
                self.payload_prelift_ready = bool(
                    self.payload_ground_clearance_m >= self.payload_prelift_clearance_m
                    and abs(float(payload.get("velocity", np.zeros(3))[2]))
                    <= self.payload_prelift_vertical_velocity_tolerance_mps
                )
        else:
            self.payload_ground_clearance_m = math.nan
            self.payload_prelift_ready = False
        # Check after the TAKEUP motion has begun, including independent
        # prelift, so a close formation never reaches the handoff gate.
        if not (self.payload_transport_active or self.payload_takeup_start_time is not None
                or self.payload_validation_landing_stage == "vehicle_descent"
                or self.payload_emergency_support):
            return False
        positions, velocities, phases = [], [], []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._state_ready(now):
                return False  # Existing state-loss handling owns this path.
            positions.append(state["position"])
            control = vehicle._build_ekf_control_state(state, now)
            velocities.append(control.get("control_velocity", np.zeros(3)))
            phases.append(vehicle.trajectory.phase if vehicle.trajectory is not None else "")
        emergency_active = "emergency_landing" in phases
        if "landed" in phases and not (
                self.payload_validation_landing_stage == "vehicle_descent"
                and any(phase in ("landing", "landing_settle") for phase in phases)):
            return False
        height_error = None
        rate_saturated = False
        if self.payload_transport_active and payload_ready and not any(
                phase in ("landing", "landing_settle") for phase in phases):
            # During the ramp, the frozen payload height is the actual target.
            # During lift, read the previous complete reference (one tick old).
            previous = next(iter(self.last_transport_overrides.values()), {})
            target_position = previous.get("payload_target_position", self.payload_transport_start_position)
            if target_position is not None:
                height_error = float(target_position[2] - payload["position"][2])
            rate_saturated = any(
                getattr(vehicle, "last_control_command", None) is not None and np.any(
                    np.abs(vehicle.last_control_command["body_rate_command"][:2]) >=
                    0.98 * vehicle.controller.config.max_body_rate[:2])
                for vehicle in self.vehicles
            )
        result = self.payload_transport_guard.check(
            now, positions, velocities, height_error, rate_saturated,
        )
        self.payload_minimum_vehicle_distance_m = result["minimum_distance_m"]
        self.payload_predicted_vehicle_distance_m = result["predicted_distance_m"]
        if emergency_active:
            # Keep measured proximity visible during the latched descent;
            # repeatedly restarting its reference would prevent landing.
            if result["reason"]:
                rospy.logwarn_throttle(1., "受控故障下降中机间距 %.3f m，预测 %.3f m。",
                                       self.payload_minimum_vehicle_distance_m,
                                       self.payload_predicted_vehicle_distance_m)
            return False
        if not result["reason"]:
            return False
        self.payload_safety_reason = result["reason"]
        self._begin_payload_emergency_landing(
            now, "%s: distance=%.3f m predicted=%.3f m height_error=%s" % (
                result["reason"], result["minimum_distance_m"],
                result["predicted_distance_m"], height_error),
        )
        if (self.payload_controlled_exit_reason
                and all(v.trajectory is not None and v.trajectory.phase == "landing"
                        for v in self.vehicles)):
            # Height fault transitioned to supported descent. Keep its caches
            # and rebuild the descent override on this same control tick.
            return True
        clearance = self.payload_ground_clearance_m
        self._reset_payload_handoff()
        self.payload_ground_clearance_m = clearance
        return True

    def _reset_payload_handoff(self):
        if getattr(self, "payload_attitude_trim", None) is not None:
            self.payload_attitude_trim.reset()
        self._reset_dynamic_attitude_trim_feedback()
        self.payload_attitude_trim_landing_bridge = None
        self.payload_transport_active = False
        self.payload_ready_since = None
        self.payload_transport_start_time = None
        self.payload_transport_start_position = None
        self.payload_transport_start_rotation = None
        self.payload_transport_goal_rotation = None
        self.payload_circle_reference = None
        self.payload_circle_state = None
        self.payload_circle_landing_yaw = None
        self.payload_circle_previous_correction = None
        self.payload_transport_initial_link_units = None
        self.payload_transport_initial_link_distances = None
        self.payload_transport_goal_link_units = None
        self.payload_transport_vehicle_start_positions = None
        self.payload_takeup_start_time = None
        self.payload_takeup_start_positions = None
        self.payload_takeup_target_positions = None
        self.payload_takeup_ready_since = None
        self.payload_prelift_ready_since = None
        self.payload_ground_clearance_m = math.nan
        self.payload_prelift_ready = False
        self.payload_prelift_reference = None
        self.payload_link_calibration_hold_reference = None
        self.payload_last_takeup_reference = None
        self.payload_geometry_ready_since = None
        self.payload_mission_hold_time = None
        self.payload_handoff_reference_positions = None
        self.payload_handoff_forces = None
        self.payload_support_trim.reset()
        self.payload_transport_integral.reset()
        self.payload_transport_integral_last_time = None
        for vehicle in self.vehicles:
            vehicle.controller.reset_transport_position_pid()
        self.payload_transport_guard.reset()
        self.payload_takeup_taut_latched = False
        self.payload_takeup_diagnostic_last_time = None
        self.payload_landing_start_time = None
        self.payload_landing_start_position = None
        self.payload_landing_profile = None
        self.payload_last_reference = None
        self.payload_landing_gain_snapshot = None
        self.payload_landing_rope_directions = None
        self.payload_landing_rope_distances = None
        self.payload_payload_aware_emergency = False
        self.payload_unload_start_time = None
        self.payload_landing_trim_at_unload = None
        self.payload_fault_since = None
        self.last_transport_overrides = {}
        if self.slung_controller is not None:
            self.slung_controller.reset()

    def _payload_emergency_override(self, vehicle, now):
        if (vehicle.trajectory is None or vehicle.trajectory.phase != "emergency_landing"
                or id(vehicle) not in self.payload_emergency_support):
            return {}
        started, initial_trim = self.payload_emergency_support[id(vehicle)]
        duration = max(2., float(vehicle.trajectory.active_landing_duration_s))
        fade = smoothstep5_profile(now - started, duration)
        return dict(independent_mode=True, transport_mode=False,
            independent_position_gain_scale=self.payload_prelift_parameters["position_gain_scale"],
            independent_velocity_gain_scale=self.payload_prelift_parameters["velocity_gain_scale"],
            max_tilt_rad=self.payload_prelift_parameters["max_tilt_rad"],
            landing_support_trim_n=(1 - fade[0]) * initial_trim,
            landing_support_trim_dot_nps=-fade[1] * initial_trim)
