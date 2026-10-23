"""Shared trim, reference bridging and transport feedback adaptation."""

from slung_circle_reference import rotate_vehicle_references
from slung_landing import SlungLandingProfile
from slung_load_controller import smoothstep5_profile
from slung_reference import _rotation_vector, _rotation_matrix
import math
import numpy as np
import rospy


class PayloadControlMixin:
    """Shared trim, reference bridging and transport feedback adaptation."""

    def _reset_dynamic_attitude_trim_feedback(self):
        self._attitude_trim_feedback_sample = None
        self._attitude_trim_feedback_cache = None
        self._attitude_trim_feedback_recover_at = -math.inf
        self.payload_attitude_trim_feedback_healthy = False
        self.payload_attitude_trim_feedback_quality_reason = "inactive"
        self.payload_attitude_trim_body_rate_norm_rps = math.nan
        self.payload_attitude_trim_pose_step_deg = math.nan
        self.payload_attitude_trim_pose_step_limit_deg = math.nan

    def _dynamic_attitude_trim_feedback_reason(self, now, state):
        """Gate adaptation on one source-pose history, without changing core feedback.

        This history uses source sample time, not callback/control dt. Duplicate
        source frames do not become zero-dt outliers. Rejected pose jumps never
        replace the last accepted pose; a good sample must then survive a short
        recovery interval before learning resumes. Existing C2 trim motion is
        allowed to finish throughout a freeze.
        """
        stamp = float(state.get("sample_time", math.nan))
        key = (float(now), stamp)
        cached = getattr(self, "_attitude_trim_feedback_cache", None)
        if cached is not None and cached[0] == key:
            return cached[1]
        reason = ""
        step, step_limit, rate_norm = math.nan, math.nan, math.nan
        try:
            rotation = _rotation_matrix(np.asarray(state["rotation"], dtype=float), "payload rotation")
            rate = np.asarray(state["body_rate"], dtype=float)
            if rate.shape != (3,) or not np.all(np.isfinite(rate)) or not math.isfinite(stamp) or stamp <= 0.:
                raise ValueError("invalid trim feedback")
            rate_norm = float(np.linalg.norm(rate))
            previous = self._attitude_trim_feedback_sample
            accepted_pose = True
            if previous is not None:
                dt = stamp-previous[0]
                if dt < -1.e-9:
                    reason, accepted_pose = "payload_sample_order", False
                else:
                    step = math.degrees(float(np.linalg.norm(_rotation_vector(previous[1].T@rotation))))
                    step_limit = (self.payload_attitude_trim_feedback_pose_jump_deg
                                  + math.degrees(self.payload_attitude_trim_feedback_max_rate_rps*max(dt, 0.)))
                    if step > step_limit+1.e-9:
                        reason, accepted_pose = "payload_pose_jump", False
            if accepted_pose and (previous is None or stamp > previous[0]+1.e-9):
                self._attitude_trim_feedback_sample = (stamp, rotation.copy())
            if not reason and rate_norm > self.payload_attitude_trim_feedback_max_rate_rps:
                reason = "payload_rate_outlier"
        except (KeyError, TypeError, ValueError, ArithmeticError, np.linalg.LinAlgError):
            reason = "payload_feedback_invalid"
        if reason:
            self._attitude_trim_feedback_recover_at = float(now)+self.payload_attitude_trim_feedback_recovery_s
        elif float(now) < self._attitude_trim_feedback_recover_at-1.e-12:
            reason = "payload_feedback_recovery"
        self.payload_attitude_trim_feedback_healthy = not bool(reason)
        self.payload_attitude_trim_feedback_quality_reason = reason
        self.payload_attitude_trim_body_rate_norm_rps = rate_norm
        self.payload_attitude_trim_pose_step_deg = step
        self.payload_attitude_trim_pose_step_limit_deg = step_limit
        self._attitude_trim_feedback_cache = (key, reason)
        return reason

    def _transport_learning_reason(self, now, payload_state, control_states,
                                   overrides, landing, enabled, supported,
                                   allow_dynamic_reference=False, allow_reference_lift=False):
        """Use one fleet health gate for the slow reference and force learners."""
        elapsed = float(now) - float(self.payload_transport_start_time)
        reference_end = max(
            self.payload_attitude_reference_s if self.payload_attitude_reference_enabled else 0.,
            self.payload_tension_ramp_s + self.payload_reference_lift_s)
        reason = ""
        if not enabled:
            reason = "disabled"
        elif not supported:
            reason = "unsupported_force_mode"
        elif self.global_abort_reason:
            reason = "global_abort"
        elif landing or self.payload_validation_landing_stage or self.payload_controlled_exit_reason:
            reason = "landing"
        elif not self.payload_transport_active:
            reason = "inactive"
        elif allow_reference_lift and (
                elapsed < self.payload_tension_ramp_s
                or any(float(item.get("transport_blend", 0.)) < 1.-1.e-9
                       for item in overrides.values())):
            reason = "tension_ramp"
        elif not allow_reference_lift and elapsed < reference_end:
            reason = "reference_moving"
        elif (not allow_dynamic_reference
              and getattr(self, "payload_circle_state", None) is not None
              and self.payload_circle_state["phase"] in ("circle", "figure_eight", "cone_tilt_in",
                                                        "attitude_cone", "cone_tilt_out")):
            reason = "reference_moving"
        elif (self.payload_flight_validation_enabled
              and elapsed >= self.payload_flight_validation_transport_s):
            reason = "validation_due"
        if not reason:
            try:
                source_age = float(rospy.Time.now().to_sec()) - float(payload_state["sample_time"])
            except (AttributeError, KeyError, TypeError, ValueError):
                source_age = math.nan
            future_tolerance = float(getattr(self.vehicles[0], "mocap_timestamp_future_tolerance_s", 0.))
            if (not self._payload_state_ready(payload_state, now) or not math.isfinite(source_age)
                    or not -future_tolerance <= source_age <= self.vehicles[0].state_timeout):
                reason = "payload_feedback"
            elif any(not state.get("ekf_state_valid", False)
                     or state.get("ekf_kinematics_held", False)
                     or not math.isfinite(float(state.get("ekf_kinematics_weight_effective", 0.)))
                     or state.get("ekf_kinematics_weight_effective", 0.) <= 0.
                     or not state.get("derivatives_valid", False)
                     or not state.get("filter_derivatives_valid", state.get("derivatives_valid", False))
                     for state in control_states):
                reason = "vehicle_feedback"
            elif any(not math.isfinite(float(overrides[id(vehicle)].get("desired_tension_signed", 0.)))
                     or overrides[id(vehicle)].get("desired_tension_signed", 0.) <= 0.
                     for vehicle in self.vehicles):
                reason = "nonpositive_tension"
            elif any(any(bool((getattr(vehicle, "last_control_command", None) or {}).get(key, False))
                         for key in ("force_limit_active", "hold_feedback_limit_active",
                                     "body_rate_limit_active", "collective_limit_active"))
                     for vehicle in self.vehicles):
                reason = "host_command_limited"
        return reason

    def _attitude_trim_metadata(self, overrides, offsets, reason):
        trim = getattr(self, "payload_attitude_trim", None)
        enabled = bool(getattr(self, "payload_attitude_trim_enabled", False)
                       and trim is not None
                       and self.slung_controller.config.transport_force_mode == "reference_cascade")
        for index, vehicle in enumerate(self.vehicles):
            overrides[id(vehicle)].update(
                transport_attitude_trim_enabled=enabled,
                transport_attitude_trim_dynamic_enabled=enabled and bool(
                    getattr(self, "payload_attitude_trim_dynamic_enabled", False)),
                transport_attitude_trim_during_lift_enabled=enabled and bool(
                    getattr(self, "payload_attitude_trim_dynamic_enabled", False)
                    and getattr(self, "payload_attitude_trim_during_lift_enabled", False)),
                transport_attitude_trim_feedback_healthy=enabled and bool(
                    getattr(self, "payload_attitude_trim_feedback_healthy", False)),
                transport_attitude_trim_feedback_quality_reason=str(
                    getattr(self, "payload_attitude_trim_feedback_quality_reason", "inactive")),
                transport_attitude_trim_body_rate_norm_rps=float(
                    getattr(self, "payload_attitude_trim_body_rate_norm_rps", math.nan)),
                transport_attitude_trim_pose_step_deg=float(
                    getattr(self, "payload_attitude_trim_pose_step_deg", math.nan)),
                transport_attitude_trim_pose_step_limit_deg=float(
                    getattr(self, "payload_attitude_trim_pose_step_limit_deg", math.nan)),
                transport_attitude_trim_learning=enabled and not reason and bool(trim.learning),
                transport_attitude_trim_freeze_reason=reason,
                transport_attitude_trim_error=(trim.error if enabled else np.zeros(3)),
                transport_attitude_trim_integral=(trim.integral if enabled else np.zeros(3)),
                transport_attitude_trim_position=offsets[0][index].copy(),
                transport_attitude_trim_velocity=offsets[1][index].copy(),
                transport_attitude_trim_acceleration=offsets[2][index].copy(),
                transport_attitude_trim_limit_active=enabled and bool(trim.saturation),
                transport_attitude_trim_rate_limit_active=enabled and bool(trim.rate_limited),
                transport_attitude_trim_acceleration_limit_active=enabled and bool(trim.acceleration_limited))

    @staticmethod
    def _evaluate_attitude_trim_bridge(bridge, now):
        elapsed = max(0., float(now) - bridge["start_time"])
        shape = bridge["segments"][0][1].shape[1:]
        for duration, coefficients in bridge["segments"]:
            if elapsed < duration:
                u = elapsed / duration
                p = np.polynomial.polynomial.polyval(u, coefficients)
                v = np.polynomial.polynomial.polyval(u,
                    coefficients[1:] * np.arange(1, len(coefficients))[:, None, None]) / duration
                a = np.polynomial.polynomial.polyval(u,
                    coefficients[2:] * (np.arange(2, len(coefficients))
                    * np.arange(1, len(coefficients)-1))[:, None, None]) / duration**2
                return p, v, a
            elapsed -= duration
        return tuple(np.zeros(shape) for _ in range(3))

    def _make_attitude_trim_landing_bridge(self, now, overrides):
        """Bridge the old command into measured rope geometry exactly once.

        The measured rope snapshot already includes achieved trim. A residual
        from the previous complete command preserves its p/v/a without adding
        that trim again. A normalized quintic removes the residual; difficult
        initial derivatives first use a C2 acceleration ramp and velocity brake.
        """
        previous = self.last_transport_overrides
        if not all(self._finite_transport_override(previous.get(id(v), {})) for v in self.vehicles):
            return None
        old = {key: dict(value) for key, value in previous.items()}
        self._refresh_cached_attitude_trim(now, old, "landing")
        names = (("position", "velocity", "acceleration"),
                 ("position_hold_position", "position_hold_velocity", "position_hold_acceleration"))
        p, v, a = [np.asarray([old[id(vehicle)][fields[order]]
                    - overrides[id(vehicle)][fields[order]]
                    for fields in names for vehicle in self.vehicles]) for order in range(3)]
        distance = float(np.max(np.linalg.norm(p, axis=1)))
        speed_limit = max(self.payload_landing_max_speed_mps,
                          float(np.max(np.linalg.norm(v, axis=1))))
        acceleration_limit = max(self.payload_landing_max_acceleration_mps2,
                                 float(np.max(np.linalg.norm(a, axis=1))))
        minimum = max(float(self.payload_landing_gain_transition_s), 1.e-6)
        duration = max(minimum, 1.875 * distance / speed_limit,
                       math.sqrt((10. / math.sqrt(3.)) * distance / acceleration_limit))
        segments = []
        for _ in range(3):
            coefficients = SlungLandingProfile._quintic_coefficients(p, v, a, np.zeros_like(p), duration)
            velocity = coefficients[1:] * np.arange(1, 6)[:, None, None] / duration
            acceleration = velocity[1:] * np.arange(1, 5)[:, None, None] / duration
            if all(SlungLandingProfile._peak_norm(velocity[:, i]) <= speed_limit + 1.e-9
                   and SlungLandingProfile._peak_norm(acceleration[:, i]) <= acceleration_limit + 1.e-9
                   for i in range(len(p))):
                segments = [(duration, coefficients)]
                break
            duration *= 1.15
        if not segments:
            # Keep every aircraft's inherited speed ceiling while ramping its
            # acceleration to zero. An outward acceleration at that ceiling
            # has no C2 feasible ramp and must use the supported retry path.
            ramp = .1
            if np.any(a):
                for initial_v, initial_a in zip(v, a):
                    anorm2 = float(initial_a @ initial_a)
                    if anorm2 == 0.:
                        continue
                    dot = float(initial_v @ initial_a)
                    margin = max(0., speed_limit**2 - float(initial_v @ initial_v))
                    discriminant = math.sqrt(dot**2 + anorm2*margin)
                    maximum = (2.*margin/(discriminant+dot) if dot >= 0. and discriminant+dot > 0.
                               else 2.*(discriminant-dot)/anorm2)
                    if maximum <= 0.:
                        raise ValueError("attitude trim landing bridge has outward acceleration at its speed ceiling")
                    ramp = min(ramp, .5*maximum)
                coefficients = np.asarray([p, v*ramp, .5*a*ramp**2,
                                           -a*ramp**2/6.])
                segments.append((ramp, coefficients))
                p = p + v*ramp + a*ramp**2/3.
                v = v + .5*a*ramp
            else:
                ramp = 0.
            brake = max(1.e-9, 1.5 * float(np.max(np.linalg.norm(v, axis=1))) / acceleration_limit)
            segments.append((brake, np.asarray([p, v*brake, np.zeros_like(p),
                                                -v*brake, .5*v*brake])))
            p = p + .5*brake*v
            distance = float(np.max(np.linalg.norm(p, axis=1)))
            duration = max(1.e-9, minimum-ramp-brake, 1.875*distance/speed_limit,
                           math.sqrt((10./math.sqrt(3.))*distance/acceleration_limit))
            segments.append((duration, SlungLandingProfile._quintic_coefficients(
                p, np.zeros_like(p), np.zeros_like(p), np.zeros_like(p), duration)))
        return {"start_time": float(now), "duration": sum(segment[0] for segment in segments),
                "segments": segments}

    def _apply_attitude_reference_trim(self, now, payload_state, control_states,
                                       overrides, landing):
        trim = getattr(self, "payload_attitude_trim", None)
        enabled = bool(getattr(self, "payload_attitude_trim_enabled", False) and trim is not None)
        supported = self.slung_controller.config.transport_force_mode == "reference_cascade"
        dynamic = bool(getattr(self, "payload_attitude_trim_dynamic_enabled", False))
        during_lift = dynamic and bool(getattr(self, "payload_attitude_trim_during_lift_enabled", False))
        reason = self._transport_learning_reason(now, payload_state, control_states,
                                                overrides, landing, enabled, supported,
                                                allow_dynamic_reference=dynamic,
                                                allow_reference_lift=during_lift)
        offsets = tuple(np.zeros((len(self.vehicles), 3)) for _ in range(3))
        if enabled and supported:
            if dynamic:
                quality_reason = self._dynamic_attitude_trim_feedback_reason(now, payload_state)
                if not reason:
                    reason = quality_reason
            goal = (next(iter(overrides.values()))["payload_target_rotation"] if dynamic
                    else self.payload_transport_goal_rotation)
            if goal is None:
                goal = next(iter(overrides.values()))["payload_target_rotation"]
            if landing:
                trim.freeze()
            circle = getattr(self, "payload_circle_state", None) if not landing else None
            measured = payload_state["rotation"]
            if not dynamic and circle is not None and self.payload_circle_config.mode != "attitude_cone":
                relative_rotation = circle["rotation"] @ self.payload_circle_reference.initial_rotation.T
                measured = relative_rotation.T @ measured
            offsets = trim.update(measured, goal, now, learning=not reason)
            if not dynamic and circle is not None and self.payload_circle_config.mode != "attitude_cone":
                offsets = rotate_vehicle_references(*offsets, np.zeros(3), np.zeros(3),
                    np.zeros(3), circle["yaw"]-self.payload_circle_reference.initial_yaw,
                    circle["yaw_rate"], circle["yaw_acceleration"])
        self._attitude_trim_metadata(overrides, offsets, reason)
        landing_bridge = bool(landing and supported and (
            (enabled and trim.initialized) or self.payload_circle_landing_yaw is not None))
        if landing_bridge:
            if self.payload_attitude_trim_landing_bridge is None:
                self.payload_attitude_trim_landing_bridge = self._make_attitude_trim_landing_bridge(now, overrides)
            bridge = self.payload_attitude_trim_landing_bridge
            applied = (self._evaluate_attitude_trim_bridge(bridge, now) if bridge is not None else
                       tuple(np.zeros((2*len(self.vehicles), 3)) for _ in range(3)))
        else:
            applied = tuple(np.concatenate((value, value), axis=0) for value in offsets)
        count = len(self.vehicles)
        for index, vehicle in enumerate(self.vehicles):
            item = overrides[id(vehicle)]
            for hold, fields in enumerate((("position", "velocity", "acceleration"),
                    ("position_hold_position", "position_hold_velocity", "position_hold_acceleration"))):
                row = index + hold*count
                for derivative, field in enumerate(fields):
                    item[field] = item[field] + applied[derivative][row]
            item["_attitude_trim_applied"] = np.asarray([value[index] for value in applied])
            item["_attitude_trim_hold_applied"] = np.asarray([value[index+count] for value in applied])
            item["_attitude_trim_landing"] = bool(landing)
            item["transport_attitude_trim_bridge_active"] = bool(landing_bridge
                and self.payload_attitude_trim_landing_bridge is not None
                and now < self.payload_attitude_trim_landing_bridge["start_time"] + self.payload_attitude_trim_landing_bridge["duration"])
            for hold, prefix in enumerate(("transport_attitude_trim_bridge_", "transport_attitude_trim_bridge_hold_")):
                for derivative, name in enumerate(("position", "velocity", "acceleration")):
                    item[prefix+name] = applied[derivative][index+hold*count].copy() if landing else np.zeros(3)

    def _refresh_cached_attitude_trim(self, now, overrides, reason, freeze_base_derivatives=False):
        """Finish bounded motion across a feedback gap without learning stale pose."""
        trim = getattr(self, "payload_attitude_trim", None)
        enabled = bool(getattr(self, "payload_attitude_trim_enabled", False)
                       and trim is not None
                       and self.slung_controller.config.transport_force_mode == "reference_cascade")
        bridge = getattr(self, "payload_attitude_trim_landing_bridge", None)
        if not enabled and bridge is None:
            return
        offsets = (trim.update(None, None, now, learning=False) if enabled else
                   tuple(np.zeros((len(self.vehicles), 3)) for _ in range(3)))
        circle = getattr(self, "payload_circle_state", None)
        if (not getattr(self, "payload_attitude_trim_dynamic_enabled", False)
                and circle is not None and self.payload_circle_config.mode != "attitude_cone"):
            offsets = rotate_vehicle_references(*offsets, np.zeros(3), np.zeros(3), np.zeros(3),
                circle["yaw"]-self.payload_circle_reference.initial_yaw,
                circle["yaw_rate"], circle["yaw_acceleration"])
        if bridge is not None and any(t.get("_attitude_trim_landing", False) for t in overrides.values()):
            applied = self._evaluate_attitude_trim_bridge(bridge, now)
        else:
            applied = tuple(np.concatenate((value, value), axis=0) for value in offsets)
        count = len(self.vehicles)
        for index, vehicle in enumerate(self.vehicles):
            item = overrides[id(vehicle)]
            for hold, (fields, key) in enumerate(((("position", "velocity", "acceleration"), "_attitude_trim_applied"),
                    (("position_hold_position", "position_hold_velocity", "position_hold_acceleration"), "_attitude_trim_hold_applied"))):
                old = np.asarray(item.get(key, np.zeros((3, 3))))
                row = index+hold*count
                for derivative, field in enumerate(fields):
                    base = (np.zeros(3) if freeze_base_derivatives and derivative else item[field]-old[derivative])
                    item[field] = base + applied[derivative][row]
                item[key] = np.asarray([value[row] for value in applied])
                prefix = "transport_attitude_trim_bridge_hold_" if hold else "transport_attitude_trim_bridge_"
                for derivative, name in enumerate(("position", "velocity", "acceleration")):
                    item[prefix+name] = applied[derivative][row].copy() if item.get("_attitude_trim_landing", False) else np.zeros(3)
            item["transport_attitude_trim_bridge_active"] = bool(bridge is not None and item.get("_attitude_trim_landing", False)
                and now < bridge["start_time"]+bridge["duration"])
        self._attitude_trim_metadata(overrides, offsets, reason)
        if getattr(self, "payload_attitude_trim_dynamic_enabled", False):
            for item in overrides.values():
                item["transport_attitude_trim_feedback_healthy"] = False
                item["transport_attitude_trim_feedback_quality_reason"] = str(reason)

    def _apply_transport_hold_integral(self, now, payload_state, vehicle_states,
                                       control_states, overrides, landing):
        """Learn differential world forces only from steady, healthy transport.

        Keep learned forces through temporary feedback gaps and load lowering.
        Touchdown releases them with the same profile as the support trim.
        All limits freeze the fleet together, preserving its zero force sum.
        """
        native_pid = self.payload_transport_feedback_controller == "native_position_pid"
        enabled = native_pid or self.payload_transport_integral_enabled
        supported = self.slung_controller.config.transport_force_mode == "reference_cascade"
        reason = self._transport_learning_reason(now, payload_state, control_states,
                                                overrides, landing, enabled, supported,
                                                allow_dynamic_reference=native_pid)
        learning = not reason
        duplicate = self.payload_transport_integral_last_time == float(now)
        errors = np.asarray([
            state["position"] - overrides[id(vehicle)]["position_hold_position"]
            for vehicle, state in zip(self.vehicles, vehicle_states)])
        dt = float(np.clip(getattr(self, "payload_control_dt", 1. / self.rate_hz), .001, .05))
        if native_pid:
            bias = np.zeros((3, 3))
            rate = np.zeros((3, 3))
        else:
            bias, rate = self.payload_transport_integral.update(
                errors, dt, learning=learning and not duplicate)
            self.payload_transport_integral_last_time = float(now)
        # Disabled/legacy branches cannot apply a previous mode's state.
        force = bias.copy() if enabled and supported and not native_pid else np.zeros_like(bias)
        force_dot = rate.copy() if enabled and supported and not native_pid else np.zeros_like(rate)
        integral_weight, integral_weight_dot = 1., 0.
        if landing and self.payload_unload_start_time is not None:
            unload = smoothstep5_profile(now - self.payload_unload_start_time, self.payload_landing_unload_s)
            integral_weight, integral_weight_dot = 1. - unload[0], -unload[1]
            force = integral_weight * force
            force_dot = integral_weight_dot * bias if enabled and supported else np.zeros_like(rate)
        for index, vehicle in enumerate(self.vehicles):
            overrides[id(vehicle)].update(
                transport_hold_integral_enabled=enabled and not native_pid,
                transport_hold_integral_learning=learning and not duplicate and not native_pid,
                transport_hold_integral_freeze_reason=("replaced_by_native_position_pid" if native_pid
                    else reason or ("duplicate_tick" if duplicate else "learning")),
                transport_hold_integral_bias=bias[index].copy(),
                transport_hold_integral_error=self.payload_transport_integral.differential_error_m[index].copy(),
                transport_hold_integral_force=force[index].copy(),
                transport_hold_integral_force_dot=force_dot[index].copy(),
                transport_feedback_controller=self.payload_transport_feedback_controller,
                transport_full_pid_learning=native_pid and learning,
                transport_full_pid_freeze_reason=reason if native_pid else "inactive_mode",
                transport_full_pid_tick=float(now),
                transport_full_pid_weight=integral_weight,
                transport_full_pid_weight_dot=integral_weight_dot)
