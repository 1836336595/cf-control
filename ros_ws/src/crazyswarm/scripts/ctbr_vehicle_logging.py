"""Per-aircraft command, feedback and timing diagnostics."""

from ctbr_control_math import rotation_to_rpy
import math
import numpy as np
import rospy
import time


class VehicleLoggingMixin:
    """Per-aircraft command, feedback and timing diagnostics."""

    def _timing_log_fields(self, now):
        fields = {name: float(getattr(self, name, math.nan)) for name in (
            "controller_tick_interval_s", "controller_compute_wall_s",
            "previous_vehicle_cycle_wall_s", "previous_vehicle_cycle_control_time_s",
            "fleet_tick_interval_s", "previous_fleet_cycle_wall_s",
            "previous_fleet_cycle_control_time_s",
        )}
        with self.lock:
            published = int(getattr(self, "last_published_command_stamp_ns", 0))
            cached = getattr(self, "latest_bridge_timing", None)
            timing = None if cached is None else dict(cached)
        fields["published_command_stamp_ns"] = published
        for name in ("command_stamp_ns", "command_seq", "command_age_s", "callback_wall_s",
                     "send_call_wall_s", "raw_thrust", "accepted", "unlock_packet"):
            fields["bridge_latest_" + name] = math.nan if timing is None else timing[name]
        fields["bridge_latest_receipt_age_s"] = (math.nan if timing is None else
            max(0.0, time.monotonic() - timing["received_time"]))
        # A stamp match is a correlation hint. ROS serialization owns header.seq,
        # and paused/coarse ROS clocks can give different commands the same stamp.
        fields["bridge_latest_matches_published_stamp"] = int(
            timing is not None and published > 0 and published == timing["command_stamp_ns"])
        return fields

    @staticmethod
    def _with_xyz(row, prefix, vector):
        row[prefix + "_x"] = float(vector[0])
        row[prefix + "_y"] = float(vector[1])
        row[prefix + "_z"] = float(vector[2])

    def _payload_observer_log_fields(self, row, now, payload):
        """Keep command inputs separate from newer, possibly rejected frames."""
        def number(sample, key):
            value = (sample or {}).get(key)
            return math.nan if value is None else float(value)

        source = getattr(self, "payload_source_diagnostics", None)
        computed_at = float(getattr(self, "payload_control_time", math.nan))
        row["payload_control_time_s"] = computed_at - self.start_time
        row["payload_control_age_s"] = float(now) - computed_at
        row["payload_control_reused"] = int(bool(
            getattr(self, "payload_control_reused", False)))
        row["payload_control_sample_time_s"] = number(payload, "sample_time")
        row["payload_source_stamp_s"] = number(source, "source_stamp")
        row["payload_source_received_age_s"] = float(now) - number(source, "received_time")
        for prefix, sample in (("payload_observer_", payload),
                               ("payload_source_observer_", source)):
            sample = sample or {}
            row[prefix + "status"] = str(sample.get("observer_status", ""))
            for suffix, key in (("updated", "observer_updated"),
                                ("quality_valid", "observer_quality_valid"),
                                ("velocity_clipped", "velocity_clipped"),
                                ("acceleration_clipped", "acceleration_clipped"),
                                ("body_rate_clipped", "body_rate_clipped")):
                row[prefix + suffix] = int(bool(sample.get(key, False)))
            for key in ("input_dt_s", "update_dt_s", "sample_count", "reset_count",
                        "rejected_count", "consecutive_rejected_count"):
                row[prefix + key] = number(sample, key)
            for key in ("position_innovation", "attitude_innovation"):
                self._with_xyz(row, prefix + key, sample.get(key, [math.nan] * 3))
        for key in ("raw_position",):
            self._with_xyz(row, "payload_source_" + key,
                           (source or {}).get(key, [math.nan] * 3))
        raw_rotation = (source or {}).get("raw_rotation")
        rpy = ([math.nan] * 3 if raw_rotation is None or not np.all(np.isfinite(raw_rotation))
               else rotation_to_rpy(raw_rotation))
        for axis, value in zip(("roll", "pitch", "yaw"), rpy):
            row["payload_source_raw_" + axis + "_rad"] = float(value)

    def _base_log_row(self, now, state):
        state_age = math.inf if state is None else now - state["received_time"]
        ros_time_s = rospy.Time.now().to_sec()
        try:
            mocap_sample_age = (
                math.inf if state is None
                else ros_time_s - float(state["mocap_sample_time_s"])
            )
        except (KeyError, TypeError, ValueError):
            mocap_sample_age = math.nan
        row = {
            "control_time_s": now - self.start_time,
            "ros_time_s": ros_time_s,
            "mode": "control",
            "flight_phase": self.flight_phase,
            "state_valid": int(state is not None and state["valid"]),
            "derivatives_valid": int(state is not None and state["derivatives_valid"]),
            "filter_derivatives_valid": int(
                state is not None and state.get("filter_derivatives_valid", False)
            ),
            "state_age_s": state_age,
            "mocap_sample_age_s": mocap_sample_age,
            "mission_time_s": (
                math.nan if self.mission_start_time is None
                else now - self.mission_start_time
            ),
            "vehicle_id": self.vehicle_id,
            "radio_uri": self.radio_uri,
            "orbit_phase_rad": self.orbit_phase_rad,
            "formation_state": self.flight_phase,
            "global_abort_reason": self.global_abort_reason,
            "invalid_reason": "",
        }
        row.update(self._battery_log_fields(now))
        row.update(self._preflight_voltage_log_fields())
        row.update(self._timing_log_fields(now))
        payload = self.payload_diagnostics
        self._payload_observer_log_fields(row, now, payload)
        payload_age = (math.inf if payload is None
                       else now - float(payload.get("received_time", -math.inf)))
        row["payload_state_age_s"] = payload_age
        row["payload_state_valid"] = int(
            payload is not None and payload.get("valid", False)
            and math.isfinite(payload_age) and payload_age <= self.state_timeout
        )
        row["emergency_landing_reason"] = (
            getattr(self.trajectory, "emergency_landing_reason", "")
            if self.trajectory is not None else ""
        )
        if payload is not None:
            for name, key in (
                    ("payload_position", "position"),
                    ("payload_raw_velocity", "raw_velocity"),
                    ("payload_raw_acceleration", "raw_acceleration"),
                    ("payload_velocity", "velocity"),
                    ("payload_acceleration", "acceleration"),
                    ("payload_body_rate", "body_rate")):
                value = payload.get(key)
                if value is not None:
                    self._with_xyz(row, name, value)
            row["payload_filter_derivatives_valid"] = int(
                bool(payload.get("derivatives_valid", False))
            )
            if "rotation" in payload:
                rpy = rotation_to_rpy(payload["rotation"])
                axis = np.asarray(payload["rotation"],dtype=float)[:,2]
                self._with_xyz(row,"payload_body_z",axis)
                row["payload_cone_half_angle_deg"] = float(np.rad2deg(np.arccos(np.clip(axis[2],-1.,1.))))
                for name, value in zip(("roll", "pitch", "yaw"), rpy):
                    row["payload_" + name + "_rad"] = float(value)
        return row

    def _preflight_voltage_log_fields(self):
        """返回冻结补偿的诊断字段，便于离线区分电压和控制器问题。"""
        return {
            "preflight_voltage_v": self.preflight_voltage_v,
            "preflight_voltage_sample_count": self.preflight_voltage_sample_count,
            "preflight_voltage_ready": int(self.preflight_voltage_ready),
            "preflight_voltage_failed": int(self.preflight_voltage_failed),
            "thrust_raw_scale": self.thrust_raw_scale,
        }

    def _battery_log_fields(self, now):
        """返回一组可直接写入 CSV 的电池字段，不把旧样本伪装成当前测量。"""
        with self.lock:
            battery = None if self.latest_battery is None else dict(self.latest_battery)
        if battery is None:
            return {
                "battery_voltage_v": math.nan,
                "battery_voltage_mv": math.nan,
                "battery_state": -1,
                "battery_level_percent": math.nan,
                "battery_age_s": math.inf,
            }
        return {
            "battery_voltage_v": battery["voltage_v"],
            "battery_voltage_mv": battery["voltage_mv"],
            "battery_state": battery["state"],
            "battery_level_percent": battery["level_percent"],
            "battery_age_s": now - battery["received_time"],
        }

    def _write_invalid_log(self, now, state, invalid_reason=""):
        """记录失效样本；保留可用原始状态，控制量字段留空以便离线识别。"""
        row = self._base_log_row(now, state)
        row["invalid_reason"] = str(invalid_reason or "state_check_failed")
        if state is not None:
            self._with_xyz(row, "position", state["position"])
            self._with_xyz(row, "velocity", state["velocity"])
            self._with_xyz(row, "acceleration", state["acceleration"])
            if "filtered_velocity" in state:
                self._with_xyz(row, "filtered_velocity", state["filtered_velocity"])
            if "filtered_acceleration" in state:
                self._with_xyz(
                    row, "filtered_acceleration", state["filtered_acceleration"]
                )
            self._with_xyz(row, "body_rate", state["body_rate"])
            rpy = rotation_to_rpy(state["rotation"])
            row.update(dict(zip(("roll_rad", "pitch_rad", "yaw_rad"), rpy)))
        self.logger.write(row)

    def _write_log(
            self, now, state, target, command, control_state=None,
            invalid_reason=""):
        """记录一次有效控制周期，所有误差均使用与实际发布相同的参考和命令。"""
        row = self._base_log_row(now, state)
        self.flight_phase = target["flight_phase"]
        row["flight_phase"] = self.flight_phase
        row["formation_state"] = self.flight_phase
        row["invalid_reason"] = str(invalid_reason or "")
        self._with_xyz(row, "position", state["position"])
        self._with_xyz(row, "velocity", state["velocity"])
        self._with_xyz(row, "acceleration", state["acceleration"])
        self._with_xyz(row, "filtered_velocity", state["filtered_velocity"])
        self._with_xyz(row, "filtered_acceleration", state["filtered_acceleration"])
        self._with_xyz(row, "body_rate", state["body_rate"])
        self._with_xyz(row, "target", target["position"])
        self._with_xyz(row, "target_jerk", target.get("jerk", np.zeros(3)))
        self._with_xyz(row, "target_velocity", target.get("velocity", np.zeros(3)))
        self._with_xyz(row, "target_acceleration", target.get("acceleration", np.zeros(3)))
        self._with_xyz(row, "position_error", command["position_error"])
        self._with_xyz(row, "velocity_error", command["velocity_error"])
        self._with_xyz(row, "desired_force", command["desired_force"])
        self._with_xyz(
            row, "desired_force_dot", command.get("desired_force_dot", np.zeros(3))
        )
        row["prelift_vertical_integral_active"] = int(bool(
            command.get("prelift_vertical_integral_active", False)
        ))
        row["prelift_vertical_integral_force_n"] = float(
            command.get("prelift_vertical_integral_force_n", 0.0)
        )
        row["prelift_vertical_integral_learning"] = int(bool(
            command.get("prelift_vertical_integral_active", False)
            and target.get("prelift_vertical_integral_learning", True)))
        row["payload_prelift_feedback_reused"] = int(bool(
            target.get("payload_prelift_feedback_reused", False)))
        row["payload_prelift_feedback_age_s"] = float(
            target.get("payload_prelift_feedback_age_s", math.nan))
        row["landing_vertical_integral_active"] = int(bool(
            command.get("landing_vertical_integral_active", False)))
        row["landing_vertical_integral_force_n"] = float(
            command.get("landing_vertical_integral_force_n", 0.0))
        for field in ("transport_support_trim_n", "payload_minimum_vehicle_distance_m",
                      "payload_predicted_vehicle_distance_m", "transport_max_tilt_rad"):
            row[field] = float(target.get(field, math.nan))
        row["payload_safety_reason"] = target.get("payload_safety_reason", "")
        row["payload_landing_rope_preserving"] = int(bool(
            target.get("payload_landing_rope_preserving", False)
        ))
        row["payload_landing_rope_distance_m"] = float(
            target.get("payload_landing_rope_distance_m", math.nan)
        )
        row["flight_validation_enabled"] = int(bool(target.get("flight_validation_enabled", False)))
        row["flight_validation_elapsed_s"] = float(target.get("flight_validation_elapsed_s", math.nan))
        row["flight_validation_landing_started"] = int(bool(target.get("flight_validation_landing_started", False)))
        row["flight_validation_landing_stage"] = str(target.get("flight_validation_landing_stage", ""))
        for field in ("link_length_nominal_m", "link_length_effective_m", "link_length_measured_m",
                      "link_calibration_window_s", "link_calibration_spread_m"):
            row[field] = float(target.get(field, math.nan))
        for field in ("link_calibration_enabled", "link_calibration_applied"):
            row[field] = int(bool(target.get(field, False)))
        row["link_calibration_samples"] = int(target.get("link_calibration_samples", 0))
        for field in ("link_calibration_status", "link_calibration_reason"):
            row[field] = str(target.get(field, ""))
        for field in ("payload_ground_reference_z_m", "payload_landing_duration_s",
                      "transport_support_requested_trim_n", "transport_support_trim_limit_n",
                      "effective_transport_link_gain_scale"):
            row[field] = float(target.get(field, math.nan))
        for field in ("payload_landing_touchdown_confirmed", "payload_landing_unloading",
                      "transport_support_initialization_clipped", "payload_landing_planning_fallback"):
            row[field] = int(bool(target.get(field, False)))
        for field in ("payload_landing_planning_failure_count", "payload_landing_planning_retry_count"):
            row[field] = int(target.get(field, 0))
        row["payload_landing_planning_failure_reason"] = str(
            target.get("payload_landing_planning_failure_reason", ""))
        row["payload_ground_clearance_m"] = float(
            target.get("payload_ground_clearance_m", math.nan)
        )
        row["payload_prelift_clearance_target_m"] = float(
            target.get("payload_prelift_clearance_target_m", math.nan)
        )
        row["payload_prelift_ready"] = int(bool(
            target.get("payload_prelift_ready", False)
        ))
        row["payload_prelift_active"] = int(bool(
            target.get("payload_prelift_active", False)
        ))
        row["payload_prelift_stage"] = target.get("payload_prelift_stage", "")
        row["payload_prelift_failure_reason"] = target.get("payload_prelift_failure_reason", "")
        for field in ("payload_prelift_reference_rise_m", "payload_prelift_reference_lead_m"):
            row[field] = float(target.get(field, math.nan))
        for prefix in (
                "payload_position_error", "payload_velocity_error",
                "payload_attitude_error", "link_direction",
                "desired_link_direction", "raw_desired_link_direction",
                "link_direction_error", "raw_link_force",
                "kinematic_perpendicular_force",
                "payload_desired_force", "payload_desired_moment"):
            if prefix in target:
                self._with_xyz(row, prefix, target[prefix])
            elif prefix in ("raw_link_force", "kinematic_perpendicular_force"):
                for axis in "xyz":
                    row[prefix + "_" + axis] = math.nan
        if "link_force" in target:
            self._with_xyz(row, "limited_link_force", target["link_force"])
        else:
            for axis in "xyz":
                row["limited_link_force_" + axis] = math.nan
        for field in ("link_force_rate_limited", "link_force_magnitude_limited"):
            row[field] = int(bool(target[field])) if field in target else math.nan
        row["payload_control_dt_s"] = float(target.get("payload_control_dt_s", math.nan))
        if "desired_tension" in target:
            row["desired_tension_n"] = float(target["desired_tension"])
        if "desired_tension_signed" in target:
            row["desired_tension_signed_n"] = float(target["desired_tension_signed"])
        if "payload_yaw_enabled" in target:
            row["payload_yaw_enabled"] = int(bool(target["payload_yaw_enabled"]))
        for field in (
                "transport_blend", "transport_blend_dot", "transport_link_gain_scale",
                "ramp_position_hold_gain_scale",
                "transport_position_hold_gain_scale"):
            if field in target:
                row[field] = float(target[field])
        if "payload_target_position" in target:
            self._with_xyz(row, "payload_target", target["payload_target_position"])
        if "payload_target_rotation" in target:
            rpy = rotation_to_rpy(target["payload_target_rotation"])
            axis = np.asarray(target["payload_target_rotation"],dtype=float)[:,2]
            self._with_xyz(row,"payload_target_body_z",axis)
            row["payload_target_cone_half_angle_deg"] = float(np.rad2deg(np.arccos(np.clip(axis[2],-1.,1.))))
            actual = np.asarray([row.get("payload_body_z_"+a,math.nan) for a in "xyz"])
            row["payload_cone_axis_error_deg"] = (float(np.rad2deg(np.arccos(np.clip(actual@axis,-1.,1.))))
                if np.all(np.isfinite(actual)) else math.nan)
            for name, value in zip(("roll", "pitch", "yaw"), rpy):
                row["payload_target_" + name + "_rad"] = float(value)
        for prefix in ("position_hold_position", "position_hold_velocity",
                       "position_hold_acceleration", "payload_target_body_rate",
                       "payload_target_body_rate_dot", "payload_target_velocity",
                       "payload_target_acceleration", "payload_circle_center",
                       "circle_formation_acceleration_feedforward"):
            if prefix in target:
                self._with_xyz(row, prefix, target[prefix])
        self._with_xyz(row, "transport_position_hold_force",
                       command.get("transport_position_hold_force", np.zeros(3)))
        row["payload_attitude_reference_progress"] = float(
            target.get("payload_attitude_reference_progress", math.nan))
        row["payload_transport_force_mode"] = str(target.get("payload_transport_force_mode", ""))
        for field in ("payload_trajectory_mode", "payload_trajectory_phase"):
            row[field] = str(target.get(field, ""))
        for field in ("payload_circle_progress", "payload_trajectory_progress", "payload_circle_radius_m", "payload_circle_yaw_rad",
                      "payload_cone_config_half_angle_deg", "payload_cone_period_s",
                      "payload_circle_yaw_rate_radps", "payload_circle_yaw_acceleration_radps2"):
            row[field] = float(target.get(field, math.nan))
        if "payload_lobe_centers" in target:
            for index, center in enumerate(target["payload_lobe_centers"], 1):
                self._with_xyz(row, "payload_lobe_center_%d" % index, center)
        for field in ("transport_hold_integral_enabled", "transport_hold_integral_learning"):
            row[field] = int(bool(target.get(field, False)))
        row["transport_hold_integral_freeze_reason"] = str(
            target.get("transport_hold_integral_freeze_reason", "inactive"))
        for prefix in ("transport_hold_integral_bias", "transport_hold_integral_error"):
            self._with_xyz(row, prefix, target.get(prefix, np.zeros(3)))
        for prefix in ("transport_hold_integral_force", "transport_hold_integral_force_dot"):
            self._with_xyz(row, prefix, command.get(prefix, np.zeros(3)))
        row["transport_feedback_controller"] = str(command.get(
            "transport_feedback_controller", "differential_integral"))
        row["transport_full_pid_freeze_reason"] = str(command.get(
            "transport_full_pid_freeze_reason", "inactive_mode"))
        row["transport_full_pid_integral_gate_m"] = float(command.get(
            "transport_full_pid_integral_gate_m", math.nan))
        row["transport_full_pid_integral_velocity_weight"] = float(command.get(
            "transport_full_pid_integral_velocity_weight", math.nan))
        for field in ("transport_attitude_trim_enabled", "transport_attitude_trim_learning",
                      "transport_attitude_trim_dynamic_enabled", "transport_attitude_trim_feedback_healthy",
                      "transport_attitude_trim_during_lift_enabled",
                      "transport_attitude_trim_limit_active", "transport_attitude_trim_rate_limit_active",
                      "transport_attitude_trim_acceleration_limit_active", "transport_attitude_trim_bridge_active"):
            row[field] = int(bool(target.get(field, False)))
        row["transport_attitude_trim_freeze_reason"] = str(
            target.get("transport_attitude_trim_freeze_reason", "inactive"))
        row["transport_attitude_trim_feedback_quality_reason"] = str(
            target.get("transport_attitude_trim_feedback_quality_reason", "inactive"))
        for field in ("transport_attitude_trim_body_rate_norm_rps",
                      "transport_attitude_trim_pose_step_deg", "transport_attitude_trim_pose_step_limit_deg"):
            row[field] = float(target.get(field, math.nan))
        for prefix in ("transport_attitude_trim_error", "transport_attitude_trim_integral",
                       "transport_attitude_trim_position", "transport_attitude_trim_velocity",
                       "transport_attitude_trim_acceleration", "transport_attitude_trim_bridge_position",
                       "transport_attitude_trim_bridge_velocity", "transport_attitude_trim_bridge_acceleration",
                       "transport_attitude_trim_bridge_hold_position", "transport_attitude_trim_bridge_hold_velocity",
                       "transport_attitude_trim_bridge_hold_acceleration"):
            self._with_xyz(row, prefix, target.get(prefix, np.zeros(3)))
        for field in ("transport_full_pid_learning", "transport_full_pid_duplicate_tick",
                      "transport_full_pid_gate_open", "transport_full_pid_integral_limit_active",
                      "transport_full_pid_feedback_limit_active", "transport_full_pid_force_budget_limited",
                      "transport_full_pid_rate_budget_limited"):
            row[field] = int(bool(command.get(field, False)))
        for prefix in ("transport_full_pid_integral_state", "transport_full_pid_integral_state_delta",
                       "transport_full_pid_integral_state_limit", "transport_full_pid_integral_gain",
                       "transport_full_pid_force", "transport_full_pid_force_dot",
                       "transport_full_pid_pd_force", "transport_full_pid_pd_force_dot",
                       "transport_full_pid_feedback_force", "transport_full_pid_feedback_force_dot"):
            self._with_xyz(row, prefix, command.get(prefix, np.zeros(3)))
        for field in ("force_limit_active", "hold_feedback_limit_active", "body_rate_limit_active",
                      "collective_limit_active", "independent_mode", "transport_mode"):
            row[field] = int(bool(command.get(field, False)))
        if "kinematic_specific_acceleration" in target:
            self._with_xyz(row, "kinematic_specific_acceleration", target["kinematic_specific_acceleration"])
        self._with_xyz(
            row, "position_integral", command.get("position_integral", np.zeros(3))
        )
        self._with_xyz(row, "attitude_error", command["attitude_error"])
        self._with_xyz(row, "computed_rate", command.get("computed_body_rate", np.zeros(3)))
        self._with_xyz(row, "command_rate", command["body_rate_command"])
        actual_rpy = rotation_to_rpy(state["rotation"])
        desired_rpy = rotation_to_rpy(command["computed_rotation"])
        row.update(dict(zip(("roll_rad", "pitch_rad", "yaw_rad"), actual_rpy)))
        row.update(dict(zip(("desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad"), desired_rpy)))
        row["target_yaw_rad"] = float(target["yaw"])
        row["omega_c_method"] = str(command.get("omega_c_method", "zero"))
        row["command_thrust_newton"] = float(command["collective_thrust"])
        if command["collective_thrust"] <= 0.0:
            row["command_raw_thrust"] = 0
            row["command_raw_thrust_age_s"] = 0.0
        else:
            with self.lock:
                raw_thrust = self.latest_raw_thrust
                raw_received_time = self.latest_raw_thrust_received_time
            if raw_thrust is not None and raw_received_time is not None:
                row["command_raw_thrust"] = int(raw_thrust)
                row["command_raw_thrust_age_s"] = max(
                    0.0, now - float(raw_received_time)
                )
        if control_state is not None:
            for prefix in (
                    "ekf_position", "ekf_velocity", "ekf_acceleration",
                    "control_velocity", "mixed_acceleration", "control_acceleration"):
                if prefix in control_state:
                    self._with_xyz(row, prefix, control_state[prefix])
            row["ekf_state_valid"] = int(
                bool(control_state.get("ekf_state_valid", False))
            )
            row["ekf_state_age_s"] = float(
                control_state.get("ekf_state_age_s", math.inf)
            )
            row["ekf_kinematics_weight_effective"] = float(
                control_state.get("ekf_kinematics_weight_effective", 0.0)
            )
            row["ekf_position_error_m"] = float(
                control_state.get("ekf_position_error_m", math.inf)
            )
            row["ekf_position_consistent"] = int(
                bool(control_state.get("ekf_position_consistent", False))
            )
            row["ekf_kinematics_held"] = int(
                bool(control_state.get("ekf_kinematics_held", False))
            )
            row["ekf_fault_age_s"] = float(
                control_state.get("ekf_fault_age_s", math.inf)
            )
            row["ekf_status"] = str(control_state.get("ekf_status", ""))
        self.logger.write(row)
