"""Cable takeup, effective-length calibration and independent prelift."""

from copy import deepcopy
from slung_link_calibration import SlungLinkCalibrator
from slung_load_controller import cable_distances
from slung_load_controller import link_lengths_vector
from slung_load_controller import payload_ground_clearance
from slung_load_controller import smoothstep5_profile
from slung_load_controller import takeup_distance_ready
from slung_load_controller import takeup_target_distance
from slung_load_controller import takeup_vehicle_targets
from slung_prelift_trajectory import SlungPreliftTrajectory
import math
import numpy as np
import rospy


class PayloadTakeupMixin:
    """Cable takeup, effective-length calibration and independent prelift."""

    def _payload_takeup_positions(self):
        """Return TAKEUP positions at the current load height and cable distance."""
        with self.payload_lock:
            geometry = self.last_valid_payload_geometry
        if geometry is None or self.slung_controller is None:
            return None
        try:
            equilibrium_links = self.slung_controller.equilibrium_link_units(
                geometry["rotation"]
            )
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            # Keep the MATLAB fallback behavior if the allocation matrix is
            # temporarily unusable; do not let one bad load pose kill the
            # multi-vehicle timer thread.
            rospy.logwarn_throttle(
                1.0,
                "负载静态平衡绳向求解失败，TAKEUP 使用外张回退几何：%s",
                error,
            )
            equilibrium_links = None
        takeup_payload_position = np.asarray(
            geometry["position"], dtype=float
        ).copy()
        # TAKEUP first establishes cable geometry at the current load height.
        # The separate pre-lift segment raises all three aircraft afterwards.
        return takeup_vehicle_targets(
            takeup_payload_position, geometry["rotation"],
            self.slung_controller.config.attachment_points_m,
            self.payload_takeup_target_distance_m,
            self.payload_takeup_outward_offset_m,
            link_units=equilibrium_links,
        ).T

    def _payload_takeup_overrides(self, now):
        """Confirm measured taut cables, then keep pulling until measured clearance."""
        if self.transport_mode != "slung_load" or self.slung_controller is None:
            return {}
        states = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None:
                return {}
            states.append(state)
        measured = np.asarray([state["position"] for state in states], dtype=float)
        targets = (self._payload_takeup_positions()
                   if self.payload_takeup_start_positions is None else None)
        if self.payload_takeup_start_positions is None:
            if targets is None:
                return {}
            self.payload_takeup_start_positions = measured.copy()
            self.payload_takeup_target_positions = targets.copy()
            self.payload_takeup_start_time = float(now)
            self.payload_mission_hold_time = float(now)
        duration = max(float(self.vehicles[0].trajectory_config.takeup_duration_s
                             or self.vehicles[0].trajectory_config.takeoff_settle_s), 1e-6)
        elapsed = float(now) - self.payload_takeup_start_time
        with self.payload_lock:
            payload_state = deepcopy(self.latest_payload_state)
        payload_ready = self._payload_state_ready(payload_state, now)
        motion_state = self._payload_prelift_motion_state(payload_state, now)
        motion_ready = motion_state is not None
        feedback_reused = motion_ready and not payload_ready
        clearance = (payload_ground_clearance(
            motion_state["position"], motion_state["rotation"],
            self.slung_controller.config.payload_size_m,
        ) if motion_ready else math.nan)
        self.payload_ground_clearance_m = clearance
        if self.payload_prelift_reference is not None:
            reference = self.payload_prelift_reference.update(measured, clearance, now)
        else:
            profile = smoothstep5_profile(elapsed, duration)
            displacement = self.payload_takeup_target_positions - self.payload_takeup_start_positions
            reference = dict(
                positions=self.payload_takeup_start_positions + profile[0] * displacement,
                velocities=profile[1] * displacement, accelerations=profile[2] * displacement,
                stage="takeup", active=False, failed=False, reason="", rise_m=0.0,
            )
            if self.payload_link_calibration_hold_reference is not None:
                reference.update(positions=self.payload_link_calibration_hold_reference.copy(),
                                 velocities=np.zeros((3, 3)), accelerations=np.zeros((3, 3)))
            if not payload_ready and self.payload_last_takeup_reference is not None:
                reference = dict(self.payload_last_takeup_reference,
                                 velocities=np.zeros((3, 3)), accelerations=np.zeros((3, 3)))
            if payload_ready and elapsed >= duration:
                distances = cable_distances(
                    payload_state["position"], payload_state["rotation"],
                    self.slung_controller.config.attachment_points_m, measured,
                )
                taut = takeup_distance_ready(
                    distances, self.slung_controller.config.link_lengths_m,
                    self.payload_takeup_target_distance_m, self.payload_takeup_distance_tolerance_m,
                )
                if taut:
                    if self.payload_geometry_ready_since is None:
                        self.payload_geometry_ready_since = float(now)
                    if now - self.payload_geometry_ready_since >= self.payload_takeup_confirm_s:
                        self.payload_takeup_taut_latched = True
                        self.payload_prelift_reference = SlungPreliftTrajectory(
                            **self.payload_prelift_parameters
                        )
                        self.payload_prelift_reference.start(measured, reference["positions"], now)
                        self.payload_link_calibration_hold_reference = None
                        reference = self.payload_prelift_reference.snapshot()
                else:
                    self.payload_geometry_ready_since = None
            if elapsed > duration + self.payload_prelift_parameters["timeout_s"]:
                reference.update(failed=True, reason="takeup_timeout", stage="failed")
        self.payload_last_takeup_reference = reference
        self._update_payload_link_calibration(now, measured, payload_state, reference, duration, states)
        if reference["failed"]:
            self._begin_payload_emergency_landing(now, "预抬参考达到边界：" + reference["reason"])
            return {}
        overrides = {}
        for index, (vehicle, state) in enumerate(zip(self.vehicles, states)):
            target = dict(vehicle._target_for(state, now))
            target.update({
                "position": reference["positions"][index].copy(),
                "velocity": reference["velocities"][index].copy(),
                "acceleration": reference["accelerations"][index].copy(),
                "jerk": np.zeros(3),
                "independent_mode": True, "transport_mode": False,
                "takeup_active": True, "flight_phase": "takeup",
                "payload_prelift_active": reference["active"],
                "payload_prelift_stage": reference["stage"],
                "payload_prelift_feedback_reused": feedback_reused,
                "payload_prelift_feedback_age_s": (
                    float(now) - float(motion_state["received_time"]) if motion_ready else math.nan),
                "payload_prelift_reference_rise_m": reference["rise_m"],
                "payload_prelift_reference_lead_m": float(
                    reference["positions"][index, 2] - measured[index, 2]
                ),
                "prelift_vertical_integral_active": (
                    motion_ready and reference["stage"] in (
                        "rising", "braking", "holding", "lead_hold"
                    )
                ),
                "prelift_vertical_integral_learning": payload_ready,
                "independent_position_gain_scale": (
                    self.payload_prelift_reference.position_gain_scale
                    if self.payload_prelift_reference is not None
                    else np.ones(3)
                ),
                "independent_velocity_gain_scale": (
                    self.payload_prelift_reference.velocity_gain_scale
                    if self.payload_prelift_reference is not None
                    else np.ones(3)
                ),
                "max_tilt_rad": (
                    self.payload_prelift_reference.max_tilt_rad
                    if self.payload_prelift_reference is not None
                    else vehicle.controller.config.max_tilt_rad
                ),
            })
            target.update(self.payload_prelift_integral_parameters)
            overrides[id(vehicle)] = target
        return overrides

    def _update_payload_link_calibration(self, now, measured, payload, reference, takeup_duration, states):
        """Use fresh, steady endpoint geometry as a bounded tautness proxy."""
        calibrator = self.payload_link_calibrator
        if not self.payload_link_calibration_enabled or self.payload_link_calibration_applied:
            return False
        if (self.payload_transport_active or self.payload_controlled_exit_reason
                or reference.get("failed", False) or reference.get("stage") not in (
                    "takeup", "rising", "braking", "holding", "lead_hold")
                or now - self.payload_takeup_start_time < takeup_duration
                or not self._payload_state_ready(payload, now)):
            self._reset_payload_link_calibration_window("phase_or_payload_not_ready")
            return False
        relative_velocities = []
        payload_age = (float(now)-float(payload.get("received_time", math.nan))
                       if payload is not None else math.nan)
        if (payload is None or not payload.get("valid", False) or not math.isfinite(payload_age)
                or payload_age > self.vehicles[0].state_timeout or payload_age < -.02):
            self._reset_payload_link_calibration_window("payload_snapshot_not_ready")
            return False
        for index, (vehicle, state) in enumerate(zip(self.vehicles, states)):
            state_age, sample_age = vehicle._mocap_state_ages(state, now)
            if (not state.get("valid", False) or not math.isfinite(state_age)
                    or state_age > vehicle.state_timeout or not math.isfinite(sample_age)
                    or sample_age > vehicle.state_timeout
                    or sample_age < -vehicle.mocap_timestamp_future_tolerance_s):
                self._reset_payload_link_calibration_window("vehicle_state_not_ready")
                return False
            control = vehicle._build_ekf_control_state(state, now)
            if not control.get("ekf_state_valid", False):
                self._reset_payload_link_calibration_window("vehicle_kinematics_not_ready")
                return False
            rho = payload["rotation"] @ self.slung_controller.config.attachment_points_m[:, index]
            attachment_velocity = (payload["velocity"]
                + np.cross(payload["rotation"] @ payload.get("body_rate", np.zeros(3)), rho))
            relative_velocities.append(attachment_velocity-control["control_velocity"])
        minimum_distance = min(float(np.linalg.norm(measured[a]-measured[b]))
                               for a,b in ((0,1),(0,2),(1,2)))
        eligible = (minimum_distance >= self.payload_transport_guard.min_distance_m
                    and np.max(np.linalg.norm(relative_velocities,axis=1))
                    <= self.payload_link_calibration_max_relative_speed_mps)
        if not eligible:
            self._reset_payload_link_calibration_window("motion_or_spacing_not_ready")
            return False
        distances = cable_distances(payload["position"], payload["rotation"],
                                   self.slung_controller.config.attachment_points_m, measured)
        self.payload_link_measured_distances = distances.copy()
        proposal = calibrator.update(payload.get("sample_time", payload["received_time"]), distances,
                                     eligible=eligible,
                                     mismatch_tolerance_m=self.payload_takeup_distance_tolerance_m)
        if proposal is not None:
            self.payload_link_calibration_pending = proposal.copy()
        proposal = self.payload_link_calibration_pending
        if proposal is None:
            if calibrator.status in ("out_of_bounds", "unstable"):
                rospy.logwarn_throttle(2., "有效绳长校准暂不采用：%s，距离波动=%s。",
                                       calibrator.reason, calibrator.spread_m.tolist())
            return False
        target_distances = takeup_target_distance(proposal, self.payload_takeup_pre_tension_slack_m)
        # A robust window may end in an outlier. Wait for an instantaneous
        # consistent frame before applying; never bypass the original gate.
        if not eligible or not takeup_distance_ready(distances, proposal, proposal,
                                                     self.payload_takeup_distance_tolerance_m):
            return False
        cfg = self.slung_controller.config
        try:
            links = self.slung_controller.equilibrium_link_units(payload["rotation"])
            new_targets = takeup_vehicle_targets(
                payload["position"], payload["rotation"], cfg.attachment_points_m,
                target_distances, self.payload_takeup_outward_offset_m, link_units=links).T
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            self._reset_payload_link_calibration_window("geometry_update_failed")
            rospy.logwarn_throttle(2., "有效绳长几何更新未采用：%s", error)
            return False
        cfg.link_lengths_m = proposal.copy()
        cfg.link_length_m = float(np.mean(proposal))
        self.payload_takeup_target_distance_m = np.asarray(target_distances).copy()
        self.payload_takeup_target_positions = new_targets
        if self.payload_prelift_reference is None:
            # The nominal TAKEUP has finished, so this reference has zero
            # derivatives. Hold it until the existing taut-confirmation then
            # align smoothly to measured anchors via SlungPreliftTrajectory.
            self.payload_link_calibration_hold_reference = reference["positions"].copy()
        self.payload_geometry_ready_since = None
        self.payload_takeup_ready_since = self.payload_prelift_ready_since = None
        self.payload_link_calibration_applied = True
        self.payload_link_calibration_pending = None
        rospy.loginfo("有效绳长校准完成：名义=%s，运行有效=%s，稳定窗口 %.3f s/%d 样本；YAML 未改写。",
                      self.payload_nominal_link_lengths.tolist(), proposal.tolist(),
                      calibrator.window_elapsed_s, calibrator.sample_count)
        return True

    def _reset_payload_link_calibration_window(self, reason):
        calibrator = self.payload_link_calibrator
        if self.payload_link_calibration_pending is not None and not self.payload_link_calibration_applied:
            # The module's accepted proposal has not reached the model yet.
            # Invalid input requires a new full window, not a stale proposal.
            settings = {name: getattr(calibrator, name) for name in (
                "window_s", "max_adjustment_m", "max_spread_m", "max_drift_mps",
                "max_sample_gap_s", "min_samples", "trim_fraction")}
            self.payload_link_calibrator = SlungLinkCalibrator(self.payload_nominal_link_lengths, **settings)
            self.payload_link_calibration_pending = None
        self.payload_link_calibrator.reset_window(reason)

    def _hold_payload_mission_clock(self, now):
        """Defer the task's hover/path time until prelift and reference lift finish."""
        if self.transport_mode != "slung_load" or self.payload_mission_hold_time is None:
            return
        release = (self.payload_transport_active and now >= self.payload_transport_start_time
                   + self.payload_tension_ramp_s + self.payload_reference_lift_s)
        dt = max(0.0, float(now) - self.payload_mission_hold_time)
        for vehicle in self.vehicles:
            trajectory = vehicle.trajectory
            if trajectory is None or trajectory.phase not in ("height_correction", "hover"):
                continue
            if release and trajectory.phase == "height_correction":
                duration = (trajectory.config.takeup_duration_s
                            or trajectory.config.takeoff_settle_s)
                trajectory.phase_start_time = now - duration
            else:
                trajectory.phase_start_time += dt
        self.payload_mission_hold_time = None if release else float(now)

    def _log_payload_takeup_status(self, now, reason, distances=None):
        """Explain why TAKEUP is not yet allowed to enter TAUT_RAMP."""
        interval = float(self.payload_takeup_diagnostic_interval_s)
        if (self.payload_takeup_diagnostic_last_time is not None and
                float(now) - self.payload_takeup_diagnostic_last_time < interval):
            return
        self.payload_takeup_diagnostic_last_time = float(now)

        with self.payload_lock:
            payload_state = None if self.latest_payload_state is None else dict(
                self.latest_payload_state
            )
        if payload_state is None:
            state_text = "负载状态=无"
        else:
            try:
                age = float(now) - float(payload_state["received_time"])
            except (KeyError, TypeError, ValueError):
                age = math.nan
            state_text = (
                "负载状态 valid=%d derivatives=%d age=%.3f s" % (
                    int(bool(payload_state.get("valid", False))),
                    int(bool(payload_state.get("derivatives_valid", False))),
                    age,
                )
            )

        length_text = "绳长不可计算"
        if distances is not None and self.slung_controller is not None:
            distances = np.asarray(distances, dtype=float).reshape(3)
            lengths = np.asarray(
                self.slung_controller.config.link_lengths_m, dtype=float
            ).reshape(3)
            targets = link_lengths_vector(
                self.payload_takeup_target_distance_m,
                "target_distance_m",
            )
            details = []
            for vehicle, distance, length, target in zip(
                    self.vehicles, distances, lengths, targets):
                error = float(distance - target)
                within = (
                    abs(error) <= self.payload_takeup_distance_tolerance_m and
                    distance <= length + self.payload_takeup_distance_tolerance_m
                )
                details.append(
                    "CF%d %.3f/目标%.3f m(配置%.3f,误差%+.3f,%s)" % (
                        vehicle.vehicle_id,
                        float(distance),
                        float(target),
                        float(length),
                        error,
                        "满足" if within else "不满足",
                    )
                )
            length_text = "；".join(details)

        rospy.logwarn(
            "TAKEUP 尚未进入 TAUT_RAMP：%s；%s；%s",
            reason,
            state_text,
            length_text,
        )

    def _payload_takeup_ready(self, now):
        """Confirm current cable geometry, held clearance and settled load velocity."""
        if self.transport_mode != "slung_load" or self.slung_controller is None:
            return True
        self.payload_prelift_ready = False
        self.payload_prelift_confirmation_waiting = False
        with self.payload_lock:
            payload = deepcopy(self.latest_payload_state)
        payload_ready = self._payload_state_ready(payload, now)
        motion_state = self._payload_prelift_motion_state(payload, now)
        if motion_state is None:
            self.payload_prelift_ready_since = self.payload_takeup_ready_since = None
            self._log_payload_takeup_status(now, "等待新鲜有效负载状态")
            return False
        payload = motion_state
        positions = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None:
                self.payload_prelift_ready_since = self.payload_takeup_ready_since = None
                return False
            positions.append(np.asarray(state["position"], dtype=float))
        distances = cable_distances(
            payload["position"], payload["rotation"],
            self.slung_controller.config.attachment_points_m, positions,
        )
        self.payload_link_measured_distances = distances.copy()
        ready = takeup_distance_ready(
            distances, self.slung_controller.config.link_lengths_m,
            self.payload_takeup_target_distance_m, self.payload_takeup_distance_tolerance_m,
        )
        clearance = payload_ground_clearance(
            payload["position"], payload["rotation"], self.slung_controller.config.payload_size_m,
        )
        self.payload_ground_clearance_m = clearance
        settled = (math.isfinite(float(payload["velocity"][2]))
                   and abs(float(payload["velocity"][2]))
                   <= self.payload_prelift_vertical_velocity_tolerance_mps)
        holding = (self.payload_prelift_reference is not None
                   and self.payload_prelift_reference.stage == "holding")
        if not (ready and holding and clearance >= self.payload_prelift_clearance_m and settled):
            self.payload_prelift_ready_since = self.payload_takeup_ready_since = None
            stage = (self.payload_prelift_reference.stage
                     if self.payload_prelift_reference is not None else "takeup")
            self._log_payload_takeup_status(
                now, "阶段=%s，间隙 %.3f/%.3f m，vz=%.3f m/s，当前绳长有效=%d" % (
                    stage, clearance, self.payload_prelift_clearance_m,
                    payload["velocity"][2], int(ready),
                ), distances,
            )
            return False
        if not payload_ready:
            # Retain an existing stable interval across a bounded sampling
            # wait, but only a newly accepted frame may confirm the handoff.
            self.payload_prelift_confirmation_waiting = True
            return False
        if self.payload_prelift_ready_since is None:
            self.payload_prelift_ready_since = float(now)
        self.payload_prelift_ready = now - self.payload_prelift_ready_since >= self.payload_prelift_confirm_s
        if not self.payload_prelift_ready:
            return False
        if self.payload_takeup_ready_since is None:
            self.payload_takeup_ready_since = float(now)
        return now - self.payload_takeup_ready_since >= self.payload_takeup_confirm_s
