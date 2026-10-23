"""Load trajectories and per-aircraft transport reference/force generation."""

from copy import deepcopy
from ctbr_control_math import project_so3
from ctbr_control_math import rope_preserving_vehicle_references
from ctbr_control_math import rotation_to_rpy
from slung_circle_reference import AttitudeBrakeReference
from slung_circle_reference import PayloadCircleReference
from slung_circle_reference import PayloadFigureEightReference
from slung_circle_reference import rotate_vehicle_references
from slung_attitude_cone_reference import PayloadConeReference, equilibrium_cable_reference
from slung_landing import PayloadTouchdownDetector
from slung_landing import SlungLandingProfile
from slung_load_controller import smoothstep5_profile
from slung_load_controller import translate_vehicle_references
from slung_prelift_trajectory import handoff_references
from slung_reference import attachment_rotation_reference
from slung_reference import cable_direction_reference
import math
import numpy as np
import rospy


class PayloadReferenceMixin:
    """Load trajectories and per-aircraft transport reference/force generation."""

    def _payload_reference_profile(self, now, desired_position):
        """Build MATLAB-style frozen-ground, tension-ramp, then lift reference."""
        if self.payload_transport_start_position is None:
            with self.payload_lock:
                state = self.latest_payload_state
            if state is None:
                return None
            self.payload_transport_start_position = np.asarray(
                state["position"], dtype=float
            ).copy()
        elapsed = max(
            0.0,
            float(now) - float(self.payload_transport_start_time),
        )
        ramp_s = float(self.payload_tension_ramp_s)
        lift_s = float(self.payload_reference_lift_s)
        circle_cfg = self.payload_circle_config
        if circle_cfg.mode in ("circle", "figure_eight", "attitude_cone"):
            if self.payload_circle_reference is None:
                start = (float(self.payload_transport_start_time)
                         + max(ramp_s + lift_s, self.payload_attitude_reference_s)
                         + circle_cfg.settle_s)
                reference_type = {"circle":PayloadCircleReference,
                                  "figure_eight":PayloadFigureEightReference,
                                  "attitude_cone":PayloadConeReference}[circle_cfg.mode]
                self.payload_circle_reference = reference_type(
                    desired_position, start, circle_cfg)
                if circle_cfg.mode == "attitude_cone":
                    rospy.loginfo(
                        "[负载轨迹 attitude_cone] 固定中心=%s m，半角 %.3f°，扫描周期 %.3f s，"
                        "进入/退出各 %.3f s；接管后 %.3f s 启动，总时长 %.3f s。",
                        self.payload_circle_reference.center_position.tolist(), circle_cfg.cone_half_angle_deg,
                        circle_cfg.cone_period_s, circle_cfg.cone_transition_s,
                        start-float(self.payload_transport_start_time), self.payload_circle_reference.duration)
                else:
                    rospy.loginfo(
                        "[负载轨迹 %s] 中心/交点=%s m，半径 %.3f m，中心高度 %.3f m，方向 %s，"
                        "最高速度 %.3f m/s；接管后 %.3f s 启动，参考时长 %.3f s。",
                        circle_cfg.mode, self.payload_circle_reference.center_position.tolist(), circle_cfg.radius_m,
                        self.payload_circle_reference.entry_position[2], circle_cfg.direction,
                        circle_cfg.speed_mps, start-float(self.payload_transport_start_time),
                        self.payload_circle_reference.duration)
            previous_phase = (self.payload_circle_state or {}).get("phase", "waiting")
            self.payload_circle_state = self.payload_circle_reference.evaluate(now)
            if previous_phase != self.payload_circle_state["phase"]:
                rospy.loginfo("[负载轨迹 %s] 参考阶段 %s，进度 %.1f%%；朝向策略=%s。",
                              circle_cfg.mode, self.payload_circle_state["phase"],
                              100.*self.payload_circle_state["progress"],
                              {"figure_eight":"沿运行切线", "circle":"朝向圆心",
                               "attitude_cone":"质心固定，机体 +Z 绕世界 +Z 扫锥"}[circle_cfg.mode])
            desired_position = self.payload_circle_reference.entry_position
            if float(now) >= self.payload_circle_reference.start_time:
                motion = self.payload_circle_state
                return (motion["position"].copy(), motion["velocity"].copy(),
                        motion["acceleration"].copy(), 1., 0.)
        if elapsed < ramp_s:
            profile = smoothstep5_profile(elapsed, ramp_s)
            return (
                self.payload_transport_start_position.copy(),
                np.zeros(3),
                np.zeros(3),
                float(profile[0]),
                float(profile[1]),
            )
        profile = smoothstep5_profile(elapsed - ramp_s, lift_s)
        displacement = np.asarray(desired_position, dtype=float) - self.payload_transport_start_position
        return (
            self.payload_transport_start_position + profile[0] * displacement,
            profile[1] * displacement,
            profile[2] * displacement,
            1.0,
            0.0,
        )

    def _circle_landing_reference(self, now):
        """Brake the current complete attitude, including startup alignment."""
        motion = self.payload_circle_state
        if motion is None:
            return None
        if self.payload_circle_landing_yaw is None:
            previous = next(iter(self.last_transport_overrides.values()), {})
            if motion["phase"] == "waiting" or self.payload_circle_config.mode == "attitude_cone":
                rotation = np.asarray(previous.get("payload_target_rotation",
                    self.latest_payload_state["rotation"]), dtype=float)
                rate = np.asarray(previous.get("payload_target_body_rate", np.zeros(3)), dtype=float)
                acceleration = np.asarray(previous.get("payload_target_body_rate_dot", np.zeros(3)), dtype=float)
            else:
                rotation = motion["rotation"].copy()
                rate = np.array([0.,0.,motion["yaw_rate"]])
                acceleration = np.array([0.,0.,motion["yaw_acceleration"]])
            brake = AttitudeBrakeReference(rotation, rate, acceleration,
                                           now, self.payload_circle_config.ramp_s)
            self.payload_circle_landing_yaw = dict(initial_rotation=rotation.copy(), reference=brake)
        profile = self.payload_circle_landing_yaw
        state = profile["reference"].evaluate(now)
        if (self.payload_circle_config.mode == "attitude_cone"
                and now >= profile["reference"].end_time):
            end = profile["reference"].evaluate(profile["reference"].end_time)
            blend = smoothstep5_profile(now-profile["reference"].end_time,
                                       self.payload_circle_config.cone_transition_s)
            level = attachment_rotation_reference(end["rotation"], np.eye(3),
                self.slung_controller.config.attachment_points_m, *blend)
            state = dict(rotation=level["rotation"], body_rate=level["omega_body"],
                         body_rate_dot=level["alpha_body"])
        return dict(state, yaw=float(rotation_to_rpy(state["rotation"])[2]),
                    yaw_rate=float(state["body_rate"][2]),
                    yaw_acceleration=float(state["body_rate_dot"][2]), phase="landing")

    def _cone_landing_profile(self, initial, ground_z, now, minimum_duration):
        """Brake translation, hold through attitude recovery, then descend.

        Reuse the bounded landing planner's C2 acceleration/velocity brake.
        Its final rest-to-rest descent starts only after the cone attitude has
        stopped and leveled. An already level, stationary attitude retains the
        original landing profile without an additional wait.
        """
        profile = SlungLandingProfile(*initial[:3], ground_z, now, minimum_duration,
            self.payload_landing_max_speed_mps, self.payload_landing_max_acceleration_mps2)
        attitude = self._circle_landing_reference(now)
        if (np.allclose(attitude["rotation"], np.eye(3), rtol=0., atol=1.e-12)
                and np.linalg.norm(attitude["body_rate"]) < 1.e-12
                and np.linalg.norm(attitude["body_rate_dot"]) < 1.e-12):
            return profile
        brake = self.payload_circle_landing_yaw["reference"]
        ready_time = brake.end_time+self.payload_circle_config.cone_transition_s
        speed_limit = max(self.payload_landing_max_speed_mps, float(np.linalg.norm(initial[1])))
        acceleration_limit = max(self.payload_landing_max_acceleration_mps2,
                                 float(np.linalg.norm(initial[2])))
        profile._build_braking_descent(minimum_duration, speed_limit, acceleration_limit)
        segments = profile._segments
        braking_duration = sum(duration for duration, _ in segments[:-1])
        hold_duration = max(0., ready_time-float(now)-braking_duration)
        stopped_position = segments[-1][1][0].copy()

        # Certify a conservative corner-height floor while attitude changes.
        # ||omega|| <= ||xi_dot|| for R=Rs exp(xi); the leveling geodesic has
        # speed <= pi*1.875/T. The Lipschitz margin covers between-grid angles.
        half_size = .5*self.slung_controller.config.payload_size_m
        coefficients = brake._coefficients
        vector_rate = coefficients[1:]*np.arange(1, len(coefficients))[:, None]/brake.duration
        angular_speed = max(SlungLandingProfile._peak_norm(vector_rate),
                            math.pi*1.875/self.payload_circle_config.cone_transition_s)
        times = np.linspace(float(now), ready_time, 129)
        support = max(float(np.abs(self._circle_landing_reference(t)["rotation"][2])@half_size)
                      for t in times)
        support += float(np.linalg.norm(half_size))*angular_speed*(times[1]-times[0])*.5
        required_height = ground_z-half_size[2]+support
        minimum_height = float(stopped_position[2])
        for duration, polynomial in segments[:-1]:
            critical = SlungLandingProfile._critical_points(
                polynomial[1:, 2]*np.arange(1, len(polynomial))/duration)
            minimum_height = min(minimum_height,
                float(np.min(np.polynomial.polynomial.polyval(critical, polynomial[:, 2]))))
        if minimum_height < required_height-1.e-10:
            raise ValueError("cone attitude recovery has insufficient box ground clearance")
        if hold_duration > 0.:
            hold = np.array([stopped_position, np.zeros(3), np.zeros(3)])
            segments = segments[:-1]+[(hold_duration, hold)]+segments[-1:]
        profile._set_segments(segments)
        return profile

    def _cone_equilibrium_vehicle_references(self, position, velocity, acceleration,
                                            rotation, omega, alpha, distances):
        """Use gravity-balanced directions for cone motion and attitude recovery."""
        core = self.slung_controller
        matrix = core._allocation_matrix()
        allocation = matrix.T@np.linalg.solve(matrix@matrix.T, np.eye(6))
        bias = core._internal_bias(matrix, core.config.payload_mass_kg, core.config.gravity_mps2)
        cables = equilibrium_cable_reference(rotation, omega, alpha, allocation, bias,
            np.array([0., 0., core.config.payload_mass_kg*core.config.gravity_mps2]))
        cross = lambda x: np.array([[0.,-x[2],x[1]],[x[2],0.,-x[0]],[-x[1],x[0],0.]])
        rotation_dot = rotation@cross(omega)
        rotation_ddot = rotation@(cross(alpha)+cross(omega)@cross(omega))
        rho = core.config.attachment_points_m
        lengths = np.asarray(distances, dtype=float)[None, :]
        return ((position[:, None]+rotation@rho-lengths*cables["directions"]).T,
                (velocity[:, None]+rotation_dot@rho-lengths*cables["direction_rates"]).T,
                (acceleration[:, None]+rotation_ddot@rho-lengths*cables["direction_accelerations"]).T)

    def _transport_overrides(self, now):
        if self.transport_mode != "slung_load" or not self.slung_controller:
            return {}
        with self.payload_lock:
            payload_state = deepcopy(self.latest_payload_state)
        if not self._payload_state_ready(payload_state, now):
            return {}
        targets = []
        vehicle_states = []
        control_states = []
        for vehicle in self.vehicles:
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._state_ready(now):
                return {}
            target = vehicle._target_for(state, now)
            flight_phase = target.get("flight_phase")
            if flight_phase not in (
                    "height_correction", "hover", "figure_eight", "landing",
                    "landing_settle"):
                return {}
            control_state = vehicle._build_ekf_control_state(state, now)
            control_states.append(control_state)
            vehicle_states.append({
                "position": state["position"],
                "velocity": control_state.get("control_velocity", np.zeros(3)),
                "acceleration": control_state.get("control_acceleration", np.zeros(3)),
                "body_rate": state.get("body_rate", np.zeros(3)),
                "mass": vehicle.controller.config.mass,
            })
            targets.append(target)

        target_velocity = np.mean([item["velocity"] for item in targets], axis=0)
        target_yaw = math.atan2(float(target_velocity[1]), float(target_velocity[0])) \
            if np.linalg.norm(target_velocity[:2]) > 1.0e-9 else 0.0
        target_rotation = np.array([
            [math.cos(target_yaw), -math.sin(target_yaw), 0.0],
            [math.sin(target_yaw), math.cos(target_yaw), 0.0],
            [0.0, 0.0, 1.0],
        ])
        if self.payload_circle_config.mode == "figure_eight":
            # Level and align with the first lobe's limiting tangent while the
            # load is still in the existing smooth startup reference.
            sign = 1. if self.payload_circle_config.direction == "ccw" else -1.
            yaw = -sign*math.pi/2.
            target_rotation = np.array([[math.cos(yaw),-math.sin(yaw),0.],
                [math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
        landing = any(target.get("flight_phase") in ("landing", "landing_settle")
                      for target in targets)
        circle_landing_motion = self._circle_landing_reference(now) if landing else None
        if circle_landing_motion is not None:
            target_rotation = circle_landing_motion["rotation"]
        if landing:
            if self.payload_landing_retry_tick == float(now):
                return self._payload_planning_hold(now)
            # A normal trajectory can enter landing before the validation
            # deadline.  Validation then clears the start timestamp while an
            # old profile may still exist; rebuild both pieces together so a
            # landing tick can never subtract ``None`` or reuse a pre-deadline
            # reference.
            if (self.payload_landing_profile is None
                    or self.payload_landing_start_time is None):
                # The landing profile must start at the current measured load
                # state.  Reusing payload_last_reference can point above the
                # load after the validation timer has elapsed.
                initial = (
                    payload_state["position"],
                    payload_state.get("velocity", np.zeros(3)),
                    payload_state.get("acceleration", np.zeros(3)),
                    1., 0.,
                )
                ground_z = (self.payload_ground_center_z if math.isfinite(self.payload_ground_center_z)
                            else .5 * float(self.payload_height_m))
                try:
                    landing_duration = float(self.vehicles[0].trajectory_config.landing_duration_s)
                    if self.payload_circle_config.mode == "attitude_cone":
                        profile = self._cone_landing_profile(initial, ground_z, now, landing_duration)
                    else:
                        profile = SlungLandingProfile(
                            initial[0], initial[1], initial[2], ground_z, now,
                            landing_duration,
                            self.payload_landing_max_speed_mps, self.payload_landing_max_acceleration_mps2)
                except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                    return self._payload_landing_planning_failed(now, error, payload_state)
                previous = next(iter(self.last_transport_overrides.values()), {})
                gains = (
                    float(previous.get("transport_position_hold_gain_scale", self.payload_ramp_position_hold_gain_scale)),
                    float(previous.get("effective_transport_link_gain_scale", self.payload_transport_link_gain_scale)),
                    float(previous.get("transport_max_tilt_rad", self.vehicles[0].controller.config.max_tilt_rad)),
                    float(previous.get("transport_blend", initial[3])),
                )
                detector = PayloadTouchdownDetector(
                    ground_clearance_m=self.payload_ground_clearance_reference_m,
                    **self.payload_touchdown_parameters)
                attachment_points = self.slung_controller.config.attachment_points_m
                measured_positions = np.asarray(
                    [item["position"] for item in vehicle_states], dtype=float
                )
                measured_attachments = (
                    np.asarray(payload_state["position"], dtype=float)[:, None]
                    + np.asarray(payload_state["rotation"], dtype=float)
                    @ attachment_points
                )
                relative = measured_attachments - measured_positions.T
                measured_distances = np.linalg.norm(relative, axis=0)
                directions = distances = None
                if (np.all(np.isfinite(relative)) and
                        np.all(measured_distances > 1.0e-6)):
                    directions = (
                        relative / measured_distances[None, :]
                    )
                    # Preserve a measured overrun rather than pulling an
                    # aircraft inward to the shorter nominal length.
                    distances = np.maximum(
                        measured_distances,
                        np.asarray(self.slung_controller.config.link_lengths_m, dtype=float),
                    )
                self.payload_landing_profile = profile
                self.payload_landing_start_time = float(now)
                self.payload_landing_start_position = np.asarray(initial[0]).copy()
                self.payload_landing_gain_snapshot = gains
                self.payload_touchdown_detector = detector
                self.payload_landing_rope_directions = directions
                self.payload_landing_rope_distances = distances
                rospy.loginfo("负载受控下降参考 %.2f s，速度上限 %.3f m/s，加速度上限 %.3f m/s²。",
                              self.payload_landing_profile.duration, self.payload_landing_max_speed_mps,
                              self.payload_landing_max_acceleration_mps2)
            blend_profile = smoothstep5_profile(now - self.payload_landing_start_time,
                                               self.payload_landing_gain_transition_s)
            initial_blend = self.payload_landing_gain_snapshot[3]
            try:
                payload_reference = (*self.payload_landing_profile.evaluate(now),
                                     initial_blend + (1-initial_blend)*blend_profile[0],
                                     (1-initial_blend)*blend_profile[1])
                if not all(np.all(np.isfinite(value)) for value in payload_reference):
                    raise ValueError("landing evaluation returned nonfinite references")
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
                return self._payload_landing_planning_failed(now, error, payload_state)
            self.payload_landing_retry_since = None
            self.payload_landing_retry_count = 0
        else:
            desired_payload_position = np.mean(
                [vehicle.trajectory.takeoff_position for vehicle in self.vehicles],
                axis=0,
            )
            desired_payload_position[2] -= float(np.mean(
                self.slung_controller.config.link_lengths_m
            ))
            if math.isfinite(self.payload_target_height_m):
                desired_payload_position[2] = self.payload_target_height_m
            payload_reference = self._payload_reference_profile(
                now, desired_payload_position
            )
        if payload_reference is None:
            return {}
        (
            payload_target_position,
            payload_target_velocity,
            payload_target_acceleration,
            transport_blend,
            transport_blend_dot,
        ) = payload_reference
        self.payload_last_reference = tuple(np.asarray(v).copy() if i < 3 else float(v)
                                            for i, v in enumerate(payload_reference))
        if self.payload_transport_vehicle_start_positions is None:
            # TAKEUP has already produced the exact per-vehicle handoff
            # anchors.  Keep those anchors and translate them with the load's
            # quintic reference instead of returning to the aircraft-only
            # trajectory when TAUT_RAMP reaches blend=1.
            if self.payload_takeup_target_positions is None:
                return {}
            self.payload_transport_vehicle_start_positions = (
                self.payload_takeup_target_positions.copy()
            )
        vehicle_reference_positions, vehicle_reference_velocities, vehicle_reference_accelerations = (
            translate_vehicle_references(
                self.payload_transport_vehicle_start_positions,
                self.payload_transport_start_position,
                payload_target_position,
                payload_target_velocity,
                payload_target_acceleration,
            )
        )
        physical_positions = vehicle_reference_positions.copy()
        hold_velocities = vehicle_reference_velocities.copy()
        hold_accelerations = vehicle_reference_accelerations.copy()
        attitude_reference = None
        attitude_progress = math.nan
        if not landing and self.payload_attitude_reference_enabled:
            if self.payload_transport_start_rotation is None:
                self.payload_transport_start_rotation = project_so3(payload_state["rotation"])
            if self.payload_transport_goal_rotation is None:
                # Freeze the hover heading as well as the measured start pose:
                # changing either endpoint mid-profile invalidates its rates.
                self.payload_transport_goal_rotation = target_rotation.copy()
            attitude_profile = smoothstep5_profile(
                max(0.0, now - self.payload_transport_start_time),
                self.payload_attitude_reference_s,
            )
            # Polynomial cancellation can put a quintic endpoint a few ULPs
            # outside [0,1]; never let roundoff terminate the control timer.
            attitude_profile[0] = np.clip(attitude_profile[0], 0.0, 1.0)
            attitude_progress = float(attitude_profile[0])
            attitude_reference = attachment_rotation_reference(
                self.payload_transport_start_rotation,
                self.payload_transport_goal_rotation,
                self.slung_controller.config.attachment_points_m,
                *attitude_profile,
            )
            if self.payload_equilibrium_reference_enabled:
                if self.payload_transport_initial_link_units is None:
                    initial_cables = (
                        self.payload_transport_start_position[:, None]
                        + self.payload_transport_start_rotation
                        @ self.slung_controller.config.attachment_points_m
                        - self.payload_transport_vehicle_start_positions.T
                    )
                    distances = np.linalg.norm(initial_cables, axis=0)
                    if np.any(distances <= 1.0e-6):
                        return {}
                    self.payload_transport_initial_link_distances = distances
                    self.payload_transport_initial_link_units = initial_cables / distances[None, :]
                    self.payload_transport_goal_link_units = self.slung_controller.equilibrium_link_units(
                        self.payload_transport_goal_rotation)
                link_reference = cable_direction_reference(
                    self.payload_transport_initial_link_units,
                    self.payload_transport_goal_link_units,
                    *attitude_profile,
                )
                distances = self.payload_transport_initial_link_distances[None, :]
                # A level payload alone does not make a consistent hover:
                # its cable directions must also agree with static allocation.
                # Preserve measured length throughout the direction transition.
                attitude_reference["position_offsets"] -= distances * (
                    link_reference["directions"] - self.payload_transport_initial_link_units)
                attitude_reference["velocity_offsets"] -= distances * link_reference["direction_rates"]
                attitude_reference["acceleration_offsets"] -= distances * link_reference["direction_accelerations"]
            target_rotation = attitude_reference["rotation"]
            physical_positions += attitude_reference["position_offsets"].T
            vehicle_reference_positions = physical_positions.copy()
            vehicle_reference_velocities += attitude_reference["velocity_offsets"].T
            vehicle_reference_accelerations += attitude_reference["acceleration_offsets"].T
            hold_velocities = vehicle_reference_velocities.copy()
            hold_accelerations = vehicle_reference_accelerations.copy()
        transport_hold_scale = self.payload_transport_position_hold_gain_scale
        transport_hold_scale_dot = 0.0
        if not landing and self.payload_handoff_reference_positions is not None:
            lift_profile = smoothstep5_profile(
                max(0.0, now - self.payload_transport_start_time - self.payload_tension_ramp_s),
                self.payload_reference_lift_s,
            )
            lift_progress = float(lift_profile[0])
            # Keep the measured anchors through reference lift, then fade to
            # the configured steady transport hold without a force step.
            hold_delta = self.payload_ramp_position_hold_gain_scale - transport_hold_scale
            transport_hold_scale += (1.0 - lift_progress) * hold_delta
            transport_hold_scale_dot = -float(lift_profile[1]) * hold_delta
        if self.payload_handoff_reference_positions is not None:
            blend_ddot = (float(smoothstep5_profile(
                float(now) - self.payload_transport_start_time, self.payload_tension_ramp_s
            )[2]) if transport_blend < 1.0 else 0.0)
            held_positions = (self.payload_handoff_reference_positions
                              + (payload_target_position - self.payload_transport_start_position))
            if attitude_reference is not None:
                # Rotate both sides of the old PID bias blend together. Its
                # difference then remains constant, so handoff_references'
                # analytic first/second derivatives stay valid.
                held_positions = held_positions + attitude_reference["position_offsets"].T
            vehicle_reference_positions, bias_velocities, bias_accelerations = handoff_references(
                physical_positions,
                held_positions,
                transport_blend, transport_blend_dot, blend_ddot,
            )
            vehicle_reference_velocities += bias_velocities
            vehicle_reference_accelerations += bias_accelerations
        link_gain_scale = transport_blend * self.payload_transport_link_gain_scale
        if (not landing and transport_blend >= 1.0
                and self.payload_transport_steady_link_gain_scale < 1.0):
            steady_profile = smoothstep5_profile(
                max(0.0, now - self.payload_transport_start_time - self.payload_tension_ramp_s),
                self.payload_reference_lift_s,
            )
            link_gain_scale *= (
                1.0 - (1.0 - self.payload_transport_steady_link_gain_scale)
                * float(steady_profile[0])
            )
        if landing:
            progress, progress_dot, _ = smoothstep5_profile(
                now - self.payload_landing_start_time, self.payload_landing_gain_transition_s)
            initial_hold, initial_link, _, _ = self.payload_landing_gain_snapshot
            # Keep horizontal support at least as strong through touchdown;
            # transition any necessary increase continuously from its value.
            final_hold = max(initial_hold, self.payload_ramp_position_hold_gain_scale)
            transport_hold_scale = initial_hold + progress * (final_hold - initial_hold)
            transport_hold_scale_dot = progress_dot * (final_hold - initial_hold)
            link_gain_scale = initial_link
            if (self.payload_landing_rope_directions is not None
                    and self.payload_landing_rope_distances is not None):
                vehicle_reference_positions, vehicle_reference_velocities, vehicle_reference_accelerations = (
                    rope_preserving_vehicle_references(
                        payload_target_position,
                        payload_target_velocity,
                        payload_target_acceleration,
                        (self.payload_circle_landing_yaw["initial_rotation"]
                         if circle_landing_motion is not None else target_rotation),
                        self.slung_controller.config.attachment_points_m,
                        self.payload_landing_rope_directions,
                        self.payload_landing_rope_distances,
                    )
                )
                if circle_landing_motion is not None and self.payload_circle_config.mode == "attitude_cone":
                    vehicle_reference_positions, vehicle_reference_velocities, vehicle_reference_accelerations = (
                        self._cone_equilibrium_vehicle_references(
                            payload_target_position, payload_target_velocity, payload_target_acceleration,
                            circle_landing_motion["rotation"], circle_landing_motion["body_rate"],
                            circle_landing_motion["body_rate_dot"], self.payload_landing_rope_distances))
                elif circle_landing_motion is not None:
                    # Startup may still contain roll/pitch motion. Preserve all
                    # three attitude axes while braking measured rope geometry.
                    r = circle_landing_motion["rotation"]
                    w, alpha = circle_landing_motion["body_rate"], circle_landing_motion["body_rate_dot"]
                    cross = lambda x: np.array([[0.,-x[2],x[1]],[x[2],0.,-x[0]],[-x[1],x[0],0.]])
                    initial_inverse = self.payload_circle_landing_yaw["initial_rotation"].T
                    q = r @ initial_inverse
                    qdot = r @ cross(w) @ initial_inverse
                    qddot = r @ (cross(alpha)+cross(w)@cross(w)) @ initial_inverse
                    relative = vehicle_reference_positions - payload_target_position
                    vehicle_reference_positions = payload_target_position + relative @ q.T
                    vehicle_reference_velocities = payload_target_velocity + relative @ qdot.T
                    vehicle_reference_accelerations = payload_target_acceleration + relative @ qddot.T
                physical_positions = vehicle_reference_positions.copy()
                hold_velocities = vehicle_reference_velocities.copy()
                hold_accelerations = vehicle_reference_accelerations.copy()
        circle_motion = self.payload_circle_state if not landing else None
        if circle_motion is not None and self.payload_circle_config.mode == "attitude_cone":
            base_rotation = target_rotation.copy()
            base_omega = (np.zeros(3) if attitude_reference is None else attitude_reference["omega_body"])
            base_alpha = (np.zeros(3) if attitude_reference is None else attitude_reference["alpha_body"])
            target_rotation = circle_motion["rotation"] @ base_rotation
            added_omega = base_rotation.T @ circle_motion["body_rate"]
            omega = base_omega+added_omega
            alpha = base_alpha+base_rotation.T@circle_motion["body_rate_dot"]-np.cross(base_omega,added_omega)
            attitude_reference = dict(omega_body=omega,alpha_body=alpha)
            if circle_motion["phase"] != "waiting":
                # A tilted load needs gravity-balanced cable directions. Merely
                # keeping the old directions (or rigidly tilting the full fleet)
                # conflicts with the static wrench even at zero tracking error.
                physical_positions, hold_velocities, hold_accelerations = (
                    self._cone_equilibrium_vehicle_references(
                        payload_target_position, payload_target_velocity, payload_target_acceleration,
                        target_rotation, omega, alpha, self.payload_transport_initial_link_distances))
                vehicle_reference_positions = physical_positions.copy()
                vehicle_reference_velocities = hold_velocities.copy()
                vehicle_reference_accelerations = hold_accelerations.copy()
        elif circle_motion is not None:
            # The finite SO(3)/S2 leveling profiles keep fixed endpoints.
            # Compose the circle's world-yaw motion onto their whole formation,
            # including both velocity and acceleration cross terms.
            arguments = (payload_target_position, payload_target_velocity,
                         payload_target_acceleration,
                         circle_motion["yaw"]-self.payload_circle_reference.initial_yaw,
                         circle_motion["yaw_rate"], circle_motion["yaw_acceleration"])
            vehicle_reference_positions, vehicle_reference_velocities, vehicle_reference_accelerations = (
                rotate_vehicle_references(vehicle_reference_positions,
                    vehicle_reference_velocities, vehicle_reference_accelerations, *arguments))
            physical_positions, hold_velocities, hold_accelerations = rotate_vehicle_references(
                physical_positions, hold_velocities, hold_accelerations, *arguments)
            base_rotation = target_rotation.copy()
            relative_rotation = circle_motion["rotation"] @ self.payload_circle_reference.initial_rotation.T
            target_rotation = relative_rotation @ base_rotation
            if attitude_reference is not None:
                attitude_reference = dict(attitude_reference)
                yaw_omega = base_rotation.T @ np.array([0., 0., circle_motion["yaw_rate"]])
                attitude_reference["alpha_body"] = (
                    attitude_reference["alpha_body"]
                    + base_rotation.T @ np.array([0., 0., circle_motion["yaw_acceleration"]])
                    - np.cross(attitude_reference["omega_body"], yaw_omega))
                attitude_reference["omega_body"] = attitude_reference["omega_body"] + yaw_omega
        if circle_landing_motion is not None:
            attitude_reference = dict(omega_body=circle_landing_motion["body_rate"].copy(),
                                     alpha_body=circle_landing_motion["body_rate_dot"].copy())
        # Apply the mode after every phase's gain selection, including an
        # existing landing snapshot. The core bypass and logged gain agree.
        if self.slung_controller.config.transport_force_mode == "reference_cascade":
            link_gain_scale = 0.0
        target = {
            "position": payload_target_position,
            "velocity": payload_target_velocity,
            "acceleration": payload_target_acceleration,
            "rotation": target_rotation,
            "body_rate": (np.zeros(3) if attitude_reference is None
                          else attitude_reference["omega_body"]),
            "body_rate_dot": (np.zeros(3) if attitude_reference is None
                              else attitude_reference["alpha_body"]),
            "payload_attitude_gain": self.payload_attitude_gain,
            "payload_rate_gain": self.payload_rate_gain,
            "payload_yaw_enabled": self.payload_yaw_enabled,
            "link_gain_scale": link_gain_scale,
            # During TENSION_RAMP the target position is the measured TAKEUP
            # anchor.  Keep an independent hold term while the load force is
            # blended in, as in Udaan's ramp_position_hold_commands().
            "ramp_position_hold_gain_scale": self.payload_ramp_position_hold_gain_scale,
            "transport_position_hold_gain_scale": (
                transport_hold_scale
            ),
            "transport_blend_dot": transport_blend_dot,
        }
        payload_control_dt = getattr(self, "payload_control_dt", now - self.last_tick)
        result = self.slung_controller.compute(
            payload_state, vehicle_states, target,
            payload_control_dt,
        )
        if result is None:
            return {}
        trim_dot = np.zeros(3)
        if not self.payload_support_trim.initialized and self.payload_handoff_forces is not None:
            self.payload_support_trim.initialize(
                self.payload_handoff_forces[:, 2],
                [output["force"][2] for output in result["vehicles"]],
                available_z=[v.controller.config.max_command_thrust for v in self.vehicles],
            )
        if self.payload_support_trim.initialized:
            saturated = [
                bool(getattr(vehicle, "last_control_command", None) is not None and
                     vehicle.last_control_command["collective_thrust"] >=
                     vehicle.controller.config.max_command_thrust - 0.01)
                for vehicle in self.vehicles
            ]
            trim, trim_dot = self.payload_support_trim.update(
                float(payload_target_position[2] - payload_state["position"][2]),
                getattr(self, "payload_control_dt", 1.0 / self.rate_hz), saturated,
            )
        else:
            trim = np.zeros(3)
        if landing and self.payload_unload_start_time is not None:
            unload = smoothstep5_profile(now - self.payload_unload_start_time, self.payload_landing_unload_s)
            trim = (1 - unload[0]) * self.payload_landing_trim_at_unload
            trim_dot = -unload[1] * self.payload_landing_trim_at_unload
        overrides = {}
        for index, vehicle in enumerate(self.vehicles):
            output = result["vehicles"][index]
            vehicle_cfg = vehicle.vehicle_config
            prelift_tilt = self.payload_prelift_parameters.get(
                "max_tilt_rad", vehicle.controller.config.max_tilt_rad)
            tilt_progress = float(smoothstep5_profile(
                max(0.0, now - self.payload_transport_start_time - self.payload_tension_ramp_s),
                self.payload_reference_lift_s,
            )[0])
            transport_tilt = (max(vehicle.controller.config.max_tilt_rad, self.payload_landing_gain_snapshot[2]) if landing else
                              (1.0 - tilt_progress) * max(prelift_tilt, vehicle.controller.config.max_tilt_rad)
                              + tilt_progress * vehicle.controller.config.max_tilt_rad)
            force = output["force"].copy()
            force[2] += trim[index]
            force_dot = output["force_dot"].copy()
            force_dot[2] += trim_dot[index]
            overrides[id(vehicle)] = {
                "transport_mode": True,
                "independent_mode": False,
                "transport_blend": transport_blend,
                "transport_blend_dot": transport_blend_dot,
                "transport_filter_reset": bool(transport_blend <= 1.0e-9),
                "position": vehicle_reference_positions[index].copy(),
                "velocity": vehicle_reference_velocities[index].copy(),
                "acceleration": vehicle_reference_accelerations[index].copy(),
                "position_hold_position": physical_positions[index].copy(),
                "position_hold_velocity": hold_velocities[index].copy(),
                "position_hold_acceleration": hold_accelerations[index].copy(),
                "payload_target_rotation": target_rotation.copy(),
                "payload_target_body_rate": target["body_rate"].copy(),
                "payload_target_body_rate_dot": target["body_rate_dot"].copy(),
                "payload_attitude_reference_progress": attitude_progress,
                "payload_transport_force_mode": result["force_mode"],
                "payload_trajectory_mode": self.payload_circle_config.mode,
                "payload_trajectory_phase": (circle_motion["phase"] if circle_motion is not None
                                             else "landing" if landing else "hover"),
                "payload_circle_progress": (circle_motion["progress"] if circle_motion is not None else 0.),
                "payload_circle_center": (self.payload_circle_reference.center_position
                                          if self.payload_circle_reference is not None else np.zeros(3)),
                "payload_circle_radius_m": self.payload_circle_config.radius_m,
                "payload_cone_config_half_angle_deg": self.payload_circle_config.cone_half_angle_deg,
                "payload_cone_period_s": self.payload_circle_config.cone_period_s,
                "payload_trajectory_progress": (circle_motion["progress"] if circle_motion is not None else 0.),
                "payload_lobe_centers": (self.payload_circle_reference.lobe_centers
                    if isinstance(self.payload_circle_reference, PayloadFigureEightReference)
                    else np.zeros((2,3))),
                "payload_circle_yaw_rad": (circle_motion["yaw"] if circle_motion is not None
                                           else circle_landing_motion["yaw"] if circle_landing_motion is not None else 0.),
                "payload_circle_yaw_rate_radps": (circle_motion["yaw_rate"] if circle_motion is not None
                                           else circle_landing_motion["yaw_rate"] if circle_landing_motion is not None else 0.),
                "payload_circle_yaw_acceleration_radps2": (circle_motion["yaw_acceleration"] if circle_motion is not None
                                           else circle_landing_motion["yaw_acceleration"] if circle_landing_motion is not None else 0.),
                "kinematic_specific_acceleration": output["kinematic_specific_acceleration"].copy(),
                "transport_position_hold_gain_scale_dot": transport_hold_scale_dot,
                "desired_force_override": force,
                "desired_force_dot_override": force_dot,
                "transport_support_trim_n": trim[index],
                "max_tilt_rad": transport_tilt,
                "transport_max_tilt_rad": transport_tilt,
                "transport_attitude_gain": vehicle.transport_attitude_gain.copy(),
                "transport_rate_gain": vehicle.transport_rate_gain.copy(),
                "payload_position_error": result["payload_position_error"],
                "payload_velocity_error": result["payload_velocity_error"],
                "payload_attitude_error": result["payload_attitude_error"],
                "payload_state_valid": True,
                "payload_target_position": payload_target_position,
                "link_direction": output["link_direction"],
                "desired_link_direction": output["desired_link_direction"],
                "raw_desired_link_direction": output["raw_desired_link_direction"],
                "link_direction_error": output["link_direction_error"],
                "desired_tension": output["desired_tension"],
                "desired_tension_signed": output["desired_tension_signed"],
                "payload_yaw_enabled": self.payload_yaw_enabled,
                "payload_desired_force": result["desired_force"],
                "payload_desired_moment": result["desired_moment"],
                "transport_link_gain_scale": self.payload_transport_link_gain_scale,
                "effective_transport_link_gain_scale": link_gain_scale,
                "payload_target_velocity": payload_target_velocity.copy(),
                "payload_target_acceleration": payload_target_acceleration.copy(),
                "transport_support_requested_trim_n": self.payload_support_trim.requested_bias[index],
                "transport_support_trim_limit_n": self.payload_support_trim.limits[index],
                "transport_support_initialization_clipped": bool(self.payload_support_trim.initialization_clipped[index]),
                "raw_link_force": output["raw_link_force"].copy(),
                "link_force": output["link_force"].copy(),
                "link_force_rate_limited": bool(output["link_force_rate_limited"]),
                "link_force_magnitude_limited": bool(output["link_force_magnitude_limited"]),
                "kinematic_perpendicular_force": output["kinematic_perpendicular_force"].copy(),
                "payload_control_dt_s": float(payload_control_dt),
                "ramp_position_hold_gain_scale": (
                    self.payload_ramp_position_hold_gain_scale
                ),
                "transport_position_hold_gain_scale": (
                    transport_hold_scale
                ),
                "transport_feedback_controller": self.payload_transport_feedback_controller,
                "transport_full_pid_integral_gate_m": self.payload_transport_pid_integral_gate_m,
                "transport_full_pid_integral_velocity_weight": self.payload_transport_pid_integral_velocity_weight,
                "transport_full_pid_integral_limit": (
                    np.asarray(vehicle.controller.config.independent_integral_limit, dtype=float)
                    * self.payload_transport_pid_integral_limit_scale
                ),
                "transport_full_pid_integral_gain_scale": self.payload_transport_pid_integral_gain_scale,
                "transport_full_pid_force_limit_n": self.payload_transport_pid_force_limit_n,
                "transport_full_pid_force_rate_limit_nps": self.payload_transport_pid_force_rate_limit_nps,
                "payload_landing_rope_preserving": bool(
                    landing and self.payload_landing_rope_directions is not None
                ),
                "payload_landing_rope_distance_m": (
                    float(self.payload_landing_rope_distances[index])
                    if landing and self.payload_landing_rope_distances is not None
                    else math.nan
                ),
                "transport_blend": transport_blend,
                "payload_position": payload_state["position"],
                "payload_raw_velocity": payload_state.get(
                    "raw_velocity", payload_state["velocity"]
                ),
                "payload_velocity": payload_state["velocity"],
                "payload_acceleration": payload_state["acceleration"],
                "payload_body_rate": payload_state["body_rate"],
                "payload_filter_derivatives_valid": payload_state.get(
                    "derivatives_valid", False
                ),
            }
            if self.payload_handoff_forces is not None:
                overrides[id(vehicle)]["handoff_force"] = self.payload_handoff_forces[index].copy()
            if transport_blend < 1.0:
                overrides[id(vehicle)]["flight_phase"] = "tension_ramp"
        try:
            self._apply_attitude_reference_trim(now, payload_state, control_states,
                                               overrides, landing)
            if circle_motion is not None or circle_landing_motion is not None:
                self._apply_circle_formation_feedforward(
                    overrides, circle_motion or circle_landing_motion, payload_control_dt)
        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
            if landing:
                return self._payload_landing_planning_failed(now, error, payload_state)
            raise
        self._apply_transport_hold_integral(now, payload_state, vehicle_states,
                                            control_states, overrides, landing)
        if not all(self._finite_transport_override(target) for target in overrides.values()):
            raise ValueError("cooperative controller returned nonfinite command references")
        for target in overrides.values():
            target["_payload_control_state"] = deepcopy(payload_state)
            target["_payload_control_time"] = float(now)
        return overrides

    def _apply_circle_formation_feedforward(self, overrides, motion, dt):
        """Add the cable's tangential reference acceleration once, in N.

        The core already includes desired attachment acceleration and measured
        cable centripetal acceleration. Subtract the reference centripetal part
        from the full aircraft-minus-attachment acceleration before adding it.
        At exact tracking this yields tension + m*(aircraft_reference_a + g).
        At rest the extra term is zero, preserving the existing hover law.
        """
        current = []
        for index, vehicle in enumerate(self.vehicles):
            item = overrides[id(vehicle)]
            rotation = item["payload_target_rotation"]
            omega = item["payload_target_body_rate"]
            alpha = item["payload_target_body_rate_dot"]
            rho = self.slung_controller.config.attachment_points_m[:, index]
            relative = (item["payload_target_position"] + rotation @ rho
                        - item["position_hold_position"])
            length = float(np.linalg.norm(relative))
            if length <= 1.e-6:
                raise ValueError("circle reference cable has zero length")
            direction = relative / length
            relative_velocity = (item["payload_target_velocity"]
                + rotation @ np.cross(omega, rho) - item["position_hold_velocity"])
            direction_rate = (relative_velocity - direction*(direction @ relative_velocity))/length
            attachment_acceleration = (item["payload_target_acceleration"]
                + rotation @ (np.cross(omega, np.cross(omega, rho)) + np.cross(alpha, rho)))
            correction = (item["position_hold_acceleration"] - attachment_acceleration
                          - length * float(direction_rate @ direction_rate) * direction)
            # Preserve the legacy leveling/trim force during startup; the circle
            # itself is the only new source of formation acceleration.
            if motion["phase"] == "waiting":
                correction = np.zeros(3)
            force = vehicle.controller.config.mass * correction
            item["desired_force_override"] += force
            item["circle_reference_link_direction"] = direction
            item["circle_formation_acceleration_feedforward"] = correction
            current.append(force)
        current = np.asarray(current)
        if self.payload_circle_previous_correction is not None:
            rate = (current-self.payload_circle_previous_correction)/float(np.clip(dt, .001, .05))
            for index, vehicle in enumerate(self.vehicles):
                overrides[id(vehicle)]["desired_force_dot_override"] += rate[index]
        self.payload_circle_previous_correction = current
