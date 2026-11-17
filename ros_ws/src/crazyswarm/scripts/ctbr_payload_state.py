"""Payload observation, paths, feedback validity and ground baseline."""

from copy import deepcopy
from ctbr_control_math import quaternion_to_rotation
from geometry_msgs.msg import PoseStamped
from slung_load_controller import mocap_top_surface_to_center
from slung_load_controller import payload_ground_clearance
from slung_load_controller import cable_distances
import math
import numpy as np
import rospy
import threading
import time


class PayloadStateMixin:
    """Payload observation, paths, feedback validity and ground baseline."""

    def _report_payload_link_lengths(self, now, payload, vehicles, overrides, landing):
        """Diagnostic only: use the same geometry snapshot as the load core."""
        positions = np.asarray([state["position"] for state in vehicles])
        distances = cable_distances(payload["position"], payload["rotation"],
            self.slung_controller.config.attachment_points_m, positions)
        self.payload_link_measured_distances = distances.copy()
        for i, vehicle in enumerate(self.vehicles):
            overrides[id(vehicle)]["link_length_measured_m"] = float(distances[i])
            overrides[id(vehicle)]["link_length_measurement_sample_time_s"] = float(
                payload.get("sample_time", math.nan))
        reporter = self.payload_link_length_reporter
        if reporter is None:
            return
        first = next(iter(overrides.values()))
        stamp = float(payload.get("sample_time", math.nan))
        stamps = np.asarray([state.get("sample_time", math.nan) for state in vehicles])
        target_rotation = first["payload_target_rotation"]
        history = (payload.get("reset_count", 0), payload.get("rejected_count", 0))
        if history != self.payload_link_length_report_history:
            reporter.reset_window()
            self.payload_link_length_report_history = history
        eligible = bool(not landing and not self.payload_controlled_exit_reason
            and now-self.payload_transport_start_time >= max(
                self.payload_tension_ramp_s+self.payload_reference_lift_s,
                self.payload_attitude_reference_s if self.payload_attitude_reference_enabled else 0.)
            and all(item.get("transport_blend", 0.) >= 1. for item in overrides.values())
            and self._payload_state_ready(payload, now)
            and np.all(np.isfinite(stamps)) and math.isfinite(stamp)
            and np.max(np.abs(stamps-stamp)) <= .03
            and -.02 <= float(rospy.Time.now().to_sec())-stamp <= .10
            and payload_ground_clearance(payload["position"], payload["rotation"],
                self.slung_controller.config.payload_size_m) >= .10
            and float(target_rotation[2, 2]) >= math.cos(math.radians(reporter.level_tolerance_deg))
            and np.linalg.norm(first["payload_target_velocity"]) < .01
            and np.linalg.norm(first["payload_target_body_rate"]) < .01
            and all(item.get("desired_tension_signed", 0.) > 0. for item in overrides.values()))
        result = reporter.update(stamp, payload["position"], payload["rotation"], positions, eligible)
        if result is not None:
            parts = ["CF%d %.4f m（P10–P90 %.4f–%.4f，配置 %.4f，有效 %.4f）" % (
                vehicle.vehicle_id, result["median_m"][i], result["p10_m"][i], result["p90_m"][i],
                self.payload_nominal_link_lengths[i], self.slung_controller.config.link_lengths_m[i])
                for i, vehicle in enumerate(self.vehicles)]
            rospy.loginfo("[调平后绳长统计] 窗口 %.2f s，%d 个独立样本，倾角中位数 %.2f°；%s；"
                          "端点为负载挂点到飞机动捕原点，供核对参数，未自动修改绳长。",
                          result["window_s"], result["samples"], result["median_tilt_deg"], "；".join(parts))
        last = reporter.last_report
        for i, vehicle in enumerate(self.vehicles):
            overrides[id(vehicle)].update(
                link_length_report_sequence=0 if last is None else last["sequence"],
                link_length_report_sample_time_s=math.nan if last is None else last["sample_time"],
                link_length_report_median_m=math.nan if last is None else last["median_m"][i],
                link_length_report_p10_m=math.nan if last is None else last["p10_m"][i],
                link_length_report_p90_m=math.nan if last is None else last["p90_m"][i],
                link_length_report_samples=0 if last is None else last["samples"],
                link_length_report_window_s=math.nan if last is None else last["window_s"],
                link_length_report_status=reporter.status)

    def _payload_state_callback(self, message):
        """Cache accepted raw geometry and pose-only observer diagnostics.

        Rejected source samples remain visible in ``raw_*`` fields.  Control
        geometry keeps the latest accepted complete raw pose, preserving the
        existing position/attitude delay for normally sampled measurements.
        """
        def raw_vector(*path):
            try:
                value = message
                for name in path:
                    value = getattr(value, name)
                return np.array([value.x, value.y, value.z], dtype=float)
            except (AttributeError, TypeError, ValueError):
                return np.full(3, np.nan)

        received_time = time.monotonic()
        position = raw_vector("pose", "position")
        raw_velocity = raw_vector("twist", "linear")
        raw_acceleration = raw_vector("acceleration")
        raw_body_rate = raw_vector("twist", "angular")
        try:
            source_stamp = float(message.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            source_stamp = math.nan
        try:
            source_age = float(rospy.Time.now().to_sec()) - source_stamp
            future_tolerance = float(getattr(
                self.vehicles[0], "mocap_timestamp_future_tolerance_s", .02))
            source_time_valid = bool(
                math.isfinite(source_stamp) and source_stamp > 0.0 and
                math.isfinite(source_age) and
                -future_tolerance - 1.e-12 <= source_age <= self.vehicles[0].state_timeout
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            source_time_valid = False
        try:
            rotation = quaternion_to_rotation(
                message.pose.orientation.x, message.pose.orientation.y,
                message.pose.orientation.z, message.pose.orientation.w,
            )
        except (AttributeError, TypeError, ValueError):
            rotation = np.full((3, 3), np.nan)
        center_position = np.full(3, np.nan)
        if np.all(np.isfinite(position)) and np.all(np.isfinite(rotation)):
            center_position = mocap_top_surface_to_center(
                position, rotation, self.payload_height_m
            )
        source_valid = bool(
            getattr(message, "valid", False) and
            np.all(np.isfinite(center_position)) and
            np.all(np.isfinite(rotation)) and
            source_time_valid
        )
        try:
            observer = getattr(self, "payload_state_observer", None)
            if observer is not None:
                # Twist and acceleration remain diagnostics.  An invalid ROS
                # message becomes a rejected observer sample, never a reset.
                observer_lock = getattr(
                    self, "payload_state_observer_lock", threading.Lock()
                )
                with observer_lock:
                    observed = observer.update(
                        center_position if source_valid else np.full(3, np.nan),
                        rotation,
                        source_stamp if source_time_valid else math.nan,
                    )
                    if observed is not None and observed.get("observer_updated", False):
                        self.payload_accepted_pose = {
                            "position": center_position.copy(),
                            "rotation": rotation.copy(),
                        }
                    accepted_pose = getattr(self, "payload_accepted_pose", None)
                if observed is None:
                    raise ValueError("负载位姿观测器拒绝当前样本")
                state = dict(observed)
                # The observer's corrected geometry is diagnostic only; raw
                # accepted geometry avoids adding control filtering delay.
                state["observer_position"] = observed["position"]
                state["observer_rotation"] = observed["rotation"]
                state["position"] = (
                    center_position.copy() if accepted_pose is None else
                    accepted_pose["position"].copy()
                )
                state["rotation"] = (
                    rotation.copy() if accepted_pose is None else
                    accepted_pose["rotation"].copy()
                )
                state["derivatives_valid"] = bool(
                    source_valid and observed.get("observer_updated", False) and
                    observed.get("observer_quality_valid", False) and
                    observed.get("derivatives_valid", False)
                )
            else:
                # Compatibility path for legacy callers without an observer.
                if not source_valid:
                    with self.payload_lock:
                        self.latest_payload_state = None
                    return
                velocity = np.where(np.isfinite(raw_velocity), raw_velocity, 0.0)
                if not bool(message.derivatives_valid):
                    with self.payload_velocity_filter_lock:
                        self.payload_velocity_filter.reset()
                    velocity = np.zeros(3)
                    acceleration = np.zeros(3)
                    filtered_derivatives_valid = False
                else:
                    with self.payload_velocity_filter_lock:
                        velocity, acceleration, filtered_derivatives_valid = (
                            self.payload_velocity_filter.update(velocity, source_stamp)
                        )
                    if velocity is None:
                        raise ValueError("负载速度低通滤波器拒绝当前样本")
                state = {
                    "position": center_position.copy(),
                    "rotation": rotation.copy(),
                    "velocity": velocity,
                    "acceleration": acceleration,
                    "body_rate": np.where(np.isfinite(raw_body_rate), raw_body_rate, 0.0),
                    "sample_time": source_stamp,
                    "derivatives_valid": bool(filtered_derivatives_valid),
                }
            state.update({
                "valid": source_valid,
                "source_valid": source_valid,
                "source_stamp": source_stamp,
                "raw_top_position": position.copy(),
                "raw_position": center_position.copy(),
                "raw_rotation": rotation.copy(),
                "raw_velocity": raw_velocity,
                "raw_acceleration": raw_acceleration,
                "raw_body_rate": raw_body_rate,
                "received_time": received_time,
            })
        except (AttributeError, TypeError, ValueError):
            with self.payload_lock:
                self.latest_payload_state = None
            return
        with self.payload_lock:
            self.latest_payload_state = state
            if state["derivatives_valid"]:
                self.payload_prelift_accepted_state = deepcopy(state)
                self.last_valid_payload_geometry = {
                    "position": state["position"].copy(),
                    "rotation": state["rotation"].copy(),
                }
            elif (state.get("observer_status") != "pending_interval"
                  or not state.get("source_valid", False)
                  or state.get("consecutive_rejected_count", 0)):
                # A real rejection or warmup invalidates prelift continuity.
                # A later short interval cannot revive a pre-fault snapshot.
                self.payload_prelift_accepted_state = None
        if (state.get("observer_updated", state["valid"]) and
                np.all(np.isfinite(state["position"]))):
            self._publish_payload_path(state)

    def _publish_payload_path(self, state):
        """Publish the measured load-center path for RViz."""
        now = time.monotonic()
        interval = self.payload_path_publish_interval_s
        if now - self.payload_path_last_publish_time < interval:
            return
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.payload_path_message.header.frame_id
        pose.pose.position.x = float(state["position"][0])
        pose.pose.position.y = float(state["position"][1])
        pose.pose.position.z = float(state["position"][2])
        rotation = state.get("rotation")
        if rotation is not None:
            # A Path pose needs a quaternion.  Keep this conversion local so
            # the plotted/path position remains the geometric load center.
            pose.pose.orientation.w = 1.0
        self.payload_path_message.header.stamp = pose.header.stamp
        self.payload_path_message.poses.append(pose)
        if len(self.payload_path_message.poses) > self.payload_path_max_poses:
            del self.payload_path_message.poses[:-self.payload_path_max_poses]
        self.payload_path_last_publish_time = now
        self.payload_path_publisher.publish(self.payload_path_message)

    def _payload_ready(self, now):
        if self.transport_mode != "slung_load":
            return True
        with self.payload_lock:
            state = self.latest_payload_state
        return self._payload_state_ready(state, now)

    def _payload_state_ready(self, state, now):
        """Check one snapshot, retaining compatibility with legacy states."""
        if self.transport_mode != "slung_load":
            return True
        if state is None:
            return False
        observer_state = "observer_status" in state or "observer_quality_valid" in state
        if observer_state and not (
                state.get("source_valid", False) and
                state.get("observer_updated", False) and
                state.get("observer_quality_valid", False)):
            return False
        try:
            age = float(now) - float(state["received_time"])
            timeout = self.vehicles[0].state_timeout
            if not (state.get("valid", False) and
                    state.get("derivatives_valid", False) and
                    math.isfinite(age) and age <= timeout):
                return False
            if observer_state:
                future_tolerance = float(getattr(
                    self.vehicles[0], "mocap_timestamp_future_tolerance_s", .02))
                if age < -future_tolerance - 1.e-12:
                    return False
                source_stamp = float(state["source_stamp"])
                sample_time = float(state["sample_time"])
                source_age = float(rospy.Time.now().to_sec()) - sample_time
                if not (math.isfinite(source_stamp) and source_stamp > 0.0 and
                        math.isfinite(sample_time) and sample_time > 0.0 and
                        math.isfinite(source_age) and
                        -future_tolerance - 1.e-12 <= source_age <= timeout):
                    return False
            return True
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

    def _payload_prelift_motion_state(self, state, now):
        """Reuse a recent accepted pose only for a benign short-interval wait.

        This does not change readiness for core control, calibration, landing
        release or handoff. Neither the accepted source nor receipt time is
        refreshed by pending frames.
        """
        if self._payload_state_ready(state, now):
            return state
        if (state is None or state.get("observer_status") != "pending_interval"
                or not state.get("valid", False) or not state.get("source_valid", False)
                or state.get("consecutive_rejected_count", 0)):
            return None
        with self.payload_lock:
            accepted = deepcopy(getattr(self, "payload_prelift_accepted_state", None))
        if not self._payload_state_ready(accepted, now):
            return None
        try:
            observer = self.payload_state_observer
            max_age = min(float(observer.max_dt), float(self.vehicles[0].state_timeout), .05)
            receipt_age = float(now) - float(accepted["received_time"])
            source_age = float(rospy.Time.now().to_sec()) - float(accepted["sample_time"])
            same_history = all(state.get(key) == accepted.get(key)
                               for key in ("sample_time", "sample_count", "reset_count", "rejected_count"))
            if (not same_history or not math.isfinite(receipt_age)
                    or not math.isfinite(source_age)
                    or receipt_age > max_age or source_age > max_age):
                return None
        except (AttributeError, KeyError, TypeError, ValueError):
            return None
        return accepted

    def _update_payload_ground_reference(self, now):
        """Measure the resting load origin before TAKEUP, then freeze that floor."""
        if self.payload_takeup_start_time is not None or self.payload_transport_active:
            return
        with self.payload_lock:
            payload = deepcopy(self.latest_payload_state)
        if not self._payload_state_ready(payload, now):
            return
        phases = [v.trajectory.phase if v.trajectory is not None else "waiting_for_ekf" for v in self.vehicles]
        if self.payload_ground_reference_samples and not all(p in (
                "waiting_for_ekf", "waiting_for_fleet", "pre_takeoff_hold", "waiting_for_reset") for p in phases):
            return
        clearance = payload_ground_clearance(payload["position"], payload["rotation"], self.slung_controller.config.payload_size_m)
        self.payload_ground_reference_samples.append((float(payload["position"][2]), max(0., clearance)))
        self.payload_ground_center_z = float(np.median([v[0] for v in self.payload_ground_reference_samples]))
        self.payload_ground_clearance_reference_m = float(np.median([v[1] for v in self.payload_ground_reference_samples]))
