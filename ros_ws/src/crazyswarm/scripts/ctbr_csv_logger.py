"""CTBR CSV schema and synchronized file writing."""

from datetime import datetime
import csv
import os
import threading


class FlightCsvLogger:
    """以固定列、线程安全方式写入一次飞行的 CSV。

    控制定时器和 ROS 回调可能并发运行，因此文件写入使用独立锁。每 25 行 flush 一次，
    既减少磁盘开销，也尽量保留异常中断前的数据。字段名是可视化脚本与后续 MATLAB/Python
    分析的稳定接口，新字段只能追加而不应重命名旧字段。
    """

    FIELDS = [
        "control_time_s", "ros_time_s", "mode", "flight_phase", "state_valid",
        "derivatives_valid", "state_age_s",
        # 来自 /cf<ID>/battery（GenericLogData）的固件主电池遥测；无样本时写 NaN/-1。
        "battery_voltage_v", "battery_voltage_mv", "battery_state", "battery_level_percent",
        "battery_age_s",
        # 起飞前冻结的电压标定结果。raw scale 只影响 C++ 桥接的 PWM 映射，不改变
        # command_thrust_newton 的物理单位含义。
        "preflight_voltage_v", "preflight_voltage_sample_count",
        "preflight_voltage_ready", "preflight_voltage_failed", "thrust_raw_scale",
        "position_x", "position_y", "position_z",
        "velocity_x", "velocity_y", "velocity_z",
        "acceleration_x", "acceleration_y", "acceleration_z",
        "body_rate_x", "body_rate_y", "body_rate_z",
        "roll_rad", "pitch_rad", "yaw_rad",
        "target_x", "target_y", "target_z", "target_yaw_rad",
        "target_jerk_x", "target_jerk_y", "target_jerk_z",
        "position_error_x", "position_error_y", "position_error_z",
        "velocity_error_x", "velocity_error_y", "velocity_error_z",
        "desired_force_x", "desired_force_y", "desired_force_z",
        "desired_roll_rad", "desired_pitch_rad", "desired_yaw_rad",
        "attitude_error_x", "attitude_error_y", "attitude_error_z",
        "computed_rate_x", "computed_rate_y", "computed_rate_z", "omega_c_method",
        "command_rate_x", "command_rate_y", "command_rate_z",
        "command_thrust_newton",
        # 仅追加诊断列，保持既有 MATLAB/CSV 按列号读取的布局不变。
        "filtered_velocity_x", "filtered_velocity_y", "filtered_velocity_z",
        "filtered_acceleration_x", "filtered_acceleration_y", "filtered_acceleration_z",
        "filter_derivatives_valid",
        "mocap_sample_age_s",
        # EKF 仅参与平移反馈；position/rotation 列仍来自 NOKOV。
        "ekf_position_x", "ekf_position_y", "ekf_position_z",
        "ekf_velocity_x", "ekf_velocity_y", "ekf_velocity_z",
        "ekf_acceleration_x", "ekf_acceleration_y", "ekf_acceleration_z",
        "ekf_state_valid", "ekf_state_age_s", "ekf_kinematics_weight_effective",
        "control_velocity_x", "control_velocity_y", "control_velocity_z",
        "mixed_acceleration_x", "mixed_acceleration_y", "mixed_acceleration_z",
        "control_acceleration_x", "control_acceleration_y", "control_acceleration_z",
        # Multi-vehicle task metadata.  Appended to preserve the legacy prefix.
        "mission_time_s", "vehicle_id", "radio_uri", "orbit_phase_rad",
        "formation_state", "global_abort_reason",
        # 诊断列只追加，避免破坏旧 MATLAB/CSV 的列号布局。
        "position_integral_x", "position_integral_y", "position_integral_z",
        "command_raw_thrust", "command_raw_thrust_age_s", "invalid_reason",
        # EKF 健康状态追加在末尾；不改变旧 CSV 的列号布局。
        "ekf_position_error_m", "ekf_position_consistent",
        "ekf_kinematics_held", "ekf_fault_age_s", "ekf_status",
        # Slung-load diagnostics appended after the legacy CSV schema.
        "payload_state_valid",
        "payload_position_error_x", "payload_position_error_y", "payload_position_error_z",
        "payload_velocity_error_x", "payload_velocity_error_y", "payload_velocity_error_z",
        "payload_attitude_error_x", "payload_attitude_error_y", "payload_attitude_error_z",
        "link_direction_x", "link_direction_y", "link_direction_z",
        "desired_link_direction_x", "desired_link_direction_y", "desired_link_direction_z",
        "link_direction_error_x", "link_direction_error_y", "link_direction_error_z",
        "desired_tension_n",
        # 负载原始/滤波状态；这些列在每架飞机行中重复，便于合并日志离线绘图。
        "payload_position_x", "payload_position_y", "payload_position_z",
        "payload_raw_velocity_x", "payload_raw_velocity_y", "payload_raw_velocity_z",
        "payload_raw_acceleration_x", "payload_raw_acceleration_y", "payload_raw_acceleration_z",
        "payload_velocity_x", "payload_velocity_y", "payload_velocity_z",
        "payload_acceleration_x", "payload_acceleration_y", "payload_acceleration_z",
        "payload_body_rate_x", "payload_body_rate_y", "payload_body_rate_z",
        "payload_filter_derivatives_valid",
        "payload_target_x", "payload_target_y", "payload_target_z",
        "payload_desired_force_x", "payload_desired_force_y", "payload_desired_force_z",
        "payload_desired_moment_x", "payload_desired_moment_y", "payload_desired_moment_z",
        "raw_desired_link_direction_x", "raw_desired_link_direction_y",
        "raw_desired_link_direction_z",
        "desired_tension_signed_n", "payload_yaw_enabled", "transport_blend",
        "transport_blend_dot",
        "transport_link_gain_scale", "ramp_position_hold_gain_scale",
        "transport_position_hold_gain_scale",
        "payload_landing_rope_preserving", "payload_landing_rope_distance_m",
        "desired_force_dot_x", "desired_force_dot_y", "desired_force_dot_z",
        "payload_ground_clearance_m", "payload_prelift_clearance_target_m",
        "payload_prelift_ready", "payload_prelift_active",
        "payload_prelift_stage", "payload_prelift_reference_rise_m",
        "payload_prelift_reference_lead_m", "payload_prelift_failure_reason",
        "target_velocity_x", "target_velocity_y", "target_velocity_z",
        "target_acceleration_x", "target_acceleration_y", "target_acceleration_z",
        "payload_state_age_s", "emergency_landing_reason",
        "prelift_vertical_integral_active", "prelift_vertical_integral_force_n",
        "transport_support_trim_n", "payload_minimum_vehicle_distance_m",
        "payload_predicted_vehicle_distance_m", "payload_safety_reason",
        "transport_max_tilt_rad",
        # Wall-clock diagnostics only; previous-cycle fields refer to the
        # completed callback named by their control-time column.
        "controller_tick_interval_s", "controller_compute_wall_s",
        "previous_vehicle_cycle_wall_s", "previous_vehicle_cycle_control_time_s",
        "fleet_tick_interval_s", "previous_fleet_cycle_wall_s",
        "previous_fleet_cycle_control_time_s", "published_command_stamp_ns",
        "bridge_latest_command_stamp_ns", "bridge_latest_command_seq",
        "bridge_latest_command_age_s", "bridge_latest_callback_wall_s",
        "bridge_latest_send_call_wall_s", "bridge_latest_raw_thrust",
        "bridge_latest_accepted", "bridge_latest_unlock_packet",
        "bridge_latest_receipt_age_s", "bridge_latest_matches_published_stamp",
        "flight_validation_enabled", "flight_validation_elapsed_s",
        "flight_validation_landing_started",
        "flight_validation_landing_stage",
        "payload_ground_reference_z_m", "payload_landing_duration_s",
        "payload_landing_touchdown_confirmed", "payload_landing_unloading",
        "payload_landing_planning_failure_count", "payload_landing_planning_retry_count",
        "payload_landing_planning_failure_reason", "payload_landing_planning_fallback",
        "transport_support_requested_trim_n", "transport_support_trim_limit_n",
        "transport_support_initialization_clipped", "effective_transport_link_gain_scale",
        "raw_link_force_x", "raw_link_force_y", "raw_link_force_z",
        "limited_link_force_x", "limited_link_force_y", "limited_link_force_z",
        "kinematic_perpendicular_force_x", "kinematic_perpendicular_force_y",
        "kinematic_perpendicular_force_z", "link_force_rate_limited",
        "link_force_magnitude_limited", "payload_control_dt_s",
        "link_length_nominal_m", "link_length_effective_m", "link_length_measured_m",
        "link_calibration_enabled", "link_calibration_applied", "link_calibration_status",
        "link_calibration_reason", "link_calibration_samples", "link_calibration_window_s",
        "link_calibration_spread_m",
        "payload_roll_rad", "payload_pitch_rad", "payload_yaw_rad",
        "payload_target_roll_rad", "payload_target_pitch_rad", "payload_target_yaw_rad",
        "payload_body_z_x", "payload_body_z_y", "payload_body_z_z",
        "payload_target_body_z_x", "payload_target_body_z_y", "payload_target_body_z_z",
        "payload_cone_half_angle_deg", "payload_target_cone_half_angle_deg", "payload_cone_axis_error_deg",
        "payload_cone_config_half_angle_deg", "payload_cone_period_s",
        "payload_target_body_rate_x", "payload_target_body_rate_y", "payload_target_body_rate_z",
        "payload_target_body_rate_dot_x", "payload_target_body_rate_dot_y", "payload_target_body_rate_dot_z",
        "position_hold_position_x", "position_hold_position_y", "position_hold_position_z",
        "position_hold_velocity_x", "position_hold_velocity_y", "position_hold_velocity_z",
        "position_hold_acceleration_x", "position_hold_acceleration_y", "position_hold_acceleration_z",
        "transport_position_hold_force_x", "transport_position_hold_force_y", "transport_position_hold_force_z",
        "payload_attitude_reference_progress",
        "payload_transport_force_mode",
        "payload_trajectory_mode", "payload_trajectory_phase", "payload_circle_progress",
        "payload_trajectory_progress",
        "payload_lobe_center_1_x", "payload_lobe_center_1_y", "payload_lobe_center_1_z",
        "payload_lobe_center_2_x", "payload_lobe_center_2_y", "payload_lobe_center_2_z",
        "payload_circle_center_x", "payload_circle_center_y", "payload_circle_center_z",
        "payload_circle_radius_m", "payload_circle_yaw_rad", "payload_circle_yaw_rate_radps",
        "payload_circle_yaw_acceleration_radps2",
        "payload_target_velocity_x", "payload_target_velocity_y", "payload_target_velocity_z",
        "payload_target_acceleration_x", "payload_target_acceleration_y", "payload_target_acceleration_z",
        "circle_formation_acceleration_feedforward_x", "circle_formation_acceleration_feedforward_y",
        "circle_formation_acceleration_feedforward_z",
        "kinematic_specific_acceleration_x", "kinematic_specific_acceleration_y", "kinematic_specific_acceleration_z",
        "landing_vertical_integral_active", "landing_vertical_integral_force_n",
        "transport_hold_integral_enabled", "transport_hold_integral_learning",
        "transport_hold_integral_freeze_reason",
        "transport_hold_integral_bias_x", "transport_hold_integral_bias_y", "transport_hold_integral_bias_z",
        "transport_hold_integral_error_x", "transport_hold_integral_error_y", "transport_hold_integral_error_z",
        "transport_hold_integral_force_x", "transport_hold_integral_force_y", "transport_hold_integral_force_z",
        "transport_hold_integral_force_dot_x", "transport_hold_integral_force_dot_y", "transport_hold_integral_force_dot_z",
        "transport_feedback_controller", "transport_full_pid_learning", "transport_full_pid_freeze_reason",
        "transport_full_pid_duplicate_tick", "transport_full_pid_integral_gate_m", "transport_full_pid_gate_open",
        "transport_full_pid_integral_limit_active", "transport_full_pid_feedback_limit_active",
        "transport_full_pid_force_budget_limited", "transport_full_pid_rate_budget_limited",
        "transport_full_pid_integral_state_x", "transport_full_pid_integral_state_y", "transport_full_pid_integral_state_z",
        "transport_full_pid_integral_state_delta_x", "transport_full_pid_integral_state_delta_y", "transport_full_pid_integral_state_delta_z",
        "transport_full_pid_integral_state_limit_x", "transport_full_pid_integral_state_limit_y", "transport_full_pid_integral_state_limit_z",
        "transport_full_pid_integral_gain_x", "transport_full_pid_integral_gain_y", "transport_full_pid_integral_gain_z",
        "transport_full_pid_force_x", "transport_full_pid_force_y", "transport_full_pid_force_z",
        "transport_full_pid_force_dot_x", "transport_full_pid_force_dot_y", "transport_full_pid_force_dot_z",
        "transport_full_pid_pd_force_x", "transport_full_pid_pd_force_y", "transport_full_pid_pd_force_z",
        "transport_full_pid_pd_force_dot_x", "transport_full_pid_pd_force_dot_y", "transport_full_pid_pd_force_dot_z",
        "transport_full_pid_feedback_force_x", "transport_full_pid_feedback_force_y", "transport_full_pid_feedback_force_z",
        "transport_full_pid_feedback_force_dot_x", "transport_full_pid_feedback_force_dot_y", "transport_full_pid_feedback_force_dot_z",
        "force_limit_active", "hold_feedback_limit_active", "body_rate_limit_active", "collective_limit_active",
        "independent_mode", "transport_mode",
        "transport_full_pid_integral_velocity_weight",
        "transport_attitude_trim_enabled", "transport_attitude_trim_learning",
        "transport_attitude_trim_dynamic_enabled", "transport_attitude_trim_feedback_healthy",
        "transport_attitude_trim_during_lift_enabled",
        "transport_attitude_trim_feedback_quality_reason", "transport_attitude_trim_body_rate_norm_rps",
        "transport_attitude_trim_pose_step_deg", "transport_attitude_trim_pose_step_limit_deg",
        "transport_attitude_trim_freeze_reason", "transport_attitude_trim_limit_active",
        "transport_attitude_trim_rate_limit_active", "transport_attitude_trim_acceleration_limit_active",
        "transport_attitude_trim_error_x", "transport_attitude_trim_error_y", "transport_attitude_trim_error_z",
        "transport_attitude_trim_integral_x", "transport_attitude_trim_integral_y", "transport_attitude_trim_integral_z",
        "transport_attitude_trim_position_x", "transport_attitude_trim_position_y", "transport_attitude_trim_position_z",
        "transport_attitude_trim_velocity_x", "transport_attitude_trim_velocity_y", "transport_attitude_trim_velocity_z",
        "transport_attitude_trim_acceleration_x", "transport_attitude_trim_acceleration_y", "transport_attitude_trim_acceleration_z",
        "transport_attitude_trim_bridge_active",
        "transport_attitude_trim_bridge_position_x", "transport_attitude_trim_bridge_position_y", "transport_attitude_trim_bridge_position_z",
        "transport_attitude_trim_bridge_velocity_x", "transport_attitude_trim_bridge_velocity_y", "transport_attitude_trim_bridge_velocity_z",
        "transport_attitude_trim_bridge_acceleration_x", "transport_attitude_trim_bridge_acceleration_y", "transport_attitude_trim_bridge_acceleration_z",
        "transport_attitude_trim_bridge_hold_position_x", "transport_attitude_trim_bridge_hold_position_y", "transport_attitude_trim_bridge_hold_position_z",
        "transport_attitude_trim_bridge_hold_velocity_x", "transport_attitude_trim_bridge_hold_velocity_y", "transport_attitude_trim_bridge_hold_velocity_z",
        "transport_attitude_trim_bridge_hold_acceleration_x", "transport_attitude_trim_bridge_hold_acceleration_y", "transport_attitude_trim_bridge_hold_acceleration_z",
    ]
    # These are appended so historical column offsets remain stable.  Ordinary
    # payload fields describe the command snapshot; source fields describe the
    # newest received frame even when that frame is rejected or commands held.
    FIELDS += ["payload_control_time_s", "payload_control_age_s", "payload_control_reused",
               "payload_control_sample_time_s", "payload_source_stamp_s",
               "payload_source_received_age_s",
               "payload_source_raw_position_x", "payload_source_raw_position_y",
               "payload_source_raw_position_z", "payload_source_raw_roll_rad",
               "payload_source_raw_pitch_rad", "payload_source_raw_yaw_rad"]
    FIELDS += [prefix + suffix
               for prefix in ("payload_observer_", "payload_source_observer_")
               for suffix in ("status", "updated", "quality_valid", "velocity_clipped",
                              "acceleration_clipped", "body_rate_clipped", "input_dt_s",
                              "update_dt_s", "sample_count", "reset_count", "rejected_count",
                              "consecutive_rejected_count", "position_innovation_x",
                              "position_innovation_y", "position_innovation_z",
                              "attitude_innovation_x", "attitude_innovation_y",
                              "attitude_innovation_z")]
    FIELDS += ["prelift_vertical_integral_learning", "payload_prelift_feedback_reused",
               "payload_prelift_feedback_age_s"]

    def __init__(self, directory, prefix):
        directory = os.path.expanduser(directory)
        os.makedirs(directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(directory, "%s_%s.csv" % (prefix, timestamp))
        self.file = open(self.path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self._writes_since_flush = 0
        self._lock = threading.Lock()

    def write(self, row):
        with self._lock:
            if self.file.closed:
                return
            self.writer.writerow(row)
            self._writes_since_flush += 1
            if self._writes_since_flush >= 25:
                self.file.flush()
                self._writes_since_flush = 0

    def flush(self):
        """Flush a task-level log after a synchronized multi-vehicle tick."""
        with self._lock:
            if not self.file.closed:
                self.file.flush()
                self._writes_since_flush = 0

    def close(self):
        with self._lock:
            if not self.file.closed:
                self.file.flush()
                self.file.close()
