"""Payload parameter loading and runtime state initialization."""

from collections import deque
from ctbr_parameters import GLOBAL_CTBR_PARAMETER_ROOT
from ctbr_parameters import PAYLOAD_PARAMETER_ROOT
from ctbr_parameters import TRAJECTORY_PARAMETER_ROOT
from nav_msgs.msg import Path
from slung_attitude_trim import SlungAttitudeReferenceTrim, SlungDynamicAttitudeReferenceTrim
from slung_circle_reference import CircleTrajectoryConfig
from slung_circle_reference import PayloadCircleReference
from slung_circle_reference import PayloadFigureEightReference
from slung_attitude_cone_reference import PayloadConeReference
from slung_landing import PayloadTouchdownDetector
from slung_link_calibration import SlungLinkCalibrator
from slung_load_controller import PayloadStateObserver
from slung_load_controller import SecondOrderVelocityFilter as PayloadVelocityFilter
from slung_load_controller import SlungLoadConfig
from slung_load_controller import SlungLoadController
from slung_load_controller import attachment_points_from_yaml
from slung_load_controller import link_lengths_vector
from slung_load_controller import takeup_target_distance
from slung_prelift_trajectory import SlungPreliftTrajectory
from slung_transport_guard import SlungSupportTrim
from slung_transport_guard import SlungTransportGuard
from slung_transport_integral import SlungDifferentialHoldIntegral
import math
import numpy as np
import rospy
import threading


class PayloadParametersMixin:
    """Validate and initialize load control, geometry and mission parameters."""

    def _initialize_payload_parameters(self):
        self.payload_lock = threading.Lock()
        self.latest_payload_state = None
        self.payload_accepted_pose = None
        self.payload_prelift_accepted_state = None
        self.payload_prelift_confirmation_waiting = False
        self.last_valid_payload_geometry = None
        self.payload_topic = str(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/payload_mocap_topic",
            "/load/mocap_state",
        ))
        self.slung_controller = None
        self.payload_height_m = None
        self.payload_hold_s = 0.20
        self.payload_emergency_land_after_s = 0.30
        self.payload_activation_hold_s = 1.0
        self.payload_tension_ramp_s = 2.0
        self.payload_reference_lift_s = 4.0
        self.payload_target_height_m = math.nan
        self.payload_circle_config = CircleTrajectoryConfig()
        self.payload_circle_reference = None
        self.payload_circle_state = None
        self.payload_circle_landing_yaw = None
        self.payload_circle_previous_correction = None
        self.payload_flight_validation_enabled = False
        self.payload_flight_validation_transport_s = 10.0
        # One-shot mission state: intentionally survives _reset_payload_handoff.
        self.payload_validation_handoff_time = None
        self.payload_validation_landing_started_time = None
        self.payload_validation_landing_stage = ""
        self.payload_validation_ground_since = None
        self.payload_controlled_exit_reason = ""
        self.payload_ground_reference_samples = deque(maxlen=100)
        self.payload_ground_center_z = math.nan
        self.payload_ground_clearance_reference_m = 0.0
        self.payload_landing_profile = None
        self.payload_landing_failure_count = 0
        self.payload_landing_failure_reason = ""
        self.payload_landing_failure_snapshot = None
        self.payload_landing_retry_since = None
        self.payload_landing_retry_count = 0
        self.payload_landing_retry_tick = None
        self.fleet_numerical_fault_reason = ""
        self.fleet_numerical_fault_since = None
        self.fleet_tick_completed_vehicles = set()
        self.fleet_tick_initial_commands = {}
        self.payload_last_reference = None
        self.payload_landing_gain_snapshot = None
        self.payload_landing_rope_directions = None
        self.payload_landing_rope_distances = None
        self.payload_payload_aware_emergency = False
        self.payload_landing_max_speed_mps = .12
        self.payload_landing_max_acceleration_mps2 = .15
        self.payload_landing_gain_transition_s = 2.0
        self.payload_landing_unload_s = 2.0
        self.payload_touchdown_detector = None
        self.payload_unload_start_time = None
        self.payload_landing_trim_at_unload = None
        self.payload_emergency_support = {}
        # Session-level effective geometry survives per-handoff resets.
        self.payload_nominal_link_lengths = np.full(3, np.nan)
        self.payload_link_calibrator = None
        self.payload_link_calibration_enabled = False
        self.payload_link_calibration_applied = False
        self.payload_link_calibration_pending = None
        self.payload_link_calibration_max_relative_speed_mps = .08
        self.payload_link_calibration_hold_reference = None
        self.payload_link_measured_distances = np.full(3, np.nan)
        self.payload_takeup_outward_offset_m = 0.12
        self.payload_takeup_pre_tension_slack_m = 0.05
        self.payload_takeup_distance_tolerance_m = 0.03
        self.payload_takeup_confirm_s = 0.30
        self.payload_takeup_position_tolerance_m = 0.03
        self.payload_takeup_link_angle_tolerance_rad = math.radians(10.0)
        self.payload_transport_link_gain_scale = 0.0
        self.payload_transport_steady_link_gain_scale = 1.0
        self.payload_ramp_position_hold_gain_scale = 2.0
        self.payload_transport_position_hold_gain_scale = 0.0
        self.payload_transport_feedback_controller = "differential_integral"
        self.payload_transport_pid_integral_gain_scale = 1.0
        self.payload_transport_pid_integral_limit_scale = 1.0
        self.payload_transport_pid_integral_gate_m = 0.20
        self.payload_transport_pid_integral_velocity_weight = 1.0
        self.payload_transport_pid_force_limit_n = 0.06
        self.payload_transport_pid_force_rate_limit_nps = 0.004
        self.payload_attitude_trim_enabled = False
        self.payload_attitude_trim_dynamic_enabled = False
        self.payload_attitude_trim_during_lift_enabled = False
        self.payload_attitude_trim_feedback_max_rate_rps = .5
        self.payload_attitude_trim_feedback_pose_jump_deg = 1.
        self.payload_attitude_trim_feedback_recovery_s = .5
        self._reset_dynamic_attitude_trim_feedback()
        self.payload_attitude_trim_parameters = {}
        self.payload_attitude_trim = None
        self.payload_attitude_trim_landing_bridge = None
        self.payload_yaw_enabled = True
        self.payload_velocity_limit_mps = np.ones(3)
        self.payload_velocity_jump_limit_mps = np.ones(3)
        self.payload_landing_approach_fraction = 0.55
        self.payload_velocity_filter = None
        self.payload_velocity_filter_lock = threading.Lock()
        self.payload_state_observer = None
        self.payload_state_observer_lock = threading.Lock()
        self.payload_fault_since = None
        self.last_transport_overrides = {}
        self.payload_transport_active = False
        self.payload_ready_since = None
        self.payload_transport_start_time = None
        self.payload_transport_start_position = None
        self.payload_transport_start_rotation = None
        self.payload_transport_goal_rotation = None
        self.payload_attitude_reference_enabled = False
        self.payload_attitude_reference_s = 12.0
        self.payload_equilibrium_reference_enabled = False
        self.payload_transport_initial_link_units = None
        self.payload_transport_initial_link_distances = None
        self.payload_transport_goal_link_units = None
        self.payload_transport_vehicle_start_positions = None
        self.payload_takeup_start_time = None
        self.payload_takeup_start_positions = None
        self.payload_takeup_target_positions = None
        self.payload_takeup_ready_since = None
        self.payload_prelift_ready_since = None
        self.payload_prelift_clearance_m = 0.10
        self.payload_prelift_parameters = {}
        self.payload_prelift_integral_parameters = {}
        self.payload_landing_integral_parameters = {}
        self.payload_prelift_reference = None
        self.payload_last_takeup_reference = None
        self.payload_geometry_ready_since = None
        self.payload_mission_hold_time = None
        self.payload_handoff_reference_positions = None
        self.payload_handoff_forces = None
        self.payload_transport_guard = SlungTransportGuard()
        self.payload_support_trim = SlungSupportTrim()
        self.payload_transport_integral = SlungDifferentialHoldIntegral()
        self.payload_transport_integral_enabled = False
        self.payload_transport_integral_last_time = None
        self.payload_safety_reason = ""
        self.payload_minimum_vehicle_distance_m = math.nan
        self.payload_predicted_vehicle_distance_m = math.nan
        self.payload_prelift_confirm_s = 0.50
        self.payload_prelift_vertical_velocity_tolerance_mps = 0.05
        self.payload_ground_clearance_m = math.nan
        self.payload_prelift_ready = False
        self.payload_takeup_taut_latched = False
        self.payload_takeup_diagnostic_interval_s = 1.0
        self.payload_takeup_diagnostic_last_time = None
        self.payload_landing_start_time = None
        self.payload_landing_start_position = None
        self.payload_path_publisher = rospy.Publisher(
            "/load/path", Path, queue_size=1, latch=True
        )
        self.payload_path_message = Path()
        self.payload_path_message.header.frame_id = str(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/path_frame_id", "world"
        ))
        self.payload_path_last_publish_time = 0.0
        self.payload_path_publish_interval_s = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/path_publish_interval_s", 0.1))
        self.payload_path_max_poses = int(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/path_max_poses", 1000))
        if (not math.isfinite(self.payload_path_publish_interval_s)
                or self.payload_path_publish_interval_s <= 0):
            raise rospy.ROSInitException("path_publish_interval_s 必须为正的有限数")
        if self.payload_path_max_poses <= 0:
            raise rospy.ROSInitException("path_max_poses 必须为正整数")
        if self.transport_mode == "slung_load":
            try:
                payload = rospy.get_param(PAYLOAD_PARAMETER_ROOT)
                size = np.asarray(payload["size_m"], dtype=float).reshape(3)
                attachment_points = attachment_points_from_yaml(
                    payload["attachment_points_m"]
                )
                attitude_bandwidth = np.asarray(
                    payload["attitude_bandwidth_hz"], dtype=float
                ).reshape(3)
                inertia_diag = np.array([
                    float(payload["mass_kg"]) * (size[1] ** 2 + size[2] ** 2) / 12.0,
                    float(payload["mass_kg"]) * (size[0] ** 2 + size[2] ** 2) / 12.0,
                    float(payload["mass_kg"]) * (size[0] ** 2 + size[1] ** 2) / 12.0,
                ])
                link_lengths = link_lengths_vector(
                    payload.get("link_lengths_m", payload.get("link_length_m")),
                    "link_lengths_m",
                )
                self.payload_nominal_link_lengths = link_lengths.copy()
                enabled = payload.get("link_calibration_enabled", False)
                if not isinstance(enabled, bool):
                    raise ValueError("link_calibration_enabled 必须为布尔值")
                self.payload_link_calibration_enabled = enabled
                try:
                    self.payload_link_calibrator = SlungLinkCalibrator(
                        link_lengths,
                        window_s=float(payload.get("link_calibration_window_s", 5.)),
                        max_adjustment_m=float(payload.get("link_calibration_max_adjustment_m", .03)),
                        max_spread_m=float(payload.get("link_calibration_max_spread_m", .006)),
                        max_drift_mps=float(payload.get("link_calibration_max_drift_mps", .003)),
                        max_sample_gap_s=float(payload.get("link_calibration_max_sample_gap_s", .10)),
                        min_samples=payload.get("link_calibration_min_samples", 100),
                    )
                    self.payload_link_calibration_max_relative_speed_mps = float(
                        payload.get("link_calibration_max_relative_speed_mps", .08))
                    if (not math.isfinite(self.payload_link_calibration_max_relative_speed_mps)
                            or self.payload_link_calibration_max_relative_speed_mps <= 0):
                        raise ValueError("max_relative_speed_mps 必须为正的有限数")
                except (TypeError, ValueError) as error:
                    raise ValueError("link_calibration 参数无效：%s" % error)
                load_cfg = SlungLoadConfig(
                    payload_mass_kg=float(payload["mass_kg"]),
                    gravity_mps2=float(rospy.get_param(
                        GLOBAL_CTBR_PARAMETER_ROOT + "/gravity_mps2", 9.80665
                    )),
                    payload_size_m=size,
                    attachment_points_m=attachment_points,
                    link_length_m=float(np.mean(link_lengths)),
                    link_lengths_m=link_lengths,
                    position_gain=np.asarray(payload["position_gain"], dtype=float),
                    velocity_gain=np.asarray(payload["velocity_gain"], dtype=float),
                    integral_gain=np.asarray(payload["integral_gain"], dtype=float),
                    integral_limit=np.asarray(payload["integral_limit"], dtype=float),
                    c1=float(payload["c1"]),
                    force_norm_epsilon=float(payload["force_norm_epsilon"]),
                    tension_pinv_tolerance=float(payload["tension_pinv_tolerance"]),
                    link_kq=float(payload["link_kq"]),
                    link_komega=float(payload["link_komega"]),
                    link_integral_gain=float(payload["link_integral_gain"]),
                    link_integral_limit=np.asarray(
                        payload["link_integral_limit"], dtype=float
                    ),
                    outward_bias_fraction=float(payload.get("outward_bias_fraction", 0.0)),
                    outward_bias_max_n=float(payload.get("outward_bias_max_n", 0.0)),
                    force_dot_limit_nps=float(payload.get("force_dot_limit_nps", 20.0)),
                    link_force_correction_limit_n=float(payload.get(
                        "link_force_correction_limit_n", 0.12
                    )),
                    link_force_correction_rate_limit_nps=float(payload.get(
                        "link_force_correction_rate_limit_nps", 5.0
                    )),
                    transport_force_mode=str(payload.get("transport_force_mode", "coupled")),
                )
                self.slung_controller = SlungLoadController(load_cfg)
                self.payload_height_m = float(size[2])
                self.payload_hold_s = float(payload.get("mocap_hold_s", 0.20))
                self.payload_emergency_land_after_s = float(
                    payload.get("emergency_land_after_s", 0.30)
                )
                payload_filter_cutoff_hz = float(
                    payload.get("velocity_filter_cutoff_hz", 5.0)
                )
                payload_filter_max_dt = float(
                    payload.get("velocity_filter_max_dt", 0.05)
                )
                self.payload_velocity_limit_mps = np.asarray(
                    payload.get("velocity_limit_mps", [1.0, 1.0, 1.0]),
                    dtype=float,
                ).reshape(-1)
                self.payload_velocity_jump_limit_mps = np.asarray(
                    payload.get("velocity_jump_limit_mps", [0.8, 0.8, 0.8]),
                    dtype=float,
                ).reshape(-1)
                if self.payload_velocity_limit_mps.size == 1:
                    self.payload_velocity_limit_mps = np.full(
                        3, float(self.payload_velocity_limit_mps[0])
                    )
                if self.payload_velocity_jump_limit_mps.size == 1:
                    self.payload_velocity_jump_limit_mps = np.full(
                        3, float(self.payload_velocity_jump_limit_mps[0])
                    )
                if (self.payload_velocity_limit_mps.size != 3 or
                        self.payload_velocity_jump_limit_mps.size != 3 or
                        not np.all(np.isfinite(self.payload_velocity_limit_mps)) or
                        not np.all(np.isfinite(self.payload_velocity_jump_limit_mps)) or
                        np.any(self.payload_velocity_limit_mps <= 0.0) or
                        np.any(self.payload_velocity_jump_limit_mps <= 0.0)):
                    raise ValueError(
                        "负载速度限幅和跳变限幅必须是三个正的有限数值"
                    )
                self.payload_velocity_filter = PayloadVelocityFilter(
                    cutoff_hz=payload_filter_cutoff_hz,
                    max_dt=payload_filter_max_dt,
                    max_speed_mps=self.payload_velocity_limit_mps,
                    max_jump_mps=self.payload_velocity_jump_limit_mps,
                )
                observer_enabled = payload.get("state_observer_enabled", True)
                if not isinstance(observer_enabled, bool):
                    raise ValueError("state_observer_enabled 必须为布尔值")
                if observer_enabled:
                    sampling = {}
                    for name, default in (
                            ("observer_min_dt_s", .005),
                            ("observer_nominal_dt_s", .01),
                            ("observer_attitude_innovation_margin_deg", 5.)):
                        value = payload.get(name, default)
                        if isinstance(value, (bool, np.bool_)):
                            raise ValueError(name + " 必须为正的有限数")
                        value = float(value)
                        if not math.isfinite(value) or value <= 0.0:
                            raise ValueError(name + " 必须为正的有限数")
                        sampling[name] = value
                    position_margin = payload.get("observer_position_innovation_margin_m", .02)
                    if np.asarray(position_margin).dtype.kind == "b":
                        raise ValueError("observer_position_innovation_margin_m 必须为正的有限数")
                    self.payload_state_observer = PayloadStateObserver(
                        position_gain=float(payload.get("observer_position_gain", 0.35)),
                        velocity_gain=float(payload.get("observer_velocity_gain", 0.08)),
                        acceleration_gain=float(payload.get("observer_acceleration_gain", 0.02)),
                        attitude_gain=float(payload.get("observer_attitude_gain", 0.35)),
                        angular_rate_gain=float(payload.get("observer_angular_rate_gain", 0.08)),
                        max_dt=float(payload.get("observer_max_dt", 0.05)),
                        max_velocity_mps=payload.get(
                            "observer_max_velocity_mps", [1.0, 1.0, 1.0]
                        ),
                        max_acceleration_mps2=payload.get(
                            "observer_max_acceleration_mps2", [8.0, 8.0, 8.0]
                        ),
                        max_body_rate_rps=payload.get(
                            "observer_max_body_rate_rps", [8.0, 8.0, 8.0]
                        ),
                        min_samples=int(payload.get("observer_min_samples", 3)),
                        min_dt_s=sampling["observer_min_dt_s"],
                        nominal_dt_s=sampling["observer_nominal_dt_s"],
                        position_innovation_margin_m=position_margin,
                        attitude_innovation_margin_rad=math.radians(
                            sampling["observer_attitude_innovation_margin_deg"]),
                    )
                    rospy.loginfo(
                        "负载状态观测器已启用：仅使用 NOKOV 位姿/姿态估计速度、加速度和角速度；"
                        "至少等待 %d 个样本。",
                        self.payload_state_observer.min_samples,
                    )
                self.payload_activation_hold_s = float(
                    payload.get("activation_hold_s", 1.0)
                )
                self.payload_tension_ramp_s = float(
                    payload.get("tension_ramp_s", 2.0)
                )
                self.payload_reference_lift_s = float(
                    payload.get("reference_lift_s", 4.0)
                )
                target_height = payload.get("target_payload_height_m", None)
                self.payload_target_height_m = (
                    math.nan if target_height is None else float(target_height)
                )
                self.payload_circle_config = CircleTrajectoryConfig.from_mapping(
                    payload.get("trajectory", {}))
                enabled = payload.get("flight_validation_enabled", False)
                if not isinstance(enabled, bool):
                    raise ValueError("flight_validation_enabled 必须为布尔值")
                self.payload_flight_validation_enabled = enabled
                self.payload_flight_validation_transport_s = float(
                    payload.get("flight_validation_transport_s", 10.0))
                if (not math.isfinite(self.payload_flight_validation_transport_s)
                        or self.payload_flight_validation_transport_s <= 0):
                    raise ValueError("flight_validation_transport_s 必须为正的有限数")
                self.payload_takeup_outward_offset_m = float(
                    payload.get("takeup_outward_offset_m", 0.12)
                )
                self.payload_takeup_pre_tension_slack_m = float(
                    payload.get("takeup_pre_tension_slack_m", 0.05)
                )
                self.payload_takeup_distance_tolerance_m = float(
                    payload.get("takeup_distance_tolerance_m", 0.03)
                )
                self.payload_takeup_confirm_s = float(
                    payload.get("takeup_confirm_s", 0.30)
                )
                self.payload_takeup_diagnostic_interval_s = float(
                    payload.get("takeup_diagnostic_interval_s", 1.0)
                )
                self.payload_takeup_position_tolerance_m = float(
                    payload.get("takeup_position_tolerance_m", 0.03)
                )
                self.payload_takeup_link_angle_tolerance_rad = math.radians(float(
                    payload.get("takeup_link_angle_tolerance_deg", 10.0)
                ))
                self.payload_prelift_clearance_m = float(
                    payload.get("prelift_clearance_m", 0.10)
                )
                self.payload_prelift_parameters = {
                    "clearance_m": self.payload_prelift_clearance_m,
                    "clearance_margin_m": float(payload.get("prelift_clearance_margin_m", 0.02)),
                    "speed_mps": float(payload.get("prelift_speed_mps", 0.05)),
                    "acceleration_mps2": float(payload.get("prelift_acceleration_mps2", 0.05)),
                    "max_rise_m": float(payload.get("prelift_max_reference_rise_m", 1.8)),
                    "max_lead_m": float(payload.get("prelift_max_reference_lead_m", 1.6)),
                    "timeout_s": float(payload.get("prelift_timeout_s", 60.0)),
                    "alignment_duration_s": float(payload.get("prelift_alignment_duration_s", 2.0)),
                    "lead_hold_s": float(payload.get("prelift_lead_hold_s", 10.0)),
                    "position_gain_scale": payload.get(
                        "prelift_independent_position_gain_scale", [3.0, 3.0, 1.0]
                    ),
                    "velocity_gain_scale": payload.get(
                        "prelift_independent_velocity_gain_scale", [1.5, 1.5, 1.0]
                    ),
                    "max_tilt_rad": math.radians(float(
                        payload.get("prelift_max_tilt_deg", 12.0)
                    )),
                }
                SlungPreliftTrajectory(**self.payload_prelift_parameters)
                self.payload_prelift_integral_parameters = {
                    "prelift_vertical_integral_limit": float(payload.get(
                        "prelift_vertical_integral_limit", 8.0)),
                    "prelift_vertical_integral_rate_limit": float(payload.get(
                        "prelift_vertical_integral_rate_limit", 0.5)),
                    "prelift_vertical_feedback_limit_mps2": float(payload.get(
                        "prelift_vertical_feedback_limit_mps2", 12.0)),
                    "prelift_vertical_integral_gate_m": (
                        self.payload_prelift_parameters["max_lead_m"] + 0.05),
                }
                if any(not math.isfinite(v) or v <= 0.0
                       for v in self.payload_prelift_integral_parameters.values()):
                    raise ValueError("预抬竖直积分/加速度限幅必须为正的有限数")
                self.payload_landing_integral_parameters = {
                    "landing_vertical_integral_limit": float(payload.get(
                        "landing_vertical_integral_limit", 2.0)),
                    "landing_vertical_integral_rate_limit": float(payload.get(
                        "landing_vertical_integral_rate_limit", 0.5)),
                    "landing_vertical_feedback_limit_mps2": float(payload.get(
                        "landing_vertical_feedback_limit_mps2", 8.0)),
                }
                if any(not math.isfinite(v) or v <= 0.0
                       for v in self.payload_landing_integral_parameters.values()):
                    raise ValueError("landing_vertical 积分/加速度限幅必须为正的有限数")
                self.payload_prelift_confirm_s = float(
                    payload.get("prelift_confirm_s", 0.50)
                )
                self.payload_prelift_vertical_velocity_tolerance_mps = float(
                    payload.get("prelift_vertical_velocity_tolerance_mps", 0.05)
                )
                self.payload_transport_link_gain_scale = float(
                    payload.get("transport_link_gain_scale", 0.0)
                )
                self.payload_transport_steady_link_gain_scale = float(
                    payload.get("transport_link_steady_gain_scale", 1.0)
                )
                self.payload_ramp_position_hold_gain_scale = float(
                    payload.get("ramp_position_hold_gain_scale", 2.0)
                )
                self.payload_transport_position_hold_gain_scale = float(
                    payload.get("transport_position_hold_gain_scale", 0.0)
                )
                self.payload_transport_feedback_controller = str(payload.get(
                    "transport_feedback_controller", "differential_integral"))
                if self.payload_transport_feedback_controller not in (
                        "differential_integral", "native_position_pid"):
                    raise ValueError(
                        "transport_feedback_controller 必须为 differential_integral 或 native_position_pid")
                if (self.payload_transport_feedback_controller == "native_position_pid"
                        and load_cfg.transport_force_mode != "reference_cascade"):
                    raise ValueError("native_position_pid 仅支持 reference_cascade")
                self.payload_transport_pid_integral_gain_scale = float(payload.get(
                    "transport_position_pid_integral_gain_scale", 1.0))
                self.payload_transport_pid_integral_limit_scale = float(payload.get(
                    "transport_position_pid_integral_limit_scale", 1.0))
                self.payload_transport_pid_integral_gate_m = float(payload.get(
                    "transport_position_pid_integral_gate_m", 0.20))
                integral_velocity_weight = payload.get(
                    "transport_position_pid_integral_velocity_weight", 1.0)
                if (isinstance(integral_velocity_weight, (bool, np.bool_))
                        or not isinstance(integral_velocity_weight, (int, float, np.integer, np.floating))
                        or not math.isfinite(float(integral_velocity_weight))
                        or integral_velocity_weight < 0.0):
                    raise ValueError("transport_position_pid_integral_velocity_weight 必须为非负的有限数")
                self.payload_transport_pid_integral_velocity_weight = float(integral_velocity_weight)
                self.payload_transport_pid_force_limit_n = float(payload.get(
                    "transport_position_pid_force_limit_n", 0.06))
                self.payload_transport_pid_force_rate_limit_nps = float(payload.get(
                    "transport_position_pid_force_rate_limit_nps", 0.004))
                if any(not math.isfinite(v) or v <= 0.0 for v in (
                        self.payload_transport_pid_integral_gain_scale,
                        self.payload_transport_pid_integral_limit_scale,
                        self.payload_transport_pid_integral_gate_m,
                        self.payload_transport_pid_force_limit_n,
                        self.payload_transport_pid_force_rate_limit_nps)):
                    raise ValueError("transport native position PID 参数必须为正的有限数")
                self.payload_attitude_reference_enabled = payload.get(
                    "transport_attitude_reference_enabled", False)
                self.payload_equilibrium_reference_enabled = payload.get(
                    "transport_equilibrium_reference_enabled", False)
                self.payload_attitude_reference_s = float(payload.get(
                    "transport_attitude_reference_s", self.payload_reference_lift_s))
                if (not isinstance(self.payload_attitude_reference_enabled, bool)
                        or not isinstance(self.payload_equilibrium_reference_enabled, bool)
                        or not math.isfinite(self.payload_attitude_reference_s)
                        or self.payload_attitude_reference_s <= 0.0):
                    raise ValueError("transport_attitude_reference_enabled/s 必须为布尔值/正有限秒数")
                if (load_cfg.transport_force_mode == "reference_cascade"
                        and (not self.payload_attitude_reference_enabled
                             or not self.payload_equilibrium_reference_enabled
                             or self.payload_ramp_position_hold_gain_scale <= 0.0
                             or self.payload_transport_position_hold_gain_scale <= 0.0)):
                    raise ValueError("reference_cascade 需要启用姿态/平衡构型参考以及正的位置保持增益")
                if self.payload_circle_config.mode in ("circle", "figure_eight", "attitude_cone"):
                    mode = self.payload_circle_config.mode
                    if load_cfg.transport_force_mode != "reference_cascade":
                        raise ValueError("payload %s 轨迹需要 reference_cascade" % mode)
                    if str(rospy.get_param(TRAJECTORY_PARAMETER_ROOT + "/trajectory_mode", "circle")) != "hover":
                        raise ValueError("payload %s 轨迹需要 ctbr_trajectory.trajectory_mode=hover" % mode)
                    reference_type = {"circle":PayloadCircleReference,
                                      "figure_eight":PayloadFigureEightReference,
                                      "attitude_cone":PayloadConeReference}[mode]
                    motion = reference_type(np.zeros(3), 0., self.payload_circle_config)
                    reference_end = max(self.payload_attitude_reference_s,
                                        self.payload_tension_ramp_s + self.payload_reference_lift_s)
                    required = reference_end + self.payload_circle_config.settle_s + motion.duration
                    if (self.payload_flight_validation_enabled
                            and self.payload_flight_validation_transport_s < required):
                        raise ValueError("payload %s 轨迹尚未完成就会触发验证降落；需要至少 %.3f s" % (mode,required))
                    if float(rospy.get_param(TRAJECTORY_PARAMETER_ROOT + "/hover_duration_s", 30.)) < required:
                        raise ValueError("payload %s 轨迹需要 ctbr_trajectory.hover_duration_s 至少 %.3f s" % (mode,required))
                self.payload_attitude_trim_enabled = payload.get("transport_attitude_trim_enabled", False)
                if not isinstance(self.payload_attitude_trim_enabled, bool):
                    raise ValueError("transport_attitude_trim_enabled 必须为布尔值")
                self.payload_attitude_trim_dynamic_enabled = payload.get(
                    "transport_attitude_trim_dynamic_enabled", False)
                if not isinstance(self.payload_attitude_trim_dynamic_enabled, bool):
                    raise ValueError("transport_attitude_trim_dynamic_enabled 必须为布尔值")
                self.payload_attitude_trim_during_lift_enabled = payload.get(
                    "transport_attitude_trim_during_lift_enabled", False)
                if not isinstance(self.payload_attitude_trim_during_lift_enabled, bool):
                    raise ValueError("transport_attitude_trim_during_lift_enabled 必须为布尔值")
                for name, default in (("max_rate_rps", .5), ("pose_jump_deg", 1.), ("recovery_s", .5)):
                    key = "transport_attitude_trim_feedback_" + name
                    value = payload.get(key, default)
                    if isinstance(value, bool):
                        raise ValueError(key + " 必须为正有限数")
                    value = float(value)
                    if not math.isfinite(value) or value <= 0.:
                        raise ValueError(key + " 必须为正有限数")
                    setattr(self, "payload_attitude_trim_feedback_" + name, value)
                if (self.payload_attitude_trim_enabled
                        and load_cfg.transport_force_mode != "reference_cascade"):
                    raise ValueError("transport_attitude_trim 仅支持 reference_cascade")
                self.payload_attitude_trim_parameters = dict(
                    gain_per_s=payload.get("transport_attitude_trim_gain_per_s", [.05, .05, .02]),
                    max_offset_m=payload.get("transport_attitude_trim_limit_m", .01),
                    max_rate_mps=payload.get("transport_attitude_trim_rate_limit_mps", .001),
                    max_acceleration_mps2=payload.get("transport_attitude_trim_acceleration_limit_mps2", .001),
                    adaptation_interval_s=payload.get("transport_attitude_trim_interval_s", 1.),
                )
                try:
                    trim_type = (SlungDynamicAttitudeReferenceTrim
                                 if self.payload_attitude_trim_dynamic_enabled else SlungAttitudeReferenceTrim)
                    self.payload_attitude_trim = trim_type(
                        load_cfg.attachment_points_m, enabled=self.payload_attitude_trim_enabled,
                        **self.payload_attitude_trim_parameters)
                except (TypeError, ValueError) as error:
                    raise ValueError("transport_attitude_trim 参数无效：%s" % error) from error
                self.payload_transport_guard = SlungTransportGuard(
                    min_distance_m=float(payload.get("minimum_vehicle_distance_m", 0.30)),
                    prediction_s=float(payload.get("separation_prediction_s", 0.35)),
                    height_error_m=float(payload.get("transport_height_error_limit_m", 0.08)),
                    confirm_s=float(payload.get("transport_fault_confirm_s", 0.20)),
                )
                self.payload_support_trim = SlungSupportTrim(
                    limit_n=float(payload.get("transport_support_trim_limit_n", 0.25)),
                    integral_gain=float(payload.get("transport_support_integral_gain_npm_s", 0.20)),
                    rate_limit_nps=float(payload.get("transport_support_trim_rate_nps", 0.03)),
                )
                self.payload_transport_integral_enabled = payload.get("transport_hold_integral_enabled", False)
                if not isinstance(self.payload_transport_integral_enabled, bool):
                    raise ValueError("transport_hold_integral_enabled must be boolean")
                try:
                    self.payload_transport_integral = SlungDifferentialHoldIntegral(
                        gain_npm_s=payload.get("transport_hold_integral_gain_npm_s", .02),
                        limit_n=payload.get("transport_hold_integral_force_limit_n", .03),
                        rate_limit_nps=payload.get("transport_hold_integral_rate_limit_nps", .002),
                    )
                except ValueError as error:
                    raise ValueError("transport_hold_integral: %s" % error) from error
                self.payload_landing_max_speed_mps = float(payload.get("landing_payload_max_speed_mps", .12))
                self.payload_landing_max_acceleration_mps2 = float(payload.get("landing_payload_max_acceleration_mps2", .15))
                self.payload_landing_gain_transition_s = float(payload.get("landing_gain_transition_s", 2.0))
                self.payload_landing_unload_s = float(payload.get("landing_unload_s", 2.0))
                if any(not math.isfinite(v) or v <= 0 for v in (
                        self.payload_landing_max_speed_mps, self.payload_landing_max_acceleration_mps2,
                        self.payload_landing_gain_transition_s, self.payload_landing_unload_s)):
                    raise ValueError("负载降落速度/加速度/交接时间必须为正的有限数")
                self.payload_touchdown_parameters = dict(
                    clearance_tolerance_m=float(payload.get("landing_touchdown_tolerance_m", .015)),
                    contact_margin_m=float(payload.get("landing_touchdown_contact_margin_m", .006)),
                    speed_limit_mps=float(payload.get("landing_touchdown_speed_mps", .05)),
                    confirm_s=float(payload.get("landing_touchdown_confirm_s", .30)),
                    window_s=float(payload.get("landing_touchdown_window_s", .40)),
                    max_height_span_m=float(payload.get("landing_touchdown_height_span_m", .012)),
                    timeout_s=float(payload.get("landing_touchdown_timeout_s", 4.0)),
                )
                self.payload_touchdown_detector = PayloadTouchdownDetector(**self.payload_touchdown_parameters)
                self.payload_yaw_enabled = payload.get("yaw_channel_enabled", True)
                if not isinstance(self.payload_yaw_enabled, bool):
                    raise ValueError("yaw_channel_enabled 必须为布尔值")
                if self.payload_circle_config.mode in ("circle", "figure_eight", "attitude_cone") and not self.payload_yaw_enabled:
                    raise ValueError("payload %s 轨迹航向控制需要 yaw_channel_enabled=true" % self.payload_circle_config.mode)
                self.payload_landing_approach_fraction = float(
                    payload.get("landing_approach_fraction", 0.55)
                )
                if self.payload_hold_s < 0.0 or self.payload_emergency_land_after_s < self.payload_hold_s:
                    raise ValueError(
                        "mocap_hold_s 和 emergency_land_after_s 时间参数无效"
                    )
                if (self.payload_activation_hold_s < 0.0 or
                        self.payload_tension_ramp_s <= 0.0 or
                        self.payload_reference_lift_s <= 0.0):
                    raise ValueError("负载接管/张力/抬升时间参数无效")
                if (math.isfinite(self.payload_target_height_m)
                        and self.payload_target_height_m <= 0.0):
                    raise ValueError("target_payload_height_m 必须为正的有限数")
                if (self.payload_takeup_outward_offset_m < 0.0 or
                        self.payload_takeup_outward_offset_m >= float(np.min(load_cfg.link_lengths_m)) or
                        self.payload_takeup_pre_tension_slack_m < 0.0 or
                        self.payload_takeup_pre_tension_slack_m >= float(np.min(load_cfg.link_lengths_m)) or
                        self.payload_takeup_distance_tolerance_m <= 0.0 or
                        self.payload_takeup_confirm_s < 0.0 or
                        not math.isfinite(self.payload_takeup_diagnostic_interval_s) or
                        self.payload_takeup_diagnostic_interval_s <= 0.0 or
                        self.payload_takeup_position_tolerance_m <= 0.0 or
                        not math.isfinite(self.payload_takeup_link_angle_tolerance_rad) or
                        self.payload_takeup_link_angle_tolerance_rad <= 0.0 or
                        not math.isfinite(self.payload_prelift_clearance_m) or
                        self.payload_prelift_clearance_m < 0.0 or
                        not math.isfinite(self.payload_prelift_confirm_s) or
                        self.payload_prelift_confirm_s < 0.0 or
                        not math.isfinite(
                            self.payload_prelift_vertical_velocity_tolerance_mps
                        ) or
                        self.payload_prelift_vertical_velocity_tolerance_mps <= 0.0 or
                        not math.isfinite(self.payload_transport_link_gain_scale) or
                        not 0.0 <= self.payload_transport_link_gain_scale <= 1.0 or
                        not math.isfinite(self.payload_transport_steady_link_gain_scale) or
                        not 0.0 <= self.payload_transport_steady_link_gain_scale <= 1.0 or
                        not math.isfinite(self.payload_ramp_position_hold_gain_scale) or
                        self.payload_ramp_position_hold_gain_scale < 0.0 or
                        not math.isfinite(self.payload_transport_position_hold_gain_scale) or
                        self.payload_transport_position_hold_gain_scale < 0.0 or
                        not 0.0 < self.payload_landing_approach_fraction <= 1.0):
                    raise ValueError("负载 TAKEUP 几何参数无效")
                self.payload_takeup_target_distance_m = takeup_target_distance(
                    load_cfg.link_lengths_m, self.payload_takeup_pre_tension_slack_m
                )
                self.payload_attitude_gain = inertia_diag * (
                    2.0 * math.pi * attitude_bandwidth
                ) ** 2
                self.payload_rate_gain = (
                    2.0 * float(payload["attitude_damping_ratio"])
                    * 2.0 * math.pi * attitude_bandwidth * inertia_diag
                )
            except (KeyError, TypeError, ValueError) as error:
                self.logger.close()
                raise rospy.ROSInitException("slung_payload 参数无效：%s" % error)
