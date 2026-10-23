"""Single-aircraft ROS node and command scheduling."""

from collections import deque
from crazyswarm.msg import CTBR
from crazyswarm.msg import CTBRTiming
from crazyswarm.msg import GenericLogData
from crazyswarm.msg import MocapState
from ctbr_control_math import as_vector
from ctbr_control_math import phase_requires_full_controller_reset
from ctbr_control_math import rotation_to_rpy
from ctbr_csv_logger import FlightCsvLogger
from ctbr_geometric_controller import ControllerConfig
from ctbr_geometric_controller import GeometricCtbrController
from ctbr_parameters import CtbrParameterResolver
from ctbr_parameters import PAYLOAD_PARAMETER_ROOT
from ctbr_state_feedback import SecondOrderVelocityFilter
from ctbr_trajectory import CircularFlightTrajectory
from ctbr_trajectory import CircularTrajectoryConfig
from ctbr_vehicle_logging import VehicleLoggingMixin
from ctbr_vehicle_preflight import VehiclePreflightMixin
from ctbr_vehicle_state import VehicleStateMixin
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from slung_load_controller import link_lengths_vector
from std_msgs.msg import UInt16
from vehicle_config import select_vehicle_entry
from vehicle_config import vehicle_uri
import math
import numpy as np
import rospy
import threading
import time


class CtbrControllerNode(VehicleLoggingMixin, VehicleStateMixin, VehiclePreflightMixin):
    """ROS I/O and mission timing for one aircraft; control laws are pure modules."""

    def __init__(self, vehicle_config=None, logger=None, auto_timer=True,
                 register_shutdown=True, start_time=None):
        # crazyflies.yaml 是车辆身份的唯一来源。多机管理器会显式传入每架配置；
        # 直接实例化时保留旧的 ~cf_id/唯一 ctbr_enabled 选择行为。
        if vehicle_config is None:
            vehicle_entries = rospy.get_param("/crazyflies", [])
            requested_id = rospy.get_param("~cf_id", None)
            try:
                vehicle_config = select_vehicle_entry(
                    vehicle_entries, cf_id=requested_id
                )
            except (KeyError, TypeError, ValueError) as error:
                raise rospy.ROSInitException("飞机参数无效：%s" % error)
        try:
            self.vehicle_config = dict(vehicle_config)
            self.vehicle_id = int(self.vehicle_config["id"])
            self.radio_uri = vehicle_uri(self.vehicle_config)
            self.orbit_phase_rad = float(self.vehicle_config.get("orbit_phase_rad", 0.0))
            self.params = CtbrParameterResolver(
                rospy.get_param, rospy.has_param, self.vehicle_id
            )
        except (KeyError, TypeError, ValueError) as error:
            raise rospy.ROSInitException("飞机参数无效：%s" % error)
        self.auto_timer = bool(auto_timer)
        self.register_shutdown = bool(register_shutdown)
        self.logger = logger
        self.fleet_gate_open = bool(auto_timer)
        self.global_abort_reason = ""
        self.shared_start_time = None if start_time is None else float(start_time)
        self.mission_start_time = self.shared_start_time

        # 保留 ~cf_prefix 作为旧 launch 的兼容覆盖；新 launch 不再需要它。
        configured_prefix = str(rospy.get_param("~cf_prefix", "")).strip()
        self.prefix = (
            configured_prefix.rstrip("/")
            if configured_prefix
            else "/cf%d" % self.vehicle_id
        )
        if not bool(self.params.global_param("target_confirmed")):
            raise rospy.ROSInitException(
                "拒绝启动控制输出：请设置 ~target_confirmed:=true"
            )

        # 状态时效性与工作空间检查在控制发布之前执行；max_mocap_velocity_mps 同时
        # 限制 Nokov 差分尖峰进入速度反馈项。
        self.rate_hz = float(self.params.global_param("control_rate_hz"))
        self.state_timeout = float(self.params.global_param("state_timeout"))
        # These multipliers are consumed by this per-vehicle timer callback,
        # so they must be resolved on every CtbrControllerNode.  The fleet
        # manager also validates the same global values for early startup
        # feedback, but it is not the owner of a vehicle's callback state.
        self.takeoff_position_gain_scale = float(self.params.global_param(
            "takeoff_position_gain_scale", 1.0
        ))
        self.takeoff_velocity_gain_scale = float(self.params.global_param(
            "takeoff_velocity_gain_scale", 1.0
        ))
        if (not math.isfinite(self.takeoff_position_gain_scale) or
                self.takeoff_position_gain_scale < 1.0 or
                not math.isfinite(self.takeoff_velocity_gain_scale) or
                self.takeoff_velocity_gain_scale < 1.0):
            raise rospy.ROSInitException("起飞阶段增益倍率必须是不小于 1 的有限数")
        # ROS 的 mocap 回调和控制定时器在不同线程运行。多机模式中，若在整个
        # 车队周期开始时读取一次 ROS 时间，后续刚收到的一帧 header.stamp 可能比该
        # 旧时间晚几毫秒；这不是时间回退，不能据此切断推力。只容忍这种很短的
        # 调度差，超过该值仍按异常时间戳保护。
        self.mocap_timestamp_future_tolerance_s = float(self.params.global_param(
            "mocap_timestamp_future_tolerance_s", 0.02
        ))
        self.max_abs_position_m = float(self.params.global_param("max_abs_position_m"))
        self.max_mocap_velocity_mps = float(
            self.params.global_param("max_mocap_velocity_mps")
        )
        self.max_mocap_acceleration_mps2 = float(
            self.params.global_param("max_mocap_acceleration_mps2", 5.0)
        )
        if (self.rate_hz <= 0.0 or self.state_timeout <= 0.0 or
                not math.isfinite(self.mocap_timestamp_future_tolerance_s) or
                self.mocap_timestamp_future_tolerance_s < 0.0 or
                self.mocap_timestamp_future_tolerance_s > self.state_timeout or
                self.max_abs_position_m <= 0.0 or self.max_mocap_velocity_mps <= 0.0 or
                not math.isfinite(self.max_mocap_acceleration_mps2) or
                self.max_mocap_acceleration_mps2 <= 0.0):
            raise rospy.ROSInitException(
                "控制频率、动捕超时/时间戳容差、动捕边界、速度和加速度上限必须有效"
            )
        try:
            filter_cutoff_hz = float(self.params.global_param(
                "mocap_velocity_filter_cutoff_hz", 5.0
            ))
            filter_max_dt = float(self.params.global_param(
                "mocap_velocity_filter_max_dt", 0.05
            ))
            if filter_max_dt > self.state_timeout:
                raise ValueError(
                    "mocap_velocity_filter_max_dt 不能大于 state_timeout"
                )
            self.velocity_filter = SecondOrderVelocityFilter(
                cutoff_hz=filter_cutoff_hz,
                max_dt=filter_max_dt,
            )
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException("速度低通滤波参数无效：%s" % error)
        self.velocity_filter_lock = threading.Lock()

        # EKF 日志提供的平移状态独立滤波；NOKOV 仍是位置和姿态的唯一来源。
        self.ekf_kinematics_weight = float(self.params.global_param(
            "ekf_kinematics_weight", 0.0
        ))
        self.ekf_state_timeout = float(self.params.global_param(
            "ekf_state_timeout", 0.15
        ))
        self.ekf_max_position_delta_m = float(self.params.global_param(
            "ekf_max_position_delta_m", 0.30
        ))
        self.ekf_alignment_position_delta_m = float(self.params.global_param(
            "ekf_alignment_position_delta_m", 0.05
        ))
        self.ekf_alignment_hold_s = float(self.params.global_param(
            "ekf_alignment_hold_s", 1.0
        ))
        self.ekf_alignment_timeout_s = float(self.params.global_param(
            "ekf_alignment_timeout_s", 5.0
        ))
        self.ekf_kinematics_hold_s = float(self.params.global_param(
            "ekf_kinematics_hold_s", 0.10
        ))
        self.ekf_fault_emergency_land_s = float(self.params.global_param(
            "ekf_fault_emergency_land_s", 0.35
        ))
        self.ekf_timestamp_future_tolerance_s = float(self.params.global_param(
            "ekf_timestamp_future_tolerance_s", 0.02
        ))
        try:
            ekf_filter_cutoff_hz = float(self.params.global_param(
                "ekf_velocity_filter_cutoff_hz", filter_cutoff_hz
            ))
            ekf_filter_max_dt = float(self.params.global_param(
                "ekf_velocity_filter_max_dt", filter_max_dt
            ))
            if (not math.isfinite(self.ekf_kinematics_weight) or
                    not 0.0 <= self.ekf_kinematics_weight <= 1.0 or
                    not math.isfinite(self.ekf_state_timeout) or
                    self.ekf_state_timeout <= 0.0 or
                    self.ekf_state_timeout > self.state_timeout or
                    not math.isfinite(self.ekf_max_position_delta_m) or
                    self.ekf_max_position_delta_m <= 0.0 or
                    not math.isfinite(self.ekf_alignment_position_delta_m) or
                    self.ekf_alignment_position_delta_m <= 0.0 or
                    self.ekf_alignment_position_delta_m > self.ekf_max_position_delta_m or
                    not math.isfinite(self.ekf_alignment_hold_s) or
                    self.ekf_alignment_hold_s < 0.0 or
                    not math.isfinite(self.ekf_alignment_timeout_s) or
                    self.ekf_alignment_timeout_s <= 0.0 or
                    self.ekf_alignment_timeout_s < self.ekf_alignment_hold_s or
                    not math.isfinite(self.ekf_kinematics_hold_s) or
                    self.ekf_kinematics_hold_s < 0.0 or
                    not math.isfinite(self.ekf_fault_emergency_land_s) or
                    self.ekf_fault_emergency_land_s < self.ekf_kinematics_hold_s or
                    not math.isfinite(self.ekf_timestamp_future_tolerance_s) or
                    self.ekf_timestamp_future_tolerance_s < 0.0 or
                    self.ekf_timestamp_future_tolerance_s > self.ekf_state_timeout or
                    ekf_filter_max_dt > self.ekf_state_timeout):
                raise ValueError(
                    "EKF 权重、对齐/故障时间、位置差和滤波时间参数无效"
                )
            self.ekf_velocity_filter = SecondOrderVelocityFilter(
                cutoff_hz=ekf_filter_cutoff_hz,
                max_dt=ekf_filter_max_dt,
            )
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException("EKF 运动学参数无效：%s" % error)
        self.ekf_velocity_filter_lock = threading.Lock()
        # 短暂丢帧时暂停参考计时；超过此时长后不尝试在未知位置继续任务，而是锁定
        # aborted 并持续发送零 CTBR。该阈值必须大于 state_timeout，才允许一次短暂
        # 网络/动捕抖动恢复。
        self.trajectory_pause_abort_s = float(
            self.params.global_param("trajectory_pause_abort_s")
        )
        if self.trajectory_pause_abort_s <= self.state_timeout:
            raise rospy.ROSInitException(
                "trajectory_pause_abort_s 必须大于 state_timeout"
            )
        mass_kg = float(self.params.vehicle_param("mass_kg"))
        rospy.loginfo(
            "CTBR 参数：mass=%.4f kg，state_timeout=%.3f s，轨迹丢帧中止阈值=%.3f s",
            mass_kg,
            self.state_timeout,
            self.trajectory_pause_abort_s,
        )

        try:
            trajectory_param = self.params.trajectory_param
            vehicle_param = self.params.vehicle_param
            offset = np.array([
                float(trajectory_param("circle_center_offset_x")),
                float(trajectory_param("circle_center_offset_y")),
            ], dtype=float)
            if not np.all(np.isfinite(offset)):
                raise ValueError("circle_center_offset_x/y 必须是有限数值")
            configured_center = trajectory_param("circle_center_xy", None)
            circle_center_xy = None
            if configured_center is not None:
                circle_center_xy = np.asarray(configured_center, dtype=float).reshape(-1)
                if circle_center_xy.size != 2 or not np.all(np.isfinite(circle_center_xy)):
                    raise ValueError("circle_center_xy 必须是 2 个有限数值")
            takeoff_height_m = float(trajectory_param("takeoff_height_m"))
            transport_mode = str(self.params.global_param(
                "transport_mode", "formation"
            )).strip().lower().replace("-", "_")
            independent_takeoff_height_m = None
            independent_takeoff_duration_s = None
            takeup_duration_s = None
            if transport_mode == "slung_load":
                payload_param = rospy.get_param(PAYLOAD_PARAMETER_ROOT, {})
                # The trajectory position belongs to the aircraft.  For the
                # MATLAB cable geometry, the aircraft must be one link length
                # above the desired payload center before transport starts.
                configured_lengths = payload_param.get(
                    "link_lengths_m", payload_param.get("link_length_m")
                )
                if configured_lengths is not None:
                    takeoff_height_m += float(np.max(link_lengths_vector(
                        configured_lengths, "link_lengths_m"
                    )))
                independent_takeoff_height_m = float(
                    payload_param.get("independent_hover_height_m", 0.30)
                )
                independent_takeoff_duration_s = float(
                    payload_param.get("independent_takeoff_duration_s", 2.5)
                )
                takeup_duration_s = float(
                    payload_param.get("takeup_duration_s", 2.5)
                )
            self.trajectory_config = CircularTrajectoryConfig(
                reference_hold_s=float(trajectory_param("reference_hold_s")),
                takeoff_height_m=takeoff_height_m,
                takeoff_duration_s=float(trajectory_param("takeoff_duration_s")),
                takeoff_settle_s=float(trajectory_param("takeoff_settle_s")),
                takeoff_altitude_tolerance_m=float(
                    trajectory_param("takeoff_altitude_tolerance_m")
                ),
                takeoff_vertical_velocity_tolerance_mps=float(
                    trajectory_param("takeoff_vertical_velocity_tolerance_mps")
                ),
                circle_center_offset_xy=offset,
                circle_radius_m=float(trajectory_param("circle_radius_m")),
                circle_revolutions=float(trajectory_param("circle_revolutions")),
                # 该参数现在表示整圈五次角度轨迹的峰值角速度。
                circle_angular_speed_radps=float(
                    trajectory_param("circle_angular_speed_radps")
                ),
                circle_start_angle_rad=float(
                    trajectory_param("circle_start_angle_rad")
                ),
                entry_duration_s=float(trajectory_param("circle_entry_duration_s")),
                # 旧参数保留用于兼容已有 YAML，但整圈 smoothstep 不使用它。
                circle_ramp_duration_s=float(
                    trajectory_param("circle_ramp_duration_s")
                ),
                hover_duration_s=float(trajectory_param("hover_duration_s", 30.0)),
                independent_takeoff_height_m=independent_takeoff_height_m,
                independent_takeoff_duration_s=independent_takeoff_duration_s,
                takeup_duration_s=takeup_duration_s,
                final_hover_s=float(trajectory_param("final_hover_s")),
                landing_duration_s=float(trajectory_param("landing_duration_s")),
                landing_max_speed_mps=float(
                    trajectory_param("landing_max_speed_mps")
                ),
                landing_altitude_tolerance_m=float(
                    trajectory_param("landing_altitude_tolerance_m")
                ),
                landing_vertical_velocity_tolerance_mps=float(
                    trajectory_param("landing_vertical_velocity_tolerance_mps")
                ),
                landing_settle_s=float(trajectory_param("landing_settle_s")),
                landing_condition_grace_s=float(
                    trajectory_param("landing_condition_grace_s")
                ),
                takeoff_max_tilt_rad=math.radians(float(
                    trajectory_param("takeoff_max_tilt_deg")
                )),
                circle_max_tilt_rad=math.radians(float(
                    trajectory_param("circle_max_tilt_deg")
                )),
                landing_max_tilt_rad=math.radians(float(
                    trajectory_param("landing_max_tilt_deg")
                )),
                takeoff_min_collective_thrust=float(
                    vehicle_param("takeoff_min_thrust_newton")
                ),
                airborne_min_collective_thrust=float(
                    vehicle_param("airborne_min_thrust_newton")
                ),
                circle_center_xy=circle_center_xy,
                orbit_phase_rad=self.orbit_phase_rad,
                trajectory_mode=str(trajectory_param("trajectory_mode", "circle")),
                formation_side_length_m=float(
                    trajectory_param("formation_side_length_m", 0.5)
                ),
                figure_eight_radius_m=float(
                    trajectory_param("figure_eight_radius_m", 0.8)
                ),
                figure_eight_angular_speed_radps=float(
                    trajectory_param("figure_eight_angular_speed_radps", 0.55)
                ),
            )
            # 这里先做一次无起点构造，尽早报告参数错误；实际圆心/编队质心到第一帧后才解析。
            CircularFlightTrajectory(self.trajectory_config)
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException("任务轨迹参数无效：%s" % error)

        # 预起飞电压补偿不是飞行中的闭环：它只用零推力阶段采集到的一段电压中位数
        # 计算一次 raw PWM 缩放值。飞行中电压因负载和 ADC 噪声快速变化，逐包补偿会
        # 直接制造推力抖动，因此明确禁止那种做法。
        self.require_preflight_voltage = bool(
            self.params.global_param("require_preflight_voltage")
        )
        self.preflight_voltage_samples_required = int(
            self.params.global_param("preflight_voltage_samples")
        )
        self.preflight_voltage_timeout_s = float(
            self.params.global_param("preflight_voltage_timeout_s")
        )
        self.preflight_battery_max_age_s = float(
            self.params.global_param("preflight_battery_max_age_s")
        )
        self.preflight_min_voltage_v = float(
            self.params.global_param("preflight_min_voltage_v")
        )
        # 这个参考电压必须与 max_total_thrust_newton 的静态标定电压一致。4.20 V
        # 目前只是单节满电的暂定值，后续 F(raw, V) 标定可直接替换它。
        self.preflight_voltage_reference_v = float(
            self.params.vehicle_param("preflight_voltage_reference_v")
        )
        # 对理想关系 F ∝ (raw * V)^2，raw 缩放为 V_ref / V_preflight，即指数为 1。
        # 保留独立参数是为了后续实验标定电压-推力幂次，而不修改接口。
        self.preflight_raw_scale_exponent = float(
            self.params.vehicle_param("preflight_raw_scale_exponent")
        )
        self.preflight_raw_scale_min = float(
            self.params.vehicle_param("preflight_raw_scale_min")
        )
        self.preflight_raw_scale_max = float(
            self.params.vehicle_param("preflight_raw_scale_max")
        )
        if (self.preflight_voltage_samples_required < 1 or
                self.preflight_voltage_timeout_s <= 0.0 or
                self.preflight_battery_max_age_s <= 0.0 or
                self.preflight_min_voltage_v <= 0.0 or
                self.preflight_voltage_reference_v <= 0.0 or
                self.preflight_raw_scale_exponent <= 0.0 or
                self.preflight_raw_scale_min <= 0.0 or
                self.preflight_raw_scale_max < self.preflight_raw_scale_min):
            raise rospy.ROSInitException(
                "预起飞电压采样、阈值和 raw 推力缩放参数必须有效"
            )

        # max_total_thrust 是物理标定上限（120 gf）；max_command_thrust 是本次控制器
        # 使用的更保守上限。两者均是四个电机的总推力。
        max_total_thrust = float(self.params.vehicle_param("max_total_thrust_newton"))
        max_command_thrust = float(
            self.params.vehicle_param("max_command_thrust_newton")
        )
        if max_total_thrust <= 0.0 or max_command_thrust <= 0.0:
            raise rospy.ROSInitException("推力上限必须为正数")
        max_command_thrust = min(max_command_thrust, max_total_thrust)
        omega_c_method = str(
            self.params.vehicle_param("omega_c_method", "analytic")
        )
        omega_c_force_norm_epsilon = float(
            self.params.vehicle_param("omega_c_force_norm_epsilon", 1e-8)
        )
        omega_c_heading_projection_epsilon = float(
            self.params.vehicle_param("omega_c_heading_projection_epsilon", 1e-6)
        )
        if (omega_c_method.strip().lower() not in
                ("analytic", "derivative", "log_difference", "log", "so3_log")):
            raise rospy.ROSInitException(
                "omega_c_method 必须为 analytic 或 log_difference"
            )
        if omega_c_method.strip().lower() in ("log_difference", "log", "so3_log"):
            rospy.logwarn(
                "omega_c_method=%s：滤波加速度不会进入 Omega_c；"
                "设为 analytic 才会启用 V2 解析角速度前馈",
                omega_c_method,
            )
        if (not math.isfinite(omega_c_force_norm_epsilon) or
                omega_c_force_norm_epsilon <= 0.0 or
                not math.isfinite(omega_c_heading_projection_epsilon) or
                omega_c_heading_projection_epsilon <= 0.0):
            raise rospy.ROSInitException("Omega_c 数值保护参数必须为正数")
        self.controller = GeometricCtbrController(ControllerConfig(
            mass=mass_kg,
            gravity=float(self.params.global_param("gravity_mps2")),
            max_total_thrust=max_total_thrust,
            max_command_thrust=max_command_thrust,
            max_tilt_rad=math.radians(float(
                self.params.vehicle_param("max_tilt_deg")
            )),
            max_body_rate=self._vector_param(self.params.vehicle_param, "max_body_rate_radps"),
            position_gain=self._vector_param(self.params.vehicle_param, "position_gain"),
            velocity_gain=self._vector_param(self.params.vehicle_param, "velocity_gain"),
            integral_gain=self._vector_param(self.params.vehicle_param, "integral_gain"),
            integral_limit=self._vector_param(self.params.vehicle_param, "integral_limit"),
            independent_position_gain=self._vector_param(
                self.params.vehicle_param, "independent_position_gain"
            ),
            independent_velocity_gain=self._vector_param(
                self.params.vehicle_param, "independent_velocity_gain"
            ),
            independent_integral_gain=self._vector_param(
                self.params.vehicle_param, "independent_integral_gain"
            ),
            independent_integral_limit=self._vector_param(
                self.params.vehicle_param, "independent_integral_limit"
            ),
            independent_integral_gate=float(
                self.params.vehicle_param("independent_integral_gate")
            ),
            independent_max_feedback_acceleration=self._vector_param(
                self.params.vehicle_param, "independent_max_feedback_acceleration"
            ),
            independent_max_body_rate=self._vector_param(
                self.params.vehicle_param, "independent_max_body_rate_radps"
            ),
            independent_attitude_gain=self._vector_param(
                self.params.vehicle_param, "independent_attitude_gain"
            ),
            independent_heading=self._vector_param(
                self.params.vehicle_param, "independent_heading"
            ),
            attitude_gain=self._vector_param(self.params.vehicle_param, "attitude_gain"),
            attitude_integral_gain=self._vector_param(
                self.params.vehicle_param, "attitude_integral_gain"
            ),
            attitude_integral_limit=self._vector_param(
                self.params.vehicle_param, "attitude_integral_limit"
            ),
            position_integral_c1=float(
                self.params.vehicle_param("position_integral_c1")
            ),
            use_body_rate_feedforward=bool(
                self.params.vehicle_param("use_body_rate_feedforward")
            ),
            omega_c_method=omega_c_method,
            omega_c_force_norm_epsilon=omega_c_force_norm_epsilon,
            omega_c_heading_projection_epsilon=omega_c_heading_projection_epsilon,
            transport_body_rate_filter_tau_s=float(self.params.vehicle_param(
                "transport_body_rate_filter_tau_s", 0.0
            )),
            transport_force_filter_tau_s=float(self.params.vehicle_param(
                "transport_force_filter_tau_s", 0.0
            )),
        ))

        # These static gains must not query the ROS master in the 70 Hz loop.
        for name, default in (("transport_attitude_gain", [240., 240., 120.]),
                              ("transport_rate_gain", [4., 4., 4.])):
            try:
                gain = as_vector(self.params.vehicle_param(name, default), name).copy()
            except ValueError as error:
                raise rospy.ROSInitException(str(error))
            if np.any(gain < 0):
                raise rospy.ROSInitException(name + " 必须为非负有限三维向量")
            gain.setflags(write=False)
            setattr(self, name, gain)

        # 每次启动创建一份独立 CSV，避免覆盖上一次实飞；状态和电池回调共享同一把锁。
        log_directory = self.params.global_param("log_directory")
        log_prefix = rospy.get_param(
            "~log_prefix", "cf%d_ctbr" % self.vehicle_id
        )
        if self.logger is None:
            self.logger = FlightCsvLogger(log_directory, log_prefix)
        self.path_frame_id = str(self.params.global_param("path_frame_id", "world"))
        self.path_publish_interval_s = float(
            self.params.global_param("path_publish_interval_s", 0.1)
        )
        self.path_max_poses = int(self.params.global_param("path_max_poses", 1000))
        if not self.path_frame_id:
            raise rospy.ROSInitException("path_frame_id 不能为空")
        if self.path_publish_interval_s <= 0.0:
            raise rospy.ROSInitException("path_publish_interval_s 必须为正数")
        if self.path_max_poses <= 0:
            raise rospy.ROSInitException("path_max_poses 必须为正整数")
        self.path_publisher = rospy.Publisher(
            rospy.get_param("~path_topic", self.prefix + "/path"),
            Path,
            queue_size=1,
            latch=True,
        )
        self.path_message = Path()
        self.path_message.header.frame_id = self.path_frame_id
        self.last_path_publish_time = 0.0
        self.lock = threading.Lock()
        self.latest_state = None
        self.latest_ekf_state = None
        self.latest_battery = None
        self.payload_diagnostics = None
        self.latest_raw_thrust = None
        self.latest_raw_thrust_received_time = None
        self.latest_bridge_timing = None
        self.last_published_command_stamp_ns = 0
        self.controller_compute_wall_s = math.nan
        self.previous_vehicle_cycle_wall_s = math.nan
        self.previous_vehicle_cycle_control_time_s = math.nan
        self.previous_fleet_cycle_wall_s = math.nan
        self.previous_fleet_cycle_control_time_s = math.nan
        # 最后一帧经过位置一致性检查的 EKF 运动学。飞行中只允许在很短的时间窗口
        # 内保持它，绝不改用 NOKOV 的差分速度/加速度。
        self.last_valid_ekf_kinematics = None
        self.ekf_alignment_started_time = None
        self.ekf_alignment_since = None
        self.ekf_alignment_failed = False
        self.ekf_alignment_failure_reason = ""
        self.ekf_fault_since = None
        # 回调只在收到一条新的 /battery 消息时追加样本，避免控制定时器把同一条
        # 10 Hz 电压消息误计为 100 个独立样本。
        self.preflight_voltage_samples = deque(
            maxlen=max(2 * self.preflight_voltage_samples_required, 50)
        )
        self.last_tick = time.monotonic()
        self.start_time = self.last_tick if self.shared_start_time is None else self.shared_start_time
        # 单机直接运行时也记录从控制器创建开始的任务时间；多机模式则沿用
        # 管理器传入的共享时钟，保证所有飞机的 CSV 时间轴一致。
        if self.mission_start_time is None:
            self.mission_start_time = self.start_time
        # 轨迹模块在第一帧有效状态后创建；latest_* 由 ROS 回调线程写入，必须受锁保护。
        self.trajectory = None
        self.last_trajectory_phase = None
        self.flight_phase = "waiting_for_state"
        self.preflight_voltage_started_time = None
        self.preflight_voltage_v = math.nan
        self.preflight_voltage_sample_count = 0
        self.thrust_raw_scale = 1.0
        self.preflight_voltage_ready = not self.require_preflight_voltage
        self.preflight_voltage_failed = False
        self.preflight_voltage_failure_reason = ""

        self.command_publisher = rospy.Publisher(
            self.prefix + "/cmd_ctbr", CTBR, queue_size=1
        )
        # C++ 桥接层发布实际送入 sendSetpoint() 的 raw PWM，供 CSV 对齐诊断。
        self.raw_thrust_subscriber = rospy.Subscriber(
            self.prefix + "/ctbr_raw_thrust", UInt16,
            self._raw_thrust_callback, queue_size=1,
        )
        self.timing_subscriber = rospy.Subscriber(
            self.prefix + "/ctbr_timing", CTBRTiming,
            self._ctbr_timing_callback, queue_size=1,
        )
        self.state_subscriber = rospy.Subscriber(
            self.prefix + "/mocap_state", MocapState, self._state_callback, queue_size=1
        )
        self.ekf_subscriber = rospy.Subscriber(
            self.prefix + "/ekf_kinematics", GenericLogData,
            self._ekf_state_callback, queue_size=1,
        )
        # hover_swarm.launch 配置 GenericLogData 的 values 顺序为
        # [pm.vbat(V), pm.vbatMV(mV), pm.state, pm.batteryLevel]；电池日志只在
        # 低频重复提供样本，控制器完成预检后不再依赖后续电池消息。
        # 未启用固件日志时该订阅者保持空闲，控制器仍可运行，只会在 CSV 中写 NaN。
        self.battery_subscriber = rospy.Subscriber(
            self.prefix + "/battery", GenericLogData, self._battery_callback, queue_size=1
        )
        self.timer = None
        if self.auto_timer:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)
        if self.register_shutdown:
            rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "CTBR 控制器已启动：cf%d，话题前缀 %s，日志：%s",
            self.vehicle_id, self.prefix, self.logger.path,
        )

    def _vector_param(self, reader, name):
        """读取必需的三维参数，并统一转换错误为 ROS 启动异常。"""
        try:
            return as_vector(reader(name), name)
        except ValueError as error:
            raise rospy.ROSInitException(str(error))

    def _target_for(self, state, now):
        """取得轨迹模块的当前参考目标。"""
        if self.trajectory is None:
            self.trajectory = CircularFlightTrajectory(self.trajectory_config)
            self.trajectory.reset(
                state["position"], rotation_to_rpy(state["rotation"])[2], now
            )

        target = self.trajectory.evaluate(state, now)
        for event in self.trajectory.pop_events():
            rospy.loginfo("%s", event)
        return target

    def _resume_or_abort_trajectory(self, state, now):
        """恢复短暂暂停的轨迹，或在长时间动捕失效后锁定零输出中止状态。"""
        if self.trajectory is None or not self.trajectory.is_paused:
            return
        pause_duration = self.trajectory.resume(now)
        # 状态失效路径已经 reset 过积分器；这里再次 reset 使恢复后的第一帧不会带入
        # 丢帧前的姿态差分历史。
        self.controller.reset()
        if pause_duration > self.trajectory_pause_abort_s:
            self.trajectory.abort(
                now,
                "Nokov 状态连续失效 %.3f s，超过 %.3f s" % (
                    pause_duration, self.trajectory_pause_abort_s
                ),
                position=state["position"],
            )

    def _mocap_state_ages(self, state, now):
        """Return callback and NOKOV-header ages for one immutable state snapshot.

        ``received_time`` is the primary freshness signal: it measures when this
        process actually received the valid mocap state.  The header timestamp is
        retained to reject queued old frames and a genuinely faulty future clock.
        ROS callbacks may publish a new frame between two control operations, so
        take ``ros_now`` only *after* the state snapshot is selected.
        """
        state_age = math.inf if state is None else float(now) - float(
            state.get("received_time", math.inf)
        )
        try:
            ros_now = rospy.Time.now().to_sec()
            mocap_sample_age = float(ros_now) - float(state["mocap_sample_time_s"])
        except (AttributeError, KeyError, TypeError, ValueError):
            mocap_sample_age = math.nan
        return state_age, mocap_sample_age

    def _state_ready(self, now):
        """Return whether this vehicle has a fresh, valid NOKOV state."""
        with self.lock:
            state = self.latest_state
        if state is None or not state.get("valid", False):
            return False
        state_age, mocap_sample_age = CtbrControllerNode._mocap_state_ages(
            self, state, now
        )
        future_tolerance = float(getattr(
            self, "mocap_timestamp_future_tolerance_s", 0.0
        ))
        return (
            math.isfinite(state_age)
            and state_age <= self.state_timeout
            and math.isfinite(mocap_sample_age)
            and -future_tolerance <= mocap_sample_age <= self.state_timeout
        )

    def set_fleet_gate(self, open_gate):
        """Allow the multi-vehicle manager to release this vehicle together."""
        self.fleet_gate_open = bool(open_gate)

    def set_global_abort(self, reason):
        """Latch a fleet-wide abort reason; subsequent cycles only send zero CTBR."""
        self.global_abort_reason = str(reason or "")

    def set_payload_diagnostics(self, state):
        """Attach the latest load sample to this vehicle's next CSV row."""
        if state is None:
            self.payload_diagnostics = None
            return
        self.payload_diagnostics = dict(state)

    def _timer_callback(self, _event, now=None, transport_override=None):
        control_now = time.monotonic() if now is None else float(now)
        wall_start = time.perf_counter()
        self.controller_tick_interval_s = control_now - self.last_tick
        self.controller_compute_wall_s = math.nan
        try:
            return CtbrControllerNode._run_control_tick(
                self, _event, now=control_now, transport_override=transport_override)
        finally:
            self.previous_vehicle_cycle_wall_s = time.perf_counter() - wall_start
            self.previous_vehicle_cycle_control_time_s = control_now - getattr(self, "start_time", control_now)

    def _run_control_tick(self, _event, now=None, transport_override=None):
        """控制周期入口：先执行状态失效保护，再计算、发布并记录同一份命令。"""
        now = time.monotonic() if now is None else float(now)
        dt = now - self.last_tick
        self.last_tick = now
        with self.lock:
            state = self.latest_state

        if getattr(self, "global_abort_reason", ""):
            self.flight_phase = "aborted"
            self._publish_zero()
            if state is None:
                self._write_invalid_log(now, state, "global_abort_without_state")
            else:
                target = self._preflight_hold_target(state, "aborted")
                command = self._zero_command(state, target)
                self._write_log(
                    now, state, target, command,
                    invalid_reason="global_abort:" + self.global_abort_reason,
                )
            return

        state_age, mocap_sample_age = CtbrControllerNode._mocap_state_ages(
            self, state, now
        )
        future_tolerance = float(getattr(
            self, "mocap_timestamp_future_tolerance_s", 0.0
        ))
        state_ready = (
            state is not None
            and state["valid"]
            and math.isfinite(state_age)
            and state_age <= self.state_timeout
            and math.isfinite(mocap_sample_age)
            and -future_tolerance <= mocap_sample_age <= self.state_timeout
        )
        if not state_ready:
            # 不使用旧 mocap 继续飞行。服务器端还有 0.1 s CTBR watchdog；这里主动
            # 发送零包可比等待 watchdog 更快地切断推力。
            reasons = []
            if state is None:
                reasons.append("尚未收到动捕状态")
            else:
                if not state["valid"]:
                    reasons.append("mocap_state.valid=false")
                if state_age > self.state_timeout:
                    reasons.append("状态超时")
            if not math.isfinite(mocap_sample_age):
                reasons.append("mocap 时间戳无效或回退")
            elif mocap_sample_age < -future_tolerance:
                reasons.append(
                    "mocap 时间戳超前 %.3f s（容差 %.3f s）" % (
                        -mocap_sample_age,
                        future_tolerance,
                    )
                )
            elif mocap_sample_age > self.state_timeout:
                reasons.append("mocap 样本超时")
            rospy.logwarn_throttle(
                1.0,
                "CTBR 安全保护：%s；发送零推力 "
                "(phase=%s, state_age=%.3f s, timeout=%.3f s)",
                "、".join(reasons) if reasons else "状态检查失败",
                self.flight_phase,
                state_age,
                self.state_timeout,
            )
            self.controller.reset()
            with self.velocity_filter_lock:
                self.velocity_filter.reset()
            if self.trajectory is not None:
                self.trajectory.pause(now)
            self._publish_zero()
            self._write_invalid_log(
                now, state, ";".join(reasons) or "state_check_failed"
            )
            return

        # 路径发布与控制阶段无关：只要收到有效动捕，就把实际位置提供给 RViz。
        self._publish_path(state)
        self._resume_or_abort_trajectory(state, now)
        preflight_target = self._preflight_voltage_target(state, now)
        if preflight_target is not None:
            # 预检阶段绝不调用几何控制器：即使刚体处于地面，重力补偿也会形成非零
            # 悬停推力。持续零包同时满足 legacy commander 的首包解锁要求。
            self.controller.reset()
            command = self._zero_command(state, preflight_target)
            self._publish_zero()
            self._write_log(now, state, preflight_target, command)
            return

        if self.trajectory is None and not self._ekf_preflight_ready(state, now):
            # The EKF may be reset/reinitialized only while all motors remain
            # at zero.  Do not enter the trajectory merely because NOKOV is
            # fresh: this controller deliberately requires EKF v/a feedback.
            target = self._preflight_hold_target(state, "waiting_for_ekf")
            self.controller.reset()
            command = self._zero_command(state, target)
            self._publish_zero()
            self._write_log(
                now, state, target, command,
                control_state=self._build_ekf_control_state(state, now),
            )
            return

        if not getattr(self, "fleet_gate_open", True):
            # In multi-vehicle mode, an individually ready vehicle must remain
            # on zero output until every selected vehicle passes preflight and
            # has a fresh NOKOV state.
            target = self._preflight_hold_target(state, "waiting_for_fleet")
            self.controller.reset()
            command = self._zero_command(state, target)
            self._publish_zero()
            self._write_log(now, state, target, command)
            return

        target = self._target_for(state, now)
        if transport_override is not None:
            target = dict(target)
            target.update(transport_override)
        elif target.get("flight_phase") in ("takeoff", "height_correction"):
            target = dict(target)
            target["position_gain_scale"] = self.takeoff_position_gain_scale
            target["velocity_gain_scale"] = self.takeoff_velocity_gain_scale
        control_state = self._build_ekf_control_state(state, now)
        if (self._ekf_kinematics_enabled and
                not control_state["ekf_state_valid"] and
                not control_state["ekf_kinematics_held"] and
                control_state["ekf_fault_age_s"] >=
                self.ekf_fault_emergency_land_s and
                target["flight_phase"] not in (
                    "emergency_landing", "landing", "landing_settle", "landed"
                )):
            # Never reset a flying EKF or fall back to NOKOV derivatives.  A
            # prolonged failure latches a slow position-reference landing;
            # control_state contains zero v/a after the short hold window.
            self.controller.reset()
            self.trajectory.begin_emergency_landing(
                now,
                state["position"],
                rotation_to_rpy(state["rotation"])[2],
                "EKF 运动学持续失效：" + control_state.get("ekf_status", "unknown"),
            )
            target = self._target_for(state, now)
        # 普通阶段的参考位置、速度和加速度连续衔接。保留位置积分可维持每架飞机的
        # 稳态推力补偿，避免入圆或降落开始时因积分清零而产生突跳。只有安全/终止
        # 阶段才执行完整 reset。
        if (target["flight_phase"] != self.last_trajectory_phase and
                phase_requires_full_controller_reset(target["flight_phase"])):
            self.controller.reset()
        self.last_trajectory_phase = target["flight_phase"]
        if target["flight_phase"] == "landed" or target.get("zero_output", False):
            # landed、起飞前保持和安全中止必须绕开正常 compute()，否则目标地面高度的
            # 重力补偿会再次产生悬停推力，使飞机在地面附近反复弹起或提前离地。
            command = self._zero_command(control_state, target)
        else:
            compute_start = time.perf_counter()
            command = self.controller.compute(control_state, target, dt)
            self.controller_compute_wall_s = time.perf_counter() - compute_start
        if target["flight_phase"] == "landed" or target.get("zero_output", False):
            self._publish_zero()
        else:
            self._publish_command(command)
        self.last_control_command = command
        self._write_log(now, state, target, command, control_state=control_state)

    def _publish_path(self, state):
        """发布动捕实际轨迹，RViz 可直接订阅 nav_msgs/Path。"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.path_frame_id
        pose.pose.position.x = float(state["position"][0])
        pose.pose.position.y = float(state["position"][1])
        pose.pose.position.z = float(state["position"][2])
        pose.pose.orientation.w = 1.0
        self.path_message.header.stamp = pose.header.stamp
        self.path_message.poses.append(pose)
        if len(self.path_message.poses) > self.path_max_poses:
            del self.path_message.poses[:-self.path_max_poses]
        now = time.monotonic()
        if now - self.last_path_publish_time >= self.path_publish_interval_s:
            self.last_path_publish_time = now
            self.path_publisher.publish(self.path_message)

    def _publish_command(self, command):
        """把 SI 单位的控制输出封装为 CTBR 消息；raw PWM 转换在 C++ 桥接层完成。"""
        message = CTBR()
        message.header.stamp = rospy.Time.now()
        message.body_rates.x = float(command["body_rate_command"][0])
        message.body_rates.y = float(command["body_rate_command"][1])
        message.body_rates.z = float(command["body_rate_command"][2])
        message.collective_thrust = float(command["collective_thrust"])
        # 物理推力仍由 collective_thrust（N）表达；冻结电压只修正 C++ 中的 raw PWM。
        message.thrust_raw_scale = float(self.thrust_raw_scale)
        with self.lock:
            self.last_published_command_stamp_ns = self._stamp_nanoseconds(message.header.stamp)
        self.command_publisher.publish(message)

    def _publish_zero(self):
        """发送一个显式零 RPYT/推力包，用于状态失效、落地和节点退出。"""
        message = CTBR()
        message.header.stamp = rospy.Time.now()
        with self.lock:
            self.last_published_command_stamp_ns = self._stamp_nanoseconds(message.header.stamp)
        self.command_publisher.publish(message)

    @staticmethod
    def _zero_command(state, target):
        """记录落地后的真实误差，但不再请求任何推力或角速度。"""
        measured_velocity = state.get(
            "control_velocity",
            state.get("filtered_velocity", state["velocity"]),
        )
        return {
            "position_error": state["position"] - target["position"],
            "velocity_error": measured_velocity - target["velocity"],
            "desired_force": np.zeros(3),
            "computed_rotation": state["rotation"],
            "computed_body_rate": np.zeros(3),
            "omega_c_method": "zero",
            "attitude_error": np.zeros(3),
            "body_rate_command": np.zeros(3),
            "collective_thrust": 0.0,
        }

    def _shutdown(self):
        """ROS 退出钩子：先保证零推力包离开主机，再关闭日志文件。"""
        self._publish_zero()
        self.logger.close()
