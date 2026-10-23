"""Fleet ROS node; delegates load missions and pure controllers."""

from copy import deepcopy
from crazyswarm.msg import MocapState
from ctbr_control_math import project_so3
from ctbr_csv_logger import FlightCsvLogger
from ctbr_parameters import GLOBAL_CTBR_PARAMETER_ROOT
from ctbr_payload_control import PayloadControlMixin
from ctbr_payload_mission import PayloadMissionMixin
from ctbr_payload_parameters import PayloadParametersMixin
from ctbr_payload_reference import PayloadReferenceMixin
from ctbr_payload_state import PayloadStateMixin
from ctbr_payload_takeup import PayloadTakeupMixin
from ctbr_vehicle_node import CtbrControllerNode
from slung_load_controller import ordered_vehicle_ids
from vehicle_config import select_vehicle_entries
from vehicle_config import validate_vehicle_entries
import math
import numpy as np
import rospy
import time


class MultiCtbrControllerNode(PayloadParametersMixin, PayloadStateMixin, PayloadTakeupMixin, PayloadReferenceMixin, PayloadControlMixin, PayloadMissionMixin):
    """Shared radio, synchronized ticks and fleet-wide preflight/recovery."""

    def __init__(self):
        entries = rospy.get_param("/crazyflies", [])
        requested_id = rospy.get_param("~cf_id", None)
        try:
            if requested_id is None:
                selected_entries = select_vehicle_entries(entries, ctbr_only=True)
                multi_mode = len(selected_entries) > 1
                self.vehicle_configs = validate_vehicle_entries(
                    selected_entries,
                    require_ctbr=True,
                    min_ctbr=1,
                    require_shared_radio=multi_mode,
                    require_phase=multi_mode,
                )
            else:
                self.vehicle_configs = select_vehicle_entries(
                    entries, cf_id=requested_id, ctbr_only=False
                )
        except (KeyError, TypeError, ValueError) as error:
            raise rospy.ROSInitException("多机飞机参数无效：%s" % error)

        self.transport_mode = str(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/transport_mode", "formation"
        )).strip().lower().replace("-", "_")
        if self.transport_mode == "slung_load":
            try:
                self.vehicle_configs = sorted(
                    self.vehicle_configs, key=lambda item: int(item["id"])
                )
                ordered_vehicle_ids(self.vehicle_configs)
            except (KeyError, TypeError, ValueError) as error:
                raise rospy.ROSInitException(
                    "slung_load 模式必须启用 CF3、CF4、CF5：%s" % error
                )

        try:
            self.takeoff_vehicle_count = int(
                rospy.get_param(
                    GLOBAL_CTBR_PARAMETER_ROOT + "/takeoff_vehicle_count",
                    len(self.vehicle_configs),
                )
            )
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException("takeoff_vehicle_count 无效：%s" % error)
        if self.takeoff_vehicle_count < 1:
            raise rospy.ROSInitException("takeoff_vehicle_count 必须为正整数")
        # ``~cf_id`` is an explicit single-vehicle compatibility mode used for
        # bench tests and recovery.  Do not reject it merely because the normal
        # formation parameter still contains the full fleet count.
        if requested_id is None and self.takeoff_vehicle_count != len(self.vehicle_configs):
            raise rospy.ROSInitException(
                "takeoff_vehicle_count=%d，但实际启用 %d 架 ctbr_enabled 飞机；"
                "请同步修改参数和 crazyflies.yaml" %
                (self.takeoff_vehicle_count, len(self.vehicle_configs))
            )

        if len(self.vehicle_configs) > 1 and rospy.get_param("~cf_prefix", ""):
            raise rospy.ROSInitException("多机模式不能使用统一的 ~cf_prefix")

        log_directory = rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/log_directory"
        )
        # Keep a legacy-looking filename for explicit/single-aircraft runs so
        # the realtime viewer does not wait for a second vehicle forever.
        self.multi_mode = requested_id is None and len(self.vehicle_configs) > 1
        log_prefix = (
            "multi_ctbr"
            if self.multi_mode else
            "cf%d_ctbr" % int(self.vehicle_configs[0]["id"])
        )
        self.logger = FlightCsvLogger(log_directory, log_prefix)
        self.rate_hz = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/control_rate_hz"
        ))
        self.trajectory_pause_abort_s = float(
            rospy.get_param(GLOBAL_CTBR_PARAMETER_ROOT + "/trajectory_pause_abort_s")
        )
        self.takeoff_position_gain_scale = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/takeoff_position_gain_scale", 1.0
        ))
        self.takeoff_velocity_gain_scale = float(rospy.get_param(
            GLOBAL_CTBR_PARAMETER_ROOT + "/takeoff_velocity_gain_scale", 1.0
        ))
        if (not math.isfinite(self.takeoff_position_gain_scale) or
                self.takeoff_position_gain_scale < 1.0 or
                not math.isfinite(self.takeoff_velocity_gain_scale) or
                self.takeoff_velocity_gain_scale < 1.0):
            self.logger.close()
            raise rospy.ROSInitException("起飞阶段增益倍率必须是不小于 1 的有限数")
        if self.rate_hz <= 0.0 or self.trajectory_pause_abort_s <= 0.0:
            self.logger.close()
            raise rospy.ROSInitException("多机控制频率和中止阈值必须为正数")

        self._initialize_payload_parameters()

        mission_start = time.monotonic()
        self.vehicles = [
            CtbrControllerNode(
                vehicle_config=vehicle_config,
                logger=self.logger,
                auto_timer=False,
                register_shutdown=False,
                start_time=mission_start,
            )
            for vehicle_config in self.vehicle_configs
        ]
        if self.transport_mode == "slung_load":
            rospy.loginfo("吊运名义绳长 CF3/CF4/CF5=%s；本次运行有效绳长校准=%s。",
                          self.payload_nominal_link_lengths.tolist(), self.payload_link_calibration_enabled)
            self.payload_subscriber = rospy.Subscriber(
                self.payload_topic, MocapState, self._payload_state_callback,
                queue_size=1,
            )
        self.mission_start_time = mission_start
        self.fleet_gate_open = False
        self.global_abort_reason = ""
        # Terminal-only mission stage monitor.  The CSV keeps the exact
        # per-vehicle ``flight_phase``; this compact view makes the shared
        # multi-vehicle state machine easy to follow during a flight.
        self.last_terminal_stage = None
        self.last_tick = mission_start
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._timer_callback
        )
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "多机 CTBR 控制器已启动：%d 架飞机，共用一份日志 %s",
            len(self.vehicles), self.logger.path,
        )
        if self.transport_mode == "slung_load":
            rospy.loginfo(
                "吊运飞机反馈：%s；native I 增益倍率 %.3f、状态上限倍率 %.3f、"
                "门限 %.3f m、积分速度权重 %.3f、附加 I 力上限 %.3f N、变化率 %.3f N/s。",
                self.payload_transport_feedback_controller,
                self.payload_transport_pid_integral_gain_scale,
                self.payload_transport_pid_integral_limit_scale,
                self.payload_transport_pid_integral_gate_m,
                self.payload_transport_pid_integral_velocity_weight,
                self.payload_transport_pid_force_limit_n,
                self.payload_transport_pid_force_rate_limit_nps,
            )
            if self.payload_flight_validation_enabled:
                rospy.loginfo(
                    "[实飞验证] 已启用：从进入 TAUT_RAMP 开始计时 %.2f s 后受控降落"
                    "（包含张力渐增和参考抬升，不等待完整轨迹结束）。",
                    self.payload_flight_validation_transport_s,
                )
            rospy.loginfo(
                "吊运主流程共 5 个阶段：1 起飞前保持，2 独立起飞，"
                "3 TAKEUP 收绳，4 负载接管/轨迹跟踪，5 降落。"
                "等待 EKF/全机就绪属于起飞前准备。"
            )

    def _terminal_mission_stage(self):
        """Return the compact five-stage view and exact per-vehicle phases."""
        phases = [str(vehicle.flight_phase) for vehicle in self.vehicles]
        phase_text = ", ".join(
            "CF%d=%s" % (vehicle.vehicle_id, phase)
            for vehicle, phase in zip(self.vehicles, phases)
        )

        if self.global_abort_reason or "aborted" in phases:
            return 5, "安全中止", phase_text
        if (any(p in ("landing", "landing_settle", "landed") for p in phases) and
                self.payload_validation_landing_started_time is not None):
            return 5, "实飞验证结束/受控降落", phase_text
        if "emergency_landing" in phases:
            return 5, "紧急降落", phase_text
        if any(phase in ("landing", "landing_settle", "landed")
               for phase in phases):
            return 5, "降落/落地确认", phase_text
        if (self.payload_transport_active or "tension_ramp" in phases or
                any(phase in (
                    "height_correction", "circle_entry", "circle",
                    "figure_eight", "hover", "final_hover"
                ) for phase in phases)):
            if self.payload_transport_active or "tension_ramp" in phases:
                return 4, "负载接管/轨迹跟踪", phase_text
            return 4, "轨迹跟踪（负载尚未接管）", phase_text
        if any(phase == "takeup" for phase in phases):
            return 3, "TAKEUP 收绳", phase_text
        if any(phase == "takeoff" for phase in phases):
            return 2, "独立起飞", phase_text
        if any(phase == "pre_takeoff_hold" for phase in phases):
            return 1, "起飞前保持", phase_text
        return 1, "起飞前准备/等待全机就绪", phase_text

    def _log_terminal_mission_stage(self):
        """Print one shared stage line whenever the fleet stage changes."""
        stage, label, phase_text = self._terminal_mission_stage()
        key = (stage, label, phase_text)
        if key == self.last_terminal_stage:
            return
        self.last_terminal_stage = key
        log_prefix = "吊运阶段" if self.transport_mode == "slung_load" else "任务阶段"
        rospy.loginfo(
            "[%s] %d/5 %s；内部状态：%s",
            log_prefix, stage, label, phase_text,
        )

    def _all_states_ready(self, now):
        return all(vehicle._state_ready(now) for vehicle in self.vehicles)

    def _all_preflight_ready(self, now):
        if not self._payload_ready(now):
            return False
        for vehicle in self.vehicles:
            if not vehicle._state_ready(now):
                return False
            with vehicle.lock:
                state = vehicle.latest_state
            if state is None or not vehicle._ekf_preflight_ready(state, now):
                return False
        return True

    def _set_abort(self, reason):
        if self.global_abort_reason:
            return
        self.global_abort_reason = str(reason)
        if getattr(self, "payload_attitude_trim", None) is not None:
            self.payload_attitude_trim.reset()
        self.payload_attitude_trim_landing_bridge = None
        self.payload_transport_integral.reset()
        self.payload_transport_integral_last_time = None
        for vehicle in self.vehicles:
            vehicle.controller.reset_transport_position_pid()
            vehicle.set_global_abort(self.global_abort_reason)
        rospy.logerr("多机 CTBR 全局中止：%s；所有飞机发送零推力", reason)

    @staticmethod
    def _finite_recovery_state(vehicle, now):
        try:
            if not vehicle._state_ready(now):
                return False
            with vehicle.lock:
                state = vehicle.latest_state
            control = vehicle._build_ekf_control_state(state, now)
            return (np.asarray(state["position"]).shape == (3,)
                    and np.asarray(state["rotation"]).shape == (3, 3)
                    and all(np.all(np.isfinite(value)) for value in (
                        state["position"], state["rotation"], state.get("body_rate", np.zeros(3)),
                        control.get("control_velocity", np.zeros(3)),
                        control.get("control_acceleration", np.zeros(3)))))
        except (ValueError, ArithmeticError, np.linalg.LinAlgError):
            return False

    def _recover_fleet_numerical_fault(self, now, error):
        """Contain numerical faults without bypassing arming or state freshness."""
        if not self.fleet_numerical_fault_reason:
            self.fleet_numerical_fault_reason = "%s: %s" % (type(error).__name__, error)
            self.fleet_numerical_fault_since = float(now)
            rospy.logerr("多机控制数值故障，转入有支撑受控退出：%s", self.fleet_numerical_fault_reason)
        now = max(float(now), float(time.monotonic()))
        self.last_tick = now
        healthy = all(self._finite_recovery_state(v, now) for v in self.vehicles)
        if self.fleet_gate_open and not healthy:
            self._set_abort("fleet_numerical: invalid or stale vehicle state")
        armed = (self.fleet_gate_open and healthy and not self.global_abort_reason
                 and all(v.preflight_voltage_ready and not v.preflight_voltage_failed
                         and v.trajectory is not None for v in self.vehicles))
        if armed:
            try:
                if self.payload_validation_landing_stage != "planning_fault_descent":
                    self._exit_payload_planning_fault(now, self.fleet_numerical_fault_reason,
                                                     source="fleet_numerical")
                # Fresh measured separation and height diagnostics still run;
                # the failed cooperative planner is never called here.
                self._check_payload_physical_safety(now)
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as guard_error:
                self._set_abort("fleet_numerical: fault descent/guard failed: %s" % guard_error)
        for vehicle in self.vehicles:
            # The vehicle latches last_control_command after publication and
            # before CSV conversion. A logging failure must not actuate the
            # same vehicle again merely because its callback did not return.
            already_published = (id(vehicle) in self.fleet_tick_initial_commands
                and getattr(vehicle, "last_control_command", None)
                is not self.fleet_tick_initial_commands[id(vehicle)])
            if id(vehicle) in self.fleet_tick_completed_vehicles or already_published:
                continue
            command_now = max(now, float(time.monotonic()))
            if not armed or self.global_abort_reason:
                vehicle.last_tick = command_now
                vehicle._publish_zero()
                continue
            try:
                vehicle._timer_callback(None, now=command_now,
                    transport_override=self._payload_emergency_override(vehicle, command_now))
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as control_error:
                if (id(vehicle) in self.fleet_tick_initial_commands
                        and getattr(vehicle, "last_control_command", None)
                        is not self.fleet_tick_initial_commands[id(vehicle)]):
                    # Publication completed before a diagnostic conversion
                    # failed. Do not compute or publish its command twice.
                    self.fleet_tick_completed_vehicles.add(id(vehicle))
                    continue
                # A finite previously published command may bridge only the
                # short watchdog window if the independent core also fails.
                # It cannot become an indefinite open-loop force command.
                supported = self._finite_supported_command(getattr(vehicle, "last_control_command", None))
                if (supported and command_now - self.fleet_numerical_fault_since <= .3
                        and self._finite_recovery_state(vehicle, command_now)):
                    vehicle.last_tick = command_now
                    vehicle._publish_command(dict(vehicle.last_control_command))
                else:
                    self._set_abort("fleet_numerical: vehicle control failed: %s" % control_error)
                    vehicle._publish_zero()
            self.fleet_tick_completed_vehicles.add(id(vehicle))
        try:
            self.logger.flush()
        except (ValueError, OSError) as logging_error:
            rospy.logwarn_throttle(1., "控制退出命令已发布，CSV 刷新失败：%s", logging_error)

    def _timer_callback(self, _event):
        now = time.monotonic()
        wall_start = time.perf_counter()
        previous_start = getattr(self, "previous_fleet_cycle_started_time", math.nan)
        for vehicle in self.vehicles:
            vehicle.fleet_tick_interval_s = now - self.last_tick
            vehicle.previous_fleet_cycle_wall_s = getattr(self, "previous_fleet_cycle_wall_s", math.nan)
            vehicle.previous_fleet_cycle_control_time_s = previous_start - vehicle.start_time
        self.fleet_tick_completed_vehicles = set()
        self.fleet_tick_initial_commands = {
            id(vehicle): getattr(vehicle, "last_control_command", None)
            for vehicle in self.vehicles}
        try:
            return MultiCtbrControllerNode._run_fleet_tick(self, _event, now)
        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
            return self._recover_fleet_numerical_fault(now, error)
        finally:
            self.previous_fleet_cycle_wall_s = time.perf_counter() - wall_start
            self.previous_fleet_cycle_started_time = now

    def _run_fleet_tick(self, _event, now):
        self.payload_control_dt = float(np.clip(now - self.last_tick, 0.001, 0.05))
        self.last_tick = now
        if self.transport_mode == "slung_load":
            self._update_payload_ground_reference(now)

        if self.global_abort_reason:
            for vehicle in self.vehicles:
                vehicle._timer_callback(None, now=now)
                self.fleet_tick_completed_vehicles.add(id(vehicle))
            self._log_terminal_mission_stage()
            self.logger.flush()
            return

        if not self.fleet_gate_open:
            for vehicle in self.vehicles:
                vehicle.set_fleet_gate(False)
                vehicle._timer_callback(None, now=now)
                self.fleet_tick_completed_vehicles.add(id(vehicle))
            failed = [
                vehicle for vehicle in self.vehicles
                if vehicle.preflight_voltage_failed or vehicle.ekf_alignment_failed
            ]
            if failed:
                ids = ",".join("cf%d" % vehicle.vehicle_id for vehicle in failed)
                self._set_abort("%s 起飞前预检失败（电压或 EKF 对齐）" % ids)
                return
            if all(
                    vehicle.preflight_voltage_ready
                    for vehicle in self.vehicles
            ) and self._all_preflight_ready(now):
                self.fleet_gate_open = True
                for vehicle in self.vehicles:
                    vehicle.set_fleet_gate(True)
                    vehicle.last_tick = now
                rospy.loginfo(
                    "全部 %d 架飞机完成预检和 NOKOV 状态检查，同时开始 CTBR 轨迹",
                    len(self.vehicles),
                )
            self._log_terminal_mission_stage()
            self.logger.flush()
            return

        if not self._all_states_ready(now):
            invalid_ids = [
                "cf%d" % vehicle.vehicle_id
                for vehicle in self.vehicles
                if not vehicle._state_ready(now)
            ]
            self._set_abort("NOKOV 状态失效：%s" % ",".join(invalid_ids))

        self._hold_payload_mission_clock(now)
        self._release_validation_payload(now)
        transport_phase_active = self._transport_phase_active(now)
        if (transport_phase_active and self.payload_transport_active and
                self._payload_landing_release_ready(now)):
            rospy.loginfo("负载已达到地面高度并稳定，结束 MATLAB LANDING_TAUT，释放绷紧控制。")
            self._reset_payload_handoff()
            transport_phase_active = False
        transport_overrides = {}
        takeup_overrides = {}
        if not transport_phase_active:
            self._reset_payload_handoff()
        elif not self.payload_transport_active:
            takeup_overrides = self._payload_takeup_overrides(now)
            with self.payload_lock:
                handoff_state = deepcopy(self.latest_payload_state)
            handoff_ready = self._payload_state_ready(handoff_state, now)
            if self._payload_prelift_motion_state(handoff_state, now) is not None:
                # A bounded pending interval keeps prelift continuous, but
                # ``handoff_ready`` remains strict for cooperative activation.
                self.payload_fault_since = None
            else:
                if self.payload_fault_since is None:
                    self.payload_fault_since = now
                if now - self.payload_fault_since > self.payload_emergency_land_after_s:
                    self._begin_payload_emergency_landing(now, "预抬期间负载状态持续失效")
                    takeup_overrides = {}
            takeup_ready = self._payload_takeup_ready(now)
            if handoff_ready and takeup_ready:
                if self.payload_ready_since is None:
                    self.payload_ready_since = now
                if now - self.payload_ready_since >= self.payload_activation_hold_s:
                    current_vehicle_positions = []
                    for vehicle in self.vehicles:
                        with vehicle.lock:
                            state = vehicle.latest_state
                        if state is None:
                            current_vehicle_positions = []
                            break
                        current_vehicle_positions.append(
                            np.asarray(state["position"], dtype=float).copy()
                        )
                    self.payload_handoff_reference_positions = (
                        self.payload_last_takeup_reference["positions"].copy()
                    )
                    self.payload_handoff_forces = np.asarray([
                        vehicle.last_control_command["desired_force"].copy()
                        for vehicle in self.vehicles
                    ])
                    self.payload_transport_active = True
                    self.payload_transport_start_time = now
                    self.payload_transport_start_position = np.asarray(
                        handoff_state["position"], dtype=float
                    ).copy()
                    self.payload_transport_start_rotation = project_so3(handoff_state["rotation"])
                    self.payload_transport_goal_rotation = None
                    if len(current_vehicle_positions) == len(self.vehicles):
                        # Preserve continuity at the TAKEUP -> TAUT_RAMP handoff.
                        # The cable geometry is measured from these actual
                        # positions; jumping back to nominal anchors can create
                        # a large position step even though the cables are taut.
                        self.payload_transport_vehicle_start_positions = np.asarray(
                            current_vehicle_positions, dtype=float
                        )
                    elif self.payload_takeup_target_positions is not None:
                        self.payload_transport_vehicle_start_positions = (
                            self.payload_takeup_target_positions.copy()
                        )
                    else:
                        self.payload_transport_vehicle_start_positions = None
                    self.payload_fault_since = None
                    rospy.loginfo(
                        "TAKEUP 已确认，负载状态连续有效 %.2f s，开始 TAUT_RAMP；"
                        "先保持地面参考 %.2f s，再抬升 %.2f s。",
                        self.payload_activation_hold_s,
                        self.payload_tension_ramp_s,
                        self.payload_reference_lift_s,
                    )
                else:
                    rospy.logwarn_throttle(
                        self.payload_takeup_diagnostic_interval_s,
                        "TAKEUP 绳长条件已满足，等待负载状态连续有效 %.2f/%.2f s 后进入 TAUT_RAMP",
                        now - self.payload_ready_since,
                        self.payload_activation_hold_s,
                    )
            elif not (self.payload_prelift_confirmation_waiting
                      and self._payload_prelift_motion_state(handoff_state, now) is not None):
                self.payload_ready_since = None
        if transport_phase_active and self.payload_transport_active:
            payload_ready = self._payload_ready(now)
            if payload_ready:
                transport_overrides = self._transport_overrides(now)
                if transport_overrides:
                    self.payload_fault_since = None
                    self.last_transport_overrides = transport_overrides
                elif self.payload_fault_since is None:
                    self.payload_fault_since = now
            elif self.payload_fault_since is None:
                self.payload_fault_since = now

            complete_cache = all(self._finite_transport_override(
                self.last_transport_overrides.get(id(vehicle), {})) for vehicle in self.vehicles)
            fault_hold_s = min(self.payload_hold_s, self.payload_emergency_land_after_s)
            if (not transport_overrides and complete_cache and
                    self.payload_fault_since is not None and
                    now - self.payload_fault_since <= fault_hold_s):
                # Keep the last complete MATLAB force command across a short
                # load-mocap gap; this avoids a thrust step at the exact moment
                # a single rigid-body frame is lost.
                transport_overrides = {
                    key: dict(value, transport_hold_integral_learning=False,
                              transport_hold_integral_freeze_reason="payload_cached_hold",
                              transport_hold_integral_force_dot=np.zeros(3),
                              transport_full_pid_learning=False,
                              transport_full_pid_freeze_reason="payload_cached_hold",
                              transport_full_pid_tick=float(now),
                              transport_full_pid_force_hold=True,
                              transport_full_pid_weight_dot=0.)
                    for key, value in self.last_transport_overrides.items()}
                self._refresh_cached_attitude_trim(now, transport_overrides, "payload_cached_hold")
            elif not transport_overrides and self.payload_fault_since is not None:
                # An expired/incomplete cooperative cache cannot become an
                # ordinary aircraft hover tick while the load is attached.
                # Select the existing residual-supported descent immediately.
                self.payload_transport_active = False
                self._begin_payload_emergency_landing(
                    now, "load 状态持续失效 %.3f s" %
                    (now - self.payload_fault_since)
                )
                self.last_transport_overrides = {}
                self.payload_fault_since = None
        if self._check_payload_physical_safety(now):
            transport_overrides = (self._transport_overrides(now)
                                   if self.payload_transport_active and self.payload_controlled_exit_reason else {})
            takeup_overrides = {}
        # Fault guards own this tick first. A planned validation stop must not
        # replace a fault reason, and no cached cooperative output may survive.
        if self._check_payload_flight_validation(now):
            # Keep the taut-load controller while lowering the payload. Its
            # reference is rebuilt for landing, never reuse the old lift tick.
            transport_overrides = self._transport_overrides(now)
            self.last_transport_overrides = transport_overrides
            takeup_overrides = {}
        # The fleet tick's ``now`` is captured before path publication and
        # before the per-vehicle callbacks.  Use one refreshed timestamp for
        # all callbacks so a freshly received EKF sample cannot appear to be
        # from the future merely because an earlier callback spent time
        # publishing a growing Path message.
        vehicle_control_now = max(float(now), float(time.monotonic()))
        with self.payload_lock:
            payload_source_diagnostics = deepcopy(self.latest_payload_state)
        for vehicle_index, vehicle in enumerate(self.vehicles):
            vehicle.set_fleet_gate(True)
            target_override = transport_overrides.get(id(vehicle))
            if target_override is None:
                target_override = takeup_overrides.get(id(vehicle))
            emergency_override = self._payload_emergency_override(vehicle, now)
            if emergency_override:
                target_override = dict(target_override or {})
                target_override.update(emergency_override)
            if (self.payload_validation_landing_stage == "vehicle_descent"
                    and vehicle.trajectory is not None
                    and vehicle.trajectory.phase in ("landing", "landing_settle", "landed")):
                # The ropes can remain taut briefly after payload touchdown.
                # Keep the demonstrated TAKEUP XY/attitude loop while each
                # aircraft descends; weak ordinary gains/5-degree tilt let
                # rope tension collapse the formation during this transition.
                target_override = dict(target_override or {})
                target_override.update(
                    independent_mode=True, transport_mode=False,
                    independent_position_gain_scale=self.payload_prelift_parameters["position_gain_scale"],
                    independent_velocity_gain_scale=self.payload_prelift_parameters["velocity_gain_scale"],
                    max_tilt_rad=self.payload_prelift_parameters["max_tilt_rad"],
                )
                target_override.update(self.payload_landing_integral_parameters)
                target_override["landing_vertical_integral_active"] = True
            if self.transport_mode == "slung_load":
                target_override = dict(target_override or {})
                target_override.update({
                    "link_length_nominal_m": self.payload_nominal_link_lengths[vehicle_index],
                    "link_length_effective_m": self.slung_controller.config.link_lengths_m[vehicle_index],
                    "link_length_measured_m": self.payload_link_measured_distances[vehicle_index],
                    "link_calibration_enabled": self.payload_link_calibration_enabled,
                    "link_calibration_applied": self.payload_link_calibration_applied,
                    "link_calibration_status": (
                        "disabled" if not self.payload_link_calibration_enabled else
                        "awaiting_current_frame" if self.payload_link_calibration_pending is not None else
                        self.payload_link_calibrator.status),
                    "link_calibration_reason": self.payload_link_calibrator.reason,
                    "link_calibration_samples": self.payload_link_calibrator.sample_count,
                    "link_calibration_window_s": self.payload_link_calibrator.window_elapsed_s,
                    "link_calibration_spread_m": self.payload_link_calibrator.spread_m[vehicle_index],
                    "payload_ground_clearance_m": self.payload_ground_clearance_m,
                    "payload_prelift_clearance_target_m": self.payload_prelift_clearance_m,
                    "payload_prelift_ready": self.payload_prelift_ready,
                    "payload_minimum_vehicle_distance_m": self.payload_minimum_vehicle_distance_m,
                    "payload_predicted_vehicle_distance_m": self.payload_predicted_vehicle_distance_m,
                    "payload_safety_reason": self.payload_safety_reason,
                    "flight_validation_enabled": self.payload_flight_validation_enabled,
                    "flight_validation_landing_started": self.payload_validation_landing_started_time is not None,
                    "flight_validation_landing_stage": self.payload_validation_landing_stage,
                    "payload_ground_reference_z_m": self.payload_ground_center_z,
                    "payload_landing_duration_s": (self.payload_landing_profile.duration
                                                  if self.payload_landing_profile is not None else math.nan),
                    "payload_landing_touchdown_confirmed": (self.payload_touchdown_detector is not None
                                                            and self.payload_touchdown_detector.detected),
                    "payload_landing_unloading": self.payload_unload_start_time is not None,
                    "payload_landing_planning_failure_count": self.payload_landing_failure_count,
                    "payload_landing_planning_retry_count": self.payload_landing_retry_count,
                    "payload_landing_planning_failure_reason": self.payload_landing_failure_reason,
                    "payload_landing_planning_fallback": (self.payload_landing_retry_since is not None
                        or self.payload_validation_landing_stage == "planning_fault_descent"),
                    "flight_validation_elapsed_s": (
                        max(0.0, (self.payload_validation_landing_started_time
                                  if self.payload_validation_landing_started_time is not None else now)
                            - (self.payload_validation_handoff_time
                               if self.payload_validation_handoff_time is not None
                               else self.payload_transport_start_time))
                        if (self.payload_validation_handoff_time is not None
                            or self.payload_transport_start_time is not None) else math.nan),
                })
            if self.transport_mode == "slung_load" and self.payload_last_takeup_reference is not None:
                target_override = dict(target_override or {})
                target_override["payload_prelift_failure_reason"] = (
                    self.payload_last_takeup_reference["reason"]
                )
                target_override["payload_prelift_stage"] = (
                    self.payload_last_takeup_reference["stage"]
                    if not self.payload_transport_active else "transport"
                )
            control_snapshot = (target_override or {}).get("_payload_control_state")
            vehicle.payload_source_diagnostics = payload_source_diagnostics
            vehicle.payload_control_time = float(
                (target_override or {}).get("_payload_control_time", math.nan))
            vehicle.payload_control_reused = bool(control_snapshot is not None
                and vehicle.payload_control_time != float(now))
            vehicle.set_payload_diagnostics(deepcopy(
                control_snapshot if control_snapshot is not None else payload_source_diagnostics))
            vehicle._timer_callback(
                None,
                now=vehicle_control_now,
                transport_override=target_override,
            )
            self.fleet_tick_completed_vehicles.add(id(vehicle))
        self._log_terminal_mission_stage()
        self.logger.flush()

    def _shutdown(self):
        for vehicle in self.vehicles:
            vehicle.set_global_abort("节点关闭")
            vehicle._publish_zero()
        self.logger.close()
