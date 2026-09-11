#!/usr/bin/python3
"""CTBR 圆周任务参考轨迹。

本模块只生成期望状态，不订阅 ROS 话题，也不发送控制命令。它把任务流程和几何
参考轨迹与 ``ctbr_controller.py`` 中的控制律分开，便于以后替换为八字、航点或其他
任务而不改动 CTBR 外环。

圆周主体使用五次角度平滑和解析运动学：

``p = c + r [cos(theta), sin(theta), 0]``

其中 ``theta`` 在整圈范围内按五次 ``smoothstep`` 推进，再由链式法则计算速度、加速度
和 jerk；机头 yaw 取从当前位置指向圆心的方向，并同时提供 ``yaw_rate``。这样圆周起止
点的速度和加速度都为零，可与入圆和终点悬停平滑衔接。所有长度单位为 m，速度为 m/s，
加速度为 m/s^2，jerk 为 m/s^3，角度为 rad。
"""

import math
from dataclasses import dataclass

import numpy as np


# The quintic smoothstep derivative reaches 15/8 at progress 0.5.
_SMOOTHSTEP5_MAX_DERIVATIVE = 15.0 / 8.0


def _as_vector(value, name, size=3):
    """把输入转成指定长度的有限浮点向量。"""
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.size != size or not np.all(np.isfinite(vector)):
        raise ValueError("%s 必须是 %d 个有限数值" % (name, size))
    return vector.copy()


def _wrap_angle(angle):
    """将角度归一到 [-pi, pi]。"""
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _smoothstep5(elapsed, duration):
    """返回五次 smoothstep 的位置、速度和加速度系数。"""
    return _smoothstep5_with_jerk(elapsed, duration)[:3]


def _smoothstep5_with_jerk(elapsed, duration):
    """返回五次 smoothstep 的位置、速度、加速度和 jerk 系数。"""
    duration = float(duration)
    progress = float(np.clip(float(elapsed) / duration, 0.0, 1.0))
    p2 = progress * progress
    p3 = p2 * progress
    position = 10.0 * p3 - 15.0 * p3 * progress + 6.0 * p3 * p2
    velocity = (
        30.0 * p2 - 60.0 * p3 + 30.0 * p3 * progress
    ) / duration
    acceleration = (
        60.0 * progress - 180.0 * p2 + 120.0 * p3
    ) / (duration * duration)
    jerk = (
        60.0 - 360.0 * progress + 360.0 * p2
    ) / (duration * duration * duration)
    return position, velocity, acceleration, jerk


@dataclass
class CircularTrajectoryConfig:
    """圆周任务配置。

    ``circle_center_offset_xy`` 是相对第一帧有效位姿的 x/y 偏移。圆心由该偏移和
    起始位置计算；若希望飞机从起飞点直接位于圆周上，可把圆心放在起飞点前方一个
    半径，并把 ``circle_start_angle_rad`` 设为 pi。
    """

    reference_hold_s: float
    takeoff_height_m: float
    takeoff_duration_s: float
    takeoff_settle_s: float
    takeoff_altitude_tolerance_m: float
    takeoff_vertical_velocity_tolerance_mps: float
    circle_center_offset_xy: object
    circle_radius_m: float
    circle_revolutions: float
    # 整圈五次角度轨迹允许的峰值角速度，rad/s。
    circle_angular_speed_radps: float
    circle_start_angle_rad: float
    entry_duration_s: float
    # 保留旧配置字段以兼容现有调用方；整圈 smoothstep 不再使用该值。
    circle_ramp_duration_s: float
    final_hover_s: float
    landing_duration_s: float
    landing_max_speed_mps: float
    landing_altitude_tolerance_m: float
    landing_vertical_velocity_tolerance_mps: float
    landing_settle_s: float
    # 落地确认的短暂失稳容忍时间。Nokov 差分速度可能偶发单帧尖峰，不能因此
    # 立刻重置 1 s 确认计时；只有超限持续达到该时间才回到 landing 阶段。
    landing_condition_grace_s: float
    takeoff_max_tilt_rad: float
    circle_max_tilt_rad: float
    landing_max_tilt_rad: float
    takeoff_min_collective_thrust: float


class CircularFlightTrajectory:
    """一圈水平圆周任务的参考生成器和安全阶段机。

    通常由调用方用第一帧有效 Nokov 状态调用 ``reset``；如果尚未调用，
    ``evaluate(state, now)`` 也会用其第一帧有效状态自动锁定起点。``state`` 至少
    包含 ``position`` 和 ``velocity``，可选 ``valid`` 字段为 ``False`` 时不会满足
    落地门控。起飞阶段结束后继续使用名义起飞点 x/y，并持续给出精确起飞高度 z
    参考；该校正阶段按固定时间推进，不再因高度/速度误差反复重放起飞曲线。落地阶段
    仍由实际状态确认，避免未接触地面时释放推力。
    """

    def __init__(self, config):
        if config is None:
            raise TypeError("CircularFlightTrajectory 需要显式传入配置")
        self.config = config
        self._validate_config()
        self.reset()

    def _validate_config(self):
        cfg = self.config
        positive = (
            ("takeoff_height_m", cfg.takeoff_height_m),
            ("takeoff_duration_s", cfg.takeoff_duration_s),
            ("circle_radius_m", cfg.circle_radius_m),
            ("circle_angular_speed_radps", cfg.circle_angular_speed_radps),
            ("entry_duration_s", cfg.entry_duration_s),
            ("landing_duration_s", cfg.landing_duration_s),
            ("landing_max_speed_mps", cfg.landing_max_speed_mps),
            ("takeoff_altitude_tolerance_m", cfg.takeoff_altitude_tolerance_m),
            ("takeoff_vertical_velocity_tolerance_mps",
             cfg.takeoff_vertical_velocity_tolerance_mps),
            ("landing_altitude_tolerance_m", cfg.landing_altitude_tolerance_m),
            ("landing_vertical_velocity_tolerance_mps",
             cfg.landing_vertical_velocity_tolerance_mps),
            ("landing_condition_grace_s", cfg.landing_condition_grace_s),
        )
        for name, value in positive:
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError("%s 必须是正数" % name)
        nonnegative = (
            ("reference_hold_s", cfg.reference_hold_s),
            ("takeoff_settle_s", cfg.takeoff_settle_s),
            ("circle_revolutions", cfg.circle_revolutions),
            ("final_hover_s", cfg.final_hover_s),
            ("landing_settle_s", cfg.landing_settle_s),
            ("takeoff_min_collective_thrust", cfg.takeoff_min_collective_thrust),
            # Deprecated compatibility field; it no longer controls the circle profile.
            ("circle_ramp_duration_s", cfg.circle_ramp_duration_s),
        )
        for name, value in nonnegative:
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError("%s 不能为负数" % name)
        if cfg.circle_revolutions <= 0.0:
            raise ValueError("circle_revolutions 必须大于 0")
        for name in ("takeoff_max_tilt_rad", "circle_max_tilt_rad", "landing_max_tilt_rad"):
            value = float(getattr(cfg, name))
            if not math.isfinite(value) or value <= 0.0 or value >= math.pi / 2.0:
                raise ValueError("%s 必须在 (0, pi/2) 内" % name)
        if cfg.circle_center_offset_xy is not None:
            _as_vector(cfg.circle_center_offset_xy, "circle_center_offset_xy", size=2)

    def reset(self, start_position=None, start_yaw=0.0, now=0.0):
        """锁定第一帧起点并重置任务阶段。"""
        self.initialized = start_position is not None
        self.start_time = float(now)
        self.start_yaw = float(start_yaw)
        self.start_position = (
            _as_vector(start_position, "start_position") if self.initialized else None
        )
        self.takeoff_position = None
        self.height_correction_position = None
        self.circle_entry_start_position = None
        self.circle_center = None
        self.circle_start_position = None
        self.landing_target_position = None
        self.active_landing_duration_s = None
        self.circle_start_angle = float(self.config.circle_start_angle_rad)
        total_angle = 2.0 * math.pi * float(self.config.circle_revolutions)
        # The derivative of 10u^3 - 15u^4 + 6u^5 peaks at 15/8.  Interpret
        # circle_angular_speed_radps as the requested peak, so the configured
        # limit is not exceeded during the smoothstep motion.
        omega_peak = float(self.config.circle_angular_speed_radps)
        self.circle_duration_s = (
            _SMOOTHSTEP5_MAX_DERIVATIVE * total_angle / omega_peak
        )
        self.phase = "waiting_for_reset"
        self.phase_start_time = float(now)
        self.takeoff_arrival_time = None
        self.landing_arrival_time = None
        self.landing_unstable_since = None
        # 动捕暂时失效时，控制节点会暂停而非继续消耗轨迹时间。恢复后把所有
        # 与时间相关的锚点整体后移，避免参考在不可观测期间跳到后续阶段。
        self.paused_at = None
        self.abort_position = None
        self._events = []
        if self.initialized:
            self._initialize_geometry()
            self.phase = "pre_takeoff_hold"
            self._emit(
                "已锁定起点 [%.3f, %.3f, %.3f] m；保持 %.1f s 后上升 %.3f m；"
                "圆心 [%.3f, %.3f] m，半径 %.3f m，机头指向圆心。" % (
                    self.start_position[0], self.start_position[1], self.start_position[2],
                    self.config.reference_hold_s, self.config.takeoff_height_m,
                    self.circle_center[0], self.circle_center[1], self.config.circle_radius_m,
                )
            )

    def _initialize_geometry(self):
        cfg = self.config
        self.takeoff_position = self.start_position.copy()
        self.takeoff_position[2] += float(cfg.takeoff_height_m)
        if cfg.circle_center_offset_xy is not None:
            center_xy = self.start_position[:2] + _as_vector(
                cfg.circle_center_offset_xy, "circle_center_offset_xy", size=2
            )
        else:
            center_xy = self.start_position[:2].copy()
        self.circle_center = np.array(
            [center_xy[0], center_xy[1], self.takeoff_position[2]], dtype=float
        )
        # Keep the nominal takeoff x/y through height correction and circle
        # entry.  The measured horizontal drift must not redefine the circle
        # center or introduce a reference bump before the circle.
        self.height_correction_position = self.takeoff_position.copy()
        self.circle_entry_start_position = self.takeoff_position.copy()
        self.circle_start_position = self.circle_center + np.array([
            cfg.circle_radius_m * math.cos(self.circle_start_angle),
            cfg.circle_radius_m * math.sin(self.circle_start_angle), 0.0,
        ])
        # 一圈结束后保持最终圆周点的 x/y，只把 z 降回起飞前高度。
        self.landing_target_position = self.circle_start_position.copy()
        self.landing_target_position[2] = self.start_position[2]
        self.active_landing_duration_s = max(
            float(cfg.landing_duration_s),
            1.875 * abs(self.takeoff_position[2] - self.start_position[2]) /
            float(cfg.landing_max_speed_mps),
        )

    def _emit(self, message):
        self._events.append(str(message))

    def pop_events(self):
        """返回并清空阶段事件，调用方可以用 rospy 或其他日志系统输出。"""
        events = self._events
        self._events = []
        return events

    def _transition(self, phase, now, message=None):
        """阶段真正变化时只设置一次新起始时刻。"""
        if self.phase == phase:
            return
        self.phase = phase
        self.phase_start_time = float(now)
        if message:
            self._emit(message)

    @property
    def is_paused(self):
        """轨迹是否正因动捕暂时失效而冻结计时。"""
        return self.paused_at is not None

    def pause(self, now):
        """冻结当前任务阶段的计时；重复调用不会重复计时。"""
        if self.initialized and self.paused_at is None and self.phase not in ("landed", "aborted"):
            self.paused_at = float(now)

    def resume(self, now):
        """恢复已暂停的轨迹，并返回此次暂停时长。"""
        if self.paused_at is None:
            return 0.0
        pause_duration = max(0.0, float(now) - self.paused_at)
        self.start_time += pause_duration
        self.phase_start_time += pause_duration
        if self.takeoff_arrival_time is not None:
            self.takeoff_arrival_time += pause_duration
        if self.landing_arrival_time is not None:
            self.landing_arrival_time += pause_duration
        self.paused_at = None
        return pause_duration

    def abort(self, now, reason, position=None):
        """锁定安全中止状态，之后只能输出零推力目标。

        长时间丢失动捕后，飞机可能已经偏离原参考位置。自动恢复并继续旧任务会产生
        大的位置误差和不可预测的 CTBR 输出，因此需要人工检查后重启控制器。
        """
        if self.phase == "aborted":
            return
        if position is None:
            position = self.start_position
        self.abort_position = _as_vector(position, "abort_position")
        self.paused_at = None
        self._transition("aborted", now, "轨迹安全中止：%s；保持零 CTBR 推力。" % reason)

    def _static_target(
            self, position, yaw, phase, max_tilt_rad=None, min_thrust=0.0,
            zero_output=False):
        target = {
            "position": np.asarray(position, dtype=float).copy(),
            "velocity": np.zeros(3),
            "acceleration": np.zeros(3),
            "jerk": np.zeros(3),
            "yaw": float(yaw),
            "yaw_rate": 0.0,
            "flight_phase": phase,
            "max_tilt_rad": (
                float(self.config.circle_max_tilt_rad)
                if max_tilt_rad is None else float(max_tilt_rad)
            ),
            "min_collective_thrust": float(min_thrust),
        }
        if zero_output:
            target["zero_output"] = True
        return target

    def _takeoff_target(self, elapsed, state):
        cfg = self.config
        position_scale, velocity_scale, acceleration_scale, jerk_scale = _smoothstep5_with_jerk(
            elapsed, cfg.takeoff_duration_s
        )
        displacement = self.takeoff_position - self.start_position
        target_position = self.start_position + position_scale * displacement
        return {
            "position": target_position,
            "velocity": velocity_scale * displacement,
            "acceleration": acceleration_scale * displacement,
            "jerk": jerk_scale * displacement,
            "yaw": self.start_yaw,
            "yaw_rate": 0.0,
            "flight_phase": "takeoff",
            "max_tilt_rad": cfg.takeoff_max_tilt_rad,
            "min_collective_thrust": (
                cfg.takeoff_min_collective_thrust
                if elapsed > 0.0 and self._takeoff_requires_support(state) else 0.0
            ),
        }

    def _inward_yaw(self, position):
        radial = self.circle_center[:2] - np.asarray(position, dtype=float)[:2]
        return math.atan2(float(radial[1]), float(radial[0]))

    def _entry_target(self, elapsed):
        cfg = self.config
        position_scale, velocity_scale, acceleration_scale, jerk_scale = _smoothstep5_with_jerk(
            elapsed, cfg.entry_duration_s
        )
        entry_start = (
            self.takeoff_position
            if self.circle_entry_start_position is None
            else self.circle_entry_start_position
        )
        displacement = self.circle_start_position - entry_start
        target_yaw = self._inward_yaw(self.circle_start_position)
        yaw_displacement = _wrap_angle(target_yaw - self.start_yaw)
        return {
            "position": entry_start + position_scale * displacement,
            "velocity": velocity_scale * displacement,
            "acceleration": acceleration_scale * displacement,
            "jerk": jerk_scale * displacement,
            "yaw": _wrap_angle(self.start_yaw + position_scale * yaw_displacement),
            "yaw_rate": velocity_scale * yaw_displacement,
            "flight_phase": "circle_entry",
            "max_tilt_rad": cfg.circle_max_tilt_rad,
            "min_collective_thrust": 0.0,
        }

    def _set_height_correction_reference(self):
        """Keep nominal takeoff x/y while correcting z to the exact height."""
        self.height_correction_position = self.takeoff_position.copy()
        self.circle_entry_start_position = self.height_correction_position.copy()

    def _height_correction_target(self, state):
        """Command exact z0 + takeoff height without a height-error gate."""
        return self._static_target(
            self.height_correction_position,
            self.start_yaw,
            "height_correction",
            max_tilt_rad=self.config.takeoff_max_tilt_rad,
            min_thrust=(
                self.config.takeoff_min_collective_thrust
                if self._takeoff_requires_support(state) else 0.0
            ),
        )

    def _circle_kinematics(
            self, angle, angular_velocity, angular_acceleration, phase,
            angular_jerk=0.0):
        cfg = self.config
        radius = float(cfg.circle_radius_m)
        radial = np.array([math.cos(angle), math.sin(angle)])
        tangent = np.array([-math.sin(angle), math.cos(angle)])
        position = self.circle_center + np.array([
            radius * math.cos(angle), radius * math.sin(angle), 0.0
        ])
        velocity = np.array([
            -radius * angular_velocity * math.sin(angle),
            radius * angular_velocity * math.cos(angle), 0.0,
        ])
        acceleration = np.array([
            -radius * (angular_velocity * angular_velocity * math.cos(angle)
                       + angular_acceleration * math.sin(angle)),
            radius * (-angular_velocity * angular_velocity * math.sin(angle)
                      + angular_acceleration * math.cos(angle)),
            0.0,
        ])
        jerk_xy = radius * (
            -3.0 * angular_velocity * angular_acceleration * radial
            + (angular_jerk - angular_velocity ** 3) * tangent
        )
        return {
            "position": position,
            "velocity": velocity,
            "acceleration": acceleration,
            "jerk": np.array([jerk_xy[0], jerk_xy[1], 0.0]),
            "yaw": self._inward_yaw(position),
            "yaw_rate": angular_velocity,
            "flight_phase": phase,
            "max_tilt_rad": cfg.circle_max_tilt_rad,
            "min_collective_thrust": 0.0,
        }

    def _circle_target(self, elapsed):
        """Return a point on the whole-circle quintic angular reference.

        The Cartesian path remains circular; only its unwrapped angular phase is
        smoothed.  ``_smoothstep5`` returns derivatives with respect to time,
        so multiplying by the total angle gives angular velocity and acceleration.
        """
        total_angle = 2.0 * math.pi * float(self.config.circle_revolutions)
        (
            angle_scale,
            angular_velocity_scale,
            angular_acceleration_scale,
            angular_jerk_scale,
        ) = _smoothstep5_with_jerk(
            elapsed, self.circle_duration_s
        )
        angle = self.circle_start_angle + total_angle * angle_scale
        angular_velocity = total_angle * angular_velocity_scale
        angular_acceleration = total_angle * angular_acceleration_scale
        angular_jerk = total_angle * angular_jerk_scale
        return self._circle_kinematics(
            angle, angular_velocity, angular_acceleration, "circle", angular_jerk
        )

    def _circle_endpoint_target(self):
        """Return the exact end point used by hover and landing references."""
        return self._circle_target(self.circle_duration_s)

    def _landing_target(self, elapsed):
        cfg = self.config
        duration = float(self.active_landing_duration_s)
        position_scale, velocity_scale, acceleration_scale, jerk_scale = _smoothstep5_with_jerk(
            elapsed, duration
        )
        final_circle = self._circle_endpoint_target()
        displacement = self.landing_target_position - final_circle["position"]
        target = {
            "position": final_circle["position"] + position_scale * displacement,
            "velocity": velocity_scale * displacement,
            "acceleration": acceleration_scale * displacement,
            "jerk": jerk_scale * displacement,
            "yaw": final_circle["yaw"],
            "yaw_rate": 0.0,
            "flight_phase": "landing",
            "max_tilt_rad": cfg.landing_max_tilt_rad,
            "min_collective_thrust": 0.0,
        }
        # 参考轨迹到达地面后必须释放推力，让飞机真正接触地面。若仍调用
        # 几何控制器，它会为 z=落地点补偿重力，把已经落地的飞机再次托起，
        # 造成 landing_settle 反复进入/退出。
        if elapsed >= duration:
            target["zero_output"] = True
        return target

    @staticmethod
    def _state_value(state, key):
        if isinstance(state, dict):
            return state.get(key)
        return getattr(state, key, None)

    def _state_is_valid(self, state):
        """没有 ``valid`` 字段的离线状态视为有效，ROS 状态显式失效时拒绝放行。"""
        valid = self._state_value(state, "valid")
        return valid is None or bool(valid)

    def _state_yaw(self, state):
        """自动初始化时使用可选 yaw；控制器显式 reset 时会传入更精确的 yaw。"""
        yaw = self._state_value(state, "yaw")
        if yaw is None or not math.isfinite(float(yaw)):
            return 0.0
        return float(yaw)

    def _takeoff_requires_support(self, state):
        """只在实测高度仍明显低于目标时施加起飞最小总推力。"""
        if not self._state_is_valid(state):
            return False
        position = self._state_value(state, "position")
        if position is None:
            return False
        try:
            position = _as_vector(position, "state.position")
        except ValueError:
            return False
        return float(position[2]) < (
            float(self.takeoff_position[2])
            - float(self.config.takeoff_altitude_tolerance_m)
        )

    def _takeoff_conditions_met(self, state):
        """保留旧门控检查接口，供离线调用方诊断起飞状态。"""
        if not self._state_is_valid(state):
            return False
        position = self._state_value(state, "position")
        velocity = self._state_value(state, "velocity")
        if position is None or velocity is None:
            return False
        try:
            position = _as_vector(position, "state.position")
            velocity = _as_vector(velocity, "state.velocity")
        except ValueError:
            return False
        return (
            abs(float(position[2]) - float(self.takeoff_position[2])) <= float(
                self.config.takeoff_altitude_tolerance_m
            )
            and abs(float(velocity[2])) <= float(
                self.config.takeoff_vertical_velocity_tolerance_mps
            )
        )

    def _landing_conditions_met(self, state):
        if not self._state_is_valid(state):
            return False
        position = self._state_value(state, "position")
        velocity = self._state_value(state, "velocity")
        if position is None or velocity is None:
            return False
        try:
            position = _as_vector(position, "state.position")
            velocity = _as_vector(velocity, "state.velocity")
        except ValueError:
            return False
        return (
            abs(float(position[2]) - float(self.landing_target_position[2]))
            <= float(self.config.landing_altitude_tolerance_m)
            and abs(float(velocity[2]))
            <= float(self.config.landing_vertical_velocity_tolerance_mps)
        )

    def _landed_target(self):
        final_circle = self._circle_endpoint_target()
        return self._static_target(
            self.landing_target_position, final_circle["yaw"], "landed",
            max_tilt_rad=self.config.landing_max_tilt_rad,
        )

    def evaluate(self, state, now=None):
        """生成当前参考目标并推进阶段状态。"""
        if now is None:
            raise TypeError("evaluate 需要 evaluate(state, now)")
        now = float(now)
        if not self.initialized:
            if not self._state_is_valid(state):
                raise RuntimeError("第一帧状态无效，不能锁定圆周轨迹起点")
            start_position = self._state_value(state, "position")
            if start_position is None:
                raise RuntimeError("第一帧状态缺少 position，不能锁定圆周轨迹起点")
            self.reset(start_position, self._state_yaw(state), now)
        elapsed = max(0.0, now - self.phase_start_time)
        cfg = self.config

        if self.phase == "pre_takeoff_hold":
            target = self._static_target(
                self.start_position, self.start_yaw, "pre_takeoff_hold",
                max_tilt_rad=cfg.takeoff_max_tilt_rad,
                # 起飞前保持只为锁定起点和等待无线电零包，不应发送重力补偿而让
                # 飞机在任务正式开始前离地。
                zero_output=True,
            )
            if elapsed >= cfg.reference_hold_s:
                self._transition(
                    "takeoff", now,
                    "起飞前保持完成，开始垂直上升至 z=%.3f m。" % self.takeoff_position[2],
                )
                return self.evaluate(state, now)
            return target

        if self.phase == "takeoff":
            target = self._takeoff_target(elapsed, state)
            if elapsed >= cfg.takeoff_duration_s:
                # Do not use the coarse altitude/vertical-speed tolerances as
                # a gate.  The next phase keeps sending the exact z target so
                # the controller can correct height while the task continues.
                self._set_height_correction_reference()
                self.takeoff_arrival_time = now
                self._transition(
                    "height_correction", now,
                    "起飞参考完成，保持名义起飞点 x/y 并在飞行中继续跟踪 z=%.3f m。"
                    % self.takeoff_position[2],
                )
                return self.evaluate(state, now)
            return target

        if self.phase == "height_correction":
            # ``takeoff_settle_s`` remains a fixed correction interval for
            # compatibility with existing launch files; it is not reset by
            # measured altitude or vertical velocity excursions.
            target = self._height_correction_target(state)
            if elapsed >= cfg.takeoff_settle_s:
                self._transition(
                    "circle_entry", now,
                    "高度校正参考完成，开始平滑进入半径 %.3f m 的圆周。"
                    % cfg.circle_radius_m,
                )
                return self.evaluate(state, now)
            return target

        if self.phase == "circle_entry":
            target = self._entry_target(elapsed)
            if elapsed >= cfg.entry_duration_s:
                self._transition("circle", now, "已进入圆周起点，开始五次角度平滑圆周轨迹。")
                return self.evaluate(state, now)
            return target

        if self.phase == "circle":
            target = self._circle_target(elapsed)
            if elapsed >= self.circle_duration_s:
                self._transition(
                    "final_hover", now,
                    "圆周完成，开始 %.1f s 最终点悬停。" % cfg.final_hover_s,
                )
                return self.evaluate(state, now)
            return target

        final_circle = self._circle_endpoint_target()
        if self.phase == "final_hover":
            target = self._static_target(
                final_circle["position"], final_circle["yaw"], "final_hover",
                max_tilt_rad=cfg.circle_max_tilt_rad,
            )
            if elapsed >= cfg.final_hover_s:
                self._transition(
                    "landing", now,
                    "最终点悬停完成，开始垂直降落至 z=%.3f m，参考时间 %.1f s，最大参考速度 %.2f m/s。"
                    % (self.landing_target_position[2], self.active_landing_duration_s,
                       cfg.landing_max_speed_mps),
                )
                return self.evaluate(state, now)
            return target

        if self.phase == "landing":
            target = self._landing_target(elapsed)
            if elapsed >= self.active_landing_duration_s and self._landing_conditions_met(state):
                self.landing_arrival_time = now
                self.landing_unstable_since = None
                self._transition(
                    "landing_settle", now,
                    "已达到落地高度，开始 %.1f s 落地稳定确认。" % cfg.landing_settle_s,
                )
                return self.evaluate(state, now)
            return target

        if self.phase == "landing_settle":
            target = self._static_target(
                self.landing_target_position, final_circle["yaw"], "landing_settle",
                max_tilt_rad=cfg.landing_max_tilt_rad,
                zero_output=True,
            )
            if not self._landing_conditions_met(state):
                # 对动捕差分的短暂尖峰留出容忍时间。确认期间仍保持地面高度
                # 参考；若超限持续超过 grace，才认为飞机确实未稳定并重启确认。
                if self.landing_unstable_since is None:
                    self.landing_unstable_since = now
                if now - self.landing_unstable_since >= float(
                        self.config.landing_condition_grace_s):
                    self.landing_arrival_time = None
                    self.landing_unstable_since = None
                    self._transition(
                        "landing", now,
                        "落地高度或垂直速度持续超限，继续保持落地参考并重新确认。",
                    )
                    self.phase_start_time = now - float(self.active_landing_duration_s)
                return target
            # 条件恢复后清除失稳计时，但保留原确认起点；短暂尖峰不打断确认。
            self.landing_unstable_since = None
            if self.landing_arrival_time is None:
                self.landing_arrival_time = now
            if now - self.landing_arrival_time >= cfg.landing_settle_s:
                self._transition(
                    "landed", now,
                    "已确认降落至 z=%.3f m，停止 CTBR 推力输出。"
                    % self.landing_target_position[2],
                )
                return self.evaluate(state, now)
            return target

        if self.phase == "landed":
            return self._landed_target()
        if self.phase == "aborted":
            return self._static_target(
                self.abort_position, self.start_yaw, "aborted",
                max_tilt_rad=cfg.landing_max_tilt_rad,
                zero_output=True,
            )
        raise RuntimeError("未知轨迹阶段: %s" % self.phase)

    @property
    def duration_s(self):
        """按计划计算的完整参考时长，不包含实际门控额外等待。"""
        return (
            float(self.config.reference_hold_s)
            + float(self.config.takeoff_duration_s)
            + float(self.config.takeoff_settle_s)
            + float(self.config.entry_duration_s)
            + float(self.circle_duration_s)
            + float(self.config.final_hover_s)
            + float(self.active_landing_duration_s or self.config.landing_duration_s)
        )
