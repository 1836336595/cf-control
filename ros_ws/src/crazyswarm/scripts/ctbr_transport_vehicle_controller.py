"""Transport aircraft force composition, native feedback, and rate filtering.

The payload's cooperative controller supplies force overrides; this module
retains the aircraft hold feedback and continuous independent/transport blend.
"""

import math
from dataclasses import dataclass

import numpy as np

from ctbr_control_math import as_vector, clamp_vector
from native_position_pid import (position_integral_step, position_pid_feedback,
                                 validate_integral_velocity_weight)


@dataclass
class VehicleForceCommand:
    """Force composition and diagnostics consumed by the geometric facade."""
    raw_desired_force: object
    raw_desired_force_dot: object
    transport_blend: object
    hold_feedback_limit_active: object
    transport_integral_force: object
    transport_integral_force_dot: object
    transport_full_pid_force: object
    transport_full_pid_force_dot: object
    transport_full_pid_pd_force: object
    transport_full_pid_pd_force_dot: object
    native_pid_info: object
    hold_acceleration: object
    transport_blend_dot: object


class TransportVehicleController:
    """Pure transport formulas sharing persistent state on their facade."""

    def reset_transport_position_pid(self):
        """Clear the transport-only native I without losing independent trim."""
        self.transport_position_integral = np.zeros(3)
        self.transport_full_pid_force_previous = np.zeros(3)
        self._transport_position_pid_tick = None
        self._transport_position_pid_signature = None
        self._transport_position_pid_cached = None
        self._transport_position_pid_reset_tick = None

    def _native_transport_feedback(self, state, target, dt, derivatives_ready,
                                   measured_velocity, position_gain,
                                   velocity_gain, integral_gain):
        """Evaluate shared native feedback once per integral/budget time step."""
        tick = target.get("transport_full_pid_tick")
        integral_velocity_weight = validate_integral_velocity_weight(
            target.get("transport_full_pid_integral_velocity_weight", 1.0))
        if (target.get("transport_full_pid_reset", False) and
                (tick is None or tick != self._transport_position_pid_reset_tick)):
            self.reset_transport_position_pid()
            self._transport_position_pid_reset_tick = tick
        same_tick = tick is not None and tick == self._transport_position_pid_tick
        ep = state["position"] - np.asarray(
            target.get("position_hold_position", target["position"]), dtype=float)
        ev = (measured_velocity - np.asarray(
            target.get("position_hold_velocity", target["velocity"]), dtype=float)
              if derivatives_ready else np.zeros(3))
        reference_acceleration = np.asarray(target.get("position_hold_acceleration",
            target.get("acceleration", np.zeros(3))), dtype=float)
        acceleration = np.asarray(state.get("control_acceleration",
            state.get("filtered_acceleration", state.get("acceleration", reference_acceleration))), dtype=float)
        if not derivatives_ready or not np.all(np.isfinite(acceleration)):
            acceleration = reference_acceleration.copy()
        beta = float(np.clip(target.get("transport_blend", 1.), 0., 1.))
        scale = float(target.get("ramp_position_hold_gain_scale", 0.) if beta < 1.
                      else target.get("transport_position_hold_gain_scale", 0.))
        if not math.isfinite(scale) or scale < 0.:
            scale = 0.
        scale_dot = float(target.get("transport_position_hold_gain_scale_dot", 0.)) if beta >= 1. else 0.
        if not math.isfinite(scale_dot):
            scale_dot = 0.
        # Match the offline matched-scale derivative at a zero hold gain.
        if scale <= 0.:
            scale_dot = 0.
        gate = float(target.get("transport_full_pid_integral_gate_m", .23))
        limit_value = target.get("transport_full_pid_integral_limit")
        if limit_value is None:
            limit_value = getattr(self.config, "independent_integral_limit", None)
            if limit_value is None:
                limit_value = self.config.integral_limit
            limit_value = np.asarray(limit_value, dtype=float) * (10. / 3.)
        integral_limit = np.asarray(limit_value, dtype=float).reshape(3)
        ki = integral_gain * float(target.get("transport_full_pid_integral_gain_scale", 3.))
        feedback_limit = getattr(self.config, "independent_max_feedback_acceleration", None)
        feedback_limit = (np.full(3, np.inf) if feedback_limit is None
                          else np.asarray(feedback_limit, dtype=float).reshape(3))
        force_limit = float(target.get("transport_full_pid_force_limit_n", .06))
        force_rate = float(target.get("transport_full_pid_force_rate_limit_nps", .004))
        force_hold = bool(target.get("transport_full_pid_force_hold", False))
        reason = str(target.get("transport_full_pid_freeze_reason", ""))
        learning = (bool(target.get("transport_full_pid_learning", False))
                    and derivatives_ready and not reason and not force_hold)
        signature = (learning, reason, force_hold, scale, scale_dot, gate,
            integral_velocity_weight, force_limit, force_rate,
            *(tuple(np.asarray(value).reshape(-1)) for value in
              (ep, ev, acceleration, reference_acceleration, position_gain, velocity_gain,
               ki, integral_limit, feedback_limit)))
        if same_tick and signature == self._transport_position_pid_signature:
            result = {key: (value.copy() if isinstance(value, np.ndarray) else value)
                      for key, value in self._transport_position_pid_cached.items()}
            result.update(learning=False, duplicate_tick=True,
                          freeze_reason=reason or "duplicate_tick")
            return result
        previous_integral = self.transport_position_integral.copy()
        self.transport_position_integral, integral_rate, gate_open = position_integral_step(
            previous_integral, ep, ev, dt, gate, integral_limit,
            learning=learning and not same_tick, velocity_weight=integral_velocity_weight)
        pd, pd_dot, _ = position_pid_feedback(ep, ev, position_gain, velocity_gain,
            np.zeros(3), np.zeros(3), feedback_limit, acceleration-reference_acceleration,
            gain_scale=scale, gain_scale_dot=scale_dot)
        pid, pid_dot, unclamped = position_pid_feedback(ep, ev, position_gain, velocity_gain,
            ki, self.transport_position_integral, feedback_limit,
            acceleration-reference_acceleration, integral_rate,
            gain_scale=scale, gain_scale_dot=scale_dot)
        native_i = self.config.mass * (pid-pd)
        previous_force = self.transport_full_pid_force_previous.copy()
        force_norm = float(np.linalg.norm(native_i))
        limited = native_i * min(1., force_limit/max(force_norm, 1.e-30))
        force_limited = not np.allclose(limited, native_i, rtol=0., atol=1.e-14)
        delta = limited-previous_force
        max_step = 0. if same_tick or force_hold else force_rate*dt
        bias = previous_force + delta * min(1., max_step/max(float(np.linalg.norm(delta)), 1.e-30))
        rate_limited = not np.allclose(bias, limited, rtol=0., atol=1.e-14)
        # fixed_force offline budgeting always uses the actual bounded force step.
        # Only the explicitly unbounded native law uses its analytic I derivative.
        bias_dot = (np.zeros(3) if force_hold else
                    (bias-previous_force)/dt if math.isfinite(force_limit) or math.isfinite(force_rate)
                    else self.config.mass * (pid_dot-pd_dot))
        self.transport_full_pid_force_previous = bias.copy()
        result = dict(pd_force=self.config.mass*pd, pd_force_dot=self.config.mass*pd_dot,
            feedback_force=self.config.mass*pd+bias,
            feedback_force_dot=self.config.mass*pd_dot+bias_dot,
            native_pid_force=self.config.mass*pid, native_pid_force_dot=self.config.mass*pid_dot,
            native_extra_i=native_i, integral_force=bias, integral_force_dot=bias_dot,
            integral_state=self.transport_position_integral.copy(),
            integral_state_delta=self.transport_position_integral-previous_integral,
            integral_state_limit=integral_limit.copy(), integral_gain=ki.copy(),
            integral_gate_m=gate, integral_velocity_weight=integral_velocity_weight,
            native_gate_open=gate_open,
            integral_state_limit_active=bool(np.any(np.abs(self.transport_position_integral)>=integral_limit-1.e-12)),
            feedback_clamp_active=bool(np.any(np.abs(unclamped)>=feedback_limit)),
            force_budget_limited=force_limited, rate_budget_limited=rate_limited,
            learning=learning and not same_tick, duplicate_tick=same_tick, force_hold=force_hold,
            freeze_reason=reason or ("vehicle_derivatives" if not derivatives_ready else
                "force_hold" if force_hold else "duplicate_tick" if same_tick else
                "learning" if learning else "external_freeze"),
            hold_gain_scale=scale)
        self._transport_position_pid_tick = tick
        self._transport_position_pid_signature = signature
        self._transport_position_pid_cached = result
        return result


    def _compose_vehicle_force(self, state, target, dt, derivatives_ready,
                               measured_velocity, current_acceleration,
                               position_error, velocity_error,
                               target_acceleration, target_jerk,
                               position_gain, velocity_gain,
                               independent_position_gain,
                               independent_velocity_gain,
                               independent_integral_gain,
                               saturated_integral_rate, independent_mode,
                               transport_mode, native_transport_pid,
                               transport_blend, raw_desired_force):
        # ROS world 系按 z 向上处理：F = m(g e3 + a_d) - Kp ep - Kv ev - Ki ei。
        hold_acceleration = np.zeros(3)
        hold_acceleration_dot = np.zeros(3)
        hold_feedback_limit_active = False
        transport_integral_force = np.zeros(3)
        transport_integral_force_dot = np.zeros(3)
        transport_full_pid_force = np.zeros(3)
        transport_full_pid_force_dot = np.zeros(3)
        transport_full_pid_pd_force = np.zeros(3)
        transport_full_pid_pd_force_dot = np.zeros(3)
        native_pid_info = None
        blend_force_delta = np.zeros(3)
        force_override = target.get("desired_force_override")
        if independent_mode:
            pass
        elif force_override is not None:
            transport_force = np.asarray(force_override, dtype=float).reshape(3)
            if native_transport_pid:
                native_pid_info = self._native_transport_feedback(
                    state, target, dt, derivatives_ready,
                    measured_velocity if derivatives_ready else np.zeros(3),
                    independent_position_gain, independent_velocity_gain,
                    independent_integral_gain)
                transport_full_pid_force = native_pid_info["integral_force"]
                transport_full_pid_force_dot = native_pid_info["integral_force_dot"]
                weight = float(np.clip(target.get("transport_full_pid_weight", 1.), 0., 1.))
                weight_dot = float(target.get("transport_full_pid_weight_dot", 0.))
                if not math.isfinite(weight_dot):
                    weight_dot = 0.
                transport_full_pid_force_dot = (
                    weight * transport_full_pid_force_dot + weight_dot * transport_full_pid_force)
                transport_full_pid_force = weight * transport_full_pid_force
                transport_full_pid_pd_force = native_pid_info["pd_force"]
                transport_full_pid_pd_force_dot = native_pid_info["pd_force_dot"]
                transport_force = transport_force + transport_full_pid_pd_force + transport_full_pid_force
                hold_feedback_limit_active = native_pid_info["feedback_clamp_active"]
            elif transport_mode:
                transport_integral_force = as_vector(
                    target.get("transport_hold_integral_force", np.zeros(3)),
                    "transport_hold_integral_force")
                transport_integral_force_dot = as_vector(
                    target.get("transport_hold_integral_force_dot", np.zeros(3)),
                    "transport_hold_integral_force_dot")
                transport_force = transport_force + transport_integral_force
            independent_force = (
                self.config.mass * (
                    target_acceleration + np.array([0.0, 0.0, self.config.gravity])
                )
                - position_gain * position_error
                - velocity_gain * velocity_error
                - self.config.integral_gain * self.position_integral
            )
            if "handoff_force" in target:
                # Reuse the PID output that was supporting the prelifted load.
                independent_force = np.asarray(target["handoff_force"], dtype=float).reshape(3)
            raw_desired_force = (
                (1.0 - transport_blend) * independent_force
                + transport_blend * transport_force
            )
            blend_force_delta = transport_force - independent_force
            # Match Udaan's TENSION_RAMP handoff: the cooperative force is
            # blended in, but an independent position-hold term remains active
            # so the aircraft stay at their measured TAKEUP anchors.  Without
            # this term, horizontal cable forces can pull all three aircraft
            # toward the payload center while tension is being established.
            # During the ramp use the Udaan handoff gain.  After the ramp,
            # keep the same independent hold around the translated TAKEUP
            # anchor while the payload reference is lifted; otherwise a cable
            # direction/model error can pull CF4/CF5 inward during hover.
            if transport_blend < 1.0:
                hold_scale = float(target.get(
                    "ramp_position_hold_gain_scale", 0.0
                ))
            else:
                hold_scale = float(target.get(
                    "transport_position_hold_gain_scale", 0.0
                ))
            if (not native_transport_pid and transport_blend > 0.0 and math.isfinite(hold_scale) and
                    hold_scale > 0.0):
                hold_position_error = state["position"] - np.asarray(
                    target.get("position_hold_position", target["position"]), dtype=float
                )
                hold_velocity_error = measured_velocity - np.asarray(
                    target.get("position_hold_velocity", target["velocity"]), dtype=float
                ) if derivatives_ready else np.zeros(3)
                hold_acceleration = hold_scale * (
                    -independent_position_gain * hold_position_error
                    -independent_velocity_gain * hold_velocity_error
                )
                hold_acceleration_dot = hold_scale * (
                    -independent_position_gain * hold_velocity_error
                    -independent_velocity_gain * (current_acceleration - np.asarray(
                        target.get("position_hold_acceleration", target_acceleration)
                    ))
                )
                hold_scale_dot = (float(target.get(
                    "transport_position_hold_gain_scale_dot", 0.0
                )) if transport_blend >= 1.0 else 0.0)
                hold_acceleration_dot += hold_scale_dot * (
                    -independent_position_gain * hold_position_error
                    -independent_velocity_gain * hold_velocity_error
                )
                max_feedback = getattr(
                    self.config, "independent_max_feedback_acceleration", None
                )
                if max_feedback is not None:
                    max_feedback = np.asarray(max_feedback, dtype=float).reshape(3)
                    unsaturated = np.abs(hold_acceleration) < max_feedback
                    hold_feedback_limit_active = bool(np.any(np.abs(hold_acceleration) >= max_feedback))
                    hold_acceleration = clamp_vector(hold_acceleration, -max_feedback, max_feedback)
                    hold_acceleration_dot = np.where(
                        unsaturated, hold_acceleration_dot, 0.0
                    )
                raw_desired_force += (
                    transport_blend * self.config.mass * hold_acceleration
                )
        else:
            transport_blend = 0.0
            raw_desired_force = (
                self.config.mass * (
                    target_acceleration + np.array([0.0, 0.0, self.config.gravity])
                )
                - position_gain * position_error
                - velocity_gain * velocity_error
                - self.config.integral_gain * self.position_integral
            )
        force_dot_override = target.get("desired_force_dot_override")
        if independent_mode:
            raw_desired_force_dot = self.config.mass * target_jerk
            raw_desired_force_dot[2] += float(target.get("landing_support_trim_dot_nps", 0.0))
        elif force_dot_override is not None or native_pid_info is not None:
            transport_force_dot = np.asarray(
                np.zeros(3) if force_dot_override is None else force_dot_override, dtype=float
            ).reshape(3) + transport_integral_force_dot
            if native_pid_info is not None:
                transport_force_dot = (transport_force_dot + transport_full_pid_pd_force_dot
                                       + transport_full_pid_force_dot)
            independent_force_dot = (
                self.config.mass * target_jerk
                - position_gain * velocity_error
                - velocity_gain * (current_acceleration - target_acceleration)
                - self.config.integral_gain * saturated_integral_rate
            )
            if "handoff_force" in target:
                independent_force_dot = np.zeros(3)
            raw_desired_force_dot = (
                (1.0 - transport_blend) * independent_force_dot
                + transport_blend * transport_force_dot
            )
        else:
            raw_desired_force_dot = (
                self.config.mass * target_jerk
                - position_gain * velocity_error
                - velocity_gain * (current_acceleration - target_acceleration)
                - self.config.integral_gain * saturated_integral_rate
            )
        transport_blend_dot = float(target.get("transport_blend_dot", 0.0))
        if math.isfinite(transport_blend_dot):
            raw_desired_force_dot += (
                transport_blend_dot * blend_force_delta
                + self.config.mass * (
                    transport_blend * hold_acceleration_dot
                    + transport_blend_dot * hold_acceleration
                )
            )
        return VehicleForceCommand(
            raw_desired_force=raw_desired_force,
            raw_desired_force_dot=raw_desired_force_dot,
            transport_blend=transport_blend,
            hold_feedback_limit_active=hold_feedback_limit_active,
            transport_integral_force=transport_integral_force,
            transport_integral_force_dot=transport_integral_force_dot,
            transport_full_pid_force=transport_full_pid_force,
            transport_full_pid_force_dot=transport_full_pid_force_dot,
            transport_full_pid_pd_force=transport_full_pid_pd_force,
            transport_full_pid_pd_force_dot=transport_full_pid_pd_force_dot,
            native_pid_info=native_pid_info,
            hold_acceleration=hold_acceleration,
            transport_blend_dot=transport_blend_dot,
        )

    def _filter_transport_force(self, desired_force, desired_force_dot,
                                target, transport_mode, dt):
        force_filter_tau = float(getattr(self.config, "transport_force_filter_tau_s", 0.0))
        if transport_mode and force_filter_tau > 0.0:
            force_filter_reset = bool(target.get("transport_filter_reset", False))
            filter_was_reset = force_filter_reset or self.previous_transport_force is None
            if filter_was_reset:
                self.previous_transport_force = desired_force.copy()
            else:
                previous_filtered_force = self.previous_transport_force.copy()
                alpha = float(np.clip(dt / (force_filter_tau + dt), 0.0, 1.0))
                self.previous_transport_force += alpha * (
                    desired_force - self.previous_transport_force
                )
            desired_force = self.previous_transport_force.copy()
            # Keep force direction, thrust projection and Omega_c consistent.
            # The force filter is intentionally separate from collective thrust.
            desired_force_dot = (
                np.zeros(3) if filter_was_reset else
                (desired_force - previous_filtered_force) / dt
            )
        elif not transport_mode:
            self.previous_transport_force = None
        return desired_force, desired_force_dot

    def _transport_body_rate(self, state, target, rotation, computed_rotation,
                             computed_body_rate, attitude_error, transport_blend):
        transport_attitude_gain = np.asarray(
            target.get("transport_attitude_gain", self.config.attitude_gain),
            dtype=float,
        ).reshape(3)
        transport_rate_gain = np.asarray(
            target.get("transport_rate_gain", np.zeros(3)),
            dtype=float,
        ).reshape(3)
        transport_body_rate_command = (
            -transport_attitude_gain * attitude_error
            -transport_rate_gain * np.asarray(
                state.get("body_rate", np.zeros(3)), dtype=float
            ).reshape(3)
        )
        if transport_blend < 1.0:
            if self.config.use_body_rate_feedforward:
                computed_body_rate_current_frame = (
                    rotation.T @ computed_rotation @ computed_body_rate
                )
            else:
                computed_body_rate_current_frame = np.zeros(3)
            independent_body_rate_command = (
                computed_body_rate_current_frame
                - self.config.attitude_gain * attitude_error
                - self.config.attitude_integral_gain * self.attitude_integral
            )
            if "handoff_force" in target:
                gain = (self.config.independent_attitude_gain
                        if self.config.independent_attitude_gain is not None
                        else self.config.attitude_gain)
                independent_body_rate_command = -np.asarray(gain) * attitude_error
            body_rate_command = (
                (1.0 - transport_blend) * independent_body_rate_command
                + transport_blend * transport_body_rate_command
            )
        else:
            body_rate_command = transport_body_rate_command
        return body_rate_command

    def _limit_body_rate(self, body_rate_command, target, independent_mode,
                         transport_mode, dt):
        body_rate_limit = self.config.max_body_rate
        if independent_mode and getattr(self.config, "independent_max_body_rate", None) is not None:
            body_rate_limit = np.asarray(
                self.config.independent_max_body_rate, dtype=float
            ).reshape(3)
        raw_body_rate_command = np.asarray(body_rate_command, dtype=float).copy()
        body_rate_limit_active = bool(np.any(np.abs(raw_body_rate_command) >= body_rate_limit))
        filter_tau = float(getattr(self.config, "transport_body_rate_filter_tau_s", 0.0))
        filter_reset = bool(target.get("transport_filter_reset", False))
        if transport_mode and filter_tau > 0.0:
            if filter_reset or self.previous_transport_body_rate_command is None:
                self.previous_transport_body_rate_command = raw_body_rate_command.copy()
            else:
                alpha = float(np.clip(dt / (filter_tau + dt), 0.0, 1.0))
                self.previous_transport_body_rate_command += alpha * (
                    raw_body_rate_command - self.previous_transport_body_rate_command
                )
            body_rate_command = self.previous_transport_body_rate_command.copy()
        else:
            body_rate_command = raw_body_rate_command
            if not transport_mode:
                self.previous_transport_body_rate_command = None
        body_rate_limit_active |= bool(np.any(np.abs(body_rate_command) >= body_rate_limit))
        body_rate_command = clamp_vector(
            body_rate_command, -body_rate_limit, body_rate_limit
        )
        return raw_body_rate_command, body_rate_command, body_rate_limit_active

