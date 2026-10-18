def compute(self, payload_state, vehicle_states, target, dt):
    """Return per-vehicle force commands or ``None`` for invalid payload state."""
    if not payload_state or not bool(payload_state.get("valid", False)):
        return None
    if len(vehicle_states) != 3:
        raise ValueError("slung-load controller requires three vehicle states")
    dt = float(np.clip(dt, 0.001, 0.05))
    try:
        p0 = self._state_vector(payload_state, "position")
        v0 = self._state_vector(payload_state, "velocity", np.zeros(3))
        a0 = self._state_vector(payload_state, "acceleration", np.zeros(3))
        r0 = _project_so3(payload_state["rotation"])
        omega0 = self._state_vector(payload_state, "body_rate", np.zeros(3))
        pd = self._state_vector(target, "position")
        vd = self._state_vector(target, "velocity", np.zeros(3))
        ad = self._state_vector(target, "acceleration", np.zeros(3))
        rd = _project_so3(target.get("rotation", np.eye(3)))
        omegad = self._state_vector(target, "body_rate", np.zeros(3))
        omegadd = self._state_vector(target, "body_rate_dot", np.zeros(3))
    except (KeyError, TypeError, ValueError):
        return None

    ep = p0 - pd
    ev = v0 - vd
    integral_rate = ev + self.config.c1 * ep
    self.position_integral = np.clip(
        self.position_integral + dt * integral_rate,
        -self.config.integral_limit,
        self.config.integral_limit,
    )
    desired_load_acceleration = (
        ad - self.config.position_gain * ep
        - self.config.velocity_gain * ev
        - self.config.integral_gain * self.position_integral
    )
    gravity = np.array([0.0, 0.0, self.config.gravity_mps2])
    desired_force = self.config.payload_mass_kg * (
        desired_load_acceleration + gravity
    )

    e_r = 0.5 * _vee(rd.T @ r0 - r0.T @ rd)
    e_omega = omega0 - r0.T @ rd @ omegad
    inertia = np.diag([
        self.config.payload_mass_kg * (
            self.config.payload_size_m[1] ** 2 + self.config.payload_size_m[2] ** 2
        ) / 12.0,
        self.config.payload_mass_kg * (
            self.config.payload_size_m[0] ** 2 + self.config.payload_size_m[2] ** 2
        ) / 12.0,
        self.config.payload_mass_kg * (
            self.config.payload_size_m[0] ** 2 + self.config.payload_size_m[1] ** 2
        ) / 12.0,
    ])
    feedforward = r0.T @ rd @ omegad
    # Desired rates live in Rd's body frame.  Their derivative after
    # transport into the measured body includes -omega x feedforward;
    # cancel measured gyroscopic coupling before commanding that rate.
    desired_moment = (
        np.cross(omega0, inertia @ omega0)
        - inertia @ np.cross(omega0, feedforward)
        + inertia @ r0.T @ rd @ omegadd
    )
    # Payload attitude gains are supplied by the caller in target.  This
    # avoids duplicating MATLAB's size-derived gains in the vehicle file.
    desired_moment += (
        -_as_vector(target["payload_attitude_gain"], "payload_attitude_gain") * e_r
        -_as_vector(target["payload_rate_gain"], "payload_rate_gain") * e_omega
    )
    if not bool(target.get("payload_yaw_enabled", True)):
        desired_moment[2] = 0.0
    omega0_dot_cmd = np.linalg.solve(
        inertia, desired_moment - np.cross(omega0, inertia @ omega0)
    )

    matrix = self._allocation_matrix()
    rhs = np.concatenate([r0.T @ desired_force, desired_moment])
    gram = matrix @ matrix.T
    if np.linalg.cond(gram) > 1.0 / self.config.tension_pinv_tolerance:
        raise ValueError("slung-load tension allocation matrix is singular")
    mu_body = matrix.T @ np.linalg.solve(gram, rhs)
    mu_body += self._internal_bias(
        matrix, self.config.payload_mass_kg, self.config.gravity_mps2
    )
    mu_desired = r0 @ mu_body.reshape(3, 3).T

    link_units = np.zeros((3, 3))
    link_rates = np.zeros((3, 3))
    for index, vehicle in enumerate(vehicle_states):
        pv = self._state_vector(vehicle, "position")
        vv = self._state_vector(vehicle, "velocity", vehicle.get("control_velocity", np.zeros(3)))
        rho_world = r0 @ self.config.attachment_points_m[:, index]
        attachment = p0 + rho_world
        relative = attachment - pv
        link_units[:, index] = _normalize(relative, [0.0, 0.0, -1.0])
        payload_omega_world = r0 @ omega0
        relative_velocity = v0 + np.cross(payload_omega_world, rho_world) - vv
        link_rates[:, index] = (
            (np.eye(3) - np.outer(link_units[:, index], link_units[:, index]))
            @ relative_velocity / max(float(np.linalg.norm(relative)), 1.0e-6)
        )

    projected_tensions = np.maximum(-np.sum(link_units * mu_desired, axis=0), 0.0)
    projected_mu = -link_units * projected_tensions[None, :]
    projected_moment = np.sum(np.cross(self.config.attachment_points_m.T,
                                       (r0.T @ projected_mu).T), axis=0)
    omega0_dot_cmd = np.linalg.solve(
        inertia, projected_moment - np.cross(omega0, inertia @ omega0))
    if self.previous_link_units is None:
        self.previous_link_units = link_units.copy()
    outputs = []
    previous_perpendicular_forces = getattr(self, "previous_perpendicular_forces", None)
    if previous_perpendicular_forces is None:
        previous_perpendicular_forces = [None, None, None]
    # Establish link-direction correction continuously with TAUT_RAMP.
    # Full-strength correction at handoff can pull an aircraft inward when
    # its measured cable direction still differs from static allocation.
    link_gain_scale = float(np.clip(target.get("link_gain_scale", 1.0), 0.0, 1.0))
    for index, vehicle in enumerate(vehicle_states):
        q = link_units[:, index]
        qdot = link_rates[:, index]
        omega_link = np.cross(q, qdot)
        rho = self.config.attachment_points_m[:, index]
        ai = (
            desired_load_acceleration + gravity
            + r0 @ (_hat(omega0) @ (_hat(omega0) @ rho))
            - r0 @ _hat(rho) @ omega0_dot_cmd
        )
        mu_id = mu_desired[:, index]
        signed_tension = -float(q @ mu_id)
        allocated_qid = _normalize(-mu_id, self.previous_link_units[:, index])
        if signed_tension <= 0.0:
            # A cable can pull but cannot push. Hold its measured direction
            # when the unconstrained wrench allocation asks for compression.
            qid = q.copy()
        else:
            qid = allocated_qid
        desired_tension = max(signed_tension, 0.0)
        mu_i = -desired_tension * q
        eq = np.cross(qid, q)
        self.link_integrals[:, index] = np.clip(
            self.link_integrals[:, index] + dt * eq,
            -self.config.link_integral_limit,
            self.config.link_integral_limit,
        )
        correction = (
            -link_gain_scale * self.config.link_kq * eq
            -link_gain_scale * self.config.link_komega * omega_link
            -link_gain_scale * self.config.link_integral_gain * self.link_integrals[:, index]
        )
        q_hat_sq = _hat(q) @ _hat(q)
        u_parallel = (
            mu_i
            + vehicle.get("mass", 0.043) * self.config.link_lengths_m[index]
            * float(omega_link @ omega_link) * q
            + vehicle.get("mass", 0.043) * q * float(q @ ai)
        )
        raw_link_force = (
            vehicle.get("mass", 0.043) * self.config.link_lengths_m[index]
            * np.cross(q, correction)
        )
        link_force, _ = self._limit_link_force(
            raw_link_force, previous_perpendicular_forces[index], dt
        )
        correction_norm = float(np.linalg.norm(raw_link_force))
        correction_limit = float(self.config.link_force_correction_limit_n)
        bounded_correction = raw_link_force * min(
            1.0, correction_limit / max(correction_norm, 1.0e-12)
        )
        kinematic_force = -vehicle.get("mass", 0.043) * q_hat_sq @ ai
        u_perp = link_force + kinematic_force
        total_force = u_parallel + u_perp
        outputs.append({
            "force": total_force,
            "force_dot": np.zeros(3),
            "raw_link_force": raw_link_force.copy(),
            "link_force": link_force.copy(),
            "link_force_magnitude_limited": correction_norm > correction_limit,
            "link_force_rate_limited": bool(
                np.linalg.norm(link_force - bounded_correction) > 1.0e-12
            ),
            "kinematic_perpendicular_force": kinematic_force.copy(),
            "perpendicular_force": u_perp.copy(),
            "payload_position_error": ep.copy(),
            "payload_velocity_error": ev.copy(),
            "payload_attitude_error": e_r.copy(),
            "link_direction": q.copy(),
            "desired_link_direction": qid.copy(),
            "raw_desired_link_direction": allocated_qid.copy(),
            "link_direction_error": eq.copy(),
            "desired_tension": desired_tension,
            "desired_tension_signed": signed_tension,
            "desired_tension_vector": mu_id.copy(),
        })
    vehicle_forces = np.asarray([output["force"] for output in outputs], dtype=float)
    # The rate limiter applies only to cable-direction correction. Keep
    # kinematic compensation out of its history, including on short ticks.
    self.previous_perpendicular_forces = [
        output["link_force"].copy() for output in outputs
    ]
    if self.previous_vehicle_forces is None:
        vehicle_force_dots = np.zeros_like(vehicle_forces)
    else:
        vehicle_force_dots = (vehicle_forces - self.previous_vehicle_forces) / dt
        vehicle_force_dots = np.clip(
            vehicle_force_dots,
            -float(self.config.force_dot_limit_nps),
            float(self.config.force_dot_limit_nps),
        )
    # ROS Noetic uses Python 3.8; these arrays both come from the same outputs.
    for output, force_dot in zip(outputs, vehicle_force_dots):
        output["force_dot"] = force_dot.copy()
    self.previous_vehicle_forces = vehicle_forces.copy()
    self.previous_link_units = link_units.copy()
    return {
        "vehicles": outputs,
        "payload_position_error": ep,
        "payload_velocity_error": ev,
        "payload_attitude_error": e_r,
        "desired_force": desired_force,
        "desired_moment": desired_moment,
        "link_units": link_units,
    }
