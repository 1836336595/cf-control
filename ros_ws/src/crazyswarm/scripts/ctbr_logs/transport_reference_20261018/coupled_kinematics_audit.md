# Coupled acceleration audit, 2026-10-18

This is an analysis of the controller's modeled attachment acceleration. It
does not establish a flight stability fix. No descent logic is modified by the
fixture or this note.

The standalone reproduction is `audit_coupled_kinematics.py`; it writes
`coupled_kinematics_fixture.json`. Run it with Python and NumPy. It constructs a
23 g rigid body, three aircraft, nonaligned cable directions, a nonzero load
body rate, and an allocated internal force in the allocation matrix's nullspace.
It solves the coupled physical equations independently of the production
controller. Its input wrench is a synthetic diagnostic fixture, not a replay
measurement.

## What the local source assumes

`MATLAB/Multi-UAV-Transportation-Simulation/crazyflie_slung_controller.m:94-98`
constructs commanded angular acceleration from the requested moment and bare
payload inertia. Its comment treats omitted coupling as higher order. At
lines 157-168 it explains replacing the physical accelerations in attachment
kinematics with the current requested outer-loop accelerations to avoid a
delayed acceleration measurement. At lines 180-182 it projects allocated
forces onto the measured cable directions; lines 191-202 then use the requested
acceleration in the aircraft's parallel force. Thus the mismatch is an explicit
model approximation, rather than an accidental Python sign conversion.

The corresponding Python implementation is
`ros_ws/src/crazyswarm/scripts/slung_load_controller.py`, around `compute`,
`omega0_dot_cmd`, `mu_i`, and `ai`.
`simulate_uav/udaan/udaan/control/crazyswarm_slung.py:173-196,232-263` uses the
same approximation, expressed in its z-down convention.

`MATLAB/Multi-UAV-Transportation-Simulation/crazyflie_slung_dynamics.m:8-15`
states the coupled physical equations. Lines 100-161 assemble their 6 by 6
mass matrix and force vector. The physical accelerations are solved from the
actual applied parallel forces; the requested wrench does not replace those
physical accelerations in the plant. These local source locations support the
analysis below. They are not independent experimental evidence and the comments
are not a substitute for an external paper derivation.

## Kinematics and the mass matrix

Use z-up world coordinates, body-to-world load rotation `R`, body attachment
vectors `rho_i`, and cable directions `q_i` from aircraft to load. Write
`H(v)` for the cross-product matrix. Define

```
X = load_acceleration + g e3
y = [X; load_body_angular_acceleration]
B_i = [I, -R H(rho_i)]
c_i = R H(Omega)^2 rho_i
Q_i = q_i q_i.T
```

The physical attachment acceleration including gravity is

```
a_i = B_i y + c_i.
```

Let `mu_bar_i = -max(-q_i.T mu_des_i, 0) q_i` denote the projected unilateral
allocation. If an aircraft perfectly tracks the controller's force, its
parallel force has the form

```
u_parallel_i = mu_bar_i + m_i l_i |omega_link_i|^2 q_i
               + m_i Q_i (B_i y_cmd + c_i).
```

The actual cable force on the load is consequently

```
mu_actual_i = mu_bar_i + m_i Q_i B_i (y_cmd - y_actual).
```

This identity is the missing qualification when describing `mu_bar` as realized
tension. It equals physical tension only when the commanded attachment
acceleration equals the actual attachment acceleration under the ideal force
model. It is not measured tension.

Define the positive definite bare payload mass matrix and positive semidefinite
aircraft contribution

```
D = diag(m_payload I, J_payload)
C = sum_i m_i B_i.T Q_i B_i.
```

The coupled acceleration equation, after cancellation of the modeled cable
centripetal terms, is

```
(D + C) y_actual = [sum mu_bar_i;
                    sum H(rho_i) R.T mu_bar_i - Omega x J Omega]
                   + C y_cmd.
```

This is the local MATLAB 6 by 6 equation in compact form, with the same signs.
It also shows why merely replacing requested wrench by projected wrench can
make the ideal equation algebraically exact:

```
y_projected = D^-1 [F_projected; M_projected - Omega x J Omega]
y_cmd = y_projected  =>  y_actual = y_projected.
```

The fixture's maximum residual is about `5.3e-14`, whereas using the requested
wrench in `y_cmd` produces a maximum residual of about `15.8` in the mixed
acceleration coordinates. This is an algebraic diagnostic, not an error bound
for the flight. In this fixture the requested moment is
`[-0.100, -0.200, -0.050] mN m`, while projection produces approximately
`[-0.398, -0.321, -0.173] mN m`. The nullspace bias has zero unprojected wrench;
projection need not preserve that property.

## Why exact ideal projection is not a robust fix

The root agent's runtime probes replaced angular compensation alone and then
both translational and angular compensation with projected-wrench versions.
Both failed the distance guard around 3.7 s after handoff, earlier than the
reference-only candidate around 11.7 s in the same zero load-damping boundary.
Those results reject promoting this substitution as a flight fix.

Aircraft position hold, support trim, command tilt/thrust constraints,
asynchronous measurements, actuator lag, and external load torques are absent
from the fixture. Their forces perturb actual tension. The projected model
does not account for them, and division by the 23 g body's small inertia can
produce larger angular acceleration compensation when the projection residual
moment is large. No noisy differentiated angular acceleration is needed to
demonstrate this problem, and adding one is not justified by this audit.

The free-translation effective rotational inertia is the Schur complement

```
J_effective = (D+C)_rotation,rotation
              - (D+C)_rotation,translation
                (D+C)_translation,translation^-1
                (D+C)_translation,rotation.
```

For this fixture its diagonal relative to bare payload inertia is approximately
`[2.70, 4.30, 1.19]`. Compensation based on bare `J_payload^-1` therefore
actively cancels a substantial aircraft contribution. A delayed realization of
that cancellation can change the damping and phase of the combined loop. The
matrix calculation establishes a strong coupling path; it does not prove the
sign or cause of the measured 2 Hz growth.

A useful isolated diagnostic is scaling only
`-R H(rho_i) omega0_dot_cmd` in `ai` while retaining its translational and
centripetal terms. Zero scale restores the corresponding passive inertia;
intermediate scales test how strongly cancellation contributes. If this
ablation helps, a final controller should choose compensation and bandwidth
from the coupled model and its delay limits. A bare replacement by
`J_effective^-1`, an arbitrary limiter, or a new gain is not validated merely
by the algebra above.

## Attitude moment wording

The added conventional moment feedforward makes commanded angular acceleration
equal the derivative of the desired body rate transported into the measured
body frame, plus feedback. This is a precise property covered by the new
finite-difference test.

The preceding desired-rate gyroscopic formulation is not, by itself, proof of
an invalid attitude stabilizer. With `f = R.T Rd omega_d` and `e = Omega - f`,
the difference between conventional exact cancellation and `f x J f` is
orthogonal to `e`. Such a difference can preserve the angular-rate error energy
calculation while producing different off-tracking angular acceleration. The
moment patch should be described by its tracking property, rather than claimed
as proof that the old law caused growth.
