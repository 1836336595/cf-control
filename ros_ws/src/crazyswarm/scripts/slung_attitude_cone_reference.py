"""ROS-independent cone attitude and gravity-balanced cable references."""

import math

import numpy as np

from slung_circle_reference import CircleTrajectoryConfig, _array, _scalar
from slung_reference import _hat, _rotation_matrix


def _smoothstep(elapsed, duration):
    if elapsed <= 0.:
        return 0., 0., 0.
    if elapsed >= duration:
        return 1., 0., 0.
    u = elapsed/duration
    return (u**3*(10.+u*(-15.+6.*u)),
            30.*u**2*(1.-u)**2/duration,
            60.*u*(1.-u)*(1.-2.*u)/duration**2)


class PayloadConeReference:
    """Keep the payload center fixed while its body +Z sweeps a cone.

    The attitude is Rz(phase) Ry(tilt) Rz(-phase), a closed convention that
    avoids an imposed full body yaw turn. This does not impose zero angular
    velocity about body +Z. The tilt enters/exits with quintic smoothsteps;
    each complete revolution repeats the same cone_period_s smoothstep.
    Cycle boundaries keep the tilt while bringing angular rate/acceleration
    smoothly to zero, so adding a revolution adds one identical scan period.
    """

    def __init__(self, hover_position, start_time, config):
        self._center = _array(hover_position, "hover_position", (3,))
        self.start_time = _scalar(start_time, "start_time")
        if not isinstance(config, CircleTrajectoryConfig) or config.mode != "attitude_cone":
            raise ValueError("config must select attitude_cone")
        self.config = config
        if config.center_xy is not None:
            self._center[:2] = config.center_xy
        self._half_angle = math.radians(config.cone_half_angle_deg)
        self._sweep_duration = config.revolutions*config.cone_period_s
        self._total_phase = 2.*math.pi*config.revolutions*(1. if config.direction == "ccw" else -1.)
        self.duration = 2.*config.cone_transition_s+self._sweep_duration
        self.end_time = self.start_time+self.duration
        if not all(math.isfinite(value) for value in
                   (self._sweep_duration, self._total_phase, self.duration, self.end_time)):
            raise ValueError("cone timing and phase must remain finite")

    @property
    def entry_position(self):
        return self._center.copy()

    @property
    def center_position(self):
        return self._center.copy()

    @property
    def initial_yaw(self):
        return 0.

    @property
    def initial_rotation(self):
        return np.eye(3)

    def evaluate(self, now):
        now = _scalar(now, "now")
        elapsed = now-self.start_time
        transition = self.config.cone_transition_s
        phase = phase_rate = phase_acceleration = 0.
        tilt = tilt_rate = tilt_acceleration = 0.
        cone_progress = 0.
        if now < self.start_time:
            stage = "waiting"
        elif now >= self.end_time:
            stage, phase, cone_progress = "complete", self._total_phase, 1.
        elif elapsed < transition:
            stage = "cone_tilt_in"
            scale, rate, acceleration = _smoothstep(elapsed, transition)
            tilt, tilt_rate, tilt_acceleration = self._half_angle*np.array([scale, rate, acceleration])
        elif elapsed < transition+self._sweep_duration:
            stage, tilt = "attitude_cone", self._half_angle
            sweep_elapsed = elapsed-transition
            period = self.config.cone_period_s
            cycle = min(int(sweep_elapsed/period), int(self.config.revolutions)-1)
            scale, rate, acceleration = _smoothstep(sweep_elapsed-cycle*period, period)
            turn = 2.*math.pi*(1. if self.config.direction == "ccw" else -1.)
            phase = turn*(cycle+scale)
            phase_rate, phase_acceleration = turn*rate, turn*acceleration
            cone_progress = (cycle+scale)/self.config.revolutions
        else:
            stage, phase, cone_progress = "cone_tilt_out", self._total_phase, 1.
            scale, rate, acceleration = _smoothstep(elapsed-transition-self._sweep_duration, transition)
            tilt, tilt_rate, tilt_acceleration = self._half_angle*np.array([1.-scale, -rate, -acceleration])

        cp, sp, ct, st = math.cos(phase), math.sin(phase), math.cos(tilt), math.sin(tilt)
        rz = np.array([[cp, -sp, 0.], [sp, cp, 0.], [0., 0., 1.]])
        ry = np.array([[ct, 0., st], [0., 1., 0.], [-st, 0., ct]])
        rotation = rz@ry@rz.T if tilt != 0. else np.eye(3)
        u = np.array([-sp, cp, 0.])
        v = np.array([-cp*st, -sp*st, ct-1.])
        u_phase = np.array([-cp, -sp, 0.])
        v_tilt = np.array([-cp*ct, -sp*ct, -st])
        v_phase = np.array([sp*st, -cp*st, 0.])
        omega = tilt_rate*u+phase_rate*v
        alpha = (tilt_acceleration*u+tilt_rate*phase_rate*u_phase
                 + phase_acceleration*v+phase_rate*tilt_rate*v_tilt+phase_rate**2*v_phase)

        # Euler yaw is diagnostic only; the controller consumes full body PVA.
        rotation_dot = rotation@_hat(omega)
        rotation_ddot = rotation@(_hat(alpha)+_hat(omega)@_hat(omega))
        x, y = rotation[0, 0], rotation[1, 0]
        xd, yd = rotation_dot[0, 0], rotation_dot[1, 0]
        xdd, ydd = rotation_ddot[0, 0], rotation_ddot[1, 0]
        denominator = x*x+y*y
        numerator = x*yd-y*xd
        yaw_rate = numerator/denominator
        yaw_acceleration = ((x*ydd-y*xdd)/denominator
                            - numerator*2.*(x*xd+y*yd)/denominator**2)
        return dict(position=self._center.copy(), velocity=np.zeros(3), acceleration=np.zeros(3),
                    rotation=rotation, body_rate=omega, body_rate_dot=alpha,
                    yaw=math.atan2(y, x), yaw_rate=yaw_rate, yaw_acceleration=yaw_acceleration,
                    cone_phase_rad=phase, cone_phase_rate=phase_rate,
                    cone_phase_acceleration=phase_acceleration, cone_tilt_rad=tilt,
                    phase=stage, progress=float(np.clip(elapsed/self.duration, 0., 1.)),
                    cone_progress=cone_progress)


def equilibrium_cable_reference(rotation, body_rate, body_rate_dot,
                                allocation_map, internal_bias, gravity_force):
    """Differentiate static gravity-balanced cable directions analytically.

    allocation_map is P.T @ inv(P @ P.T), with stacked three-vector vehicle
    blocks. The nominal moment is zero; the actual controller still supplies
    dynamic attitude feedback/feedforward moments in its tension allocation.
    Returned arrays have one world-frame unit cable direction per column.
    """
    rotation = _rotation_matrix(_array(rotation, "rotation", (3, 3)), "rotation")
    omega = _array(body_rate, "body_rate", (3,))
    alpha = _array(body_rate_dot, "body_rate_dot", (3,))
    allocation = _array(allocation_map, "allocation_map", (9, 6))
    bias = _array(internal_bias, "internal_bias", (9,))
    gravity = _array(gravity_force, "gravity_force", (3,))
    rdot = rotation@_hat(omega)
    rddot = rotation@(_hat(alpha)+_hat(omega)@_hat(omega))
    forces, rates, accelerations = [], [], []
    for index in range(3):
        block = allocation[3*index:3*index+3, :3]
        offset = bias[3*index:3*index+3]
        force = rotation@block@rotation.T@gravity+rotation@offset
        rate = (rdot@block@rotation.T@gravity+rotation@block@rdot.T@gravity+rdot@offset)
        acceleration = (rddot@block@rotation.T@gravity+2.*rdot@block@rdot.T@gravity
                        + rotation@block@rddot.T@gravity+rddot@offset)
        forces.append(force)
        rates.append(rate)
        accelerations.append(acceleration)
    force, rate, acceleration = (np.asarray(values).T for values in (forces, rates, accelerations))
    norms = np.linalg.norm(force, axis=0)
    if (not all(np.all(np.isfinite(value)) for value in (force, rate, acceleration, norms))
            or np.any(norms <= 1.e-12)):
        raise ValueError("equilibrium cable force must be finite and nonzero")
    norm_rate = np.sum(force*rate, axis=0)/norms
    norm_acceleration = (np.sum(rate*rate+force*acceleration, axis=0)-norm_rate**2)/norms
    q = -force/norms
    qdot = -rate/norms+force*norm_rate/norms**2
    qddot = (-acceleration/norms+2.*rate*norm_rate/norms**2
             + force*norm_acceleration/norms**2-2.*force*norm_rate**2/norms**3)
    return dict(directions=q, direction_rates=qdot, direction_accelerations=qddot)
