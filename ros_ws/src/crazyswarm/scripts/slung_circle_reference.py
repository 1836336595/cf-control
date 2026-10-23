"""ROS-independent payload circles, figure eights, and aircraft geometry."""

from collections.abc import Mapping
from dataclasses import dataclass, fields
import math
from numbers import Real

import numpy as np

from slung_reference import _hat, _rotation_exp, _rotation_matrix


def _scalar(value, name, minimum=None, strict=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(name + " must be a finite number")
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(name + " must be a finite number") from None
    if not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    if minimum is not None and (value < minimum or (strict and value == minimum)):
        raise ValueError(name + (" must be positive" if strict else " must be nonnegative"))
    return value


def _array(value, name, shape=None):
    try:
        original = np.asarray(value, dtype=object)
        if any(isinstance(item, (bool, np.bool_)) or not isinstance(item, Real)
               for item in original.flat):
            raise ValueError
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(name + " must contain finite numbers") from None
    if not np.all(np.isfinite(result)) or (shape is not None and result.shape != shape):
        raise ValueError(name + " has invalid shape or nonfinite values")
    return result.copy()


@dataclass(frozen=True)
class CircleTrajectoryConfig:
    """Validate the optional payload trajectory even when hover is selected.

    ``speed_mps`` is a peak linear speed. ``ramp_s`` is the duration of each
    smooth acceleration/deceleration ramp; short arcs reduce their peak speed.
    An omitted center uses the existing hover position as circle entry or
    figure-eight crossing. Figure-eight revolutions count complete cycles.
    """

    mode: str = "hover"
    radius_m: float = .5
    speed_mps: float = .10
    revolutions: float = 1.
    ramp_s: float = 5.
    settle_s: float = 3.
    center_xy: object = None
    direction: str = "ccw"
    cone_half_angle_deg: float = 10.
    cone_period_s: float = 30.
    cone_transition_s: float = 5.

    def __post_init__(self):
        if not isinstance(self.mode, str) or self.mode not in ("hover", "circle", "figure_eight", "attitude_cone"):
            raise ValueError("trajectory mode must be hover, circle, figure_eight, or attitude_cone")
        if not isinstance(self.direction, str) or self.direction not in ("ccw", "cw"):
            raise ValueError("trajectory direction must be ccw or cw")
        for name in ("radius_m", "speed_mps", "revolutions", "ramp_s"):
            object.__setattr__(self, name, _scalar(getattr(self, name), name, 0., True))
        if self.mode in ("figure_eight", "attitude_cone") and not self.revolutions.is_integer():
            raise ValueError(self.mode + " revolutions must count complete integer cycles")
        for name in ("cone_half_angle_deg", "cone_period_s", "cone_transition_s"):
            object.__setattr__(self, name, _scalar(getattr(self, name), name, 0., True))
        if self.cone_half_angle_deg >= 90.:
            raise ValueError("cone_half_angle_deg must be below 90 degrees")
        object.__setattr__(self, "settle_s", _scalar(self.settle_s, "settle_s", 0.))
        if self.center_xy is not None:
            object.__setattr__(self, "center_xy", tuple(_array(self.center_xy, "center_xy", (2,))))

    @classmethod
    def from_mapping(cls, mapping):
        if not isinstance(mapping, Mapping):
            raise ValueError("trajectory must be a mapping")
        unknown = set(mapping) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError("unknown trajectory options: " + ", ".join(sorted(map(str, unknown))))
        return cls(**dict(mapping))


def _yaw_rotation(yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array([[cosine, -sine, 0.], [sine, cosine, 0.], [0., 0., 1.]])


def _speed_ramp_phase(elapsed, duration, ramp, angle, peak_rate):
    """Return angular displacement/rate/acceleration for one smooth arc."""
    if elapsed <= 0.:
        return 0., 0., 0.
    if elapsed >= duration:
        return angle, 0., 0.
    if ramp <= elapsed <= duration-ramp:
        return peak_rate*(elapsed-.5*ramp), peak_rate, 0.
    decelerating = elapsed > duration-ramp
    u = (duration-elapsed if decelerating else elapsed)/ramp
    blend = u**3*(10. + u*(-15. + 6.*u))
    integral = u**4*(2.5 + u*(-3. + u))
    blend_rate = 30.*u**2*(1.-u)**2/ramp
    displacement = peak_rate*ramp*integral
    acceleration = peak_rate*blend_rate
    if decelerating:
        displacement, acceleration = angle-displacement, -acceleration
    return displacement, peak_rate*blend, acceleration


class PayloadCircleReference:
    """Circle with smooth speed ramps and payload body +X pointing inward.

    The reference starts at the circle's leftmost point with yaw zero. Positive
    unwrapped yaw traverses counterclockwise. Each speed ramp uses a quintic
    smoothstep; its analytic integral determines position along the circle.
    ``settle_s`` is applied by the caller when choosing ``start_time``.
    """

    def __init__(self, hover_position, start_time, config):
        hover = _array(hover_position, "hover_position", (3,))
        self.start_time = _scalar(start_time, "start_time")
        if not isinstance(config, CircleTrajectoryConfig):
            raise ValueError("config must be a CircleTrajectoryConfig")
        self.config = config
        self._center = hover.copy()
        if config.center_xy is None:
            self._center[0] += config.radius_m
        else:
            self._center[:2] = config.center_xy
        self._entry = self._center - np.array([config.radius_m, 0., 0.])
        self._angle = 2.*math.pi*config.revolutions
        self._peak_rate = min(config.speed_mps/config.radius_m, self._angle/config.ramp_s)
        self.duration = self._angle/self._peak_rate + config.ramp_s
        self.end_time = self.start_time + self.duration
        self._sign = 1. if config.direction == "ccw" else -1.
        if not all(math.isfinite(value) for value in (self.duration, self.end_time)):
            raise ValueError("circle timing must remain finite")

    @property
    def entry_position(self):
        return self._entry.copy()

    @property
    def center_position(self):
        return self._center.copy()

    @property
    def initial_yaw(self):
        return 0.

    @property
    def initial_rotation(self):
        return np.eye(3)

    def _phase(self, elapsed):
        return _speed_ramp_phase(elapsed, self.duration, self.config.ramp_s,
                                 self._angle, self._peak_rate)

    def evaluate(self, now):
        now = _scalar(now, "now")
        elapsed = now-self.start_time
        angle, rate, angular_acceleration = self._phase(elapsed)
        yaw = self._sign*angle
        yaw_rate = self._sign*rate
        yaw_acceleration = self._sign*angular_acceleration
        cosine, sine = math.cos(yaw), math.sin(yaw)
        radial = np.array([cosine, sine, 0.])
        tangent = np.array([-sine, cosine, 0.])
        return {
            "position": self._center-self.config.radius_m*radial,
            "velocity": -self.config.radius_m*yaw_rate*tangent,
            "acceleration": self.config.radius_m*(yaw_rate**2*radial-yaw_acceleration*tangent),
            "rotation": _yaw_rotation(yaw),
            "yaw": yaw,
            "yaw_rate": yaw_rate,
            "yaw_acceleration": yaw_acceleration,
            "phase": ("waiting" if now < self.start_time else
                      "complete" if now >= self.end_time else "circle"),
            "progress": float(np.clip(angle/self._angle, 0., 1.)),
        }


class PayloadFigureEightReference:
    """Two exact circular lobes joined at a stationary crossing.

    The right lobe runs first in the configured direction. The left lobe runs
    in the opposite direction, preserving the tangent heading at the crossing.
    Every lobe starts and stops with zero speed and acceleration. Body +X follows
    the analytic tangent even at rest, so yaw stays continuous at each seam.
    """

    def __init__(self, hover_position, start_time, config):
        self._center = _array(hover_position, "hover_position", (3,))
        self.start_time = _scalar(start_time, "start_time")
        if not isinstance(config, CircleTrajectoryConfig) or config.mode != "figure_eight":
            raise ValueError("config must select figure_eight")
        self.config = config
        if config.center_xy is not None:
            self._center[:2] = config.center_xy
        self._lobe_centers = (self._center
                              + np.array([[config.radius_m, 0., 0.], [-config.radius_m, 0., 0.]]))
        self._peak_rate = min(config.speed_mps/config.radius_m, 2.*math.pi/config.ramp_s)
        self._lobe_duration = 2.*math.pi/self._peak_rate + config.ramp_s
        self.duration = 2.*config.revolutions*self._lobe_duration
        self.end_time = self.start_time+self.duration
        self._sign = 1. if config.direction == "ccw" else -1.
        if not all(math.isfinite(value) for value in (self.duration, self.end_time)):
            raise ValueError("figure-eight timing must remain finite")
        self._lobe_count = 2*int(config.revolutions)

    @property
    def entry_position(self):
        return self._center.copy()

    @property
    def center_position(self):
        return self._center.copy()

    @property
    def lobe_centers(self):
        return self._lobe_centers.copy()

    @property
    def initial_yaw(self):
        return -.5*self._sign*math.pi

    @property
    def initial_rotation(self):
        return _yaw_rotation(self.initial_yaw)

    def evaluate(self, now):
        now = _scalar(now, "now")
        elapsed = float(np.clip(now-self.start_time, 0., self.duration))
        lobe = min(int(elapsed/self._lobe_duration), self._lobe_count-1)
        local_elapsed = elapsed-lobe*self._lobe_duration
        if now >= self.end_time:
            lobe, local_elapsed = self._lobe_count-1, self._lobe_duration
        angle, rate, angular_acceleration = _speed_ramp_phase(
            local_elapsed, self._lobe_duration, self.config.ramp_s, 2.*math.pi, self._peak_rate)
        index = lobe % 2
        side = -1. if index == 0 else 1.
        cosine, sine = math.cos(angle), math.sin(angle)
        radial = np.array([side*cosine, -self._sign*sine, 0.])
        tangent = np.array([-side*sine, -self._sign*cosine, 0.])
        curvature = np.array([-side*cosine, self._sign*sine, 0.])
        yaw = self.initial_yaw+self._sign*(angle if index == 0 else 2.*math.pi-angle)
        yaw_sign = self._sign if index == 0 else -self._sign
        return {
            "position": self._lobe_centers[index]+self.config.radius_m*radial,
            "velocity": self.config.radius_m*rate*tangent,
            "acceleration": self.config.radius_m*(rate**2*curvature+angular_acceleration*tangent),
            "rotation": _yaw_rotation(yaw),
            "yaw": yaw,
            "yaw_rate": yaw_sign*rate,
            "yaw_acceleration": yaw_sign*angular_acceleration,
            "phase": ("waiting" if now < self.start_time else
                      "complete" if now >= self.end_time else "figure_eight"),
            "progress": float(np.clip((lobe+angle/(2.*math.pi))/self._lobe_count, 0., 1.)),
            "lobe_index": index,
        }


def _right_jacobian_with_derivative(vector, vector_rate):
    """SO(3) right Jacobian and its time derivative along a rotation vector."""
    theta = float(np.linalg.norm(vector))
    theta_squared = theta*theta
    dot_squared = 2.*float(vector@vector_rate)
    if theta < .01:
        # Differentiate in theta squared to avoid dividing by theta at rest.
        a = .5+theta_squared*(-1./24.+theta_squared*(1./720.-theta_squared/40320.))
        b = 1./6.+theta_squared*(-1./120.+theta_squared*(1./5040.-theta_squared/362880.))
        a_dot = dot_squared*(-1./24.+theta_squared*(1./360.-theta_squared/13440.))
        b_dot = dot_squared*(-1./120.+theta_squared*(1./2520.-theta_squared/120960.))
    else:
        cosine, sine = math.cos(theta), math.sin(theta)
        a = (1.-cosine)/theta_squared
        b = (theta-sine)/(theta_squared*theta)
        theta_dot = float(vector@vector_rate)/theta
        a_dot = theta_dot*(theta*sine-2.*(1.-cosine))/(theta_squared*theta)
        b_dot = theta_dot*(theta*(1.-cosine)-3.*(theta-sine))/(theta_squared**2)
    cross, cross_dot = _hat(vector), _hat(vector_rate)
    squared_cross = cross@cross
    jacobian = np.eye(3)-a*cross+b*squared_cross
    derivative = (-a_dot*cross-a*cross_dot+b_dot*squared_cross
                  + b*(cross_dot@cross+cross@cross_dot))
    return jacobian, derivative


class AttitudeBrakeReference:
    """Bring an arbitrary initial body rate and acceleration smoothly to rest.

    The initial-frame rotation vector follows a quintic boundary profile and
    the attitude is ``R_initial @ exp(vector)``. The SO(3) right Jacobian maps
    vector derivatives to body angular velocity and acceleration. In general
    these differ from the rotation-vector derivatives when the axes vary.
    """

    def __init__(self, initial_rotation, initial_body_rate,
                 initial_body_acceleration, start_time, duration):
        self._initial_rotation = _rotation_matrix(
            _array(initial_rotation, "initial_rotation", (3, 3)), "initial_rotation").copy()
        self._initial_rate = _array(initial_body_rate, "initial_body_rate", (3,))
        self._initial_acceleration = _array(initial_body_acceleration,
                                           "initial_body_acceleration", (3,))
        self.start_time = _scalar(start_time, "start_time")
        self.duration = _scalar(duration, "duration", 0., True)
        self.end_time = self.start_time+self.duration
        # The requested quintic endpoint makes its fifth-order coefficient zero.
        # Use normalized time so short braking windows remain well conditioned.
        with np.errstate(over="ignore", invalid="ignore"):
            rate = self.duration*self._initial_rate
            acceleration = np.float64(self.duration)*self.duration*self._initial_acceleration
            self._endpoint = .5*rate+acceleration/12.
            self._coefficients = np.array([
                np.zeros(3), rate, .5*acceleration,
                -rate-(2./3.)*acceleration, .5*rate+.25*acceleration])
            endpoint_norm = float(np.linalg.norm(self._endpoint))
        if (not math.isfinite(self.end_time)
                or not np.all(np.isfinite(self._coefficients))
                or not math.isfinite(endpoint_norm)):
            raise ValueError("attitude brake timing and coefficients must remain finite")
        self._end_rotation = self._initial_rotation@_rotation_exp(self._endpoint)

    def evaluate(self, now):
        now = _scalar(now, "now")
        if now <= self.start_time:
            return dict(rotation=self._initial_rotation.copy(), body_rate=self._initial_rate.copy(),
                        body_rate_dot=self._initial_acceleration.copy())
        if now >= self.end_time:
            return dict(rotation=self._end_rotation.copy(), body_rate=np.zeros(3),
                        body_rate_dot=np.zeros(3))
        u = (now-self.start_time)/self.duration
        coefficients = self._coefficients
        vector = np.polynomial.polynomial.polyval(u, coefficients)
        vector_rate = np.polynomial.polynomial.polyval(
            u, coefficients[1:]*np.arange(1, len(coefficients))[:, None])/self.duration
        vector_acceleration = np.polynomial.polynomial.polyval(
            u, coefficients[2:]*(np.arange(2, len(coefficients))
                * np.arange(1, len(coefficients)-1))[:, None])/self.duration**2
        jacobian, jacobian_dot = _right_jacobian_with_derivative(vector, vector_rate)
        return dict(rotation=self._initial_rotation@_rotation_exp(vector),
                    body_rate=jacobian@vector_rate,
                    body_rate_dot=jacobian@vector_acceleration+jacobian_dot@vector_rate)


def rotate_vehicle_references(p_i, v_i, a_i, p_load, v_load, a_load,
                              yaw, yaw_rate, yaw_acceleration):
    """Rotate world-frame aircraft PVA about the moving payload reference.

    Aircraft arrays use one row per vehicle. The returned tuple contains new
    position, velocity, and acceleration arrays in that layout. This evaluates
    ``p_load + Q @ (p_i-p_load)`` and its full first/second derivatives, including
    the Coriolis term when the incoming relative formation is already moving.
    """
    positions = _array(p_i, "p_i")
    if positions.ndim != 2 or positions.shape[1] != 3 or positions.shape[0] == 0:
        raise ValueError("p_i must have shape (N, 3), N >= 1")
    velocities = _array(v_i, "v_i", positions.shape)
    accelerations = _array(a_i, "a_i", positions.shape)
    position = _array(p_load, "p_load", (3,))
    velocity = _array(v_load, "v_load", (3,))
    acceleration = _array(a_load, "a_load", (3,))
    yaw = _scalar(yaw, "yaw")
    yaw_rate = _scalar(yaw_rate, "yaw_rate")
    yaw_acceleration = _scalar(yaw_acceleration, "yaw_acceleration")
    rotation = _yaw_rotation(yaw)
    z_cross = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 0.]])
    rotation_dot = yaw_rate*z_cross @ rotation
    rotation_ddot = (yaw_acceleration*z_cross+yaw_rate**2*(z_cross @ z_cross)) @ rotation
    relative_position = positions-position
    relative_velocity = velocities-velocity
    relative_acceleration = accelerations-acceleration
    return (
        position + relative_position @ rotation.T,
        velocity + relative_velocity @ rotation.T + relative_position @ rotation_dot.T,
        acceleration + relative_acceleration @ rotation.T
        + 2.*relative_velocity @ rotation_dot.T + relative_position @ rotation_ddot.T,
    )
