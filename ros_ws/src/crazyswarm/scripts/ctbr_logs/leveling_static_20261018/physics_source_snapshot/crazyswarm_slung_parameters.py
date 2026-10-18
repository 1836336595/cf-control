"""CrazySwarm slung-load hardware and simulation parameters."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CrazyflieParameters:
    name: str
    mass: float
    max_total_thrust: float
    max_command_thrust: float
    min_flight_thrust: float
    max_tilt_rad: float
    max_body_rate: tuple[float, float, float]
    position_gain: tuple[float, float, float]
    velocity_gain: tuple[float, float, float]
    integral_gain: tuple[float, float, float]
    integral_limit: tuple[float, float, float]
    independent_position_gain: tuple[float, float, float]
    independent_velocity_gain: tuple[float, float, float]
    independent_integral_gain: tuple[float, float, float]
    independent_integral_limit: tuple[float, float, float]
    independent_integral_gate: float
    independent_max_feedback_acceleration: tuple[float, float, float]
    independent_attitude_gain: tuple[float, float, float]
    transport_attitude_gain: tuple[float, float, float]
    transport_rate_gain: tuple[float, float, float]


@dataclass(frozen=True)
class CrazySwarmSlungConfig:
    vehicles: tuple[CrazyflieParameters, ...]
    payload_mass: float = 0.054
    payload_size: tuple[float, float, float] = (0.08, 0.06, 0.05)
    cable_lengths: tuple[float, float, float] = (0.694, 0.692, 0.631)
    attachment_points: tuple[tuple[float, float, float], ...] = (
        (0.040, 0.000, 0.025),
        (-0.040, 0.030, 0.025),
        (-0.040, -0.030, 0.025),
    )
    timestep: float = 0.002
    # External CTBR loop; physics and motor/rate dynamics retain 2 ms substeps.
    control_rate_hz: float = 70.0
    thrust_time_constant: float = 0.012  # Legacy force-lag model only.
    rate_loop_bandwidth: tuple[float, float, float] = (35.0, 35.0, 20.0)
    # Select the onboard plant represented by the MuJoCo actuator bridge.
    onboard_controller: str = "mellinger"  # first_order / pid / mellinger
    # Graefe et al., arXiv:2603.05944, Eq. 5/9/10 and Table II.
    motor_model: str = "graefe2026"  # graefe2026 / legacy_quadratic
    brushless_motor_time_constant_s: float = 0.05
    brushless_motor_speed_gain_radps: float = 2900.0
    brushless_speed_normalization_radps: float = 2900.0
    brushless_rotor_inertia_kgm2: float = 5.0e-8
    # Descending powers (s^3, s^2, s); more precise author-code coefficients.
    brushless_thrust_coefficients: tuple[float, float, float] = (-0.23009526, 0.56176458, -0.0433191)
    brushless_torque_coefficients: tuple[float, float, float] = (-0.0003396, 0.00087032, 0.0002896)
    # Host collective calibration/cap and final per-motor range are distinct.
    bridge_collective_pwm_reference: float = 60000.0
    bridge_collective_pwm_limit: float = 60000.0
    bridge_thrust_curve_exponent: float = 2.0
    bridge_thrust_raw_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    motor_pwm_max: float = 65535.0
    # Author-style available-command ceiling; not a calibrated voltage model.
    motor_available_command_fraction: tuple[float, float, float] = (1.0, 1.0, 1.0)
    onboard_pid_rate_hz: float = 500.0
    onboard_pid_rate_kp: tuple[float, float, float] = (250.0, 250.0, 120.0)
    onboard_pid_rate_ki: tuple[float, float, float] = (500.0, 500.0, 16.7)
    onboard_pid_rate_kd: tuple[float, float, float] = (2.5, 2.5, 0.0)
    onboard_pid_rate_integral_limit: tuple[float, float, float] = (33.3, 33.3, 166.7)
    onboard_pid_yaw_gains: tuple[float, float, float] = (6.0, 1.0, 0.35)
    onboard_pid_yaw_integral_limit: float = 360.0
    mellinger_rate_hz: float = 500.0
    mellinger_attitude_gains: tuple[float, float, float] = (70000.0, 70000.0, 60000.0)
    mellinger_rate_gains: tuple[float, float, float] = (20000.0, 20000.0, 12000.0)
    mellinger_rate_derivative_gain: float = 200.0
    mellinger_yaw_integral_gain: float = 500.0
    mellinger_yaw_integral_limit: float = 1500.0
    motor_thrust_efficiency: tuple[float, float, float] = (1.0, 1.0, 1.0)
    motor_yaw_torque_coefficient: float = 0.006
    # Paper Table II with propeller guards; actual vehicle masses stay measured.
    vehicle_inertia: tuple[float, float, float] = (3.3e-5, 3.6e-5, 5.9e-5)
    body_size: tuple[float, float, float] = (0.050, 0.050, 0.014)
    arm_length: float = float(0.0707 / np.sqrt(2.0))  # Paper width l=7.07 cm.
    rotor_radius: float = 0.023
    prop_guard_outer_diameter: float = 0.052
    prop_guard_bar_radius: float = 0.001
    prop_guard_z_offset: float = 0.002
    prop_guard_segments: int = 16
    payload_position_gain: tuple[float, float, float] = (3.0, 3.0, 3.75)
    payload_velocity_gain: tuple[float, float, float] = (3.12, 3.12, 3.12)
    payload_integral_gain: tuple[float, float, float] = (1.60, 1.60, 1.60)
    payload_integral_limit: tuple[float, float, float] = (0.50, 0.50, 0.50)
    payload_integral_c1: float = 0.50
    payload_attitude_bandwidth_hz: tuple[float, float, float] = (2.0, 2.0, 0.45)
    payload_attitude_damping: float = 0.90
    payload_yaw_enabled: bool = True
    payload_rotational_damping: float = 1.0e-3
    tension_pinv_tolerance: float = 1.0e-9
    link_kq: float = 55.0
    link_komega: float = 20.0
    link_integral_gain: float = 0.0
    link_integral_limit: tuple[float, float, float] = (0.30, 0.30, 0.30)
    outward_bias_fraction: float = 0.25
    outward_bias_max_n: float = 0.15
    transport_link_gain_scale: float = 1.00
    ground_slack_duration: float = 0.50
    independent_hover_height: float = 0.50
    independent_takeoff_duration: float = 5.0
    takeup_duration: float = 8.0
    takeup_outward_offset: float = 0.12
    takeup_distance_tolerance: float = 0.01
    takeup_angle_tolerance_rad: float = float(np.deg2rad(10.0))
    takeup_confirm_duration: float = 0.50
    activation_hold_duration: float = 0.30
    tension_ramp_duration: float = 5.0
    ramp_position_hold_gain_scale: float = 2.0
    reference_lift_duration: float = 12.0
    # Payload center target height above the world ground in the default mission.
    hover_height: float = 0.60
    hover_duration: float = 30.0
    landing_minimum_duration: float = 4.0
    landing_max_speed: float = 0.25
    landing_approach_fraction: float = 0.55
    landing_settle_duration: float = 0.30
    landing_velocity_tolerance: float = 0.03
    payload_state_hold_duration: float = 0.20
    payload_state_estimator: str = "observer" # observer / second_order_low_pass / mujoco_truth
    payload_pose_sample_rate_hz: float = 100.0
    payload_velocity_filter_cutoff_hz: float = 3.0
    payload_observer_position_gain: float = 0.35
    # At 100 Hz, these gains reduce velocity/body-rate phase lag during lift.
    payload_observer_velocity_gain: float = 0.20
    payload_observer_acceleration_gain: float = 0.02
    payload_observer_attitude_gain: float = 0.35
    payload_observer_angular_rate_gain: float = 0.30
    payload_observer_max_dt: float = 0.05
    payload_observer_max_velocity_mps: tuple[float, float, float] = (1.0, 1.0, 1.0)
    payload_observer_max_acceleration_mps2: tuple[float, float, float] = (8.0, 8.0, 8.0)
    payload_observer_max_body_rate_rps: tuple[float, float, float] = (8.0, 8.0, 8.0)
    payload_observer_min_samples: int = 3
    emergency_land_after: float = 0.30
    ground_contact_tolerance: float = 0.003

    @property
    def vehicle_masses(self):
        return np.asarray([vehicle.mass for vehicle in self.vehicles])

    @property
    def attachment_array(self):
        return np.asarray(self.attachment_points, dtype=float)

    @property
    def payload_inertia(self):
        x, y, z = self.payload_size
        mass = self.payload_mass
        return np.array(
            [
                mass * (y * y + z * z) / 12.0,
                mass * (x * x + z * z) / 12.0,
                mass * (x * x + y * y) / 12.0,
            ]
        )


def default_config():
    """Return CrazySwarm's current CF3/CF4/CF5 hardware configuration."""
    shared = {
        "max_total_thrust": 1.176798,
        "max_command_thrust": 1.00,
        "min_flight_thrust": 0.30,
        "max_tilt_rad": float(np.deg2rad(10.0)),
        "max_body_rate": (3.0, 3.0, 2.0),
        "independent_integral_gain": (0.80, 0.80, 0.80),
        "independent_integral_limit": (0.20, 0.20, 0.20),
        "independent_integral_gate": 0.20,
        "independent_max_feedback_acceleration": (6.0, 6.0, 8.0),
        "independent_attitude_gain": (12.0, 12.0, 6.0),
        # From the real-controller replay with Graefe2026 + Mellinger: lower
        # delayed rate feedback; 20/10 ms delays and a 120 s run were checked.
        "transport_attitude_gain": (12.0, 12.0, 6.0),
        "transport_rate_gain": (0.25, 0.25, 0.125),
    }
    vehicles = (
        CrazyflieParameters(
            name="CF3",
            mass=0.0434,
            position_gain=(0.75, 0.75, 0.45),
            velocity_gain=(0.45, 0.45, 0.35),
            integral_gain=(0.40, 0.40, 0.20),
            integral_limit=(0.8, 0.8, 0.8),
            independent_position_gain=(3.0, 3.0, 4.0),
            independent_velocity_gain=(3.12, 3.12, 3.60),
            **shared,
        ),
        CrazyflieParameters(
            name="CF4",
            mass=0.0440,
            position_gain=(0.40, 0.44, 0.45),
            velocity_gain=(0.30, 0.38, 0.35),
            integral_gain=(0.03, 0.03, 0.20),
            integral_limit=(0.35, 0.35, 0.8),
            independent_position_gain=(4.0, 4.0, 4.0),
            independent_velocity_gain=(3.6, 3.6, 3.6),
            **shared,
        ),
        CrazyflieParameters(
            name="CF5",
            mass=0.0424,
            position_gain=(0.40, 0.42, 0.45),
            velocity_gain=(0.25, 0.25, 0.35),
            integral_gain=(0.05, 0.05, 0.20),
            integral_limit=(0.8, 0.8, 0.8),
            independent_position_gain=(4.0, 4.0, 4.0),
            independent_velocity_gain=(3.6, 3.6, 3.6),
            **shared,
        ),
    )
    return CrazySwarmSlungConfig(vehicles=vehicles)
