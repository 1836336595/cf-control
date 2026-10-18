"""Payload moments must track desired body rates in the measured body frame."""

import numpy as np

from slung_load_controller import (
    SlungLoadConfig, SlungLoadController, attachment_points_ros_z_up,
)


# Uniform 23 g box, dimensions 0.08 x 0.06 x 0.05 m; kg m^2.
INERTIA = np.diag([1.1691666666666667e-5,
                   1.7058333333333333e-5, 1.9166666666666667e-5])


def _rotation(vector):
    angle = float(np.linalg.norm(vector))
    axis = np.asarray(vector, dtype=float) / angle
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (np.cos(angle) * np.eye(3)
            + (1.0 - np.cos(angle)) * np.outer(axis, axis)
            + np.sin(angle) * skew)


def _case():
    config = SlungLoadConfig(
        payload_mass_kg=0.023, gravity_mps2=9.80665,
        payload_size_m=np.array([0.08, 0.06, 0.05]),
        attachment_points_m=attachment_points_ros_z_up([0.08, 0.06, 0.05]),
        link_length_m=0.850, position_gain=np.zeros(3),
        velocity_gain=np.zeros(3), integral_gain=np.zeros(3),
        integral_limit=np.ones(3), c1=0.5, force_norm_epsilon=1e-9,
        tension_pinv_tolerance=1e-9, link_kq=40.0, link_komega=20.0,
        link_integral_gain=0.0, link_integral_limit=np.ones(3),
    )
    payload = dict(valid=True, position=np.zeros(3), velocity=np.zeros(3),
                   rotation=np.eye(3), body_rate=np.array([0.7, -0.5, 0.4]))
    vehicles = [dict(position=config.attachment_points_m[:, index] + [0, 0, 0.850],
                     velocity=np.zeros(3), mass=0.043)
                for index in range(3)]
    target = dict(position=np.zeros(3), velocity=np.zeros(3),
                  acceleration=np.zeros(3), rotation=np.eye(3),
                  body_rate=np.zeros(3), body_rate_dot=np.zeros(3),
                  payload_attitude_gain=np.zeros(3),
                  payload_rate_gain=np.zeros(3))
    return SlungLoadController(config), payload, vehicles, target


def test_stationary_reference_cancels_rigid_body_gyroscopic_acceleration():
    controller, payload, vehicles, target = _case()
    result = controller.compute(payload, vehicles, target, 1.0 / 70.0)

    # With feedback disabled, a fixed rate reference commands zero angular
    # acceleration even when the measured body spins about multiple axes.
    np.testing.assert_allclose(result["desired_moment"],
                               [-4.2166666666666667e-7, -2.093e-6,
                                -1.8783333333333333e-6],
                               rtol=1e-12, atol=1e-18)
    omega = payload["body_rate"]
    angular_acceleration = np.linalg.solve(
        INERTIA, result["desired_moment"] - np.cross(omega, INERTIA @ omega))
    np.testing.assert_allclose(angular_acceleration, 0.0, atol=1e-14)


def test_moving_reference_uses_derivative_of_transported_desired_body_rate():
    controller, payload, vehicles, target = _case()
    payload["rotation"] = _rotation(np.array([0.2, -0.1, 0.3]))
    target["rotation"] = _rotation(np.array([-0.3, 0.4, 0.1]))
    target["body_rate"] = np.array([-0.35, 0.8, 0.25])
    target["body_rate_dot"] = np.array([0.16, -0.11, 0.24])
    target["payload_attitude_gain"] = np.array([1.3e-5, 2.1e-5, 1.7e-5])
    target["payload_rate_gain"] = np.array([2.2e-5, 1.6e-5, 1.9e-5])
    result = controller.compute(payload, vehicles, target, 1.0 / 70.0)

    rotation, desired_rotation = payload["rotation"], target["rotation"]
    omega, desired_omega = payload["body_rate"], target["body_rate"]
    desired_alpha = target["body_rate_dot"]
    step = 1e-5

    def desired_rate_in_moving_measured_body(time):
        measured_at_time = rotation @ _rotation(time * omega)
        desired_at_time = desired_rotation @ _rotation(
            time * desired_omega + 0.5 * time**2 * desired_alpha)
        return measured_at_time.T @ desired_at_time @ (
            desired_omega + time * desired_alpha)

    transported_rate_derivative = (
        desired_rate_in_moving_measured_body(step)
        - desired_rate_in_moving_measured_body(-step)) / (2 * step)
    relative = desired_rotation.T @ rotation
    attitude_error = 0.5 * np.array([
        relative[2, 1] - relative[1, 2],
        relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1],
    ])
    rate_error = omega - rotation.T @ desired_rotation @ desired_omega
    feedback_acceleration = np.linalg.solve(
        INERTIA, -target["payload_attitude_gain"] * attitude_error
        - target["payload_rate_gain"] * rate_error)
    angular_acceleration = np.linalg.solve(
        INERTIA, result["desired_moment"] - np.cross(omega, INERTIA @ omega))
    np.testing.assert_allclose(angular_acceleration,
                               transported_rate_derivative + feedback_acceleration,
                               rtol=2e-10, atol=2e-10)
