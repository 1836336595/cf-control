"""Reference cascade separates aircraft inertia support from load feedback."""

from dataclasses import replace

import numpy as np
import pytest

from slung_load_controller import (
    SlungLoadConfig, SlungLoadController, attachment_points_ros_z_up,
)


def _rotation(vector):
    angle = np.linalg.norm(vector)
    if angle < 1e-14:
        return np.eye(3)
    axis = np.asarray(vector) / angle
    x, y, z = axis
    cross = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (np.cos(angle) * np.eye(3)
            + (1 - np.cos(angle)) * np.outer(axis, axis)
            + np.sin(angle) * cross)


def _config():
    return SlungLoadConfig(
        payload_mass_kg=0.023, gravity_mps2=9.80665,
        payload_size_m=np.array([0.08, 0.06, 0.05]),
        attachment_points_m=attachment_points_ros_z_up([0.08, 0.06, 0.05]),
        link_length_m=0.850, position_gain=np.array([3.0, 3.0, 3.75]),
        velocity_gain=np.full(3, 3.12), integral_gain=np.zeros(3),
        integral_limit=np.ones(3), c1=0.5, force_norm_epsilon=1e-9,
        tension_pinv_tolerance=1e-9, link_kq=40.0, link_komega=20.0,
        link_integral_gain=0.0, link_integral_limit=np.ones(3),
    )


def _case(mode=None):
    config = _config()
    if mode is not None:
        config = replace(config, transport_force_mode=mode)
    payload = dict(valid=True, position=np.zeros(3), velocity=np.zeros(3),
                   rotation=_rotation([0.1, -0.2, 0.15]),
                   body_rate=np.array([0.6, -0.4, 0.8]))
    vehicles = [dict(position=payload["rotation"] @ config.attachment_points_m[:, index]
                     + np.array([0.15 * (index - 1), 0.12, 0.84]),
                     velocity=np.array([0.05, -0.03, 0.01]), mass=0.043)
                for index in range(3)]
    target = dict(position=np.zeros(3), velocity=np.zeros(3),
                  acceleration=np.array([0.12, -0.08, 0.09]),
                  rotation=_rotation([-0.3, 0.25, 0.2]),
                  body_rate=np.array([-0.2, 0.3, 0.15]),
                  body_rate_dot=np.array([0.4, -0.1, 0.2]),
                  payload_attitude_gain=np.full(3, 2e-4),
                  payload_rate_gain=np.full(3, 7e-5), link_gain_scale=1.0)
    return SlungLoadController(config), payload, vehicles, target


def test_reference_support_does_not_inject_measured_load_feedback_acceleration():
    first, payload, vehicles, target = _case("reference_cascade")
    second, changed, _, _ = _case("reference_cascade")
    changed.update(position=np.array([0.04, -0.03, 0.05]),
                   velocity=np.array([0.1, -0.15, 0.12]),
                   rotation=_rotation([0.5, 0.2, -0.4]),
                   body_rate=np.array([-1.3, 0.7, -0.9]))
    baseline = first.compute(payload, vehicles, target, 1 / 70)
    disturbed = second.compute(changed, vehicles, target, 1 / 70)

    assert np.linalg.norm(baseline["desired_moment"] - disturbed["desired_moment"]) > 1e-5
    assert np.linalg.norm(baseline["desired_force"] - disturbed["desired_force"]) > 1e-3
    for before, after in zip(baseline["vehicles"], disturbed["vehicles"]):
        np.testing.assert_allclose(before["kinematic_specific_acceleration"],
                                   after["kinematic_specific_acceleration"], atol=1e-14)
    assert baseline["force_mode"] == "reference_cascade"


def test_reference_attachment_specific_acceleration_matches_position_curvature():
    controller, payload, vehicles, target = _case("reference_cascade")
    result = controller.compute(payload, vehicles, target, 1 / 70)
    step = 1e-4
    rho = controller.config.attachment_points_m

    def reference_attachment_positions(time):
        position = (target["position"] + time * target["velocity"]
                    + 0.5 * time**2 * target["acceleration"])
        rotation = target["rotation"] @ _rotation(
            time * target["body_rate"] + 0.5 * time**2 * target["body_rate_dot"])
        return position[:, None] + rotation @ rho

    acceleration = (reference_attachment_positions(step)
                    - 2 * reference_attachment_positions(0)
                    + reference_attachment_positions(-step)) / step**2
    acceleration[2] += controller.config.gravity_mps2
    for index, output in enumerate(result["vehicles"]):
        np.testing.assert_allclose(output["kinematic_specific_acceleration"],
                                   acceleration[:, index], atol=4e-9, rtol=2e-7)


def test_cascade_bypasses_direction_correction_with_full_requested_link_gain():
    controller, payload, vehicles, target = _case("reference_cascade")
    result = controller.compute(payload, vehicles, target, 1 / 70)

    assert any(np.linalg.norm(output["link_direction_error"]) > 0.05
               for output in result["vehicles"])
    for output in result["vehicles"]:
        np.testing.assert_array_equal(output["raw_link_force"], np.zeros(3))
        np.testing.assert_array_equal(output["link_force"], np.zeros(3))
        assert output["desired_tension"] >= 0


def test_cascade_retains_cable_centripetal_support_without_double_counting_gravity():
    controller, payload, vehicles, target = _case("reference_cascade")
    result = controller.compute(payload, vehicles, target, 1 / 70)
    omega_world = payload["rotation"] @ payload["body_rate"]
    for index, (vehicle, output) in enumerate(zip(vehicles, result["vehicles"])):
        rho_world = payload["rotation"] @ controller.config.attachment_points_m[:, index]
        relative = payload["position"] + rho_world - vehicle["position"]
        q = relative / np.linalg.norm(relative)
        relative_velocity = (payload["velocity"] + np.cross(omega_world, rho_world)
                             - vehicle["velocity"])
        qdot = (relative_velocity - q * np.dot(q, relative_velocity)) / np.linalg.norm(relative)
        # |q x qdot| = |qdot| for the tangent cable derivative.
        cable_centripetal = vehicle["mass"] * 0.850 * np.dot(qdot, qdot) * q
        expected = (-output["desired_tension"] * q + cable_centripetal
                    + vehicle["mass"] * output["kinematic_specific_acceleration"])
        np.testing.assert_allclose(output["force"], expected, atol=1e-14)
        assert np.linalg.norm(cable_centripetal) > 1e-5


def test_coupled_default_preserves_its_feedback_and_direction_correction():
    default, payload, vehicles, target = _case()
    explicit, _, _, _ = _case("coupled")
    baseline = default.compute(payload, vehicles, target, 1 / 70)
    selected = explicit.compute(payload, vehicles, target, 1 / 70)

    assert baseline["force_mode"] == "coupled"
    assert any(np.linalg.norm(output["raw_link_force"]) > 1e-4
               for output in baseline["vehicles"])
    for first, second in zip(baseline["vehicles"], selected["vehicles"]):
        np.testing.assert_array_equal(first["force"], second["force"])
    disturbed, changed, _, _ = _case()
    changed["body_rate"] = np.array([-1.3, 0.7, -0.9])
    result = disturbed.compute(changed, vehicles, target, 1 / 70)
    assert any(np.linalg.norm(first["kinematic_specific_acceleration"]
                              - second["kinematic_specific_acceleration"]) > 1e-2
               for first, second in zip(baseline["vehicles"], result["vehicles"]))


@pytest.mark.parametrize("mode", ["", "lee", "REFERENCE_CASCADE", None, 0, []])
def test_invalid_transport_force_modes_are_rejected(mode):
    with pytest.raises(ValueError, match="transport_force_mode"):
        replace(_config(), transport_force_mode=mode)
