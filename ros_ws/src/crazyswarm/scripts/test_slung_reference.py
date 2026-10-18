"""Attachment references must preserve measured cable geometry while leveling."""

import importlib

import numpy as np
import pytest


def _reference(*args, **kwargs):
    return importlib.import_module("slung_reference").attachment_rotation_reference(
        *args, **kwargs)


def _axis_rotation(axis, angle):
    """Independent axis-angle fixture; no production conversion helpers."""
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (np.cos(angle) * np.eye(3)
            + (1.0 - np.cos(angle)) * np.outer(axis, axis)
            + np.sin(angle) * cross)


def _blend(time, duration=2.7):
    u = time / duration
    return (10 * u**3 - 15 * u**4 + 6 * u**5,
            (30 * u**2 - 60 * u**3 + 30 * u**4) / duration,
            (60 * u - 180 * u**2 + 120 * u**3) / duration**2)


def _fixture():
    initial = _axis_rotation([1.0, -2.0, 0.5], 0.43)
    target = initial @ _axis_rotation([0.3, 0.5, -0.2], 0.81)
    attachments = np.array([[0.12, -0.06, -0.06],
                            [0.0, 0.104, -0.104],
                            [0.015, -0.01, 0.02]])
    return initial, target, attachments


def test_endpoint_references_preserve_initial_and_final_attachment_geometry():
    initial, target, attachments = _fixture()
    first = _reference(initial, target, attachments, 0.0)
    last = _reference(initial, target, attachments, 1.0)

    np.testing.assert_allclose(first["rotation"], initial, atol=1e-14)
    np.testing.assert_allclose(first["position_offsets"], 0.0, atol=1e-14)
    np.testing.assert_allclose(last["rotation"], target, atol=1e-14)
    np.testing.assert_allclose(last["position_offsets"],
                               target @ attachments - initial @ attachments,
                               atol=1e-14)
    for sample in (first, last):
        for key in ("omega_body", "alpha_body", "velocity_offsets",
                    "acceleration_offsets"):
            np.testing.assert_array_equal(sample[key], 0.0)


def test_vehicle_offsets_preserve_measured_cable_lengths_and_directions():
    initial, target, attachments = _fixture()
    payload_initial = np.array([0.2, -0.3, 0.8])[:, None]
    anchors = np.array([[0.4, 0.0, -0.03],
                        [-0.25, -0.1, -0.5],
                        [1.65, 1.59, 1.7]])
    initial_cables = anchors - payload_initial - initial @ attachments
    lengths = np.linalg.norm(initial_cables, axis=0)
    for time in np.linspace(0.0, 2.7, 15):
        sample = _reference(initial, target, attachments, *_blend(time))
        translation = np.array([0.1, -0.05, 0.3])[:, None] * (time / 2.7)
        vehicle_reference = anchors + translation + sample["position_offsets"]
        payload_reference = payload_initial + translation
        cables = (vehicle_reference - payload_reference
                  - sample["rotation"] @ attachments)
        np.testing.assert_allclose(cables, initial_cables, atol=2e-15)
        np.testing.assert_allclose(np.linalg.norm(cables, axis=0), lengths,
                                   atol=2e-15)
        np.testing.assert_allclose(cables / lengths, initial_cables / lengths,
                                   atol=2e-15)


def test_rotation_stays_on_so3_and_follows_the_short_geodesic():
    initial, _, attachments = _fixture()
    axis = np.array([2.0, -3.0, 4.0]) / np.sqrt(29.0)
    target = initial @ _axis_rotation(axis, 1.3)
    for blend in np.linspace(0.0, 1.0, 13):
        sample = _reference(initial, target, attachments, blend)
        rotation = sample["rotation"]
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=2e-14)
        assert np.linalg.det(rotation) == pytest.approx(1.0, abs=2e-14)
        np.testing.assert_allclose(rotation,
                                   initial @ _axis_rotation(axis, blend * 1.3),
                                   atol=2e-14)


def test_attachment_velocity_and_acceleration_match_time_derivatives():
    initial, target, attachments = _fixture()
    time, step = 1.03, 1e-4
    before = _reference(initial, target, attachments, *_blend(time - step))
    middle = _reference(initial, target, attachments, *_blend(time))
    after = _reference(initial, target, attachments, *_blend(time + step))

    velocity_fd = (after["position_offsets"] - before["position_offsets"]) / (2 * step)
    acceleration_fd = (after["position_offsets"] - 2 * middle["position_offsets"]
                       + before["position_offsets"]) / step**2
    np.testing.assert_allclose(middle["velocity_offsets"], velocity_fd,
                               atol=2e-9, rtol=2e-7)
    np.testing.assert_allclose(middle["acceleration_offsets"], acceleration_fd,
                               atol=2e-8, rtol=3e-6)
    np.testing.assert_allclose(
        middle["acceleration_offsets"],
        (after["velocity_offsets"] - before["velocity_offsets"]) / (2 * step),
        atol=2e-9, rtol=2e-7)


def test_body_angular_rates_match_rotation_and_angular_velocity_derivatives():
    initial, target, attachments = _fixture()
    time, step = 0.81, 1e-5
    before = _reference(initial, target, attachments, *_blend(time - step))
    middle = _reference(initial, target, attachments, *_blend(time))
    after = _reference(initial, target, attachments, *_blend(time + step))
    rotation_dot = (after["rotation"] - before["rotation"]) / (2 * step)
    body_skew = middle["rotation"].T @ rotation_dot
    omega_fd = np.array([body_skew[2, 1], body_skew[0, 2], body_skew[1, 0]])

    np.testing.assert_allclose(middle["omega_body"], omega_fd, atol=2e-10)
    np.testing.assert_allclose(
        middle["alpha_body"],
        (after["omega_body"] - before["omega_body"]) / (2 * step),
        atol=2e-10)


@pytest.mark.parametrize("angle", [0.0, 1e-13, 1e-7, np.pi - 1e-7,
                                   np.pi - 1e-12, np.pi])
def test_identity_and_half_turns_remain_finite_with_correct_geodesic(angle):
    initial, _, attachments = _fixture()
    axis = np.array([2.0, -3.0, 4.0]) / np.sqrt(29.0)
    target = initial @ _axis_rotation(axis, angle)
    sample = _reference(initial, target, attachments, 0.37, 0.23, -0.17)

    for value in sample.values():
        assert np.all(np.isfinite(value))
    np.testing.assert_allclose(sample["rotation"],
                               initial @ _axis_rotation(axis, angle * 0.37),
                               atol=2e-9)
    np.testing.assert_allclose(sample["omega_body"], axis * angle * 0.23,
                               atol=2e-9)
    np.testing.assert_allclose(sample["alpha_body"], -axis * angle * 0.17,
                               atol=2e-9)


@pytest.mark.parametrize("angle", [np.pi - 1e-7, np.pi - 1e-13,
                                   np.pi + 1e-7])
def test_near_half_turn_keeps_resolvable_axis_sign_and_uses_shorter_path(angle):
    initial, _, attachments = _fixture()
    axis = np.array([-2.0, 3.0, -4.0]) / np.sqrt(29.0)
    target = initial @ _axis_rotation(axis, angle)
    sample = _reference(initial, target, attachments, 0.37, 0.23)
    shorter_angle = angle if angle <= np.pi else angle - 2 * np.pi

    np.testing.assert_allclose(
        sample["rotation"], initial @ _axis_rotation(axis, shorter_angle * 0.37),
        atol=2e-9)
    np.testing.assert_allclose(sample["omega_body"], axis * shorter_angle * 0.23,
                               atol=2e-9)


def test_quintic_endpoints_have_zero_reference_velocity_and_acceleration():
    initial, target, attachments = _fixture()
    for time in (0.0, 2.7):
        sample = _reference(initial, target, attachments, *_blend(time))
        for key in ("omega_body", "alpha_body", "velocity_offsets",
                    "acceleration_offsets"):
            np.testing.assert_allclose(sample[key], 0.0, atol=1e-14)


@pytest.mark.parametrize("argument,value", [
    ("initial_rotation", np.full((3, 3), np.nan)),
    ("target_rotation", np.full((3, 3), np.inf)),
    ("attachment_points", np.array([[0.0], [np.nan], [0.0]])),
    ("blend", np.nan),
    ("blend_rate", np.inf),
    ("blend_acceleration", -np.inf),
    ("blend", -0.01),
    ("blend", 1.01),
    ("blend", [0.5]),
    ("initial_rotation", np.eye(2)),
    ("target_rotation", np.diag([1.0, 1.0, -1.0])),
    ("target_rotation", np.diag([1.0, 1.0, 1.01])),
    ("attachment_points", np.zeros((2, 3))),
    ("attachment_points", np.zeros((3, 0))),
])
def test_invalid_inputs_are_rejected_before_producing_references(argument, value):
    initial, target, attachments = _fixture()
    arguments = dict(initial_rotation=initial, target_rotation=target,
                     attachment_points=attachments, blend=0.5,
                     blend_rate=0.2, blend_acceleration=-0.1)
    arguments[argument] = value
    with pytest.raises(ValueError):
        _reference(**arguments)


def test_inputs_are_not_modified_or_reused_as_mutable_outputs():
    initial, target, attachments = _fixture()
    originals = [value.copy() for value in (initial, target, attachments)]
    sample = _reference(initial, target, attachments, 0.0)
    sample["rotation"][:] = 0.0
    sample["position_offsets"][:] = 3.0
    for actual, expected in zip((initial, target, attachments), originals):
        np.testing.assert_array_equal(actual, expected)


def _direction_reference(*args, **kwargs):
    return importlib.import_module("slung_reference").cable_direction_reference(
        *args, **kwargs)


def _direction_fixture():
    initial = np.array([[-0.42, 0.22, 0.24],
                        [0.11, -0.29, 0.22],
                        [-0.89, -0.93, -0.94]])
    target = np.array([[-0.36, 0.18, 0.19],
                       [0.0, -0.34, 0.33],
                       [-0.94, -0.92, -0.925]])
    initial /= np.linalg.norm(initial, axis=0)
    target /= np.linalg.norm(target, axis=0)
    return initial, target


def test_cable_direction_endpoints_match_requested_unit_geometry():
    initial, target = _direction_fixture()
    for blend, expected in ((0.0, initial), (1.0, target)):
        sample = _direction_reference(initial, target, blend)
        np.testing.assert_allclose(sample["directions"], expected, atol=2e-15)
        np.testing.assert_array_equal(sample["direction_rates"], 0.0)
        np.testing.assert_array_equal(sample["direction_accelerations"], 0.0)


def test_cable_directions_follow_short_spherical_arc_at_constant_blend_rate():
    initial = np.array([[0.0], [0.0], [-1.0]])
    angle = 0.63
    target = np.array([[np.sin(angle)], [0.0], [-np.cos(angle)]])
    for blend in np.linspace(0.0, 1.0, 9):
        sample = _direction_reference(initial, target, blend, 0.3, -0.2)
        phi = angle * blend
        expected = np.array([[np.sin(phi)], [0.0], [-np.cos(phi)]])
        tangent = np.array([[np.cos(phi)], [0.0], [np.sin(phi)]])
        np.testing.assert_allclose(sample["directions"], expected, atol=2e-15)
        np.testing.assert_allclose(sample["direction_rates"], 0.3 * angle * tangent,
                                   atol=2e-15)
        np.testing.assert_allclose(
            sample["direction_accelerations"],
            -0.2 * angle * tangent - (0.3 * angle)**2 * expected, atol=2e-15)


def test_cable_direction_rates_and_accelerations_match_time_derivatives():
    initial, target = _direction_fixture()
    time, step = 1.03, 1e-4
    before = _direction_reference(initial, target, *_blend(time - step))
    middle = _direction_reference(initial, target, *_blend(time))
    after = _direction_reference(initial, target, *_blend(time + step))
    rate_fd = (after["directions"] - before["directions"]) / (2 * step)
    acceleration_fd = (after["directions"] - 2 * middle["directions"]
                       + before["directions"]) / step**2
    np.testing.assert_allclose(middle["direction_rates"], rate_fd,
                               atol=2e-9, rtol=2e-7)
    np.testing.assert_allclose(middle["direction_accelerations"], acceleration_fd,
                               atol=5e-8, rtol=5e-6)
    np.testing.assert_allclose(
        middle["direction_accelerations"],
        (after["direction_rates"] - before["direction_rates"]) / (2 * step),
        atol=2e-9, rtol=2e-7)


def test_cable_direction_derivatives_respect_unit_sphere_constraints():
    initial, target = _direction_fixture()
    for time in np.linspace(0.0, 2.7, 15):
        sample = _direction_reference(initial, target, *_blend(time))
        q = sample["directions"]
        qd = sample["direction_rates"]
        qdd = sample["direction_accelerations"]
        np.testing.assert_allclose(np.linalg.norm(q, axis=0), 1.0, atol=2e-15)
        np.testing.assert_allclose(np.sum(q * qd, axis=0), 0.0, atol=2e-15)
        np.testing.assert_allclose(np.sum(q * qdd, axis=0),
                                   -np.sum(qd * qd, axis=0), atol=2e-15)


def test_combined_attitude_and_direction_reference_preserves_cable_lengths():
    initial_rotation, target_rotation, attachments = _fixture()
    initial_directions, target_directions = _direction_fixture()
    lengths = np.array([0.850, 0.848, 0.853])
    payload_initial = np.array([0.2, -0.3, 0.8])[:, None]
    measured_aircraft = (payload_initial + initial_rotation @ attachments
                         - lengths * initial_directions)
    for time in np.linspace(0.0, 2.7, 15):
        blend = _blend(time)
        attitude = _reference(initial_rotation, target_rotation, attachments, *blend)
        cable = _direction_reference(initial_directions, target_directions, *blend)
        translation = np.array([0.1, -0.05, 0.3])[:, None] * (time / 2.7)
        aircraft_reference = (measured_aircraft + translation
                              + attitude["position_offsets"]
                              - lengths * (cable["directions"] - initial_directions))
        physical_links = (payload_initial + translation
                           + attitude["rotation"] @ attachments - aircraft_reference)
        np.testing.assert_allclose(np.linalg.norm(physical_links, axis=0), lengths,
                                   atol=2e-15)
        np.testing.assert_allclose(physical_links / lengths, cable["directions"],
                                   atol=2e-15)


def test_quintic_cable_direction_endpoints_have_zero_rates_and_accelerations():
    initial, target = _direction_fixture()
    for time in (0.0, 2.7):
        sample = _direction_reference(initial, target, *_blend(time))
        np.testing.assert_allclose(sample["direction_rates"], 0.0, atol=2e-15)
        np.testing.assert_allclose(sample["direction_accelerations"], 0.0, atol=2e-15)


def test_unchanged_cable_directions_have_zero_motion_at_any_blend_rate():
    initial, _ = _direction_fixture()
    sample = _direction_reference(initial, initial, 0.37, 0.8, -0.6)
    np.testing.assert_allclose(sample["directions"], initial, atol=2e-15)
    np.testing.assert_array_equal(sample["direction_rates"], 0.0)
    np.testing.assert_array_equal(sample["direction_accelerations"], 0.0)


def test_antipodal_cable_directions_use_deterministic_perpendicular_axis():
    initial = np.array([[0.0], [0.0], [-1.0]])
    target = -initial
    sample = _direction_reference(initial, target, 0.5, 0.2)
    np.testing.assert_allclose(sample["directions"], [[1.0], [0.0], [0.0]], atol=2e-15)
    np.testing.assert_allclose(sample["direction_rates"],
                               [[0.0], [0.0], [0.2 * np.pi]], atol=2e-15)
    np.testing.assert_allclose(sample["direction_accelerations"],
                               [[-(0.2 * np.pi)**2], [0.0], [0.0]], atol=2e-15)


@pytest.mark.parametrize("angle", [1e-13, np.pi - 1e-7, np.pi - 1e-13])
def test_near_identity_and_antipodal_directions_keep_resolvable_arc_sign(angle):
    initial = np.array([[0.0], [0.0], [-1.0]])
    target = np.array([[-np.sin(angle)], [0.0], [-np.cos(angle)]])
    sample = _direction_reference(initial, target, 0.37, 0.23, -0.17)
    phi = 0.37 * angle
    expected = np.array([[-np.sin(phi)], [0.0], [-np.cos(phi)]])
    np.testing.assert_allclose(sample["directions"], expected, atol=2e-15)
    for value in sample.values():
        assert np.isfinite(value).all()


@pytest.mark.parametrize("argument,value", [
    ("initial_directions", np.full((3, 3), np.nan)),
    ("target_directions", np.full((3, 3), np.inf)),
    ("initial_directions", np.zeros((3, 3))),
    ("target_directions", np.full((3, 3), 1.0)),
    ("initial_directions", np.zeros((2, 3))),
    ("initial_directions", np.zeros((3, 0))),
    ("target_directions", np.array([[0.0], [0.0], [-1.0]])),
    ("blend", np.nan),
    ("blend", -0.01),
    ("blend", 1.01),
    ("blend", [0.5]),
    ("blend_rate", np.inf),
    ("blend_acceleration", -np.inf),
])
def test_invalid_cable_direction_inputs_are_rejected(argument, value):
    initial, target = _direction_fixture()
    arguments = dict(initial_directions=initial, target_directions=target,
                     blend=0.5, blend_rate=0.2, blend_acceleration=-0.1)
    arguments[argument] = value
    with pytest.raises(ValueError):
        _direction_reference(**arguments)


def test_cable_direction_inputs_are_not_modified_or_shared_with_outputs():
    initial, target = _direction_fixture()
    originals = [value.copy() for value in (initial, target)]
    sample = _direction_reference(initial, target, 0.0)
    for value in sample.values():
        value[:] = 3.0
    for actual, expected in zip((initial, target), originals):
        np.testing.assert_array_equal(actual, expected)
