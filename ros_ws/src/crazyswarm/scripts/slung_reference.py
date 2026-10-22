"""ROS-independent attitude and cable references for a suspended payload."""

import numpy as np


def _finite_array(value, name):
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError("{} must contain finite numbers".format(name)) from error
    if not np.all(np.isfinite(array)):
        raise ValueError("{} must contain finite numbers".format(name))
    return array


def _rotation_matrix(value, name):
    rotation = _finite_array(value, name)
    if rotation.shape != (3, 3):
        raise ValueError("{} must have shape (3, 3)".format(name))
    if (not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0.0, atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0, rtol=0.0, atol=1e-6)):
        raise ValueError("{} must be a proper rotation matrix".format(name))
    return rotation


def _scalar(value, name):
    array = _finite_array(value, name)
    if array.shape != ():
        raise ValueError("{} must be a finite scalar".format(name))
    return float(array)


def _hat(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _rotation_vector(rotation):
    """Principal SO(3) logarithm, including identity and pi rotations."""
    sine_axis = 0.5 * np.array([rotation[2, 1] - rotation[1, 2],
                                rotation[0, 2] - rotation[2, 0],
                                rotation[1, 0] - rotation[0, 1]])
    sine = np.linalg.norm(sine_axis)
    cosine = np.clip(0.5 * (np.trace(rotation) - 1.0), -1.0, 1.0)
    angle = np.arctan2(sine, cosine)
    if angle < 1e-8:
        return sine_axis * (1.0 + angle * angle / 6.0)
    if np.pi - angle < 1e-5:
        # The skew part vanishes at pi. The symmetric part retains the axis
        # as its unit eigenvector with eigenvalue one.
        _, eigenvectors = np.linalg.eigh(0.5 * (rotation + rotation.T))
        axis = eigenvectors[:, -1]
        if sine > 8.0 * np.finfo(float).eps:
            if np.dot(axis, sine_axis) < 0.0:
                axis = -axis
        elif axis[np.argmax(np.abs(axis))] < 0.0:
            # At exactly pi both signs are valid; choose one deterministically.
            axis = -axis
        return angle * axis
    return (angle / sine) * sine_axis


def _rotation_exp(vector):
    angle = np.linalg.norm(vector)
    cross = _hat(vector)
    if angle < 1e-8:
        angle_squared = angle * angle
        a = 1.0 - angle_squared / 6.0
        b = 0.5 - angle_squared / 24.0
    else:
        a = np.sin(angle) / angle
        b = (1.0 - np.cos(angle)) / (angle * angle)
    return np.eye(3) + a * cross + b * (cross @ cross)


def attachment_rotation_reference(initial_rotation, target_rotation,
                                  attachment_points, blend, blend_rate=0.0,
                                  blend_acceleration=0.0):
    """Blend payload attitude and return consistent attachment reference offsets.

    ``initial_rotation`` and ``target_rotation`` map payload body coordinates
    into world coordinates. ``attachment_points`` has shape ``(3, N)`` with
    one body-frame attachment per column. ``blend`` is in [0, 1]; its first
    and second time derivatives are supplied in inverse seconds and inverse
    seconds squared. A quintic smoothstep gives zero rates at both endpoints.

    The result contains ``rotation``, ``omega_body``, ``alpha_body``, and
    ``position_offsets``, ``velocity_offsets``, ``acceleration_offsets``. The
    angular rates use the reference payload body frame, with
    ``rotation_dot = rotation @ hat(omega_body)``. All offsets are world-frame
    arrays of shape ``(3, N)`` relative to the initial rotated attachments.

    For measured aircraft anchors ``x_h`` and a payload translation ``dp``,
    use ``x_ref = x_h + dp + position_offsets``. This preserves each initial
    world-frame cable vector as the payload follows the attitude reference.
    Add the translation's velocity and acceleration to the corresponding
    offset derivatives. Invalid/nonfinite inputs raise ``ValueError``.

    The shorter SO(3) geodesic is used. Exactly pi admits two equally short
    paths; the axis whose largest component is positive selects the path.
    """
    initial = _rotation_matrix(initial_rotation, "initial_rotation")
    target = _rotation_matrix(target_rotation, "target_rotation")
    attachments = _finite_array(attachment_points, "attachment_points")
    if attachments.ndim != 2 or attachments.shape[0] != 3 or attachments.shape[1] == 0:
        raise ValueError("attachment_points must have shape (3, N), with N >= 1")
    s = _scalar(blend, "blend")
    sd = _scalar(blend_rate, "blend_rate")
    sdd = _scalar(blend_acceleration, "blend_acceleration")
    if not 0.0 <= s <= 1.0:
        raise ValueError("blend must be in [0, 1]")

    rotation_vector = _rotation_vector(initial.T @ target)
    rotation = initial @ _rotation_exp(s * rotation_vector)
    if s == 0.0:
        rotation = initial.copy()
    elif s == 1.0:
        rotation = target.copy()

    # Along a one-axis geodesic the body angular velocity and acceleration
    # commute, so their coordinates are simply the fixed log vector's rates.
    omega_body = sd * rotation_vector
    alpha_body = sdd * rotation_vector
    omega_cross = _hat(omega_body)
    alpha_cross = _hat(alpha_body)
    return {
        "rotation": rotation,
        "omega_body": omega_body,
        "alpha_body": alpha_body,
        "position_offsets": (rotation - initial) @ attachments,
        "velocity_offsets": rotation @ omega_cross @ attachments,
        "acceleration_offsets": rotation @ (alpha_cross + omega_cross @ omega_cross) @ attachments,
    }


def _unit_direction_columns(value, name):
    directions = _finite_array(value, name)
    if directions.ndim != 2 or directions.shape[0] != 3 or directions.shape[1] == 0:
        raise ValueError("{} must have shape (3, N), with N >= 1".format(name))
    lengths = np.linalg.norm(directions, axis=0)
    if not np.allclose(lengths, 1.0, rtol=0.0, atol=1e-6):
        raise ValueError("{} columns must be unit vectors".format(name))
    return directions / lengths


def cable_direction_reference(initial_directions, target_directions, blend,
                              blend_rate=0.0, blend_acceleration=0.0):
    """Blend world-frame unit cable directions along their shorter S2 arcs.

    Both direction inputs have shape ``(3, N)`` with matching column counts.
    Norm roundoff up to ``1e-6`` is accepted and normalized; other invalid or
    nonfinite inputs raise ``ValueError``. ``blend`` is in [0, 1], with its
    first and second time derivatives supplied by ``blend_rate`` and
    ``blend_acceleration``. A quintic blend has zero endpoint derivatives.

    Return ``directions``, ``direction_rates``, ``direction_accelerations``,
    each a world-frame ``(3, N)`` array. Each column follows a fixed-axis
    Rodrigues rotation with its analytical first and second derivatives.
    Unchanged directions remain stationary for any supplied blend rates.

    Exact antipodes admit multiple equally short arcs. Choose their rotation
    axis as ``normalize(cross(initial, basis))``, where ``basis`` is the first
    Cartesian unit vector least aligned with ``initial``. Resolvable cross
    products near antipodes retain their sign. Downward suspension cables
    normally avoid this ambiguous case.

    For directions pointing from aircraft to payload, use the aircraft
    position correction ``-length * (directions - initial_directions)`` in
    addition to the payload attachment's rotation/translation correction.
    Its velocity and acceleration corrections are ``-length`` times the
    corresponding direction derivatives. This preserves cable lengths.
    """
    initial = _unit_direction_columns(initial_directions, "initial_directions")
    target = _unit_direction_columns(target_directions, "target_directions")
    if initial.shape != target.shape:
        raise ValueError("initial_directions and target_directions must have matching shapes")
    s = _scalar(blend, "blend")
    sd = _scalar(blend_rate, "blend_rate")
    sdd = _scalar(blend_acceleration, "blend_acceleration")
    if not 0.0 <= s <= 1.0:
        raise ValueError("blend must be in [0, 1]")

    cross = np.cross(initial.T, target.T).T
    sine = np.linalg.norm(cross, axis=0)
    cosine = np.clip(np.sum(initial * target, axis=0), -1.0, 1.0)
    angles = np.arctan2(sine, cosine)
    axes = np.zeros_like(initial)
    resolvable = sine > 8.0 * np.finfo(float).eps
    axes[:, resolvable] = cross[:, resolvable] / sine[resolvable]
    for index in np.flatnonzero(~resolvable):
        if cosine[index] >= 0.0:
            angles[index] = 0.0
            continue
        basis = np.zeros(3)
        basis[np.argmin(np.abs(initial[:, index]))] = 1.0
        axis = np.cross(initial[:, index], basis)
        axes[:, index] = axis / np.linalg.norm(axis)

    tangent_initial = np.cross(axes.T, initial.T).T
    directions = (initial * np.cos(s * angles)
                  + tangent_initial * np.sin(s * angles))
    if s == 0.0:
        directions = initial.copy()
    elif s == 1.0:
        directions = target.copy()
    tangent = np.cross(axes.T, directions.T).T
    curvature = np.cross(axes.T, tangent.T).T
    return {
        "directions": directions,
        "direction_rates": tangent * (sd * angles),
        "direction_accelerations": (tangent * (sdd * angles)
                                    + curvature * (sd * angles)**2),
    }
