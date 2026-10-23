"""Shared ROS-independent CTBR vector, rotation, and force-limit math."""

import math

import numpy as np


def as_vector(value, name):
    """把 ROS 参数或消息字段转成长度为 3 的有限浮点向量。"""
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.size != 3 or not np.all(np.isfinite(vector)):
        raise ValueError("%s 必须是 3 个有限数值" % name)
    return vector


def clamp_vector(vector, lower, upper):
    """逐元素限幅；所有积分器、角速度和推力相关向量均通过此函数防止发散。"""
    return np.minimum(np.maximum(vector, lower), upper)


def vee(matrix):
    """反对称矩阵到三维向量的 vee 映射。"""
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def project_so3(rotation):
    """用 SVD 将数值误差造成的近似旋转矩阵投影回右手 SO(3)。"""
    u_matrix, _, v_matrix = np.linalg.svd(rotation)
    result = u_matrix @ v_matrix
    if np.linalg.det(result) < 0.0:
        u_matrix[:, 2] *= -1.0
        result = u_matrix @ v_matrix
    return result


def rope_preserving_vehicle_references(
        payload_position, payload_velocity, payload_acceleration,
        payload_rotation, attachment_points_m, link_directions,
        link_distances_m):
    """Translate vehicle references while preserving measured rope geometry.

    ``link_directions`` point from each vehicle toward its payload attachment.
    Keeping those directions and distances fixed prevents the payload landing
    profile from pulling aircraft toward the load or shrinking their spacing.
    Returned arrays use one row per vehicle.
    """
    position = np.asarray(payload_position, dtype=float).reshape(3)
    velocity = np.asarray(payload_velocity, dtype=float).reshape(3)
    acceleration = np.asarray(payload_acceleration, dtype=float).reshape(3)
    rotation = project_so3(np.asarray(payload_rotation, dtype=float).reshape(3, 3))
    attachments = np.asarray(attachment_points_m, dtype=float)
    directions = np.asarray(link_directions, dtype=float)
    distances = np.asarray(link_distances_m, dtype=float).reshape(-1)
    if (attachments.shape != (3, 3) or directions.shape != (3, 3)
            or distances.shape != (3,)
            or not all(np.all(np.isfinite(value)) for value in
                       (position, velocity, acceleration, attachments,
                        directions, distances))
            or np.any(distances <= 0.0)):
        raise ValueError("invalid rope-preserving landing geometry")
    norms = np.linalg.norm(directions, axis=0)
    if np.any(norms <= 1.0e-9):
        raise ValueError("link directions must be nonzero")
    directions = directions / norms[None, :]
    attachment_world = position[:, None] + rotation @ attachments
    positions = (attachment_world - directions * distances[None, :]).T
    return positions, np.tile(velocity, (3, 1)), np.tile(acceleration, (3, 1))


def so3_log(rotation):
    """SO(3) 对数映射，返回可用于有限差分的旋转向量（rad）。"""
    cos_theta = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    theta = math.acos(cos_theta)
    if theta < 1e-7:
        return 0.5 * vee(rotation - rotation.T)
    if abs(math.pi - theta) < 1e-5:
        values, vectors = np.linalg.eigh((rotation + np.eye(3)) * 0.5)
        axis = vectors[:, int(np.argmax(values))]
        return theta * axis / max(float(np.linalg.norm(axis)), 1e-12)
    return theta * vee(rotation - rotation.T) / (2.0 * math.sin(theta))


def _limit_force_with_derivative(force, force_dot, max_command_thrust, max_tilt_rad):
    """Apply the force safety limits and differentiate the active branches.

    The analytic V2 feedforward must be differentiated from the same force that
    determines ``b3``.  Treating a clipped component as locally constant avoids
    asking the attitude loop to follow a direction that the safety limiter has
    already removed.
    """
    force = np.asarray(force, dtype=float).reshape(3).copy()
    force_dot = np.asarray(force_dot, dtype=float).reshape(3).copy()
    limited = force.copy()
    limited_dot = force_dot.copy()

    raw_vertical = float(force[2])
    limited[2] = float(np.clip(raw_vertical, 0.0, max_command_thrust))
    if raw_vertical <= 0.0 or raw_vertical >= max_command_thrust:
        limited_dot[2] = 0.0

    horizontal = force[:2].copy()
    horizontal_dot = force_dot[:2].copy()
    horizontal_norm = float(np.linalg.norm(horizontal))
    horizontal_limit = limited[2] * math.tan(float(max_tilt_rad))
    if horizontal_norm <= horizontal_limit:
        limited[:2] = horizontal
        limited_dot[:2] = horizontal_dot
    elif horizontal_norm <= 1e-12 or horizontal_limit <= 0.0:
        limited[:2] = 0.0
        limited_dot[:2] = 0.0
    else:
        norm_dot = float(horizontal @ horizontal_dot) / horizontal_norm
        scale = horizontal_limit / horizontal_norm
        scale_dot = -horizontal_limit * norm_dot / (horizontal_norm ** 2)
        limited[:2] = scale * horizontal
        limited_dot[:2] = scale_dot * horizontal + scale * horizontal_dot
    return limited, limited_dot


def analytic_omega_c(computed_rotation, desired_force, desired_force_dot, target, config):
    """Calculate V2's continuous-time ``Omega_c = vee(R_c.T @ R_c_dot)``.

    This implementation follows the MATLAB V2 column-derivative construction,
    with the sign changed for the ROS world-z-up convention (``b3 = F / ||F||``).
    The returned vector is expressed in the desired body frame.
    """
    force = np.asarray(desired_force, dtype=float).reshape(3)
    force_dot = np.asarray(desired_force_dot, dtype=float).reshape(3)
    if not np.all(np.isfinite(force)) or not np.all(np.isfinite(force_dot)):
        return np.zeros(3)
    force_norm = float(np.linalg.norm(force))
    force_epsilon = float(getattr(config, "omega_c_force_norm_epsilon", 1e-8))
    if force_norm < force_epsilon:
        return np.zeros(3)

    rotation = np.asarray(computed_rotation, dtype=float).reshape(3, 3)
    b3_desired = rotation[:, 2]
    b3_dot = (
        (np.eye(3) - np.outer(b3_desired, b3_desired)) @ force_dot
        / force_norm
    )

    yaw = float(target.get("yaw", 0.0))
    yaw_rate = float(target.get("yaw_rate", 0.0))
    b1_reference = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    b1_reference_dot = yaw_rate * np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    projection_matrix = np.eye(3) - np.outer(b3_desired, b3_desired)
    projection = projection_matrix @ b1_reference
    projection_norm = float(np.linalg.norm(projection))
    projection_epsilon = float(
        getattr(config, "omega_c_heading_projection_epsilon", 1e-6)
    )
    b1_desired = rotation[:, 0]
    if projection_norm < projection_epsilon:
        b1_dot = np.zeros(3)
    else:
        projection_dot = (
            -(np.outer(b3_dot, b3_desired) + np.outer(b3_desired, b3_dot))
            @ b1_reference
            + projection_matrix @ b1_reference_dot
        )
        b1_dot = (
            (np.eye(3) - np.outer(b1_desired, b1_desired))
            @ projection_dot
            / projection_norm
        )

    b2_desired = rotation[:, 1]
    b2_dot = np.cross(b3_dot, b1_desired) + np.cross(b3_desired, b1_dot)
    rotation_dot = np.column_stack((b1_dot, b2_dot, b3_dot))
    omega_hat = rotation.T @ rotation_dot
    omega_hat = 0.5 * (omega_hat - omega_hat.T)
    return vee(omega_hat)


def quaternion_to_rotation(x, y, z, w):
    """将 ROS ``[x,y,z,w]`` 四元数转换为机体系到 world 系的旋转矩阵。"""
    quaternion = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-8 or not np.all(np.isfinite(quaternion)):
        raise ValueError("收到无效姿态四元数")
    x, y, z, w = quaternion / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def rotation_to_rpy(rotation):
    """将机体系到 world 系旋转矩阵转为 ZYX ``[roll,pitch,yaw]``（rad）。"""
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return np.array([roll, pitch, yaw])


def phase_requires_full_controller_reset(phase):
    """Return whether a trajectory phase has a discontinuous/safe-state reset.

    Normal references meet continuously at takeoff correction, circle entry,
    circle, final hover and landing.  Their position integral represents the
    per-vehicle trim required to hover and must remain intact.  Only phases
    that intentionally leave normal closed-loop flight may clear it.
    """
    return str(phase) in ("emergency_landing", "landed")

