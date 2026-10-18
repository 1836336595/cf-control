#!/usr/bin/env python3
"""Analysis-only fixture: ideal force tracking versus requested acceleration.

Run with Python + NumPy. No controller or simulator files are modified. This
fixture deliberately excludes CTBR hold, support trim, actuator lag, clipping,
slack and external load moments. Its consistency result is not a stability test.
"""

import json
from pathlib import Path

import numpy as np


def hat(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotation(vector):
    vector = np.asarray(vector, dtype=float)
    angle = np.linalg.norm(vector)
    axis = vector / angle
    return (np.cos(angle) * np.eye(3)
            + (1.0 - np.cos(angle)) * np.outer(axis, axis)
            + np.sin(angle) * hat(axis))


def fixture():
    mass = 0.023
    vehicle_masses = np.array([0.0434, 0.0440, 0.0424])
    inertia = np.diag([1.1691666666666667e-5,
                       1.7058333333333333e-5, 1.9166666666666667e-5])
    rho = np.array([[0.04, -0.04, -0.04], [0.0, 0.03, -0.03],
                    [0.025, 0.025, 0.025]])
    orientation = rotation([0.12, 0.38, -0.16])
    omega = np.array([0.4, -0.3, 0.7])
    links = np.array([[-0.43, 0.22, 0.24], [0.11, -0.30, 0.22],
                      [-0.897, -0.928, -0.945]])
    links /= np.linalg.norm(links, axis=0)
    force_requested = np.array([0.0, 0.0, mass * 9.80665])
    moment_requested = np.array([-1.0e-4, -2.0e-4, -5.0e-5])

    allocation = np.zeros((6, 9))
    for index in range(3):
        allocation[:3, 3*index:3*index+3] = np.eye(3)
        allocation[3:, 3*index:3*index+3] = hat(rho[:, index])
    gram = allocation @ allocation.T
    desired_body = allocation.T @ np.linalg.solve(
        gram, np.r_[orientation.T @ force_requested, moment_requested])
    radial = rho.copy()
    radial[2] = 0.0
    radial /= np.linalg.norm(radial, axis=0)
    bias_seed = 0.035 * radial.T.reshape(-1)
    null_bias = bias_seed - allocation.T @ np.linalg.solve(
        gram, allocation @ bias_seed)
    desired_body += null_bias
    desired_world = orientation @ desired_body.reshape(3, 3).T
    tensions = np.maximum(-np.sum(links * desired_world, axis=0), 0.0)
    projected = -links * tensions
    force_projected = np.sum(projected, axis=1)
    moment_projected = sum(np.cross(rho[:, index], orientation.T @ projected[:, index])
                           for index in range(3))
    gyro = np.cross(omega, inertia @ omega)
    requested_y = np.r_[force_requested / mass,
                        np.linalg.solve(inertia, moment_requested - gyro)]
    projected_y = np.r_[force_projected / mass,
                        np.linalg.solve(inertia, moment_projected - gyro)]

    bare_mass = np.zeros((6, 6))
    bare_mass[:3, :3] = mass * np.eye(3)
    bare_mass[3:, 3:] = inertia
    coupling = np.zeros((6, 6))
    maps = []
    for index in range(3):
        point_map = np.column_stack((np.eye(3), -orientation @ hat(rho[:, index])))
        maps.append(point_map)
        projector = np.outer(links[:, index], links[:, index])
        coupling += vehicle_masses[index] * point_map.T @ projector @ point_map
    coupled_mass = bare_mass + coupling

    def forces_from_compensation(y):
        commands = np.empty((3, 3))
        for index in range(3):
            centripetal = orientation @ hat(omega) @ hat(omega) @ rho[:, index]
            commands[:, index] = (projected[:, index]
                                  + vehicle_masses[index] * (maps[index] @ y + centripetal))
        return commands

    def solve_coupled(commands):
        # Directly assemble the physical equations using u_parallel. This
        # avoids assuming that projected allocation equals physical tension.
        rhs = np.r_[np.zeros(3), -gyro]
        for index in range(3):
            q = links[:, index]
            projector = np.outer(q, q)
            centripetal = orientation @ hat(omega) @ hat(omega) @ rho[:, index]
            contribution = (projector @ commands[:, index]
                            - vehicle_masses[index] * projector @ centripetal)
            rhs[:3] += contribution
            rhs[3:] += hat(rho[:, index]) @ orientation.T @ contribution
        return np.linalg.solve(coupled_mass, rhs)

    actual_requested_compensation = solve_coupled(forces_from_compensation(requested_y))
    actual_projected_compensation = solve_coupled(forces_from_compensation(projected_y))
    old_residual = np.max(np.abs(actual_requested_compensation - projected_y))
    new_residual = np.max(np.abs(actual_projected_compensation - projected_y))
    assert new_residual < 1e-10
    assert old_residual > 0.1
    effective_inertia = (coupled_mass[3:, 3:]
                         - coupled_mass[3:, :3]
                         @ np.linalg.solve(coupled_mass[:3, :3], coupled_mass[:3, 3:]))
    result = dict(
        assumptions=["taut inextensible massless cables", "ideal aircraft force tracking",
                     "no CTBR hold or trim", "no actuator delay, clipping or external load moment",
                     "zero instantaneous cable angular rate; load body rate is nonzero"],
        acceleration_order="[world xddot + gravity, body omega_dot]",
        vehicle_masses_kg=vehicle_masses.tolist(),
        payload_mass_kg=mass, payload_inertia_kg_m2=inertia.tolist(),
        payload_rotation=orientation.tolist(), payload_body_rate=omega.tolist(),
        attachments_body_m=rho.tolist(), links_vehicle_to_load=links.tolist(),
        requested_force_n=force_requested.tolist(), requested_moment_nm=moment_requested.tolist(),
        projected_force_n=force_projected.tolist(), projected_moment_nm=moment_projected.tolist(),
        null_bias_wrench_residual=float(np.max(np.abs(allocation @ null_bias))),
        requested_acceleration=requested_y.tolist(),
        acceleration_from_projected_tensions=projected_y.tolist(),
        actual_with_requested_compensation=actual_requested_compensation.tolist(),
        actual_with_projected_compensation=actual_projected_compensation.tolist(),
        requested_compensation_residual_max=float(old_residual),
        projected_compensation_residual_max=float(new_residual),
        bare_generalized_mass=bare_mass.tolist(), aircraft_coupled_mass=coupling.tolist(),
        effective_inertia_free_translation=effective_inertia.tolist(),
        effective_to_bare_inertia_diagonal=(np.diag(effective_inertia) / np.diag(inertia)).tolist(),
        warning="Algebraic consistency is not delayed-loop stability; runtime probes rejected projected compensation.",
    )
    return result


if __name__ == "__main__":
    result = fixture()
    path = Path(__file__).with_name("coupled_kinematics_fixture.json")
    path.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: result[key] for key in (
        "requested_compensation_residual_max", "projected_compensation_residual_max",
        "requested_moment_nm", "projected_moment_nm", "effective_to_bare_inertia_diagonal")}, indent=2))
