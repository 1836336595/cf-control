#!/usr/bin/env python3
"""Analysis only: recover logged q geometry and estimate anchor-hold stiffness.

No ROS import, controller/config mutation, hardware connection or simulation.
All assumptions are frozen below so a later YAML edit cannot change the result.
"""

import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "multi_ctbr_20261018_211815.csv"
IDS = (3, 4, 5)
# Rows: CF3/4/5; configured geometric-center-relative payload anchors.
RHO = np.array([[.04, 0., .025], [-.04, .03, .025], [-.04, -.03, .025]])
VEHICLE_MASSES = np.array([.0434, .0440, .0424])
INDEPENDENT_KP = np.array([[3., 3., 4.], [4., 4., 4.], [4., 4., 4.]])
LEVEL_ROTATION = np.eye(3)  # Horizontal payload, yaw=0, frozen analysis assumption.


def vector(row, key):
    return np.array([float(row[key + "_" + axis]) for axis in "xyz"])


def sample_geometry(fleet, index):
    record = fleet[3][index]
    p0 = vector(record, "payload_position")
    aircraft = np.array([vector(fleet[i][index], "position") for i in IDS])
    q = np.array([vector(fleet[i][index], "link_direction") for i in IDS])

    def residual(rotvec):
        rotation = Rotation.from_rotvec(rotvec).as_matrix()
        links = p0 + RHO @ rotation.T - aircraft
        links /= np.linalg.norm(links, axis=1)[:, None]
        return (links - q).ravel()

    solution = least_squares(residual, np.zeros(3), xtol=1e-12,
                             ftol=1e-12, gtol=1e-12)
    rotation = Rotation.from_rotvec(solution.x).as_matrix()
    # At fixed load center and fixed world q, rotating the payload requires
    # aircraft to move by precisely the corresponding anchor displacement.
    delta_aircraft = RHO @ LEVEL_ROTATION.T - RHO @ rotation.T
    hold_scale = float(record["ramp_position_hold_gain_scale"])
    if float(record["transport_blend"]) >= 1.:
        hold_scale = float(record["transport_position_hold_gain_scale"])
    blend = float(record["transport_blend"])
    # Incremental position-only hold force at the hypothetical leveled pose.
    # This omits pre-existing position error, damping and all other force terms.
    full_blend_hold_force = (
        -hold_scale * VEHICLE_MASSES[:, None] * INDEPENDENT_KP * delta_aircraft
    )
    # Quasistatic ideal mapping: only the component along an unchanged q is
    # transferred through the massless taut cable. It is not measured tension.
    full_blend_load_force = (
        np.sum(full_blend_hold_force * q, axis=1)[:, None] * q
    )
    moment_world = np.sum(np.cross(RHO @ LEVEL_ROTATION.T,
                                  full_blend_load_force), axis=0)
    requested_body = vector(record, "payload_desired_moment")
    requested_world = rotation @ requested_body
    return {
        "control_time_s": float(record["control_time_s"]),
        "flight_phase": record["flight_phase"],
        "transport_blend": blend,
        "position_hold_gain_scale": hold_scale,
        "q_geometry_fit_residual_norm": float(np.linalg.norm(solution.fun)),
        "q_geometry_inferred_rpy_deg": Rotation.from_matrix(rotation).as_euler(
            "xyz", degrees=True).tolist(),
        "q_geometry_inferred_rotation": rotation.tolist(),
        "logged_link_direction": q.tolist(),
        "fixed_q_leveling_aircraft_displacement_m": delta_aircraft.tolist(),
        "full_blend_incremental_position_hold_force_n": full_blend_hold_force.tolist(),
        "full_blend_quasistatic_projected_load_force_n": full_blend_load_force.tolist(),
        "full_blend_quasistatic_restoring_moment_world_nm": moment_world.tolist(),
        "at_sample_blend_quasistatic_restoring_moment_world_nm": (blend * moment_world).tolist(),
        "logged_requested_moment_body_nm": requested_body.tolist(),
        "logged_requested_moment_world_nm": requested_world.tolist(),
    }


def main():
    records = list(csv.DictReader(SOURCE.open(newline="", encoding="utf-8")))
    fleet = {i: [row for row in records if row["vehicle_id"] == str(i)] for i in IDS}
    lengths = {len(rows) for rows in fleet.values()}
    if len(lengths) != 1:
        raise ValueError("Analysis requires equally sized, time-aligned fleet rows")
    for index in range(len(fleet[3])):
        stamps = [float(fleet[i][index]["control_time_s"]) for i in IDS]
        if max(stamps) - min(stamps) > 1e-6:
            raise ValueError("Fleet control timestamps are not aligned")
    handoff = next(i for i, row in enumerate(fleet[3])
                   if row["flight_phase"] == "tension_ramp")
    fault = next(i for i, row in enumerate(fleet[3])
                 if row["flight_phase"] == "emergency_landing")
    handoff_time = float(fleet[3][handoff]["control_time_s"])
    ramp_end = next(i for i in range(handoff, fault)
                    if float(fleet[3][i]["transport_blend"]) >= 1.)
    samples = [sample_geometry(fleet, i) for i in (handoff, ramp_end, fault - 1)]
    for item in samples:
        item["time_since_handoff_s"] = item["control_time_s"] - handoff_time
    result = {
        "source": str(SOURCE),
        "assumptions": {
            "vehicle_order": IDS, "payload_anchors_body_m": RHO.tolist(),
            "vehicle_masses_kg": VEHICLE_MASSES.tolist(),
            "independent_position_acceleration_gains": INDEPENDENT_KP.tolist(),
            "hypothetical_level_target_rotation": LEVEL_ROTATION.tolist(),
            "payload_geometric_center_equals_com": True,
            "aircraft_mocap_origin_equals_cable_endpoint": True,
        },
        "caveats": [
            "R is inverted from the controller's own q and logged positions; it is not an independent measurement of true physical attitude.",
            "Timestamp-asynchronous vehicle/payload snapshots can make the geometry inversion inconsistent; inspect fit residual.",
            "The finite anchor displacement preserves world q and cable length only if load center is fixed and both cable endpoints follow the ideal point model.",
            "Projected hold moment is a quasistatic sensitivity estimate, not a replay, actual tension measurement or prediction of closed-loop motion.",
            "Position-only incremental feedback excludes existing errors, velocity feedback, actuator dynamics, blend-history dynamics and changes of q along an actual transition.",
            "The first sample has almost zero transport blend: full-blend moment describes potential stiffness, not a force actually commanded then.",
            "Moment lever arms assume geometric-center anchors and COM=geometric center; real COM offset, hang hardware, aircraft endpoint offset or NOKOV frame bias can change these values.",
            "No claim is made that this mechanism alone explains the growing 2Hz instability.",
        ],
        "samples": samples,
    }
    destination = HERE / "geometry_conflict.json"
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    for item in samples:
        print("t=%.6f blend=%.6g fit=%.3g" % (item["time_since_handoff_s"],
              item["transport_blend"], item["q_geometry_fit_residual_norm"]))
        print("  leveling dz mm:", np.array(item["fixed_q_leveling_aircraft_displacement_m"])[:, 2] * 1000)
        print("  full-blend hold world moment mNm:", np.array(item["full_blend_quasistatic_restoring_moment_world_nm"]) * 1000)
        print("  requested world moment mNm:", np.array(item["logged_requested_moment_world_nm"]) * 1000)
    print(destination)


if __name__ == "__main__":
    main()
