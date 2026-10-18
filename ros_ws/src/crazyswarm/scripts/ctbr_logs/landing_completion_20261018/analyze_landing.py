#!/usr/bin/env python3
"""Offline audit of the 2026-10-18 22:24:18 aircraft landing stall.

Only reads CSV and writes analysis artifacts beside this script. No ROS/hardware.
The flight gains below are independently checked against logged force values;
using fixed values keeps this report reproducible after controller tuning.
"""

import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "multi_ctbr_20261018_222418.csv"
MASS = {3: .0434, 4: .0440, 5: .0424}
GRAVITY = 9.80665
KP_Z, KV_Z, KI_Z, I_LIMIT_Z = 4., 3.6, .8, .2
LANDING_ALTITUDE_TOLERANCE_M = .08
FIELDS = (
    "position_z", "target_z", "position_error_z", "target_velocity_z",
    "target_acceleration_z", "velocity_z", "filtered_velocity_z",
    "control_velocity_z", "velocity_error_z", "position_integral_z",
    "desired_force_z", "command_thrust_newton", "command_raw_thrust",
    "battery_voltage_v", "preflight_voltage_v", "thrust_raw_scale",
    "position_error_x", "position_error_y", "transport_position_hold_force_z",
    "prelift_vertical_integral_force_n", "state_age_s", "mocap_sample_age_s",
    "ekf_state_age_s", "bridge_latest_command_age_s",
)


def number(rows, field):
    return np.array([float(row.get(field) or "nan") for row in rows])


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(mean=float(np.mean(values)), std=float(np.std(values)),
                min=float(np.min(values)), max=float(np.max(values)))


def subset(rows, start, end):
    return [row for row in rows if start <= float(row["control_time_s"]) <= end]


def first_time(rows, predicate):
    return next((float(row["control_time_s"]) for row in rows if predicate(row)), None)


def window_stats(rows):
    result = {field: stats(number(rows, field)) for field in FIELDS}
    time = number(rows, "control_time_s")
    if len(time) > 1:
        result["z_regression_slope_mps"] = float(np.polyfit(
            time-time[0], number(rows, "position_z"), 1)[0])
    result["samples"] = len(rows)
    return result


def analyze_vehicle(rows, vehicle_id):
    descent = [row for row in rows
               if row["flight_validation_landing_stage"] == "vehicle_descent"]
    time = number(descent, "control_time_s")
    mass = MASS[vehicle_id]
    ground_row = next(row for row in rows if row["flight_phase"] == "pre_takeoff_hold")
    ground_z = float(ground_row["position_z"])
    z_target = number(descent, "target_z")
    ep = number(descent, "position_error_z")
    ev = number(descent, "velocity_error_z")
    integral = number(descent, "position_integral_z")
    force_terms = dict(
        gravity=np.full(len(descent), mass*GRAVITY),
        target_acceleration=mass*number(descent, "target_acceleration_z"),
        position=-mass*KP_Z*ep,
        velocity=-mass*KV_Z*ev,
        integral=-mass*KI_Z*integral,
    )
    reconstructed = sum(force_terms.values())
    residual = number(descent, "desired_force_z") - reconstructed
    assert np.max(np.abs(residual)) < 1e-10, (vehicle_id, np.max(np.abs(residual)))
    assert abs(z_target[-1]-ground_z) < 1e-12
    reference_complete = first_time(descent, lambda row: (
        abs(float(row["target_z"])-ground_z) < 1e-8
        and abs(float(row["target_velocity_z"])) < 1e-8))
    last_five = subset(descent, time[-1]-5, time[-1])
    last_twenty = subset(descent, time[-1]-20, time[-1])
    final = {field: descent[-1][field] for field in (
        "control_time_s", "flight_phase", "flight_validation_landing_stage",
        "position_z", "target_z", "position_error_z", "position_integral_z",
        "command_thrust_newton", "command_raw_thrust", "omega_c_method",
    )}
    minimum_index = int(np.argmin(number(descent, "position_z")))
    min_clearance = float(number(descent, "position_z")[minimum_index]-ground_z)
    mean_terms = {key: stats(values[time >= time[-1]-5]) for key, values in force_terms.items()}
    mean_ep = float(np.mean(number(last_five, "position_error_z")))
    mean_i = float(np.mean(number(last_five, "position_integral_z")))
    result = dict(
        row_count=len(rows), descent_row_count=len(descent),
        descent_start_s=float(time[0]), descent_end_s=float(time[-1]),
        descent_duration_s=float(time[-1]-time[0]),
        preflight_ground_anchor_z_m=ground_z,
        ground_measurement_2_to_3s=stats(number(subset(rows, 2, 3), "position_z")),
        final_target_minus_ground_anchor_m=float(z_target[-1]-ground_z),
        reference_complete_s=reference_complete,
        recorded_time_after_reference_complete_s=float(time[-1]-reference_complete),
        first_positive_integral_limit_s=first_time(descent, lambda row:
            float(row["position_integral_z"]) >= I_LIMIT_Z-1e-10),
        integral_at_release=float(integral[0]),
        max_integral_compensation_n=mass*KI_Z*I_LIMIT_Z,
        max_integral_equivalent_position_offset_m=KI_Z*I_LIMIT_Z/KP_Z,
        min_clearance_above_ground_origin_m=min_clearance,
        minimum_z_time_s=float(time[minimum_index]),
        closest_approach_margin_outside_landing_gate_m=min_clearance-LANDING_ALTITUDE_TOLERANCE_M,
        xy_target_peak_to_peak_m=[float(np.ptp(number(descent, "target_"+axis)))
                                   for axis in "xy"],
        force_law_residual_n=stats(residual),
        last5s_force_terms_n=mean_terms,
        last5s=window_stats(last_five), last20s=window_stats(last_twenty),
        # Algebraic counterfactual only: hold the observed average Fz/velocity,
        # eliminate z error, and solve the same law for I. Not a plant estimate.
        required_integral_for_same_command_at_zero_z_error=mean_i+KP_Z/KI_Z*mean_ep,
        phase_counts=dict(Counter(row["flight_phase"] for row in descent)),
        method_counts=dict(Counter(row["omega_c_method"] for row in descent)),
        status_counts={field: dict(Counter(row[field] for row in descent)) for field in (
            "state_valid", "derivatives_valid", "filter_derivatives_valid",
            "ekf_state_valid", "ekf_kinematics_held", "ekf_status",
            "invalid_reason", "global_abort_reason", "payload_safety_reason",
            "bridge_latest_accepted",
        )},
        final=final,
    )
    windows = [(54.955,56), (58,60), (62,64), (65,67), (70,72), (75,77), (80,82), (85,87.7)]
    result["windows"] = {f"{start}-{end}s": window_stats(subset(descent,start,end))
                         for start,end in windows}
    return result, descent, force_terms


def plot(descent_by_vehicle, forces_by_vehicle):
    fig, axes = plt.subplots(4, 3, figsize=(15, 12), sharex="col", constrained_layout=True)
    for column, vehicle_id in enumerate(MASS):
        rows = descent_by_vehicle[vehicle_id]
        time = number(rows, "control_time_s")
        ground_z = number(rows, "target_z")[-1]
        ax = axes[0, column]
        ax.plot(time, number(rows,"position_z"), label="Measured z")
        ax.plot(time, number(rows,"target_z"), "--", label="Reference z")
        ax.axhline(ground_z, color="k", lw=.8, label="Preflight ground origin")
        ax.axhline(ground_z+LANDING_ALTITUDE_TOLERANCE_M, color="0.5",ls=":",lw=.8,label="Landing altitude gate")
        ax.set(title=f"CF{vehicle_id}", ylabel="z (m)")
        ax = axes[1,column]
        ax.plot(time, number(rows,"control_velocity_z"),label="Control vz")
        ax.plot(time, number(rows,"target_velocity_z"),"--",label="Reference vz")
        ax.axhline(0,color="0.5",lw=.6)
        ax.set_ylabel("Vertical velocity (m/s)")
        ax = axes[2,column]
        ax.plot(time, number(rows,"position_integral_z"),label="Vertical I state")
        ax.axhline(I_LIMIT_Z,color="k",ls=":",label="I limit")
        ax.axhline(-I_LIMIT_Z,color="k",ls=":")
        ax.set_ylabel("Vertical integral")
        ax = axes[3,column]
        for key in ("position","velocity","integral","target_acceleration"):
            ax.plot(time,1000*forces_by_vehicle[vehicle_id][key],label=key)
        ax.plot(time,1000*(number(rows,"desired_force_z")-MASS[vehicle_id]*GRAVITY),
                color="k",lw=1,label="Total minus mg")
        ax.set(xlabel="Control time (s)",ylabel="Feedback force (mN)")
        for row in range(4):
            axes[row,column].grid(alpha=.25)
            axes[row,column].legend(fontsize=7,loc="best")
    fig.suptitle("22:24:18 flight: correct ground target, saturated vertical integral, persistent airborne offset")
    fig.savefig(HERE/"landing_analysis.png",dpi=160)
    plt.close(fig)


def write_notes(metrics):
    vehicles = metrics["vehicles"]
    lines = [
        "# 2026-10-18 22:24:18 三机降落停滞审计",
        "",
        "输入是当前 ctbr_logs 根目录最新实飞 CSV：`multi_ctbr_20261018_222418.csv`。"
        "运行 `python analyze_landing.py` 重建 metrics.json 与 landing_analysis.png；"
        "本目录只做离线分析，不调用 ROS 或硬件，不修改控制器或参数。",
        "",
        "## 结论",
        "",
        "三机已于 t=54.955 s 切到 `independent_matlab`，各自保持释放点 XY 并下降。"
        "终点 z 精确等于该机起飞前测得的地面原点，约 t=60.7–60.9 s 已走完下降参考；"
        "随后仍在地面目标上方约 0.12–0.19 m 波动。未发现地面目标被误设成悬停高度。",
        "",
        "垂直积分确实存在，但独立控制限幅为 +0.20，积分力最大只约 7 mN；"
        "达到上限后无法继续吸收偏差。三机最低高度也未进入现有 0.08 m 接地门限，"
        "因而一直停留在 landing，相同固定位置目标与受限 PID 没有继续减小推力的机制。"
        "这足以解释降落流程为何不结束。真实推力与命令之间的偏差来源仍不能仅凭 CSV 唯一确定。",
        "",
        "|量|CF3|CF4|CF5|",
        "|---|---:|---:|---:|",
    ]
    table = [
        ("起飞前地面原点 / 最终目标 z (m)",lambda v:v["preflight_ground_anchor_z_m"],6),
        ("下降参考完成 (s)",lambda v:v["reference_complete_s"],3),
        ("参考完成后继续记录 (s)",lambda v:v["recorded_time_after_reference_complete_s"],2),
        ("最后 5 s 实际 z 均值 (m)",lambda v:v["last5s"]["position_z"]["mean"],6),
        ("最后 5 s 高于地面目标 (m)",lambda v:v["last5s"]["position_error_z"]["mean"],6),
        ("最低点高于地面原点 (m)",lambda v:v["min_clearance_above_ground_origin_m"],6),
        ("首次 I 达到 +0.20 (s)",lambda v:v["first_positive_integral_limit_s"],3),
        ("最后 5 s I 均值",lambda v:v["last5s"]["position_integral_z"]["mean"],6),
        ("最后 5 s 推力命令均值 (N)",lambda v:v["last5s"]["command_thrust_newton"]["mean"],6),
        ("最后 5 s raw 推力均值",lambda v:v["last5s"]["command_raw_thrust"]["mean"],1),
        ("冻结 raw scale",lambda v:v["last5s"]["thrust_raw_scale"]["mean"],6),
        ("最后 5 s 电池电压均值 (V)",lambda v:v["last5s"]["battery_voltage_v"]["mean"],4),
        ("最大积分力幅值 (N)",lambda v:v["max_integral_compensation_n"],6),
        ("最后 5 s P 力均值 (N)",lambda v:v["last5s_force_terms_n"]["position"]["mean"],6),
        ("最后 5 s I 力均值 (N)",lambda v:v["last5s_force_terms_n"]["integral"]["mean"],6),
    ]
    for title, getter, decimals in table:
        lines.append("|"+title+"|"+"|".join(f"{getter(vehicles[str(i)]):.{decimals}f}" for i in MASS)+"|")
    lines.extend([
        "",
        "## 力律复核",
        "",
        "记录与以下独立控制力律在整个 vehicle_descent 阶段逐点一致，最大误差小于 10⁻¹⁰ N：",
        "",
        "`Fz = m × (9.80665 + ad_z − 4 ep_z − 3.6 ev_z − 0.8 Iz)`。",
        "",
        "因此释放后没有额外的协同支撑残差：该残差可直接用 Fz 减去以上各项重建。"
        "`transport_position_hold_force_z`、`prelift_vertical_integral_force_n` 也全为 0。"
        "这里的 Fz 和 command_thrust_newton 是主机期望力/推力命令，不能当成实测升力。",
        "",
        "积分最大加速度补偿为 0.8×0.2=0.16 m/s²，约 1.63% 重力；"
        "按 Kpz=4 换算，只相当于 0.04 m 的 P 项高度偏差。"
        "最后 5 s 仍有约 0.12/0.19/0.13 m 偏差，P 项仍在提供额外负向力。"
        "若保持日志平均命令/速度并仅把 z 误差变为 0，代数上所需 I 为 "
        + "/".join(f"{vehicles[str(i)]['required_integral_for_same_command_at_zero_z_error']:.3f}" for i in MASS)
        + "，显著超过 0.20；这只是同一力律的代数比较，不是实际飞机辨识或增益建议。",
        "",
        "## 状态与限制",
        "",
        "下降全过程 NOKOV/状态有效列均为 1，未触发全局中止。CF3/4 的 EKF 始终 fresh；"
        "CF5 仅 6 个周期使用 held_last_valid，其余 fresh，最后 5 s 全部 fresh。"
        "三机 XY 目标峰峰值均为 0，说明持续保持释放点。"
        "飞行末尾仍为 landing / vehicle_descent，未进入 landing_settle 或 landed。",
        "",
        "静态推力标定、近地气流、机载推力映射等物理因素在这个 CSV 中没有独立实测量，"
        "不能指定唯一物理根因。可确认的软件问题是：地面目标结束后依赖带很小积分上限的"
        "位置保持控制，允许一个高于地面且一直不满足落地门限的持久平衡状态。",
        "",
        "`metrics.json` 保留各时窗高度、参考速度/加速度、各速度来源、积分、原始推力、"
        "电压、状态新鲜度和分项力；`landing_analysis.png` 显示高度、速度、积分及分项反馈力。",
    ])
    (HERE/"analysis.md").write_text("\n".join(lines)+"\n",encoding="utf-8")


def main():
    with SOURCE.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    root_files = sorted(HERE.parent.glob("*.csv"),key=lambda path:path.stat().st_mtime,reverse=True)
    vehicle_metrics, descent_by_vehicle, forces_by_vehicle = {}, {}, {}
    for vehicle_id in MASS:
        own_rows = [row for row in rows if row["vehicle_id"] == str(vehicle_id)]
        result, descent, terms = analyze_vehicle(own_rows, vehicle_id)
        vehicle_metrics[str(vehicle_id)] = result
        descent_by_vehicle[vehicle_id] = descent
        forces_by_vehicle[vehicle_id] = terms
    cf3 = [row for row in rows if row["vehicle_id"] == "3"]
    metrics = dict(
        source=str(SOURCE), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        newest_root_csv_by_mtime=root_files[0].name,
        row_count=len(rows),
        verified_flight_parameters=dict(mass_kg=MASS, gravity_mps2=GRAVITY,
            independent_position_gain_z=KP_Z, independent_velocity_gain_z=KV_Z,
            independent_integral_gain_z=KI_Z, independent_integral_limit_z=I_LIMIT_Z,
            landing_altitude_tolerance_m=LANDING_ALTITUDE_TOLERANCE_M),
        timeline=dict(
            payload_lowering_s=first_time(cf3,lambda row:row["flight_validation_landing_stage"]=="payload_lowering"),
            payload_unloading_s=first_time(cf3,lambda row:row["payload_landing_unloading"]=="1"),
            vehicle_descent_s=first_time(cf3,lambda row:row["flight_validation_landing_stage"]=="vehicle_descent")),
        vehicles=vehicle_metrics,
    )
    (HERE/"metrics.json").write_text(json.dumps(metrics,indent=2,allow_nan=False)+"\n")
    write_notes(metrics)
    plot(descent_by_vehicle, forces_by_vehicle)
    print("Saved metrics.json, analysis.md, landing_analysis.png")
    for vehicle_id, result in vehicle_metrics.items():
        print(f"CF{vehicle_id}: ground={result['preflight_ground_anchor_z_m']:.6f} m, "
              f"last5 error={result['last5s']['position_error_z']['mean']:.6f} m, "
              f"max force-law residual={result['force_law_residual_n']['max']:.3g} N")


if __name__ == "__main__":
    main()
