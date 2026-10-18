#!/usr/bin/env python3
"""Offline audit of the full 20 s reference-cascade flight.

No ROS/hardware imports. RPY is logged mocap-derived attitude. Projected cable
wrenches use commanded tension, and must not be read as measured physical force.
"""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "multi_ctbr_20261018_230530.csv"
PRIOR = HERE.parent / "multi_ctbr_20261018_222418.csv"
IDS = (3, 4, 5)
RHO = np.array([[.04, 0., .025], [-.04, .03, .025], [-.04, -.03, .025]])
WINDOWS = ((0., 5.), (5., 12.), (12., 16.), (16., 20.), (12., 20.))


def number(rows, name):
    return np.array([float(row.get(name) or "nan") for row in rows])


def vector(rows, prefix):
    return np.column_stack([number(rows, prefix + "_" + axis) for axis in "xyz"])


def rpy(rows, prefix):
    return np.column_stack([number(rows, prefix + axis + "_rad")
                            for axis in ("roll", "pitch", "yaw")])


def stats(values):
    return {name: function(values, axis=0).tolist() for name, function in (
        ("mean", np.nanmean), ("std", np.nanstd),
        ("min", np.nanmin), ("max", np.nanmax))}


def transitions(rows, name):
    return [dict(time_s=float(row["control_time_s"]), value=row.get(name, ""))
            for index, row in enumerate(rows)
            if not index or row.get(name, "") != rows[index - 1].get(name, "")]


def load(path):
    with path.open(newline="") as source:
        raw = list(csv.DictReader(source))
    fleet = {ident: [row for row in raw if row["vehicle_id"] == str(ident)] for ident in IDS}
    count = min(map(len, fleet.values()))
    aligned = {ident: rows[:count] for ident, rows in fleet.items()}
    rows = aligned[3]
    time = number(rows, "control_time_s")
    assert all(np.array_equal(time, number(aligned[ident], "control_time_s")) for ident in IDS)
    handoff = next(float(row["control_time_s"]) for row in rows if row["flight_phase"] == "tension_ramp")
    landing = next(float(row["control_time_s"]) for row in rows if row["flight_phase"] in ("landing", "emergency_landing"))
    positions = np.stack([vector(aligned[ident], "position") for ident in IDS], axis=1)
    return dict(raw=raw, fleet=fleet, aligned=aligned, rows=rows, time=time,
                handoff=handoff, landing=landing, rel=time-handoff, positions=positions,
                active=(time >= handoff) & (time < landing))


def main():
    data, prior = load(SOURCE), load(PRIOR)
    rows, rel, time, active = data["rows"], data["rel"], data["time"], data["active"]
    count = len(rows)
    fleet = data["aligned"]
    attitude, target = rpy(rows, "payload_"), rpy(rows, "payload_target_")
    rotations = Rotation.from_euler("xyz", attitude).as_matrix()
    target_rotations = Rotation.from_euler("xyz", target).as_matrix()
    tilt = np.rad2deg(np.arccos(np.clip(rotations[:, 2, 2], -1., 1.)))
    target_tilt = np.rad2deg(np.arccos(np.clip(target_rotations[:, 2, 2], -1., 1.)))
    progress = number(rows, "payload_attitude_reference_progress")
    hold = np.stack([vector(fleet[ident], "position_hold_position") for ident in IDS], axis=1)
    hold_error = data["positions"] - hold
    hold_force = np.stack([vector(fleet[ident], "transport_position_hold_force") for ident in IDS], axis=1)
    rates = np.stack([vector(fleet[ident], "body_rate") for ident in IDS], axis=1)
    thrust = np.column_stack([number(fleet[ident], "command_thrust_newton") for ident in IDS])
    trims = np.column_stack([number(fleet[ident], "transport_support_trim_n") for ident in IDS])
    q = np.stack([vector(fleet[ident], "link_direction") for ident in IDS], axis=1)
    qd = np.stack([vector(fleet[ident], "desired_link_direction") for ident in IDS], axis=1)
    tensions = np.column_stack([number(fleet[ident], "desired_tension_n") for ident in IDS])
    cable_angle = np.rad2deg(np.arccos(np.clip(np.sum(q * qd, axis=2), -1., 1.)))
    requested_moment = vector(rows, "payload_desired_moment")
    projected_moment = np.cross(RHO[None, :, :],
        np.einsum("nvi,nij->nvj", -tensions[:, :, None] * q, rotations)).sum(axis=1)
    payload_position = vector(rows, "payload_position")
    attachments = payload_position[:, None, :] + np.einsum("nij,vj->nvi", rotations, RHO)
    geometry_length = np.linalg.norm(attachments-data["positions"], axis=2)
    handoff_index = np.flatnonzero(active)[0]
    last_index = np.flatnonzero(active)[-1]
    d0 = np.linalg.norm(attachments[handoff_index]-hold[handoff_index], axis=1)
    distances = np.column_stack([np.linalg.norm(data["positions"][:, a]-data["positions"][:, b], axis=1)
                                 for a, b in ((0, 1), (0, 2), (1, 2))])

    def window(mask):
        result = dict(samples_per_vehicle=int(mask.sum()),
            actual_rpy_deg=stats(np.rad2deg(attitude[mask])),
            target_rpy_deg=stats(np.rad2deg(target[mask])),
            actual_tilt_deg=stats(tilt[mask]), target_tilt_deg=stats(target_tilt[mask]),
            attitude_reference_progress=stats(progress[mask]),
            payload_body_rate_rps=stats(vector(rows, "payload_body_rate")[mask]),
            payload_position_error_m=stats(vector(rows, "payload_position_error")[mask]),
            requested_body_moment_mnm=stats(requested_moment[mask]*1e3),
            projected_desired_tension_body_moment_mnm=stats(projected_moment[mask]*1e3),
            projected_minus_requested_body_moment_mnm=stats((projected_moment-requested_moment)[mask]*1e3),
            geometric_rope_lengths_m=stats(geometry_length[mask]),
            cable_angle_to_desired_deg=stats(cable_angle[mask]),
            pair_distances_m=stats(distances[mask]), vehicles={})
        for index, ident in enumerate(IDS):
            vehicle_rows = fleet[ident]
            integral = vector(vehicle_rows, "position_integral")[mask]
            desired_tilt = np.arccos(np.clip(np.cos(number(vehicle_rows, "desired_roll_rad")[mask])
                                           * np.cos(number(vehicle_rows, "desired_pitch_rad")[mask]), -1., 1.))
            cap = number(vehicle_rows, "transport_max_tilt_rad")[mask]
            result["vehicles"][ident] = dict(
                hold_position_error_m=stats(hold_error[mask, index]),
                hold_position_error_rms_m=np.sqrt(np.mean(hold_error[mask, index]**2, axis=0)).tolist(),
                hold_force_n=stats(hold_force[mask, index]),
                desired_force_n=stats(vector(vehicle_rows, "desired_force")[mask]),
                support_trim_n=stats(trims[mask, index]),
                requested_initial_support_trim_n=stats(number(vehicle_rows, "transport_support_requested_trim_n")[mask]),
                position_integral=stats(integral),
                position_integral_span=np.ptp(integral, axis=0).tolist(),
                command_thrust_n=stats(thrust[mask, index]),
                command_raw_thrust=stats(number(vehicle_rows, "command_raw_thrust")[mask]),
                battery_voltage_v=stats(number(vehicle_rows, "battery_voltage_v")[mask]),
                preflight_thrust_raw_scale=stats(number(vehicle_rows, "thrust_raw_scale")[mask]),
                desired_tension_n=stats(tensions[mask, index]),
                body_rate_rps=stats(rates[mask, index]),
                transverse_body_rate_rms_rps=float(np.sqrt(np.mean(np.sum(rates[mask, index, :2]**2, axis=1)/2))),
                desired_tilt_deg=stats(np.rad2deg(desired_tilt)),
                host_transport_tilt_limit_deg=stats(np.rad2deg(cap)),
                fraction_at_host_1n_thrust_cap=float(np.mean(thrust[mask, index] >= .999)),
                fraction_at_desired_tilt_limit=float(np.mean(desired_tilt >= cap-1e-5)),
                max_abs_command_rate_rps=np.max(np.abs(vector(vehicle_rows, "command_rate")[mask]), axis=0).tolist(),
                link_force_rate_limited_fraction=float(np.mean(number(vehicle_rows, "link_force_rate_limited")[mask])),
                link_force_magnitude_limited_fraction=float(np.mean(number(vehicle_rows, "link_force_magnitude_limited")[mask])))
        return result

    windows = {f"{a:g}-{b:g}s": window(active & (rel >= a) & (rel < b)) for a, b in WINDOWS}
    final_mask = active & (time >= data["landing"]-1.)
    final = window(final_mask)
    full_progress = active & (progress >= 1.-1e-12)
    snapshots = []
    for elapsed in (0., 5., 10., 12., 16., 19., 20., rel[last_index]):
        index = int(np.argmin(np.abs(rel-elapsed)))
        snapshots.append(dict(elapsed_s=float(rel[index]), control_time_s=float(time[index]),
            actual_rpy_deg=np.rad2deg(attitude[index]).tolist(),
            target_rpy_deg=np.rad2deg(target[index]).tolist(),
            actual_tilt_deg=float(tilt[index]), reference_progress=float(progress[index]),
            flight_phase=rows[index]["flight_phase"],
            payload_z_m=float(payload_position[index, 2]),
            payload_target_z_m=float(number(rows, "payload_target_z")[index])))
    comparisons = {}
    for a, b in ((0., 2.), (2., 5.), (5., 7.), (7., 9.5)):
        comparisons[f"{a:g}-{b:g}s"] = {}
        for name, flight in (("current", data), ("previous_222418", prior)):
            mask = flight["active"] & (flight["rel"] >= a) & (flight["rel"] < b)
            rv = np.stack([vector(flight["aligned"][ident], "body_rate") for ident in IDS], axis=1)
            tv = np.column_stack([number(flight["aligned"][ident], "command_thrust_newton") for ident in IDS])
            comparisons[f"{a:g}-{b:g}s"][name] = dict(
                transverse_body_rate_rms_rps=np.sqrt(np.mean(np.sum(rv[mask, :, :2]**2, axis=2)/2, axis=0)).tolist(),
                thrust_std_n=np.std(tv[mask], axis=0).tolist())
    metrics = dict(source=str(SOURCE), raw_rows=len(data["raw"]),
        rows_by_vehicle={ident: len(data["fleet"][ident]) for ident in IDS},
        aligned_ticks=count, handoff_s=data["handoff"], landing_s=data["landing"],
        transport_duration_s=data["landing"]-data["handoff"],
        first_completed_reference_elapsed_s=float(rel[np.flatnonzero(full_progress)[0]]),
        full_reference_hold_until_landing_s=float(data["landing"]-time[np.flatnonzero(full_progress)[0]]),
        windows=windows, final_second=final,
        frozen_reference_geometry_d0_m=d0.tolist(),
        model_nominal_link_lengths_m=[float(fleet[ident][handoff_index]["link_length_nominal_m"]) for ident in IDS],
        model_effective_link_lengths_m=[float(fleet[ident][handoff_index]["link_length_effective_m"]) for ident in IDS],
        active_force_modes=sorted({row["payload_transport_force_mode"] for row, ok in zip(rows, active) if ok}),
        active_effective_link_gain=stats(number(rows, "effective_transport_link_gain_scale")[active]),
        active_raw_link_force_max_abs_n=float(max(np.max(np.abs(vector(fleet[ident], "raw_link_force")[active])) for ident in IDS)),
        active_pair_distance_min_m=np.min(distances[active], axis=0).tolist(),
        active_payload_state_age_s=stats(number(rows, "payload_state_age_s")[active]),
        active_payload_safety_reasons=sorted({row["payload_safety_reason"] for row, ok in zip(rows, active) if ok}),
        active_trim_difference_spans_n=np.ptp((trims[:, 1:]-trims[:, :1])[active], axis=0).tolist(),
        snapshots=snapshots, matched_previous_windows=comparisons,
        transitions={name: transitions(rows, name) for name in (
            "flight_phase", "payload_prelift_stage", "flight_validation_landing_stage",
            "payload_landing_touchdown_confirmed", "payload_landing_unloading", "emergency_landing_reason", "global_abort_reason")},
        last_transport=dict(elapsed_s=float(rel[last_index]),
            actual_rpy_deg=np.rad2deg(attitude[last_index]).tolist(),
            target_rpy_deg=np.rad2deg(target[last_index]).tolist(),
            reference_progress=float(progress[last_index])),
        ending_states={ident: {key: data["fleet"][ident][-1][key] for key in (
            "control_time_s", "flight_phase", "flight_validation_landing_stage", "position_z",
            "command_thrust_newton", "command_raw_thrust", "payload_position_z")} for ident in IDS},
        caveats=["Payload RPY is logged mocap-derived controller state, not an independent physical leveling measurement.",
            "Projected moment uses desired tensions: sum rho cross (R.T @ (-T*q)); not actual cable tension, physical moment, or COM identification.",
            "1 N thrust cap and configured tilt bounds are host command bounds; no motor telemetry is logged and these cannot exclude individual actuator limitations.",
            "Current disk YAML is not an immutable capture of every launch parameter; actual logged mode, gains, references and commands are used where available."])
    (HERE / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=True)+"\n")

    fig, axes = plt.subplots(6, 2, figsize=(15, 22), constrained_layout=True)
    shown = (rel >= -3.) & (rel <= 21.)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for axis, label in enumerate(("roll", "pitch", "yaw")):
        axes[0, 0].plot(rel[shown], np.rad2deg(attitude[shown, axis]), color=colors[axis], label=label)
        axes[0, 0].plot(rel[shown], np.rad2deg(target[shown, axis]), "--", color=colors[axis], label=label+" target")
        axes[4, 0].plot(rel[shown], requested_moment[shown, axis]*1e3, label=label)
        axes[4, 1].plot(rel[shown], projected_moment[shown, axis]*1e3, label=label)
    axes[0, 1].plot(rel[shown], tilt[shown], label="actual tilt")
    axes[0, 1].plot(rel[shown], target_tilt[shown], "--", label="target tilt")
    axes[0, 1].plot(rel[shown], progress[shown]*20., ":", label="reference progress x20")
    for index, ident in enumerate(IDS):
        for axis, panel in enumerate((axes[1, 0], axes[1, 1], axes[2, 0])):
            panel.plot(rel[shown], hold_error[shown, index, axis]*1000., label=f"CF{ident}")
        axes[2, 1].plot(rel[shown], np.sqrt(np.sum(rates[shown, index, :2]**2, axis=1)/2), label=f"CF{ident}")
        axes[3, 0].plot(rel[shown], thrust[shown, index], label=f"CF{ident}")
        axes[3, 1].plot(rel[shown], trims[shown, index], label=f"CF{ident}")
        axes[5, 0].plot(rel[shown], cable_angle[shown, index], label=f"CF{ident}")
    axes[5, 1].plot(rel[shown], payload_position[shown, 2], label="actual")
    axes[5, 1].plot(rel[shown], number(rows, "payload_target_z")[shown], "--", label="target")
    titles = ("Logged payload attitude", "Payload tilt / reference progress", "Aircraft hold error X", "Aircraft hold error Y",
              "Aircraft hold error Z", "Aircraft transverse body rate", "Host thrust command", "Support trim (common update)",
              "Requested payload body moment", "Desired-tension projected body moment", "Actual vs allocated cable direction", "Payload height")
    units = ("deg", "deg / scaled progress", "mm", "mm", "mm", "rad/s", "N", "N", "mN m", "mN m", "deg", "m")
    for panel, title, unit in zip(axes.flat, titles, units):
        panel.set(title=title, ylabel=unit, xlabel="Time since handoff [s]", xlim=(-3., 21.))
        for at, style in ((0., ":"), (5., ":"), (12., "--"), (17., ":"), (data["landing"]-data["handoff"], "--")):
            panel.axvline(at, color="gray", linestyle=style, linewidth=.8)
        panel.grid(True, alpha=.2)
        panel.legend(fontsize=8, ncol=3)
    fig.suptitle("Real flight 2026-10-18 23:05:30 — full 20 s reference cascade\nHandoff 0 s; attitude reference complete 12 s; planned landing 20.013 s")
    fig.savefig(HERE / "analysis.png", dpi=150)
    plt.close(fig)

    def fmt(values, digits=3):
        return "[" + ", ".join(f"{value:+.{digits}f}" for value in values) + "]"
    lines = ["# 2026-10-18 23:05:30：20 s 已生效，但水平姿态仍有明显残差", "",
        f"输入 `../{SOURCE.name}`，{len(data['raw'])} 行。离线运行 `python analyze_flight.py` 可复现 metrics.json 与 analysis.png；脚本不调用 ROS 或硬件。", "",
        f"接管 {data['handoff']:.6f} s，下降 {data['landing']:.6f} s，吊运窗口 **{data['landing']-data['handoff']:.6f} s**。退出字段直接记录 `flight_validation_complete: 20.013 s`。姿态参考首次到达终点为接管后 {metrics['first_completed_reference_elapsed_s']:.6f} s，随后还有 {metrics['full_reference_hold_until_landing_s']:.3f} s 空中水平目标保持。此前 10 s 窗口未覆盖 12 s 参考终点的问题已经排除。", "",
        "| 接管后窗口 | 实际 R/P/Y 均值 (deg) | 目标 R/P/Y 均值 (deg) | 实际倾角均值 (deg) |", "|---|---|---|---|"]
    for name, result in windows.items():
        lines.append(f"| {name} | `{fmt(result['actual_rpy_deg']['mean'])}` | `{fmt(result['target_rpy_deg']['mean'])}` | {result['actual_tilt_deg']['mean']:.3f} |")
    lines += ["", f"实际最后一秒（下降前的精确 1 s）RPY 平均 `{fmt(final['actual_rpy_deg']['mean'])}°`。最后吊运帧 `{fmt(metrics['last_transport']['actual_rpy_deg'])}°`，目标为零。俯仰在逐渐下降，但这 8 s 的完整水平目标保持仍未消除十余度的残差，继续单独延长验证时长不能证明已经修复。", "",
        "## 飞机参考跟踪和限幅", "", "12–20 s 按实际位置减位置保持参考计算，顺序 XYZ。", "",
        "| 飞机 | 位置保持误差均值 (mm) | 位置保持反馈力均值 (N) | 支撑 trim 均值 (N) | 主机推力均值/最大值 (N) | 电压均值 (V) | 横向角速度 RMS (rad/s) |", "|---|---|---|---|---|---|---|"]
    steady = windows["12-20s"]
    for ident, result in steady["vehicles"].items():
        lines.append(f"| CF{ident} | `{fmt(np.array(result['hold_position_error_m']['mean'])*1e3, 1)}` | `{fmt(result['hold_force_n']['mean'], 4)}` | {result['support_trim_n']['mean']:.4f} | {result['command_thrust_n']['mean']:.3f}/{result['command_thrust_n']['max']:.3f} | {result['battery_voltage_v']['mean']:.3f} | {result['transverse_body_rate_rms_rps']:.4f} |")
    lines += ["", "三机积分状态在该段每轴跨度均为 0；运输分支冻结独立位置积分，实际位置保持力仅含 PD。三机支撑 trim 的差值在整个吊运段保持固定（同一个负载高度误差积分同时加到三架机），因此该通道可以调整共同高度，却不学习单机差分静差。CF4/CF5 高于参考约 5 cm，CF3 低约 1 cm，并伴随数厘米到约 10 cm 的 XY 静差；这与实际平衡构型没有跟上参考相容，但不能从 CSV 直接指定推力标定、质心或刚体坐标配准中的唯一根因。", "",
        "主机命令没有触及 1.0 N 总推力上限，具体倾角/速率/推力范围和触界比例见 metrics.json。CSV 未记录四电机转速或电机饱和标志，因此这些检查只能排除主机命令限幅，不能证明实物每个执行器均有余量。12–20 s 机体横向角速度仍小，没有出现此前逐渐增强的摆动。", "",
        "## 分配模型与实际构型", "",
        f"12–20 s 请求负载力矩均值 `{fmt(steady['requested_body_moment_mnm']['mean'], 5)}` mN·m；用期望张力沿实测绳向投影后为 `{fmt(steady['projected_desired_tension_body_moment_mnm']['mean'], 5)}` mN·m。本次俯仰投影与请求基本接近，不能重复使用上轮明显俯仰投影失配的解释。请求俯仰仍为负，控制器在要求纠正正俯仰。横滚投影偏差较大。", "",
        f"三机实际绳向相对期望绳向夹角均值为 `{fmt(steady['cable_angle_to_desired_deg']['mean'], 3)}°`。冻结接管几何 d0=`{fmt(d0, 6)}` m；名义模型长度为 0.850 m；12–20 s 每帧重建的实际几何长度均值 `{fmt(steady['geometric_rope_lengths_m']['mean'], 6)}` m。后者按实测负载姿态/位置与飞机位置重建，不是缓存的校准列。", "",
        "力矩公式为 `Σ rho_i × (R.T @ (-T_i*q_i))`。T 是期望张力，不是实测张力；不包含未知的真实飞机保持反馈/支撑 trim 对张力的全部影响。不能把该投影当成实际净力矩，也不能据此倒推出质心位置。", "",
        "## 修改方向的证据边界", "",
        "已确认的问题是：参考已到水平；姿态误差仍大；三机有持续不同的构型跟踪静差；现有本机保持仅 PD，公共高度 trim 无法独立消除差分静差。一个针对该缺口的最小候选是运输位置保持增加有界、缓慢、带抗饱和的逐机积分修正，先离线回放并验证再决定参数。该候选不会自动校正未核实的负载刚体坐标/实体水平面、质心或挂点模型；不能宣称它已被本次飞行验证。不要以盲目增加姿态 P、恢复强绳向反馈、或给水平目标加反向角偏置替代证据验证。", "",
        "所有建议均为离线诊断结果；本目录只新增分析脚本、指标、图和报告，没有修改飞行控制器或 YAML。", "",
        "## 退出", ""]
    for name in ("flight_validation_landing_stage", "payload_landing_touchdown_confirmed", "payload_landing_unloading"):
        lines.append(f"- `{name}`: " + "; ".join(f"{item['time_s']:.3f} s → {item['value'] or '(empty)'}" for item in metrics["transitions"][name]))
    lines += ["", "CF3/CF4 的末状态为 landed、零推力，高度约 0.049/0.046 m；CF5 的末状态仍为 landing，高度约 0.122 m、主机推力 0.372 N/raw34352。因此日志证明负载接地及前两机停推，未证明 CF5 已落地。负载实际 RPY 仍只是控制器使用的动捕姿态，实体负载是否完全水平需独立机械/坐标核对。", ""]
    (HERE / "analysis.md").write_text("\n".join(lines))
    print(json.dumps({key: metrics[key] for key in (
        "transport_duration_s", "first_completed_reference_elapsed_s", "last_transport",
        "active_trim_difference_spans_n", "ending_states")}, indent=2))


if __name__ == "__main__":
    main()
