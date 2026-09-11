#!/usr/bin/python3
"""实时读取 ctbr_controller.py 写出的 CSV 并绘制控制摘要图。"""

import argparse
import csv
import glob
import os
import sys

import numpy as np
from scipy.signal import butter, filtfilt


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
LOG_DIRECTORY = os.path.join(SCRIPT_DIRECTORY, "ctbr_logs")


def second_order_velocity_filter(velocity, time_s, cutoff_hz=5.0):
    """Filter Nokov velocity with a 2nd-order zero-phase low-pass, then differentiate.

    Each contiguous valid segment is resampled to its median spacing before
    filtering, then interpolated back to the original timestamps. This avoids
    applying a fixed-rate digital filter to jittered mocap samples and never
    carries filter state across invalid samples.
    """
    values = np.asarray(velocity, dtype=float)
    timestamps = np.asarray(time_s, dtype=float).reshape(-1)
    cutoff_hz = float(cutoff_hz)
    if values.ndim != 2 or values.shape[0] != 3 or values.shape[1] != timestamps.size:
        raise ValueError("velocity must have shape (3, N) matching time_s")
    if not np.isfinite(cutoff_hz) or cutoff_hz <= 0.0:
        raise ValueError("cutoff_hz must be positive")

    filtered_velocity = np.full(values.shape, np.nan, dtype=float)
    filtered_acceleration = np.full(values.shape, np.nan, dtype=float)
    valid = np.isfinite(timestamps) & np.all(np.isfinite(values), axis=0)
    segment_start = None
    for index in range(timestamps.size + 1):
        segment_continues = index < timestamps.size and valid[index]
        if segment_continues and segment_start is None:
            segment_start = index
        if segment_continues:
            continue
        if segment_start is None:
            continue
        segment_end = index
        segment_time = timestamps[segment_start:segment_end]
        segment_velocity = values[:, segment_start:segment_end]
        segment_start = None
        if segment_time.size == 1:
            filtered_velocity[:, segment_end - 1] = segment_velocity[:, 0]
            continue
        dt = np.diff(segment_time)
        median_dt = float(np.median(dt))
        if (not np.all(np.isfinite(dt)) or median_dt <= 0.0 or
                np.any(dt <= 0.0) or cutoff_hz >= 0.5 / median_dt):
            continue
        uniform_time = np.arange(segment_time[0], segment_time[-1] + 0.5 * median_dt, median_dt)
        uniform_velocity = np.vstack([
            np.interp(uniform_time, segment_time, axis_values)
            for axis_values in segment_velocity
        ])
        # filtfilt needs more than its padding length. Short segments are copied
        # unchanged but still differentiated so plotting remains continuous.
        if uniform_time.size > 9:
            b, a = butter(2, cutoff_hz / (0.5 / median_dt), btype="low")
            uniform_filtered = filtfilt(b, a, uniform_velocity, axis=1)
        else:
            uniform_filtered = uniform_velocity
        segment_filtered = np.vstack([
            np.interp(segment_time, uniform_time, axis_values)
            for axis_values in uniform_filtered
        ])
        filtered_velocity[:, segment_end - segment_time.size:segment_end] = segment_filtered
        if segment_time.size >= 2:
            filtered_acceleration[:, segment_end - segment_time.size:segment_end] = np.vstack([
                np.gradient(axis_values, segment_time)
                for axis_values in segment_filtered
            ])
    return filtered_velocity, filtered_acceleration


def rpy_to_rotation(rpy_rad):
    """Convert ZYX roll/pitch/yaw angles in radians to a world/body rotation."""
    values = np.asarray(rpy_rad, dtype=float).reshape(3)
    if not np.all(np.isfinite(values)):
        return np.full((3, 3), np.nan)
    roll, pitch, yaw = values
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def attitude_trace_error(actual_rpy_rad, desired_rpy_rad):
    """Return ``tr(I - R_d.T @ R)`` for one or more ZYX RPY samples."""
    actual = np.asarray(actual_rpy_rad, dtype=float)
    desired = np.asarray(desired_rpy_rad, dtype=float)
    if actual.shape != desired.shape or actual.ndim not in (1, 2) or actual.shape[0] != 3:
        raise ValueError("RPY arrays must have matching shape (3,) or (3, N)")
    if actual.ndim == 1:
        rotation = rpy_to_rotation(actual)
        desired_rotation = rpy_to_rotation(desired)
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(desired_rotation)):
            return float("nan")
        return float(np.trace(np.eye(3) - desired_rotation.T @ rotation))

    errors = np.full(actual.shape[1], np.nan)
    for index in range(actual.shape[1]):
        errors[index] = attitude_trace_error(actual[:, index], desired[:, index])
    return errors


def latest_ctbr_log(log_directory=LOG_DIRECTORY):
    candidates = glob.glob(os.path.join(log_directory, "*_ctbr_*.csv"))
    if not candidates:
        raise SystemExit(
            "No CTBR CSV found in %s. Start ctbr_controller.launch first, "
            "or provide a CSV path explicitly." % log_directory
        )
    return max(candidates, key=os.path.getmtime)


def load_log(path):
    with open(path, newline="") as log_file:
        rows = list(csv.DictReader(log_file))
    rows = [row for row in rows if row.get("mode") in ("control", "shadow")]
    if not rows:
        raise ValueError("日志中没有有效控制样本")

    def column(name):
        values = []
        for row in rows:
            try:
                values.append(float(row[name]))
            except (KeyError, TypeError, ValueError):
                values.append(float("nan"))
        return np.asarray(values)

    return rows, column


def require_matplotlib():
    try:
        import matplotlib.pyplot as pyplot
        # 显式导入以注册 3D projection；部分发行版不会由 pyplot 自动加载。
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError as error:
        raise SystemExit(
            "可视化需要 matplotlib；请安装 python3-matplotlib 或 matplotlib。\n"
            "原始错误：%s" % error
        )
    pyplot.rcParams["font.family"] = "DejaVu Sans"
    pyplot.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    pyplot.rcParams["axes.unicode_minus"] = False
    return pyplot


def load_plot_data(log_path, max_abs_position_m):
    """读取一份 CSV，并转换为绘图所需的数组。"""
    rows, column = load_log(log_path)
    time_s = column("control_time_s")
    position = np.vstack((column("position_x"), column("position_y"), column("position_z")))
    target = np.vstack((column("target_x"), column("target_y"), column("target_z")))
    velocity = np.vstack((column("velocity_x"), column("velocity_y"), column("velocity_z")))
    acceleration = np.vstack((
        column("acceleration_x"), column("acceleration_y"), column("acceleration_z")
    ))
    logged_filtered_velocity = np.vstack((
        column("filtered_velocity_x"), column("filtered_velocity_y"),
        column("filtered_velocity_z")
    ))
    logged_filtered_acceleration = np.vstack((
        column("filtered_acceleration_x"), column("filtered_acceleration_y"),
        column("filtered_acceleration_z")
    ))
    has_logged_filter_columns = all(
        name in rows[0]
        for name in (
            "filtered_velocity_x", "filtered_velocity_y", "filtered_velocity_z",
            "filtered_acceleration_x", "filtered_acceleration_y",
            "filtered_acceleration_z",
        )
    )
    filter_derivatives_valid = column("filter_derivatives_valid")
    position_error = np.vstack((
        column("position_error_x"), column("position_error_y"), column("position_error_z")
    ))
    rpy_rad = np.vstack((column("roll_rad"), column("pitch_rad"), column("yaw_rad")))
    desired_rpy_rad = np.vstack((
        column("desired_roll_rad"), column("desired_pitch_rad"), column("desired_yaw_rad")
    ))
    rpy = np.rad2deg(rpy_rad)
    desired_rpy = np.rad2deg(desired_rpy_rad)
    attitude_error_trace = attitude_trace_error(rpy_rad, desired_rpy_rad)
    rate_command = np.rad2deg(np.vstack((
        column("command_rate_x"), column("command_rate_y"), column("command_rate_z")
    )))
    thrust = column("command_thrust_newton")

    # Nokov 丢失刚体时可能写入 9999.999 m；这些样本在图中留出缺口。
    sample_is_plausible = np.all(np.isfinite(position), axis=0) & np.all(
        np.abs(position) <= max_abs_position_m, axis=0
    )
    discarded_samples = int(np.size(sample_is_plausible) - np.count_nonzero(sample_is_plausible))
    invalid_samples = ~sample_is_plausible
    for values in (
            position, velocity, acceleration, logged_filtered_velocity,
            logged_filtered_acceleration, position_error, rpy, desired_rpy, rate_command):
        values[:, invalid_samples] = np.nan
    thrust[invalid_samples] = np.nan
    attitude_error_trace[invalid_samples] = np.nan
    # New controller logs include the causal filter output actually used for
    # CTBR.  Its warmup/reset samples are unknown derivatives, not measured
    # zeros, so draw them as gaps. Legacy CSVs lack these fields and keep the
    # offline plotting fallback for historical flights.
    if has_logged_filter_columns:
        if "filter_derivatives_valid" in rows[0]:
            invalid_filter_samples = (
                ~np.isfinite(filter_derivatives_valid)
                | (filter_derivatives_valid < 0.5)
            )
            logged_filtered_velocity[:, invalid_filter_samples] = np.nan
            logged_filtered_acceleration[:, invalid_filter_samples] = np.nan
        velocity_filtered = logged_filtered_velocity
        acceleration_filtered = logged_filtered_acceleration
        filtered_source = "controller"
    else:
        velocity_filtered, acceleration_filtered = second_order_velocity_filter(velocity, time_s)
        filtered_source = "offline"

    return {
        "log_path": log_path,
        "time_s": time_s,
        "position": position,
        "target": target,
        "velocity": velocity,
        "velocity_filtered": velocity_filtered,
        "acceleration": acceleration,
        "acceleration_filtered": acceleration_filtered,
        "filtered_source": filtered_source,
        "position_error": position_error,
        "rpy": rpy,
        "desired_rpy": desired_rpy,
        "attitude_error_trace": attitude_error_trace,
        "rate_command": rate_command,
        "thrust": thrust,
        "sample_is_plausible": sample_is_plausible,
        "discarded_samples": discarded_samples,
        "rows": rows,
    }


def create_figure(pyplot):
    """创建固定布局；左侧 3D 轨迹图纵向占满并放大显示。"""
    figure = pyplot.figure("Crazyflie CTBR Realtime Visualization", figsize=(16, 14))
    grid = figure.add_gridspec(
        5,
        2,
        width_ratios=(1.65, 1.0),
        left=0.05,
        right=0.95,
        bottom=0.07,
        top=0.91,
        wspace=0.28,
        hspace=0.42,
    )

    trajectory_axes = figure.add_subplot(grid[:, 0], projection="3d")
    trajectory_axes.set(
        xlabel="World X (m)", ylabel="World Y (m)", zlabel="World Z (m)",
        title="3D Position",
    )
    measured_line, = trajectory_axes.plot(
        [], [], [], color="#0067b1", label="Measured path"
    )
    target_line, = trajectory_axes.plot(
        [], [], [], "--", color="#e05a33", label="Reference path"
    )
    start_marker = trajectory_axes.scatter(
        [np.nan], [np.nan], [np.nan], color="#0067b1", label="Start"
    )
    final_marker = trajectory_axes.scatter(
        [np.nan], [np.nan], [np.nan], marker="x", color="#e05a33", label="Final reference"
    )
    trajectory_axes.legend(loc="upper left", fontsize=8)

    error_axes = figure.add_subplot(grid[0, 1])
    error_lines = [
        error_axes.plot([], [], color="#0067b1", label="X error")[0],
        error_axes.plot([], [], color="#e05a33", label="Y error")[0],
        error_axes.plot([], [], color="#2a9d45", label="Z error")[0],
    ]
    error_axes.set(xlabel="Time (s)", ylabel="Position error (m)", title="Position Error")
    error_axes.grid(True)
    error_axes.legend(loc="best", fontsize=9)

    attitude_axes = figure.add_subplot(grid[1, 1])
    attitude_lines = [
        attitude_axes.plot([], [], color="#0067b1", label="Roll")[0],
        attitude_axes.plot([], [], color="#e05a33", label="Pitch")[0],
        attitude_axes.plot([], [], color="#2a9d45", label="Yaw")[0],
        attitude_axes.plot([], [], "--", color="#0067b1", label="Roll reference")[0],
        attitude_axes.plot([], [], "--", color="#e05a33", label="Pitch reference")[0],
        attitude_axes.plot([], [], "--", color="#2a9d45", label="Yaw reference")[0],
    ]
    attitude_axes.set(xlabel="Time (s)", ylabel="Attitude (deg)", title="Attitude Tracking")
    attitude_axes.grid(True)
    attitude_axes.legend(loc="best", fontsize=8, ncol=2)
    attitude_error_axes = attitude_axes.twinx()
    attitude_error_line, = attitude_error_axes.plot(
        [], [], color="#8a3ffc", linewidth=1.2, label="tr(I - R_d.T R)"
    )
    attitude_error_axes.set_ylabel("tr(I - R_d.T R)")
    attitude_error_axes.grid(False)
    attitude_error_axes.legend(loc="lower right", fontsize=8)

    velocity_axes = figure.add_subplot(grid[2, 1])
    velocity_lines = [
        velocity_axes.plot([], [], "--", color="#0067b1", alpha=0.65,
                           label="Velocity X raw")[0],
        velocity_axes.plot([], [], "--", color="#e05a33", alpha=0.65,
                           label="Velocity Y raw")[0],
        velocity_axes.plot([], [], "--", color="#2a9d45", alpha=0.65,
                           label="Velocity Z raw")[0],
    ]
    velocity_filtered_lines = [
        velocity_axes.plot([], [], color="#0067b1", linewidth=1.4,
                           label="Velocity X filtered")[0],
        velocity_axes.plot([], [], color="#e05a33", linewidth=1.4,
                           label="Velocity Y filtered")[0],
        velocity_axes.plot([], [], color="#2a9d45", linewidth=1.4,
                           label="Velocity Z filtered")[0],
    ]
    velocity_axes.set(xlabel="Time (s)", ylabel="Velocity (m/s)", title="Nokov Velocity")
    velocity_axes.grid(True)
    velocity_axes.legend(loc="upper left", fontsize=8)

    acceleration_axes = figure.add_subplot(grid[3, 1])
    acceleration_lines = [
        acceleration_axes.plot([], [], "--", color="#0067b1", alpha=0.65,
                               label="Acceleration X raw")[0],
        acceleration_axes.plot([], [], "--", color="#e05a33", alpha=0.65,
                               label="Acceleration Y raw")[0],
        acceleration_axes.plot([], [], "--", color="#2a9d45", alpha=0.65,
                               label="Acceleration Z raw")[0],
    ]
    acceleration_filtered_lines = [
        acceleration_axes.plot([], [], color="#0067b1", linewidth=1.4,
                               label="Acceleration X filtered")[0],
        acceleration_axes.plot([], [], color="#e05a33", linewidth=1.4,
                               label="Acceleration Y filtered")[0],
        acceleration_axes.plot([], [], color="#2a9d45", linewidth=1.4,
                               label="Acceleration Z filtered")[0],
    ]
    acceleration_axes.set(
        xlabel="Time (s)", ylabel="Acceleration (m/s^2)", title="Nokov Acceleration"
    )
    acceleration_axes.grid(True)
    acceleration_axes.legend(loc="upper left", fontsize=8)

    command_axes = figure.add_subplot(grid[4, 1])
    command_lines = [
        command_axes.plot([], [], color="#0067b1", label="Roll rate command")[0],
        command_axes.plot([], [], color="#e05a33", label="Pitch rate command")[0],
        command_axes.plot([], [], color="#2a9d45", label="Yaw rate command")[0],
    ]
    command_axes.set(
        xlabel="Time (s)", ylabel="Body-rate command (deg/s)", title="CTBR Command"
    )
    command_axes.grid(True)
    command_axes.legend(loc="upper left", fontsize=8)
    thrust_axes = command_axes.twinx()
    thrust_line, = thrust_axes.plot(
        [], [], color="#202020", linewidth=1.2, label="Collective thrust"
    )
    thrust_axes.set_ylabel("Collective thrust (N)")
    thrust_axes.legend(loc="upper right", fontsize=8)

    return {
        "figure": figure,
        "trajectory_axes": trajectory_axes,
        "measured_line": measured_line,
        "target_line": target_line,
        "start_marker": start_marker,
        "final_marker": final_marker,
        "error_axes": error_axes,
        "error_lines": error_lines,
        "attitude_axes": attitude_axes,
        "attitude_lines": attitude_lines,
        "attitude_error_axes": attitude_error_axes,
        "attitude_error_line": attitude_error_line,
        "velocity_axes": velocity_axes,
        "velocity_lines": velocity_lines,
        "velocity_filtered_lines": velocity_filtered_lines,
        "acceleration_axes": acceleration_axes,
        "acceleration_lines": acceleration_lines,
        "acceleration_filtered_lines": acceleration_filtered_lines,
        "command_axes": command_axes,
        "command_lines": command_lines,
        "thrust_axes": thrust_axes,
        "thrust_line": thrust_line,
    }


def _set_3d_marker(marker, point):
    if point is None:
        marker._offsets3d = ([], [], [])
    else:
        marker._offsets3d = ([point[0]], [point[1]], [point[2]])


def _update_3d_limits(axes, position, target):
    point_sets = []
    for values in (position, target):
        valid = np.all(np.isfinite(values), axis=0)
        if np.any(valid):
            point_sets.append(values[:, valid])
    if not point_sets:
        return
    points = np.hstack(point_sets)
    minimum = np.min(points, axis=1)
    maximum = np.max(points, axis=1)
    span = np.maximum(maximum - minimum, 0.1)
    padding = 0.10 * span
    axes.set_xlim(minimum[0] - padding[0], maximum[0] + padding[0])
    axes.set_ylim(minimum[1] - padding[1], maximum[1] + padding[1])
    axes.set_zlim(minimum[2] - padding[2], maximum[2] + padding[2])


def update_figure(handles, data, title_prefix):
    time_s = data["time_s"]
    position = data["position"]
    target = data["target"]

    handles["measured_line"].set_data(position[0], position[1])
    handles["measured_line"].set_3d_properties(position[2])
    handles["target_line"].set_data(target[0], target[1])
    handles["target_line"].set_3d_properties(target[2])
    valid_position = np.flatnonzero(data["sample_is_plausible"])
    start_point = position[:, valid_position[0]] if valid_position.size else None
    valid_target = np.flatnonzero(np.all(np.isfinite(target), axis=0))
    final_point = target[:, valid_target[-1]] if valid_target.size else None
    _set_3d_marker(handles["start_marker"], start_point)
    _set_3d_marker(handles["final_marker"], final_point)
    _update_3d_limits(handles["trajectory_axes"], position, target)

    for line, values in zip(handles["error_lines"], data["position_error"]):
        line.set_data(time_s, values)
    for line, values in zip(
            handles["attitude_lines"],
            np.vstack((data["rpy"], data["desired_rpy"]))):
        line.set_data(time_s, values)
    handles["attitude_error_line"].set_data(time_s, data["attitude_error_trace"])
    for line, values in zip(handles["velocity_lines"], data["velocity"]):
        line.set_data(time_s, values)
    for line, values in zip(handles["velocity_filtered_lines"], data["velocity_filtered"]):
        line.set_data(time_s, values)
    for line, values in zip(handles["acceleration_lines"], data["acceleration"]):
        line.set_data(time_s, values)
    for line, values in zip(
            handles["acceleration_filtered_lines"], data["acceleration_filtered"]):
        line.set_data(time_s, values)
    for line, values in zip(handles["command_lines"], data["rate_command"]):
        line.set_data(time_s, values)
    handles["thrust_line"].set_data(time_s, data["thrust"])

    for axes in (
            handles["error_axes"], handles["attitude_axes"], handles["command_axes"],
            handles["thrust_axes"], handles["attitude_error_axes"],
            handles["velocity_axes"], handles["acceleration_axes"]):
        axes.relim()
        axes.autoscale_view()

    title = "%s: %s" % (title_prefix, os.path.basename(data["log_path"]))
    if data["discarded_samples"]:
        title += " (discarded %d out-of-range samples)" % data["discarded_samples"]
    handles["figure"].suptitle(title, fontsize=14)

    artists = [
        handles["measured_line"], handles["target_line"], handles["start_marker"],
        handles["final_marker"], handles["thrust_line"],
    ]
    artists.extend(handles["error_lines"])
    artists.extend(handles["attitude_lines"])
    artists.append(handles["attitude_error_line"])
    artists.extend(handles["velocity_lines"])
    artists.extend(handles["velocity_filtered_lines"])
    artists.extend(handles["acceleration_lines"])
    artists.extend(handles["acceleration_filtered_lines"])
    artists.extend(handles["command_lines"])
    return artists


def plot(log_path, output_path, show, max_abs_position_m):
    """一次性绘制完整 CSV。"""
    pyplot = require_matplotlib()
    data = load_plot_data(log_path, max_abs_position_m)
    handles = create_figure(pyplot)
    update_figure(handles, data, "CTBR Flight Log")
    if output_path:
        output_path = os.path.abspath(output_path)
        handles["figure"].savefig(output_path, dpi=150)
        print("已保存图像：%s" % output_path)
    if show:
        pyplot.show()
    else:
        pyplot.close(handles["figure"])


def realtime_plot(log_path, output_path, interval_s, max_abs_position_m, log_directory):
    """周期性重读正在增长的 CSV，实时更新曲线。"""
    pyplot = require_matplotlib()
    requested_log_path = log_path
    data = None
    # launch 与控制器同时启动时，CSV 可能尚未创建；等待首个有效样本。
    while data is None:
        try:
            active_log_path = requested_log_path or latest_ctbr_log(log_directory)
            data = load_plot_data(active_log_path, max_abs_position_m)
        except (OSError, ValueError, SystemExit) as error:
            if requested_log_path:
                raise SystemExit("无法读取 CTBR CSV：%s" % error)
            pyplot.pause(max(0.05, float(interval_s)))

    handles = create_figure(pyplot)
    update_figure(handles, data, "CTBR Realtime")

    from matplotlib.animation import FuncAnimation

    def refresh(_frame):
        try:
            # 未指定路径时跟踪最新文件，确保本次 launch 创建的日志会被接管。
            active_log_path = requested_log_path or latest_ctbr_log(log_directory)
            latest = load_plot_data(active_log_path, max_abs_position_m)
        except (OSError, ValueError, SystemExit):
            # 控制器刚创建文件或正在写入表头时，保留上一帧等待下一次刷新。
            return []
        return update_figure(handles, latest, "CTBR Realtime")

    animation = FuncAnimation(
        handles["figure"],
        refresh,
        interval=max(50.0, float(interval_s) * 1000.0),
        blit=False,
        cache_frame_data=False,
    )
    # 保留引用，避免 matplotlib 在窗口显示前回收动画对象。
    handles["figure"]._ctbr_animation = animation
    print("实时绘图：跟踪最新 CTBR CSV（每 %.2f s 刷新）" % interval_s)
    pyplot.show()
    if output_path:
        output_path = os.path.abspath(output_path)
        handles["figure"].savefig(output_path, dpi=150)
        print("已保存图像：%s" % output_path)


def main():
    parser = argparse.ArgumentParser(
        description="实时绘制最新 CTBR CSV；使用 --static 进行一次性绘图"
    )
    parser.add_argument("log", nargs="?", help="CSV 路径；默认读取最新 CTBR CSV")
    parser.add_argument("--output", help="保存 PNG、PDF 或其他 matplotlib 支持的格式")
    parser.add_argument("--static", action="store_true", help="只绘制一次，不实时刷新")
    parser.add_argument("--no-show", action="store_true", help="保存静态图像但不打开窗口")
    parser.add_argument(
        "--ros-params", action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--interval", type=float, default=0.2,
        help="实时刷新周期，s（默认 0.2）",
    )
    parser.add_argument(
        "--max-abs-position-m", type=float, default=10.0,
        help="过滤超出该位置范围的动捕样本（默认 10 m）",
    )
    # roslaunch 会追加 __name:= 和 __log:= 等重映射参数；它们不属于绘图脚本自身。
    ros_remap_args = [argument for argument in sys.argv[1:] if not argument.startswith("__")]
    args, _ = parser.parse_known_args(ros_remap_args)
    log_directory = LOG_DIRECTORY
    if args.ros_params:
        try:
            import rospy
            rospy.init_node("ctbr_visualization", disable_signals=True)
            log_directory = os.path.expanduser(
                str(rospy.get_param("~log_directory", log_directory))
            )
            args.interval = float(rospy.get_param("~interval_s", args.interval))
            args.max_abs_position_m = float(
                rospy.get_param("~max_abs_position_m", args.max_abs_position_m)
            )
        except Exception:
            # 只有 roslaunch 启动时才会请求 ROS 参数；离线绘图不依赖 ROS master。
            pass
    if args.max_abs_position_m <= 0.0:
        parser.error("--max-abs-position-m must be positive")
    if args.interval <= 0.0:
        parser.error("--interval must be positive")

    if args.static or args.no_show:
        log_path = args.log or latest_ctbr_log(log_directory)
        print("Using CTBR log: %s" % log_path)
        plot(log_path, args.output, not args.no_show, args.max_abs_position_m)
    else:
        realtime_plot(
            args.log, args.output, args.interval, args.max_abs_position_m, log_directory
        )


if __name__ == "__main__":
    main()
