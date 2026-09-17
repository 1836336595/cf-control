# Crazyswarm 单机 CTBR 控制

本仓库基于 Crazyswarm，当前配置用于通过一条 Crazyradio 控制一架 Crazyflie 执行主机端 CTBR（Collective Thrust and Body Rates）圆周轨迹。NOKOV 动捕提供实时位置和姿态，Crazyflie 固件 EKF 回传速度和加速度，主机端控制器计算并发送总推力和机体角速度。

## 主要文件

文件位于 `ros_ws/src/crazyswarm`：

- `config/ctbr_controller.yaml`：单机 CTBR、圆周轨迹、起飞/降落和绘图参数。
- `launch/crazyflies.yaml`：当前飞机的 ID、radio URI、`ctbr_enabled` 和 Crazyswarm 机型配置。
- `launch/hover_swarm.launch`：启动 Crazyswarm server、NOKOV、固件 EKF 和电池日志。
- `launch/ctbr_controller.launch`：启动 CTBR 控制器、RViz Path 和实时绘图。
- `scripts/ctbr_controller.py`：状态机、CTBR 控制律、CSV 日志和安全保护。
- `scripts/ctbr_trajectory.py`：单机圆周参考轨迹。
- `scripts/ctbr_visualization.py`：读取 CSV 并绘制控制结果。
- `scripts/test_ctbr_controller_v2.py`：控制律单元测试（参数解析、运动学融合、滤波器、状态机、CSV 列）。
- `scripts/test_ctbr_trajectory_smoothstep.py`：参考轨迹与阶段切换的单元测试。
- `scripts/test_vehicle_config.py`：飞机参数块与选择逻辑测试。
- `scripts/test_ctbr_visualization.py`：日志读取与绘图数据的单元测试。
- `scripts/ctbr_logs/`：控制器生成的 CSV 日志目录（`.gitignore` 已排除实飞日志，只保留一份 `sample_flight.csv` 作为列格式样例）。
- `MATLAB/`：几何 CTBR 控制律与仿真的 MATLAB 参考实现（`v2/` 为较新版本），用于与 Python 实现逐项对照。

> 实飞日志体积很大（单次飞行可达 30 MB 以上），因此不进入版本库。`ctbr_logs/sample_flight.csv`
> 是降采样后的完整飞行样例（CF4 全流程，含全部 10 个阶段和 91 列），可用于核对 CSV 列定义或离线跑绘图脚本。

## 编译

```bash
cd ros_ws
catkin_make
source devel/setup.bash
```

修改消息或 C++ 节点后需要重新执行 `catkin_make`。只修改 Python、YAML 或 launch 文件时，重启对应节点即可。

## 当前飞机配置

当前 `ros_ws/src/crazyswarm/launch/crazyflies.yaml` 只启用 CF4：

```yaml
crazyflies:
- channel: 80
  id: 4
  uri: "radio://0/80/2M/E7E7E7E704"
  ctbr_enabled: true
  initialPosition: [1.5, 1.5, 0.0]
  type: default
```

`ctbr_enabled: true` 的唯一条目就是 CTBR 控制对象。真实飞行中的位置和姿态来自 NOKOV；`initialPosition` 只供 Crazyswarm 物体跟踪的初始猜测或仿真使用，不会替代动捕状态。`type` 仍由 Crazyswarm server 用于机型和默认配置选择。

如需更换飞机，只需修改该文件中的 `id` 和 `uri`，并确保 NOKOV 中的刚体名称与控制器配置一致。单机模式下不要同时保留多个 `ctbr_enabled: true` 条目。

## 参数配置

当前参数集中在 `ros_ws/src/crazyswarm/config/ctbr_controller.yaml` 的 `ctbr_controller` 下，主要包括：

- `target_confirmed`：真实飞行确认开关，由 launch 命令传入。
- `control_rate_hz`：CTBR 控制循环频率，当前为 90 Hz。
- `ekf_kinematics_weight`：EKF 速度/加速度参与比例，当前为 1.0。
- `mass_kg`、`max_command_thrust_newton`：质量和推力标定参数。
- `position_gain`、`velocity_gain`、`integral_gain`：位置 PID 参数。
- `attitude_gain`、`attitude_integral_gain`：姿态环参数。
- `mocap_velocity_filter_cutoff_hz`、`ekf_velocity_filter_cutoff_hz`：速度滤波参数。

轨迹参数也在同一配置块中：

- 圆心相对起飞点偏移：`circle_center_offset_x/y`。
- 圆周半径：`circle_radius_m`。
- 圈数和角速度：`circle_revolutions`、`circle_angular_speed_radps`。
- 起飞高度：`takeoff_height_m`，当前为相对 NOKOV 起点上升 1 m。
- 起飞、入口、悬停和降落时间：对应的 `*_duration_s`、`final_hover_s` 参数。

当前轨迹流程为：起飞前保持、垂直上升、高度校正、平滑进入圆周、完成一圈、终点悬停、垂直降落。

## 运行

先启动 Crazyswarm server、NOKOV 和固件日志：

```bash
roslaunch crazyswarm hover_swarm.launch
```

再启动 CTBR 控制器。真实飞行必须显式确认：

```bash
roslaunch crazyswarm ctbr_controller.launch target_confirmed:=true
```

只检查参数、NOKOV 状态或绘图而不允许真实控制输出：

```bash
roslaunch crazyswarm ctbr_controller.launch \
  target_confirmed:=false enable_realtime_visualization:=false
```

控制器启动后会等待有效 NOKOV 状态、EKF 与 NOKOV 位置对齐以及一次电池电压预检。状态失效、通信超时或 EKF 持续异常时，会停止正常轨迹并进入受控降落或中止保护。

## 状态来源和控制方式

- NOKOV：位置 `p` 和姿态旋转矩阵 `R_WB`。
- EKF：速度和加速度；主机端对 EKF 速度进行因果二阶低通，并由滤波结果得到加速度。
- CTBR：根据位置、速度、积分误差、参考加速度和 jerk 计算期望合力，再生成期望姿态、角速度和总推力。

控制器不使用 `initialPosition` 作为实时状态，也不使用 NOKOV 原始速度/加速度直接闭环。姿态 `R_WB` 保持使用 NOKOV 数据。

## CSV 日志和绘图

日志目录：

```text
ros_ws/src/crazyswarm/scripts/ctbr_logs/
```

单机日志文件通常命名为 `cf4_ctbr_YYYYMMDD_HHMMSS.csv`，包含状态、目标、位置/速度误差、姿态误差、期望合力、推力、角速度、EKF 状态和滤波状态等字段。

离线绘制指定 CSV：

```bash
python3 ros_ws/src/crazyswarm/scripts/ctbr_visualization.py \
  ros_ws/src/crazyswarm/scripts/ctbr_logs/cf4_ctbr_<timestamp>.csv
```

实时绘图由 `ctbr_controller.launch` 默认启动，也可通过 `enable_realtime_visualization:=false` 关闭。绘图包括位置轨迹、位置误差、姿态跟踪、姿态误差、速度、加速度和 CTBR 输出。控制器同时发布 `nav_msgs/Path`，可在 RViz 中查看实际路径。

## 飞行前检查

- NOKOV 已识别 CF4 刚体，且位置、姿态坐标系正确。
- `crazyflies.yaml` 中的 radio URI 与实际飞机一致。
- 电池电压高于 `preflight_min_voltage_v`。
- 螺旋桨、电机和机架安装正常，飞行区域无障碍物。
- 首次测试先使用 `target_confirmed:=false` 检查状态和参数。
- 真正起飞时保持单个 `ctbr_enabled: true`，并确认急停方式可用。

本项目保留上游 Crazyswarm 的通用 API 和仿真能力。通用文档见 [Crazyswarm documentation](https://crazyswarm.readthedocs.io/en/latest/)。

## 单元测试

测试不依赖 ROS 运行时（导入时替换 ROS 消息类型），可直接运行：

```bash
cd ros_ws/src/crazyswarm/scripts
python3 -m pytest test_ctbr_controller_v2.py \
                  test_ctbr_trajectory_smoothstep.py \
                  test_vehicle_config.py \
                  test_ctbr_visualization.py
```

覆盖范围包括：飞机参数块的选择与校验、NOKOV 姿态与 EKF 运动学的融合与失效回退、
二阶速度滤波器的收敛与复位、解析角速度与数值微分的一致性、圆周轨迹的连续性与阶段切换、
运动学与姿态的日志列定义，以及绘图脚本的日志读取。
