# Crazyswarm CTBR 与三机吊运控制

本仓库在 Crazyswarm 基础上实现了主机端 CTBR（Collective Thrust and Body Rates）控制。控制器从 NOKOV 读取 Crazyflie 的位置和姿态，通过 Crazyswarm server 接收 EKF 运动学状态，并把 CTBR 指令发送回 Crazyflie。多机任务使用同一台 Crazyradio；实际参加任务的飞机由 `crazyflies.yaml` 中的 `ctbr_enabled` 决定。

当前代码包含两类任务：

- 普通 CTBR：`hover`、`circle` 和 `figure_eight_triangle` 轨迹。
- `slung_load`：CF3、CF4、CF5 组成三点吊运系统，负载刚体名称默认为 `load`，控制流程参考 `MATLAB/Multi-UAV-Transportation-Simulation`。

项目使用 ROS Noetic、Python 3、C++ Crazyswarm server 和 NOKOV 动捕。世界坐标采用 z 轴向上；MATLAB 参考代码中的 z 轴向下时，已在 Python 控制器和负载几何中转换。

## 目录和职责

主要文件位于 `ros_ws/src/crazyswarm`：

- `config/ctbr_controller.yaml`：全局控制、安全、日志、运输模式和公共轨迹参数。
- `config/ctbr_vehicle.yaml`：每架飞机独立的质量、推力标定、PID、姿态增益和角速度限制。
- `config/slung_payload.yaml`：负载质量、尺寸、绳长、挂点、负载控制器和位姿观测器参数。
- `launch/crazyflies.yaml`：飞机 ID、radio URI、是否参加 CTBR 以及编队相位。
- `launch/hover_swarm.launch`：启动 Crazyswarm server、NOKOV、外部位姿广播和 EKF 日志。
- `launch/ctbr_controller.launch`：加载参数并启动 CTBR 控制器和可选实时绘图。
- `scripts/ctbr_controller.py`：多机同步、状态机、几何 CTBR、吊运接管、安全保护和 CSV 日志。
- `scripts/slung_load_controller.py`：负载位置/姿态外环、绳方向控制和张力分配。
- `scripts/ctbr_trajectory.py`：五次平滑参考轨迹、圆周轨迹和三机连续八字轨迹。
- `scripts/ctbr_visualization.py`：读取单机或合并 CSV，绘制飞机和负载轨迹、误差、姿态、速度、加速度及 CTBR 输出。
- `scripts/ctbr_logs/`：控制器生成的飞行 CSV；实飞日志默认不提交到版本库。
- `MATLAB/`：MATLAB 几何 CTBR 和三机吊运参考实现。

## 编译和启动

```bash
cd /home/csy/my_project/crazyswarm/ros_ws
catkin_make
source devel/setup.bash
```

首次编译或修改消息、C++ server 后重新执行 `catkin_make`。只修改 Python 或 YAML 时，重启对应的 launch 即可。

先启动 Crazyswarm server、NOKOV 和 EKF 日志：

```bash
roslaunch crazyswarm hover_swarm.launch
```

另开终端加载工作空间后启动 CTBR：

```bash
source /home/csy/my_project/crazyswarm/ros_ws/devel/setup.bash
roslaunch crazyswarm ctbr_controller.launch target_confirmed:=true
```

`target_confirmed:=true` 是真实飞行的人工确认。使用 `false` 时控制器保持零推力，只适合检查参数、话题和绘图启动是否正常。起飞前控制器还会等待全机 NOKOV 状态、EKF 与 NOKOV 位置对齐，并完成一次电池电压预检。

## 飞机配置

在 `ros_ws/src/crazyswarm/launch/crazyflies.yaml` 中配置每架飞机：

```yaml
crazyflies:
  - channel: 80
    id: 3
    uri: "radio://0/80/2M/E7E7E7E703"
    ctbr_enabled: true
    initialPosition: [1.5, 1.5, 0.0]
    type: default
    orbit_phase_rad: 0.0
    orbit_yaw_mode: face_partner
```

`id` 和 `uri` 必须唯一，所有飞机可以共用一个 radio/channel。`ctbr_enabled: true` 的条目才会参加 CTBR，且数量必须等于 `ctbr_controller.takeoff_vehicle_count`。

`initialPosition` 只是 Crazyswarm 的初始猜测或仿真参数，真实飞行位置和姿态来自 NOKOV；它不会替代动捕状态。`type` 是 Crazyswarm 通用机型字段，不是 CTBR 的 PID 参数来源。

吊运模式要求启用且按顺序包含 CF3、CF4、CF5：

- CF3：负载 +x 棱中点，前方。
- CF4：负载 -x/+y 挂点，左下方。
- CF5：负载 -x/-y 挂点，右下方。

每架飞机的 `payload_attachment_index` 在 `ctbr_vehicle.yaml` 中与上述顺序对应。

## 参数配置

### 全局控制器和轨迹

`ctbr_controller.yaml` 的 `ctbr_controller` 区域控制频率、状态超时、EKF 对齐、推力上限、安全保护、日志目录以及：

```yaml
transport_mode: slung_load       # circle/formation 或 slung_load
payload_rigid_body: "load"
payload_mocap_topic: "/load/mocap_state"
```

`ctbr_trajectory` 区域是所有参考轨迹的唯一参数来源。当前支持：

- `hover`：起飞后按 MATLAB 风格定高悬停，结束后降落。
- `circle`：单机或普通多机圆周轨迹；圆心、半径、圈数和角速度均从 YAML 读取。
- `figure_eight_triangle`：三机保持等边三角形编队，整体连续绕八字，交叉点不中停。

修改 `trajectory_mode`、`circle_center_xy`、`circle_radius_m`、`formation_side_length_m`、`figure_eight_radius_m` 或时间参数后，控制器会从 YAML 生成新参考轨迹；离线绘图读取 CSV 中实际记录的 `target_*`，因此显示的曲线与本次运行所用参数一致，代码中没有另一套固定轨迹数值。

### 各机参数

`ctbr_vehicle.yaml` 以 `ctbr_controller_cf3`、`ctbr_controller_cf4`、`ctbr_controller_cf5` 等参数块区分飞机。质量、最大推力、推力曲线、普通起飞 PID、运输姿态增益和角速度限制都从对应 ID 的参数块读取。新增飞机时，需要同时添加：

1. `crazyflies.yaml` 中的飞机条目；
2. `ctbr_vehicle.yaml` 中同名的 `ctbr_controller_cf<ID>` 参数块；
3. 普通多机模式下相应的 `takeoff_vehicle_count` 和轨迹相位。

### 负载参数和状态估计

`slung_payload.yaml` 的 `rigid_body` 默认为 `load`。NOKOV 刚体原点定义为负载上表面中心，控制器根据 `size_m[2]` 沿负载自身 z 轴向下换算为几何中心；三个 `attachment_points_m` 均为负载上表面挂点，坐标按负载自身坐标系给出。

吊运模式默认 `state_observer_enabled: true`：

- 负载位置和姿态使用 NOKOV；
- 负载速度、加速度和机体角速度由 `PayloadStateObserver` 根据连续位姿/姿态样本估计，并受 `observer_*` 参数限幅；
- `MocapState` 中的原始 twist 和 acceleration 只写入 CSV 诊断，不直接进入负载控制器。

飞机本身在普通 CTBR 和吊运飞机外环中仍使用经过位置一致性检查的 Crazyflie EKF 速度/加速度。NOKOV 只负责飞机的实时位置/姿态；EKF 与 NOKOV 位置持续失配或状态超时会触发保持、受控降落或全局中止。

## 吊运流程

`transport_mode: slung_load` 时，终端会显示五个主阶段：

1. 起飞前保持/等待全机就绪；
2. 三架飞机独立起飞到 `independent_hover_height_m`；
3. `TAKEUP` 按五次平滑轨迹收紧绳索，并依据实测绳长和绳向确认；
4. 内部状态 `tension_ramp`（对应 MATLAB `TAUT_RAMP`）建立张力，然后按五次参考轨迹抬升负载并跟踪任务轨迹；
5. 按 MATLAB 风格先降负载、确认负载接地，再释放吊运控制并完成飞机降落。

等待 EKF、NOKOV 和电池预检属于第 1 阶段。接管前若负载位姿、绳长或绳向不满足条件，控制器会在终端输出原因和各架飞机的绳长计算值；负载状态持续失效会进入受控降落。

## 数据、绘图和 RViz

CSV 保存在：

```text
ros_ws/src/crazyswarm/scripts/ctbr_logs/
```

单机日志命名为 `cf<ID>_ctbr_*.csv`，多机同步日志命名为 `multi_ctbr_*.csv`。多机日志通过 `vehicle_id` 区分飞机，并包含目标、位置误差、姿态误差、EKF 状态、CTBR 输出和负载诊断列。负载列包括：

- `payload_position_*`：换算到几何中心的负载位置；
- `payload_target_*`：负载参考轨迹；
- `payload_raw_velocity_*`、`payload_raw_acceleration_*`：动捕消息中的原始诊断值；
- `payload_velocity_*`、`payload_acceleration_*`、`payload_body_rate_*`：位姿观测器实际提供给吊运控制器的值。

离线绘制指定 CSV：

```bash
python3 ros_ws/src/crazyswarm/scripts/ctbr_visualization.py \
  ros_ws/src/crazyswarm/scripts/ctbr_logs/multi_ctbr_<timestamp>.csv \
  --static --no-show --output /tmp/ctbr.png
```

不指定 CSV 时，脚本默认读取最新日志；实时绘图由 `ctbr_controller.launch` 中的可选节点启动。只看一架飞机时可加 `--vehicle-id 4`。

控制器发布的 RViz `nav_msgs/Path` 话题为：

- `/cf<ID>/path`：对应飞机的实际 NOKOV 轨迹；
- `/load/path`：负载几何中心的实际轨迹。

默认坐标系是 `world`，发布周期由 `ctbr_controller.path_publish_interval_s` 设置。

## 安全检查

真实飞行前确认：

- NOKOV 能同时识别所有启用的 CF 和 `load` 刚体；
- `load` 原点、尺寸、挂点和 `link_lengths_m` 与实物一致；
- radio URI 唯一，且 `ctbr_enabled` 数量和 `takeoff_vehicle_count` 一致；
- 每个启用 ID 都有对应的 `ctbr_controller_cf<ID>` 参数块；
- 电池电压预检、螺旋桨安装、飞行区域和急停方式均已确认；
- 首次调试先使用 `target_confirmed:=false` 检查状态和话题，再使用较低的轨迹速度实飞。

NOKOV、EKF、CTBR 或负载状态出现持续超时时，控制器优先发送零推力或进入受控降落；不要在飞机已经起飞后直接修改参数文件，修改后应重启相关节点。

## 测试

控制器和轨迹的 ROS 无关单元测试可在脚本目录运行：

```bash
cd ros_ws/src/crazyswarm/scripts
python3 -m pytest \
  test_ctbr_controller_v2.py \
  test_ctbr_trajectory_smoothstep.py \
  test_slung_load_controller.py
```

需要完整检查参数、可视化或飞机配置时，再加入对应的 `test_ctbr_visualization.py`、`test_vehicle_config.py`。完整 Crazyswarm 测试还需要仓库的外部依赖和 ROS 环境。

MATLAB 参考实现位于 `MATLAB/Multi-UAV-Transportation-Simulation`；Python 运行时使用 ROS 的 z-up 世界坐标，并对 MATLAB 的 z-down 负载模型进行相应转换。
