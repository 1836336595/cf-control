# Crazyswarm CTBR 与三机吊运控制

基于 [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm) 的 Crazyflie 主机端控制项目，支持 NOKOV 动捕、CTBR 指令和三机协同吊运。主机读取飞机位置、姿态及 EKF 运动学状态，计算整机总推力和机体系角速度，经 Crazyswarm server 与 Crazyradio 发送到飞机。

当前三机吊运采用 `reference_cascade` 与 `native_position_pid`。实飞机载控制器和离线机载模型均使用 Mellinger；仿真物理与传感器参数已经从实飞参数中分离。

本页提供配置、运行和代码导航。控制公式集中在 [控制器公式说明](docs/slung_controller_formulas.md)，模块职责见 [控制器模块与参数入口](docs/ctbr_controller_modules.md)。

## 功能

- 单机独立起飞、位置保持、收绳预抬和降落。
- CF3、CF4、CF5 三点吊运，包含负载位姿观测、张力分配、构型跟踪和接管混合。
- 负载定点悬停、圆周、双圆八字和固定质心圆锥姿态轨迹。
- NOKOV、EKF、电压预检、机间距保护、状态失效处理和协同放载。
- 实飞 CSV、飞机与负载位姿绘图、RViz 路径。
- 使用实际主机控制代码的 Udaan/MuJoCo 离线回放。

普通 CTBR 任务另有 `hover`、`circle` 和 `figure_eight_triangle`；它们与负载轨迹分别配置。

## 环境与依赖

| 用途 | 环境 |
|---|---|
| 实飞主机 | Ubuntu、ROS Noetic、系统 Python 3.8、C++ Crazyswarm server |
| 硬件与定位 | Crazyflie、Crazyradio、NOKOV SDK 与相应网络配置 |
| 无刷机 ARM | `cf_arm.py` 所用 Python 环境中可导入支持当前飞机的 `cflib` 和 PyYAML |
| 离线动力学 | 本项目修改版 Udaan、Python 3.10 或更高、MuJoCo、NumPy、SciPy、Matplotlib、PyYAML |
| 自动测试 | 对应 Python 环境中的 pytest；部分检查还需要 ROS 或 MuJoCo |

基础依赖与 USB 配置参考 [Crazyswarm 安装文档](https://crazyswarm.readthedocs.io/en/latest/installation.html)。本项目的 ROS Noetic 实飞环境与 Udaan 仿真环境应分别准备。

**离线回放依赖仓库外的修改版 Udaan。** 当前脚本按以下相邻目录布局查找模型；仅克隆本仓库或安装上游同名包，不代表已经包含本项目的吊运模型。

```text
workspace/
├── crazyswarm/                 # 本仓库
└── simulate_uav/
    └── udaan/                 # 本项目使用的修改版 Udaan
```

## 编译

在已准备好 ROS 和 NOKOV 依赖的环境中，从本仓库根目录执行：

```bash
git submodule update --init --recursive
source /opt/ros/noetic/setup.bash
cd ros_ws
catkin_make
source devel/setup.bash
```

子模块更新不能替代 NOKOV 厂商 SDK 的准备。修改消息、C++ server 或安装配置后需重新编译；修改 Python 或 YAML 后需重启相应节点。

## 参数文件

文件位于 [ros_ws/src/crazyswarm/config](ros_ws/src/crazyswarm/config)。

| 文件 | 主要内容 | 读取范围 |
|---|---|---|
| [ctbr_controller.yaml](ros_ws/src/crazyswarm/config/ctbr_controller.yaml) | 主机频率、反馈过滤、预检、全局轨迹与可视化 | 实飞；仿真沿用控制设置 |
| [ctbr_vehicle.yaml](ros_ws/src/crazyswarm/config/ctbr_vehicle.yaml) | 每机质量、推力映射、独立及运输控制增益 | 实飞；仿真沿用控制设置 |
| [slung_payload.yaml](ros_ws/src/crazyswarm/config/slung_payload.yaml) | 负载质量、挂点、绳长、负载控制及任务参考 | 实飞；仿真沿用控制设置 |
| [slung_simulation.yaml](ros_ws/src/crazyswarm/config/slung_simulation.yaml) | 电机物理、惯量、Mellinger 模型、采样、延迟和观察窗口 | 仅离线仿真 |

实飞 launch 不加载 `slung_simulation.yaml`。仿真读取实飞控制配置作为基线，再独立加载物理与传感器设置；仿真不会回写实飞文件。

飞机条目在 [crazyflies.yaml](ros_ws/src/crazyswarm/launch/crazyflies.yaml) 中设置。每个启用的飞机 ID 都应有对应的 `ctbr_controller_cf<ID>` 参数块；`ctbr_enabled`、`takeoff_vehicle_count`、radio URI 和实际飞机必须匹配。当前吊运任务要求 CF3、CF4、CF5。

### 当前吊运基线

| 项目 | 当前配置 |
|---|---|
| 主机模式 / 飞机运输反馈 | `slung_load` / `native_position_pid` |
| 吊运力构造 | `reference_cascade` |
| 负载总质量 | 0.023 kg |
| 名义绳长 | 三根均为 0.850 m，可在接管前校准一次运行有效值 |
| 负载几何中心目标高度 | 0.60 m |
| 主机控制 / EKF 样本 / 电池样本 | 70 Hz / 50 Hz / 2 Hz |
| 仿真机载模型 | 固定 Mellinger，500 Hz |
| 当前负载轨迹 | `hover`；切换到圆锥时已配置 15°、两圈、每圈 30 s |
| 动态构型补偿 | 动态与抬升期间更新均启用，反馈健康与限幅门控继续生效 |
| 实飞计划降落 | 进入 `TAUT_RAMP` 后 100 s |
| 离线观察窗口 | `slung_simulation.post_handoff_s: 100.0` |

这些数值是当前实验配置，不是其他飞机或负载可直接使用的通用标定。

### 坐标与反馈

- 世界坐标采用 ROS 的 Z 轴向上约定；位置为米，角度为弧度，推力为整机牛顿。
- NOKOV 的 `load` 刚体原点定义在负载上表面中心；控制器结合尺寸和姿态换算为几何中心。
- `attachment_points_m` 使用负载几何中心坐标系。当前飞机端几何使用动捕刚体原点，未自动加入飞机挂点偏移。
- 飞机位置、姿态来自 NOKOV；平移控制速度、加速度来自机载 EKF 日志的主机过滤与差分。
- 运输使用的飞机角速度来自动捕姿态差分与过滤，不能当作机载陀螺仪遥测。
- 负载导数由位姿观测器估计；动捕消息中的原始导数另作诊断。

负载观测器拒绝重复/倒序时间戳、异常位姿和达到物理边界的候选导数。
小于 5 ms 的间隔先累积时间；正常 100 Hz 保留原观测增益响应。
被拒绝的帧不会继续进入负载姿态 PD。短异常复用完整运输指令，缓存预算耗尽后
转入保留支撑残差的受控下降。CSV 将控制输入快照与最新源帧分开记录，
可用 `payload_control_reused` 和 `payload_source_observer_status` 查明保持原因。
预抬期间的短采样等待可在最多 50 ms 内复用最近可信状态，保持上升和已有支撑连续；
新积分更新冻结，接管仍需新接受样本。该修复不延长 60 s 预抬超时。
配置、状态和推导见 [负载观测与诊断](docs/slung_payload_observer.md)。

## 负载轨迹

在 `slung_payload.yaml` 的 `trajectory` 中选择模式。普通飞机的 `ctbr_trajectory.trajectory_mode` 保持 `hover`，由负载参考统一带动三机。

| 模式 | 路径 | 负载朝向参考 |
|---|---|---|
| `hover` | 固定位置 | 水平、零航向 |
| `circle` | 单圆 | 指向圆心 |
| `figure_eight` | 两个精确相切圆 | 沿运行切线 |
| `attitude_cone` | 质心固定，自身 +Z 绕世界 +Z 扫锥 | 采用起止闭合姿态参考 |

当前 `trajectory.mode` 为 `circle`。切换到固定质心圆锥姿态时，可使用已配置的 15°、两圈参考：

```yaml
slung_payload:
  target_payload_height_m: 0.60
  trajectory:
    mode: attitude_cone
    center_xy: [0.0, 0.0]
    cone_half_angle_deg: 15.0
    cone_period_s: 30.0
    cone_transition_s: 5.0
    direction: ccw
    revolutions: 2.0
    ramp_s: 5.0
    settle_s: 3.0
  flight_validation_enabled: true
  flight_validation_transport_s: 100.0
```

这里的 `center_xy` 是固定质心的世界 XY，目标高度为 0.6 m。自身 +Z 在扫描期间与世界 +Z 保持 15° 夹角。`revolutions` 指定完整圈数，当前为两圈，每圈由 `cone_period_s` 指定为 30 秒；每增加一圈，扫描时间增加 30 秒。

从进入 `TAUT_RAMP` 开始计时，该圆锥配置在第 20–25 秒倾斜到 15°，第 25–55 秒扫描第一圈，第 55–85 秒扫描第二圈，第 85–90 秒恢复水平，第 100 秒开始计划降落。每圈采用相同的平滑起停；圈与圈之间保持 15° 倾斜，角速度和角加速度归零后开始下一圈。参考固定不代表实际质心没有误差。`settle_s` 规定参考等待时间，不表示实际负载姿态已经收敛。

增加圈数时需要同步核对实飞 `flight_validation_transport_s`、普通飞机 `ctbr_trajectory.hover_duration_s` 和离线 `slung_simulation.post_handoff_s`。例如三圈参考在接管后第 120 秒完成，应将两个任务窗口至少设为 120 秒，并按实验需要增加水平保持时间，离线观察窗口同步延长。程序保留原窗口校验，不会自动延长这些参数。

圆周模式中的 `center_xy` 表示单圆圆心。省略该项时自动确定圆心；不要填写 `null`，ROS1 参数服务器不支持空值。

详细轨迹、时间和切换方法见 [圆锥姿态说明](docs/slung_payload_attitude_cone.md)、[圆周说明](docs/slung_payload_circle.md) 与 [八字说明](docs/slung_payload_figure_eight.md)。

### 姿态构型补偿

`reference_cascade` 的慢姿态 trim 修正三机位置、速度和加速度参考，负载目标姿态继续由轨迹指定。
在 `slung_payload.yaml` 中，动态模式需同时启用以下两个开关：

```yaml
slung_payload:
  transport_attitude_trim_enabled: true
  transport_attitude_trim_dynamic_enabled: true
  transport_attitude_trim_during_lift_enabled: true
  transport_attitude_trim_feedback_max_rate_rps: 0.50
  transport_attitude_trim_feedback_pose_jump_deg: 1.0
  transport_attitude_trim_feedback_recovery_s: 0.50
```

动态开关当前为 `true`；设为 `false` 可沿用原静态策略：运动期间冻结学习，圆周/八字旋转已有偏置，圆锥保持已有世界系偏置。
设为 `true` 后，补偿使用瞬时完整目标姿态，可在健康的运动参考期间继续学习；已有偏置直接保存在世界系，圆周/八字不再额外按 yaw 旋转它。
每机偏置范数、速度和加速度分别限制为当前配置的 10 mm、1.5 mm/s 和 2 mm/s²；各机偏置之和为零，每个 1 s 五次段在连接处保持位置、速度和加速度连续。冻结时完成当前段，再保持端点。

`transport_attitude_trim_during_lift_enabled: true` 允许动态补偿在张力渐增完成、协同混合达到 100% 后继续更新，当前最早接管后第 5 秒，覆盖随后抬升和调平阶段。设为 `false` 恢复参考全部建立后才更新，当前第 17 秒。该开关不提前放开单机位置 I 或差分力积分；反馈健康和限幅门控仍生效。

动态 trim 额外检查负载角速度范数和相邻源位姿旋转步长，异常后需连续恢复 0.5 s 才重新学习；步长门限还包含源样本间隔允许的转动量。
这些 trim 门控只暂停新更新；负载核心输入另受[观测质量准入](docs/slung_payload_observer.md)保护。
这是有界、低带宽的构型补偿；运动滞后也可能被慢积分学习，限幅和参考连续性不能保证闭环收敛或不振荡。使用前应对照相同轨迹的关闭、静态和动态回放，并检查真实几何及机体差异。
推导、健康门控和诊断坐标见 [控制器公式说明](docs/slung_controller_formulas.md#9-姿态到构型的慢修正与下降衔接)。
本次完整仿真对照、开关和适用范围见 [动态构型补偿](docs/slung_dynamic_attitude_trim.md)。

## 实飞启动

先核对飞机、负载几何、定位网络和每机参数。以下命令使用真实硬件。

从仓库根目录，在第一个终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ros_ws/devel/setup.bash
roslaunch crazyswarm hover_swarm.launch
```

`hover_swarm.launch` 会通过 `cf_arm_wrapper.sh` 对配置中的无刷飞机发送 ARM，随后启动 server；它不是仅查看定位的启动方式。ARM 脚本使用的解释器须能加载正确的 `cflib`；如使用源码版库，可通过 `CFLIB_PATH` 指定其目录。

另开终端，从仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
source ros_ws/devel/setup.bash
roslaunch crazyswarm ctbr_controller.launch target_confirmed:=true enable_realtime_visualization:=false
```

通过预检后会自动进入配置的任务。`target_confirmed:=true` 表示允许真实控制输出；设为 `false` 会拒绝启动控制节点，不是运行中的只读模式。需要实时绘图时，将 `enable_realtime_visualization` 设为 `true`。

任务流程为：全机预检、独立起飞、收绳和预抬、协同接管与轨迹、协同降负载、确认接地并卸载、三机独立下降停推。

`prelift_clearance_m` 是负载底面离地门槛，`target_payload_height_m` 是负载几何中心的最终目标高度。预抬通过独立位置控制建立拉力；协同阶段使用负载与三机参考。计划降落先放下负载，确认接地后才释放协同支撑。

## 离线仿真

先在单独的 Python 3.10 或更高版本环境中准备本项目修改版 Udaan，以及 PyYAML；开发测试还需要 pytest。若采用上述相邻目录布局，可从本仓库根目录安装已有的本地模型：

```bash
python -m pip install -e ../simulate_uav/udaan
python -m pip install pyyaml pytest
```

以下命令均在 **本仓库的 scripts 目录** 执行，使用能导入 Udaan/MuJoCo 的解释器。离线回放不需要启动 `hover_swarm.launch`，不连接 radio，也不发送 ARM。

```bash
cd ros_ws/src/crazyswarm/scripts
python simulate_slung_handoff.py --output ctbr_logs/trajectory_check
python plot_slung_circle_trace.py ctbr_logs/trajectory_check/simulation_trace.csv
```

绘图包括实际与期望 XYZ、RPY 和误差；圆周/八字显示 XY 路径和朝向，圆锥模式显示自身 +Z 方向、锥角和轴方向误差。保存图像而不打开窗口：

```bash
python plot_slung_circle_trace.py ctbr_logs/trajectory_check/simulation_trace.csv \
  --save ctbr_logs/trajectory_check/trajectory.png --no-show
```

实时 MuJoCo 窗口与视频录制：

```bash
python simulate_slung_handoff.py --render \
  --record ctbr_logs/trajectory_check.mp4 \
  --output ctbr_logs/trajectory_check
```

指定另一份仿真配置，或单独检查更大延迟：

```bash
python simulate_slung_handoff.py --simulation-config ../config/slung_simulation.yaml \
  --output ctbr_logs/custom_simulation

python simulate_slung_handoff.py --pose-delay 0.02 --command-delay 0.01 \
  --output ctbr_logs/delay_check
```

默认读取独立的 `slung_simulation.yaml`。电机模型为 Graefe 2026 逐电机模型，机载环固定 Mellinger；`legacy_quadratic` 作为电机物理对照保留。

每次输出包括：

| 文件 | 内容 |
|---|---|
| `input_parameters.yaml` | 控制配置、独立仿真配置和实际模型设置的快照 |
| `simulation_trace.csv` | 模型实际状态、控制指令、负载目标和轨迹进度 |
| `multi_ctbr_*.csv` | 同一主机控制器生成的诊断日志 |
| `summary.json` | 任务完成情况、限幅及实际轨迹跟踪误差 |

修改输入文件后需重新运行才能生成新结果。输出目录中的 `input_parameters.yaml` 是快照，修改它不会改变下一次默认运行。

`post_handoff_s` 只控制离线观察窗口；实飞计划降落由 `flight_validation_transport_s` 控制。当前两个窗口均为 100 s，分别保存在仿真和实飞文件中。

候选覆盖文件使用独立参数根，传给 `--candidate` 后只影响本次回放：

```yaml
slung_payload:
  trajectory:
    mode: hover
  flight_validation_enabled: false
slung_simulation:
  post_handoff_s: 60.0
  pose_delay_s: 0.02
  command_delay_s: 0.01
```

历史候选文件可能覆盖控制增益、任务模式和时间，不能视为当前默认配置的等价验证。旧的嵌套仿真参数与已删除的机载控制器切换参数会提示迁移。

## 实飞日志与绘图

日志默认位于 `ros_ws/src/crazyswarm/scripts/ctbr_logs/`。单机文件为 `cf<ID>_ctbr_*.csv`，多机文件为 `multi_ctbr_*.csv`，通过 `vehicle_id` 区分飞机。

从仓库根目录绘制指定实飞 CSV：

```bash
python3 ros_ws/src/crazyswarm/scripts/ctbr_visualization.py \
  ros_ws/src/crazyswarm/scripts/ctbr_logs/multi_ctbr_YYYYMMDD_HHMMSS.csv \
  --static --no-show --output /tmp/ctbr.png
```

将文件名中的时间戳替换为实际日志；只看一架飞机可增加 `--vehicle-id 5`。有负载数据时会另存 `/tmp/ctbr_payload.png`，显示负载实际与期望 XYZ/RPY。旧 CSV 缺少的字段保持未知，不补造零目标。

| 诊断内容 | 主要字段或话题 |
|---|---|
| 飞机实际/参考 | `position_*`、`target_*`、`desired_*` |
| 负载实际/参考位姿 | `payload_position_*`、`payload_target_*`、负载 RPY 列 |
| 轨迹运行 | `payload_trajectory_mode`、`payload_trajectory_phase`、`payload_trajectory_progress` |
| 圆锥姿态 | `payload_body_z_*`、`payload_target_body_z_*`、`payload_cone_half_angle_deg`、`payload_cone_axis_error_deg` |
| EKF 与反馈状态 | `ekf_status`、`ekf_state_age_s`、`mocap_sample_age_s` |
| 运输积分与补偿 | `transport_full_pid_*`、`transport_attitude_trim_*`、`transport_support_trim_n` |
| 动态 trim 质量 | `transport_attitude_trim_dynamic_enabled`、`transport_attitude_trim_feedback_healthy`、`transport_attitude_trim_feedback_quality_reason`、`transport_attitude_trim_body_rate_norm_rps`、`transport_attitude_trim_pose_step_deg`、`transport_attitude_trim_pose_step_limit_deg` |
| 降落原因与阶段 | `emergency_landing_reason`、`flight_validation_landing_stage` |
| RViz 实际路径 | `/cf<ID>/path`、`/load/path`，坐标系为 `world` |

`emergency_landing_reason` 沿用原字段名，也记录计划结束；含 `flight_validation_complete` 表示验证时间到，不等于 EKF 故障。
动态 trim 的 `transport_attitude_trim_integral_*` 是世界系小角增量累计诊断，不能用当前目标姿态重建已学偏置；实际补偿以每机世界系 `transport_attitude_trim_position/velocity/acceleration_*` 为准。

## 常见问题

| 现象 | 优先检查 |
|---|---|
| launch 报 `cannot marshal None` | YAML 中是否存在 `null` 或没有值的参数 |
| 电压预检样本数为零 | 对应 `/cf<ID>/battery` 是否持续收到数据、样本是否新鲜 |
| EKF 起飞前对齐失败 | EKF 与 NOKOV 位置差、连续对齐时间和导数是否就绪 |
| TAKEUP 无法接管 | 三根有效绳长、负载离地间隙、低竖直速度和当前诊断原因 |
| 绘图报找不到 CSV | 仿真是否完成、实际输出目录及当前工作目录是否匹配 |
| 运行时间短于观察窗口 | `summary.json` 中的结束原因、实飞验证期限、任务时长和保护条件 |

server 启动后可使用只读话题检查，例如：

```bash
rostopic hz /cf3/battery
rostopic hz /cf3/ekf_kinematics
rostopic echo -n 5 /cf3/ctbr_timing
```

当前电池目标为 2 Hz、EKF 目标为 50 Hz；实收频率会受链路影响。`ctbr_timing` 是主机发送链路诊断，不能直接当作飞机执行指令的单向延迟。修改 server 日志配置需重启 server；仅重启控制器不会改变已建立的机载日志周期。

## 代码结构与控制设计

控制计算分为三个职责模块：

| 职责 | 文件 |
|---|---|
| 单机独立飞行、预抬和放载后降落 | [ctbr_independent_controller.py](ros_ws/src/crazyswarm/scripts/ctbr_independent_controller.py) |
| 负载外环、张力分配和绳向处理 | [slung_load_controller.py](ros_ws/src/crazyswarm/scripts/slung_load_controller.py) |
| 吊运时的每机位置反馈和姿态命令 | [ctbr_transport_vehicle_controller.py](ros_ws/src/crazyswarm/scripts/ctbr_transport_vehicle_controller.py) |

[ctbr_geometric_controller.py](ros_ws/src/crazyswarm/scripts/ctbr_geometric_controller.py) 组合独立与运输计算；[ctbr_controller.py](ros_ws/src/crazyswarm/scripts/ctbr_controller.py) 保留启动入口。ROS 收发、日志、参考和机群调度分别放在 `ctbr_vehicle_*`、`ctbr_payload_*`、`ctbr_fleet_node.py` 中。

控制层次为负载位置与姿态反馈、协同张力分配、三机构型参考、单机位置反馈、CTBR 姿态接口、机载 Mellinger 与电机混控。`reference_cascade` 保留负载反馈，使用三机参考跟踪替代原绳向闭环；它与原始完整耦合几何控制律存在工程差别。

`coupled` 和 `differential_integral` 保留作主机对照，当前默认使用 `reference_cascade` 与 `native_position_pid`。姿态 trim 默认沿用静态策略，动态学习需显式启用；差分积分继续在运动期间冻结。位置积分和支撑配平分别管理。

| 文档 | 内容 |
|---|---|
| [控制器公式说明](docs/slung_controller_formulas.md) | 当前控制公式、坐标约定、积分、前馈和代码对应关系 |
| [模块与参数入口](docs/ctbr_controller_modules.md) | 拆分后的文件职责、实飞/仿真配置和覆盖方式 |
| [负载圆周](docs/slung_payload_circle.md) | 圆周配置、向心朝向和参考衔接 |
| [负载八字](docs/slung_payload_figure_eight.md) | 双圆路径、切线朝向与完整任务时序 |
| [圆锥姿态](docs/slung_payload_attitude_cone.md) | 固定质心、可配置锥角、每圈 30 秒的多圈扫描与方向误差检查 |
| [设计推导对照](docs/slung_controller_design_comparison.md) | 原笔记与工程实现的差别、历史试验与边界 |
| [MATLAB 参考实现](MATLAB/Multi-UAV-Transportation-Simulation) | 几何控制与吊运参考代码 |

## 验证范围

当前重构已通过控制公式、状态处理、起飞/接管、轨迹、降落和参数隔离测试。完整八字 MuJoCo 回放覆盖 100 s 接管窗口及降落停推；模块拆分前后记录逐项一致。

圆锥姿态参考曾以单圈配置完成默认延迟和较大延迟的两组 100 s MuJoCo 回放，均降落停推。
扫描期间质心位置误差 RMS 约 4.9–5.0 mm，+Z 轴方向误差 RMS 约 1.33°；
这些是历史 10° 单圈结果，不代表当前 15° 两圈圆锥配置或动态构型补偿的验证。参考与实际锥角仍有差异，详细定义和历史结果见 [圆锥姿态说明](docs/slung_payload_attitude_cone.md)。

模型中仍存在实际与期望姿态误差。仿真没有完整复现机载 EKF、无线随机丢包、所有动捕异常和机体差异；模型回放完成不能代替对应实飞验证。具体配置以各次生成的快照为准。

在 scripts 目录运行当前主机测试：

```bash
python -m pytest -q test_ctbr*.py test_slung*.py \
  test_native_position_pid*.py test_transport_native_pid_equivalence.py
```

绘图和飞机配置还可运行 `test_ctbr_visualization.py`、`test_vehicle_config.py`。ROS 实飞使用系统 Python 3.8，Udaan 使用独立环境；应同时检查实际解释器兼容性。

## 数据与版本管理

飞行 CSV、仿真输出、分析图、视频和 ROS bag 属于运行数据，默认日志目录由 [.gitignore](.gitignore) 排除。README 的使用入口不依赖这些本地文件。

提交改动时应包含实际使用的新模块、参数文件和文档。底层库以子模块管理，子模块内部改动需要在对应仓库保存版本；父仓库仅记录其提交指针。

## 项目来源与许可

本项目基于 [USC-ACTLab/Crazyswarm](https://github.com/USC-ACTLab/crazyswarm)，底层依赖与其许可证见相应子模块。仓库许可为 [MIT License](LICENSE)。
