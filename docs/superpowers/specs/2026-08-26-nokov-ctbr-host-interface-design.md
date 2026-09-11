# Nokov 状态与 CTBR 主机接口：设计、实施与当前状态

## 范围

为外部 Crazyflie 控制器增加一个使用 Nokov 刚体测量数据的主机端 ROS 接口，并在主机
实现可替换的几何 CTBR 外环、飞行日志和离线可视化。控制器参数和轨迹状态机保留在
Python 层，后续可以直接替换控制律，不必再次修改机载固件。

已烧录到飞机中的 Crazyflie 固件不会被修改或重新烧录。主机使用现有的 legacy RPYT
CRTP 指令通道，以及固件已有的角速度模式参数。当前实机为 40 g、单节电池的无刷
Crazyflie 2.1；配置中的 120 gf 是四个电机合计的满电静态最大推力。

## 接口

每架已配置的 Crazyflie 发布：

```
/cf<ID>/mocap_state  crazyswarm/MocapState
```

`MocapState.msg` 内容如下：

```
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Vector3 acceleration
bool valid
bool derivatives_valid
```

`header.frame_id` 固定为 `world`。位置、姿态、线速度和线加速度均在 `world`
坐标系中表达；`twist.angular` 是 Crazyflie 机体系中的角速度。线性量的单位为米、
秒及其 SI 导数，角速度单位为 `rad/s`。

只有 Nokov 成功跟踪到刚体时，`valid` 才为真。只有累计足够的连续帧以计算速度、
角速度和加速度后，`derivatives_valid` 才为真。长时间帧间隔或无效时间间隔会重置
微分历史。

飞行前必须使 Nokov 刚体定义与 Crazyflie 机体系对齐。第一版假定两者已经对齐；在
允许非零 CTBR 指令飞行前，必须通过受限台架测试验证 roll、pitch、yaw 三轴正负号。

每架已配置的 Crazyflie 还订阅：

```
/cf<ID>/cmd_ctbr  crazyswarm/CTBR
```

`CTBR.msg` 内容如下：

```
std_msgs/Header header
geometry_msgs/Vector3 body_rates
float32 collective_thrust
float32 thrust_raw_scale
```

`body_rates` 是 `[p, q, r]`，单位为 `rad/s`；`collective_thrust` 是整机总推力，
单位为 N。对于当前飞机，`ctbr_max_thrust_newton` 默认值为 `1.176798` N，即总静态
最大推力为 120 gf。主机负责将该指令转换为固件 legacy 角速度命令所需的 `deg/s`，
并映射到原始推力范围 `0..60000`。`thrust_raw_scale` 只影响最后一步 raw PWM 映射，
不是物理推力单位；值为 `0` 时服务端按 `1.0` 处理以兼容旧发布者。

## 数据流

```
Nokov -> libmotioncapture -> CrazyflieGroup::runFast()
      -> /cf<ID>/mocap_state -> 后续外部控制器
      -> /cf<ID>/cmd_ctbr -> CrazyflieROS -> legacy RPYT CRTP packet
      -> 已有机载角速度控制器和电机混控
```

现有的 `crazyswarm_server` Nokov 连接仍然是飞行时唯一使用的连接。`mocap_helper`
仍只作为终端调试工具，`hover_swarm.launch` 不会启动它。

测量状态发布器属于每个 `CrazyflieROS` 对象。在获得对应刚体后，由
`CrazyflieGroup::runFast()` 立即调用。这使刚体关联关系和每架飞机的微分历史均
封装在同一个对象中。

## 微分估计

第一版从相邻的有效 mocap 帧中计算导数量：

- 线速度为位置差除以帧间隔；
- 机体系角速度为四元数增量转换为轴角向量后除以帧间隔；
- 线加速度为滤波后线速度之差除以帧间隔；
- 对导数量使用可配置的一阶低通滤波器。

初始版本以主机收到帧的时间作为时间戳来源，避免依赖 Nokov `iTimeStamp` 未说明的
单位。时间间隔为非正、无效或过大时，重置历史数据，而不产生异常的微分尖峰。

## CTBR 桥接与安全性

`CrazyflieROS` 增加独立的 `cmd_ctbr` 订阅者；`/cmd_vel` 保持现有 RPYT 语义，不
复用。桥接层拒绝非有限输入，使用 ROS 参数限制角速度与总推力，完成单位转换后，
通过已有的 `Crazyflie::sendSetpoint()` 转发。

launch 配置在运行时设置固件已有参数：

```
flightmode.stabModeRoll: 0
flightmode.stabModePitch: 0
flightmode.stabModeYaw: 0
```

已烧录固件中 `0` 表示 RATE。Crazyswarm 在服务器启动时，经由无线电参数接口写入
这些值，不需要刷写固件。legacy commander 的推力锁要求先收到零推力包，因此桥接层
在接受非零 CTBR 推力前必须先发送零推力包。桥接层将物理推力范围映射到 `0..60000`，
当前使用平方推力曲线反解：

```
F / Fmax = (raw_thrust / 60000)^2
raw_nominal = 60000 * sqrt(F / Fmax)
raw_thrust = clip(raw_scale * raw_nominal, 0, 60000)
```

其中 `Fmax = 1.176798 N`。因此 `0.392266 N -> raw 34641`、`0.50 N -> raw 39110`、
`0.80 N -> raw 49470`；`raw_thrust` 是无单位 PWM 数值，不是牛顿。该曲线仍以满电
标定为前提。`raw_scale` 由控制器的预起飞阶段冻结，服务端仅接受 `0.90..1.12` 的正值；
范围外命令会被拒绝并发送零推力。

流式 CTBR 指令的优先级高于 `takeoff()`、`goTo()` 等高层指令。后续控制器必须持续
发布指令；本项目的起飞、最终点悬停和降落全部由同一个 CTBR 节点完成，不在 CTBR
飞行中混用高层 `takeoff()`、`goTo()` 或 `land()`。

## 控制器、日志与可视化

主机端控制器位于 `ros_ws/src/crazyswarm/scripts/ctbr_controller.py`。它参考
`MATLAB/crazyflie_ctbr_controller.m` 的几何位置外环和姿态外环，但采用 ROS world
坐标系的 z 轴向上约定；节点订阅 `/cf<ID>/mocap_state`，并按固定频率向
`/cf<ID>/cmd_ctbr` 发布角速度和物理单位的总推力。

控制器默认是影子模式：只计算和记录，不创建 CTBR publisher。只有同时设置
`enable_control:=true`、`target_confirmed:=true` 才允许发送指令。状态失效或超时会立即
发送零 CTBR，退出时同样发送零 CTBR。

每个控制周期会记录 CSV，包括状态、目标、位置/速度/姿态误差、期望合力和 CTBR
输出。`ros_ws/src/crazyswarm/scripts/ctbr_visualization.py` 可读取该 CSV 并生成与
MATLAB 对应的位置轨迹、位置误差、姿态跟踪和 CTBR 输出四图；当存在有效电压样本时，
还会增加 Battery Voltage 图，并输出最新、最低和最高电压。CSV 还记录固件主电池遥测：
电压、电压毫伏值、电源状态、电量和样本年龄；解析 Omega_c 使用的当前加速度、目标
jerk、解析角速度和计算方法也写入 CSV。电池遥测来自 `/cf2/battery` 的
`GenericLogData`，其字段顺序由 `hover_swarm.launch` 固定为
`[pm.vbat, pm.vbatMV, pm.state, pm.batteryLevel]`。固件 EKF 的
`/cf2/ekf_kinematics` 日志固定为 `[stateEstimate.x/y/z, stateEstimate.vx/vy/vz]`；
控制器只将 EKF 的速度和由速度得到的加速度按 `ekf_kinematics_weight` 与 NOKOV
滤波结果混合，位置和 `R_WB` 始终来自 NOKOV。

## 控制器流程

`ctbr_controller.py` 将控制器与 ROS I/O 分离：`GeometricCtbrController` 不依赖 ROS，
只实现几何控制律；`CtbrControllerNode` 负责参数、话题、失效保护和 CSV。飞行任务的
阶段机和参考生成已经独立到 `ctbr_trajectory.py`，后续替换轨迹不需要修改控制律或 CTBR
消息桥接。
外环在 world z-up 坐标系计算：

```
F_d = m (a_d + g e3) - Kp ep - Kv ev - Ki ei
```

再由期望合力方向和目标 yaw 构造 `R_c`，利用 SO(3) 姿态误差产生 p、q、r。默认的
V2 `analytic` 模式不再对相邻 `R_c` 做差分，而是使用：

```text
F_dot = m j_d - Kp ev - Kv (a - a_d) - Ki d(sat(e_i))/dt
b3_dot = (I - b3 b3^T) F_dot / ||F_d||
R_c_dot = [b1_dot, b2_dot, b3_dot]
Omega_c = vee(skew(R_c^T R_c_dot))
```

其中 `a` 是控制器实际采用的（NOKOV/EKF 混合后）加速度；当
`derivatives_valid=false` 或 EKF 数据过期时自动回退到可用的 NOKOV 值，若两者均不可用则按
MATLAB V2 回退为 `a_d`。轨迹目标同时提供 `jerk` 和
`yaw_rate`，使航向投影的导数也进入 `R_c_dot`。`Omega_c` 再通过
`R^T R_c Omega_c` 转到当前机体系并作为 CTBR 角速度前馈。`omega_c_method=log_difference`
仍可用于与旧版离散实现对比。实际发送的总推力为 `F_d` 投影到当前机体 z 轴后的值，
并受 `max_command_thrust_newton` 限制。

当前控制输出模式的状态机为：

```text
第一帧有效状态
  -> preflight_voltage     零 CTBR，读取一次锁存的 pm.vbat 并判断电压
  -> 第一帧有效状态锁定起点
  -> pre_takeoff_hold       保持 reference_hold_s
  -> takeoff                五次轨迹向上 takeoff_height_m
  -> height_correction      保持名义起飞点 x/y，持续跟踪精确起飞高度 takeoff_settle_s
  -> circle_entry           平滑转向并进入圆周起点
  -> circle                 整圈角度按五次 smoothstep 平滑推进，机头始终朝向圆心
  -> final_hover            一圈完成后固定悬停 final_hover_s
  -> landing                保持最终圆周 x/y，z 五次轨迹回到起飞前高度
  -> landing_settle         到达落地高度附近的确认阶段
  -> landed                 持续发布零 CTBR，禁止重力补偿再次起飞
  -> aborted                Nokov 连续失效超过 0.25 s 后锁定零 CTBR，需人工重启
```

预检仅在 `enable_control=true` 时阻塞飞行。一次性电池日志没有新鲜样本、中位电压低于
`3.8 V`、或计算出的 raw 缩放超出安全范围时，状态机进入 `preflight_voltage_failed` 并
持续发送零 CTBR；需要检查电池/`/cf2/battery` 后重启控制器。采样完成后，默认一阶模型为：

```text
raw_scale = (V_reference / V_preflight)^raw_scale_exponent
```

默认 `V_reference=4.20 V`、`raw_scale_exponent=1.0`，对应假设
`F ∝ (raw * V)^2`。`V_reference` 必须在后续 120 gf 静态推力标定时替换为实际标定电压；
冻结值在全程保持不变，绝不逐帧跟随飞行中抖动的 `pm.vbat`。

### 当前圆周任务

当前 `ctbr_controller.launch` 默认任务为：相对第一帧有效高度上升 `1 m`，起飞参考结束后继续使用
名义起飞点 x/y，在 `takeoff_settle_s` 时间内持续跟踪精确高度，再以半径 `1 m` 画一圈。圆心默认是第一帧
x/y 向 `+x` 偏移 `1 m`，
圆周起始角为 `pi`，所以起飞点本身正好位于圆周左端点，不会在升空后立即横移一米。

圆周路径仍使用解析圆周位置，但整圈的非匀速角度由五次 smoothstep 生成。令
`u=clip(t/T, 0, 1)`，`S(u)=10u^3-15u^4+6u^5`，则：

```text
p_d = c + r [cos(theta), sin(theta), 0]
theta = theta_start + 2*pi*N*S(u)
theta_dot = 2*pi*N*S'(u)/T
theta_ddot = 2*pi*N*S''(u)/T^2
theta_dddot = 2*pi*N*S'''(u)/T^3
v_d = r theta_dot [-sin(theta), cos(theta), 0]
a_d = r [-theta_dot^2 cos(theta) - theta_ddot sin(theta),
         -theta_dot^2 sin(theta) + theta_ddot cos(theta), 0]
j_d = r [-3 theta_dot theta_ddot e_r
         + (theta_dddot - theta_dot^3) e_t]
yaw_d = atan2(c_y - y_d, c_x - x_d)
yaw_rate_d = theta_dot
```

因此 `yaw_d` 始终指向圆心，且圆周起止点的速度、加速度均为零。配置项
`circle_angular_speed_radps` 表示整圈轨迹的峰值角速度；由于
`max(S')=1.875`，整圈时长为 `T=1.875*(2*pi*N)/circle_angular_speed_radps`。
默认 `0.55 rad/s`、半径 `1 m` 时峰值切向速度为 `0.55 m/s`，一圈约 `21.4 s`。
旧的 `circle_ramp_duration_s` 仅为兼容已有参数保留，不再参与轨迹计算。

起飞前的 `pre_takeoff_hold` 只锁定坐标原点和等待零推力解锁包，明确发送零 CTBR，
不会提前输出重力补偿。短于 `trajectory_pause_abort_s=1.0 s` 的 Nokov 丢帧会冻结轨迹
计时；更长的失效将进入 `aborted` 并持续零推力，恢复动捕后也不会自动追赶旧参考。

一圈结束后在最终圆周点悬停 `3 s`，再只沿 z 轴下降至第一帧高度。通过 launch 的
`circle_center_offset_x/y`、`circle_radius_m`、`circle_revolutions`、
`circle_angular_speed_radps` 和 `circle_start_angle_rad` 可修改轨迹几何。

## 修改历程

以下按本次工作从最初接口改造到当前可飞行原型的实施顺序记录，便于后续更换控制律时
区分“基础设施”与“当前实验参数”。

1. 增加 Nokov 后端支持，并将每架刚体的位姿、线速度、机体系角速度和线加速度发布为
   `/cf<ID>/mocap_state`。动捕丢失时的 `9999.999 m` 哨兵值和异常时间间隔会被拒绝。
2. 增加 `CTBR.msg`、`/cf<ID>/cmd_ctbr` 桥接、输入限幅和 0.1 s watchdog。主机端以 N 和
   rad/s 表达指令，服务器负责发送现有固件支持的 legacy RPYT rate 包，不修改已烧录固件。
3. 增加无刷 CF21BL 的 ARM/DISARM 包装脚本，并在 launch 中配置 RATE 模式、40 g 质量、
   四电机合计 120 gf 最大推力和 CTBR 符号参数。
4. 新建 Python 几何 CTBR 外环。先以影子模式验证状态、目标和日志，再加入“第一帧锁定
   起点、起飞、起飞稳定、任务轨迹、最终点固定悬停、垂直降落”的单一低层状态机。
5. 根据实飞日志修正推力映射：由错误的线性 N-to-raw 改为平方曲线反解；将任务结束后的
   行为改为固定 3 s 悬停，避免位置门控反复重置定时器。
6. 新建 CTBR CSV 和可视化程序。CSV 统一保存至 `scripts/ctbr_logs/`，可视化自动选择最新
   文件、过滤明显超出动捕工作空间的位置，并显示完整图例与单位。
7. 新增 EKF 平移日志和 `pm.vbat` 主电池遥测。服务端把 ROS 通用日志与旧式
   `logcf<ID>.csv` 分离：启动 `/cf2/ekf_kinematics` 连续日志和 `/cf2/battery` 一次性日志，
   电池数据写入 CTBR CSV，不在服务端工作目录创建额外日志文件。
8. 新增预起飞电压冻结阶段和 `CTBR.thrust_raw_scale`。控制器先发送零包、取电压中位数，
   再把有界 raw PWM 缩放值随每条 CTBR 命令发送；飞行中不根据电压遥测改变该值。
9. 将原先写在 `ctbr_controller.py` 内的起飞、位移、悬停和降落状态机拆分到
   `ctbr_trajectory.py`。当前实现为“起飞 1 m、半径 1 m 圆周、机头朝向圆心、一圈后悬停
   3 s 再降落”；圆周角度和边界均使用五次平滑参考。

## 文件与实施记录

| 文件 | 已实施内容 |
| --- | --- |
| `ros_ws/src/crazyswarm/msg/MocapState.msg` | Nokov 位姿、速度、机体系角速度、加速度及有效性标志。 |
| `ros_ws/src/crazyswarm/msg/CTBR.msg` | p/q/r（rad/s）、整机总推力（N）和冻结 raw PWM 缩放的主机接口。 |
| `ros_ws/src/crazyswarm/src/crazyswarm_server.cpp` | 发布 mocap 状态、订阅 CTBR、输入限幅/超时保护、角速度符号转换、N 到 raw thrust 的平方曲线反解、冻结 raw 缩放和 ROS 专用 generic 日志开关。 |
| `ros_ws/src/crazyswarm/launch/hover_swarm.launch` | RATE 模式、Nokov 边界、CTBR 桥接参数、自动 ARM wrapper 和固件电池日志块。 |
| `ros_ws/src/crazyswarm/scripts/ctbr_controller.py` | 影子模式、几何 CTBR 控制律、V2 analytic/log-difference Omega_c、预起飞电压冻结、ROS 话题、失效保护、CSV 与电池遥测缓存；不再保存轨迹公式或飞行阶段机。 |
| `ros_ws/src/crazyswarm/scripts/ctbr_trajectory.py` | 独立圆周任务状态机、实际起飞/落地门控、五次角度平滑的解析圆周位置/速度/加速度/jerk、朝向圆心 yaw 与 yaw_rate、平滑入圆/降落和离线测试。 |
| `ros_ws/src/crazyswarm/launch/ctbr_controller.launch` | 半径 1 m 圆周任务默认参数、圆心相对起点偏移、预起飞电压门控、40 g 质量、120 gf 总推力和降落参数。 |
| `ros_ws/src/crazyswarm/scripts/ctbr_visualization.py` | 自动选择最新 CSV、异常位置剔除、带单位和图例的四图可视化、电池电压图以及预检冻结值。 |
| `ros_ws/src/crazyswarm/scripts/hover_swarm.py` | 独立的高层 takeoff-hover-land 验证脚本；不与 CTBR 流式控制混用。 |

两个 CTBR Python 文件和可视化脚本已加入 `CMakeLists.txt` 的安装清单；使用
`catkin_make install` 时，控制器和轨迹模块会被安装到同一 ROS 可执行目录，
`ctbr_controller.py` 可以继续通过同目录导入 `ctbr_trajectory.py`；对应的
`ctbr_controller.launch` 也会安装到包的 `launch/` 目录。

### 电池遥测与推力标定

当前硬件的无刷电机路径不使用固件的电压推力补偿。因此固定 `raw_thrust` 在低电压时
产生的实际推力会下降；以满电 `Fmax` 建立的 CTBR 映射不可能自动适用于所有电压。

为区分“电压不足”和“控制器/动捕问题”，`hover_swarm.launch` 现已设置
`enable_generic_logging=true`。其中 EKF 运动学日志按 10 ms 周期发布，电池日志的周期设为
`0`，由固件只采样一次；服务端对 GenericLogData publisher 使用 latch，控制器晚订阅时仍能
收到这一个样本：

```text
/cf2/battery  crazyswarm/GenericLogData
values[0] = pm.vbat          # V
values[1] = pm.vbatMV        # mV
values[2] = pm.state
values[3] = pm.batteryLevel
```

新启动的 `ctbr_controller.py` 会把这些量写入同一个 CSV。旧 CSV 没有这些字段，不能
事后证明旧电池是否低压。对单节电池，实验安全阈值暂定为 `3.8 V` 警告、`3.7 V` 开始
受控降落；固件的 3.2/3.0 V 只是最后硬件保护，不能作为正常飞行阈值。

当前控制输出还要求预起飞电压不低于 `3.8 V`。获得这一个新鲜 `pm.vbat` 样本后，
控制器把电压、样本数、预检状态和固定 `thrust_raw_scale` 写入 CSV，并由可视化脚本
在 Battery Voltage 图中显示中位电压虚线。预检阶段始终发送零 CTBR，不会把采样等待
误当成悬停。

`enable_logging` 仍只用于兼容旧式 `logcf<ID>.csv` 文件；当前保持为 `false`。这避免
服务器在未知工作目录写入第二份 CSV，所有本次飞行数据仍只保存到
`scripts/ctbr_logs/`。

当前 `crazyflies.yaml` 中 `cf2` 的类型仍为 `medium`，而该类型配置的是 bigQuad 和
7.6/7.4 V 阈值，不符合单节 CF21BL。这一配置错配尚未修改，不能使用 chooser 的该类型
电压判断作为飞行依据；CTBR 电池日志固定读取正确的 `pm.vbat`，不读取 `pm.extVbat`。

当前 CTBR 飞机的身份和无线地址统一保存在 `launch/crazyflies.yaml`：`id` 生成 ROS
命名空间 `/cf<ID>`，`uri` 是 cflib 和 C++ server 使用的完整 `radio://...` 地址，
`ctbr_enabled: true` 标记默认控制对象。`cf_arm.py`、ARM wrapper 和 CTBR 控制器均从
该条目读取配置；换飞机时只需修改对应条目的 `id`、`uri`，必要时修改 NOKOV 刚体名。
其中 `initialPosition` 仍是 Crazyswarm 物体跟踪/仿真的初始猜测，不是实时控制状态；
实时位置、姿态和导数来自 NOKOV。`type` 仍由服务端用于选择 marker、动力学和固件参数，
因此不能因为 CTBR 外环参数已在 `ctbr_controller.yaml` 中就删除。

### 已观察的实飞结果

- 早期日志 `cf2_ctbr_20260826_213702.csv` 中，第一阶段目标约为 `z=1.0435 m`，实测
  最高约 `0.7313 m`，所以原先的“必须到达起飞高度才执行后续任务”门控没有放行。这一段
  同时受螺旋桨故障、旧推力线性映射和电池状态等因素影响，不能据此单独归因。
- 使用满电电池后的 `cf2_ctbr_20260826_221849.csv` 已完成起飞、旧位移任务和完整的
  3 s 最终点悬停，并进入 15 s 降落参考。这支持“电压会影响可用推力”的判断，但这次
  日志产生于电压遥测加入之前，仍不能量化证明。
- 同一份最新日志中，落地确认多次短暂进入又退出。落地高度已接近起点，但 Nokov 差分
  的 `vz` 多次快速变化并被裁剪到 `1.0 m/s`，使速度确认无法连续满足 1 s；此外，旧逻辑
  在 `landing_settle` 仍运行重力补偿，可能把飞机重新托起。现已改为落地参考结束后和
  `landing_settle` 均发送零 CTBR，并对短暂速度尖峰提供 0.5 s 容忍窗口；持续超限才会
  重新进入降落阶段。这不是单纯电压问题。
- `cf2_ctbr_20260827_133706.csv` 已首次记录电压：其起飞前的静止记录段中位数约为
  `4.049 V`，飞行后期中位数约为 `3.90 V`。按当前暂定 `4.20 V` 参考值，下一次真正的
  预检将使用约 `1.037` 的 raw 缩放。该日志生成于预检冻结功能加入之前，图中的
  `0..0.8 N` 高频命令跳变不能由当时仅记录的电压话题直接造成。

## 运行与数据查看

先启动服务器（该 launch 会自动 ARM 实机），再启动 CTBR 控制器：

```bash
source ~/my_project/crazyswarm/ros_ws/devel/setup.bash
roslaunch crazyswarm hover_swarm.launch
```

在另一个终端：

```bash
source ~/my_project/crazyswarm/ros_ws/devel/setup.bash
roslaunch crazyswarm ctbr_controller.launch enable_control:=true target_confirmed:=true
```

日志保存在 `ros_ws/src/crazyswarm/scripts/ctbr_logs/`。绘制最新日志：

```bash
cd ~/my_project/crazyswarm/ros_ws/src/crazyswarm/scripts
python3 ctbr_visualization.py
```

停止 CTBR 节点会发送零推力；除非飞机已经可靠接地，不要用 `Ctrl-C` 作为降落方式。
每次改变 `hover_swarm.launch` 的电池日志配置后，必须重启 `hover_swarm.launch`；每次改
Python 控制器后，必须重启 `ctbr_controller.launch`。

## 验证结果

已完成的离线验证：

- `catkin_make --pkg crazyswarm` 已成功构建服务器和新消息，包括 ROS 专用 generic
  logging 开关以及新增的 `CTBR.thrust_raw_scale`；
- `ctbr_controller.py --self-test` 验证了 40 g 悬停推力 `0.392266 N`、零姿态角速度、
  独立圆周轨迹的半径/速度/加速度/yaw/阶段门控；圆周轨迹单元测试进一步验证了整圈五次角度平滑、
  起止速度/加速度为零和峰值角速度语义。该自检还验证了预检先保持零推力、取新电压样本中位数、冻结
  `4.20 / 4.05` 的 raw 缩放，且后续低电压样本不会改写冻结值；中位数 `3.750 V`
  时会可靠进入失败状态并保持零推力；
- Python 语法检查与两个 launch 文件的 XML 检查通过；
- `roslaunch crazyswarm ctbr_controller.launch --dump-params` 应确认起飞、半径 1 m 圆周、
  3 s 悬停、降落和推力参数加载正确；
- `roslaunch crazyswarm hover_swarm.launch --dump-params` 已确认
  `enable_generic_logging=true`、`enable_logging=false`、10 ms 的 `ekf_kinematics` 日志块、
  一次性的 `battery` 日志块及其 `pm.vbat` 字段顺序；
- 伪 `GenericLogData` 回调已验证：`[4.05, 4050, 1, 86]` 会写入对应的电压、电压毫伏值、
  电源状态和电量字段；
- 使用真实的历史电池日志和无窗口 matplotlib 验证了可视化；使用含预检字段的内存样本
  验证了 Battery Voltage 图会显示冻结中位电压和 raw 缩放值；
- 圆周模块离线仿真确认：起飞参考结束后会保持名义起飞点 x/y 并持续跟踪精确起飞高度，再进入圆周；整圈五次角度平滑结束后悬停 3 s 并进入
  降落参考，阶段计时只在真实阶段转换时开始；落地参考结束和稳定确认阶段均保持零推力，
  单次速度尖峰不会清零确认计时，持续超限才重新进入降落；动捕短暂失效会暂停阶段计时，
  超时后锁定零推力中止。

尚未完成硬件验收的项目见下一节。离线测试不验证无线链路、螺旋桨、电池内阻、实际
推力曲线、Nokov 坐标方向或飞行稳定性。

## 完成与未完成清单

### 已完成

- [x] Nokov 状态的 ROS 发布，包括速度、机体系角速度和线加速度。
- [x] 主机 CTBR 消息、服务器桥接、输入验证、watchdog、角速度符号和 RATE 模式设置。
- [x] 40 g / 120 gf 总推力的物理接口和平方 raw thrust 映射。
- [x] 影子模式、双确认实际控制、第一帧起点锁定、起飞门控、独立圆周轨迹、固定 3 s
  悬停和垂直降落参考。
- [x] 圆周主段的解析位置/速度/加速度及“机头指向圆心” yaw 参考；控制器和轨迹模块
  已拆分，可单独替换 `ctbr_trajectory.py`。
- [x] CSV 自动存储、异常 Nokov 位置过滤、最新日志自动绘图和电池遥测写入接口。
- [x] EKF 位置/速度日志接入、速度/加速度按权重混合，且位置和姿态保持 NOKOV 来源。
- [x] ROS EKF/一次性电池日志与旧式服务器 CSV 分离，避免额外写入 `logcf<ID>.csv`。
- [x] 起飞前零推力电压中位数预检、冻结 raw PWM 缩放、低电/缺样本拒绝起飞和 CSV 诊断字段。
- [x] 落地末端零推力输出及 0.5 s 速度尖峰容忍，避免重力补偿重新托起飞机并反复进入落地确认。
- [x] 控制器关键数学、状态机、失效保护、话题和日志接口的中文代码注释。

### 已实现但需要实飞确认

- [ ] 重启服务器后确认 `/cf2/battery` 存在，且新 CSV 的 `battery_voltage_v` 非空；
  随后确认 `preflight_voltage_v`、`thrust_raw_scale` 和预检阶段均符合预期，再比较满电与
  不同电压的高度误差和命令推力。
- [ ] 校验平方推力曲线在不同电压下的实际悬停 raw 值，并形成 `F(raw, V)` 标定表或
  电压补偿模型。
- [ ] 在安全环境中确认 roll、pitch、yaw 三轴正负号和全部 CTBR 限幅均符合实机方向。
- [ ] 在影子模式检查新的圆周 CSV：圆心、半径、yaw 方向和阶段顺序正确后，再以低速
  实飞确认实际圆心相对起飞点的位置符合场地安排。
- [ ] 确认起飞高度 `起点 z + 1 m` 和半径 1 m 圆周均处于 Nokov 覆盖范围，并调节 z 轴
  位置/速度/积分增益以减小高度静差。

### 尚未完成或需要修改

- [ ] 自动落地仍需实飞验收：当前使用落地高度、垂直速度容差和 0.5 s 容忍窗口，后续应
  根据更多位置、`vz` 和接触行为数据，验证窗口是否合适，不能只依赖单次瞬时速度。
- [ ] `crazyflies.yaml` 的 `type: medium` 与单节 CF21BL 不匹配；需要选择或建立正确的
  CF21BL 类型，尤其是电池阈值和 marker 配置。
- [ ] 当前预检电压补偿仍是 `F ∝ (raw * V)^2` 的一阶假设，未标定电压指数、内阻压降、
  温度、螺旋桨和电机差异；不能把 `1.176798 N` 视为所有电压下的真实最大推力。
- [ ] 多机命名空间、避碰、通信丢失后的更高层受控返航/降落策略尚未实现。
