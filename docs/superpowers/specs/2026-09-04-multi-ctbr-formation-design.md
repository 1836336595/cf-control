# 多无人机 CTBR 圆周编队设计

## 目标

在一条 Crazyradio 上同时控制 `crazyflies.yaml` 中两个或更多架 Crazyflie，复用现有 CTBR 控制律和 EKF/NOKOV 状态混合逻辑，使所有飞机同步执行以世界原点为圆心、半径 1 m 的圆周轨迹。当前两架飞机位于圆周直径两端并相互朝向；以后通过增加 YAML 条目扩展飞机数量。

## 现状与约束

- `crazyswarm_server` 已支持同一 Radio 上的多个 URI、多个 `/cf<ID>` 命名空间和多个 `cmd_ctbr` 话题。
- 当前 `ctbr_controller.py` 只选择一个 `ctbr_enabled=true` 条目并创建一组控制器、回调、Path 和 CSV。
- 当前 `ctbr_visualization.py` 只读取一份最新 CSV；RViz 的 Path 需要使用各自的 `/cf<ID>/path` 话题。
- 每架飞机的实时位置、姿态和起点必须来自 NOKOV；`initialPosition` 保留为 Crazyswarm 物体跟踪/初始猜测字段。
- CTBR 输出继续通过现有 `/cf<ID>/cmd_ctbr` 接口发送；不改变 C++ CTBR 桥接、推力换算、watchdog 和飞控内环。

## 配置设计

`crazyflies.yaml` 是多机任务的身份和编队配置唯一来源。每个条目保留现有字段并增加：

```yaml
crazyflies:
  - id: 2
    channel: 80
    uri: "radio://0/80/2M/E7E7E7E702"
    ctbr_enabled: true
    type: default
    initialPosition: [1.5, 1.5, 0.0]
    orbit_phase_rad: 0.0
    orbit_yaw_mode: face_partner
  - id: 4
    channel: 80
    uri: "radio://0/80/2M/E7E7E7E704"
    ctbr_enabled: true
    type: default
    initialPosition: [-1.5, 1.5, 0.0]
    orbit_phase_rad: 3.141592653589793
    orbit_yaw_mode: face_partner
```

- `orbit_phase_rad` 决定该飞机在圆周上的初始相位；相位差 `π` 时两机保持直径两端。
- `orbit_yaw_mode=face_partner` 时，期望 yaw 指向编队中与自身相位相差 `π` 的对向位置。对于当前两机，cf2 起始在 `[+1,0]`、cf4 起始在 `[-1,0]`，两机 yaw 相差 `π`。
- 圆心、半径、角速度、圈数和高度仍放在 `ctbr_controller.yaml` 的任务参数中；圆心固定为 `[0,0]`，半径为 `1.0 m`，高度为各自 NOKOV 起点 `z0+1.0 m`。
- `ctbr_enabled` 为 `true` 的条目参加任务；至少需要两架。重复 id、空 URI、重复 URI、不同 channel 或缺少 phase 时在启动阶段拒绝。

## 控制架构

将 `ctbr_controller.py` 改为一个 ROS 节点管理多个 `VehicleController` 实例：

1. 启动时读取所有启用条目，按 id 建立命名空间、`GeometricCtbrController`、轨迹实例、状态缓存、Path 发布器和日志句柄。
2. 每架独立订阅 `/cf<ID>/mocap_state`、`/cf<ID>/ekf_kinematics` 和 `/cf<ID>/battery`，沿用当前 NOKOV 姿态、EKF/NOKOV 速度/加速度混合及限幅逻辑。
3. 单个全局控制时钟计算任务相对时间；每架只根据自己的状态和配置相位生成目标，因此发送时间同步但状态处理隔离。
4. 任务开始前，所有启用飞机都必须完成一次电压预检并拥有有效 NOKOV 状态；未全部就绪时所有输出保持零推力。
5. 任务期间任意飞机状态超时、位置越界、姿态非法或轨迹异常，所有飞机立即发送零推力并进入 aborted 状态，避免编队中单机继续运行。
6. 关机时向所有飞机发送零推力，关闭全部 CSV 文件并发布最终 Path。

## 轨迹与朝向

对第 `i` 架飞机，设圆周相位为 `θ_i(t)=θ_start + orbit_phase_rad_i + θ_motion(t)`，则：

```text
p_i = [R cos(θ_i), R sin(θ_i), z0_i + takeoff_height]
```

速度、加速度、jerk 继续由现有五次 smoothstep/解析圆周轨迹生成。`face_partner` 使用对向相位的圆周位置作为 yaw 参考，使两架当前飞机始终互相朝向；未来多机若没有唯一对向飞机，则退化为朝向圆心，并允许通过配置覆盖。

## 日志设计

采用一份任务级合并 CSV，而不是只保留某一架的“最新 CSV”。每个控制周期为每架飞机写一行，保留现有全部单机字段，并增加：

- `mission_time_s`
- `vehicle_id`
- `radio_uri`
- `orbit_phase_rad`
- `formation_state`
- `global_abort_reason`

字段顺序保持稳定；每架飞机缺失状态时只写该架的 NaN/无效标志，任务级中止原因仍写入所有后续行。文件名采用 `multi_ctbr_YYYYmmdd_HHMMSS.csv`。如需兼容旧的单机工具，可额外生成按 `vehicle_id` 过滤的视图，但不重复写控制数据。

## 可视化与 RViz

- `ctbr_visualization.py` 增加多机 CSV 读取：按 `vehicle_id` 分组，3D 轨迹图显示每架实测/目标轨迹，位置误差、姿态跟踪、速度、加速度和 CTBR 输出使用同一图中的多组颜色/线型。
- 绘图标题和图例包含 `cf<ID>`；`--vehicle-id` 可选择单架，默认显示全部。
- 实时模式跟踪当前任务级 CSV，而不是按文件修改时间在多个单机文件之间跳转。
- 每个控制器实例继续发布 `/cf<ID>/path`，消息类型为 `nav_msgs/Path`、frame 为 `world`；`test.rviz` 增加 cf2、cf4 的 Path display，并预留按命名空间扩展的配置。Path 只绘制该架飞机的实际 NOKOV 轨迹。

## 错误处理与兼容性

- 保留 `~cf_id`、`~cf_prefix` 作为旧单机启动兼容参数；没有指定时启用多机模式。
- 若只有一个 `ctbr_enabled=true` 条目，允许以单机兼容模式运行，但多机任务参数和编队朝向不启用。
- 不改变 `/cf<ID>/mocap_state`、`/cf<ID>/ekf_kinematics`、`/cf<ID>/battery` 和 `/cf<ID>/cmd_ctbr` 话题名称。
- 多机同一 channel/Radio 的合法性由 Crazyswarm server 负责；控制器只验证 URI 和 id 不重复，并在日志中记录每架 URI。

## 验证标准

1. 配置两个条目时，服务端连接两架飞机，控制器同时订阅两个命名空间并创建一个合并 CSV。
2. 两架飞机在未全部通过预检时均保持零推力；全部就绪后同时进入起飞阶段。
3. 影子/离线测试验证相位 `0` 和 `π` 的位置始终相差一个直径，目标 z 为各自 `z0+1.0`，yaw 方向相互对着。
4. CSV 行数和每架 `vehicle_id` 数量一致，字段可被可视化脚本读取；缺失某架状态不会污染其他架的状态列。
5. RViz 同时显示 `/cf2/path` 和 `/cf4/path`，并能在 YAML 增加第三架后按 `/cf3/path` 扩展。
6. 离线单元测试覆盖配置解析、相位/朝向计算、多机任务状态机、合并日志和多机绘图数据分组；Python 文件通过 `py_compile`。
