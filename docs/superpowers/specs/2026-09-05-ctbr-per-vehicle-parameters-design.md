# CTBR Per-Vehicle Parameters Design

## Goal

将目前集中在 `ctbr_controller` 的 ROS 参数拆为全局调度、公共轨迹和每架飞机的专属
控制参数。控制器必须根据 `crazyflies.yaml` 的飞机 `id` 自动选择正确参数块；以后新增
启用 CTBR 的飞机时，只需新增 `ctbr_controller_cf<ID>` 参数块，无需改 Python 源码。

## Scope

本变更只重组 CTBR 控制器的 ROS 参数与读取方式，不改变：

- `crazyflies.yaml` 中的飞机身份、URI、`ctbr_enabled` 或 `orbit_phase_rad` 的含义；
- NOKOV 位置/姿态、EKF 速度/加速度的控制数据来源；
- 现有圆周和三角形八字轨迹的数学定义；
- CSV 字段、CTBR 消息接口和 C++ 无线电桥接。

## Parameter Namespaces

ROS 参数 token 使用下划线，避免连字符带来的非法或不兼容名称。因此每机参数块采用
`ctbr_controller_cf<ID>`，例如 `ctbr_controller_cf2`，而不是
`ctbr-controller-cf2`。

### `/ctbr_controller` — shared coordinator and safety parameters

保留所有多机共同使用、不能因机体调参而悄然分叉的参数：

- 启动与调度：`target_confirmed`、`takeoff_vehicle_count`、`control_rate_hz`；
- NOKOV 状态有效性与限幅：`state_timeout`、时间戳容差、工作空间边界、速度/加速度
  限幅及 NOKOV 低通参数；
- EKF 健康检查与滤波：EKF 权重、超时、NOKOV-EKF 位置差、预检对齐、故障保持、紧急
  降落以及 EKF 滤波参数；
- 任务安全：`trajectory_pause_abort_s`；
- 通用预检准入：是否要求电压预检、采样数、超时、样本年龄与最低允许电压；
- 输出基础设施：`log_directory`、`path_frame_id`、`path_max_poses`、
  `path_publish_interval_s`。

### `/ctbr_trajectory` — shared trajectory parameters

把所有几何参考、时序、编队和落地参考参数移动到该块：

- 轨迹选择及几何：`trajectory_mode`、`circle_center_xy`、旧的圆心偏移、圆半径、圈数、
  起始角、三角形边长、八字范围与速度；
- 起飞、入轨、终点悬停和降落的时间/距离/速度/容差；
- 轨迹阶段共同的最大倾角：`takeoff_max_tilt_deg`、`circle_max_tilt_deg`、
  `landing_max_tilt_deg`。

同一次多机任务中的每个 `CtbrControllerNode` 构造内容相同的轨迹配置，唯一例外是仍从
`crazyflies.yaml` 注入的 `orbit_phase_rad`，它决定圆周相位或三角形顶点偏移。

### `/ctbr_controller_cf<ID>` — vehicle-specific controller calibration

每架启用 CTBR 的飞机必须有一个同名块。例如 `id: 4` 对应
`/ctbr_controller_cf4`。该块只包含容易因机体、螺旋桨、电机、推力标定而不同的控制
参数：

- 物理与推力：`mass_kg`、`max_total_thrust_newton`、
  `max_command_thrust_newton`、飞行阶段最低推力；
- 几何外环约束与增益：默认最大倾角、最大机体系角速度、位置/速度/位置积分增益及
  积分限幅；
- 姿态外环增益：姿态增益、姿态积分增益及积分限幅；
- V2 期望角速度前馈：`position_integral_c1`、`use_body_rate_feedforward`、
  `omega_c_method`、两个数值保护阈值；
- 电压到 raw-PWM 的机体标定：参考电压、缩放指数、缩放上下限。

`gravity_mps2` 保持为公共物理常数，放在 `/ctbr_controller`，不做每机复制。

## Runtime Resolution and Validation

`CtbrControllerNode` 在解析 `vehicle_config["id"]` 后计算三个绝对参数根：

```text
/ctbr_controller
/ctbr_trajectory
/ctbr_controller_cf<ID>
```

新增一个小型参数读取适配层，分别提供 `global_param()`、`trajectory_param()` 和
`vehicle_param()`。所有已有 `rospy.get_param("~...")` 调用迁移到相应入口，使读取位置
可审计，且不会依赖节点私有命名空间的偶然行为。

在创建控制器前必须验证 `/ctbr_controller_cf<ID>` 存在且为字典。缺少块、字段缺失或值
无效时抛出 `rospy.ROSInitException`，错误中同时显示 CF ID 与期望路径，例如：

```text
CF4 缺少专属参数块：/ctbr_controller_cf4
```

多机管理器仍从 `crazyflies.yaml` 选择全部 `ctbr_enabled: true` 条目；因此新增一架
飞机的配置步骤明确为：增加飞机条目、设置 `ctbr_enabled: true`、并增加同 ID 的专属
控制参数块。不会静默继承 CF2 或共享默认控制增益。

## YAML Layout

配置文件顶层结构变为：

```yaml
ctbr_controller:
  target_confirmed: $(arg target_confirmed)
  takeoff_vehicle_count: 3
  control_rate_hz: 60.0
  # shared safety, EKF, logging and Path settings

ctbr_trajectory:
  trajectory_mode: figure_eight_triangle
  circle_center_xy: [0.0, 0.0]
  formation_side_length_m: 0.8
  figure_eight_radius_m: 0.8
  # shared flight-reference geometry, timing and phase limits

ctbr_controller_cf2:
  mass_kg: 0.0434
  max_total_thrust_newton: 1.176798
  # CF2 gains and thrust calibration

ctbr_controller_cf4:
  mass_kg: 0.0434
  max_total_thrust_newton: 1.176798
  # CF4 gains and thrust calibration

ctbr_controller_cf5:
  mass_kg: 0.0434
  max_total_thrust_newton: 1.176798
  # CF5 gains and thrust calibration
```

`ctbr_visualization` 块保持原样。

## Tests

新增不依赖 ROS 主机的测试，覆盖：

1. ID 到 `/ctbr_controller_cf<ID>` 名称的映射；
2. CF2、CF4、CF5 分别获取自己的质量、推力和增益，而非共享同一块；
3. 缺少某个启用飞机的专属块时启动被拒绝且报错包含 ID/路径；
4. `ctbr_trajectory` 仅提供公共轨迹值，三架飞机的 `orbit_phase_rad` 仍来自
   `crazyflies.yaml`；
5. 完整 YAML 可解析，且 `crazyflies.yaml` 中当前启用的三个 ID 都有参数块。

现有几何控制、轨迹、可视化和车辆配置测试必须继续通过。

## Non-goals

- 不为未知飞机自动生成参数；
- 不在 `crazyflies.yaml` 中复制控制增益或推力标定；
- 不改变多机的共同起飞门控、无线电传输频率或 CSV 格式；
- 不将轨迹参数分别复制到 CF2/CF4/CF5。
