# 负载圆周跟踪与朝向圆心

当前 `trajectory.mode` 为 `circle`；[圆锥姿态轨迹](slung_payload_attitude_cone.md) 保留为切换选项，
其角度参数已设为 15°。本文下面的单圈、60 s 是原始测试配置示例，当前任务总窗口为 100 s，
圈数与圆心以实际 YAML 或运行快照为准；省略 `center_xy` 可使用自动圆心。

实飞任务参数入口为 `ros_ws/src/crazyswarm/config/slung_payload.yaml`，仿真物理/延迟/观察窗口
位于独立 `config/slung_simulation.yaml`。实飞和
`simulate_slung_handoff.py` 调用同一个 `MultiCtbrControllerNode` 参考生成流程。
普通飞机轨迹 `ctbr_trajectory.trajectory_mode` 保持 `hover`，不要改成普通单机的 `circle`。

## 单圈配置示例和时间

```yaml
slung_payload:
  target_payload_height_m: 0.60
  flight_validation_transport_s: 60.0
  trajectory:
    mode: circle
    radius_m: 0.50
    # center_xy: [0.0, 0.0]  # 可选；省略时自动计算圆心。
    speed_mps: 0.10
    direction: ccw
    revolutions: 1.0
    ramp_s: 5.0
    settle_s: 3.0
slung_simulation:             # 实际放入独立 config/slung_simulation.yaml。
  post_handoff_s: 60.0
```

省略 `center_xy` 表示圆心位于原悬停目标世界 +X 方向 0.5 m 处，负载从圆的最左侧出发，
初始目标 yaw 为零。原悬停目标 XY 取三机起飞位置的均值；每次摆放不同，圆心可能不同。
也可填写绝对世界坐标 `[x, y]`；此时抬升参考会先移动到该圆的最左侧入口。
不要填写 `center_xy: null` 或空值；ROS1 参数服务器不支持这类值，会在启动加载参数时退出。
所有位置均使用负载几何中心，0.6 m 是绝对中心高度。

时间零点为进入 TAUT_RAMP。当前 5 s 张力建立、12 s 抬升，姿态参考在 12 s 内完成；
两者均完成后额外保持 3 s，所以 h20 开始圆周。起步和停车各用 5 s 平滑角速度，
最高角速度 0.2 rad/s、最高线速度 0.1 m/s，一圈总时长约 36.416 s。
h56.416 完成后保持终点，h60 由原验证流程开始协同降落。

`settle_s` 是参考运动后的等待时间，并不检查实测姿态已经收敛。
2026-10-22 17:24:38 实飞日志在早段仍存在明显负载 roll/pitch 偏差，不能据此宣布
h20 的实际状态已经稳定，或把悬停结果当成圆周实飞验证。

更改速度、圈数、等待时间后，若验证截止时间不足以完成圆周，节点会在启动时拒绝该配置，
并输出所需最短时间；普通飞机的 `ctbr_trajectory.hover_duration_s` 也必须覆盖该保守时间预算。
关闭 `yaw_channel_enabled` 与朝向圆心目标不兼容，启动时会报错。
`slung_simulation.post_handoff_s` 是独立文件中的离线观察窗口，`flight_validation_transport_s`
是实飞验证的计划降落时间；生成目录中的 `input_parameters.yaml` 是输出快照。
`mode: hover` 可恢复原定点参考。顺时针用 `direction: cw`。

## 位置、姿态和导数

设原悬停点为 $p_h$、半径为 $r$、圆心为 $c=p_h+[r,0,0]^T$，
正方向圆周相位为 $\theta$。当前参考为：

$$
p_d=c-r[\cos\theta,\sin\theta,0]^T,\qquad R_d=R_z(\theta).
$$

因此负载机体系 +X 轴满足 $R_de_1=(c-p_d)/r$，始终指向圆心；roll、pitch 目标均为零。
正相位为从上向下看逆时针。速度、加速度使用同一相位的解析导数：

$$
v_d=r\dot\theta[\sin\theta,-\cos\theta,0]^T,
$$

$$
a_d=r[\dot\theta^2\cos\theta+\ddot\theta\sin\theta,
\dot\theta^2\sin\theta-\ddot\theta\cos\theta,0]^T.
$$

姿态角速度、角加速度参考分别为 $[0,0,\dot\theta]^T$、$[0,0,\ddot\theta]^T$。
角速度的起停使用五次 smoothstep；其解析积分生成相位，保证位置、速度和加速度连续。
CSV 中常规 Euler yaw 包裹到 $[-\pi,\pi]$，绕一圈时显示的跨界不代表姿态矩阵跳变。
新增 `payload_circle_yaw_rad` 保留连续相位。

## 三机构型与前馈

保持原来的预抬、张力混合、SO(3) 姿态与 S2 绳向平滑衔接。圆周只在这些参考完成后启动，
用 $Q=R_z(\theta)$ 旋转完整的负载相对构型，而不是仅移动负载目标：

$$
x_{i,d}=p_d+Qb_i,\quad
v_{i,d}=v_d+\dot Qb_i,\quad
a_{i,d}=a_d+\ddot Qb_i.
$$

实现同时保留基准构型仍有速度/加速度时的全部交叉项，使用
`slung_circle_reference.rotate_vehicle_references`。两套飞机位置参考、挂点和绳向
同步转动，参考绳长与三机间距保持一致。

现有 `reference_cascade` 合力已经含期望挂点加速度与实测绳向向心项。
圆周补充以下每机加速度前馈，避免重复计算向心加速度：

$$
\Delta f_i=m_i\left(a_{i,d}-a_{attachment,i,d}
-l_{i,d}\|\dot q_{i,d}\|^2q_{i,d}\right).
$$

零跟踪误差时，合力为绳张力支撑加 $m_i(a_{i,d}+g e_3)$。
静止时新增圆周项为零。该项经过原有总推力、倾角和角速度边界，没有更改机载控制器。

保留当前 `native_position_pid`，位置积分在有效反馈下继续运行；差分扰动学习仍在圆周运动期间冻结，当前不叠加差分积分输出。
姿态 trim 默认使用 `transport_attitude_trim_dynamic_enabled: false`：运动期间冻结学习，
已有偏置随构型旋转，并补上相应速度和加速度。

显式启用 `transport_attitude_trim_dynamic_enabled: true` 且保持总开关开启后，trim 使用瞬时完整负载目标姿态，
可在健康的圆周参考期间继续学习，直接在旋转后的名义三机参考上叠加世界系 PVA。
该模式不再反旋测量或按 yaw 重转既有 trim 端点；各机偏置之和为零，每机范数、速度、加速度保持原预算，1 s 五次段连接为 C2。
健康门控暂停新段时，已有段走完后保持端点；降落沿用完整 PVA 残差桥。
额外的角速度、源位姿跳变和恢复门控只作用于 trim 学习，不改核心反馈或观测器。
配置及 CSV 字段见 [README](../README.md#姿态构型补偿)，世界偏置与小角累计诊断的区别见
[公式说明第 9 节](slung_controller_formulas.md#9-姿态到构型的慢修正与下降衔接)。
动态滞后可能被慢积分学习，限幅只能保证补偿自身有界；使用前需对照同条件的关闭、静态和动态仿真，不能据此保证无振荡或实飞误差必然减小。

中途进入协同降落时，负载 yaw 从当时参考平滑减速并保持，不突然切回零。
三机完整位置/速度/加速度参考通过现有有界桥接转入实测绳索构型，关闭姿态 trim 时也保留
这个圆周退出桥接。正常走完一圈后，角速度和角加速度已经为零。

## 仿真和绘图

在原 `udaan` 环境和 scripts 目录运行：

```bash
cd ~/my_project/crazyswarm/ros_ws/src/crazyswarm/scripts
python simulate_slung_handoff.py --output ctbr_logs/circle_check
python plot_slung_circle_trace.py ctbr_logs/circle_check/simulation_trace.csv --save ctbr_logs/circle_check/circle.png --no-show
```

不加 `--no-show` 可以打开绘图界面。需要 MuJoCo 视频时在仿真命令增加
`--render --record ctbr_logs/circle_check.mp4`；该命令读取同一参数文件，不必重复指定半径和高度。
旧候选 YAML 会覆盖同名参数，比较时应检查输出快照。

实飞仍使用原启动命令，新增轨迹通过参数生效：

```bash
roslaunch crazyswarm ctbr_controller.launch target_confirmed:=true enable_realtime_visualization:=false
```

每次仿真同时保留实际控制器 CSV、动力学真值 `simulation_trace.csv`、输入快照和结果 JSON。
新增诊断包括 `payload_trajectory_mode/phase`、圆心、半径、进度、连续 yaw、
期望负载 XYZ/RPY 和构型加速度前馈。完成进度仅说明参考执行完毕；位置/yaw 误差指标
才描述模型中的实际跟踪情况。动力学仿真未包含实际动捕偶发错误、无线丢包、全部机体差异，
也不能替代圆周实飞验证。

2026-10-22 的默认和加大延迟回放均完成完整圆周与降落：圆周位置误差 RMS 约为
8.15/8.54 mm，yaw 误差 RMS 约为 6.07°/6.00°。这些是模型实际跟踪值，
姿态参考的向心一致性单独由几何检查验证；不要将后者当成实际姿态已经无误差。
