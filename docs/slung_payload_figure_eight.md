# 负载八字轨迹

当前模式为 `circle`，[固定质心圆锥姿态](slung_payload_attitude_cone.md) 保留为切换选项。以下配置和运行时间对应
将 `trajectory.mode` 切换为 `figure_eight` 后的完整八字任务，保留用于切换和对照。

新增模式为 `slung_payload.trajectory.mode: figure_eight`，与原 `hover`、`circle`
共用实飞和仿真的参考生成流程。它由两个精确相切圆组成，不使用正弦近似的八字曲线。

## 当前配置

任务参数位于 `ros_ws/src/crazyswarm/config/slung_payload.yaml`，
仿真观察窗口与电机/传感器设置位于独立 `config/slung_simulation.yaml`：

```yaml
slung_payload:
  target_payload_height_m: 0.60
  trajectory:
    mode: figure_eight
    radius_m: 0.50
    center_xy: [0.0, 0.0]
    speed_mps: 0.10
    direction: ccw
    revolutions: 1.0
    ramp_s: 5.0
    settle_s: 3.0
  flight_validation_transport_s: 100.0
slung_simulation:             # 实际放入独立 config/slung_simulation.yaml。
  post_handoff_s: 100.0
```

`center_xy` 在八字模式中表示世界坐标下的交点。当前交点 `[0,0]`，右圆心 `[0.5,0]`，
左圆心 `[-0.5,0]`，半径均为 0.5 m。整体 XY 覆盖 X∈[-1,1] m、Y∈[-0.5,0.5] m。
高度 0.6 m 指负载几何中心。参考先抬升到交点上方，再开始八字。
省略 `center_xy` 时，交点取原三机起飞 XY 均值；不得填写 `null`，ROS1 参数服务器不支持空值。

`direction: ccw` 表示先右圆逆时针、再左圆顺时针，首次离开交点沿世界 −Y 方向。
`cw` 为其镜像：右圆顺时针、左圆逆时针，首次沿 +Y。
`revolutions` 表示完整八字次数，须为正整数；单圆模式仍允许非整数圈数。

普通飞机参数 `ctbr_trajectory.trajectory_mode` 保持 `hover`，
`ctbr_trajectory.hover_duration_s` 当前为 100 s，覆盖整个负载轨迹。
只修改普通飞机轨迹为八字不会启用这里的负载参考。

## 切线朝向与交点连续性

八字模式的负载自身 +X 轴沿运动切线，roll、pitch 目标为零。它与单圆模式“+X 朝向圆心”
是不同策略。起步和停止时速度为零，使用对应的极限切线定义朝向，不通过 `atan2(0,0)` 推算。
第一圆开始前，已有的 12 s 姿态参考平滑过渡到初始切线方向（默认 yaw=−90°）。

默认方向、半径 $r$、单圆相位 $\phi\in[0,2\pi]$，交点为 $c$ 时：

$$
p_{right}=c+[r-r\cos\phi,-r\sin\phi,0]^T,
\qquad \psi_{right}=\phi-\pi/2,
$$

$$
p_{left}=c+[-r+r\cos\phi,-r\sin\phi,0]^T,
\qquad \psi_{left}=3\pi/2-\phi.
$$

因此 $R_de_1=v_d/\|v_d\|$（运动时），负载沿切线运行。
右圆结束的连续 yaw 为 270°，左圆从同一个 270° 开始，随后减小至 −90°。
常规 Euler yaw 在 ±180° 之间包裹，CSV 绘图中的跨界不代表实际旋转矩阵突变。

两个相切圆的曲率方向在交点发生反转。如果以非零速度直接切换，向心加速度和 yaw 角速度
会改变符号。因此每一圆都以五次平滑角速度起步并停车：在中间交点速度、加速度、yaw
角速度和角加速度均降到零，再进入另一圆。没有额外停留时间，也不会在交点重新指定一个跳变航向。

三机参考与负载姿态同步转动，位置、速度、加速度含全部旋转交叉项。已有的
SO(3)/S2 起始构型衔接仍使用固定初末姿态；运行阶段只叠加相对初始切线方向的旋转，
避免将 −90° 初始朝向重复乘入。负载和挂点的角速度、角加速度进入力矩和惯性前馈。
默认 `transport_attitude_trim_dynamic_enabled: false` 时，姿态 trim 在运动中冻结学习，已学偏置随构型转动。
显式设为 `true` 且总开关开启后，trim 使用瞬时完整目标姿态，可在健康的八字参考期间继续学习；
世界系偏置 PVA 在名义构型旋转后叠加，测量和既有端点均不再按轨迹 yaw 反旋或重转。
动态偏置零和，每机范数、速度和加速度受原预算限制，1 s 五次段连接为 C2；冻结时当前段走完后保持。
native 位置 PID 和差分积分仍沿用原策略。额外的角速度、源位姿跳变与恢复门控只暂停新 trim 更新，不改负载核心反馈或观测器。
参数和 CSV 诊断见 [README](../README.md#姿态构型补偿)，世界端点状态与小角累计诊断的区别见
[公式说明第 9 节](slung_controller_formulas.md#9-姿态到构型的慢修正与下降衔接)。
持续跟踪滞后也可能被慢积分学习，参考有界和连续不能保证闭环不振荡；需用相同的几何、机体和延迟条件比较关闭、静态及动态模式，并覆盖交点切换和中途降落。

中途降落从当时完整姿态、角速度和角加速度平滑制动。若起始切线对齐尚未完成，
不会跳到最终 −90°；带 roll/pitch 初始偏差时也保持三轴运动连续，再通过原飞机 PVA 桥接进入下降。

## 时间安排

保留 0.10 m/s 最高速度时，每圆长度约 3.142 m；起步、停止各 5 s 后，每圆参考时长约
36.416 s，整个八字约 72.832 s。当前进入 TAUT_RAMP 后：

| 时间 | 参考过程 |
|---|---|
| h0–5 | 张力渐增，保持起点位置 |
| h0–12 | 姿态/绳向平滑转到水平及初始切线 |
| h5–17 | 负载位置抬升到交点上方 0.6 m |
| h17–20 | 额外参考保持 |
| h20–56.416 | 右圆 |
| h56.416–92.832 | 左圆 |
| h92.832–100 | 交点终点保持 |
| h100 起 | 原协同降落、接地确认、卸载、三机下降停推 |

100 s 是用户为完整八字确认的总窗口。独立文件的 `slung_simulation.post_handoff_s` 是离线观察窗口，
`flight_validation_transport_s` 是实飞计划降落时刻，两者都相对接管计时。
若修改速度、圈数或等待时间，使计划时长超过验证/普通 hover 预算，启动时会拒绝配置。
`settle_s` 仅规定等待，不证明实测姿态已收敛。

## 仿真与可视化

在已有 udaan 环境的 scripts 目录：

```bash
python simulate_slung_handoff.py --output ctbr_logs/figure_eight_check
python plot_slung_circle_trace.py ctbr_logs/figure_eight_check/simulation_trace.csv --save ctbr_logs/figure_eight_check/figure_eight.png --no-show
```

绘图脚本名称保持兼容，能同时识别 circle 和 figure_eight；不加 `--no-show` 则显示交互图。
图中包括实际/期望 XY、负载 +X 方向箭头、两个圆心与交点、XYZ、RPY 及跟踪误差。
需要视频时给仿真命令增加 `--render --record ctbr_logs/figure_eight_check.mp4`。

新增 `payload_trajectory_progress` 表示整个八字进度：右圆完成为 0.5，左圆完成为 1。
CSV 记录两个圆心 `payload_lobe_center_1_*`、`payload_lobe_center_2_*`，
以及期望负载位姿、角速度和角加速度。旧 `payload_circle_*` 字段作为兼容别名保留，
八字模式下其 center 表示交点，不能当成某一个圆心。
summary 的通用 `payload_trajectory_*` 字段区分参考完成与模型实际位置/yaw 跟踪误差。

恢复单圆时改为 `mode: circle`；当前 `center_xy: [0,0]` 将表示单圆世界圆心，
若要恢复原来的“圆心=悬停点+[0.5,0]”，应同时注释掉 `center_xy`。
恢复定点用 `mode: hover`。各模式均读取同一参数文件中的高度和控制增益。

## 本次验证结果

默认延迟 10/5 ms 与加大位姿/命令延迟 20/10 ms 的两组完整 MuJoCo 回放均完成
100 s 接管窗口、完整八字、计划降落及停推。八字期间位置误差 RMS 约 9.39/9.60 mm，
yaw 误差 RMS 约 6.01°/5.94°。参考 +X 精确沿切线并不表示实际 yaw 已经无误差。
自动测试 903 项通过、1 项跳过，另完成真实 Noetic launch 参数加载的离线验证。
动力学模型没有覆盖全部实飞误差，这些结果仅描述仿真跟踪。
