# Velocity-free β 验证记录（2026-11-17）

## 结论与范围

新增 β 分支在现有 Udaan / MuJoCo 环境中跑通了起飞、预抬、接管、悬停/圆锥、放载和三机停推。
这次跑的是 `simulate_slung_handoff.py` 调用的真实主机控制器和状态机，未启动 ROS、radio 或真机。
在给定姿态噪声下，β 降低了角速度反馈误差和飞机指令抖动；名义无噪声悬停没有显著改善，
也不能据此承诺实际负载会更快调平。公式和与 MATLAB 的区别见 [设计说明](slung_velocity_free.md)。

实飞配置现选择 `velocity_free_beta`、`hover`、接管 60 s 后计划降落。
没有改变当前惯量、质量、绳长、姿态/位置控制增益或飞机姿态 PI 参数。
`transport_force_filter_tau_s` 仍为 0。

## 仿真设置

- 主机 70 Hz；位姿 100 Hz；飞机 EKF 模拟遥测 50 Hz；机载环 Mellinger 500 Hz。
- Graefe 电机模型，23 g 负载；参数逐次冻结在各目录 `input_parameters.yaml`。
- 名义延迟：位姿 10 ms，指令 5 ms。扰动对照：位姿 20 ms，指令 10 ms。
- 扰动组各轴姿态噪声标准差 0.003 rad，约 0.172°；新旧方式使用相同随机种子 20261117。
- 悬停的接管验证时长均为 60 s；离线 `post_handoff_s: 100` 不是额外悬停，启用实飞验证时按 60 s 降落。
- 圆锥组半角 10°、每圈 30 s、两圈，接管窗口单独设为 100 s，足以完成参考和下降流程。
- 位置/姿态误差由 MuJoCo 真值计算；表内 RMS 统计接管第 17 秒到计划降落开始前，不计预抬和降落。

| 场景 | 接管时长 | roll / pitch / yaw RMS (°) | 三维位置 RMS (mm) | 最大飞机角速度分量 (rad/s) | 最小机间距离 (m) | 结果 |
|---|---:|---|---:|---:|---:|---|
| 原观测器悬停 | 60 s | 0.199 / 0.127 / 0.023 | 4.45 | 0.0122 | 0.519 | 完成并停推 |
| β 悬停 | 60 s | 0.199 / 0.127 / 0.023 | 4.45 | 0.0122 | 0.519 | 完成并停推 |
| 原观测器：噪声及较大延迟 | 60 s | 0.210 / 0.129 / 0.022 | 4.42 | 0.1957 | 0.519 | 完成并停推 |
| β：相同噪声及延迟 | 60 s | 0.207 / 0.137 / 0.022 | 4.39 | 0.0545 | 0.519 | 完成并停推 |
| β：10° 圆锥两圈 | 100 s | 0.634 / 0.859 / 0.164 | 4.42 | 0.0121 | 0.507 | 完成并停推 |

五组角速度命令饱和占比均为 0。圆锥运行区间最大总姿态误差为 2.232°，最大位置误差为 6.92 mm。

噪声组的三维负载角速度反馈误差 RMS：旧观测器 0.213 rad/s，β 为 0.109 rad/s，约降低 49%。
这里比较的是控制快照的反馈与同时刻物理真值，包含测量延迟，未通过时间平移“消除”滞后。
无噪声时这项误差反而从 0.000054 增至 0.000097 rad/s，均很小，但说明 β 不是无代价的速度真值。
噪声组 pitch RMS 也没有下降，不能把抖动降低等同于姿态拟合精度全面提高。

## 查看数据

数据目录：`ros_ws/src/crazyswarm/scripts/ctbr_logs/velocity_free_20261117/`。

- [新旧对照图](../ros_ws/src/crazyswarm/scripts/ctbr_logs/velocity_free_20261117/comparison.png)
- [可导出的 PDF](../ros_ws/src/crazyswarm/scripts/ctbr_logs/velocity_free_20261117/comparison.pdf)
- [圆锥姿态跟踪图](../ros_ws/src/crazyswarm/scripts/ctbr_logs/velocity_free_20261117/cone_tracking.png)
- `beta_hover/simulation_trace.csv`：β 悬停完整过程。
- `beta_noise_delay/simulation_trace.csv`：β 噪声/延迟实验。
- `beta_cone/simulation_trace.csv`：两圈圆锥。
- `baseline_hover/` 和 `baseline_noise_delay/`：对应旧观测器对照。
- `metrics.json`、`analyze_comparison.py`：统计和图形复现脚本。

CSV、图片是本地验证产物，受 `ctbr_logs` 的忽略规则管理；提交代码后，其他电脑需按保存的候选 YAML 重新生成。
命令及候选入口见 [离线运行](slung_velocity_free.md#6-离线运行)。

## 代码检查及已知未通过项

- 最终常规测试：**1383 passed，4 skipped**。跳过项包含需要额外环境/历史资源的测试。
- udaan 环境的 β、负载核心、ROS 状态边界、日志和仿真输出专项测试：**97 passed**。
- Python 3.8 语法检查通过；没有引入 ROS Noetic 不支持的 `zip(strict=...)` 等写法。
- 数学测试包含：SE(3) 指数与矩阵指数对照、与 MATLAB 的 Adjoint/梯度公式对照、常速运动收敛。
- 接入测试包含：原始 twist 被故意填入错误大数仍不影响 β；重复/倒序/坏帧不推进状态；
  β 超界拒绝；真实长间断重新预热；CSV 持久化；新旧分支的陀螺项差异。

**不能把含 MuJoCo 的全套测试称为全通过。**
`test_slung_prelift_mujoco.py::SlungPreliftMujocoTest::test_real_height_loss_triggers_supported_descent_before_vehicle_collision`
在故意向下拉拽负载后，最后的 `_payload_ready(final_time)` 断言失败。
该实验已经触发 `height_drop` 和受控下降，失败发生在它额外要求终止帧观测仍然 ready 的检查。
用修改前 HEAD 导出的独立代码副本、原 `pose_observer` 配置重新运行，也在同一断言失败。
因此这是已复现的基线问题，本次没有通过放宽保护或修改该断言来把它变绿。

以上通过的五组回放未覆盖全部动捕丢失、线缆松弛、推力偏差、外扰和实机频率变化。
这版已具备实飞选择与日志诊断，但当前证据是仿真验证，不是实飞通过记录。
