本目录为离线 MuJoCo 对照，使用真实任务/控制器代码，无 ROS、无线电或实机连接。新增 `slung_payload.simulation.initial_payload_rpy_deg` 直接设置载荷自由关节的物理姿态，单位为度，旋转为 `Rz(yaw) @ Ry(pitch) @ Rx(roll)`；默认 `[0,0,0]`。倾斜箱体按最低角点重新计算中心离地高度。未设置飞机坐标时，每架飞机随对应载荷挂点位移，保留原世界绳向和绳长；`initial_vehicle_positions_m` 显式值始终是世界坐标。`--initial-tilt` 仍只设置飞机 roll。

候选由 `prepare_tilted_starts.py` 从 `multi_ctbr_20261018_211815.csv` 的首个 tension_ramp 周期恢复。姿态来自日志自身的 `q` 与位置几何反演，约 `[6.0363,20.4093,-10.4268]°`，不是独立的姿态测量。原始端点几何距离约 `[.862995,.858672,.863754] m`；直接复制会超过本次指定的 `.850 m` 物理绳长。本实验保留原始世界绳向，将三机端点沿各自绳方向投影到 `.850 m`，载荷中心平移到地面，中心 z 为 `.0402064 m`，初始最小机间距 `.499151 m`。三根初始物理绳长误差均小于 `2e-16 m`。姿态、中心、上表面中心、四元数、旋转矩阵和三个实际挂点均记录在 `input_parameters.yaml` 与 `summary.json`，首行物理真值也记录在 `simulation_trace.csv`。

四组只改变参考一致性开关及载荷转动阻尼。均为23 g、三根物理/名义绳长 `.850 m`、固定70 Hz控制、70 Hz采样EKF反馈、目标中心高度 `.60 m`，候选关闭定时实飞验证与在线绳长校准。执行器采用现有 Graefe2026/Mellinger 参数，桥接 thrust_raw_scale 冻结为该次实飞的三个值。未修改下降或退出保护逻辑。完整有效参数以各目录的 `input_parameters.yaml` 为准。

| 转动阻尼 | 一致性参考 | 实际接管 pitch | 接管后观测 | 末样本 pitch | 结果 |
|---|---|---:|---:|---:|---|
| .001 | 关闭 | 20.8912° | 60.0143 s | 4.1744° | 完成，未见增长振荡 |
| .001 | 开启 | 20.8912° | 60.0143 s | 3.3278° | 完成，静态偏差小幅减小 |
| 0 | 关闭 | 19.0589° | 12.1714 s | 7.5353° | 触发既有0.30 m机间距保护 |
| 0 | 开启 | 19.0589° | 11.6571 s | 9.3510° | 触发既有0.30 m机间距保护 |

默认阻尼下两组都稳定，因此这组对照不能证明实飞增长振荡已经解决。零阻尼边界下，两组均出现约2.5 Hz、以 yaw 增长最显著的失稳，开启新参考路径未延迟保护触发。它支持对物理阻尼和执行器/绳向闭环稳定性的进一步验证，不能作为新参考路径已修复振荡的证据。零阻尼运行按原有保护提前结束，未强行续跑60秒，也未放宽保护。

`comparison.png` 对比载荷 pitch/roll/yaw 和最小机间距；虚线5 s为张力渐增结束时刻，叉号是该组保护停止前最后一个物理记录样本。`comparison.json` 与各目录的 `physical_attitude_metrics.json` 保存实际接管姿态、时窗统计和1–3 Hz频带指标。指标时窗的 `actual_duration_s` 可识别提前结束的不完整窗；不可将5–15秒标签误读为完整10秒观测。

实测控制周期序列另保存在 `control_intervals_latest.json`：去掉115.4 ms启动等待后有4369个实际间隔，平均14.2857 ms，范围3.9745–25.0628 ms。本次四组使用固定70 Hz，未使用该抖动序列；重放时应保留原始次序，因为长短周期存在明显交替。生成器默认固定70 Hz；只有显式传入 `--measured-intervals` 才会生成使用实测序列的候选，避免无意改变已完成对照。

复现单组，从 scripts 目录运行：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /home/csy/miniconda3/envs/udaan/bin/python simulate_slung_handoff.py --candidate ctbr_logs/transport_reference_20261018/baseline_observed_q_damping001/candidate.yaml --output ctbr_logs/transport_reference_20261018/baseline_observed_q_damping001
python ctbr_logs/transport_reference_20261018/compare_cases.py
```

新增姿态功能先以11个失败测试确认缺失行为，再实现通过；与初始飞机坐标、观测姿态噪声和物理配置检查合并为29个通过测试。原有物理配置测试将54 g写死，现改为检查当前控制器配置质量，仍能捕获仿真未继承所配置质量的问题。仿真脚本通过Python3.8语法解析检查。

后续零阻尼绳向增益辨识见 `link_gain_probe_notes.md` 与 `link_gain_comparison.png/json`。降低kq/kω后，间距保护由11.66秒推迟到15.70秒、21.49秒，但三组仍均未通过60秒；它提供闭环敏感性证据，仅几何参考和这两组降增益尚未修复增长。再后续的 `reference_cascade` 结构已在四组60秒离线对照中通过，结果与约4.25 cm残余高度偏差见 `cascade_validation_notes.md`、`cascade_comparison.png/json`。
