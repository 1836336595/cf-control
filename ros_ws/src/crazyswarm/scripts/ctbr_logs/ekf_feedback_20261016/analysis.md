# 第 3 步：50 Hz EKF 回传与实飞主机滤波

这次修改接入固件日志的采样/延迟和实际主机 `_ekf_state_callback`，未改实飞机载控制器和控制增益。新的 `sampled_filtered` 模式为默认；原来的 `delayed_truth` 模式保留用于同条件对照。

## 数据路径

```text
MuJoCo 位置/速度 → 独立 50 Hz 六字段日志包 → 20 ms 延迟队列
→ 实飞 _ekf_state_callback → 主机 5 Hz 二阶速度滤波
→ 滤波速度差分加速度 → 原有运动学健康检查和几何控制器
```

包只携带 x/y/z 与 vx/vy/vz，不直接给控制器注入真值加速度。滤波用包的采样时间，健康检查用主机实际交付时刻。首包到达前为 `missing`，首包滤波预热为 `stale_or_unready`，随后才变为 `fresh`；默认运行在控制时间约 0.0429 秒获得有效滤波导数。CSV 反算的回调接收频率约 50.01 Hz。

仿真初始化没有绕过滤波预热，调度按位姿、EKF 及指令各自的时刻推进。实际有效延迟参数、模式、采样频率和滤波设置写入 `input_parameters.yaml: simulation_runtime` 和 `summary.json`。成功条件新增完整接管后观察时长检查，不会因先达到运行时间上限而把不足 60 秒的 hover 误报为成功。

## 验证结果

固定 Mellinger 原始机载增益、70 Hz 主机控制、100 Hz 位姿、50 Hz EKF、20 ms EKF 延迟、5 Hz 主机滤波，以及 `post_handoff_s: 60.0`。

| 输出目录 | 位姿/指令延迟 | 主机 transport_rate_gain | 接管后实际时长 | 结果 |
| --- | --- | --- | ---: | --- |
| filtered_nominal | 10 ms / 5 ms | [2, 2, 1] | 60.00 s | 成功，hover |
| filtered_delay | 20 ms / 10 ms | [2, 2, 1] | 8.56 s | distance 保护 |
| delay_rate1.0 | 20 ms / 10 ms | [1, 1, 0.5] | 8.67 s | distance 保护 |
| delay_rate0.5 | 20 ms / 10 ms | [0.5, 0.5, 0.25] | 8.73 s | distance 保护 |
| delay_rate0.0 | 20 ms / 10 ms | [0, 0, 0] | 8.54 s | distance 保护 |

`filtered_nominal`：约 28.8857 秒接管、88.8857 秒结束，最小机间距 0.348803 m，最低负载间隙 0.328759 m，最大实测角速度分量 0.017073 rad/s，角速度指令饱和率 0%，最终负载高度约 0.599999 m。

较大延迟组没有通过。单独降低角速度反馈并未解决本组机间距收缩问题，因此本次没有据此覆盖实飞控制增益。

用户提到较大延迟测试画面正常，但磁盘中的 `mellinger_delay_test/summary.json` 明确是 `success=false`、`failure=emergency_landing`，仅记录接管后约 8 秒；它没有完成 60 秒验证。判断稳定应同时查看阶段、保护原因、时长和曲线。

## 参数和查看方式

在 `config/slung_payload.yaml` 的 `slung_payload.simulation` 中配置：

```yaml
ekf_feedback_mode: sampled_filtered   # delayed_truth 仅用于旧反馈对照
ekf_sample_rate_hz: 50.0
ekf_delay_s: 0.02
pose_sample_rate_hz: 100.0
pose_delay_s: 0.01
command_delay_s: 0.005
time_limit_s: 180.0
```

从 scripts 目录运行 `python simulate_slung_handoff.py --output ctbr_logs/ekf_feedback_run` 即可加载，不必在命令行重复设置延迟。

默认通过的曲线在 `filtered_nominal/trace.png`，较大延迟失败曲线在 `filtered_delay/trace.png`。

## 范围

本次覆盖真实主机回调、采样、滤波和健康检查；日志包内位置/速度仍是理想真值，未复刻机载 EKF 的 IMU 融合、估计误差、外部 pose 注入，也没有模拟无线电随机丢包/重传、ROS 队列覆盖或逐机不同延迟。包的主机交付在 70 Hz tick 完成。这些限制不能通过“60 秒仿真成功”消除。

新增 8 项测试先失败再通过；连同相关 ROS 控制器回归共 143 项通过。新增和修改的运行脚本通过 Python 3.8 编译，未启动真实飞机。
