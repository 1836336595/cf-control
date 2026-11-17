# 调平后的有效绳长统计

用于换绳后核对 `slung_payload.link_lengths_m`。端点定义与当前控制模型一致：
**负载挂点到飞机 NOKOV 刚体原点**。如果以后移动刚体原点或改变挂点，必须同时核对该定义。

## 测量公式

$$d_i=\left\|x_i-\left(x_0+R_0\rho_i\right)\right\|.$$

- $d_i$：第 i 根绳子的有效端点距离，单位 m；使用实际位置计算，不读取配置绳长作为测量值。
- $x_i$：飞机动捕刚体原点的世界位置，单位 m；CF3、CF4、CF5 按参数挂点顺序对应。
- $x_0$：负载几何中心的世界位置，单位 m；已经由上表面动捕原点转换，不能再减一次负载高度。
- $R_0$：负载实测姿态矩阵；即使负载略微倾斜，也按实际姿态旋转挂点。
- $\rho_i$：负载几何中心到挂点的机体系向量；当前三项为 `[.04,0,.025]`、`[-.04,.03,.025]`、`[-.04,-.03,.025]` m。

这测得的是当前模型的有效距离。没有张力传感器时，无法单凭距离证明绳子完全绷紧，
也不能从中区分绳结、弹性伸长、动捕原点与飞机挂载点偏移。稳定带载状态比地面松绳阶段更适合核对。

## 自动输出条件

在 `slung_payload.yaml` 设置：

```yaml
link_length_report_enabled: true
link_length_report_window_s: 3.0
link_length_report_level_tolerance_deg: 3.0
```

以下条件同时满足才累计报告：

- 已完成接管及参考抬升，目标位置和目标姿态静止，负载目标倾角不超过配置门槛；扫描和下降不作调平测量。
- 实测负载离地至少 0.10 m，当前请求张力均为正；请求张力只是筛选依据，并非实测张力。
- 负载反馈通过既有质量检查，源年龄不超过 0.10 s，三机与负载源时间差不超过 0.03 s。
- 至少 30 个独立负载源时间戳覆盖完整窗口；重复帧不计数，源间隔超过 0.10 s 重新累计。
- 窗口倾角中位数不超过 3°，每个采样的瞬时倾角不超过 5°（配置门槛再加 2°）。
- 负载及各飞机在窗口中的 XYZ 包围盒对角线不超过 0.03 m，每根距离 P90−P10 不超过 0.005 m。

首次满足条件时输出一次，持续稳定时最多每 10 s 输出一次；条件中断后重新累计完整窗口。
真实拒绝或观测器重置会清空正在统计的窗口。正常短采样等待不能增加独立样本数。
未满足条件时可从 CSV 的 `link_length_report_status` 查看 `collecting`、`not_level`、
`pose_motion`、`length_variation` 或 `ineligible` 等状态，不把未收敛状态称为已完成调平。

终端形式如下（数字仅为现有 CSV 的示例，不是程序写死值）：

```text
[调平后绳长统计] 窗口 3.01 s，... 个独立样本，倾角中位数 ...°；
CF3 0.8589 m（P10–P90 ...，配置 0.8570，有效 0.8570）；
CF4 0.8610 m（P10–P90 ...，配置 0.8600，有效 0.8600）；
CF5 0.8554 m（P10–P90 ...，配置 0.8540，有效 0.8540）；...
```

统计本身不修改 YAML、不调整运行时绳长、不改变力控制和接管门槛。
接管前已有的 `link_calibration_*` 一次性校准继续独立工作。

## CSV 字段

运输阶段的 `link_length_measured_m` 现在按核心控制所用的位姿快照重新计算，
不会一直保留 TAKEUP 最后一次测距。缓存运输输出时保持原测量时间。

| 字段 | 含义 |
|---|---|
| `link_length_measurement_sample_time_s` | 当前几何测距使用的负载源时间戳 |
| `link_length_report_sequence` | 已完成的稳定统计编号；0 表示尚未完成 |
| `link_length_report_sample_time_s` | 最近一次统计结束时的源时间戳 |
| `link_length_report_median_m` | 当前飞机对应绳长的窗口中位数 |
| `link_length_report_p10_m` / `link_length_report_p90_m` | 窗口 10% / 90% 分位数 |
| `link_length_report_samples` / `link_length_report_window_s` | 独立样本数量及覆盖时间 |
| `link_length_report_status` | 当前窗口状态；历史统计仍可保留，需结合编号与时间判断 |

各机一行只写本机对应的距离，终端一次打印全部三项。

## 用已有 CSV 重新计算

在 `scripts` 目录、Udaan 环境执行：

```bash
python analyze_slung_link_lengths.py \
  ctbr_logs/multi_ctbr_20261025_152703.csv \
  ctbr_logs/multi_ctbr_20261023_222628.csv \
  --output ctbr_logs/link_length_report_20261116/lengths.json
```

脚本读取飞机位置、负载位置和姿态重新计算，不使用历史的 `link_length_measured_m` 作为结果。
旧 CSV 如果没有源时间戳，不能通过这个工具的同步样本准入。挂点及尺寸默认从当前 YAML 读取，
分析不同负载的历史数据时需通过 `--config` 指定当时配置。

`stable_reports` 使用与在线相同的窗口统计；`last_10s_stationary_reference_summary`
只是末段目标静止时的描述性统计，未必满足完整连续窗口条件，二者不混为一谈。
