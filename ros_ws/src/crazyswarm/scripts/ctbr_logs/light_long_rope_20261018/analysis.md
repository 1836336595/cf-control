# 新负载：23g、三根绳均0.830m的验证

用户已确认23g是包含挂件/定位标记的总质量，三根有效绳长均0.830m。尺寸、负载挂点、动捕坐标定义按当前配置保持不变；若实物有变化，下面结论不直接覆盖。

本轮直接使用已有Udaan/MuJoCo和实机主机控制器，冻结当前所有控制参数，仅改变负载质量及三根绳长。正常外环70Hz、50Hz采样EKF进入主机滤波；主要六组重放最新20:32实飞接管段的491个控制周期，并保留10ms位姿/5ms指令延迟及Graefe2026四电机/Mellinger模型。试验没有连接飞机，也未改实飞YAML。

## 是否可以只改两个参数就实飞

**不能据此直接实飞。两项物理参数修改是必要的，但当前提前距离故障退出存在已复现的漏洞。**
新组合在正常仿真里承载与间距明显改善，故障退出仍需补齐；默认通过也不保证仿真已完全覆盖之前实飞约2Hz增长。

主要物理配置应变为：

```yaml
mass_kg: 0.023
link_lengths_m: [0.830, 0.830, 0.830]
```

在尺寸/挂点/质量分布近似不变的前提下，惯量和负载惯量相关姿态增益自动重算。不要误改ctbr_vehicle里三架飞机质量。
`link_length_m`为列表未提供时的回退值；已有link_lengths_m时不必同步改它。

## 保留的其他参数

```yaml
attitude_bandwidth_hz: [1.0, 1.0, 0.45]
attitude_damping_ratio: 0.90
link_kq: 40.0
link_komega: 20.0
transport_link_gain_scale: 0.4
transport_link_steady_gain_scale: 0.4
takeup_outward_offset_m: 0.12
independent_hover_height_m: 0.50
takeup_duration_s: 8.0
prelift_clearance_m: 0.20
target_payload_height_m: 0.60
```

这些候选均未要求为了长绳同步加大推力、倾角或XY增益。绳向修正含飞机质量×绳长，长绳会改变响应，因此不能仅凭负载减重就按质量比例减小全部增益。
自动绳长校准保留当前设置，但所有正常新组合未实际提交长度修改。校准max_adjustment当前.045m、触发容差.030m已不再等值；这里没有放宽保护阈值。

## 结果

| 场景 | 接管后时长 | 最小三机间距 | 最大机体单轴角速度 | 结果 |
|---|---:|---:|---:|---|
|默认参数，完整观察|60.000s|.5180m|.00966rad/s|通过|
|旋转阻尼降低10倍至.0001|60.000s|.5179m|.00973rad/s|通过|
|位姿/指令延迟增加为20/10ms|60.000s|.5181m|.00938rad/s|通过|
|观测姿态噪声std[.005,.004,.004]rad|60.000s|.5193m|.26383rad/s|通过|
|10秒验证并完整降落|26.214s含下降|.5264m|.00966rad/s|负载接地、三机零推力|
|零旋转阻尼敏感性边界|12.129s|.2999m|2.941rad/s|距离保护失败|

零阻尼不是实物标定值，姿态噪声也是显式假设，不代表NOKOV实测误差分布。默认模型仍有约4.8°pitch静差；通过位置/距离判据不等同于负载完全水平。

正常60秒三机命令峰值约.533/.500/.482N，没有模拟单电机达到上限；实际执行器依赖电池与标定，此数值不能当作硬件测得推力。

## 静态目标可行性：之前忽略的倾角上限

本轮还直接检查目标外张构型（详见feasibility.json）：

- 新23g/.83m组合需要的静态飞机倾角约6.39/4.94/5.10°；理想实际推力约.539/.491/.475N。
- 旧54g/.699/.699/.674m同样外张目标需要倾角约12.51/10.68/11.02°，超过当前10°运输限制，CF3略超12°预抬限制。

因此“当前推力没到上限”确实不能证明旧外张目标可实现：倾角限制可能先限制XY力。
新组合更容易实现外张，不只是总推力变小。水平间距增加但绳也变长，TAKEUP绳角反而略降，不能只按到负载中心的水平距离判断张力。

当前TAKEUP最小间距约.752m。最终负载中心目标.60m时，飞机预期高度约1.29–1.41m（依具体绳向/挂点/静态构型变化）；独立悬停仍可从.50m开始，TAKEUP目标会按新绳长自动生成。仿真验证在绳索初始布置附近开始，**没有完整重演真实地面到独立起飞的所有状态、机载EKF或无线电失效**。

## 提前距离故障的实测代码路径回归

`run_injected_distance_exit.py`使用实际MuJoCo，在接管后6.014s（实测间距仍.547m）只注入一次distance故障原因。物理状态没有人为拉近，因此这是分支覆盖回归，不是碰撞恢复效果测试。

结果：三机进入旧emergency_landing，payload_transport_active变False，rope-preserving未初始化。它重现了20:32实飞的漏洞：distance故障早于10秒计划下降时，仍绕过新的负载绳向下降流程。

另外，计划下降中设置payload_payload_aware_emergency后，当前代码会跳过后续物理安全检查；退出时仍应持续监测和约束间距，不能靠静默绕过保护。

所以应先完善所有有效吊运状态下的提前distance退出、间距恢复/限速、触地卸载和三机零推力验证。仅修改mass和绳长不会修复这些路径。此前正常10秒计划下降的通过不能覆盖这个问题。

## 可直接复查的文件和命令

当前实飞文件仍为54g和旧绳长；新组合保存在离线候选 `simulation_configs/slung_light_23g_830mm.yaml`。这个文件关闭了10秒实飞验证计时器，以便查看完整60秒，**不要将它直接作为实飞参数加载**。

在scripts目录、Udaan环境运行：

```bash
python simulate_slung_handoff.py --candidate simulation_configs/slung_light_23g_830mm.yaml --output ctbr_logs/light_long_rope_check
python plot_slung_handoff_trace.py ctbr_logs/light_long_rope_check/simulation_trace.csv
```

完整冻结输入与参数快照见各场景目录candidate.yaml/input_parameters.yaml；所有结果见comparison.json。
查看已经生成的正常60秒图用mass23_rope830_nominal60/trace.png，完整降落图用mass23_rope830_validation10/trace.png。
