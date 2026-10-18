# 首次实飞：接管 10 s 后自动降落

## 默认行为

`config/slung_payload.yaml` 已开启：

```yaml
flight_validation_enabled: true
flight_validation_transport_s: 10.0
post_handoff_s: 60.0
```

`10.0` 从实际进入 TAUT_RAMP 计时，包括 5 s 张力渐增和后续参考抬升，
不等待完整 12 s 抬升结束。预抬过程不消耗这个窗口。
`post_handoff_s` 仍是离线长时观察设置，不改变实飞倒计时。

到时输出 `flight_validation_complete`，三机进入正常 `landing` 路径。
首先保留协同控制放下负载；负载底面间隙 ≤0.015 m、|竖直速度|≤0.05 m/s
连续 0.30 s 后，释放协同控制，从各机当前实测 XY/yaw 重新生成垂直降落。
后段继续使用已有预抬独立 PID 的 XY 倍率/姿态增益/倾角限制，直到落地。
正常的高度及速度门槛、`landing_settle` 和 `landed` 负责停推。

一次性结束状态不随负载接管缓存清空而复位。上一周期抬升输出不会盖住下降参考。
原状态失效、掉高和间距保护先处理；计划结束不覆盖故障原因，不设置全机零推力中止。
三机下降过程中继续检查间距，即使已有一架先达到 `landed`。

CSV 新增 enabled、elapsed、landing_started、landing_stage 四项诊断；
`flight_validation_landing_stage=payload_lowering/vehicle_descent` 区分两段下降。
计划结束原因写入现有 `emergency_landing_reason` 列，但实际阶段为正常 landing。

## 物理仿真验证

使用现有 `simulate_slung_handoff.py`、真实 ROS 任务/控制器逻辑和 Graefe2026
四电机模型。回放从已建立的 TAKEUP 构型开始；真实起飞前预检及地面起飞不在
此次物理回放范围内。回放专门将降落地面参考设为 MuJoCo 机体中心高度 0.007 m，
避免把 TAKEUP 构型的 0.7 m 初始锚点误当成地面；参数快照记录了此假设。

| 回放 | 接管时间 | 开始计划降落 | 完成停推/稳定 | 全程最小机间距 |
| --- | ---: | ---: | ---: | ---: |
| `final_nominal`：位姿10 ms/指令5 ms | 29.443 s | 39.443 s | 48.271 s | 0.411880 m |
| `final_delay`：位姿20 ms/指令10 ms | 29.443 s | 39.443 s | 48.700 s | 0.411805 m |

两组均严格完成接管 10.000 s 后开始下降，依次经过
`TAKEUP → TAUT_RAMP → height_correction → landing → landing_settle → landed`。
默认组约 42.200 s 释放负载，较大延迟组约 42.657 s 释放负载。
最终负载中心高度约 0.025 m，三机中心高度均约 0.006998 m，三机推力命令全部为零，
竖直速度接近零；连续满足落地、低速、零命令 0.5 s 后才给出
`success_category=short_flight_validation`。没有仅凭计时器触发就判成功。

在下降前的 10 s，最大单轴角速度约 0.020 rad/s，角速度指令饱和为零。
全程间距统计包括下降和落地，没有删除危险下降样本来获得通过结果。

开发时保留的 `nominal`/`delay` 记录展示了直接切旧单机降落时的间距收缩；
`cooperative_nominal`/`cooperative_delay` 展示了释放绳张力时普通弱姿态环的不足。
这几组明确失败，不是最终实现。应查看 `final_nominal` 和 `final_delay`。

完整 60 s 回归使用 `simulation_configs/slung_70hz_candidate.yaml`，其中显式关闭
实飞倒计时；结果类别为 `post_handoff_hover`，不能与短时落地验证混淆。

## 检查和运行

182 项 ROS 控制器/轨迹/回放及参数检查通过。
实际 ROS Noetic 的 `/usr/bin/python3`（Python 3.8）编译通过，并用离线 ROS 桩
实例化管理器，验证默认开关、10 s 触发及正常 landing 路径。
本次未启动 ROS master，未发任何实机指令。

Python/YAML 修改无需重编 C++。重启控制器 launch 后生效：

```bash
source /home/csy/my_project/crazyswarm/ros_ws/devel/setup.bash
roslaunch crazyswarm ctbr_controller.launch target_confirmed:=true
```

启动时应看到 `[实飞验证] 已启用`。完成 TAKEUP 并接管 10 s 后，应看到
`flight_validation_complete`；随后负载落地时应看到释放协同拉力的日志。
已有 server/NOKOV 启动流程不变；手动停止及故障保护仍有效。

查看离线轨迹：

```bash
python plot_slung_handoff_trace.py ctbr_logs/flight_validation_20261016/final_nominal/simulation_trace.csv
```

这是已通过上述仿真条件的短时实机验证入口，尚未等同于实机闭环稳定性验证。
