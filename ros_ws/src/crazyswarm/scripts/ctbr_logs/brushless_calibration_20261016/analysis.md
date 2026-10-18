# Gräfe 2026 标定接入与闭环对照

实现说明：`simulate_uav/udaan/docs/graefe2026_calibration.md`。
来源：arXiv:2603.05944，公式 (5)、(7)、(9)、(10)、表 II，以及作者
CrazyflieBrushJAX 的高精度多项式系数。

## 已接入

- 四个独立电机转速状态，50 ms 一阶转速动态、2900 rad/s 增益，精确指数积分。
- 转速到推力的三次曲线；转速到气动反扭矩的三次曲线。
- 转子惯量 5e-8 kg m² 及加减速的反作用力矩，按各电机旋向合成偏航力矩。
- 带保护圈机体惯量 `[3.3e-5, 3.6e-5, 5.9e-5]` kg m² 和电机方形宽度 0.0707 m。
- 现有 PID/Mellinger 混控后的各电机指令进入同一个物理模型；旧平方曲线可通过
  `motor_model: legacy_quadratic` 对照。原生 Udaan 和实机控制器回放均支持新模型。
- 各机实测质量 `[0.0434, 0.0440, 0.0424]` kg、负载 0.054 kg，保持真实质量，
  不用论文的统一 44 g 替换。原生场景中 CF4/CF5 的旧质量已与现有实飞配置对齐。
- 仿真 CSV 增加实际推力、四电机 PWM/转速/推力/反扭矩及限幅计数；绘图显示命令和实际推力。
  参数快照记录完整生效物理配置。

主机旧的牛顿到 raw 映射仍被作为仿真输入链的一部分：1.176798 N 映射参考为 60000，
主机集体命令上限 60000，混控后单电机满量程 65535。论文物理曲线在四电机均为 60000 时
预测整机稳态推力 1.018558 N，均为 65535 时预测 1.153401 N。这是论文模型预测，
不代表当前实物已完成推力标定。

## 相同控制增益的回放结果

所有回放均使用实际 `ctbr_controller.py` 的任务和控制逻辑，70 Hz 外环、50 Hz 理想 EKF
包进入实机主机滤波器、位姿延迟 10 ms、EKF 延迟 20 ms、命令延迟 5 ms。
目标观察时间统一为 **接管后 60.0 s**，未放宽保护。

| 目录 | 物理模型/机载控制器 | 实际接管后时间 | 结果 |
| --- | --- | ---: | --- |
| `mellinger_legacy` | 旧平方推力、12 ms 推力滞后、旧惯量/机臂，Mellinger | 60.000 s | 通过；最小间距 0.348804 m，最大角速度 0.017073 rad/s，负载最终约 0.600 m |
| `mellinger_paper` | 论文标定，Mellinger | 6.386 s | 保护降落：`distance`；最小间距 0.296085 m，最大角速度 4.645484 rad/s |
| `pid_paper` | 论文标定，PID | 8.386 s | 保护降落：`rate_saturation`；最大角速度 4.476442 rad/s，速率饱和比例 0.28109 |

Mellinger 新模型在 29.442857 s 接管，35.828571 s 中止；PID 在 29.328571 s 接管，
37.714286 s 中止。新模型未完成 60 s 观察，**不能据此判断当前增益满足实飞稳定条件**。
进程退出码 1 对应记录到的保护失败，不是 Python 异常。

该对照同时改变了论文指定的电机动态、曲线、反扭矩、惯量及电机位置，尚未隔离各项
对失稳的独立贡献。它说明旧物理模型的稳定结果不能直接迁移，不能宣称论文模型已经
精确复现最新实飞的全部现象。后续应在这个模型基础上分析接管闭环并调参。

复现三组回放，在 `scripts` 目录运行：

```bash
python ctbr_logs/brushless_calibration_20261016/run_validation.py mellinger_paper
python ctbr_logs/brushless_calibration_20261016/run_validation.py pid_paper
python ctbr_logs/brushless_calibration_20261016/run_validation.py mellinger_legacy
```

可视化本次主要回放：

```bash
python plot_slung_handoff_trace.py ctbr_logs/brushless_calibration_20261016/mellinger_paper/simulation_trace.csv
```

## 验证与限制

ROS 控制器及参数回归 152 项通过；Udaan 电机物理、混控、时钟、初始化、CSV 及旧模型
回归 49 项检查通过。原生 CLI 无窗口运行 3 s 并输出新模型电机日志；这只是入口检查，
不是原生任务完整稳定性验证。单独的标定曲线和阶跃图在
`simulate_uav/udaan/logs/graefe2026_calibration/calibration.png`。

论文未标定完整 `F(PWM,V)` 或电池内阻，本实现没有凭空加入电压压降模型。
`motor_available_command_fraction` 是作者式的可用指令上限假设，默认 1，不能当作伏特值。
当前实际 ESC/PWM 归一化、带载电压影响、标记引起的惯量变化仍需实物核对。
完整机载 EKF、测量噪声和多机气流相互作用仍不在当前验证范围内。
