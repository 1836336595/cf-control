# Crazyflie 2.1 Brushless CTBR 仿真

## 运行环境

- MATLAB R2021b 或更高版本
- 不依赖 Simulink、Aerospace Toolbox 或 Robotics System Toolbox
- 仿真主文件：`crazyflie_ctbr_simulation.m`
- 参数文件：`crazyflie_parameters.m`
- 控制器文件：`crazyflie_ctbr_controller.m`
- 可视化文件：`crazyflie_visualization.m`

## 运行

在 MATLAB 当前目录切换到本目录后执行：

```matlab
sim = crazyflie_ctbr_simulation();
```

默认会打开两个窗口：

1. 位置轨迹、位置误差、姿态角和 CTBR 指令曲线；
2. Crazyflie 的简化三维动画。

无图形环境或只想测试数值时：

```matlab
cfg = struct();
cfg.visualization.plot = false;
cfg.visualization.animate = false;
sim = crazyflie_ctbr_simulation(cfg);
```

## Omega_c 计算方式切换

参数 `cfg.attitudeController.omegaCMethod` 支持两种方式：

```matlab
% 默认：相邻 Rc 的 SO(3) 李群对数差分，适合离散实时数据
cfg.attitudeController.omegaCMethod = 'log_difference';

% 解析方式：由 Rc=[b1c,b2c,b3c] 的列向量导数计算 Rc' * Rc_dot
cfg.attitudeController.omegaCMethod = 'analytic';
sim = crazyflie_ctbr_simulation(cfg);
```

例如要在相同条件下分别运行两种算法，可关闭绘图后比较日志：

```matlab
cfg.visualization.plot = false;
cfg.visualization.animate = false;

cfg.attitudeController.omegaCMethod = 'log_difference';
simLog = crazyflie_ctbr_simulation(cfg);

cfg.attitudeController.omegaCMethod = 'analytic';
simAnalytic = crazyflie_ctbr_simulation(cfg);
```

解析方式使用
\[
\dot A=-k_x\dot e_x-k_v\dot e_v-k_i\frac{d}{dt}\operatorname{sat}(e_i)+m\dddot{x}_d,
\]
因此轨迹函数最好提供期望 jerk。轨迹函数仍兼容原来的四个输出，也可以扩展为：

```matlab
function [position, velocity, acceleration, rotation, jerk, rotationDot] = myReference(t)
    % position, velocity, acceleration, jerk: 3x1
    % rotation, rotationDot: 3x3
end
```

如果只提供四个输出，程序将把 `jerk` 和 `rotationDot` 设为零。仿真文件会把当前动力学加速度传给控制器，以计算 `A_dot`。绘图窗口标题会显示当前使用的 `Omega_c` 方法。

如果需要保存动画视频：

```matlab
cfg.visualization.saveVideo = true;
cfg.visualization.videoFile = 'crazyflie_ctbr_demo.mp4';
sim = crazyflie_ctbr_simulation(cfg);
```

## 文件职责

```text
crazyflie_parameters.m
    所有物理参数、控制增益、初始状态、目标状态和可视化参数

crazyflie_ctbr_controller.m
    几何位置外环、计算姿态 Rc、姿态误差、CTBR 角速度和推力百分比

crazyflie_ctbr_simulation.m
    推力执行器、内部角速度控制器、刚体动力学和数据记录

crazyflie_visualization.m
    轨迹图、误差图、姿态图、CTBR 曲线和三维动画
```

## 仿真结构

```text
理想动捕状态 x, v, R, Omega
              |
              v
几何位置外环: A, Rc, f
              |
几何姿态外环: Omega_cmd
              |
推力百分比 + body rates (CTBR)
              |
模拟 Crazyflie 内部速率控制器
              |
刚体动力学: x, v, R, Omega
              |
              +---- 下一周期状态反馈
```

仿真中使用真实状态作为“理想动捕输出”，因此没有加入动捕噪声和延迟。后续接入真实
动捕时，应将 `position`、`velocity`、`R` 和 `bodyRate` 的来源替换为动捕消息，并保持
坐标系约定一致。

## 坐标系

代码沿用论文约定：

- 惯性系 `e3 = [0; 0; 1]` 指向重力方向；
- 位置的 `z` 轴正方向向下；
- 机体系到惯性系的姿态矩阵为 `R`；
- 推力为 `-f*R*e3`。

为了便于阅读，绘图将纵轴显示为 `altitude = -z`，也就是向上为正。若要改用常见的
世界系 z 轴向上，需要同时修改动力学方程、`A` 的重力项、`b3c` 和绘图坐标，不能只
修改标签。

## J 在仿真中的作用

当前实时 CTBR 外环只输出：

```text
thrust_percentage
roll_rate_deg_s
pitch_rate_deg_s
yaw_rate_deg_s
```

因此实时外环不需要惯性矩阵 `J`。但本仿真还要推进六自由度刚体动力学，并模拟
Crazyflie 的角速度内环，所以配置中的

```matlab
cfg.vehicle.inertia
```

仍然是必需的。它用于：

```text
J * Omega_dot + Omega x (J * Omega) = M_inner
```

这并不表示 Python CTBR 外环向真实 Crazyflie 发送了力矩；仿真中的 `M_inner` 只是为了
让角速度指令经过一个可观察的内部速率环后再作用于刚体。

## 主要可调参数

```matlab
cfg.simulation.duration
cfg.simulation.dt

cfg.vehicle.mass
cfg.vehicle.inertia
cfg.vehicle.maxTotalThrust
cfg.vehicle.thrustTimeConstant

cfg.positionController.kx
cfg.positionController.kv
cfg.positionController.ki

cfg.attitudeController.kr
cfg.attitudeController.maxBodyRateCommand

cfg.rateController.kp
cfg.rateController.ki
cfg.rateController.maxMoment
```

`maxTotalThrust` 和推力百分比映射目前是理想线性模型，只用于闭环仿真。真实飞行前应
用推力台或悬停实验建立 Crazyflie 的 `f -> thrust_percentage` 标定曲线。

## 替换为轨迹参考

可以提供一个函数句柄，返回目标位置、速度、加速度和姿态：

```matlab
cfg.referenceFcn = @myReference;

function [x, v, a, R] = myReference(t)
    x = [0.2*cos(0.4*t); 0.2*sin(0.4*t); -0.45];
    v = [-0.08*sin(0.4*t); 0.08*cos(0.4*t); 0];
    a = [-0.032*cos(0.4*t); -0.032*sin(0.4*t); 0];
    R = eye(3);
end
```

函数必须返回论文坐标系下的 `R`，并且 `R` 是机体系到惯性系的旋转矩阵。

## 与真实 CTBR 接口的对应

仿真输出可以这样对应到 `cflib`：

```matlab
ratesDeg = sim.bodyRateCommandDeg(:, k);
thrustPct = sim.thrustPercentage(k);
```

然后在 Python/`cflib` 中持续发送：

```python
cf.commander.send_setpoint_manual(
    roll=float(rates_deg[0]),
    pitch=float(rates_deg[1]),
    yawrate=float(rates_deg[2]),
    thrust_percentage=float(thrust_pct),
    rate=True,
)
```

仿真默认没有实现无线通信，也没有直接给出四个电机 PWM。CTBR 的内部速率控制和电机
混控由真实 Crazyflie 固件负责。

## Lee 直接推力-力矩模式

需要直接使用论文中的总推力 `f` 和机体系力矩 `M`，并观察四路电机命令时，请进入
`force_torque` 子目录，阅读其中的 [README.md](force_torque/README.md)，然后运行：

```matlab
sim = crazyflie_force_torque_simulation();
```

该目录与本 CTBR 仿真完全分开，包含固件一致的 M1--M4 混控、16 位电机命令和电机推力动态。
