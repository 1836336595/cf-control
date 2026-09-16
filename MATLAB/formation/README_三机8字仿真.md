# 三架飞机 8 字编队仿真

本目录新增的 MATLAB 文件基于 `simulator/CTBR/codex/v2` 的 CTBR 外环和刚体积分方式，使用 `编队.pdf` 与 `几何控制单机.pdf` 中的符号约定。控制器内部采用 `e3=[0;0;1]` 指向重力、z 轴向下；绘图时把 z 取反显示为高度。

## 运行

在 MATLAB R2021b 中将当前目录切换到本目录，然后运行：

```matlab
simDisplacement = run_three_quadrotor_displacement;
simLeaderFollower = run_three_quadrotor_leader_follower;
```

两个入口都会打开轨迹/误差/推力曲线和三维动画。若只需要数值结果：

```matlab
cfg = formation_three_quadrotor_parameters;
cfg.visualization.plot = false;
cfg.visualization.animate = false;
cfg.simulation.duration = 5;

cfg.formation.type = 'displacement';
sim = formation_three_quadrotor_simulation(cfg);
```

切换期望角速度的计算方式：

```matlab
cfg.attitudeController.omegaCMethod = 'analytic';       % 解析 Rc' * Rc_dot
cfg.attitudeController.omegaCMethod = 'log_difference';  % SO(3) 离散李群差分
```

## 文件说明

* `编队推导说明.md`：两种编队的 `A_i`、`R_c,i` 和 `Omega_c,i` 推导。
* `formation_three_quadrotor_parameters.m`：物理参数、控制增益、通信图和编队偏移。
* `formation_figure_eight_reference.m`：水平 8 字轨迹、速度、加速度、jerk、航向和航向导数；六输出顺序为 V2 的 `[position, velocity, acceleration, rotation, jerk, rotationDot]`。
* `formation_three_quadrotor_controller.m`：两种编队控制律、`R_c`、解析/离散 `Omega_c` 和 CTBR 输出。
* `formation_three_quadrotor_simulation.m`：推力执行器、角速度内环和六自由度动力学。
* `formation_three_quadrotor_visualization.m`：轨迹曲线、误差曲线、推力曲线和三机动画。
* `run_three_quadrotor_displacement.m`：1 号机几何领航、2/3 号机位移一致性跟随的混合入口。
* `run_three_quadrotor_leader_follower.m`：虚拟中心领导者、三机全分布式 pinning 入口。

位移型入口中 1 号机使用几何控制单机的位置误差、速度误差和积分项跟踪 8 字
参考，2/3 号机使用 PDF 中的相对位移/速度一致性控制；`commonAcceleration`
提供共同加速度前馈。若要复现原先三机都只使用纯相对控制的版本，将
`cfg.formation.type='displacement_pure'`。

领导-跟随入口把 `p_c` 定义为三架飞机几何中心的虚拟领导者，并采用
`bl=[1;1;1]`，三架飞机都直接使用中心误差和通信图相对误差。该模式没有物理
领航飞机，也没有一架单独使用单机位置控制器；但每架飞机仍必须经过
`u_i -> A_i -> R_c,i -> Omega_c,i` 的几何姿态/推力映射，这是四旋翼实现期望
平动加速度所必需的。

自定义轨迹函数也建议采用 V2 的六输出顺序 `[p, v, a, R, jerk, Rdot]`；仿真解析器同时兼容本目录早期的 `[p, v, a, jerk, R, Rdot]` 顺序。
