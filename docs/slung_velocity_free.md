# 实飞负载 Velocity-free β 反馈

本版新增 `state_feedback_mode: velocity_free_beta`，以负载位姿驱动一个 SE(3) 辅助系统，
用其输出 β 替换原负载观测器输出的速度和角速度。原方式通过 `pose_observer` 保留。
配置入口为 [slung_payload.yaml](../ros_ws/src/crazyswarm/config/slung_payload.yaml)，
实现为 [slung_velocity_free.py](../ros_ws/src/crazyswarm/scripts/slung_velocity_free.py)。

这里的 velocity-free 指**负载反馈不读取测得的速度、角速度或加速度**。
飞机位置控制仍使用每架飞机自己的 EKF 速度；飞机仍接收 CTBR 总推力和体坐标角速度指令。
本版不是 MATLAB 完整负载 SE(3) 合力/力矩控制律的逐项移植，也不继承其稳定性结论。

## 1. 对照资料

- 用户提供的 `负载的全驱动 SE(3)、非混杂、velocity-free 控制器.md`：重点是 §3、§5、§8。
- [GitHub vel-free 分支](https://github.com/1836336595/Multi-UAV-Transportation-Simulation/tree/vel-free)。
  在线访问未成功，实际逐行对照的是用户下载到工作区的 `Multi-UAV-Transportation-Simulation-vel-free`。
- 已核对 `crazyflie_slung_se3.m`、`crazyflie_slung_controller.m` 和 `crazyflie_slung_parameters.m`。
  下载文件没有可验证的 Git commit，下面用 SHA256 固定本次对照来源，避免把分支后续变化混入结论。

| 文件 | SHA256 |
|---|---|
| `crazyflie_slung_se3.m` | `684f53ca64ce3fba198fadfb6072ceccfa5a0f9921f9aa0c0399dcfc5903ff83` |
| `crazyflie_slung_controller.m` | `c1f4568485b572c66c5a644d03bb24d5866412600bbcc2fa434d85138402af73` |
| `crazyflie_slung_parameters.m` | `a051a0ca5bb3eda783d58476acf9f88f478e8d17822d9694d0c27a94140d886c` |

## 2. 辅助系统的公式

$$X_0=\begin{bmatrix}R_0&p_0\\0&1\end{bmatrix},\quad X_r=\begin{bmatrix}R_r&p_r\\0&1\end{bmatrix},\quad X_e=X_r^{-1}X_0,\quad\widetilde X=\widehat X^{-1}X_0.$$

- (p_0)：NOKOV 上表面原点换算后的负载几何中心位置，世界坐标，m。
- (R_0)：负载体坐标到 ROS 世界坐标的旋转矩阵。
- (p_r,R_r)：现有悬停、圆周、八字或圆锥轨迹给出的负载参考。
- (widehat X)：新增辅助位姿；首个有效样本取 (widehat X=X_0)，不人为加入偏移。
- (widetilde R,widetilde p)：(widetilde X) 的旋转和平移部分。

$$\widetilde\Psi_\omega=\tfrac12\operatorname{vee}(W\widetilde R-\widetilde R^T W),\qquad\widetilde\Psi_p=\tfrac d2\widetilde R^T\widetilde p.$$

- (W)：`diag(velocity_free_attitude_weights)`，当前为 `diag(1.0, 1.1, 1.2)`。
- (overline W)：(	frac12(\operatorname{tr}(W)I-W))，当前为 `diag(1.15, 1.10, 1.05)`。
- (d)：仅在观测器内部取 2；与下一式中的 (2/d) 抵消，未增加没有实际作用的配置项。
- (operatorname{vee})：把三维反对称矩阵变回向量。

$$\beta=\operatorname{Ad}_{X_e^{-1}}\begin{bmatrix}\omega_r\\R_r^Tv_r\end{bmatrix}+\begin{bmatrix}\kappa_\omega\overline W^{-1}&0\\0&(2\kappa_p/d)I\end{bmatrix}\widetilde\Psi.$$

- (eta=[\beta_\omega;\beta_v])：六维速度替身，顺序为**体坐标角速度、体坐标线速度**。
- (kappa_p)：`velocity_free_kappa_position_per_s: 12.0`，单位 s⁻¹。
- (kappa_omega)：`velocity_free_kappa_attitude_per_s: 20.0`，单位 s⁻¹。
- (v_r)：世界坐标参考速度，m/s；由轨迹提供，不是测量差分。
- (omega_r)：参考姿态坐标系中的参考角速度，rad/s。

$$\operatorname{Ad}_{X_e^{-1}}\xi_r=\begin{bmatrix}R_0^TR_r\omega_r\\R_0^T\left[v_r+(R_r\omega_r)\times(p_0-p_r)\right]\end{bmatrix}.$$

- 第二块的叉乘项保留了参考旋转与位置误差之间的耦合；没有把 SE(3) 简化成两个互不相关的一阶滤波器。
- 参考由控制线程写入，下一次位姿回调使用；按源时间差以参考速度、加速度短时外推。
- 参考超过 `observer_max_dt: 0.05` s 不再前馈，转为零参考群速度下的位姿辅助系统。
- 这个 ROS 异步离散实现与 MATLAB 同步同拍参考不同，存在一个参考更新周期内的差异。

$$\dot{\widehat X}=\widehat X\left(\operatorname{Ad}_{\widetilde X}\beta\right)^\wedge,\qquad\widehat X_{k+1}=\widehat X_k\exp\left[\Delta t\left(\operatorname{Ad}_{\widetilde X_k}\beta_k\right)^\wedge\right].$$

- (Delta t)：已接受 NOKOV 源时间戳的真实间隔；不是固定 70 Hz 控制周期。
- 回调到来时先用上次已接受的辅助群速度传播到当前时间，再计算当前 β，因果顺序与上述离散式一致。
- 平移使用 SE(3) 指数映射的左 Jacobian，保留旋转和平移耦合；不是 `p += dt*v` 的独立更新。
- 每次更新将辅助旋转投影回 SO(3)。重复、倒序、坏帧和小于 5 ms 的等待不会推进辅助状态。

$$v_{0,\mathrm{feedback}}=R_0\beta_v,\qquad\Omega_{0,\mathrm{feedback}}=\beta_\omega.$$

- `payload_velocity_*`：上式世界坐标速度，供现有调用方使用。
- `payload_body_rate_*`：上式体坐标角速度。
- β 是控制辅助量，包含滞后和参考瞬态；不能把它当成独立测得的真实速度。
- 不对 β 做差分获取加速度。`payload_acceleration_*` 在此模式为 0，并记录 `payload_acceleration_available=0`。
  负载核心不用该实测加速度；降落规划初始加速度采用零近似，仍保留速度与轨迹限幅。

## 3. 哪些位置使用 β

| 使用位置 | 新模式行为 |
|---|---|
| 负载位置反馈与位置积分 | `v0-vd` 使用世界坐标 `R0 @ beta_v` |
| 负载姿态阻尼 | `Omega0-R0.T@Rd@Omegad` 使用 `beta_omega` |
| 运动参考角速度的坐标导数 | 现有 `-J*(Omega0 × R0.T@Rd@Omegad)` 中的 `Omega0` 替换为 β |
| 实测陀螺补偿 | 新模式不补 `Omega0 × J*Omega0`；按笔记 §8.4 和 MATLAB 的处理，不拿滞后 β 抵消这项 |
| 负载角加速度指令 | 新模式为 `J^-1 Md`；旧模式仍有原陀螺项处理 |
| 绳索方向变化、离心力 | 挂点速度用 `R0@beta_v + (R0@beta_omega) × rho_world`，再减飞机独立 EKF 速度 |
| `reference_cascade` 挂点加速度 | 仍使用参考加速度、参考角速度和参考角加速度 |
| `coupled` 挂点加速度 | 负载角速度项使用 β；该分支仍保留，但本次整机验证使用 `reference_cascade` |
| 预抬稳定确认、绳长校准、触地确认、trim 门控 | 读取同一份 β 状态，不暗中混入旧观测导数 |
| 位姿、测距、离地高度、目标构型 | 仍使用已接受的原始位姿，不用辅助位姿替换真实几何 |
| 飞机位置 PID、姿态 PI、Mellinger | 保持原实现；不是整套飞机也 velocity-free |

## 4. 与 MATLAB / 笔记的差异

| 项目 | MATLAB / 笔记 | 本版实飞实现 |
|---|---|---|
| β 与辅助元 | SE(3)、Adjoint、加权梯度、指数映射 | 保留同一数学结构；有单元测试按矩阵公式逐项比对 |
| 负载控制律 | 六维 `Upsilon-kX*PsiE-kXtilde*PsiT` | 保留现有位置 PID、姿态 PD 与参考前馈，以 β 替换反馈；**没有改成该六维合力/力矩公式** |
| 刚度和势函数权重 | W、d 由负载刚度反解 | W 只用于辅助观测，采用正定互异的简单权重；现有刚度独立保留 |
| 观测增益 | 旋转逐轴，通常取十倍自然频率并按仿真 dt 截断 | 平移 12、旋转统一 20 s⁻¹，参数显式固定；不随控制周期暗中改增益 |
| 采样 | MATLAB 同步物理步长 | NOKOV 约 100 Hz、主机约 70 Hz；独立回调、有延迟/抖动/质量门控 |
| 数值条件 | 笔记建议 κΔt≤0.1，MATLAB 放宽到约 0.5 | 名义 10 ms 时为 0.12 / 0.20；构造时要求最大 κ×最大允许间隔≤1；仅是数值约束，不是稳定性保证 |
| 初值 | MATLAB 增加 1 mm 平移扰动 | 直接等于首帧位姿，至少接受 3 个样本才放行，从起飞前开始建立 β |
| 超界 | MATLAB 逐轴裁剪 β 后继续推进 | 拒绝整帧、保留上次状态和时间；不把触界 β 当成健康速度；持续失效走原受控退出 |
| 世界坐标 | 下载的 MATLAB 主系统 z-down | ROS z-up；保留现有 `+m*g*e3` 支撑力，不能复制 MATLAB 重力负号 |
| 绳速 | MATLAB 约束模型提供 linkRates，另有置零离心项选项 | 有独立飞机 EKF 速度，因此重建绳速，保留离心项，不将它置零 |
| 积分和构型补偿 | 完整 SE(3) 模式没有当前实飞的全部积分/trim | 保留位置 PID、构型 trim、支撑 trim、飞机姿态 PI 的原门控 |
| 安全与流程 | 理想动力学和仿真交接闸门 | 保留预检、TAKEUP、TAUT_RAMP、预抬、负载放下、卸载、三机下降和紧急缓存策略 |

**理论边界：**本版辅助状态演化是连续非混杂结构，但整机任务当然仍有起飞/接管/降落状态切换。
因为合力公式、积分、限幅、采样和内环均不同，不应将笔记中的 Lyapunov 证明直接用于本版实飞。
笔记中反求 W 的某些近似写法也未照搬；MATLAB 实际使用 `W=tr(Wbar)I-2Wbar`，
而本版 W 是独立正定参数，完全避免了该刚度反解问题。

## 5. 配置与日志

```yaml
slung_payload:
  state_observer_enabled: true
  state_feedback_mode: velocity_free_beta  # 切回 pose_observer 即恢复旧负载反馈
  velocity_free_kappa_position_per_s: 12.0
  velocity_free_kappa_attitude_per_s: 20.0
  velocity_free_attitude_weights: [1.0, 1.1, 1.2]
```

`observer_velocity_gain: 0.20` 是旧 alpha-beta-gamma 观测器的增益，与六维 β 无关。
新模式不使用旧五项残差增益，仍使用原 `observer_min_dt_s`、`observer_max_dt`、
`observer_min_samples`、位姿跳变门限以及速度/角速度边界。
新模式若 `state_observer_enabled=false` 会在初始化时报错，不会静默回退到消息 twist。

| CSV 字段 | 含义 |
|---|---|
| `payload_state_feedback_mode` | 确认实际使用 `velocity_free_beta` |
| `payload_beta_omega_body_x/y/z` | β 的角速度部分，rad/s |
| `payload_beta_velocity_body_x/y/z` | β 的体坐标线速度部分，m/s |
| `payload_beta_reference_omega_body_x/y/z` | Adjoint 变换后的参考角速度 |
| `payload_beta_reference_velocity_body_x/y/z` | Adjoint 变换后的参考线速度，含叉乘项 |
| `payload_beta_reference_active` / `payload_beta_reference_age_s` | 参考前馈是否新鲜、相对源时间的年龄 |
| `payload_acceleration_available` | 新模式为 0，防止误把零占位当实测加速度 |
| 原 `payload_observer_*` 与 `payload_source_observer_*` | 命令所用状态与最新源帧的有效性、拒绝、等待、重置计数 |

原消息 twist/acceleration 只留在 `raw_*` 诊断字段。控制使用的 β 与其他负载字段属于同一份快照，
短时复用完整运输指令时不会假装生成了新 β。参数切换需要重启节点，未增加空中热切换。

## 6. 离线运行

在 `ros_ws/src/crazyswarm/scripts`、已安装 MuJoCo 的 udaan 环境运行：

```bash
python simulate_slung_handoff.py --candidate simulation_configs/velocity_free_beta.yaml --output ctbr_logs/velocity_free_beta
python simulate_slung_handoff.py --candidate simulation_configs/velocity_free_baseline.yaml --output ctbr_logs/velocity_free_baseline
python plot_slung_handoff_trace.py ctbr_logs/velocity_free_beta/simulation_trace.csv
```

动态参考和噪声延迟的覆盖文件分别为 `velocity_free_cone.yaml`、`velocity_free_noise_delay.yaml`。
它们只影响离线运行，不会把实飞 hover 改成圆锥。
验证结果另见 [Velocity-free 验证记录](slung_velocity_free_verification.md)。
