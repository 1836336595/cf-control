# 实飞负载 Velocity-free β 反馈

本版新增 `state_feedback_mode: velocity_free_beta`，以负载位姿驱动一个 SE(3) 辅助系统，
用其输出 β 提供负载运动反馈，并以 MATLAB 的六维 SE(3) 外环生成负载合力与力矩。
原观测器及位置 PID/姿态 PD 通过 `pose_observer` 保留。
配置入口为 [slung_payload.yaml](../ros_ws/src/crazyswarm/config/slung_payload.yaml)，
辅助系统实现为 [slung_velocity_free.py](../ros_ws/src/crazyswarm/scripts/slung_velocity_free.py)，
六维外环实现为 [slung_load_controller.py](../ros_ws/src/crazyswarm/scripts/slung_load_controller.py)。

这里的 velocity-free 指**负载反馈不读取测得的速度、角速度或加速度**。
飞机位置控制仍使用每架飞机自己的 EKF 速度；飞机仍接收 CTBR 总推力和体坐标角速度指令。
本版已移植六维名义控制律；限幅坐标、采样、增益和下游实飞实现仍有差异，详见 §4。
这不意味着直接继承笔记的稳定性结论。

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

- $p_0$：NOKOV 上表面原点换算后的负载几何中心位置，世界坐标，m。
- $R_0$：负载体坐标到 ROS 世界坐标的旋转矩阵。
- $p_r,R_r$：现有悬停、圆周、八字或圆锥轨迹给出的负载参考。
- $\widehat X$：辅助位姿；首个有效样本按实测位姿加 1 mm 体系平移偏移初始化。
- $\widetilde R,\widetilde p$：$\widetilde X$ 的旋转和平移部分。

$$\widetilde\Psi_\omega=\tfrac12\operatorname{vee}(W\widetilde R-\widetilde R^T W),\qquad\widetilde\Psi_p=\tfrac d2\widetilde R^T\widetilde p.$$

- $W$：从实飞负载姿态刚度派生，辅助系统与六维外环使用同一权重。
- $\overline W=\frac12(\operatorname{tr}(W)I-W)$：用于归一化 β 的旋转修正增益。
- $d$：六维外环的平移势函数权重，当前为 0.1495；辅助更新中的 $d/2$ 与 $2/d$ 精确抵消，
  因而观测器直接计算 $\kappa_p\widetilde R^T\widetilde p$，没有单独保存 $d$。
- $\operatorname{vee}$：把三维反对称矩阵变回向量。

$$\beta=\operatorname{Ad}_{X_e^{-1}}\begin{bmatrix}\omega_r\\R_r^Tv_r\end{bmatrix}+\begin{bmatrix}\kappa_\omega\overline W^{-1}&0\\0&(2\kappa_p/d)I\end{bmatrix}\widetilde\Psi.$$

- $\beta=[\beta_\omega;\beta_v]$：六维速度替身，顺序为**体坐标角速度、体坐标线速度**。
- $\kappa_p$：`velocity_free_kappa_position_per_s: 12.0`，单位 s⁻¹。
- $\kappa_\omega$：`velocity_free_kappa_attitude_per_s: 20.0`，单位 s⁻¹。
- $v_r$：世界坐标参考速度，m/s；由轨迹提供，不是测量差分。
- $\omega_r$：参考姿态坐标系中的参考角速度，rad/s。

$$\operatorname{Ad}_{X_e^{-1}}\xi_r=\begin{bmatrix}R_0^TR_r\omega_r\\R_0^T\left[v_r+(R_r\omega_r)\times(p_0-p_r)\right]\end{bmatrix}.$$

- 第二块的叉乘项保留了参考旋转与位置误差之间的耦合；没有把 SE(3) 简化成两个互不相关的一阶滤波器。
- 参考由控制线程写入，下一次位姿回调使用；按源时间差以参考速度、加速度短时外推。
- 参考超过 `observer_max_dt: 0.05` s 不再前馈，转为零参考群速度下的位姿辅助系统。
- 这个 ROS 异步离散实现与 MATLAB 同步同拍参考不同；偏差与参考更新、测量延迟和调度有关，见 §4.2。

$$\dot{\widehat X}=\widehat X\left(\operatorname{Ad}_{\widetilde X}\beta\right)^\wedge,\qquad\widehat X_{k+1}=\widehat X_k\exp\left[\Delta t\left(\operatorname{Ad}_{\widetilde X_k}\beta_k\right)^\wedge\right].$$

- $\Delta t$：已接受 NOKOV 源时间戳的真实间隔；不是固定 70 Hz 控制周期。
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
| 负载合力与位置积分 | 六维外环通过 `PsiE/PsiT` 反馈；旧位置积分清零，`v0-vd` 仅保留为该外环的诊断量 |
| 负载姿态阻尼 | 来自 `-kXtilde*PsiT`；β 驱动辅助位姿，不再直接代入旧姿态 PD |
| 运动参考前馈 | 改由六维 `Upsilon` 计算，使用参考群速度及其导数；不叠加旧 PD 的参考坐标导数项 |
| 实测陀螺补偿 | 新模式不补 `Omega0 × J*Omega0`；按笔记 §8.4 和 MATLAB 的处理，不拿滞后 β 抵消这项 |
| 负载角加速度指令 | 新模式为 `J^-1 Md`；旧模式仍有原陀螺项处理 |
| 绳索方向变化、离心力 | 挂点速度用 `R0@beta_v + (R0@beta_omega) × rho_world`，再减飞机独立 EKF 速度 |
| `reference_cascade` 挂点加速度 | 仍使用参考加速度、参考角速度和参考角加速度 |
| `coupled` 挂点加速度 | 以当前 `Fd/m0`、`J^-1 Md` 和 `beta_omega` 构造；当前配置选择 `reference_cascade` |
| 预抬稳定确认、绳长校准、触地确认、trim 门控 | 读取同一份 β 状态，不暗中混入旧观测导数 |
| 位姿、测距、离地高度、目标构型 | 仍使用已接受的原始位姿，不用辅助位姿替换真实几何 |
| 飞机位置 PID、姿态 PI、Mellinger | 保持原实现；不是整套飞机也 velocity-free |

## 4. 与 MATLAB / 笔记的差异

本节只列出当前仍存在的实现差异和未完成的验证事项。
下文“MATLAB”特指 §1 固定哈希的本地 `Multi-UAV-Transportation-Simulation-vel-free`；
“笔记”指连续时间设计与整定建议。实飞采样、反馈来源和任务流程造成的差异不一定都需要消除，
但需要在对照公式和验证结果时明确区分。

| 对照项 | MATLAB 的实际处理 | 当前 ROS 实现及影响 |
|---|---|---|
| 参数覆盖与辅助增益 | 可覆盖 `W/d`；旋转 κ 逐轴，默认取约十倍自然频率再按仿真步长截断 | 未暴露 `W/d` 覆盖；平移 κ 固定 12、旋转统一 20 s⁻¹，见 §4.1 |
| 时序与预热 | 同步计算，在 SE(3) 接管分支首次运行时初始化辅助元 | 位姿回调与控制线程异步，接管前已预热，见 §4.2 |
| β 超界 | 在体坐标逐轴裁剪后继续推进辅助元 | 世界系线速度或体系角速度触界时拒绝整帧，见 §4.3 |
| 合力限幅 | 先旋转到世界系，再逐分量做软限幅 | **先在体系限幅，再旋转到世界系**，两者一般不等价，见 §4.3 |
| 张力与绳速 | 直接沿绳投影；约束模型提供 `linkRates`，可选置零离心项 | 增加单向张力约束；用负载 β 和飞机 EKF 速度重建绳速，见 §4.4 |
| 挂点补偿与力导数 | `coupled` 分支额外限制 β 角速度范数；禁止旧 PID/PD 的解析导数链 | 无该范数缩放；负载核心采用输出差分及限幅，见 §4.4 |
| 飞机控制与任务流程 | 仿真接管条件和内环 | 实飞飞机积分、构型/支撑补偿、接管及异常处理仍有差异，见 §4.5 |
| 日志与验证 | MATLAB 运行结果与笔记推导各有适用条件 | 六维中间量尚未写入 CSV；缺少新版本 MATLAB 轨迹对照、整机回放及实飞验证，见 §4.6 |

### 4.1 参数覆盖、辅助增益与采样

MATLAB 的 `se3vf.W/d` 允许显式覆盖；当前 ROS 参数入口没有暴露任意 $W,d$ 覆盖，
也不再读取旧的 `velocity_free_attitude_weights` YAML 键。
ROS 的 $W,d$ 从实飞刚度派生，修改 $k_X$ 会同时反比例重算两者；
若只改 $k_X$ 而保持 $k_{\widetilde X}$ 不变，跟踪刚度尺度会抵消，辅助误差反馈的相对强度会改变。
不能将这种设置等同于固定 $W,d$ 后单独调整 $k_X$。

MATLAB 默认按目标刚度和惯量选择
$\kappa_{\omega,i}=10\sqrt{K_{R,i}/J_{0,i}}$、
$\kappa_p=10\sqrt{\operatorname{mean}(\texttt{kx})}$，
再分别截断到 $0.5/\Delta t_{\rm sim}$。
因此它的默认物理增益会随仿真步长改变；步长收敛试验需要显式固定 κ。

ROS 保留固定的 $\kappa_p=12$ s⁻¹、$\kappa_\omega=20$ s⁻¹，
三个旋转轴共用一个 κ。它不会根据 70 Hz 主机周期或 100 Hz 位姿周期自动调增益。
按当前实飞配置的有效刚度估算，三个旋转自然频率约为 $[9.16,10.22,4.40]$ rad/s，
因此 $\kappa_\omega/\omega_n$ 约为 $[2.18,1.96,4.55]$，并不是 MATLAB 整定建议的 10。
**当前 $k_{\widetilde X}=16$ 不能据此宣称三轴阻尼比均为 0.8**；
笔记中该近似依赖逐轴 $\kappa_i=10\omega_{n,i}$，当前实飞增益不满足这个条件。

| 采样条件 | 当前值 | 可得出的结论 |
|---|---|---|
| 名义 NOKOV 间隔 | 10 ms | $\kappa_p\Delta t=0.12$、$\kappa_\omega\Delta t=0.20$ |
| 笔记建议 | $\kappa_i\Delta t\le0.1$ | 当前名义值已超过建议；指数映射本身不消除采样滞后 |
| ROS 构造检查 | $\max(\kappa_p,\kappa_\omega)\,\texttt{observer\_max\_dt}\le1$ | 当前 $20\times0.05=1$，通过数值边界检查 |
| 检查的适用范围 | 辅助系统允许的间隔与增益组合 | 不保证含绳索、飞机内环、延迟和饱和的整机稳定 |

### 4.2 异步参考与接管前预热

MATLAB 每次调用以同一组 $X_0,X_r,\widehat X_k$ 计算 $\Psi_E,\Psi_T,\beta_k$ 和控制量，
再保存 $\widehat X_{k+1}$。ROS 将这条链拆成两个时序：

1. **位姿回调**以 NOKOV 源时间戳计算已接受样本间隔，用上一接受样本的辅助群速度传播
   $\widehat X$，再用当前位姿和缓存参考计算 β；通过质量检查后保存完整快照。
2. **控制线程**以当前任务参考和已接受的位姿/辅助位姿快照计算六维控制量。
   它同时把本次参考写入观测器缓存，供下一次位姿回调使用，不会追溯重算这份快照中的 β。

接口在
[ctbr_payload_reference.py](../ros_ws/src/crazyswarm/scripts/ctbr_payload_reference.py) 的
`observer.set_reference(target, payload_state["sample_time"])`。
回调按参考源时刻的年龄外推位置、速度、姿态和角速度；参考年龄超出
`observer_max_dt=0.05` s 时，该回调的参考群速度前馈置零。
这只影响辅助系统的缓存前馈，**不代表控制线程的 $\Upsilon$ 也被置零**。
因此同一次输出中的 $\Psi_T$ 来自先前参考驱动的辅助历史，
而 $\Psi_E,\Upsilon$ 使用当前控制参考；其偏差取决于参考变化、延迟和实际调度。
负载核心还将传入的控制步长限制在 1–50 ms，用于积分及每机输出差分等运算；
辅助元传播使用的是已接受 NOKOV 样本的真实时间间隔。二者不是同一个时钟，
不能通过只提高主机调用频率来认为辅助系统也获得了更高频的位姿信息。

MATLAB 在 SE(3) 接管分支首次运行时初始化辅助元；ROS 在接管前的位姿回调中已经运行辅助系统。
ROS 初始化帧的 β 和用于传播的辅助群速度显式置零，后续有效样本才开始更新，
至少接受 `observer_min_samples=3` 个样本后才放行。
因此接管瞬间的 $\widehat X,\Psi_T,\beta$ 取决于各自的预热历史，不保证与 MATLAB 相同。

### 4.3 β 拒帧与合力软限幅

**β 的约束处理仍未照搬 MATLAB。**MATLAB 默认将体系角速度裁剪到每轴
$\pm10$ rad/s、体系线速度裁剪到每轴 $\pm3$ m/s，再用裁剪后的 β 推进辅助元。
当前 ROS 检查的是 $R_0\beta_v$ 的世界系逐轴速度界
$[1,1,1]$ m/s，以及 $\beta_\omega$ 的体系逐轴角速度界 $[8,8,8]$ rad/s。
达到边界也判为失败，状态标为 `beta_limit`；候选位姿、β、辅助位姿和接受时间都不提交。

重复/倒序时间戳、非有限位姿、过大的位姿跳变同样拒绝；
间隔小于 `observer_min_dt_s=0.005` s 时等待累计。
只有源时间流确实发生超过 0.05 s 的间断才重新初始化；
若接受时间过旧而新候选通过了前面的位姿跳变检查，则返回 `auxiliary_stale`。
连续坏帧不会触发这种源时间间断重置，不能靠不断拒绝把异常位姿最终“等成”新初值。
持续无可用状态由现有输出保持/受控退出流程处理。

**合力软限幅的坐标顺序是当前实现的一项未对齐差异。**
定义逐分量饱和 $S_L(x)=L\tanh(x/L)$，并令 $f_b=u_c[4{:}6]$，则：

$$
F_{d,\mathrm{MATLAB}}=S_{F_{\max}}(R_0f_b),\qquad
F_{d,\mathrm{ROS}}=R_0S_{F_{\max}}(f_b).
$$

即使先统一世界坐标，逐分量非线性仍一般不能与旋转交换顺序。
在线性小输入范围内两者接近；姿态倾斜且某些分量较大时，合力方向和大小都可能不同。
MATLAB 直接约束世界轴分量，ROS 直接约束体轴分量，因此 ROS 的世界轴分量也不一定各自小于
$F_{\max}$。这属于实现差异，不能只用“做了 z-up 转换”来解释。

### 4.4 单向张力、绳速与下游力指令

当前 ROS 对每根实测绳向 $q_i$ 继续执行：

$$
T_i^{\rm signed}=-q_i^T\mu_{id},\qquad
T_i=\max(0,T_i^{\rm signed}),\qquad \mu_i=-T_iq_i.
$$

若请求相当于压绳，ROS 将有效张力置零、有效目标绳向保持为当前绳向；
下载的 MATLAB 控制器这里直接采用 $q_iq_i^T\mu_{id}$。
所以即使上游 $F_d,M_d$ 完全相同，经过单向约束后实际请求也可能不同；
分配矩阵满秩不能替代张力方向、构型和执行器裕度检查。

ROS 没有直接可读的 `linkRates`，而是重建挂点相对飞机的速度：

$$
v_{a_i}=R_0\beta_v+(R_0\beta_\omega)\times(R_0\rho_i),\qquad
\dot q_i=\frac{I-q_iq_i^T}{\|p_0+R_0\rho_i-p_i\|}(v_{a_i}-v_i),\qquad
\omega_i=q_i\times\dot q_i.
$$

其中 $v_i$ 仍来自飞机自己的 EKF。离心项 $m_iL_i\|\omega_i\|^2q_i$ 保留，
所以“负载 velocity-free”不能扩展为“所有反馈都不使用速度”。
MATLAB 约束模型直接给出绳速，并有置零离心项的配置，当前 ROS 没有采用该置零策略。

MATLAB 的 SE(3) `coupled` 分支在使用 $\beta_\omega$ 构造 $a_i$ 前，
还将其范数限制到 `betaRateMax`；ROS 目前只有 §4.3 的逐轴拒帧，没有这次额外范数缩放。
ROS 负载核心输出的每机力导数由相邻控制输出差分并受 `force_dot_limit_nps=20` 逐分量限幅，
后级仍可能加入支撑 trim、构型参考等力及其导数；
没有复用 MATLAB 对旧 PID/PD 推导的完整解析导数链。
MATLAB 明确禁止 `se3vf` 与其旧 `omegaCMethod='analytic'` 链同时启用；
这不能与 ROS 后级“给定力及力导数，解析求期望机体角速度”的运算混为一谈。

### 4.5 飞机控制、构型补偿与实飞接管

负载外环之外，ROS 仍有以下飞机控制和补偿环节，不能将 MATLAB 的负载外环结果直接当作整机结果：

| 环节 | β 模式中的当前行为 |
|---|---|
| 飞机运输位置控制 | 当前选择 `native_position_pid`，使用各机 EKF 反馈和运输专用积分 |
| 差分保持力积分 | 属于另一种运输反馈选择，不能与 `native_position_pid` 的生效路径混为一谈 |
| 构型 trim | 动态姿态补偿仍调整飞机参考构型，并受原来的速度、位姿质量和阶段门控 |
| 支撑 trim | 仍承接预抬到运输的竖直力差额，按门控和限幅慢调 |
| 飞机姿态控制及机载内环 | 继续执行原 CTBR/姿态积分与固件速率控制，未移植为负载的六维控制律 |

MATLAB 的 SE(3) 接管条件默认是其代码定义的负载高度差至少 0.10 m、
真值竖直速度不超过 0.02 m/s，连续满足 0.50 s 后锁存。
ROS 使用完整 TAKEUP/预抬流程；当前预抬基准为旋转负载**最低点离地间隙**
0.20 m、β 世界系竖直速度不超过 0.05 m/s、确认 0.50 s，并结合绳长、构型和状态新鲜度。
两个高度量的几何定义和速度来源也不同，不能只比较门限数值。

ROS 接管后继续 TAUT_RAMP、参考抬升、运输、负载下降、触地确认、卸载和三机下降；
状态失效时走现有缓存/受控退出流程，不会自动在空中切回 `pose_observer`。
MATLAB 的单次 SE(3) 锁存不是这些实飞状态机和异常处理的替代品。
“非混杂”只描述所采用辅助控制结构中没有额外姿态跳变机制，
不表示整机软件没有状态切换、拒帧或重新初始化。

### 4.6 尚未完成的日志与验证

**尚未调用 MATLAB 运行同一输入轨迹进行直接对照。**
现有 Python 非零参考测试的期望合力也采用“体系 $\tanh$ 后旋转”，
因此该测试通过不能证明 §4.3 的限幅顺序与 MATLAB 一致。

任务到 CSV 的转发链尚未记录 `payload_se3_upsilon`、`payload_se3_psi_error`、
`payload_se3_psi_tilde` 和未限幅的 `payload_se3_control_wrench` 四个六维中间量。
现有 CSV 中的 `payload_desired_force_*`、`payload_desired_moment_*` 是交给分配器的最终值，
不能拿它们直接替代未限幅的 $u_c$ 逐项核对。

[Velocity-free 验证记录](slung_velocity_free_verification.md) 中的五组悬停/圆锥/噪声回放
来自此前“β 替换旧 PID/PD 速度反馈”的版本。
那些 RMS、命令抖动和完成降落的数据不能当作本次六维外环移植后的整机结果；
动态参考、实体近地效应及真实异常工况仍需进一步验证；当前没有完整实飞通过结论。
2026-11-18 已补充名义与噪声/较大延迟下的完整回放，但模型未复现最新实飞的大振荡和近地残差，
其适用范围见 [放载与降落控制](slung_descent_control.md)。

笔记的连续时间 Lyapunov 推导也不能直接成为当前实飞证明。
当前实现还有异步采样、固定 κ、β 拒帧、软限幅坐标差异、单向绳力投影、
构型/支撑补偿和有限带宽飞机内环等因素。
“全驱动”描述的是上层六维虚拟广义力模型，不保证绳索和执行器在任意构型下都能实现该请求。

## 5. 配置与日志

```yaml
slung_payload:
  state_observer_enabled: true
  state_feedback_mode: velocity_free_beta  # 切回 pose_observer 恢复旧观测器和负载 PID/PD
  velocity_free_kappa_position_per_s: 12.0
  velocity_free_kappa_attitude_per_s: 20.0
  velocity_free_initial_offset_m: 0.001
  velocity_free_kx: 1.0
  velocity_free_kxtilde: 16.0
  # 可选覆盖；省略时按机队推力和挂点力臂派生：
  # velocity_free_force_limit_n: 7.060788
  # velocity_free_moment_limit_nm: 0.394710
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
历史 β 反馈版本的验证结果见 [Velocity-free 验证记录](slung_velocity_free_verification.md)；
其数据不代表当前六维外环已完成同样回放，适用范围见 §4.6。
