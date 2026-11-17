# 三机吊运控制公式与代码对照

本文记录当前实现的数学关系，供本地支持 LaTeX 的 Markdown 阅读器查看。
运行入口、操作步骤和参数索引见 [README](../README.md)。公式对应重构后的模块，
并区分当前启用分支、保留的对照分支和实验接口；参数以实际加载的 YAML 为准。

## 1. 当前任务与实现位置

| 项目 | 当前配置 |
|---|---|
| 主机任务 | `slung_load`，主机控制频率 70 Hz |
| 负载力构造 | `reference_cascade` |
| 飞机运输反馈 | `native_position_pid`，运输积分速度权重 1.0 |
| 负载参考 | 当前 `attitude_cone`；轨迹参数与普通飞机 `hover` 分别配置 |
| 圆锥切换配置 | 15°、两圈各 30 s、倾斜/恢复各 5 s，中心固定在世界 `[0,0,0.6]` m |
| 圆锥接管后的时钟 | h20 倾斜，h25/55 分别开始两圈，h85 恢复水平，h100 计划下降 |
| 动态构型补偿 | 动态开关和抬升期间更新均为 `true`，使用世界系端点学习 |
| 名义几何 | 负载质量 0.023 kg，尺寸 `[0.08,0.06,0.05]` m，CF3/CF4/CF5 绳长为 `[0.857,0.860,0.854]` m |
| 负载姿态带宽 | `[1.5,1.5,0.7]` Hz，阻尼比 0.90 |

这里的 h0 是 `TAUT_RAMP` 开始，不是解锁、离地或到达 0.6 m 的时刻。
普通飞机任务仍为 `ctbr_trajectory.trajectory_mode: hover`；负载轨迹单独驱动三机参考。
配置见 [slung_payload.yaml](../ros_ws/src/crazyswarm/config/slung_payload.yaml)、
[ctbr_vehicle.yaml](../ros_ws/src/crazyswarm/config/ctbr_vehicle.yaml)。

| 数学职责 | 当前源码 |
|---|---|
| 独立起飞、预抬和放载后下降 | [ctbr_independent_controller.py](../ros_ws/src/crazyswarm/scripts/ctbr_independent_controller.py) |
| 飞机运输反馈、接管混合、滤波 | [ctbr_transport_vehicle_controller.py](../ros_ws/src/crazyswarm/scripts/ctbr_transport_vehicle_controller.py) |
| 合力到姿态、角速度与推力 | [ctbr_geometric_controller.py](../ros_ws/src/crazyswarm/scripts/ctbr_geometric_controller.py) |
| 向量、旋转和限幅数学 | [ctbr_control_math.py](../ros_ws/src/crazyswarm/scripts/ctbr_control_math.py) |
| 负载外环、分配和每机基础力 | [slung_load_controller.py](../ros_ws/src/crazyswarm/scripts/slung_load_controller.py) |
| 负载及三机一致参考 | [ctbr_payload_reference.py](../ros_ws/src/crazyswarm/scripts/ctbr_payload_reference.py) |
| 慢修正、门控、下降参考衔接 | [ctbr_payload_control.py](../ros_ws/src/crazyswarm/scripts/ctbr_payload_control.py) |
| 任务、触地与故障退出 | [ctbr_payload_mission.py](../ros_ws/src/crazyswarm/scripts/ctbr_payload_mission.py) |

## 2. 坐标、端点与单位

令 $e_3=[0,0,1]^T$；世界 z 向上，物理重力为 $-ge_3$。
$R_0,R_i$ 分别把负载、飞机机体系向量转换到世界系；$\Omega_0,\Omega_i$ 使用各自机体系坐标。
$\widehat a\,b=a\times b$，$(\cdot)^\vee$ 是对应的反对称矩阵到向量映射。
位置、速度、加速度、力、力矩和角速度分别用 m、m/s、m/s²、N、N·m、rad/s。

负载动捕原点在上表面中心，进入控制器前转换为几何中心：

$$x_0=x_{\rm top}-R_0[0,0,h/2]^T.$$

当前将该几何中心作为质心；没有单独辨识质心偏移。
$\rho_i$ 是中心到负载挂点的负载体系向量，CF3/CF4/CF5 顺序为
`[0.04,0,0.025]`、`[-0.04,0.03,0.025]`、`[-0.04,-0.03,0.025]` m。
飞机端点 $x_i$ 使用受控动捕原点，未自动加入飞机挂点相对该原点的机体固定偏移。

绳向从飞机指向负载挂点：

$$r_i=x_0+R_0\rho_i-x_i,\quad d_i=\|r_i\|,\quad q_i=r_i/d_i.$$

$$\dot q_i={I-q_iq_i^T\over d_i}
\left[v_0+(R_0\Omega_0)\times(R_0\rho_i)-v_i\right],
\qquad \omega_i=q_i\times\dot q_i.$$

$q_i,\dot q_i,\omega_i$ 均在世界系；实时归一化分母为实测端点距离 $d_i$。
模型绳长 $l_i$、参考冻结距离 $d_{ih}$、实时距离 $d_i$ 必须区分。
启用一次性绳长校准时，运行有效 $l_i$ 可以不同于 YAML 名义值。
箱体离地间隙按旋转后的最低边界计算，不能将动捕 z 直接当成底面高度。

飞机位置/旋转来自 NOKOV；平移速度来自机载 EKF 速度包，经主机滤波后差分得到加速度。
EKF 位置用于与动捕位置的一致性检查。运输机体角速度目前来自动捕姿态差分滤波，
不是直接订阅的机载陀螺仪值。负载导数来自负载位姿观测器。
导数未就绪时关闭飞机速度误差反馈，并以期望加速度代替未知测量加速度。
主机控制计算将积分/差分步长限制在 `[0.001,0.05]` s；任务相位仍按单调时钟计算。

## 3. 独立飞行 PID 与专用 Z 积分

下文误差统一使用“测量减参考”：$e_p=x_i-x_{id}$、$e_v=v_i-v_{id}$。
独立分支使用加速度增益 $K_p,K_v,K_I$，其更新和输出为：

$$I_i^+=\operatorname{clip}_{\pm I_{\max}}
\left[I_i+\Delta t(e_v+0.5e_p)\right],\qquad \|e_p\|<\varepsilon_I,$$

$$a_{\rm fb}=\operatorname{clip}_{\pm a_{\max}}
(-K_pe_p-K_ve_v-K_II_i),\qquad
F_{i,\rm ind}=m_i(a_{id}+ge_3+a_{\rm fb}).$$

位置门限之外保持积分；当前普通独立门限 0.20 m、各轴积分限幅 0.20。
预抬时 XY 增益额外乘配置倍率 P=`[4,4,1]`、D=`[2,2,1]`。
其轨迹生成及实测绷紧/间隙门槛见
[ctbr_payload_takeup.py](../ros_ws/src/crazyswarm/scripts/ctbr_payload_takeup.py)。

预抬或放载后独立下降启用专用 Z 更新，恢复普通步骤前保存的 Z 积分，再计算候选值：

$$\dot I_z^{\rm req}=\operatorname{clip}_{\pm r_z}(e_{v,z}+0.5e_{p,z}),\quad
I_z^{\rm cand}=\operatorname{clip}_{\pm L_z}(I_z+\Delta t\dot I_z^{\rm req}).$$

只有导数有效、Z 误差门限满足、且候选增量不继续加深反馈加速度或投影推力饱和时接受。
下降检查还包括阶段最小推力，防止积分继续把输出压向下限。
预抬 `(L_z,r_z,a_max,z)` 为 `(8.0,0.5,12)`；放载后下降为 `(2.0,0.5,8)`。
预抬后保存的支撑偏差在交接及放载流程中另行处理，不能简单在每个阶段清零。

普通非吊运几何模式保留另一条原有关系：

$$\dot I_i=e_v+c_1e_p,\qquad
F_i=m_i(a_{id}+ge_3)-K_xe_p-K_ve_v-K_II_i.$$

这里普通几何配置的力增益与独立分支的加速度增益单位不同，不能直接互换。
运输阶段冻结普通位置积分，运输专用积分具有独立状态。
正常连续阶段不清除悬停积分；状态失效、紧急下降、终止或新任务按相应复位路径处理。

## 4. 负载合力、姿态力矩和分配

负载位置误差 $e_x=x_0-x_{0d}$、$e_v=v_0-v_{0d}$：

$$\dot I_x=e_v+c_1e_x,\quad
a_{0c}=a_{0d}-K_xe_x-K_ve_v-K_II_x,\quad F_d=m_0(a_{0c}+ge_3).$$

$I_x$ 每轴限幅 0.50。令 $Q=R_0^TR_{0d}$，则：

$$e_R={1\over2}(R_{0d}^TR_0-R_0^TR_{0d})^\vee,\qquad
 e_\Omega=\Omega_0-Q\Omega_{0d}.$$

当前启用的负载姿态律为 PD 加刚体前馈：

$$M_d=-K_Re_R-K_\Omega e_\Omega
+\Omega_0\times J_0\Omega_0
-J_0(\Omega_0\times Q\Omega_{0d})+J_0Q\dot\Omega_{0d}.$$

$$J_0=\operatorname{diag}(J_{xx},J_{yy},J_{zz}).$$

- $J_{xx}$：负载质心处绕负载 X 轴的惯量，当前取 $1.1691666666666667\times10^{-5}$ kg·m²。
- $J_{yy}$：负载质心处绕负载 Y 轴的惯量，当前取 $1.7058333333333336\times10^{-5}$ kg·m²。
- $J_{zz}$：负载质心处绕负载 Z 轴的惯量，当前取 $1.9166666666666667\times10^{-5}$ kg·m²。

三项从 `slung_payload.inertia_diag_kgm2` 显式读取；当前假设惯量积为零。
主机增益、刚体力矩补偿及 `simulate_slung_handoff.py` 的 MuJoCo 负载均使用同一组值。
修改 `mass_kg` 或 `size_m` 不会再自动改变惯量；更换负载后需分别核对质量、几何和惯量。

按轴设置 $K_{R,j}=J_{0,jj}(2\pi f_j)^2$、
$K_{\Omega,j}=2\zeta J_{0,jj}(2\pi f_j)$；当前 $f=[1.5,1.5,0.7]$ Hz、$\zeta=0.90$。
关闭 yaw 通道时清零请求力矩 Z；当前运动任务要求该通道开启。

代码还包含 [实验负载力矩积分](../ros_ws/src/crazyswarm/scripts/slung_attitude_integral.py)
及分配投影方向检查，由 `payload_attitude_integral_*` target 字段显式启用。
当前任务管理器没有启用该接口；不要将它混同于当前运行的参考 trim，也不要说代码中不存在它。

将三机虚拟力的负载体系表达堆叠为 $z$：

$$P=\begin{bmatrix}I&I&I\\\hat\rho_1&\hat\rho_2&\hat\rho_3\end{bmatrix},\quad
z=P^T(PP^T)^{-1}\begin{bmatrix}R_0^TF_d\\M_d\end{bmatrix}+z_{\rm null},
\qquad Pz_{\rm null}=0.$$

$z_{\rm null}$ 由挂点朝外的候选偏置投影到分配矩阵零空间并按预算缩放。
转到世界系得到 $\mu_{id}$ 后，以实际绳向实施单边张力：

$$T_i^{\rm signed}=-q_i^T\mu_{id},\quad
T_i=\max(T_i^{\rm signed},0),\quad \mu_i=-T_iq_i.$$

正张力时期望绳向 $q_{id}=-\mu_{id}/\|\mu_{id}\|$；非正张力时取 $q_{id}=q_i$。
投影、单边裁剪后，三根绳的实际请求扳手不一定仍等于虚拟分配目标。
零空间偏置的零扳手性质也不能无条件延续到裁剪之后。
这里是模型请求张力，不是张力传感器读数，也不是完整可行扳手 QP。

## 5. `coupled` 与 `reference_cascade` 基础飞机力

两分支共享负载外环、分配和实测绳几何，核心输出均可写成：

$$u_i=\mu_i+m_il_i\|\omega_i\|^2q_i+m_i a_i+F_{q,i}.$$

`coupled` 使用测量状态和请求力矩反算的角加速度：

$$\alpha_{0c}=J_0^{-1}(M_d-\Omega_0\times J_0\Omega_0),\quad
 a_i=a_{0c}+ge_3+R_0\hat\Omega_0^2\rho_i-R_0\hat\rho_i\alpha_{0c}.$$

其绳向误差 $e_q=q_{id}\times q_i$，逐轴限幅的绳向积分为 $I_{q,i}$：

$$c_{q,i}=-\gamma(K_qe_q+K_\omega\omega_i+K_{Iq}I_{q,i}),\quad
 F_{q,i}^{\rm raw}=m_il_i(q_i\times c_{q,i}).$$

绳向纠正力单独作范数、变化率限制；运动惯性项不进入这个纠正力限幅历史。
当前配置的绳向积分增益为 0；该分支保留用于对照，不等同于完整论文自适应控制律。

当前 `reference_cascade` 使用期望挂点加速度：

$$a_i=a_{0d}+ge_3+R_{0d}\hat\Omega_{0d}^{\,2}\rho_i
-R_{0d}\hat\rho_i\dot\Omega_{0d},\qquad \gamma=0,\quad F_{q,i}=0.$$

负载反馈仍进入张力分配；飞机位置反馈负责跟踪构型。
代码分解为沿绳的 $m_iq_iq_i^Ta_i$ 与垂直绳的 $-m_i\hat q_i^2a_i$，
因 $q_iq_i^T-\hat q_i^2=I$，合起来正是上式的 $m_ia_i$。
每机基础力导数由相邻输出差分，并按 `force_dot_limit_nps` 逐轴限制。

## 6. 静态构型建立与接管连续性

接管冻结实测 $x_{0h},R_{0h},x_{ih}$，计算 $d_{ih},q_{ih}$。
使用 $s(u)=10u^3-15u^4+6u^5$，$u=\operatorname{clip}(h/12,0,1)$：

$$R_b=R_{0h}\exp\left(s\log(R_{0h}^TR_g)\right),\quad
 q_{ib}=\operatorname{Slerp}(q_{ih},q_{i\star};s).$$

$R_g$ 为水平初始任务航向；圆锥/单圆为 yaw 0，八字为 yaw $-\pi/2$。
$q_{i\star}$ 是同一个分配器在重力、零力矩、$R_g$ 下得到的静态平衡绳向，
不是每周期带负载反馈的 `desired_link_direction`。

$$x_{ib}=x_{ih}+(x_{0d}-x_{0h})+(R_b-R_{0h})\rho_i
-d_{ih}(q_{ib}-q_{ih}).$$

速度/加速度由同一旋转及球面插值解析求导，包含 $-d_{ih}\dot q_{ib}$、$-d_{ih}\ddot q_{ib}$。
保留的独立 PID 目标偏差另在张力混合中消退；`position` 与 `position_hold_position`
在接管时可能不同，后者才是实际保持反馈使用的参考。
实现见 [slung_reference.py](../ros_ws/src/crazyswarm/scripts/slung_reference.py)。

h0–5 用五次曲线将协同混合系数 $\beta$ 从 0 增至 1；h5–17 用相同形状抬升位置参考。
姿态/平衡绳向在 h12 到端点；h17 后额外保持 3 s，再开始所选负载轨迹。
这些是参考时间，不表示实测姿态已经收敛。

## 7. 圆周、八字、圆锥及构型的完整 PVA

### 7.1 圆周和八字

见 [slung_circle_reference.py](../ros_ws/src/crazyswarm/scripts/slung_circle_reference.py)。
每个整圆的相位 $\phi$ 从 0 增至 $2\pi$，峰值角速度
$\omega_p=\min(v_{\max}/r,2\pi/T_r)$；单圆持续时间 $D=2\pi/\omega_p+T_r$。
起步 $u=t/T_r$ 时 $\dot\phi=\omega_p s(u)$，
$\phi=\omega_pT_r(2.5u^4-3u^5+u^6)$；停止段为时间镜像，中段匀速。
单圆允许正数圈数 $N$，其公式将总角度替换为 $2\pi N$，按整条圆周任务平滑起停；
八字要求正整数个完整双圆周期，每个圆瓣都独立起停。

单圆 $c$ 的参考为 $x_{0d}=c-r[\cos\phi,\sin\phi,0]^T$，yaw=$\phi$，+X 朝圆心；
这里 $\phi$ 取带方向符号的相位。
八字交点为 $c$，令 $\sigma=+1$ 对应 `ccw`、$-1$ 对应 `cw`：

$$x_{0d}^{\rm right}=c+[r,0,0]^T-r[\cos\phi,\sigma\sin\phi,0]^T,
\quad \psi^{\rm right}=\sigma\phi-\sigma\pi/2,$$

$$x_{0d}^{\rm left}=c-[r,0,0]^T+r[\cos\phi,-\sigma\sin\phi,0]^T,
\quad \psi^{\rm left}=\sigma(2\pi-\phi)-\sigma\pi/2.$$

平面表达的 z 分量增量为 0，交点 $c_z=0.6$ m；每个圆在交点静止后再进入另一个圆。
航向采用切线的解析极限，停车时不对零速度调用 `atan2`；两圆连接处 PVA、角速度、角加速度连续。
当前单圆 $D=36.4159265$ s，完整八字 $2D=72.8318531$ s。

先建立基准构型，再令 $Q=R_{\rm motion}R_{\rm initial}^T$、$c_i=x_{ib}-x_{0d}$：

$$x_{id}=x_{0d}+Qc_i,\quad
v_{id}=v_{0d}+Q\dot c_i+\dot Qc_i,$$

$$a_{id}=a_{0d}+Q\ddot c_i+2\dot Q\dot c_i+\ddot Qc_i.$$

$Q$ 的 yaw 增量是 $\psi-\psi_{\rm initial}$，八字初始 yaw=$-\sigma\pi/2$。
同一变换作用于实际保持参考、接管参考；负载旋转也使用 $R_{0d}=QR_b$。
静态 trim 的已有偏置同样旋转，动态 trim 的世界系 PVA 则在名义构型旋转后叠加，不再次乘 $Q$。
完整链式求导包括科氏项，不能只移动负载而固定飞机偏移，或只旋转位置却漏掉导数。

基础力已含挂点惯性和实测绳摆向心项，因此附加构型加速度为：

$$\Delta a_i=a_{id}-a_{{\rm attachment},id}
-d_{i,\rm ref}\|\dot q_{i,\rm ref}\|^2q_{i,\rm ref},$$

$$a_{{\rm attachment},id}=a_{0d}+R_{0d}
[\Omega_{0d}\times(\Omega_{0d}\times\rho_i)+\dot\Omega_{0d}\times\rho_i].$$

管理器增加 $m_i\Delta a_i$，并对该附加力作离散差分加入力导数。
`waiting` 阶段该项强制为 0，保持既有启动律。
在精确参考跟踪且 $l_i=d_{i,\rm ref}$ 时，基础力与该项相加为 $\mu_i+m_i(a_{id}+ge_3)$；
实测/参考绳长不一致或 trim 改变长度时，应以代码完整表达为准，不能无条件套用此化简。

### 7.2 固定质心的圆锥姿态

实现见 [slung_attitude_cone_reference.py](../ros_ws/src/crazyswarm/scripts/slung_attitude_cone_reference.py)。
固定 $x_{0d}=c=[0,0,0.6]^T$，$v_{0d}=a_{0d}=0$。令半角 $\theta_0$ 为配置值，当前为 $15^\circ$，
进入/退出时间 $T_e=5$ s，单圈扫描时间 $T=30$ s，正整数圈数 $N=2$，方向 $\sigma=\pm1$。
倾斜阶段 $\theta=\theta_0s(t/T_e),\phi=0$，此处 $t$ 从倾斜起点计时。
扫描阶段令 $\tau\in[0,NT)$ 从扫描起点计时，并取圈序号和圈内归一化时间

$$k=\left\lfloor{\tau\over T}\right\rfloor,\qquad
u={\tau-kT\over T},\qquad k\in\{0,\ldots,N-1\},\quad u\in[0,1).$$

$$\theta=\theta_0,\qquad
\phi=\sigma2\pi[k+s(u)],\qquad
\dot\phi={\sigma2\pi\over T}s'(u),\qquad
\ddot\phi={\sigma2\pi\over T^2}s''(u).$$

这里 $s(u)=10u^3-15u^4+6u^5$，导数对 $u$ 求取；扫描进度为 $[k+s(u)]/N$。
每圈独立使用同一五次相位曲线，因此一圈总时间均为 30 s，角速度并非常量。
圈边界的 $\phi$ 连续，$\dot\phi=\ddot\phi=0$，同时维持 $\theta=\theta_0$；不在圈间恢复水平。
恢复阶段 $\theta=\theta_0[1-s(t/T_e)],\phi=\sigma2\pi N$，此处 $t$ 从恢复起点计时。
完整姿态参考时长为 $2T_e+NT$，默认 70 s；每增加一圈增加 $T$，进入/退出时长不变。

$$R_c=R_z(\phi)R_y(\theta)R_z(-\phi),\qquad
R_c e_3=[\cos\phi\sin\theta,\ \sin\phi\sin\theta,\ \cos\theta]^T.$$

这选定了 +Z 方向之外的完整姿态自由度：整圈参考闭合，没有额外指定 yaw 转一圈；
它不是绕自身 +Z 的瞬时自旋严格为零的运输规则。
定义 $u=[-\sin\phi,\cos\phi,0]^T$、
$v=[-\cos\phi\sin\theta,-\sin\phi\sin\theta,\cos\theta-1]^T$，则

$$\Omega_c=\dot\theta u+\dot\phi v,\qquad
\dot\Omega_c=\ddot\theta u+\dot\theta\dot\phi\partial_\phi u
+\ddot\phi v+\dot\phi\dot\theta\partial_\theta v+\dot\phi^2\partial_\phi v.$$

与有限启动参考 $R_b$ 的组合为 $R_{0d}=R_cR_b$，
$\Omega_{0d}=\Omega_b+R_b^T\Omega_c$，
$\dot\Omega_{0d}=\dot\Omega_b+R_b^T\dot\Omega_c-\Omega_b\times(R_b^T\Omega_c)$。
扫描开始时 $R_b$ 已到水平端点。

圆锥模式不将整个水平构型刚性倾斜。令 $A=P^T(PP^T)^{-1}$，$A_i$ 为其第 i 个
三维力块的前三列，$b_i$ 为原内张力零空间向量的相应块，$F_g=m_0ge_3$：

$$\mu_i^{\rm eq}(R)=RA_iR^TF_g+Rb_i,\quad
q_i^{\rm eq}=-\mu_i^{\rm eq}/\|\mu_i^{\rm eq}\|,\quad
x_{id}=c+R\rho_i-d_{ih}q_i^{\rm eq}.$$

$$v_{id}=\dot R\rho_i-d_{ih}\dot q_i^{\rm eq},\qquad
a_{id}=\ddot R\rho_i-d_{ih}\ddot q_i^{\rm eq},\quad
\dot R=R\widehat\Omega_{0d},\quad
\ddot R=R(\widehat{\dot\Omega}_{0d}+\widehat\Omega_{0d}^{\,2}).$$

$\dot q_i^{\rm eq},\ddot q_i^{\rm eq}$ 由归一化向量的解析导数得到；参考绳距 $d_{ih}$ 冻结。
这是名义重力、零静态力矩对应的构型，实际张力分配仍包含负载 PD 和动态力矩前馈。
完整构型加速度沿用上面的 $\Delta a_i$，避免重复添加挂点和向心项。
真实轴方向误差用 $\arccos((R_0e_3)^T(R_{0d}e_3))$，锥角用
$\arccos(e_3^TR_0e_3)$；仅检查 Euler yaw 无法判断是否正确扫描圆锥。

## 8. 飞机运输 native PID、差分积分与支撑 trim

本节对应 `GeometricCtbrController` 内的运输位置反馈，源码为
[ctbr_transport_vehicle_controller.py](../ros_ws/src/crazyswarm/scripts/ctbr_transport_vehicle_controller.py)
和 [native_position_pid.py](../ros_ws/src/crazyswarm/scripts/native_position_pid.py)。
当前启用 `native_position_pid`；第 8.6 节仅保留另一分支的对照说明。
每个公式下逐行给出符号、单位及当前取值；实测状态、轨迹参考、在线积分没有固定常数值。
向量按世界 XYZ 排列，增益向量在公式中写为对角矩阵。

### 8.1 构型位置与速度误差

$$e_{p,i}=x_i-x_{i,\mathrm{hold}}.$$

- $i$：飞机编号，依次为 CF3、CF4、CF5。
- $e_{p,i}$：构型位置误差，单位 m；在线计算，目标为 $[0,0,0]^T$。
- $x_i$：飞机动捕刚体原点的世界位置，单位 m；取 NOKOV 实测值。
- $x_{i,\mathrm{hold}}$：当前构型位置参考，单位 m；取 `position_hold_position`，包含第 9 节的构型 trim。

$$e_{v,i}=v_i-v_{i,\mathrm{hold}}.$$

- $e_{v,i}$：构型速度误差，单位 m/s；在线计算，目标为 $[0,0,0]^T$。
- $v_i$：飞机平移控制速度，单位 m/s；取经过主机滤波的 EKF 速度，CSV 对应 `control_velocity`。
- $v_{i,\mathrm{hold}}$：构型速度参考，单位 m/s；取 `position_hold_velocity`，由第 7、9 节的参考解析生成。

静态参考建立后，名义构型速度为零；动态圆锥或 trim 平滑段中的速度参考一般非零。
导数未就绪时，代码暂设速度误差为零并禁止积分学习，不能将其理解为已测得零速度。

### 8.2 位置 PD 反馈与当前增益

$$p_i=-K_{p,i}e_{p,i}-K_{v,i}e_{v,i}.$$

- $p_i$：尚未乘保持倍率的 PD 反馈加速度，单位 m/s²；在线计算。
- $K_{p,3}$：CF3 位置增益，取 $\operatorname{diag}(3.0,3.0,4.0)$ s⁻²，对应 `independent_position_gain`。
- $K_{p,4}$：CF4 位置增益，取 $\operatorname{diag}(4.0,4.0,4.0)$ s⁻²。
- $K_{p,5}$：CF5 位置增益，取 $\operatorname{diag}(4.0,4.0,4.0)$ s⁻²。
- $K_{v,3}$：CF3 速度增益，取 $\operatorname{diag}(3.12,3.12,3.60)$ s⁻¹，对应 `independent_velocity_gain`。
- $K_{v,4}$：CF4 速度增益，取 $\operatorname{diag}(3.60,3.60,3.60)$ s⁻¹。
- $K_{v,5}$：CF5 速度增益，取 $\operatorname{diag}(3.60,3.60,3.60)$ s⁻¹。
- $e_{p,i}$：位置误差，定义及来源见第 8.1 节。
- $e_{v,i}$：速度误差，定义及来源见第 8.1 节。

$$a_{PD,i}=\mathcal C_{a_{\max}}(\lambda p_i).$$

- $a_{PD,i}$：实际使用的 PD 反馈加速度，单位 m/s²；在线计算。
- $\mathcal C_{a_{\max}}$：逐轴裁剪算子，各分量裁剪到 $[-a_{\max,j},a_{\max,j}]$。
- $a_{\max}$：三机均取 $[6.0,6.0,8.0]^T$ m/s²，对应 `independent_max_feedback_acceleration`。
- $\lambda$：无量纲保持倍率；张力渐增时取 2.0，随后在 12 s 抬升段内平滑过渡到 1.5，稳态取 1.5。
- $p_i$：式中上一条 PD 加速度，不是飞机滚转角速度符号。

上述增益先产生加速度，再乘飞机质量得到力。
稳态 CF3 的有效 P/D 分别为 $[4.5,4.5,6.0]$、$[4.68,4.68,5.40]$；
CF4、CF5 分别为 $[6.0,6.0,6.0]$、$[5.40,5.40,5.40]$。
这里不使用普通非吊运分支的 `position_gain`、`velocity_gain` 替代这两组增益。

### 8.3 运输专用位置积分

$$I_{t,i}^{k+1}=\mathcal C_{L_I}\!\left(I_{t,i}^{k}+\chi_i^k\Delta t_k\left[w_v e_{v,i}^{k}+c_Ie_{p,i}^{k}\right]\right).$$

- $I_{t,i}^{k}$：第 $k$ 个控制周期的运输位置积分状态，等效单位 m；接管初始化为 $[0,0,0]^T$，随后在线更新。
- $\mathcal C_{L_I}$：逐轴积分状态限幅，将每轴裁剪到 $[-L_{I,j},L_{I,j}]$。
- $L_I$：三机均取 $[2/3,2/3,2/3]^T$ m，来自 $[0.2,0.2,0.2]\times3.3333333333333335$。
- $\chi_i^k$：积分准入量，满足全部学习条件且 $\|e_{p,i}\|<0.23$ m 时取 1，否则取 0；同一 tick 不重复积分。
- $\Delta t_k$：实际积分步长，单位 s；代码裁剪到 $[0.001,0.05]$，70 Hz 名义值约 0.014286 s。
- $w_v$：积分中的速度误差权重，当前取 1.0，对应 `transport_position_pid_integral_velocity_weight`；只缩放 I 输入中的速度项。
- $c_I$：积分中的位置误差系数，代码固定取 0.5 s⁻¹；与普通位置环的 `position_integral_c1: 0.35` 不同。
- $e_{v,i}^{k}$：本周期速度误差，单位 m/s；来源同第 8.1 节。
- $e_{p,i}^{k}$：本周期位置误差，单位 m；来源同第 8.1 节。

$$a_{PID,i}=\mathcal C_{a_{\max}}\!\left(\lambda p_i-K_{It,i}I_{t,i}^{k+1}\right).$$

- $a_{PID,i}$：包含运输积分后的限幅反馈加速度，单位 m/s²；在线计算。
- $K_{It,i}$：三机均取 $\operatorname{diag}(2.4,2.4,2.4)$ s⁻²，由 `independent_integral_gain: [0.8,0.8,0.8]` 乘 `transport_position_pid_integral_gain_scale: 3.0`。
- $I_{t,i}^{k+1}$：本周期更新后的积分状态，来源为上一公式；冻结时保留原状态。
- $\lambda$：保持倍率，张力渐增 2.0、稳态 1.5；只乘 P/D，不再额外乘积分项。
- $p_i$：第 8.2 节定义的 PD 加速度。
- $a_{\max}$：逐轴反馈限幅，当前为 $[6.0,6.0,8.0]^T$ m/s²。

学习条件还包括参考建立完成、负载和飞机反馈新鲜、EKF/滤波导数有效、
各绳带符号张力为正、上一周期主机输出未限幅，且未进入下降、故障或验证截止。
当前参考建立时间为接管后约 17 s；健康的动态参考允许 native I 学习。
普通单机位置积分与运输积分使用不同状态，运输阶段不重复更新普通位置积分。

### 8.4 新增积分力的幅值和变化率限制

$$b_i^{\mathrm{req}}=m_i(a_{PID,i}-a_{PD,i}).$$

- $b_i^{\mathrm{req}}$：运输积分请求增加的三维反馈力，单位 N；在线计算。
- $m_3$：CF3 质量，取 0.0434 kg，对应 CF3 的 `mass_kg`。
- $m_4$：CF4 质量，取 0.0440 kg。
- $m_5$：CF5 质量，取 0.0424 kg。
- $a_{PID,i}$：第 8.3 节的含积分反馈加速度，单位 m/s²。
- $a_{PD,i}$：第 8.2 节的纯 PD 反馈加速度，单位 m/s²。

$$\mathcal B_r(y)=y\min\!\left(1,\frac{r}{\max(\|y\|,\varepsilon_b)}\right).$$

- $\mathcal B_r$：向量范数限幅算子，保持向量方向，使输出范数不超过 $r$。
- $y$：待限制的三维力或力增量，单位 N；由当前控制计算得到。
- $r$：该次限制使用的非负范数上限，单位 N；下两式分别取 $B_I$ 和 $\rho_I\Delta t_k$。
- $\varepsilon_b$：分母数值保护量，代码取 $10^{-30}$ N。

$$\bar b_i=\mathcal B_{B_I}(b_i^{\mathrm{req}}).$$

- $\bar b_i$：只经过幅值限制的积分力，单位 N；在线计算。
- $B_I$：三机均取 0.06 N，对应 `transport_position_pid_force_limit_n`；限制的是三维范数，不是每轴各 0.06 N。
- $b_i^{\mathrm{req}}$：上一请求力公式的输出，单位 N。
- $\mathcal B_{B_I}$：上述范数限幅算子，以 0.06 N 为半径。

$$b_i^{k+1}=b_i^k+\mathcal B_{\rho_I\Delta t_k}(\bar b_i-b_i^k).$$

- $b_i^{k+1}$：本周期实际增加的运输积分力，单位 N；作为持久状态保存。
- $b_i^k$：上一周期实际积分力，单位 N；初始化为 $[0,0,0]^T$。
- $\bar b_i$：当前幅值受限的目标积分力，单位 N。
- $\rho_I$：三机均取 0.004 N/s，对应 `transport_position_pid_force_rate_limit_nps`。
- $\Delta t_k$：本周期步长，名义 70 Hz 下每步最大力增量约 $5.7143\times10^{-5}$ N。
- $\mathcal B_{\rho_I\Delta t_k}$：对积分力增量作范数限制；显式缓存保持或重复 tick 时增量预算取零。

先裁剪加速度，再取 PID 与 PD 的差，因此发生加速度饱和时，
请求积分力一般不等于直接计算的负质量乘积分增益乘积分状态。
冻结积分状态和冻结实际力是两种不同操作；缓存保持显式冻结后者。

### 8.5 实际运输位置反馈力与卸载

$$F_{\mathrm{fb},i}=m_i a_{PD,i}+w_u b_i.$$

- $F_{\mathrm{fb},i}$：当前 `native_position_pid` 提供的位置反馈力，单位 N；加入第 10 节的协同基础力。
- $m_i$：飞机质量，CF3/CF4/CF5 分别为 0.0434/0.0440/0.0424 kg。
- $a_{PD,i}$：限幅后的 PD 加速度，单位 m/s²；当前保持倍率已包含在内。
- $w_u$：积分力卸载权重，无量纲；空中取 1，负载触地确认后在 2.0 s 内按五次曲线降到 0。
- $b_i$：第 8.4 节的实际受限积分力，单位 N；空中范数不超过 0.06 N。

$$\frac{\mathrm d}{\mathrm dt}(w_u b_i)=w_u\dot b_i+\dot w_u b_i.$$

- $\dot b_i$：实际受限积分力变化率，单位 N/s；当前有限预算下由本周期实际力增量除以实际步长计算。
- $\dot w_u$：卸载权重变化率，单位 s⁻¹；空中取 0，卸载时取 2.0 s 五次曲线的解析导数。
- $w_u$：当前卸载权重，空中为 1、卸载完成为 0。
- $b_i$：本周期实际受限积分力，单位 N；不是无限制的请求力。

PD 力导数另包含参考加速度误差和保持倍率的变化率。
参考加速度的动力学前馈已由基础力及第 7 节构型补偿负责，不在位置 PID 中重复添加。

### 8.6 保留的差分积分分支：当前未启用

$$\widetilde e_i=e_{p,i}-\frac13\sum_{j\in\mathcal V}e_{p,j}.$$

- $\widetilde e_i$：去除三机共同平移后的相对构型误差，单位 m；在线计算。
- $e_{p,i}$：本机位置误差，定义同第 8.1 节。
- $e_{p,j}$：第 $j$ 架飞机的位置误差，单位 m；$j$ 遍历集合 $\mathcal V$。
- $\mathcal V$：当前飞机编号集合，取 $\{3,4,5\}$，对应 CF3、CF4、CF5。
- $1/3$：三机算术平均系数，当前飞机数量为 3。

$$\dot b_{d,i}^{\mathrm{req}}=-k_d\widetilde e_i.$$

- $\dot b_{d,i}^{\mathrm{req}}$：差分力补偿的请求更新率，单位 N/s；仅对照分支使用。
- $k_d$：取 0.05 N/(m·s)，对应 `transport_hold_integral_gain_npm_s`。
- $\widetilde e_i$：上式的相对构型位置误差，单位 m。

该分支共同缩放三机更新，保持三机补偿力之和为零。
每机力范数上限取 0.04 N，变化率范数上限取 0.002 N/s。
当前 `transport_feedback_controller: native_position_pid`，所以即使
`transport_hold_integral_enabled: true`，也不叠加这份差分力或第二份 PD。

### 8.7 竖直支撑残差的初始化

$$b_{z,i}^{\mathrm{req}}(0)=\max(F_{i,h,z}-u_{i,z}(0),0).$$

- $b_{z,i}^{\mathrm{req}}(0)$：接管时希望继承的竖直支撑残差，单位 N；由接管前实际命令在线计算，不是固定负载重量。
- $F_{i,h,z}$：预抬结束时冻结的本机期望合力 Z 分量，单位 N；来自 `payload_handoff_forces`。
- $u_{i,z}(0)$：接管初次计算的协同基础力 Z 分量，单位 N；由第 5 节实时模型计算。
- $0$：残差下限，取 0 N，初始化不额外施加向下的支撑残差。

$$H_i=\max(T_{\max,i}-u_{i,z}(0),0).$$

- $H_i$：初始化可用的竖直命令余量，单位 N；由当前基础力与推力上限计算。
- $T_{\max,i}$：三机主机命令上限均取 1.00 N，对应 `max_command_thrust_newton`。
- $u_{i,z}(0)$：接管初次协同基础力 Z 分量，单位 N；不固定为三机相同值。
- $0$：可用余量下限，取 0 N。

$$L_{z,i}=\min\!\left(\max(B_z,b_{z,i}^{\mathrm{req}}(0)),H_i\right).$$

- $L_{z,i}$：本次运行的支撑残差上限，单位 N；逐机在线冻结，不一定相同。
- $B_z$：普通支撑残差预算，取 0.25 N，对应 `transport_support_trim_limit_n`。
- $b_{z,i}^{\mathrm{req}}(0)$：初始化请求残差，单位 N；较大时允许在可用余量内完整继承。
- $H_i$：初始化可用竖直余量，单位 N；对提高后的预算仍有约束。

$$b_{z,i}(0)=\min(b_{z,i}^{\mathrm{req}}(0),L_{z,i}).$$

- $b_{z,i}(0)$：实际接受的初始竖直支撑残差，单位 N；在线计算并保存。
- $b_{z,i}^{\mathrm{req}}(0)$：请求初值，单位 N。
- $L_{z,i}$：上式得到的本次运行上限，单位 N。

### 8.8 竖直支撑残差的后续更新

$$\dot b_{z,i}^{\mathrm{req}}=\operatorname{clip}_{[-\rho_z,\rho_z]}\!\left(k_z(x_{0d,z}-x_{0,z})\right).$$

- $\dot b_{z,i}^{\mathrm{req}}$：本周期请求的支撑力变化率，单位 N/s；在线计算。
- $\rho_z$：取 0.03 N/s，对应 `transport_support_trim_rate_nps`。
- $k_z$：取 0.20 N/(m·s)，对应 `transport_support_integral_gain_npm_s`。
- $x_{0d,z}$：负载几何中心参考高度，单位 m；抬升后取 0.60 m，下降中随参考变化。
- $x_{0,z}$：实测负载几何中心高度，单位 m；由 NOKOV 上表面位置和姿态换算。

$$b_{z,i}^{k+1}=\operatorname{clip}_{[0,L_{z,i}]}\!\left(b_{z,i}^{k}+\Delta t_k\dot b_{z,i}^{\mathrm{req}}\right).$$

- $b_{z,i}^{k+1}$：本周期实际支撑残差，单位 N；加入本机世界 Z 方向合力。
- $b_{z,i}^{k}$：上一周期支撑残差，单位 N；初值来自第 8.7 节。
- $\Delta t_k$：该支撑更新实际使用的机群控制步长，单位 s；名义 70 Hz 时约 0.014286 s。
- $\dot b_{z,i}^{\mathrm{req}}$：受变化率约束后的更新请求，单位 N/s。
- $L_{z,i}$：初始化时冻结的本机上限，单位 N。

上一周期总推力达到上限减 0.01 N 时，正向更新率改为零，负向消退仍允许。
负载触地确认后的 2.0 s 卸载阶段另按冻结残差作五次消退，最终复位。
此支撑项不要求零和，也不是再次给三机叠加一份负载重力。
实现见 [slung_transport_guard.py](../ros_ws/src/crazyswarm/scripts/slung_transport_guard.py)。

## 9. 姿态到构型的慢修正与下降衔接

`transport_attitude_trim_enabled` 是总开关，仅支持 `reference_cascade`。
`transport_attitude_trim_dynamic_enabled` 默认 `false`，选择静态 helper；设为 `true` 选择
`SlungDynamicAttitudeReferenceTrim`，不改负载目标姿态、核心控制器或机载模型。
两者共用当前配置 $K_\eta=\operatorname{diag}(0.05,0.10,0.01)$ s⁻¹、段时长 $T_\eta=1$ s，
每机学习偏置上限 $L_x=0.010$ m、段内速度上限 $L_v=0.0015$ m/s、加速度上限 $L_a=0.002$ m/s²。
实现见 [slung_attitude_trim.py](../ros_ws/src/crazyswarm/scripts/slung_attitude_trim.py)。

### 9.1 默认静态模式

中心化挂点 $\bar\rho_i=\rho_i-\frac13\sum_j\rho_j$；在固定基准目标坐标系中：

$$e_\eta=\log(R_g^TR_0)^\vee,\quad
\eta^+=\eta+K_\eta e_\eta T_\eta,\quad
\delta x_i=-R_g(\eta\times\bar\rho_i),\quad \sum_i\delta x_i=0.$$

圆周/八字运动时先用 $Q^TR_0$ 得到基准帧测量，冻结学习并按第 7.1 节旋转已有段的完整 PVA。
圆锥运动期间冻结学习并保留已有世界系参考偏置，避免将其随倾斜旋转而引入额外运动。

### 9.2 显式启用的动态模式

动态模式直接累计每架飞机的世界系偏置端点 $p_{i,k}$，不要求 $R_d$ 固定。
在旧段结束后的下一次健康更新，使用当时完整目标姿态与测量姿态计算：

$$e_k=\operatorname{Log}(R_{d,k}^TR_{0,k})^\vee,\qquad
g_k=K_\eta e_kT_\eta,\qquad
d_{i,k}=-R_{d,k}(g_k\times\bar\rho_i).$$

定义 $\|p\|_{\max}=\max_i\|p_i\|_2$。先把请求端点整体投影，再对整组步长统一缩放：

$$p_i^*=p_{i,k}+d_{i,k},\qquad
\beta=\min\left(1,{L_x\over\|p^*\|_{\max}}\right),\qquad
q_i=\beta p_i^*,\qquad \Delta_i=q_i-p_{i,k},$$

$$\alpha=\min\left(1,{L_vT_\eta\over1.875\|\Delta\|_{\max}},
{L_aT_\eta^2\over(10/\sqrt3)\|\Delta\|_{\max}}\right),\qquad
p_{i,k+1}=p_{i,k}+\alpha\Delta_i.$$

分母为零的比值视为不限制，相应系数取 1。所有飞机共用 $\alpha,\beta$，因此中心化增量、
整体投影和插值都保留 $\sum_i p_i=0$，不引入共同平移。段内令 $u=(t-t_k)/T_\eta$：

$$\delta x_i=p_{i,k}+s(u)(p_{i,k+1}-p_{i,k}),\quad
\delta v_i={s'(u)\over T_\eta}(p_{i,k+1}-p_{i,k}),\quad
\delta a_i={s''(u)\over T_\eta^2}(p_{i,k+1}-p_{i,k}),$$

$$s(u)=10u^3-15u^4+6u^5,\quad
\max_{[0,1]}s'=1.875,\quad \max_{[0,1]}|s''|=10/\sqrt3.$$

两端点均在位置范数球内，段内是它们的凸组合；上述精确极值给出整个段的速度、加速度范数界。
每段两端速度和加速度为零，只有旧段完成后才开始新段，所以偏置参考为 C2。
冻结时当前段继续执行，之后保持世界系端点；同一时间戳只返回缓存，时钟缺口也只启动一段，不追赶漏掉的积分步。
限幅仅约束学习偏置自身，名义轨迹与下降残差桥各有自己的边界。

段内 $R_d$ 的变化不旋转既有端点，圆周/八字也不对动态偏置额外乘 yaw 旋转。
因此输出已经是完整世界系 PVA，不需要也不能补加一个未定义的旋转基底导数。
从零偏置出发，精确跟踪 $R_0=R_d$ 或零增益都不会因为目标运动而产生偏置。
动态参考下的持续滞后也可能被慢积分学习；有界、低带宽修正与 C2 参考不能证明闭环稳定、不振荡或误差必然下降，仍需相同条件的关闭/静态/动态回放及几何、延迟和机体差异对照。

`transport_attitude_trim_error_*` 记录瞬时目标系 $e_k$；动态模式下兼容字段
`transport_attitude_trim_integral_*` 改为世界系小角累计诊断 $\eta_{w,k}$，更新使用同一投影与步长系数：

$$\eta_{w,k+1}=(1-\alpha)\eta_{w,k}
+\alpha\beta(\eta_{w,k}+R_{d,k}g_k).$$

不同历史增量作用时的挂点方向不同，故这个三维诊断不是能从当前 $R_d$ 重建 $p_{i,k}$ 的单一旋转，
也不参与后续 offset 反馈或由位置上限独立保证有界。真实学习状态是世界系端点；执行值记录在每机
`transport_attitude_trim_position_*`、`transport_attitude_trim_velocity_*`、`transport_attitude_trim_acceleration_*`。

### 9.3 动态 trim 的反馈质量门控

仅动态模式可启用 `transport_attitude_trim_during_lift_enabled`，省略默认false，当前实飞配置true。
关闭时等待 $h\ge\max(T_R,T_{\rm ramp}+T_{\rm lift})$，当前17s；
开启时只等待 $h\ge T_{\rm ramp}$ 且所有车辆 $\beta_i\ge1-10^{-9}$，当前最早5s。
随后用当时完整 $R_d(h)$ 更新，不能将最终水平目标代替过渡姿态。
该选项只提前trim，native位置I与差分力积分不受影响。
因为第一次五次段在 $u=0$ 输出零PVA，后续仍为有界C2段，不会给参考增加首次启用跳变。

动态学习继续服从第 8 节的反馈新鲜度、导数有效、正张力和输出未限幅等条件，并增加：

| 参数 | 当前值 | 含义 |
|---|---|---|
| `transport_attitude_trim_feedback_max_rate_rps` | 0.50 rad/s | 观测器输出的负载机体系角速度范数上限 |
| `transport_attitude_trim_feedback_pose_jump_deg` | 1.0° | 相邻源位姿旋转步长的额外裕量 |
| `transport_attitude_trim_feedback_recovery_s` | 0.50 s | 异常后重新学习前的恢复时间 |

源时间为 $t_s$ 时，位姿步长为
$\vartheta_s=\|\operatorname{Log}(R_{s,\rm previous}^TR_s)^\vee\|$，
允许值为 $\vartheta_{\max}=\vartheta_{\rm margin}+\Omega_{\max}\max(\Delta t_s,0)$；角度统一换算后比较。
倒序、非有限反馈、过大的原始位姿步长或角速度都会暂停新段；拒绝的位姿跳变不替换上次接受的位姿。
门控使用源样本时间，同一控制时刻的重复质量判断复用结果，不把重复源帧误当作零间隔异常。
恢复等待期间也只冻结新学习，已有 C2 段继续执行。

这些检查只管理 trim 学习，不改负载核心环实际采用的姿态/角速度反馈，也不修改观测器的估计或裁剪。
CSV 的 `transport_attitude_trim_dynamic_enabled`、`transport_attitude_trim_feedback_healthy`、
`transport_attitude_trim_feedback_quality_reason`、`transport_attitude_trim_body_rate_norm_rps`、
`transport_attitude_trim_pose_step_deg`、`transport_attitude_trim_pose_step_limit_deg` 分别给出所选模式、
额外质量门控结果及原始诊断量；质量健康仍不代表其他学习门控全部放行，需结合 `transport_attitude_trim_freeze_reason`。

### 9.4 下降衔接

下降从当前实测负载 PVA 规划，冻结实测绳向/距离；飞机参考另用残差桥连接上一完整命令。
桥同时保持两套飞机参考的 PVA，逐渐去掉残差；该桥不受学习 trim 的 1 cm 幅值预算约束。
圆锥模式在姿态恢复时改用第 7.2 节的重力平衡绳向及其解析导数，仍使用测量距离与同一残差桥。
启动中断时一般姿态制动采用 $R(t)=R_s\exp(\hat\xi(t))$、
$\Omega=J_r(\xi)\dot\xi$、$\dot\Omega=J_r\ddot\xi+\dot J_r\dot\xi$，
保持初始旋转/角速度/角加速度后平滑停转；正常平面运动中断使用连续 yaw 制动。
圆锥参考中断时先连续制动实测平动，在停止的位置保持，完整姿态制动与恢复水平后才开始下降；
同时用旋转箱体半尺寸的最低高度及角速度界检查地面间隙，不可行时进入原支撑保持/重规划。
正常圆锥结束、参考已水平静止时直接使用原下降规划，不增加等待。
落地需实测低高度、窗口内稳定和低竖直速度共同确认，超时本身不是触地确认。

## 10. 合力限制、滤波与最终 CTBR

本节说明“各机几何控制器”的最后计算步骤：三维合力先决定飞机期望姿态，
姿态误差再决定机体系角速度命令，同时将合力投影到实际机体推力轴得到总推力。
对应 [ctbr_geometric_controller.py](../ros_ws/src/crazyswarm/scripts/ctbr_geometric_controller.py)
及 [ctbr_transport_vehicle_controller.py](../ros_ws/src/crazyswarm/scripts/ctbr_transport_vehicle_controller.py)。

### 10.1 构型运动补偿与原始合力

$$\Delta u_i=m_i\Delta a_i.$$

- $\Delta u_i$：构型运动产生的附加惯性补偿力，单位 N；在线计算。
- $m_i$：飞机质量，CF3/CF4/CF5 分别取 0.0434/0.0440/0.0424 kg。
- $\Delta a_i$：第 7 节的完整构型加速度补偿，单位 m/s²；静态名义参考取零，运动或构型平滑修正时按参考计算。

$$F_{i,\mathrm{raw}}=(1-\beta)F_{i,h}+\beta\left(u_i+\Delta u_i+b_{z,i}e_3+F_{\mathrm{fb},i}\right).$$

- $F_{i,\mathrm{raw}}$：限幅前的世界系期望合力，单位 N；在线计算。
- $\beta$：接管权重，无量纲；从 0 平滑增加到 1，当前渐增时间为 5.0 s。
- $F_{i,h}$：接管前冻结的本机期望合力，单位 N；本次任务取预抬结束时的实际命令快照，不是固定常数。
- $u_i$：第 5 节的协同基础力，单位 N；当前 `reference_cascade` 包含分配绳力、重力和挂点运动等模型项。
- $\Delta u_i$：上一公式给出的构型附加补偿力，单位 N；按第 7 节避免重复补偿挂点和向心运动。
- $b_{z,i}$：第 8.7、8.8 节的竖直支撑残差，单位 N；在线状态，普通预算为 0.25 N。
- $e_3$：世界 Z 轴单位向量，固定取 $[0,0,1]^T$。
- $F_{\mathrm{fb},i}$：第 8.5 节的 native 位置反馈力，单位 N；包含 PD 和有界运输 I。

代码没有接管力快照时，$F_{i,h}$ 按原独立控制关系回退计算。
计算合力导数时，接管权重、保持倍率、卸载权重都保留乘积导数项。

$$\beta=S_5\!\left(\operatorname{clip}_{[0,1]}\!\left(\frac{h}{T_r}\right)\right).$$

- $\beta$：本次接管权重；h=0 时为 0，h≥5 s 时为 1。
- $h$：从进入 TAUT_RAMP 开始计的实际单调时钟时间，单位 s；在线计算。
- $T_r$：张力渐增时间，取 5.0 s，对应 `tension_ramp_s`。
- $S_5$：归一化五次平滑函数，其表达式为下一公式。

$$S_5(s)=10s^3-15s^4+6s^5.$$

- $s$：归一化进度，范围为 $[0,1]$；在上式中取裁剪后的 $h/T_r$。
- $S_5(s)$：无量纲平滑权重；起终点为 0、1，起终点一阶与二阶导数均为零。
- $10,-15,6$：五次平滑曲线的固定系数，不是可调控制增益。

### 10.2 合力与倾角限制

$$F_{i,z}^{\mathrm{lim}}=\operatorname{clip}_{[0,T_{\max,i}]}(F_{i,z}^{\mathrm{raw}}).$$

- $F_{i,z}^{\mathrm{lim}}$：受限后的世界 Z 方向合力，单位 N；在线计算。
- $F_{i,z}^{\mathrm{raw}}$：原始合力 Z 分量，单位 N；取第 10.1 节的输出。
- $T_{\max,i}$：三机命令推力上限均为 1.00 N，对应 `max_command_thrust_newton`。
- $0$：竖直命令力下限，取 0 N。

$$F_{i,xy}^{\mathrm{lim}}=F_{i,xy}^{\mathrm{raw}}\min\!\left(1,\frac{F_{i,z}^{\mathrm{lim}}\tan\theta_{\max,i}}{\|F_{i,xy}^{\mathrm{raw}}\|}\right).$$

- $F_{i,xy}^{\mathrm{lim}}$：受限后的世界 XY 水平合力向量，单位 N；在线计算。
- $F_{i,xy}^{\mathrm{raw}}$：原始水平合力向量，单位 N；为零时直接保留零向量，不作除法。
- $F_{i,z}^{\mathrm{lim}}$：上一公式的受限竖直合力，单位 N；其大小决定当前水平力预算。
- $\theta_{\max,i}$：飞机倾角上限；当前接管初段取 12°，在 12 s 抬升段内平滑降至三机均为 10° 的稳态值。
- $\tan\theta_{\max,i}$：竖直力与允许水平力之间的比例；计算时角度转换为 rad。

代码对极小水平模长采用数值保护，低于 $10^{-12}$ N 且进入受限分支时直接输出零水平力。
限制后的三维向量由水平分量和竖直分量组合为 $F_i^{\mathrm{lim}}$。
这一步限制竖直分量与倾角；完整合力模长不一定小于 1.00 N，最终总推力仍单独裁剪。
下降阶段沿用切入时冻结的倾角保护配置，不能直接套用上述抬升过渡时间。

### 10.3 运输合力低通滤波

$$F_{i,f}^{k+1}=F_{i,f}^{k}+\frac{\Delta t_k}{\tau_F+\Delta t_k}\left(F_i^{\mathrm{lim}}-F_{i,f}^{k}\right).$$

- $F_{i,f}^{k+1}$：本周期用于构造期望姿态和推力的滤波合力，单位 N；在线状态。
- $F_{i,f}^{k}$：上一周期滤波合力，单位 N；复位时直接继承当前限幅合力，不从零缓慢启动。
- $F_i^{\mathrm{lim}}$：第 10.2 节限制后的三维合力，单位 N。
- $\Delta t_k$：本周期控制步长，单位 s；代码裁剪到 $[0.001,0.05]$。
- $\tau_F$：三机均取 0.005 s，对应 `transport_force_filter_tau_s`。

$$\dot F_{i,f}=\frac{F_{i,f}^{k+1}-F_{i,f}^{k}}{\Delta t_k}.$$

- $\dot F_{i,f}$：与实际滤波力一致的离散力导数，单位 N/s；在线计算。
- $F_{i,f}^{k+1}$：本周期实际滤波力，单位 N。
- $F_{i,f}^{k}$：上一周期实际滤波力，单位 N。
- $\Delta t_k$：当前控制步长，单位 s；不是固定写死的 1/70。

滤波复位当周期的力导数取零。
下文为简洁记 $F_{i,f}=F_{i,f}^{k+1}$，始终使用本周期实际滤波输出。

### 10.4 从三维合力构造期望姿态

$$b_{3c,i}=\frac{F_{i,f}}{\|F_{i,f}\|}.$$

- $b_{3c,i}$：飞机期望机体 Z 轴在世界系中的方向，为单位向量；在线计算。
- $F_{i,f}$：第 10.3 节的当前滤波合力，单位 N。
- $\|F_{i,f}\|$：当前合力模长，单位 N；低于 $10^{-7}$ N 时使用上一有效 Z 轴，无历史时取 $[0,0,1]^T$。

$$b_{1r,i}=\begin{bmatrix}\cos\psi_{i,d}&\sin\psi_{i,d}&0\end{bmatrix}^{T}.$$

- $b_{1r,i}$：用于确定期望航向的世界水平参考向量，模长为 1；在线计算。
- $\psi_{i,d}$：本机航向参考，单位 rad；来自飞机任务的 `yaw`，当前 hover 任务保存各机起飞航向，不是负载的 15° 锥角。

$$\bar b_{1c,i}=(I_3-b_{3c,i}b_{3c,i}^{T})b_{1r,i}.$$

- $\bar b_{1c,i}$：航向参考在垂直于期望 Z 轴平面上的投影，尚未归一化；在线计算。
- $I_3$：三阶单位矩阵，固定取 $\operatorname{diag}(1,1,1)$。
- $b_{3c,i}$：当前期望机体 Z 轴，来源为本节第一条公式。
- $b_{1r,i}$：上一公式的水平航向参考。

$$b_{1c,i}=\frac{\bar b_{1c,i}}{\|\bar b_{1c,i}\|}.$$

- $b_{1c,i}$：归一化的飞机期望 X 轴，为单位向量；在线计算。
- $\bar b_{1c,i}$：上一公式的航向投影。
- $\|\bar b_{1c,i}\|$：投影模长；小于 $10^{-6}$ 时使用上一有效航向，无历史时回退到世界 $[1,0,0]^T$，随后仍执行数值归一化与正交化。

$$b_{2c,i}=b_{3c,i}\times b_{1c,i}.$$

- $b_{2c,i}$：飞机期望 Y 轴，为单位向量；在线计算。
- $b_{3c,i}$：飞机期望 Z 轴，方向与当前合力一致。
- $b_{1c,i}$：上一公式得到的期望 X 轴。
- $\times$：向量叉乘，顺序保证右手坐标系。

$$R_{ic}=\begin{bmatrix}b_{1c,i}&b_{2c,i}&b_{3c,i}\end{bmatrix}.$$

- $R_{ic}$：把期望机体系变换到世界系的旋转矩阵，维度 3×3；在线计算，不是固定欧拉角。
- $b_{1c,i}$：期望 X 轴，作为第一列。
- $b_{2c,i}$：期望 Y 轴，作为第二列。
- $b_{3c,i}$：期望 Z 轴，作为第三列。

代码会再次归一化 Y 轴、以 Y/Z 轴重算 X 轴，并将矩阵投影到 SO(3)，消除数值误差。

### 10.5 几何姿态误差与运输角速度命令

$$e_{R,i}=\frac12\left(R_{ic}^{T}R_i-R_i^{T}R_{ic}\right)^{\vee}.$$

- $e_{R,i}$：机体系表达的几何姿态误差，无量纲，接近目标时可近似理解为 rad 角误差；在线计算，目标为零向量。
- $R_{ic}$：本周期期望旋转矩阵，来自第 10.4 节。
- $R_i$：本机实际旋转矩阵，将实际机体系变换到世界系；取 NOKOV 姿态。
- $(\cdot)^{\vee}$：反对称矩阵到三维向量的逆映射，定义见第 2 节。
- $1/2$：SO(3) 几何姿态误差的固定系数。

$$\Omega_{ic,k}=\frac{\operatorname{Log}(R_{ic,k-1}^{T}R_{ic,k})^{\vee}}{t_k-t_{k-1}}.$$

- $\Omega_{ic,k}$：通过 `log_difference` 得到的期望角速度前馈，单位 rad/s。
- $R_{ic,k}$：由本周期最终限幅、滤波合力及飞机航向构造的期望旋转矩阵。
- $R_{ic,k-1}$：上一有效运输控制周期的期望旋转矩阵；不是负载实测姿态矩阵。
- $t_k-t_{k-1}$：相邻实际控制周期的时间差，单位 s；使用节点单调控制时间，不使用缓存负载时间或裁剪后的积分步长。
- $\operatorname{Log}(\cdot)^{\vee}$：SO(3) 主值旋转向量；这里计算相邻姿态的有限步长角速度。

$$\Omega_{i,\mathrm{tr}}=R_i^{T}R_{ic}\Omega_{ic}-K_{R,i}^{\mathrm{tr}}e_{R,i}.$$

- $\Omega_{i,\mathrm{tr}}$：运输分支要求的三轴机体系角速度，单位 rad/s；是角速度命令，不是力矩。
- $K_{R,3}^{\mathrm{tr}}$：CF3 主机姿态增益，取 $\operatorname{diag}(12,12,6)$ s⁻¹，对应 `transport_attitude_gain`。
- $K_{R,4}^{\mathrm{tr}}$：CF4 主机姿态增益，取 $\operatorname{diag}(12,12,6)$ s⁻¹。
- $K_{R,5}^{\mathrm{tr}}$：CF5 主机姿态增益，取 $\operatorname{diag}(8,8,6)$ s⁻¹。
- $R_i^{T}R_{ic}\Omega_{ic}$：将期望机体系角速度转换到当前实际机体系，单位 rad/s；前馈系数为 1。
- $e_{R,i}$：上一公式的几何姿态误差；不直接使用欧拉角相减代替。

2026-11-17 先接入 `transport_attitude_mode: feedforward_p`、
`omega_c_method: log_difference`、`use_body_rate_feedforward: true`。
此模式只有前馈和姿态 P，不叠加飞机姿态 I 或 `transport_rate_gain` 的实测角速度阻尼。
第 8 节的位置 PID 和机载 Mellinger 仍然存在。

随后增加 `feedforward_pi`，当前实飞配置选择此模式。它在上述前馈＋P 后加入运输专用有界积分：

$$\dot b_{R,i}^{\mathrm{req}}=\operatorname{clip}_{\pm r_{R,I}}(-K_{R,I}e_{R,i}).$$

- $b_{R,i}$：飞机机体系上的积分角速度补偿，单位 rad/s；初值为零，不复用普通姿态积分状态。
- $K_{R,I}$：三机均为 $\operatorname{diag}(0.4,0.4,0.2)$ s⁻²，对应 `transport_attitude_integral_gain`。
- $e_{R,i}$：飞机自身几何姿态误差；不是负载姿态误差或实际角速度误差。
- $r_{R,I}$：逐轴取 $[0.02,0.02,0.01]$ rad/s²，对应 `transport_attitude_integral_rate_limit_rps2`。

$$b_{R,i}^{k+1}=\operatorname{clip}_{\pm L_{R,I}}(b_{R,i}^{k}+\Delta t\dot b_{R,i}^{\mathrm{req}}).$$

- $b_{R,i}^{k+1}$：本周期输出的有界积分角速度补偿，单位 rad/s。
- $L_{R,I}$：三机均为 $[0.15,0.15,0.10]$ rad/s，对应 `transport_attitude_integral_limit_rps`；为逐轴上限。
- $\Delta t$：积分步长不大于 0.05 s，也不大于本周期实际控制时间增量；重复、倒序或超过 0.10 s 的间断不学习。
- $\dot b_{R,i}^{\mathrm{req}}$：上一公式的更新请求；冻结期间不推进积分状态。

$$\Omega_i^{\mathrm{raw}}=(1-\beta)\Omega_{i,\mathrm{ind}}+\beta(\Omega_{i,\mathrm{tr}}+b_{R,i}).$$

- $\Omega_i^{\mathrm{raw}}$：最终逐轴限幅之前的角速度命令，单位 rad/s。
- $\beta$：原接管混合系数；新积分在接管和抬升参考建立完成前冻结。
- $\Omega_{i,\mathrm{tr}}$：上文 `log_difference` 前馈＋姿态 P 的输出。
- $b_{R,i}$：独立保存的运输姿态积分补偿；不改变期望合力和位置 PID 状态。

学习要求公共负载/飞机反馈健康、未处于缓存或下降、参考建立完成、上一周期主机未限幅，
且飞机实际与期望姿态的 SO(3) 夹角小于 `transport_attitude_integral_gate_deg: 15.0`。
本周期力、推力或保持反馈限幅也冻结更新；候选 I 增量若继续加深角速度某轴饱和则拒绝该轴增量。
本地逐轴检查允许反向消退，但公共门控因上一周期饱和而冻结时仍优先保持。
暂停和协同下降保持已有补偿；退出运输、紧急复位或新任务清零。
位置积分、此处飞机姿态积分、负载姿态 trim 是三个独立机制。

CSV 的 `transport_attitude_integral_rate_*` 为保存的 I 补偿，
`transport_attitude_integral_applied_rate_*` 为乘接管系数后的实际附加量；
`transport_attitude_integral_learning` / `freeze_reason` 记录学习准入，
`limit_active` / `antiwindup` 记录补偿上限和逐轴防饱和。
把三机模式改回 `feedforward_p` 就关闭并清除此运输姿态 I。

第一运输帧、完整复位、独立模式切回运输、力/航向退化、显式滤波重置或超过 0.10 s 的控制间断，
前馈取零并重新建立历史。同控制时间戳重复计算不增加差分样本。
普通短缓存期间仍根据实际发送时刻及最终期望姿态差分：如果最终姿态不变，前馈为零；
如果飞机反馈或既有 trim 段使最终期望姿态变化，就对该变化计算前馈。

保留 `transport_attitude_mode: rate_damping` 可回到旧的姿态 P 加角速度阻尼：

$$\Omega_{i,\mathrm{tr}}^{\mathrm{old}}=-K_{R,i}^{\mathrm{tr}}e_{R,i}-K_{\Omega,i}^{\mathrm{tr}}\Omega_i.$$

- $\Omega_{i,\mathrm{tr}}^{\mathrm{old}}$：仅旧模式使用的角速度命令，单位 rad/s，期望角速度前馈为零。
- $K_{R,i}^{\mathrm{tr}}$：与上文相同的运输姿态 P 增益。
- $K_{\Omega,i}^{\mathrm{tr}}$：原三机值为 $\operatorname{diag}(0.25,0.25,0.125)$，无量纲；对应 `transport_rate_gain`，当前新模式不使用。
- $\Omega_i$：实际机体系角速度，单位 rad/s；来自动捕姿态差分滤波。

### 10.6 接管角速度混合与逐轴限幅

$$\Omega_{i,\mathrm{raw}}=(1-\beta)\Omega_{i,\mathrm{ind}}+\beta\Omega_{i,\mathrm{tr}}.$$

- $\Omega_{i,\mathrm{raw}}$：限幅前的机体系角速度命令，单位 rad/s；在线计算。
- $\beta$：第 10.1 节接管权重，0–5 s 从 0 增加到 1，之后为 1。
- $\Omega_{i,\mathrm{ind}}$：独立分支的角速度命令，单位 rad/s；当前有接管力快照时，按下一公式计算。
- $\Omega_{i,\mathrm{tr}}$：第 10.5 节的运输角速度命令，单位 rad/s。

$$\Omega_{i,\mathrm{ind}}=-K_{R,i}^{\mathrm{ind}}e_{R,i}.$$

- $\Omega_{i,\mathrm{ind}}$：当前接管路径使用的独立姿态命令，单位 rad/s；在线计算。
- $K_{R,i}^{\mathrm{ind}}$：三机均取 $\operatorname{diag}(12,12,6)$ s⁻¹，对应 `independent_attitude_gain`。
- $e_{R,i}$：相对于当前混合合力生成姿态的几何误差，不是另一个旧姿态目标的误差。

$$\Omega_i^{\mathrm{cmd}}=\mathcal C_{\Omega_{\max}}(\Omega_{i,\mathrm{raw}}).$$

- $\Omega_i^{\mathrm{cmd}}$：最终输出的物理机体系角速度，单位 rad/s；依次为滚转、俯仰、偏航角速度。
- $\mathcal C_{\Omega_{\max}}$：逐轴限幅算子，将各轴裁剪到对应的正负角速度上限。
- $\Omega_{\max}$：三机均取 $[3.0,3.0,2.0]^T$ rad/s，对应 `max_body_rate_radps`。
- $\Omega_{i,\mathrm{raw}}$：上一混合公式的输出，单位 rad/s；接管完成后等于运输分支输出。

当前 `transport_body_rate_filter_tau_s: 0.0`，即不额外滤波角速度命令。
新模式的接管独立侧始终使用上述姿态 P，前馈按 $\beta$ 平滑混入，避免重复叠加前馈。
普通非运输几何模式的解析/差分前馈与姿态积分保留，但不是运输分支的输出公式。

### 10.7 总推力投影与最终输出

$$T_i^{\mathrm{cmd}}=\operatorname{clip}_{[0,T_{\max,i}]}\!\left(\max(F_{i,f}^{T}R_i e_3,T_{\min,i})\right).$$

- $T_i^{\mathrm{cmd}}$：整架飞机的总推力命令，单位 N；在线计算，不是单个电机推力。
- $F_{i,f}$：当前世界系滤波合力，单位 N；由第 10.3 节给出。
- $R_i e_3$：飞机实际机体 Z 轴在世界系中的单位方向，来自 NOKOV 实测姿态。
- $e_3$：机体系 Z 轴单位向量，固定取 $[0,0,1]^T$；与 $R_i$ 相乘后才转到世界系。
- $T_{\min,i}$：任务阶段规定的最低推力，当前空中 hover 任务取 0.30 N；下降/停推阶段按任务参考调整，控制计算缺省为 0 N。
- $T_{\max,i}$：三机主机命令推力上限均取 1.00 N；不同于推力映射参考值 1.176798 N。

投影使用实际飞机姿态，因此飞机尚未倾斜到期望方向时，不能直接把合力模长当成总推力发送。
绳张力与这里的飞机总推力是两个不同物理量。
最终主机接口是总推力 N 与三轴机体系角速度 rad/s；
C++ 桥接负责固件推力值、角速度单位/符号及包格式转换，机载 Mellinger 负责执行与电机混控。
主机不直接发送本机完整姿态力矩。

## 11. 仿真机载环与适用范围

[slung_simulation.yaml](../ros_ws/src/crazyswarm/config/slung_simulation.yaml) 仅供离线回放读取，
实飞 launch 不加载该文件。仿真机载环固定 Mellinger、500 Hz；保留 `graefe2026` 与
`legacy_quadratic` 两种电机物理模型，机载 PID/理想一阶内环选择器已移除。
这不移除主机 native 位置 PID、`coupled` 或差分反馈分支。

仿真 Mellinger 使用固件的未乘 $1/2$ 姿态反对称误差及机体系速率误差，
输出有界混控量后进入四个独立电机；其 `mellinger_*` 增益不同于上述主机角速度增益。
Graefe 模型先使转速按 50 ms 时间常数变化，再由多项式得到推力、气动反扭矩和转子惯性项；
旧模型保留平方推力曲线及 12 ms 力滞后。两者都不是主机 5 ms 期望力滤波器。

源码中的限幅、导数回退、张力单边性和任务故障退出都是实现的一部分。
这些公式记录工程控制律，不构成完整自适应扰动估计器或实机稳定性证明。
历史实验、论文逐项比较和当时参数见
[设计对照文档](slung_controller_design_comparison.md)；其中历史默认值不能代替本文和当前配置。
