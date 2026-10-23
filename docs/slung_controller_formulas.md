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
| 负载参考 | 当前 `circle`；轨迹参数与普通飞机 `hover` 分别配置 |
| 圆锥切换配置 | 15°、两圈各 30 s、倾斜/恢复各 5 s，中心固定在世界 `[0,0,0.6]` m |
| 圆锥接管后的时钟 | h20 倾斜，h25/55 分别开始两圈，h85 恢复水平，h100 计划下降 |
| 动态构型补偿 | `transport_attitude_trim_dynamic_enabled: false`，显式启用后使用世界系端点学习 |
| 名义几何 | 负载质量 0.023 kg，尺寸 `[0.08,0.06,0.05]` m，三绳均 0.850 m |
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

均匀箱体 $J_{0,xx}=m_0(b^2+h^2)/12$，其余轴循环置换。
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

保持误差使用 $x_{i,\rm hold},v_{i,\rm hold}$；设
$p_i=-K_{p,i}e_{p,i}-K_{v,i}e_{v,i}$，保持倍率 $\lambda$ 从 2.0 衔接到 1.5。
当前运输 native PID 的专用积分满足：

$$I_{t,i}^+=\operatorname{clip}_{\pm L_I}
[I_{t,i}+\Delta t(w_ve_{v,i}+0.5e_{p,i})],\qquad \|e_{p,i}\|<0.23\ \mathrm{m}.$$

当前 $w_v=1.0$，$K_{It}=3K_{I,\rm independent}$，$L_I=(10/3)L_{I,\rm independent}$。
逐轴反馈加速度限幅算子记为 $C_a$，则：

$$a_{PD}=C_a(\lambda p_i),\quad a_{PID}=C_a(\lambda p_i-K_{It}I_{t,i}),
\qquad b_i^{\rm req}=m_i(a_{PID}-a_{PD}).$$

先将 $b_i^{\rm req}$ 按三维范数限制到 0.06 N，再把相对上一实际偏置的增量限制为
$0.004\Delta t$ N；得到 $b_i$。预算只限制新增 I 力，整个 PD/PID 还受反馈、合力和推力限制。
由于先作加速度裁剪，$b_i^{\rm req}$ 不一定等于未饱和的 $-m_iK_{It}I_{t,i}$。
同 tick 重算可更新 PD，但不得再次积分或消耗变化率预算。

当前运输反馈为 $F_{\rm fb,i}=m_i a_{PD}+w_u b_i$。
卸载权重 $w_u$ 空中为 1，触地后沿 2 s 五次曲线降到 0；其导数必须包含
$\frac{d}{dt}(w_ub_i)=w_u\dot b_i+\dot w_ub_i$。
PID 反馈导数还包含 $-\lambda K_v(\ddot x_i-\ddot x_{i,\rm hold})$、$\dot\lambda p_i$，
不是把参考加速度额外再加入一份力前馈。
这里 $\ddot x_i$ 为测量/反馈飞机加速度，不是第 5 节包含重力的核心比力 $a_i$。

保留的 `differential_integral` 分支使用 $F_{\rm fb,i}=m_iC_a(\lambda p_i)+w_ub_{d,i}$：

$$\widetilde e_i=e_{p,i}-{1\over3}\sum_j e_{p,j},\qquad
\dot b_{d,i}^{\rm req}=-k_d\widetilde e_i,\qquad \sum_i b_{d,i}=0.$$

用全组三机共同缩放分别限制最大每机范数与更新范数，维持零和；配置为
$k_d=0.05$ N/(m·s)、每机 0.04 N、0.002 N/s。
当前 native 模式即使保留 `transport_hold_integral_enabled: true`，也不叠加该差分偏置或第二份 PD。

两种运输反馈都保留独立的竖直支撑 trim：

$$b_{z,i}^{\rm req}(0)=\max(F_{i,h,z}-u_{i,z},0),\qquad
H_i=\max(T_{\max,i}-u_{i,z},0),$$

$$L_i=\min\!\left(\max(0.25,b_{z,i}^{\rm req}(0)),H_i\right),\qquad
b_{z,i}(0)=\min(b_{z,i}^{\rm req}(0),L_i).$$

$$\dot b_{z,i}^{\rm req}=\operatorname{clip}_{\pm0.03}[0.20(x_{0d,z}-x_{0,z})].$$

其状态限制在 $[0,L_i]$；普通预算为 0.25 N，接管初值可在可用竖直推力余量内提高该上限，
但实际初值仍受该余量裁剪。上一周期命令总推力达到 $T_{\max,i}-0.01\ \mathrm{N}$ 时停止正向更新，
仍允许负向消退，直至状态下限。初值可逐机不同，既非零和也非额外再加负载重量。
实现见 [slung_transport_guard.py](../ros_ws/src/crazyswarm/scripts/slung_transport_guard.py) 和
[slung_transport_integral.py](../ros_ws/src/crazyswarm/scripts/slung_transport_integral.py)。

运输 native I、差分 I 在参考建立完成后才更新；姿态参考 trim 的提前更新选项见第9.3节。
学习还要求负载/车辆反馈新鲜、
EKF/滤波导数有效、各绳带符号张力为正、
上一周期主机各输出未限幅，且未进入下降、故障或验证截止。
native I 允许在健康的运动参考期间学习；差分 I 在圆周/八字/圆锥期间冻结学习。
姿态参考 trim 默认也冻结，显式启用动态模式后才允许在健康的运动参考期间继续更新，且额外使用第 9.3 节门控。
冻结学习保留状态；缓存保持另设显式力保持，触地后才卸载和复位。

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

令 $\Delta u_i=m_i\Delta a_i$，则运输接管期间可概括为：

$$F_{i,\rm raw}=(1-\beta)F_{i,h}
+\beta[u_i+\Delta u_i+b_{z,i}e_3+F_{\rm fb,i}].$$

$F_{i,h}$ 优先使用冻结的接管实际期望力；没有该快照时按原独立保持关系计算。
对 $\beta$、保持倍率与卸载权重的乘积求导时均保留乘积导数项。
先限制竖直合力至 $[0,T_{\max}]$，再限制水平模长至 $F_z\tan\theta_{\max}$；
相同活动分支用于期望力导数。该操作限制倾角与竖直分量，不直接把完整合力模长限制到 $T_{\max}$。

当前运输力滤波时间常数 $\tau_F=0.005$ s：

$$F_f^+=F_f+{\Delta t\over\tau_F+\Delta t}(F_{\rm limited}-F_f),\qquad
\dot F_f=(F_f^+-F_f)/\Delta t.$$

复位时直接继承输入，导数取零；机体角速度命令滤波当前为 0。
以 $b_{3c}=F_f/\|F_f\|$，将 yaw 参考在垂直 $b_{3c}$ 的平面正交化，得到 $R_{ic}$。
低力模长或航向投影退化时使用上一有效轴/航向回退。

$$e_{R,i}={1\over2}(R_{ic}^TR_i-R_i^TR_{ic})^\vee,\qquad
\Omega_i^{\rm cmd}=-K_{R,i}e_{R,i}-K_{\Omega,i}\Omega_i.$$

当前运输分支显式取 $\Omega_{ic}=0$；`use_body_rate_feedforward` 不覆盖这个分支。
CF3/CF4 的主机 $K_R=[12,12,6]$，CF5 为 `[8,8,6]`；三机 $K_\Omega=[0.25,0.25,0.125]$。
接管时与独立命令混合，之后按每轴机体速率上限裁剪。
普通几何飞行仍保留解析/对数差分 $\Omega_c$ 与姿态积分；独立分支使用自己的姿态 P 增益。

$$T_i^{\rm cmd}=\operatorname{clip}
\left(\max(F_f^TR_ie_3,T_{\min}),0,T_{\max}\right),\qquad T_{\max}=1.0\ \mathrm{N}.$$

绳张力 $T_i$ 与飞机总推力 $T_i^{\rm cmd}$ 是不同量。
主机输出总推力 N 和物理机体系角速度 rad/s；C++ 桥接完成 PWM/符号/包格式转换，
固件负责机载环与电机混控，主机没有直接发送完整飞机力矩。

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
