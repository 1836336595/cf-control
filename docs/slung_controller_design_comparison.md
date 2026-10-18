# 当前三机吊运控制器与《几何控制协同搬运》设计笔记的对照

本文说明原始几何控制推导如何对应到当前 Crazyflie/NOKOV/CTBR 实现，以及为了完成实际起飞、接管、运输和降落增加了哪些控制与调度逻辑。数值以 2026-10-18 工作区中的配置和源码为准。

当前运行配置选择 **`reference_cascade`**：保留载荷位置 PID、载荷姿态几何 PD 和载荷六维力/力矩分配，采用一致的载荷姿态/绳向几何参考、各机位置保持 PD，以及接管竖直残差补偿。源码同时保留 `coupled` 分支用于对照。两个分支都属于基于原始几何框架的工程实现；当前程序没有完整实现原笔记中的多通道自适应扰动估计，也没有在主机输出原笔记的 UAV 姿态力矩控制律。

本文的原始对照材料为本地文件 `/home/csy/my_project/几何控制协同搬运/几何控制协同搬运.md`，共 932 行，题为《Geometric Control of Quadrotor UAVs Transporting a Cable-Suspended Rigid Body》，标注 Taeyoung Lee、TCST 2018。该笔记位于 `crazyswarm` Git 仓库外，因此这里直接给出所需公式，并用笔记章节名定位，不依赖 GitHub 上不可用的上级目录链接。下文“原笔记”专指这份本地推导；其公式、后续工程修改与原论文的完整证明应分别核对。

主要实现入口：

- [slung_load_controller.py](../ros_ws/src/crazyswarm/scripts/slung_load_controller.py)：载荷外环、分配、绳向状态及每机三维力。
- [ctbr_controller.py](../ros_ws/src/crazyswarm/scripts/ctbr_controller.py)：状态来源、全机调度、参考、位置保持、力到 CTBR、预检与日志。
- [slung_reference.py](../ros_ws/src/crazyswarm/scripts/slung_reference.py)：$SO(3)$ 姿态与 $S^2$ 绳向参考及解析导数。
- [slung_payload.yaml](../ros_ws/src/crazyswarm/config/slung_payload.yaml)、[ctbr_vehicle.yaml](../ros_ws/src/crazyswarm/config/ctbr_vehicle.yaml)、[ctbr_controller.yaml](../ros_ws/src/crazyswarm/config/ctbr_controller.yaml)：负载、各机与全局配置。

## 1. 坐标、对象与作用力

### 1.1 先统一坐标约定

| 对象 | 原笔记 | 当前 ROS/NOKOV 实现 |
| --- | --- | --- |
| 惯性系第三轴 | $e_3$ 向下，重力为 $+g e_3$ | 右手世界系 $z$ 向上，记 $e_z=[0,0,1]^T$，物理重力为 $-g e_z$ |
| 载荷姿态 | $R_0$，载荷机体系到世界系 | 同样为 body-to-world 的 $R_0$ |
| UAV 姿态 | 机体第三轴向下，实际推力 $-f_iR_ie_3$ | 当前机体第三轴为推力方向，实际推力 $+f_iR_ie_z$ |
| 绳向 | $q_i$ 从 UAV 指向载荷挂点 | 定义不变；正常悬吊时 $q_{iz}<0$ |
| 载荷角速度 | $\Omega_0$ 在载荷机体系表达 | 定义不变，满足 $\dot R_0=R_0\widehat\Omega_0$ |
| 绳角速度 | $\omega_i$ 在世界系表达 | $\omega_{\mathrm{link},i}=q_i\times\dot q_i$ 在世界系表达 |
| 载荷力矩 | 在载荷机体系表达 | 分配器右端的 $M_d$ 也在载荷机体系表达 |
| UAV 输出 | 最终为 $f_i,M_i$ | 主机输出 collective thrust 与 body-rate，机载控制器实现角速度跟踪和混控 |

$\widehat a b=a\times b$，$\vee$ 为其逆映射。变换坐标时，向量、旋转矩阵、角速度、挂点和叉乘关系必须一致变换，不能只对某个公式的重力项改号。代码中旧 MATLAB 挂点转为 ROS 上表面挂点的辅助函数，是具体几何数据转换；它不能替代对整套控制律的坐标约定说明。

为避免混淆，本文在当前实现公式中始终使用 $e_z$。例如原笔记中的 $\ddot x_0-ge_3$ 对应当前向上坐标的 $\ddot x_0+g e_z$。

### 1.2 挂点、几何中心与实测绳向

设 $\rho_i$ 是载荷几何中心到第 $i$ 个挂点的载荷机体系向量，$x_{a_i}$ 是其世界位置。原笔记的几何关系继续使用：

$$
x_{a_i}=x_0+R_0\rho_i,\qquad
x_i=x_0+R_0\rho_i-L_iq_i.
$$

当前配置的载荷尺寸为 $[0.08,0.06,0.05]\ \mathrm m$、质量 $m_0=0.023\ \mathrm{kg}$，顺序固定为 CF3、CF4、CF5，对应挂点：

$$
\rho_3=\begin{bmatrix}0.040\\0\\0.025\end{bmatrix},\quad
\rho_4=\begin{bmatrix}-0.040\\0.030\\0.025\end{bmatrix},\quad
\rho_5=\begin{bmatrix}-0.040\\-0.030\\0.025\end{bmatrix}\ \mathrm m.
$$

YAML 中每行为一个挂点；控制核心内部每列对应一架飞机。这个转置关系影响分配器堆叠顺序，必须与 CF3/4/5 一致。

NOKOV 的 `load` 原点位于上表面中心，控制器需要的是长方体几何中心，并以均匀长方体假设将其当作质心。实际载荷的质心尚未由该转换或当前日志唯一辨识；若实体质心偏离几何中心，挂点力臂和惯量模型会有误差。当前原点转换为：

$$
x_0=x_{\mathrm{top}}+R_0\begin{bmatrix}0\\0\\-h/2\end{bmatrix}.
$$

这是随载荷姿态旋转的偏置，载荷倾斜时不能仅减去世界系高度 $h/2$。最低点离地间隙按有向长方体计算：

$$
c=x_{0z}-\frac12\sum_{j=1}^{3}|R_{0,3j}|s_j-z_{\mathrm{ground}}.
$$

`payload_ground_clearance` 辅助函数默认取世界地面 $z_{\mathrm{ground}}=0$；实际下降/触地逻辑另记录起飞前实测的负载地面几何中心高度与最低点间隙基线，以其作为本次落地参考。这两个量不能与实体绝对地面或动捕原点偏差混为一谈。

控制器以当前飞机位置和载荷挂点重新计算绳向，而不是始终强制 $\|x_{a_i}-x_i\|=L_i$：

$$
d_i=\|x_{a_i}-x_i\|,\qquad q_i=\frac{x_{a_i}-x_i}{d_i},
$$

$$
\dot q_i=\frac{I-q_iq_i^T}{d_i}
\left[v_0+(R_0\Omega_0)\times(R_0\rho_i)-v_i\right],\qquad
\omega_{\mathrm{link},i}=q_i\times\dot q_i.
$$

这里 $q_i^T\dot q_i=0$，$q_i^T\omega_{\mathrm{link},i}=0$。归一化几何和 $S^2$ 切平面投影保留了原笔记的几何结构；在松绳阶段，它们只是两个实测点的方向，不能据此认定绳索已经有张力。因此实际接管还需要独立的距离、方向、离地间隙和时间确认。

## 2. 保留的载荷几何控制骨架

原笔记“载荷控制器”和“每架 UAV 的理想三维力”的主链为：

$$
(x_{0d},R_{0d})\longrightarrow(F_d,M_d)
\longrightarrow\mu_{id}\longrightarrow(q_{id},\mu_i)
\longrightarrow u_i\longrightarrow(f_i,R_{ic}).
$$

当前仍使用这条链中的载荷反馈、分配和作用力投影；`reference_cascade` 额外通过几何参考和各机位置保持来形成飞机间的相对构型。

### 2.1 载荷位置 PID

记 $e_x=x_0-x_{0d}$，$e_v=v_0-v_{0d}$。当前载荷积分器是有界 C1 形式：

$$
\dot\eta_x=e_v+c_1e_x,\qquad
\eta_x\leftarrow\operatorname{clip}(\eta_x,-\eta_{\max},\eta_{\max}),
$$

$$
a_{0c}=a_{0d}-K_xe_x-K_ve_v-K_I\eta_x,\qquad
F_d=m_0(a_{0c}+g e_z).
$$

向量增益均逐轴作用。当前 $K_x=[3,3,3.75]$、$K_v=[3.12,3.12,3.12]$、$K_I=[1.6,1.6,1.6]$、$c_1=0.5$，积分状态上限 $[0.5,0.5,0.5]$。

这保留了原笔记“用积分补偿固定扰动力”的思想。若定义 $\bar\Delta_{\mathrm{eq}}=m_0K_I\eta_x$，可将它理解为聚合平动扰动力的有界积分补偿；它没有逐项维护原笔记的 $\bar\Delta_{x_0}$、$\bar\Delta_{R_0}$、各机 $\bar\Delta_{x_i}$ 及其耦合自适应更新律。

### 2.2 载荷 $SO(3)$ 姿态 PD 与当前前馈

姿态误差和角速度坐标变换保持原定义：

$$
Q=R_0^TR_{0d},\qquad
e_R=\frac12(R_{0d}^TR_0-R_0^TR_{0d})^\vee,\qquad
e_\Omega=\Omega_0-Q\Omega_{0d}.
$$

当前代码的力矩公式为：

$$
M_d=-K_Re_R-K_\Omega e_\Omega
+\Omega_0\times(J_0\Omega_0)
-J_0\left(\Omega_0\times Q\Omega_{0d}\right)
+J_0Q\dot\Omega_{0d}.
$$

随后计算用于 `coupled` 挂点补偿的命令角加速度：

$$
\dot\Omega_{0c}=J_0^{-1}\left[M_d-\Omega_0\times(J_0\Omega_0)\right].
$$

惯量 $J_0$ 由均匀长方体近似计算。例如 $J_{xx}=m_0(s_y^2+s_z^2)/12$。姿态增益由惯量、带宽和阻尼比生成；当前带宽为 $[1,1,0.45]\ \mathrm{Hz}$、阻尼比 $0.90$，yaw 通道开启。该负载姿态带宽与各机 `transport_attitude_gain` 是两组不同的参数。

原笔记“载荷控制器”给出的名义前馈是：

$$
M_{d,\mathrm{note}}=-K_Re_R-K_\Omega e_\Omega
+v\times(J_0v)+J_0Q\dot\Omega_{0d},\qquad v=Q\Omega_{0d},
$$

并附加载荷/飞机扰动力矩估计。它与当前代码的测量角速度陀螺项及参考坐标导数补偿**一般不相等**。在 $e_\Omega=0$ 时两种名义前馈一致；静态参考 $\Omega_{0d}=0$ 时，当前代码仍保留 $\Omega_0\times J_0\Omega_0$，原笔记该式的 $v\times J_0v$ 为零。当前使用标准的测量陀螺耦合抵消形式，不能把两式说成简单重排。

### 2.3 六维载荷力/力矩分配

原笔记的固定分配矩阵继续使用：

$$
\mathcal P=
\begin{bmatrix}
I&I&I\\
\widehat\rho_3&\widehat\rho_4&\widehat\rho_5
\end{bmatrix}\in\mathbb R^{6\times9},\qquad
z=\begin{bmatrix}R_0^T\mu_{3d}\\R_0^T\mu_{4d}\\R_0^T\mu_{5d}\end{bmatrix}.
$$

$$
\mathcal Pz=\begin{bmatrix}R_0^TF_d\\M_d\end{bmatrix},\qquad
z=\mathcal P^T(\mathcal P\mathcal P^T)^{-1}
\begin{bmatrix}R_0^TF_d\\M_d\end{bmatrix}+z_{\mathrm{null}}.
$$

前三个分量表示载荷机体系合力，后三个表示载荷机体系合力矩；求解后各块再乘 $R_0$ 返回世界系。代码使用条件数/容差检查后求线性方程，没有直接显式求逆。

与原笔记的最小范数解相比，代码增加了 `outward_bias`：将水平外张力投影进 $\ker\mathcal P$，得到 $\mathcal Pz_{\mathrm{null}}=0$ 的内部力。当前比例为 $0.25$、配置上限 $0.15\ \mathrm N$，用于形成更分散的静态绳向。它在虚拟分配层保持原合力/力矩；实际绳向投影、单向张力和执行器饱和之后，不能继续假设实现的广义力恰好等于 $(F_d,M_d)$。

### 2.4 请求力、实测绳向与单向张力

分配器得到的是可在三维中任意指向的请求力 $\mu_{id}$；实际绳索只能沿当前实测 $q_i$ 拉载荷。原笔记先取：

$$
\mu_i=q_iq_i^T\mu_{id},\qquad q_{id}=-\frac{\mu_{id}}{\|\mu_{id}\|}.
$$

当前进一步执行单向约束：

$$
T_i^{\mathrm{signed}}=-q_i^T\mu_{id},\qquad
T_i=\max(0,T_i^{\mathrm{signed}}),\qquad
\mu_i=-T_iq_i.
$$

当请求相当于压绳时，代码令有效 $q_{id}=q_i$；`raw_desired_link_direction` 仍记录分配器原始方向，`desired_tension_signed` 仍记录带符号请求。不能把 `desired_tension` 当成实测张力传感器数据，也不能把 `desired_tension_vector` 当成经过当前绳向投影后真正作用于载荷的力。

当 $T_i^{\mathrm{signed}}>0$ 时，上式等于原笔记的 $q_iq_i^T\mu_{id}$；被截断为零时则不再等于无约束分配。因此“$\operatorname{rank}\mathcal P=6$”只说明该几何允许三维虚拟力分配，不保证某个瞬间的实测绳向、张力方向和推力余量能实现全部六维请求。当前没有以 $T_i\ge0$、推力上限和绳锥约束重新求解的完整优化分配器。

## 3. 两个每机三维力分支的准确区别

### 3.1 原笔记的理想沿绳/垂绳分解

原笔记定义的重力补偿挂点加速度，在当前坐标下写成：

$$
a_i=\ddot x_0+g e_z
+R_0\widehat\Omega_0^2\rho_i
-R_0\widehat\rho_i\dot\Omega_0.
$$

沿绳分量为：

$$
u_i^\parallel=\mu_i+m_iL_i\|\omega_i\|^2q_i+m_iq_iq_i^Ta_i.
$$

完整绳向跟踪中，原笔记还定义：

$$
\omega_{id}=q_{id}\times\dot q_{id},\quad
e_{q_i}=q_{id}\times q_i,\quad
e_{\omega_i}=\omega_i+\widehat q_i^2\omega_{id},
$$

$$
u_i^\perp=m_iL_i\widehat q_i
\left[-k_qe_{q_i}-k_\omega e_{\omega_i}
-(q_i\cdot\omega_{id})\dot q_i
-\widehat q_i^2\dot\omega_{id}\right]
-m_i\widehat q_i^2a_i-\bar\Delta_{x_i}^{\perp}.
$$

这里包含移动绳向参考的角速度/角加速度前馈，以及垂绳扰动估计。原笔记中的跟踪误差、$S^2$ 势函数及后续稳定性分析围绕这些完整项展开。

### 3.2 `coupled`：保留耦合补偿的工程近似

当前 `coupled` 不直接使用未知的真实 $\ddot x_0,\dot\Omega_0$，而是构造：

$$
a_{i,c}=a_{0c}+g e_z
+R_0\widehat\Omega_0^2\rho_i
-R_0\widehat\rho_i\dot\Omega_{0c}.
$$

绳向校正采用：

$$
c_i=-\lambda_q\left(k_qe_{q_i}+k_\omega\omega_{\mathrm{link},i}+k_{Iq}\eta_{q_i}\right),
\qquad \dot\eta_{q_i}=e_{q_i},
$$

$$
F_{q_i}=\operatorname{limit}_{\mathrm{norm,rate}}
\left(m_iL_i\,q_i\times c_i\right).
$$

因此控制核心的输出可合并为：

$$
F_{i,\mathrm{core}}=-T_iq_i+m_ia_{i,c}
+m_iL_i\|\omega_{\mathrm{link},i}\|^2q_i+F_{q_i}.
$$

其中使用 $q_iq_i^Ta_i-\widehat q_i^2a_i=a_i$ 合并沿绳和垂绳的运动学补偿。当前 `link_kq=40`、`link_komega=20`、`link_integral_gain=0`；方向校正力幅值上限 $0.40\ \mathrm N$、变化率上限 $20\ \mathrm{N/s}$，其限幅历史只保存方向校正力，运动学补偿不进入该历史。

这是原笔记几何结构的近似实现：代码没有从 $\mu_{id}(t)$ 生成 $\dot q_{id},\omega_{id},\dot\omega_{id}$ 并代入完整绳向前馈，阻尼项使用的是 $\omega_{\mathrm{link},i}$；也没有实现 $\bar\Delta_{x_i}^{\perp}$。`link_gain_scale` 还会随接管/运输阶段变化并接受限幅。不能仅因变量名和沿绳/垂绳形式相同，就称其为 Lee 2018 全控制律复现。

### 3.3 `reference_cascade`：当前启用的参考串级

该模式仍用载荷误差生成 $F_d,M_d,\mu_{id},T_i$，但飞机自身惯性支撑使用**期望挂点运动**：

$$
a_{a_i,d}=a_{0d}+g e_z
+R_{0d}\widehat{\Omega}_{0d}^{2}\rho_i
-R_{0d}\widehat\rho_i\dot\Omega_{0d}.
$$

它把核心的有效绳向校正倍率强制设为 $\lambda_q=0$，由各机位置保持提供构型恢复与阻尼。YAML 中仍保留 `transport_link_gain_scale=0.4` 与 `transport_link_steady_gain_scale=0.4`，但该模式下它们不会使绳向反馈生效；运行日志的 `effective_transport_link_gain_scale` 应为零。

在参考串级中，完整接管后、力/倾角限幅和滤波前，每机实际组合为：

$$
\boxed{
F_{i,\mathrm{raw}}=-T_iq_i+m_ia_{a_i,d}
+m_iL_i\|\omega_{\mathrm{link},i}\|^2q_i
+F_{i,\mathrm{hold}}+b_{z,i}e_z
}
$$

$$
F_{i,\mathrm{hold}}=m_i s_h
\left[-K_{p,i}^{\mathrm{ind}}(x_i-x_{i,h})
-K_{v,i}^{\mathrm{ind}}(v_i-v_{i,h})\right].
$$

保持加速度还接受每轴反馈上限。$x_{i,h},v_{i,h}$ 为实际 `position_hold_position/velocity`，$s_h$ 是保持倍率，$b_{z,i}$ 为接管竖直 trim。当前保持项是 **PD**，没有新的各机运输位置积分。

这里使用各机 `independent_position_gain`/`independent_velocity_gain`，单位分别为 $\mathrm{s^{-2}}$、$\mathrm{s^{-1}}$；先得到加速度再乘质量，不能与普通位置环直接以力为输出的增益混用。CF3 的两组增益分别为 $[3,3,4]$、$[3.12,3.12,3.60]$，CF4/CF5 为 $[4,4,4]$、$[3.6,3.6,3.6]$。三机质量依次为 43.4、44.0、42.4 g。

这一分支避免将载荷位置/姿态反馈产生的命令加速度再次作为各机自身惯性补偿，同时把构型跟踪交给飞机位置环。它是控制结构的变化，不能套用原笔记中“沿绳/垂绳闭环误差动力学已经按完整前馈解耦”的结论。

### 3.4 张力渐增时实际还存在混合

前一节框中公式对应 `transport_blend=1`。在 `TAUT_RAMP` 中，设五次混合系数为 $\beta\in[0,1]$，则送入后处理前的力是：

$$
F_{i,\mathrm{raw}}=(1-\beta)F_{i,\mathrm{handoff}}
+\beta\left(F_{i,\mathrm{core}}+b_{z,i}e_z+F_{i,\mathrm{hold}}\right).
$$

$F_{i,\mathrm{handoff}}$ 是接管前实际支持负载的独立 PID 输出，来自锁定的 `handoff_force`。它包含已经积累的独立控制补偿，因而不是简单的飞机自重。核心力中已有载荷张力，trim 也只继承模型差额，不能再额外叠加一遍完整载荷重量。

力导数同时包含 $\dot\beta$、保持倍率导数和 trim 导数。随后还要经过竖直力上限、倾角对应的水平力上限、可配置力滤波以及当前推力轴投影，因此控制核心 `output["force"]`、`raw_desired_force`、滤波后的 `desired_force`、发送的 collective thrust 和物理电机升力是不同层次。

## 4. 几何参考使载荷调平与各机位置保持一致

### 4.1 只改变载荷姿态参考还不够

从接管实测姿态 $R_h$ 切换到水平目标 $R_*$ 时，挂点位置随之移动。如果飞机位置保持仍锁在旧挂点构型，负载姿态环和各机保持环会对同一个绳长约束提出不一致的要求。当前在 `transport_attitude_reference_enabled` 和 `transport_equilibrium_reference_enabled` 都开启时，同步生成姿态、挂点与静态平衡绳向参考。

五次时间参数为：

$$
\tau=\operatorname{clip}(t/T,0,1),\qquad
s(\tau)=10\tau^3-15\tau^4+6\tau^5.
$$

它在两端具有零一阶、二阶导数。当前 $T=12\ \mathrm s$，从 `TAUT_RAMP` 开始计时。

载荷姿态沿 $SO(3)$ 的短测地线变化：

$$
\phi=\log(R_h^TR_*)^\vee,\qquad
R_d=R_h\exp(s\widehat\phi),\quad
\Omega_d=\dot s\phi,\quad\dot\Omega_d=\ddot s\phi.
$$

挂点相对初始姿态的偏移及导数为：

$$
\Delta r_i=(R_d-R_h)\rho_i,\quad
\Delta\dot r_i=R_d\widehat\Omega_d\rho_i,\quad
\Delta\ddot r_i=R_d(\widehat{\dot\Omega}_d+\widehat{\Omega}_d^{2})\rho_i.
$$

### 4.2 静态平衡绳向与 $S^2$ 插值

目标 $q_{i,*}$ 由水平目标姿态下的静态分配求得：$F_*=[0,0,m_0g]^T$、$M_*=0$，并使用同一内部外张力。它是**静态构型目标**；当前载荷外环每周期产生的动态 $q_{id}$ 仍用于请求力诊断，两者不是同一变量。

设 $q_{i,h}$ 为接管时绳向，选短弧旋转轴 $a_i$ 和夹角 $\theta_i$，则：

$$
q_{i,r}=\exp(s\theta_i\widehat a_i)q_{i,h},\qquad
\dot q_{i,r}=\dot s\theta_i(a_i\times q_{i,r}),
$$

$$
\ddot q_{i,r}=\ddot s\theta_i(a_i\times q_{i,r})
+(\dot s\theta_i)^2a_i\times(a_i\times q_{i,r}).
$$

同向时保持不动；精确反向的非唯一短弧由确定性规则选轴。正常向下悬吊绳通常远离该反向情形。

设 $d_{i,h}$ 为接管实测距离，飞机物理保持参考为：

$$
x_{i,h}^{\mathrm{ref}}=x_{i,h}^{\mathrm{meas}}
+(x_{0d}-x_{0,h})+(R_d-R_h)\rho_i
-d_{i,h}(q_{i,r}-q_{i,h}).
$$

速度、加速度使用同式的解析导数。这样既考虑挂点随姿态移动，又让绳向向静态平衡构型变化，并保留接管时的实测距离。`slung_reference.py` 生成的 $\dot q_{i,r},\ddot q_{i,r}$ 用来构造飞机几何位置参考；这不代表控制核心已经实现原笔记动态 $q_{id}$ 的完整绳向前馈。

另外，接管前 PID 的命令参考与实测锚点之间可能有持续支撑负载所需的偏差。`handoff_references` 对该偏差进行混合，避免切换时位置/速度/加速度突变。日志中的普通 `target position` 与实际 `position_hold_position` 因而可能不同；计算保持误差或讨论构型静差时应使用后者。

### 4.3 当前时间轴

| 从接管起计时 | 当前行为 |
| --- | --- |
| $0\sim5$ s | `TAUT_RAMP`：负载平动参考固定，独立支持力向协同力平滑混合 |
| $0\sim12$ s | 同时完成载荷姿态、挂点及静态平衡绳向的五次过渡 |
| $5\sim17$ s | 负载参考以 12 s 五次曲线抬升到几何中心目标高度 $0.60$ m |
| 保持倍率 | ramp 为 $2.0$；参考抬升期间平滑消退至稳态 $1.5$ |
| $20$ s | 当前实飞验证计时到达，开始协同放下载荷 |

因此当前 20 s 窗口包括参考建立过程；两类参考都完成后只剩约 3 s。它不是“在最终目标已稳定后悬停 20 s”，也不是起飞后第 20 s。`post_handoff_s=60` 是离线回放观察窗口，与实飞验证计时参数分开。

## 5. 从三维力到 CTBR：保留几何映射，改变最终接口

当前先限制和滤波 $F_i$，再令：

$$
b_{3c}=\frac{F_i}{\|F_i\|},\qquad
R_{ic}=[b_{1c},b_{2c},b_{3c}],\qquad
f_i=\operatorname{clip}(F_i^TR_ie_z,0,f_{i,\max}).
$$

$b_{1c},b_{2c}$ 由期望航向与 $b_{3c}$ 正交化得到，并对接近共线、力范数过小等情形提供数值回退。原笔记的 $b_{3c}=-u/\|u\|$ 和 $f=-u^TR_ie_3$ 的负号来自其机体第三轴约定；当前第三轴沿推力方向，因此这里为正。

普通 CTBR 模式支持由 $\dot F_i$ 解析计算 $\Omega_{ic}$，也保留旋转矩阵对数差分方式。**当前独立吊运和协同运输分支都将 `computed_body_rate` 置零**，运输分支采用：

$$
\Omega_i^{\mathrm{cmd}}=-K_{R_i}^{\mathrm{transport}}e_{R_i}
-K_{\Omega_i}^{\mathrm{transport}}\Omega_i.
$$

当前三机均使用 $K_{R_i}^{\mathrm{transport}}=[12,12,6]$、$K_{\Omega_i}^{\mathrm{transport}}=[0.25,0.25,0.125]$；body-rate 上限 $[3,3,2]\ \mathrm{rad/s}$，每机命令推力上限 $1.0\ \mathrm N$。当前运输力滤波时间常数为 $0.005\ \mathrm s$，body-rate 滤波时间常数为零。

原笔记还讨论了三种相关但不同的姿态层：角速度 PI+前馈、2014 型几何力矩控制，以及 2018 型有限时间滑模力矩控制。当前主机不会发送 $M_i$，没有实现滑模面 $s_i$、逐元素分数幂 $S(r,\cdot)$ 或有限时间力矩反馈。实际角速度响应由所选机载控制器、电机、混控、延迟与饱和共同决定。仿真中的 Mellinger、500 Hz 机载更新和电机模型用于模拟该接口，不能替代主机完整力矩控制律的实现。

## 6. 新增的状态估计、预检与飞行阶段

### 6.1 状态来源的变化

原笔记以完整状态可用为前提展开推导；当前需要区分各数据链路：

| 控制量 | 当前来源 | 关键处理 |
| --- | --- | --- |
| 飞机位置、姿态 | NOKOV | 唯一位姿来源；检查消息有效性、年龄、时间戳及工作空间 |
| 飞机平移速度、加速度 | 固件 EKF 的位置/速度日志 | 使用 5 Hz 因果二阶速度滤波，滤后差分产生加速度；当前为纯 EKF 运动学 |
| 飞机 body-rate | NOKOV 姿态的服务端差分/滤波，`mocap_state.twist.angular` | 以机体系角速度进入运输阻尼项；并非固件 EKF 平移包中的陀螺值 |
| 负载位置、姿态 | NOKOV | 上表面原点转换为几何中心；姿态反馈使用接收到的旋转矩阵 |
| 负载速度、加速度、body-rate | `PayloadStateObserver` | 只读取负载位姿/姿态，以因果平动 $\alpha$-$\beta$-$\gamma$ 和 $SO(3)$ 观测得到 |
| 负载原始 NOKOV twist/acceleration | CSV 诊断 | 当前负载观测器不消费这些字段；飞机平移控制也不在 EKF 失效时回退为 NOKOV 差分，飞机 angular twist 的用途见上行 |

负载平动预测及修正为：

$$
\hat x^-=\hat x+\Delta t\hat v+\tfrac12\Delta t^2\hat a,\quad
\hat v^-=\hat v+\Delta t\hat a,\quad r=x_{\mathrm{meas}}-\hat x^-,
$$

$$
\hat x^+=\hat x^-+\alpha r,\quad
\hat v^+=\hat v^-+\beta r/\Delta t,\quad
\hat a^+=\hat a+\gamma r/\Delta t^2.
$$

当前 $\alpha=0.35,\beta=0.20,\gamma=0.02$；姿态修正增益 $0.35$、body-rate 修正增益 $0.30$。观测器至少需要 3 个独立样本，间隔超过 $0.05$ s 重置，并对速度、加速度和角速度限幅。这是状态导数观测器，不是原笔记的外部扰动力自适应估计器。

起飞前要求 EKF/NOKOV 位置差小于 $0.05$ m 连续 1 s，并要求滤波导数就绪。飞行中位置差阈值为 $0.30$ m，新鲜 EKF 数据年龄上限 $0.15$ s，可短时保持最后有效运动学 $0.10$ s，持续失效 $0.35$ s 后转受控降落。飞行中不通过重置 EKF 来掩盖失效。

### 6.2 电压预检与推力映射

启动时锁定每机新鲜电压采样，当前最低起飞电压 $3.8$ V。以各机配置计算：

$$
s_{\mathrm{raw},i}=\left(\frac{V_{\mathrm{ref},i}}{V_i}\right)^{p_i},
$$

当前 $V_{\mathrm{ref}}=4.20$ V、$p=1$，允许尺度范围 $[0.90,1.12]$。采样通过后本次运行冻结尺度；三机全部通过电压/位姿/EKF 预检才共同释放起飞。该比例是命令映射补偿，不是在线辨识出的实际升力效率，更不等于实际电机推力已经精确校准。

### 6.3 松绳起飞、收绳与预抬

实际程序增加了原笔记理想绷紧模型以外的飞行阶段：

```mermaid
flowchart TD
    A[电压 / 位姿 / EKF 全机预检] --> B[独立起飞]
    B --> C[TAKEUP 收绳]
    C --> D[独立控制预抬与几何确认]
    D --> E[TAUT_RAMP 混合接管]
    E --> F[参考抬升与协同运输]
    F --> G[协同放下载荷]
    G --> H[实测稳定触地确认]
    H --> I[2 s 有界卸载]
    I --> J[各机保持 XY 独立下降]
    J --> K[落地确认与零输出]
```

当前独立起飞目标高度 $0.50$ m、时间 5 s，TAKEUP 名义时长 8 s。目标由当前负载几何、静态平衡绳向及 $0.12$ m 水平外张余量产生，外张后重新计算垂直分量以保留目标绳距。

接管条件包括：实测绳距满足目标与名义长度容差 $0.030$ m、相对目标绳向误差不超过 $10^\circ$、连续确认 $0.50$ s；名义飞机位置偏差阈值 $0.15$ m 保留为诊断，已不单独阻塞接管。预抬还要求负载最低点离地 $0.20$ m、低竖直速度等连续条件成立。其参考上移速度上限 $0.05$ m/s、加速度上限 $0.05$ m/s²，并设置参考总上移、参考相对实测超前和 60 s 超时边界。

预抬通过独立 PID 自身建立真实支撑力：位置增益逐轴乘 $[4,4,1]$、速度增益乘 $[2,2,1]$，加强 XY 抗绳索向内拉的能力，并开放单独的 z 积分门控和限幅。预抬 z 积分状态上限 8、变化率上限 0.5/s，反馈加速度上限 12 m/s²，倾角上限 $12^\circ$，仍受单机 1.0 N 总推力限制及防积分饱和逻辑约束。

### 6.4 一次性有效绳长校准

当前名义 `link_lengths_m=[0.850,0.850,0.850]` m；旧 `link_length_m=0.830` 仅为没有三根长度配置时的兼容回退。`SlungLinkCalibrator` 在收绳后符合条件的稳定独立样本窗口内，允许一次正向有效长度修正：

$$
L_{i,\mathrm{eff}}=\max(L_{i,\mathrm{nom}},\widehat d_i).
$$

当前窗口 3 s、至少 100 个独立时间戳，限制 P90-P10 距离波动、漂移和相对速度；最大增加 $0.045$ m。它不缩短绳长，不把明显松绳距离当真值，应用一次后冻结本次运行有效值，也不回写 YAML。稳定几何平台只是绷紧状态的代理证据，校准器没有张力传感器，因此它不是物理绳长/弹性的完整在线辨识。

### 6.5 接管支撑残差 trim

`SlungSupportTrim` 继承的是接管前支持力与核心模型支持力之间的**正向竖直差额**：

$$
b_{z,i}(0)=\operatorname{clip}\left(
\max(F_{i,h,z}-F_{i,\mathrm{core},z},0),0,b_{i,\max}\right).
$$

普通上限为 $0.25$ N。若实际接管差额超过普通上限，则可在当时可用竖直推力余量内扩大该机本次上限，以避免支持力突降；日志记录请求残差、实际上限和初始化是否被截断。

接管后用同一个负载高度误差调整三机各自的标量 trim：

$$
\dot b_{z,i}=\operatorname{clip}\left(0.20(z_{0d}-z_0),-0.03,0.03\right)\ \mathrm{N/s},
\qquad 0\le b_{z,i}\le b_{i,\max}.
$$

推力饱和时禁止继续增加，实际导数按限幅后的变化计算。三个初始化残差和上限可以不同，但后续误差信号来自共同载荷高度；这不是以每机位置误差学习的三维差动构型积分。

## 7. 积分状态矩阵：哪些正在起作用

| 积分/补偿 | 当前存在 | 在协同运输时是否生效 | 状态与含义 |
| --- | --- | --- | --- |
| 载荷平动 C1 积分 $\eta_x$ | 是 | 是 | 有界三维积分，进入载荷请求力 $F_d$ |
| 载荷姿态积分/扰动力矩估计 | 否 | 否 | 当前姿态环为几何 PD+前馈；没有 $\bar\Delta_{R_0}$ 状态 |
| 绳向积分 $\eta_{q_i}$ | 代码有状态 | 当前无力输出作用 | `link_integral_gain=0`；且 `reference_cascade` 有效绳向倍率为零 |
| 飞机普通位置积分 | 是 | 冻结 | 协同模式不更新；接管后完整力覆盖普通 PID，已有支撑差额另由 handoff/trim 处理 |
| 独立起飞/TAKEUP 位置积分 | 是 | 阶段结束后冻结 | $e_v+0.5e_x$，普通误差门控和逐轴限幅 |
| 预抬单独 z 积分 | 是 | 仅预抬阶段 | 扩大 z 上限和单独变化率/饱和检查，XY 继续原门控 |
| `SlungSupportTrim` | 是 | 是 | 继承各机竖直差额，以共同载荷高度有界慢调 |
| 当前运输各机位置保持积分 | **否** | **否** | `transport_position_hold` 为 PD；不能因配置中存在普通 Ki 就称为运输 PID |
| 主机普通姿态积分 | 是 | 当前运输不更新 | 普通 CTBR 有界 PI；吊运独立/运输分支分别使用其专用姿态反馈 |
| 载荷触地/卸载后各机下降 z 积分 | 是 | 仅该下降阶段 | 单独 z 门控，上限 2、变化率 0.5/s、反馈加速度上限 8 m/s² |
| 离线试验的运输差动 hold I | **仅离线 runner** | **生产控制器未接入** | 去除机群均值后学习相对构型误差；见第 10 节 |

积分“有状态”“配置了非零 Ki”和“当前模式在实际输出中使用它”是三个问题。阅读日志时应同时检查阶段、模式、有效增益、参考和饱和状态。

## 8. 放载、卸载与三机下降

原笔记的绷紧绳索模型没有覆盖地面接触及释放，当前为计划验证结束增加了如下退出链路。

1. 计划验证时间到达或有效负载状态下指定的掉高分支触发时，从**当前实测负载位置、速度、加速度**生成连续五次下降参考。横向终点保持当前 x/y，下降到起飞前采集的地面基线；参考时间自适应增长以满足 $0.12$ m/s 速度与 $0.15$ m/s² 加速度上限。
2. 载荷下降期间维持协同控制和各机位置保持。飞机参考保持切入下降时实测绳向及距离；距离取实测值与有效名义绳长的较大者，避免名义绳长稍短而把飞机突然向内拉。
3. 由实测独立时间戳构成触地窗口：检查地面基线附近高度、P90-P10 波动、拟合高度斜率及竖直速度中位数。当前窗口 0.40 s、确认 0.30 s，高度容差 0.015 m，接触中位数上界为基线上方 0.006 m。等待超时不会被当成已触地。
4. 实测稳定触地后执行完整 2 s 五次 trim 卸载，再释放协同支持，进入各机独立下降。保持已验证的 XY/姿态控制强度以避免短时仍绷紧的绳索拉拢飞机。
5. 各机下降单独开放有界 z 积分，用来克服推力映射静差造成的离地悬停；它已经在当前生产源码中实现，与尚未接入的运输差动积分是两件事。

当前离线下降验证表明，旧普通独立积分上限 $0.20$ 时，单机 $+7.5\%$ 或 $+10\%$ 物理推力误差可形成离地约 0.18 m 或 0.26 m 的静平衡；新下降 z 积分使所测名义、卸载后 $+7.5\%$、$+10\%$ 三个案例都完成实际物理接地和零输出。该证据针对下降完成性。现有飞机落地容差仍可能允许触地前几厘米停推，离线模型也记录了后续无动力下落，因此不能把“最终接地完成”自动解释为已验证柔和着陆。

## 9. 实际保护、日志与仿真验证增加了什么

### 9.1 保护来自实测物理构型

当前运输保护检查三机实测间距、基于相对速度的最近接近预测、负载掉高和持续横向速率饱和。实测最小间距小于 $0.30$ m 立即触发受控下降；预测窗 $0.35$ s；预测靠近、负载高度误差超过 $0.08$ m、速率饱和等各自需要连续确认，当前为 2.20 s。预抬阶段也检查实际间距，避免危险构型进入接管。

负载短时丢帧可保持最后完整协同输出 $0.20$ s，持续无效超过 $0.50$ s 进入受控退出。原笔记的连续时间模型没有这些保持、超时和混合切换逻辑。

当前故障退出与计划退出并不完全统一，需要记录以下实现边界：

| 入口条件 | 当前路径/边界 |
| --- | --- |
| 短时验证到时 | 转 `payload_lowering`，执行上一节的负载下降、接地、卸载、飞机下降 |
| 有效协同状态下的指定 `height_drop` 分支 | 可以保留协同支持进入负载下降 |
| 已在 `payload_lowering` 时出现相关保护且负载状态有效 | 可以保留负载下降；`payload_payload_aware_emergency` 置位后，当前 `_check_payload_physical_safety` 会提前返回，不能称为仍完整重复执行全部物理保护 |
| 更早发生的机间距故障，或不满足上述条件的退出 | 仍可走带支撑残差消退的独立紧急下降，不等价于保持绳向的协同下降 |

这些是当前基线的已知边界，并非本文新增的修复。正常下降成功、某个故障分支通过和任意故障都安全退出是不同验证命题；后续控制改动不能据此假定全状态机已经覆盖。

### 9.2 日志应能区分控制链每一层

CSV 记录飞机/负载实测状态、EKF 健康、原始与滤波导数、参考、阶段、CTBR 命令、电池和调度时序，并增加了以下用于核对公式的诊断：

- `payload_transport_force_mode`、配置与有效绳向倍率、保持倍率、`transport_blend`。
- 载荷期望位置/速度/加速度、期望姿态/body-rate/body-rate-dot、姿态参考进度。
- 实测绳向、有效/原始期望绳向、带符号张力、非负张力、方向误差。
- 原始/受限绳向校正力、运动学补偿、幅值和变化率限幅标记。
- trim 请求、实际值、上限、初始化截断，以及预抬/下降 z 积分状态。
- 实测/预测间距、有效绳长校准状态、触地/卸载状态和完整任务计时。

评估静差时应同步比较真实反馈源、实际保持参考、命令力和物理升力。`desired_tension`、机载 EKF 速度、NOKOV 差分速度和仿真 truth 各自代表不同信号。

### 9.3 仿真通过真实控制接口复现

[simulate_slung_handoff.py](../ros_ws/src/crazyswarm/scripts/simulate_slung_handoff.py) 使用实际 Python 控制器和调度路径；[slung_simulated_telemetry.py](../ros_ws/src/crazyswarm/scripts/slung_simulated_telemetry.py) 管理采样、延迟与滤波输入。当前配置提供 Mellinger/其他机载模型、Graefe 2026 电机动态、逐电机转速到推力/反扭矩、机体惯量、混控饱和、绳索和地面接触等环节。

当前默认离线参数包括 70 Hz 主机 tick、100 Hz 位姿/10 ms 延迟、50 Hz EKF 包/20 ms 延迟、5 ms 命令延迟。`sampled_filtered` 让样本进入实机 `_ekf_state_callback` 的过滤路径；它不是完整固件 EKF 内部状态的仿真。固定周期、实测周期序列、姿态噪声、物理负载转动阻尼、绳长与电机效率都可以作为独立变量对照。

物理回放中还必须说明初态：多项接管/静态调平试验从已在空中的 TAKEUP 锚点启动，覆盖后续接管、运输或下降，不能据此声称复现了飞机从地面独立起飞的全过程。

当前 `slung_payload.yaml` 的物理负载转动阻尼为 $0.001$；第 10 节调平假设试验特意设为零，并加入测量噪声、延迟和电机效率偏置。对不同试验的“稳定”结果，应同时保留这些条件及最终质量判据。

## 10. 已有证据与尚未实现的运输差动积分

### 10.1 实飞状态与模型假设应分开

本地 `multi_ctbr_20261018_230530.csv` 的实飞协同窗口为 **20.012562 s**。姿态参考在接管后约 12.017 s 到达水平；16–20 s 动捕姿态均值为 roll $4.974^\circ$、pitch $15.273^\circ$、yaw $-0.131^\circ$，目标三轴均为零。该段机体横向角速度较小，飞机保持飞行，但负载持续倾斜仍未消除。这里的实际姿态是控制器使用的动捕姿态；实体水平面与刚体坐标系仍需独立核对。

12–20 s 的实际保持误差显示 CF4/CF5 高于保持参考约 5 cm，CF3 低约 1 cm，且三机有不同的 XY 静差。主机命令未触及 1.0 N 总推力上限；CSV 没有逐电机转速/饱和标记，不能把主机余量直接解释为实际电机余量。本次日志的俯仰请求力矩与沿实测绳向投影的请求张力力矩已较接近，不能继续套用其他轮次明显投影失配的解释；请求张力也不是真实测量张力。

退出阶段确认了负载触地和完整卸载，CF3/CF4 最终为 landed/零推力；CF5 日志末帧仍为 landing、高约 0.122 m，不能声称三机均已落地。后续诊断中 CF5 的 z 积分约 0.974，未达到当前上限 2，因此也不能直接归因为新积分上限饱和。

推力映射差异、载荷刚体坐标偏置、质心偏移、绳长/挂点几何和张力辨识均是待分别检验的机制；当前仿真没有完成对真实飞机参数的唯一辨识。

本地 `ros_ws/src/crazyswarm/scripts/ctbr_logs/leveling_static_20261018/` 保存冻结源码、参数、物理模型、真实状态 trace 和分析。该目录的报告属于本地实验产物；下面列出关键数值，使本文件在只上传文档和源码时也能独立阅读。

冻结试验使用当前参考串级、12 s 完整姿态/绳向参考、保持倍率 2.0/1.5、负载姿态带宽 $[1,1,0.45]$ Hz，60 s 接管后窗口、实测控制时序、采样/滤波 EKF、延迟、姿态噪声和零物理负载转动阻尼。物理电机效率 $[1,1.05,1.10]$ 是对三机差异的**假设注入**，不是辨识得到的实机系数。

### 10.2 七个离线案例的实际结果

| 案例 | 50–60 s 真值 roll/pitch 均值（度） | 最终载荷 XYZ 误差范数（m） | 原判据结果 |
| --- | --- | --- | --- |
| 名义效率 $[1,1,1]$，无差动 I | $-1.122,-3.178$ | 0.0434 | pass |
| 效率 $[1,1.05,1.10]$ 从开始即存在 | $-6.476,+20.631$ | 0.1609 | hover_position_error |
| 同一效率偏差从接管时施加 | $-6.491,+20.550$ | 0.1608 | hover_position_error |
| 名义效率，离线差动 $K_I=0.02$ | $-0.379,-0.118$ | 0.0611 | pass |
| 偏置效率，离线差动 $K_I=0.02$ | $-1.106,+2.970$ | 0.1409 | hover_position_error |
| 偏置效率，离线差动 $K_I=0.05$ | $-0.629,+1.253$ | 0.1405 | hover_position_error |
| 偏置效率，无差动 I，仅 roll/pitch 带宽升至 2 Hz | $-5.147,+14.820$ | 0.1457 | hover_position_error |

这些结果支持“持续每机保持误差可伴随载荷持续倾斜”的机制。加入差动补偿后构型/姿态误差下降，但带偏置模型的完整位置判据依旧失败，不能写成“差动 I 已解决运输误差”或“七个案例全部通过”。名义案例加 I 后姿态更平，但最终平动误差也没有因此必然更小。

### 10.3 离线差动 I 的定义及未完成部分

离线 runner 以三机实际保持参考计算：

$$
e_i=x_i-x_{i,h},\qquad
\widetilde e_i=e_i-\frac13\sum_j e_j,\qquad
\dot b_i=-K_I\widetilde e_i.
$$

对整组三机统一缩放，保持：

$$
\sum_i b_i=0,\quad \sum_i\dot b_i=0,\quad
\|b_i\|\le0.03\ \mathrm N,\quad
\|\dot b_i\|\le0.002\ \mathrm{N/s}.
$$

补偿及其真实有界导数只由离线 wrapper 加到 `desired_force_override` 与 `desired_force_dot_override`，现有接管混合仍负责把它们送入各机力输出。它不修改公共载荷支持 trim，并在该力命令注入层保持三机增量零和；后续各机限幅、推力轴投影和不同物理升力效率不保证实际合力增量仍零和。其设计目标是构型差动误差，不能据此声称公共推力映射误差或公共负载位置偏差也已校正。

学习起点为两类参考都完成：

$$
t_{\mathrm{learn}}\ge\max(12,5+12)=17\ \mathrm s.
$$

当前 20 s 实飞窗口仅提供约 3 s 学习。该偏置模型在 20 s 时，未加 I 的真实 pitch 为 $20.36^\circ$，$K_I=0.02$ 时仍为 $17.77^\circ$，$K_I=0.05$ 时仍为 $17.18^\circ$。$K_I=0.02$ 首次连续 2 s 满足 roll/pitch 均小于 $5^\circ$ 的起点约为接管后 43.16 s；$K_I=0.05$ 约为 30.67 s。因此 60 s 后段调平改善不能外推为当前 20 s 窗口已经能调平。

**当前生产 `ctbr_controller.py` 尚未接入上述运输差动 I。** 若后续采用，需要具体定义阶段/有效状态门控、力/倾角/body-rate 饱和时的停止学习、故障和落地复位/消退、导数一致性，以及对相应行为的验证。现有离线 wrapper 检查了一些有效性和上一周期饱和条件，但该批静态试验没有注入所有这些失效路径；它不是已完成的生产功能。

## 11. 为什么不能直接继承原笔记的稳定性结论

原笔记“积分作用来构造扰动估计”采用多通道更新，概念上包括：

$$
\dot{\bar\Delta}_{x_0}=\frac{h_{x_0}}{m_0}(e_v+c_xe_x),\qquad
\dot{\bar\Delta}_{R_0}=h_{R_0}(e_\Omega+c_Re_R),
$$

$$
\dot{\bar\Delta}_{x_i}
=h_{x_i}q_iq_i^T\left[\frac{e_v+c_xe_x}{m_0}
-R_0\widehat\rho_i(e_\Omega+c_Re_R)\right]
+\frac{h_{x_i}}{m_iL_i}\widehat q_i(e_{\omega_i}+c_qe_{q_i}).
$$

原笔记随后构造跟踪能量、交叉项及扰动估计误差能量，通过相消及正定矩阵条件得到 $\dot V$ 的界。这依赖相应控制项和估计更新真的存在，并受其定义的误差域、参考/扰动有界、增益条件、绳索模型和实际姿态实现约束。

当前与这套证明有以下实质差别：

- 用有界载荷 C1 积分和竖直 trim 替代了完整多通道扰动估计；没有完整自适应扰动估计状态或对应的自适应参数投影算子。代码中的 $q_iq_i^T$ 是几何投影，逐轴 `clip` 是工程限幅，两者均不能被称为完整自适应投影更新的实现。
- `coupled` 缺少移动 $q_{id}$ 的完整角速度/角加速度前馈；`reference_cascade` 进一步关闭方向反馈并加入各机保持环，改变了原误差动力学。
- 载荷力矩前馈采用不同的陀螺耦合抵消形式；当前名义姿态环可以单独推导，但不是原笔记该式的逐项复制。
- 单向张力截断、外张内部力、力和倾角限幅、滤波、70 Hz 离散采样、位姿/EKF 延迟、丢帧保持与阶段切换均进入实际闭环。
- 主机 CTBR 接口依赖机载速率环，且运输分支没有非零 $\Omega_{ic}$ 前馈；当前没有原笔记有限时间滑模力矩控制器。
- 松绳、预抬、负载接地、卸载和飞机落地属于额外的混合动力学阶段，超出了原笔记恒长绷紧绳索连续模型。

所以当前能够成立的描述是：**沿用几何建模、载荷误差与 wrench 分配的结构，采用可复现的工程实现与分阶段证据进行验证。** 当前文档不把原笔记的 Lyapunov/有限时间结论当作新参考串级和整段真实飞行的证明。

## 12. 原笔记到当前源码的索引

| 原笔记章节/概念 | 当前实现位置 | 对应状态 |
| --- | --- | --- |
| “多四旋翼—刚体载荷—无质量缆绳模型” | `slung_load_controller.py` 的 `mocap_top_surface_to_center`、`compute` | 保留挂点/绳向几何，增加上表面原点换算 |
| “运动方程（T. Lee）”中的 $SO(3),S^2$ | `PayloadStateObserver`、`SlungLoadController.compute`、`slung_reference.py` | 保留几何表示；完整约束动力学在物理仿真层求解 |
| “运动方程（Sun）”的 load-cable/UAV 分解 | 载荷核心、飞机 CTBR、物理回放的分层接口 | 用于理解力/张力关系；当前没有 NMPC 的 $\ddot\omega_i$ 输入优化 |
| “载荷控制器”位置反馈 | `SlungLoadController.compute` 的 `position_integral`、`desired_force` | C1 有界 PID，完整扰动估计未实现 |
| “载荷控制器”姿态反馈 | 同函数 `e_r`、`e_omega`、`desired_moment` | 几何误差不变，前馈形式有实质差别 |
| $\mathcal P$、最小范数解 | `_allocation_matrix`、`_internal_bias`、`equilibrium_link_units` | 保留六维分配，增加零空间外张偏置 |
| $\mu_{id}\to\mu_i,q_{id}$ | `signed_tension`、`desired_tension`、`allocated_qid` | 增加单向张力约束与请求/有效方向分别记录 |
| 沿绳/垂绳控制 | `SlungLoadController.compute` 的 `u_parallel`、`u_perp` | `coupled` 为近似；当前参考串级禁用方向反馈 |
| 动态绳向参考前馈 | 原笔记 $\omega_{id},\dot\omega_{id}$ | 核心未完整实现；`slung_reference.py` 的导数用于另一条几何位置参考链 |
| UAV 推力/期望姿态 | `GeometricCtbrController.compute`（`ctbr_controller.py`） | 保留力投影与几何姿态构造，改为 z-up |
| UAV PI/力矩/有限时间滑模 | 同函数 `body_rate_command`，机载接口 | 运输仅为姿态误差+测量速率反馈的 CTBR，没有主机完整力矩控制 |
| 原笔记未覆盖的独立收绳/预抬 | [slung_prelift_trajectory.py](../ros_ws/src/crazyswarm/scripts/slung_prelift_trajectory.py)、fleet tick | 新增几何门控、有界预抬和连续接管 |
| 原笔记未覆盖的一次绳长校准 | [slung_link_calibration.py](../ros_ws/src/crazyswarm/scripts/slung_link_calibration.py) | 稳定几何窗口的一次正向有效值修正 |
| 原笔记未覆盖的支撑残差/实测保护 | [slung_transport_guard.py](../ros_ws/src/crazyswarm/scripts/slung_transport_guard.py) | 有界竖直 trim、实际与预测间距/掉高/速率保护 |
| 原笔记未覆盖的地面接触/释放 | [slung_landing.py](../ros_ws/src/crazyswarm/scripts/slung_landing.py)、fleet landing stage | 连续下降、窗口触地确认、卸载、各机下降 |
| 原笔记连续模型到执行链验证 | `simulate_slung_handoff.py`、`slung_simulated_telemetry.py`、`test_slung_*` | 控制核心、时序、参考、故障、触地/卸载与下降的分层验证 |

各模块的回归测试与离线物理试验覆盖不同范围。源码测试可以检查公式、参考连续性和状态机行为；电机模型回放可以检验特定假设下的物理响应；真实飞行日志则反映实际硬件表现。三者需要以相同的模式、参数、反馈源、误差定义与通过判据互相核对。
