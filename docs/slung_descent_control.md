# 协同放载前馈约束与独立降落 Z 增益

> 2026-11-18 14:33 实飞复查后已增加第二轮处理：协同下降 2 s 内将负载旋转反馈平滑降至 0.30，飞机姿态差分前馈平滑退出至 0；详见 §5。第一轮的前馈限幅/滤波和降落 PID 分离保留。

本次修改针对 `multi_ctbr_20261118_133350.csv` 中的两个现象：CF4/CF5 在协同放载末段出现约 2.9 Hz 姿态振荡，CF4 在释放负载后又长时间保持在地面目标上方约 10 cm。它们发生在不同控制阶段，使用两项分别生效的控制处理。

## 1. 协同放载只处理姿态差分前馈

在 `flight_validation_landing_stage=payload_lowering` 且仍为运输控制时，启用以下配置；该阶段包含负载下降、触地等待和 2 s 卸载：

```yaml
# config/slung_payload.yaml
slung_payload:
  landing_attitude_feedforward_enabled: true
  landing_attitude_feedforward_filter_tau_s: 0.10
  landing_attitude_feedforward_limit_rps: [0.6, 0.6, 0.5]
  landing_attitude_feedforward_gain: 0.0
  landing_payload_attitude_feedback_scale: 0.30
```

处理入口为 [ctbr_transport_vehicle_controller.py](../ros_ws/src/crazyswarm/scripts/ctbr_transport_vehicle_controller.py) 的 `_transport_log_difference()` 与 `_shape_transport_landing_feedforward()`。

原始目标角速度仍由最终合力形成的目标姿态差分计算：

$$\omega_k^{raw}=\operatorname{Log}(R_{c,k-1}^{T}R_{c,k})^\vee/\Delta t_R.$$

先在期望体系逐轴限制输入，防止大尖峰给低通状态充入过多量；再转到世界系低通：

$$b_k=\operatorname{clip}(\omega_k^{raw},-L,L),\qquad
w_k=w_{k-1}+\alpha_k(R_{c,k}b_k-w_{k-1}),\quad
\alpha_k=1-e^{-\Delta t_R/\tau}.$$

输出为

$$\omega_k^{applied}=g_{FF}(t)\operatorname{clip}(R_{c,k}^{T}w_k,-L,L).$$

其中 $g_{FF}$ 按 §5 平滑调度；滤波内部历史保存未乘该权重的有界值，避免将权重重复累乘。

最后一次裁剪避免旋转目标坐标系使输出突破逐轴界；滤波状态同时按该输出更新。世界系滤波保证相邻目标体系不同的角速度分量经过正确变换后才混合。

姿态 P、已有有界姿态 I 和最终命令限幅仍独立计算。该处理不平滑整条最终角速度命令，避免额外延迟姿态误差的直接修正。它不是增加速率阻尼，也没有把负载速度测量加入 velocity-free 外环。

控制时钟拥有差分和低通的 dt，不能使用负载缓存帧的时间差。首次/复位/退化/倒序/大于 100 ms 间断不产生新的前馈，并清除低通历史；重复时刻不推进、不重复发送旧滤波值。切到独立下降、退出运输或关闭处理后清理滤波状态。正常悬停、起飞、普通降落与非下降任务仍使用原分支。省略 enabled 的旧配置默认不启用，当前实飞 YAML 显式开启。

## 2. 放载后独立下降使用每机专用 Z PID

配置在 [ctbr_vehicle.yaml](../ros_ws/src/crazyswarm/config/ctbr_vehicle.yaml)：

| 飞机 | 共用 independent Z：P / D / I | 放载后 landing Z：P / D / I |
|---|---|---|
| CF3 | 4 / 3.6 / 0.8 | 4 / 3.6 / 0.8 |
| CF4 | **恢复为 4 / 3.6 / 0.8** | **5 / 4 / 1.6** |
| CF5 | 4 / 3.6 / 0.8 | 4 / 3.6 / 0.8 |

每机使用三个标量，顺序清楚区分 P、D、I：

```yaml
ctbr_controller_cf4:
  independent_position_gain: [4.0, 4.0, 4.0]
  independent_velocity_gain: [3.6, 3.6, 3.6]
  independent_integral_gain: [0.80, 0.80, 0.80]
  landing_position_gain_z: 5.0
  landing_velocity_gain_z: 4.0
  landing_integral_gain_z: 1.60
```

仅在 `independent_mode=true` 且 `landing_vertical_integral_active=true` 时覆盖 Z。实飞任务层在负载释放后的 `vehicle_descent` 阶段设置这个标记；XY 仍保留原独立/预抬保持倍率。未配置新标量时逐项兼容原 independent 增益；配置值必须是有限非负标量。

有效增益参与实际出力、Z 积分反饱和判断及积分力诊断，不只是修改显示字段：

$$F_z=m\left(g+a_{d,z}-K_{p,z}^{land}e_z-K_{d,z}^{land}e_{v,z}-K_{I,z}^{land}I_z\right).$$

现有积分状态限值、更新速率、反馈加速度限值以及停推高度/速度门槛继续生效。CF4 将降落 I 从此前实飞的 1.2 提至 1.6，加强持续近地偏差修正；此值仍是待实飞核对的候选，不能承诺落地时间。

起飞、预抬和运输 native PID 不使用 landing 覆盖，CF4 运输名义 Z 积分增益随共用基础值回到 `0.8 × 3 = 2.4`。因此降落调参不再连带增大悬吊运输积分。

## 3. 新日志

新增 CSV 列均追加，不修改旧列语义：

| 字段 | 含义 |
|---|---|
| `transport_landing_feedforward_active` | 本拍是否启用放载前馈约束 |
| `transport_landing_feedforward_limit_active` | 本拍输入或转回目标体系后的输出是否触发限幅 |
| `transport_landing_feedforward_filter_tau_s` | 当前实际滤波时间常数 |
| `transport_landing_feedforward_raw_x/y/z` | 原始目标体系姿态差分前馈，rad/s |
| `transport_landing_feedforward_applied_x/y/z` | 最终使用的目标体系前馈，rad/s |
| `landing_pid_active` | 独立下降 Z PID 路径是否启用 |
| `landing_pid_gain_p_z/d_z/i_z` | 本拍使用的实际 P/D/I 标量 |

`computed_rate_*` 仍表示实际使用的目标角速度；查看尖峰来源时，应同时比较新增 raw/applied 列。停推/零输出行的 PID 增益可为空值（NaN），不能由它推断空中实际使用了零增益。

## 4. 已完成的验证与边界

当前脚本目录完整回归为 **1442 passed、4 skipped**（`python3 -m pytest -q`）；Python 3.8 语法、配置 XML-RPC 与差异格式检查通过。pytest 排除 `ctbr_logs`，避免收集运行目录中的历史源码副本。

新增测试覆盖仅放载生效、P/I 直接路径、世界系滤波、输入及输出界、重复/倒序时刻、100 ms 间断、退化、复位、卸载、独立下降退出、参数校验，以及 CSV 实际持久化；降落 PID 测试覆盖作用阶段、兼容回退、实际出力与反饱和。旧纯控制器数值基线保留，仅显式核对并移出新增诊断字段后继续比较。

在原始实飞姿态/参考时钟上调用新前馈函数、保留原记录 P/I 重算，CF4/CF5 角速度限幅占比由约 2.96%/3.23% 变为 0。**这是固定实测状态的指令重算，不是新轨迹或实飞改善证据。**

修改前后分别做了名义和噪声/较大延迟两组完整 MuJoCo 回放，四组均完成放载、独立下降并全部停推。噪声组姿态波动对照：

| 协同放载 roll/pitch 峰峰值 | 修改前 | 修改后 |
|---|---|---|
| CF4 | 0.714° / 0.726° | 0.478° / 0.491° |
| CF5 | 0.692° / 0.679° | 0.287° / 0.298° |

修改后最小机间距约 0.519 m；协同下降采样中未出现负请求张力或角速度/力限幅。CSV 检查确认前馈只在 `payload_lowering` 生效、降落 PID 只在释放后的独立下降生效，CF4 实际读到 `5/4/1.6`。

**名义模型没有复现实飞约 23° 振荡和 CF4 近地悬停。** 上述结果用于检查现有模型下的完整流程与数值回归，不能证明实飞大振荡已消失，也不能预测 CF4 新落地时间。实际推力标定、近地效应与耦合问题仍需新日志验证。张力单向裁剪本次未改成新的约束分配算法。

可复现产物（本地运行目录，受 Git 忽略）位于 `scripts/ctbr_logs/analysis_descent_fix_20261118/`：

- 修改前后控制源码/参数独立快照与 SHA256 清单。
- `baseline_nominal/`、`baseline_noise_delay/`、`candidate_nominal/`、`candidate_noise_delay/`。
- `comparison_metrics.json`、`summarize_runs.py`、`离线验证说明.md`。
- `recorded_state_feedforward.py/.json`：实飞记录状态上的前馈重算。

本次没有启动真实 ROS/radio。配置和 Python 改动需重启控制节点后生效。


## 5. 第二轮：降低负载旋转反馈并退出力方向差分前馈

最新 `multi_ctbr_20261118_143303.csv` 确认第一轮约束确实启用；放载中前馈范数约在 0.43 rad/s 内，仍出现负载 roll 和 CF4/5 pitch 逐步增大的约 2.4 Hz 耦合振荡。负载请求 roll 力矩幅值接近 0.015 N·m，后两机的请求张力反复变负。单纯限制角速度尖峰不能消除上游负载力矩闭环的振荡。

设放载开始时刻为 $t_L$，$s=S_5((t-t_L)/T_L)$，当前 $T_L=\texttt{landing\_gain\_transition\_s}=2$ s。采用原五次曲线的端点零速度/加速度性质，平滑调度：

$$a_R(t)=1+s(0.30-1),\qquad g_{FF}(t)=1+s(0-1).$$

负载广义力中的旋转反馈改为

$$u_{c,\omega}=\Upsilon_\omega-a_R(t)(k_X\Psi_{E,\omega}+k_{\widetilde X}\Psi_{T,\omega}).$$

只缩放旋转反馈两项，保留参考惯性前馈、平移反馈、重力补偿和支撑配平；不通过过滤整个请求力矩额外增加反馈时延。辅助元状态和 β 增益仍连续，悬停及起飞保持原有效增益。降低姿态反馈会降低放载阶段的姿态恢复强度，这是为减小差分张力振荡所做的取舍，不能宣称保持了原闭环带宽。

飞机姿态差分前馈按上式的 $g_{FF}$ 退出。2 s 后由飞机姿态 P 与已学的有界姿态 I 进行姿态跟踪，不继续把反馈形成的力方向变化当成目标角速度前馈。已有前馈逐轴限幅/世界系滤波在过渡期间继续生效，原始差分值仍保留到日志中。

配置项均要求 0–1 的有限标量；省略时使用 1 兼容第一轮行为。调度的开始时刻是实际负载下降规划开始时刻，重规划会重新建立过渡；不使用全部任务的 h0。当前 YAML 显式设置 0.30 和 0。

新增 CSV 列：

- `payload_attitude_feedback_scale`：当前下发给负载核心的旋转反馈倍率；该倍率作用于 β 六维分支。
- `transport_landing_feedforward_gain`：本拍实际应用的前馈权重；应在放载约 2 s 后达到 0，同时 raw 列可继续非零。

### 第二轮验证

当前脚本目录回归：**1455 passed、4 skipped**；Udaan 环境相关专项：**93 passed**。新增测试核对旋转反馈缩放不改变平移支撑和参考惯性项、前馈为零时仍直接保留姿态 P、真实任务的连续调度、CSV 和非法参数拒绝。

除名义及噪声/较大延迟回放外，增加一个模型敏感性试验：两边起飞/运输均用原 `payload_rotational_damping=0.001`，仅在进入 `payload_lowering` 后把物理负载阻尼切为 0；其他噪声/延迟条件一致。

| 同一低阻尼试验 | 第二轮修改前 | 第二轮修改后 |
|---|---|---|
| 完整任务 | 协同下降中负载状态持续失效，进入 emergency_landing，回放脚本终止 | 完成放载、三机独立下降、全部停推 |
| 负载 roll 峰峰值 | 约 66.0°，到失效前的区间 | 约 2.97°，完整下降区间 |
| CF4 roll/pitch 峰峰值 | 约 11.09° / 10.59° | 约 0.51° / 0.56° |
| CF5 roll/pitch 峰峰值 | 约 7.12° / 7.15° | 约 0.31° / 0.32° |
| CF4/CF5 负请求张力占比 | 16.0% / 11.2% | 0 / 0 |

前后统计窗口因失败和完整完成而不同。该试验暴露出对负载阻尼假设的敏感性，不意味着实际负载阻尼已经辨识为零。尝试从起飞前就将阻尼置零的更强试验，旧控制器在运输阶段即失效，未进入下降，不能用作本次下降修复的成功证据。

第二轮名义、噪声/较大延迟和下降低阻尼三个候选回放均完成并全部停推。细节位于 `scripts/ctbr_logs/analysis_descent_v2_20261118/` 的 `validation_metrics.json`、`run_validation.py`、各运行的参数/CSV/summary，以及对比图。源码快照位于 ROS 包外 `/home/csy/my_project/backups/ctbr_descent_v2_20261118/`，避免 launch 重名。

### 动捕中断单独处理

最新实飞在下降约 5.5 s 后，三机位姿与负载源状态同时不再更新，EKF 接收年龄也增长。约 0.8 s 后触发原状态超时并全局零输出。晃动先于这次中断，但 CSV 无法证明是振荡导致遮挡、动捕系统停止还是服务端链路异常。

本次保留原超时/全局中止语义，没有延长旧状态可用时间，没有在全机失去位姿后继续盲目下降。实际改进仍需新实飞日志确认；不能以延长超时来掩盖独立的数据源中断。
