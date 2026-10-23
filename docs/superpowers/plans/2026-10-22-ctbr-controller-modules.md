# CTBR 参数与控制器模块拆分计划

**目标：** 实飞 launch 不加载仿真参数；按独立飞行、负载、吊运单机三种控制职责组织公式，保持当前计算结果。

**约束：** 保留用户未提交改动、全部主机对照分支、当前 Mellinger 模型及参数数值。Python 3.8 兼容；不运行 ROS 飞行节点或 ARM，不提交 Git。

## 参数

- 将 `slung_payload.simulation` 和仅回放使用的 `post_handoff_s` 移到 `config/slung_simulation.yaml` 的 `slung_simulation` 根。
- 实飞 `hover_swarm.launch`、`ctbr_controller.launch` 不加载该文件；仿真显式读取它。
- 仿真候选覆盖使用独立的 `slung_simulation` 根；控制增益覆盖仍使用原控制参数根。
- 输出快照保留独立仿真根，数值与重构前相同；验证配置覆盖不会修改实飞文件。

## 控制公式

- `ctbr_independent_controller.py`：独立飞行位置 PID、预抬/放载后的 Z 积分、独立姿态命令。
- `slung_load_controller.py`：负载位置/姿态、张力分配和绳索控制，保留现有模块。
- `ctbr_transport_vehicle_controller.py`：运输位置反馈、专用积分、接管混合及运输姿态命令。
- `ctbr_geometric_controller.py`：组合独立与运输计算，保留 `GeometricCtbrController` 和 `ControllerConfig` 接口。
- `ctbr_control_math.py`：共同坐标/姿态/力限幅和解析前馈函数，无 ROS 依赖。

## ROS 入口与调度

- `ctbr_controller.py`：启动入口和原公开接口重导出；不存放算法或大型节点实现。
- `ctbr_vehicle_node.py`：单机订阅、预检、任务时钟和指令发布。
- `ctbr_fleet_node.py`：机群同步、安全恢复和统一控制 tick。
- `ctbr_payload_parameters.py`：负载参数读取及初始化。
- `ctbr_payload_control.py`：负载参考生成和控制覆盖。
- `ctbr_payload_mission.py`：负载观测、预抬/接管和降落状态机。
- `ctbr_csv_logger.py`、`ctbr_vehicle_logging.py`：CSV 文件及逐机诊断写入。
- `ctbr_parameters.py`、`ctbr_state_feedback.py`：参数路由与反馈过滤；任务几何工具统一放入 `ctbr_control_math.py`。

## 执行和验证

- [x] 保存现有代码/配置基线到 `/tmp/ctbr_module_refactor_baseline_20261022`。
- [x] 参数拆分：先编写实飞不含仿真设置、独立覆盖和原值等价测试，观察失败后迁移。
- [x] 算法拆分：先比较多阶段命令与积分历史；新模块无需 ROS 即可导入。
- [x] ROS 模块迁移：机械搬移方法，显式导入依赖；测试工具重新加载使用 ROS stub 的模块，修正异常注入测试的真实依赖位置。
- [x] 执行 `python -m pytest -q test_ctbr*.py test_slung*.py test_native_position_pid*.py test_transport_native_pid_equivalence.py`，994 passed / 1 skipped。
- [x] 用真实 Noetic 配置加载器离线读取两个 launch 并 XML-RPC 编码，确认不含 `/slung_simulation`；不启动节点。
- [x] 用原 udaan 环境运行 `simulate_slung_handoff.py --output ctbr_logs/controller_modules_20261022/figure_eight`。
- [x] 与 `ctbr_logs/mellinger_only_cleanup_20261022/figure_eight/simulation_trace.csv` 逐行比较，29,802 行、74 列全部一致；完整八字、100 s 观察及降落停推通过。
- [x] 更新 README 的文件职责和参数入口，记录测试及对照结果。
