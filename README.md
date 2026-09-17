# Crazyswarm CTBR 多机控制

本仓库基于 Crazyswarm，增加了面向 Crazyflie 的主机端 CTBR（Collective Thrust and Body Rates）控制流程。当前任务使用 NOKOV 动捕提供外部位置和姿态，使用 Crazyflie 固件 EKF 回传的平移速度和加速度参与控制，可通过一条 Crazyradio 管理多架飞机。

## 项目结构

主要文件位于 `ros_ws/src/crazyswarm`：

- `config/ctbr_controller.yaml`：全局调度参数、公共轨迹参数，以及每架飞机的质量、推力标定和 PID 参数。
- `launch/crazyflies.yaml`：飞机 ID、radio URI、`ctbr_enabled` 和轨迹相位等身份配置。
- `launch/hover_swarm.launch`：启动 Crazyswarm server、NOKOV、EKF 日志和无线通信配置。
- `launch/ctbr_controller.launch`：启动 CTBR 控制器、Path 发布和实时绘图。
- `scripts/ctbr_controller.py`：多机 CTBR 控制器、状态机、CSV 日志和安全保护。
- `scripts/ctbr_trajectory.py`：圆周和三机等边编队连续八字轨迹。
- `scripts/ctbr_visualization.py`：读取 CSV 并绘制位置、误差、姿态、速度、加速度和 CTBR 输出。
- `scripts/test_ctbr_controller_v2.py`：控制律单元测试（限制器、编队律、推力标定、状态机）。
- `scripts/test_ctbr_trajectory_smoothstep.py`：参考轨迹与航向的单元测试。
- `scripts/test_vehicle_config.py`：飞机参数块校验测试。
- `scripts/ctbr_logs/`：控制器生成的 CSV 日志目录（`.gitignore` 已排除实飞日志，只保留一份 `sample_flight.csv` 作为列格式样例）。
- `MATLAB/`：几何 CTBR 与三机编队控制律的 MATLAB 参考实现，用于与 Python 实现逐项对照。

> 实飞日志体积很大（单次飞行可达 30 MB 以上），因此不进入版本库。`ctbr_logs/sample_flight.csv`
> 是降采样后的完整飞行样例（含全部 10 个阶段和 119 列），可用于核对 CSV 列定义或离线跑绘图脚本。

## 编译

```bash
cd ros_ws
catkin_make
source devel/setup.bash
```

首次使用或修改消息、C++ 节点后需要重新执行 `catkin_make`。Python 控制器修改后只需重启对应 launch。

## 飞机配置

在 `ros_ws/src/crazyswarm/launch/crazyflies.yaml` 中为每架飞机配置唯一的 `id` 和 radio `uri`：

```yaml
- channel: 80
  id: 4
  uri: "radio://0/80/2M/E7E7E7E704"
  ctbr_enabled: true
  orbit_phase_rad: 2.0943951023931953
  orbit_yaw_mode: face_partner
```

只有 `ctbr_enabled: true` 的条目参加 CTBR 任务。飞机数量必须与 `ctbr_controller.yaml` 中的 `takeoff_vehicle_count` 一致，并且每个启用的 ID 都必须存在对应的 `ctbr_controller_cf<ID>` 参数块。

`initialPosition` 和 `type` 仍是 Crazyswarm 通用配置字段：真实飞行位置和姿态来自 NOKOV，`initialPosition` 不会替代动捕状态；`type` 主要用于 Crazyswarm 的机型/默认参数选择，不是 CTBR PID 参数来源。

## 参数配置

`ctbr_controller.yaml` 分为三层：

1. `ctbr_controller`：控制频率、EKF/NOKOV 有效性检查、电压预检、编队控制律、日志和安全阈值。
2. `ctbr_trajectory`：轨迹模式、圆心、半径、编队边长、八字半径、速度、高度和起降时间。
3. `ctbr_controller_cf<ID>`：该飞机的质量、推力标定、位置/速度/积分增益、姿态增益和角速度限制，以及该机的编队增益。

当前支持的轨迹模式：

- `circle`：圆周轨迹。
- `figure_eight_triangle`：多架飞机保持等边三角形编队，整体连续绕八字；通过 `orbit_phase_rad` 分配编队顶点。八字交叉处不中停。

新增飞机时，需要同时完成三处配置：在 `crazyflies.yaml` 增加启用条目，在 `ctbr_controller.yaml` 增加同 ID 的参数块，并更新 `takeoff_vehicle_count`。不需要修改 Python 源码。

### 控制律适用范围

| 阶段 | 使用的控制律 |
| --- | --- |
| `takeoff` / `height_correction` / `circle_entry` / `landing` | 几何 PID 外环（`position_gain` / `velocity_gain` / `integral_gain`） |
| `figure_eight` / `final_hover` | MATLAB `leader_follower` 编队律（`formation_*` 增益） |

终点悬停**继续使用编队控制律**，这样从八字切换到悬停时控制律和积分状态都连续：
编队积分里积累的推力补偿会直接延续，不需要由几何 PID 重新建立，避免切换后长时间下沉。
降落阶段必须切回几何 PID 并解散编队。

### 编队控制律

编队阶段使用带虚拟中心积分项的分布式 leader-follower 律：

```text
u_i = p̈_c - k_f·Σa_ij·e_ij - k_vf·Σa_ij·(η_i-η_j)
          - k_l·b_i·e_i - k_vl·b_i·η_i - k_il·b_i·e_ic
F_i = m_i·(u_i + g·e_3)

e_ij = (p_i-p_j) - (r_i-r_j)      相对编队误差
e_i  = p_i - p_c - r_i            相对虚拟中心位置误差
η_i  = v_i - ṗ_c                  相对虚拟中心速度误差
e_ic = ∫(η_i + c1·e_i)dτ          积分项
```

与 MATLAB 一致，编队阶段**不使用倾角锥限制**：姿态直接由未限幅的合力构造
（`b3c = -A/‖A‖`），只对集体推力做标量裁剪。如果在编队阶段限制水平力，其上限会依赖
逐机不同的竖直力，导致三架飞机饱和程度不一致并拉开队形。可选的三维统一安全阀
`formation_max_accel_mps2` 默认关闭（`0`），开启时各轴同比例缩放，不改变合力方向。

`formation_kf` / `formation_kvf` / `formation_kbl` / `formation_kvl` / `formation_kil`
与 `position_gain` 一样**逐机可调**，写法也一致：标量表示 xyz 相同，3 元列表表示
world xyz 逐轴独立。这 8 个增益（含 `formation_integral_c1`、
`formation_integral_limit_m` 和逐机标量 `formation_bl`）**只在各自的
`ctbr_controller_cf<ID>` 块里声明**，全局块不再提供默认值：一旦某架漏写，参数解析
会静默落到标称值而不报错，三架很容易拿到不一致的增益（`formation_kil` 的全局默认曾
与三架实际值相差 30 倍）。新增飞机时必须在本机块中写全这 8 个键，
`test_vehicle_config.py` 会校验。

`formation_bl` 是 MATLAB 的虚拟中心 pinning 权重 $b_i$，表示该机被锚定到编队中心的
强度（$b_i=0$ 表示只靠邻居相对项跟随）：它原本是全局的位置数组，下标对应飞机在
`crazyflies.yaml` 里的启动顺序，调换顺序就会静默串位；现在下放到机块后按 CF ID 索引，
与顺序无关。

只有 `formation_adjacency`（MATLAB $a_{ij}$ 通信图）是全局参数，它描述机间拓扑，
不逐机覆盖。

### 机头朝向

`ctbr_trajectory.figure_eight_heading` 控制八字阶段的机头方向：

- `center`（默认）：机头恒定指向虚拟编队中心。由于每架的编队偏移是常量，指向中心
  等价于朝向 `-offset`，因此航向恒定、`yaw_rate` 恒为 0，姿态环负担最小，入轨也无
  航向跳变。
- `velocity`：复现 MATLAB 的 `b1d`（水平速度方向）。机头随轨迹切线旋转，整圈约转
  288° 且入轨瞬间有约 45° 跳变。

### 推力标定

每架飞机有独立的 `thrust_scale_correction`，用于补偿电机/螺旋桨/推力曲线的偏差，
与电压补偿**相乘**后写入 `thrust_raw_scale`。取值方法：悬停时实测
`k = m·g / ‖F_desired‖`（期望合力包含了编队律的竖直补偿），修正量取 `1/√k`，因为
桥接层按 `F ∝ raw²` 映射。

注意 `hover_swarm.launch` 中的 `ctbr_thrust_raw_scale_min/max` 校验的是**合并后**的
缩放值，因此该窗口必须同时覆盖电压补偿和标定修正；改动标定值后要一并核对。

编队积分增益 `formation_kil` 的取值需要让积分自身的拐点 `√kil / 2π` 明显低于编队
位置环的共振频率（约 `√kbl / 2π`），否则积分会放大环路的振荡而不是抑制它。

## 运行

先启动 Crazyswarm server、NOKOV 和 EKF 日志：

```bash
roslaunch crazyswarm hover_swarm.launch
```

再启动 CTBR 控制器。真实飞行必须显式确认：

```bash
roslaunch crazyswarm ctbr_controller.launch \
  target_confirmed:=true
```

只检查参数、话题或绘图时，不发送控制输出：

```bash
roslaunch crazyswarm ctbr_controller.launch \
  target_confirmed:=false enable_realtime_visualization:=false
```

`target_confirmed` 未设为 `true` 时，控制器拒绝真实推力输出。起飞前控制器会等待启用飞机的 EKF 与 NOKOV 位置连续对齐，并进行一次电池电压预检。EKF 状态持续失效、NOKOV 状态失效或通信超时会触发受控降落或全局中止。

## 数据来源和控制逻辑

- NOKOV：位置 `p` 和姿态 `R_WB`。
- Crazyflie EKF：平移速度和加速度；控制器对速度进行因果二阶低通，并由滤波结果得到加速度。
- CTBR 外环：根据轨迹位置、速度、加速度和 jerk 计算期望合力，再生成期望姿态和机体角速度。
- 每架飞机使用自己的 `ctbr_controller_cf<ID>` 标定和 PID 参数。
- 编队阶段由多机管理器统一求解各机的期望加速度，再分发给每架飞机，因此同一个控制周期内三架飞机使用同一份同步状态。

控制器不会把 `crazyflies.yaml` 的 `initialPosition` 当作实时状态。`R_WB` 使用 NOKOV 姿态；EKF 与 NOKOV 位置持续失配时，不再使用不可信的 EKF 平移运动学，并进入保持、降落或中止保护流程。

## 单元测试

测试不依赖 ROS 运行时（导入时替换 ROS 消息类型），可直接运行：

```bash
cd ros_ws/src/crazyswarm/scripts
python3 -m pytest test_ctbr_controller_v2.py \
                  test_ctbr_trajectory_smoothstep.py \
                  test_vehicle_config.py
```

覆盖范围包括：几何 PID 与编队律的阶段切换、编队积分器的累积/限幅/重置、
编队增益的逐机与逐轴解析、推力标定与电压补偿的组合、限制器的方向保真、
参考轨迹与航向连续性，以及飞机参数块校验。

## CSV 日志和绘图

控制器日志保存到：

```text
ros_ws/src/crazyswarm/scripts/ctbr_logs/
```

多机日志包含 `vehicle_id`，同一个 CSV 可按 CF2、CF4、CF5 等飞机分别绘图。启动 launch 时默认开启实时绘图；也可以离线绘制最新日志：

```bash
python3 ros_ws/src/crazyswarm/scripts/ctbr_visualization.py \
  --log ros_ws/src/crazyswarm/scripts/ctbr_logs/<log>.csv
```

可视化内容包括位置轨迹、位置误差、姿态跟踪、姿态误差、速度、动捕/滤波加速度和 CTBR 输出。RViz 中的 `nav_msgs/Path` 由控制器按飞机 ID 发布，可用于查看各机实际路径。

## 安全检查

真实飞行前确认：

- NOKOV 已识别所有刚体，名称/ID 与 `crazyflies.yaml` 一致。
- 每个 radio URI 唯一，且使用同一 channel。
- `ctbr_enabled` 数量与 `takeoff_vehicle_count` 一致。
- 每个启用 ID 都有完整的 `ctbr_controller_cf<ID>` 参数块。
- 已确认起飞区域、螺旋桨安装、电池电压和急停方式。
- 首次调参使用较低轨迹速度，并保留 `target_confirmed:=false` 做空载检查。

本项目仍保留上游 Crazyswarm 的通用 API 和仿真能力。上游文档见 [Crazyswarm documentation](https://crazyswarm.readthedocs.io/en/latest/)，新项目也可参考 [Crazyswarm2](https://imrclab.github.io/crazyswarm2/)。
