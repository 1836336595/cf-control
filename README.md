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
- `scripts/ctbr_logs/`：控制器生成的 CSV 日志目录。

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

1. `ctbr_controller`：控制频率、EKF/NOKOV 有效性检查、电压预检、日志和安全阈值。
2. `ctbr_trajectory`：轨迹模式、圆心、半径、编队边长、八字半径、速度、高度和起降时间。
3. `ctbr_controller_cf<ID>`：该飞机的质量、推力标定、位置/速度/积分增益、姿态增益和角速度限制。

当前支持的轨迹模式：

- `circle`：圆周轨迹。
- `figure_eight_triangle`：多架飞机保持等边三角形编队，整体连续绕八字；通过 `orbit_phase_rad` 分配编队顶点。八字交叉处不中停。

新增飞机时，需要同时完成三处配置：在 `crazyflies.yaml` 增加启用条目，在 `ctbr_controller.yaml` 增加同 ID 的参数块，并更新 `takeoff_vehicle_count`。不需要修改 Python 源码。

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

控制器不会把 `crazyflies.yaml` 的 `initialPosition` 当作实时状态。`R_WB` 使用 NOKOV 姿态；EKF 与 NOKOV 位置持续失配时，不再使用不可信的 EKF 平移运动学，并进入保持、降落或中止保护流程。

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
