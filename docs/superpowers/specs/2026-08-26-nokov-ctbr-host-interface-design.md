# Nokov 状态与 CTBR 主机接口

## 范围

为外部 Crazyflie 控制器增加一个使用 Nokov 刚体测量数据的主机端 ROS 接口。控制器
本身明确不在本次范围内：本次改动只发布测量状态，并接收 CTBR 指令。

已烧录到飞机中的 Crazyflie 固件不会被修改或重新烧录。主机使用现有的 legacy RPYT
CRTP 指令通道，以及固件已有的角速度模式参数。

## 接口

每架已配置的 Crazyflie 发布：

```
/cf<ID>/mocap_state  crazyswarm/MocapState
```

`MocapState.msg` 内容如下：

```
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Vector3 acceleration
bool valid
bool derivatives_valid
```

`header.frame_id` 固定为 `world`。位置、姿态、线速度和线加速度均在 `world`
坐标系中表达；`twist.angular` 是 Crazyflie 机体系中的角速度。线性量的单位为米、
秒及其 SI 导数，角速度单位为 `rad/s`。

只有 Nokov 成功跟踪到刚体时，`valid` 才为真。只有累计足够的连续帧以计算速度、
角速度和加速度后，`derivatives_valid` 才为真。长时间帧间隔或无效时间间隔会重置
微分历史。

飞行前必须使 Nokov 刚体定义与 Crazyflie 机体系对齐。第一版假定两者已经对齐；在
允许非零 CTBR 指令飞行前，必须通过受限台架测试验证 roll、pitch、yaw 三轴正负号。

每架已配置的 Crazyflie 还订阅：

```
/cf<ID>/cmd_ctbr  crazyswarm/CTBR
```

`CTBR.msg` 内容如下：

```
std_msgs/Header header
geometry_msgs/Vector3 body_rates
float32 collective_thrust
```

`body_rates` 是 `[p, q, r]`，单位为 `rad/s`；`collective_thrust` 是整机总推力，
单位为 N。对于当前飞机，`ctbr_max_thrust_newton` 默认值为 `1.176798` N，即总静态
最大推力为 120 gf。主机负责将该指令转换为固件 legacy 角速度命令所需的 `deg/s`，
并映射到原始推力范围 `0..60000`。

## 数据流

```
Nokov -> libmotioncapture -> CrazyflieGroup::runFast()
      -> /cf<ID>/mocap_state -> 后续外部控制器
      -> /cf<ID>/cmd_ctbr -> CrazyflieROS -> legacy RPYT CRTP packet
      -> 已有机载角速度控制器和电机混控
```

现有的 `crazyswarm_server` Nokov 连接仍然是飞行时唯一使用的连接。`mocap_helper`
仍只作为终端调试工具，`hover_swarm.launch` 不会启动它。

测量状态发布器属于每个 `CrazyflieROS` 对象。在获得对应刚体后，由
`CrazyflieGroup::runFast()` 立即调用。这使刚体关联关系和每架飞机的微分历史均
封装在同一个对象中。

## 微分估计

第一版从相邻的有效 mocap 帧中计算导数量：

- 线速度为位置差除以帧间隔；
- 机体系角速度为四元数增量转换为轴角向量后除以帧间隔；
- 线加速度为滤波后线速度之差除以帧间隔；
- 对导数量使用可配置的一阶低通滤波器。

初始版本以主机收到帧的时间作为时间戳来源，避免依赖 Nokov `iTimeStamp` 未说明的
单位。时间间隔为非正、无效或过大时，重置历史数据，而不产生异常的微分尖峰。

## CTBR 桥接与安全性

`CrazyflieROS` 增加独立的 `cmd_ctbr` 订阅者；`/cmd_vel` 保持现有 RPYT 语义，不
复用。桥接层拒绝非有限输入，使用 ROS 参数限制角速度与总推力，完成单位转换后，
通过已有的 `Crazyflie::sendSetpoint()` 转发。

launch 配置在运行时设置固件已有参数：

```
flightmode.stabModeRoll: 0
flightmode.stabModePitch: 0
flightmode.stabModeYaw: 0
```

已烧录固件中 `0` 表示 RATE。Crazyswarm 在服务器启动时，经由无线电参数接口写入
这些值，不需要刷写固件。legacy commander 的推力锁要求先收到零推力包，因此桥接层
在接受非零 CTBR 推力前必须先发送零推力包。桥接层将物理推力范围映射到 `0..60000`；
此线性模型是暂时的保守映射，后续可替换为与电池电压相关的推力标定，而无需修改
控制器接口。

流式 CTBR 指令的优先级高于 `takeoff()`、`goTo()` 等高层指令。后续控制器必须持续
发布指令；在恢复高层模式前，必须调用 `notify_setpoints_stop()`。

## 控制器边界

本次不包含控制器源文件、launch 节点、轨迹生成器或自动 CTBR 发布器。后续控制器
只需订阅 `/cf<ID>/mocap_state`，并按其控制频率发布 `/cf<ID>/cmd_ctbr`。

## 验证

- 构建 catkin 工作区，并确认消息已生成；
- 在不连接无线电的情况下启动，确认消息和话题已注册；
- 连接 Nokov 后，手动移动被跟踪刚体，检查状态话题的时间戳、位姿、导数有效性和
  导数单位；
- 使用单元测试验证 CTBR 输入校验和限幅。硬件 CTBR 飞行不属于本次改动，必须在
  独立的台架和系留测试流程中完成。
