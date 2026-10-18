# 运输参数缓存与主机链路计时

## 改动

`transport_attitude_gain`、`transport_rate_gain` 在单机节点启动时读取，校验为有限、非负三维向量并保存副本。运输循环不再每个 tick 查询 ROS 参数服务器，消除三机 70 Hz 下最多每秒 420 次的重复查询。负载 Path 发布间隔也改为启动缓存。修改这些 YAML 参数需重启控制器，现有增益和控制公式保持不变。

使用 `time.perf_counter()` 记录 Python 执行耗时，继续使用原来的 `time.monotonic()` 计算轨迹时钟和 dt。完整周期结束前已经写出 CSV，因此完整耗时明确记录为上一周期，并附上该周期开始的控制时间。当前单机几何计算耗时另外记录。

新增主机 `/cf<ID>/ctbr_timing`，只记录已接受的正常命令或首个解锁包，不添加 radio 遥测包。命令发布到 C++ 回调入口的时间与 `sendSetpoint()` 调用耗时分开测量。发送调用包含 radio mutex、USB、传输和 ACK/回调等待，不代表飞机执行指令的单向延迟。

## CSV 字段

| 字段 | 含义 |
| --- | --- |
| controller_tick_interval_s | 本次单机 callback 与上次起点之间的实际间隔，未裁剪 |
| controller_compute_wall_s | 本次单机几何控制器 compute 耗时；未计算时 NaN |
| previous_vehicle_cycle_wall_s | 上一完整单机 callback 耗时，包含日志写入 |
| previous_vehicle_cycle_control_time_s | 上一单机 callback 起点对应的 control_time_s |
| fleet_tick_interval_s | 本次多机 callback 与上次起点之间的间隔 |
| previous_fleet_cycle_wall_s | 上一完整多机 callback 耗时，包含负载控制、安全检查、三机计算和日志 flush |
| previous_fleet_cycle_control_time_s | 上一多机 callback 起点对应的 control_time_s |
| published_command_stamp_ns | 最近发布命令的 ROS 时间戳，纳秒整数 |
| bridge_latest_command_stamp_ns / command_seq | 最近异步桥接报告对应的原命令戳/序号 |
| bridge_latest_command_age_s | 原命令发布戳到 C++ 回调入口的时间 |
| bridge_latest_callback_wall_s | C++ 回调入口到诊断构造的耗时，包含发送调用，不含报告本身发布 |
| bridge_latest_send_call_wall_s | sendSetpoint 调用耗时，包含锁/USB/ACK |
| bridge_latest_raw_thrust | 该发送报告对应的 raw PWM |
| bridge_latest_accepted / unlock_packet | 接受/首个解锁零包标志 |
| bridge_latest_receipt_age_s | 最近发送报告到达 Python 后的年龄 |
| bridge_latest_matches_published_stamp | 报告戳与最近发布戳相同的提示；不是唯一命令 ID 校验 |

`bridge_latest_*` 经常属于上一条命令，不能把它无条件当成本行刚发布指令的响应。需要按原命令戳及显式回显序号关联。ROS 序列化可能重写顶层 Header.seq，所以诊断额外提供 command_seq。NaN 表示没有可用报告；零不能替代缺失值。

既有 `state_age_s`、`mocap_sample_age_s` 与 `ekf_state_age_s` 仍可用来分析收到的数据是否变旧，但它们不覆盖相机/NOKOV 内部采集处理和完整固件 EKF 估计延迟。

## 验证

- 150 项 Python 控制器回归测试通过；新增测试先复现运行期参数查询、无效增益未提前拒绝和缺少时序字段，再验证缓存、原子报告、早退路径及 dt/控制输出一致。
- C++ server 与新的 Python 消息绑定编译成功。编译器仍有原第三方 objecttracker 的格式警告，本次未修改第三方实现。
- Python 3.8 真实 ROS 消息序列化检查通过：报告顶层 header.seq 改写后，显式 command_seq 和命令纳秒戳仍正确。
- 70 Hz Mellinger + 50 Hz EKF/5 Hz 主机滤波回放完整通过接管后 60 秒，最小机间距 0.348803 m，最低负载间隙 0.328759 m，最大角速度分量 0.017073 rad/s，饱和率 0%。物理控制结果与缓存修改前的 filtered_nominal 一致。
- 回放 CSV 的桥接字段保持 NaN，未把没有真实 radio 的仿真伪装成已测量无线电。

本机该次离线回放中的单机 compute 中位耗时约 0.176 ms、完整多机周期中位约 2.43 ms，仅反映本机仿真负载。实机优化效果需要重启新 server/控制器后从新 CSV 测量；本次未调整 radio 轮询节奏或外环频率，未启动真实飞机。

## 使用

本工作区已编译。重新 `source /home/csy/my_project/crazyswarm/ros_ws/devel/setup.bash` 并重启 server 和控制器后，新诊断会自动进入后续 CSV。

可用 `rostopic echo -n 5 /cf3/ctbr_timing` 查看话题，CF4/CF5 同理。若 command_age_s 大而 send_duration_s 小，优先查 ROS 队列/轮询等待；若 send_duration_s 大，继续查共享 radio 锁、USB/ACK/重传。不能只靠此二者推算机载执行延迟。
