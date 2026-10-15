# CF3 电池预检与无线电链路诊断

分析输入：`multi_ctbr_20261015_210135.csv`、`multi_ctbr_20261015_210324.csv`，以及对应 ROS 日志目录 `72020e06-c898-11f1-9404-75f142f0cea6`、`b6e4f29a-c898-11f1-9404-75f142f0cea6` 的 `rosout.log`。

## 已确认的现象

- 两次运行 CF3 都连续等待 10 秒，没有任何通过校验的电池样本；CF4、CF5 均完成电压和 EKF 预检，进入 `waiting_for_fleet`。
- CF4、CF5 的 `battery_age_s` 多次下降，说明周期电池遥测确实在到达。不能用冻结后的 `preflight_voltage_sample_count=1` 推断整个运行只收到一包。
- 两次 CSV 中三机 NOKOV 状态全部有效，最大状态年龄约 0.0162 秒。因此本次直接失败条件不是主机侧 NOKOV 刚体定位丢失。
- 用户报告的 21:01 运行中，CF3 自 ROS 时间 1792069277.7197 开始反复输出 `ESTKALMAN: State out of bounds, resetting`，约每两秒一次；电压预检于 1792069305.9608 失败。21:03 运行也有同样现象。它表明 CF3 固件估计器同时异常，但不能据此断言估计器重置导致电池日志缺失。
- 完整日志中，低链路质量告警在电压预检失败前就已出现。日志没有发现控制器“忽略无效电池遥测”的告警。

## Link Quality 的含义与限制

`Crazyflie.cpp` 的 `sendPacketInternal()` 统计发送调用是否收到 radio ACK，每 100 次形成一个窗口。该指标不是 RSSI，也不是电池日志的接收率。

当前实现把 `numPackets`、`numAcks` 定义为函数静态变量，多个 Crazyflie 实例、线程共享计数，且计数更新没有完整的同步保护。因此窗口可能混入其他飞机的数据；告警中的 CF 名称不能可靠地代表该机独立链路质量。

服务端的低 Link Quality 回调只输出警告。触发全机零推力的是控制器未收到有效电池样本的预检超时，而不是 Link Quality 数值本身。无线电丢包可以导致遥测缺失，但目前还不能排除 CF3 固件日志块、设备状态等其他问题。

## 源码与运行产物不一致

源码已经改成 `ENABLE_SAFELINK=1`、两处 `setArc(3)`。但本次 roslaunch 启动的是本工作区 `devel/lib/crazyswarm/crazyswarm_server`；`ldd` 解析到 `devel/lib/libcrazyflie_cpp.so`。

反汇编该库确认：Crazyflie 和 CrazyflieBroadcaster 初始化时仍向 `setArc()` 传入 0，尚未包含源码中的 `setArc(3)`。因此仅重启 launch 不能验证这些 C++ 修改的效果。不能把尚未编译的 SafeLink 修改当成本次故障的已验证原因或解决方案。

## 下一步地面隔离

先仅启动 server，观察三机话题，不启动已确认起飞的控制器：

```bash
rosparam get /crazyswarm_server/genericLogTopicFrequencies
rostopic hz /cf3/battery
rostopic hz /cf3/ekf_kinematics
timeout 10s rostopic echo -n 1 /cf3/battery
```

日志周期应为 `[20, 500]` 毫秒，即 EKF 目标 50 Hz、电池目标 2 Hz；实际接收率受链路影响。对 CF4、CF5 运行相同命令比较。

优先对 CF3 完整断电重启、静止放置，再检查上述话题和固件估计器重置消息。若仍只有 CF3 异常，单独连接 CF3 做同样观察，以区分单机故障与三机共享无线电负载；单机试验应只启用该机的 server 配置，不能通过放宽多机预检让其他飞机带故障起飞。

需要应用 C++ 通信改动时必须重新编译，但 SafeLink 和自动重传应分别验证。当前源码保留了 SafeLink 历史丢包问题的注释；不能把同时启用两项设置视为已经验证的通信修复。

本次仅保存诊断证据；未改变控制增益、日志频率或预检门槛，也没有启动真实飞行。检查时 ROS master 已退出，无法直接测量当前在线话题频率。
