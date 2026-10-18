# 无线电速率与反馈延迟的隔离检查

当前 `crazyflies.yaml` 三架机的 URI 都为 `radio://0/80/2M/...`。本地 Crazyradio 驱动只列出 250 Kbps、1 Mbps、2 Mbps 三档，当前已经使用最高的 2 Mbps 档位；无法通过把 URI 改成更大的数字提高速率。

先前把定位/命令延迟从 10 ms/5 ms 同时改为 20 ms/10 ms，不能单独归因到 radio。为区分两项影响，本次保持 Mellinger 控制参数、50 Hz EKF、20 ms EKF 延迟、5 Hz 主机滤波、70 Hz 控制及 60 秒观察窗口相同，分别只增大一个延迟。

| 记录 | 位姿延迟 | 命令延迟 | 接管后时长 | 结果 |
| --- | ---: | ---: | ---: | --- |
| ekf_feedback_20261016/filtered_nominal | 10 ms | 5 ms | 60.0 s | 成功 |
| command_only | 10 ms | 10 ms | 60.0 s | 成功 |
| pose_only | 20 ms | 5 ms | 10.74 s | distance 保护 |
| ekf_feedback_20261016/filtered_delay | 20 ms | 10 ms | 8.56 s | distance 保护 |

`command_only` 最小机间距 0.348798 m，最大角速度分量 0.017343 rad/s，饱和率 0%。`pose_only` 最小机间距降至 0.294645 m、最大角速度分量约 2.5847 rad/s，触发保护。

这说明在当前确定性模型中，位姿反馈延迟比 5–10 ms 范围的命令延迟更敏感；不能据此宣称真实无线电没有影响。`pose_delay_s` 和 `command_delay_s` 是模型设定，均未由机载响应测量标定。实机的主机 NOKOV 位姿来自服务端发布，主机位姿路径本身不经过 Crazyradio；外部 pose 注入固件 EKF 则会使用 radio。

## 已查到的延迟来源

- 三架机共用 radio 0/channel 80，单播和广播在共享 radio mutex 下串行。
- Crazyradio 的单包路径包含同步 USB 发送和 ACK 读取；没有空口速率参数可以消除这些等待。
- 服务端 `runSlow()` 在每轮发送三机 ping 后处理 ROS 回调，并睡眠 10 ms。增加轮询频率会减少部分等待，但也增加 ping/USB 负载，需要实际测量后调整。
- 主机速度与角速度滤波会产生反馈时滞，与无线电空口位速率不同。
- `ctbr_controller.py` 的 `_transport_overrides()` 每架机每个控制 tick 用 `rospy.get_param` 读取两项运输姿态/速率增益，70 Hz 三机最多每秒 420 次参数读取。当前实现没有使用 `get_param_cached`；本地 rospy 源码在缓存不存在时执行 XML-RPC 查询。仿真使用内存里的假参数服务器，未体现这一运行耗时。应优先改为启动时读取/校验，并记录其优化前后的周期耗时，不能仅凭源码认定其耗时大小。

## 下一步

保持现有 2M 档位，先测量 NOKOV/ROS 位姿到控制器的数据年龄、控制回调耗时，以及 CTBR 发布到 C++ 排队/发送的时间。原 CSV 中 `command_raw_thrust_age_s` 只代表桥接结果回调的年龄，不能充当“飞机已执行指令”的端到端延迟。

随后优先消除控制周期中的远程参数查询、检查回调队列和滤波相位，再以较大反馈延迟下完整 60 秒稳定为验证目标。此次没有更改无线电或实飞控制参数。
