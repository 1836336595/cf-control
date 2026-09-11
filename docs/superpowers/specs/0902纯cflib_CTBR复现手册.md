# 0902 纯 cflib CTBR 飞行链路 —— 调试全记录与复现手册

> 目标：仅凭本手册 + 本仓库代码，在相同环境（Ubuntu 22.04 + CF2.1 无刷 + NOKOV 动捕 + Crazyswarm2 v1.0.7 + cflib 0.1.33）上**复现** 2026-09-02 全部调试过程并再次飞起 0.35m 悬停圆轨迹。
> 状态（2026-09-02 20:03 起）：✅ 纯 cflib CTBR 圆轨迹（R=1m @0.35m）真实飞行验证通过；心形脚本 [0902heart.py](0902heart.py) 已就绪（未实飞）。

---

## 1. 最终架构（一句话版）

```
NOKOV(XINGYING) ─SDK广播─▶ motioncapture(C++绑定, 100Hz) ─▶ 主循环
                                                     │ cf.extpos.send_extpose ─▶ 固件 Kalman
固件 EKF ─logblock 下行─▶ cflib 日志块(st/att@100Hz, 诊断入块降频)
主循环(100Hz): 几何外环(0901 geometric) → legacy RPYT send_setpoint → 固件速率内环
```

**关键前提**（全部有实测证据，违反任何一条都飞不起来）：

| # | 前提 | 证据出处（见 §6 故障表） |
|---|---|---|
| 1 | **发送通道 = legacy RPYT**（`send_setpoint`），不是 `send_setpoint_manual`（本机固件 2026.08 release **不含** TYPE_MANUAL 解码器） | chan_compare 实测 |
| 2 | 固件参数：`stabilizer.controller=1`，`estimator=2`，`locSrv.extPosStdDev=1e-3/expQuat=0.05`，`flightmode.stabMode*=0`，`commander.enHighLevel` 按阶段切换 | 0901 遗留 + 每轮回读确认 |
| 3 | **必须先 ARM 且等 supervisor 到 `IS_ARMED\|CAN_FLY`（0x020A=0x0002\|0x0008）**：未到 `canFly` 时固件把执行层 setpoint 强制清零（stabilizer.c:332-343）——未 ARM 时任何自检/飞行判断全部失真 | 19:30/19:39 实测 |
| 4 | **ARM 后自检的 RPYT 探针会把 commander 优先级顶到 CRTP=2**，之后任何 HIGHLEVEL(1) 指令（takeoff/land）都被 `priority>=current` 丢弃 → 需要 `send_notify_setpoint_stop()` 连发 3 次 relax | 19:34（goto 卡 z=0.041 15s）、20:03（复发） |
| 5 | 起飞用 **HL takeoff**（不带 yaw），不用 go_to —— HL 的 yaw 环在 EKF 坐标系下正反馈（yaw 2°→127° 指数发散）；CTBR 接管时**锁当前 yaw** | 19:34 CSV yaw 列 |
| 6 | **动捕注入必须先"EKF 复位再注入"**：`kalman.resetEstimation=1` 后不注入的纯 IMU EKF 会漂移（|v| 0.56→2.667 m/s 爬升）→ wait_mocap 阶段复位+开启注入+三条件收敛门 | 19:01 影子未收敛 |
| 7 | 日志块**必须等 TOC 就绪再配置**（`len(cf.log.toc.toc)>0 and param.is_updated`），且配置前 `cf.log.reset()` 清上一进程残留块；块间 sleep | 19:39/19:45/19:47 三次飞行中流断 |
| 8 | 诊断块**降频**：st/att@100Hz，ct/ctr@10Hz，bat@1Hz（全 100Hz = 500包/s 下行，与上行 200包/s 挤爆 2M 链路） | 19:39 带宽隐患 |
| 9 | **时钟基准必须统一 `time.monotonic()`**（MocapLmc 线程保存帧时刻时若用 `time.time()` 与主循环的 monotonic 相减会恒为负 → 永远"断流"） | 20:01 纯诊断打印抓出 |
| 10 | 动捕源默认 **nokov SDK 100% 命中**（2002/2002），VRPN 后端仅 41%（动捕软件侧数据源问题，非脚本）；VRPN 有 cf4/zcar1 死刚体刷屏 | 地面命中率对照 |

---

## 2. 环境与文件清单

### 2.1 环境

```bash
conda activate crazyplay-231          # Python 3.11 + cflib 0.1.33 + numpy 2.4.6
# 额外（仅构建 mocap_direct 模块需要；脚本运行只需要 lib/ 里的 .so）：
conda run -n crazyplay-231 pip install pybind11==2.13.6   # 本机镜像有；仅手动 g++ 3.11 模块时用
```

### 2.2 本仓库本次新增/关键文件

| 文件 | 作用 |
|---|---|
| [0902ctbr.py](0902ctbr.py) | ★ 纯 cflib CTBR 圆轨迹飞行（主脚本，已实飞验证） |
| [0902heart.py](0902heart.py) | 心形版本（同架构，`HeartMission` 参数式心形，未实飞） |
| [mocap_direct/](mocap_direct/) | libmotioncapture Python 绑定直连：`lib/motioncapture.cpython-311/310-*.so` + `libnokov_sdk.so`、[build.sh](mocap_direct/build.sh)、[README.md](mocap_direct/README.md)、[test_direct.py](mocap_direct/test_direct.py) |
| [diagnostics/](diagnostics/) | 本次全部诊断脚本（复现排障必需）：`fw_version.py`(固件版本)、`chan_compare.py`(双通道对比)、`ground_log_test.py`(地面日志流三阶段)、`dump_toc.py`(TOC)、`test_link.py`(裸链) |
| [crazyflie-firmware-master/crazyflie-firmware-master](crazyflie-firmware-master/crazyflie-firmware-master) | 2026.08 固件源码（解包自用户提供的 zip，符号链/优先级/解码器查证用） |
| logs/0902ctbr_*.csv | 每次飞行/影子的 40 列数据（复现判读依据） |

> 原 [reference/crazyswarm](../reference/crazyswarm)（ROS1 参考实现）内的 `externalDependencies/libmotioncapture` 是本次 mocap 直连方案的**源码来源**（含被收录的 NOKOV SDK：`deps/nokov_sdk/`）。

---

## 3. 复现步骤（分层验收，一层不通不碰下一层）

### G0 构建 mocap 直连模块（一次性）

```bash
cd mocap_direct && bash build.sh    # 由 reference 源码 + vendored NOKOV SDK 构建
# 产物: lib/motioncapture.cpython-311-x86_64-linux-gnu.so + libnokov_sdk.so
```

### G1 动捕直连体检（不连飞机）

```bash
cd ~/All_code_project/mocap_crazyflie
PYTHONPATH=mocap_direct/lib python3 mocap_direct/test_direct.py nokov --n 120
# 期望: ~100Hz, cf3 命中率 100%, 四元数范数 1.0000
# 注意: 若命中率<100% 或看到 VRPN Warning, 检查 XINGYING SDK 广播(动捕软件侧开"多播/sdk 输出")
```

### G2 固件版本确认（服务端状态判断）

```bash
python3 diagnostics/fw_version.py
# 正式输出: console "Build 0:54f31e243a0b (2026.08) CLEAN"；TOC 特征: ctrltarget 含 mode_*, stateEstimateZ.quat 存在
# 若 banner 错过: usec.reset=1 重启后抓 console
```

### G3 纯轨迹自检（不连飞机）

```bash
python3 0902ctbr.py --dry-run
python3 0902heart.py --dry-run
# 期望: R=1 ω=0.4 峰值 v=0.75m/s 倾角5.38° 时长24.2s（心形: 峰值0.37m/s 2.09° 29.4s）
```

### G4 链路影子（连飞机+参数+日志+预检，不发指令不 ARM）

```bash
# 飞机放平 → 断电 → 重上电（每次 estop 或异常中断后必须）
python3 0902ctbr.py
# 期望序列: 连接→参数回读→TOC 就绪→日志5块→动捕在流+注入开启→EKF收敛(散布<1mm,|v|<0.05,偏差<0.3m)
#          → 电压预检 → (影子退出)
# 判读 CSV logs/0902ctbr_*.csv: st 行数≈(预检秒数×100)+ 且持续推进
```

### G5 低高度实飞

```bash
python3 0902ctbr.py --fly --circle-z 0.35 --circle-omega 0.3
# 期望: ARM 确认(0x020A)→通道自检(mode(2,2,2)+yaw=-5.0 ✔)→relax×3→takeoff(z爬升0.04→0.15)
#      →交棒 CTBR(yaw=当前值, 30°内)→圆轨迹29.4s→回点悬停→land→DISARM
# 中途异常: 一次 Ctrl+C=紧急降落(notify→enHighLevel=1→hl.land→等落地→DISARM), 二次=硬退
```

### G6 心形

```bash
python3 0902heart.py --fly --circle-z 0.35 --circle-omega 0.3
```

---

## 4. 关键参数表（0902ctbr.py 默认值，取自 §6 的实测定案）

```text
--uri radio://0/80/2M/E7E7E7E703       --mocap-source auto (→nokov)   --lmc-backend nokov
--goto-height 0.15  --goto-duration 2.5   (HL takeoff, 不带yaw)
--circle-omega 0.40  --circle-radius 1.0  --circle-loops 1.0
--entry-t 2.5 --ramp-t 1.5 --return-t 2.5 --final-hold-s 2.0 --rate 100
--mass 0.0434 --fmax 1.234                 (悬停raw反推标定: 0901 1.254 → 1.234 实测)
--max-thrust 0.80 --max-tilt-deg 10.0  --max-body-rate 3 3 2
--kp 0.30 0.30 0.45  --kv 0.25 0.25 0.35  --katt 8.0 8.0 4.0   (0901 定案)
--state-timeout 0.30 --max-track-error 0.50 --max-room 1.70 --max-z-over 0.60
--max-tilt-abort-deg 30 --max-velocity 2.0 --critical-voltage 3.60
--min-voltage 3.80 --voltage-ref 4.20 --voltage-exponent 0.0   (0901 定案: 关电压补偿)
--roll-sign 1 --pitch-sign 1 --yaw-sign 1                        (legacy RATE 符号已实测)
```

符号链（legacy RPYT，cflib 打包只翻 pitch + 固件 RATE 解码只翻 yaw + 速率环反馈 −gyro.y）：

```text
api.roll    = +p     (真实体轴角速度 deg/s)
api.pitch   = +q
api.yawrate = -r     ← 唯一取负!(实测 api +5 → ctrltarget.yaw −5)
thrust      = raw16 = 60000·(F/Fmax)^(1/2)·raw_scale · 65535/60000
```

---

## 5. 命令速查（复现排障用）

| 目的 | 命令 |
|---|---|
| 裸链测试 | `python3 diagnostics/test_link.py` |
| 固件版本/特征 | `python3 diagnostics/fw_version.py` |
| 双通道对比(legacy vs manual) | `python3 diagnostics/chan_compare.py`（ARM+零推力, 安全） |
| 日志流三阶段(地面/ARM/1cm起飞) | `python3 diagnostics/ground_log_test.py` |
| TOC dump | `python3 diagnostics/dump_toc.py` |
| radio 被占 | `pkill -f cfclient; pkill -f crazyflie_server` + `python3 -c "import usb.core;usb.core.find(idVendor=0x1915,idProduct=0x7777).reset()"` |

---

## 6. 本次调试全记录（15 个坑：现象→定位方法→修复——全部数据实证）

> 时间线 2026-09-02 18:51 — 20:03。

| # | 时间 | 现象 | 定位方法 | 根因 | 修复 |
|---|---|---|---|---|---|
| 1 | 18:51 | `KeyError Type[uint32]` | cflib log.py `LogTocElement.types` 读源码 | cflib 类型名是 `uint32_t` 不是 `uint32`（float 同理） | 改 `'uint32_t'` |
| 2 | 18:51 | 回调频频该异常、`stateEstimateZ.quat` KeyError | CLAUDE.md 已有记录 + cflib log.py | **多块共用一个回调时 `data` 只含本块变量**（st 块没有 quat） | 按块缓存（`_R/_t_att`），两路都新鲜才装配 `self.state` |
| 3 | 18:59 | `_ph_init() takes 1 positional argument but 2 were given` | 读脚本 | `tick()` 调 `handler(dt)`，但所有 `_ph_*` 定义无参 | 改 `handler()`（dt 只在日志用） |
| 4 | 19:00 | 影子跑完不退出 | 读状态机 | `goto("done")` 未置 `finished` | 置 `finished=True` + 明确文案 |
| 5 | 19:01 | EKF 永不收敛（散布 395→2346mm, \|v\| 0.56→2.667） | CSV 打印 + 老脚本对照 | **影子无注入, 纯 IMU EKF 复位后漂移**（老路线 server 常驻注入） | `wait_mocap`: 复位 EKF→开启注入（地面安全）→三条件门控(散布<2cm/\|v\|<0.05/动捕偏差<0.30m)；注入全局化 `_maybe_inject()` |
| 6 | 19:28 | 通道自检回读 r_* 恒 0 (manual) → 拒飞 | chan_compare 双通道 | **本机 2026.08 固件不含 TYPE_MANUAL 解码器**（源码 2025.02 tags 0..10；2026.08 实测不支持；master 源码有 manual=11——确认 manual 是新提交未发布） | 改 **legacy RPYT `send_setpoint`**：符号链 api(roll=+p, pitch=+q, yawrate=−r)；推力 raw16 换算 |
| 7 | 19:30 | ARM 后自检 ctrltarget 仍 0 | master stabilizer.c:332-343 | **未 `canFly` 时 `crtpCommanderBlock(true)` + 执行 setpoint 强制清零**（自检在 ARM 前即使通道正也全 0） | 自检移到 `wait_armed` 之后（探针推力 0, ARM idle 电机不转安全） |
| 8 | 19:32 | 自检判据用 `ctrltarget.roll`：RATE 下恒 0 → 误判 | master `ctrltarget` 日志组 | `ctrltarget.roll/pitch` 是**角度态**, RATE 模式只填 attitudeRate（且同样变量 15/5 值实测才知）；`cmd_roll` 又受"零推力强制清零"(controller_pid.c) | 判据 = `ctrltarget.mode_roll/pitch==2`（速率态贯透）+ `ctrltarget.yaw≈−5`（取负验证）；cmd 仅打印 |
| 9 | 19:34 | 自检通过→goto 完全不起飞 (z 0.041 卡 15s) | master commander.c:81 `priority>=currentPriority` | 自检 RPYT 探针把优先级顶到 **CRTP=2**，HL goto(1) 被丢弃（0901 约束2同款） | 自检后 `send_notify_setpoint_stop(0)` 改为×3 连发（还防丢包, 见 #15） |
| 10 | 19:34 | 交棒后状态失效 ESTOP, 且 yaw 2°→127° 指数 | CSV `yaw(EKF)` vs `up_yaw(mocap)` 同步漂(dup≈1mm) | **HL go_to 的 yaw 环在 EKF 系正反馈**(物理真转: 漂移速率每0.25s翻倍 8→45°/s) | 起飞改 HL **takeoff**(不碰yaw), CTBR 接管锁当前 yaw（0901 同款, 老库也记过 go_to 大回旋坑） |
| 11 | 19:39 | 交棒后 state 冻结 (t=16.766 与 16.817 完全相同) → estop | CSV 逐行 + 与已知对照 | 日志块在 **TOC 未就绪时 add**(只 1.5s 后) → 固件 log 表错 → 飞行中断流 | **配置前等 TOC**(log.toc 非空 + param.is_updated, 20s 超时拒飞) + 块间 sleep 0.3s |
| 12 | 19:45 | `✗ 动捕断流 > 0.5s`（nokov 后端, 数据健康） | 加诊断 `mocap线程: 总帧=3766 命中=1551` | **VRPN 后端 cf3 命中率只有 41%**（地面也如此, 动捕软件 VRPN server 问题），交棒时恰逢刚体消失>0.5s | 用 **nokov SDK 后端(100% 命中)**；代码再保留 `mocap.diag()` 打印 |
| 13 | 19:47 | VRPN: z 爬升后卡 0.062m 15s 超时 | CSV: pz(EKF) 冻结 vs up_z(动捕) 持续爬→0.168 | 同 #11 更细版（EKF log 流断），且日志 `Error no LogEntry id=255`=固件收到未配置块 | 加 **`cf.log.reset()`** 配置前（清上一进程 os._exit 残留的固件日志块） |
| 14 | 20:01 | nokov+TOC 修复后交棒仍 `✗ 动捕断流>0.5s`，诊断却显示帧龄=1ms、命中=2004 | **诊断打印 vs 判定矛盾**是最关键信号 | **时钟基准混用**：`MocapLmc` 线程存 `time.time()`(墙钟)，主循环 age 用 `time.monotonic()` → `monotonic()-time()` 恒负 → `abs>0.5` 恒真 | 全链路统一 `time.monotonic()`（Udp 源同步改） |
| 15 | 20:03 | 自检后 takeoff 仍卡0.041 + `id=255` | 与 #9 复发时间点（上轮 estop 后未断电→块残留+relax 丢包） | relax 单发丢包 → CRTP 又压 HL | relax 三连发；文档明确"每次 estop 后断电重上电" |

**验证点**：地面 3 块 30s + 100Hz extpose（3000/3000 完美）、ARM 电机空转 15s（2000 完美）、真实 takeoff 0.1m 20s（每秒 st/att 各 100 帧几乎零丢）——确保"流断"只在"TOC 未就绪坏块"场景出现，排除电磁干扰/杆量。

---

## 7. 安全设计要点（已内置，勿删改）

```text
Ctrl+C 一次: 信号函数只置标志 → 主循环 emergency_land:
   send_notify_setpoint_stop ×3 → enHighLevel=1 → hl.land(0.0,1.0) → 等z<0.08 → disarm
   （land 失败: 零推力兜底 + 提示断电）
Ctrl+C 二次: os._exit(1) 硬退（不经 Python 析构→radio 残留会被 USB reset 清）
CTBR 期间 guard: 状态超龄/动捕断流/持续偏差/位置越界/倾角/速度/电压 → estop
固件兜底: 主机停发时最后指令残留 500ms 才回正、2000ms 停桨(commander xQueuePeek)
⭐ 每次 estop/异常后: 先飞机断电重上电再飞 (supervisor LOCKED / 固件日志块残留)
⭐ 飞机必须放平再 ARM (TUMBLED 位 0x20 会静默拒飞)
```

---

## 8. 已知遗留问题（不影响飞行，按优先级）

| 项 | 说明 |
|---|---|
| yaw 环最弱(CTBR) | 悬停 0.11°/s 漂移(0901 实测)；心形/高速机动需关注 |
| Fmax 电压标定 | 当前 1.234N 固定；3.60V 以下未测；`--voltage-exponent 0`（电压几乎不塌已 0901 证实）；run_dash 高速档电流大需重标 |
| packed 注入升级 | 现用 float 格式 `send_extpose`(28B/包)；master 支持 packed(12B/包, `extPosePackedHandler`) —— 若未来射频带宽紧张再切换（需手写 CRTP 包） |
| VRPN 数据质量 | nokov SDK 100% vs VRPN 41%；动机软件侧配置待查（不影响本链路, 默认 SDK） |
| manual(TYPE_MANUAL) | 源码 master 有、本固件 release 无；等官方 release 后可把发送通道切回 manual（sign 表已备: `--roll/pitch/yaw-sign`） |
| 心形未实飞 | 轨迹/包线离线全绿(peak 0.37m/s/2.09°)；需按 G4→G5 步进验证 |

---

## 9. 附录：本次会话用到的关键源码锚点（master 2026.08 固件）

```text
crazyflie-firmware-master/crazyflie-firmware-master/
  src/modules/src/stabilizer.c:328-343    未 canFly 时 crtpCommanderBlock + setpoint清零
  src/modules/src/commander.c:74-98       commanderSetSetpoint(priority>=current)+Relax
  src/modules/src/crtp_commander.c:115-135 端口分发: port3(RPYT)/port7(generic,ch0 type,ch1 meta)
  src/modules/src/crtp_commander_generic.c:65-131  packetDecoders[0..11], manualDecoder
  src/modules/src/crtp_localization_service.c:216-235 extPoseHandler (7×float), 245+ packed
  src/modules/interface/quatcompress.h    29bit 压缩四元数(2bit index+3×10bit)
  src/modules/src/controller/controller_pid.c:151-172 rate 环反馈(gyro.x,-gyro.y,gyro.z); 190+ thrust==0 强制清 cmd
```
