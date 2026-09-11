#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
0902ctbr.py — 纯 cflib（无 ROS 控制栈）CTBR 大圆轨迹飞行
   复用 0901 悬停测试的几何 SE(3) 外环（GeometricCtbrController），
   发送通道 = cflib `send_setpoint(roll, pitch, yawrate, thrust)`（legacy RPYT，
              稳定率模式 flightmode.stabMode*=0）。
   ★ 2026-09-02 实测定案：本机 2026.08 固件【不接】TYPE_MANUAL(11) 的
     send_setpoint_manual（ARM 前后 ctrltarget 均无更新）；legacy RPYT 通道
     实测可用（ctrltarget.roll=+20 直通 / api yaw +30→setpoint −30 净取负）。

任务序列（全程自动）：
   通道贯透自检(地面) → 电压预检 → 估计器收敛门控(动捕 vs 融合)
   → ARM(无刷必需) → 等 supervisor IS_ARMED|CAN_FLY
   → 高层 goto 起飞到原点 (0,0,0.15m)
   → CTBR：定点入圆(R=1m) → 圆周整圈 → 返回原点 → 回点悬停
   → 交回高层（notify_setpoints_stop + enHighLevel=1）→ land → DISARM

急停（用户要求：最高优先级，Ctrl+C = 立即进入降落 land 指令）：
   第 1 次 Ctrl+C  → 立刻 stop CTBR 发包 → send_notify_setpoint_stop(0)
                    → enHighLevel=1 → hl.land(0.0, 1.0)（最快降落）→ 等落地 → disarm
   第 2 次 Ctrl+C  → os._exit(1)（硬退）
   任务中状态失效/包线越界/低电量 → 同一 emergency_land 通路。

数据链（无 ROS 控制栈）：
   [mocap]  VRPN(10.1.1.198:3883, 刚体 cf3) 或 UDP(127.0.0.1:9000, 桥格式 <7f)
      → 本机 → cf.extpos.send_extpose(x,y,z, qx,qy,qz,qw) ──CRTP──▶ 固件 locSrv/Kalman
   [融合]   固件 EKF 回读 logblock @100Hz：
               stateEstimate.x/y/z (float, m)      ← 融合位置 (主)
               stateEstimate.vx/vy/vz (float, m/s) ← 融合速度
               stateEstimateZ.quat (uint32)        ← 压缩四元数(解压→旋转矩阵)
               gyro.x/y/z (deg/s) 与 pm.vbat       ← 诊断/电压
               controller.r_roll/r_pitch/r_yaw     ← 通道贯透回读(rateDesired)
      → 本机 100Hz：几何外环 compute(state, target) → (p,q,r)+F
      → send_setpoint(deg(p), deg(q), -deg(r), legacy_raw) ──▶ 固件速率内环

★ legacy RPYT 通道符号链（2026-09-02 chan_compare 实测 + 源码推导，见文件尾注释）：
     api 参数 (roll,pitch,yawrate) 即真实体轴角速度，映射：
       · roll  : cflib 原样打包、固件 legacy 解码直通、速率环反馈 gyro.x  ⇨ api.roll  = +p
       · pitch : cflib 打包取负(-pitch) 与 固件速率环反馈(-gyro.y) 恰好相消 ⇨ api.pitch = +q
       · yaw   : 固件 legacy 解码对 yaw 取负("legacy rate input is inverted")
                 ⇒ 净翻转一次 ⇒ api.yawrate = -r        （实测: api+30 → setpoint −30）
     thrust: uint16 raw（0..65535 满量程）；按 N→raw(0..60000) 平方反解后 × 65535/60000。
     若上机后发现某轴正反馈，用 --roll-sign/--pitch-sign/--yaw-sign 翻转（默认 +1）。

mocap 来源（自动优先级 lmc → udp；--mocap-source 可强制）：
   · lmc  : libmotioncapture Python 绑定直连（推荐，真·无 ROS、无需 UDP 桥）——
            mocap_direct/lib 下的 motioncapture 模块（本地源码构建，已实测）。
            后端默认 --lmc-backend nokov（官方 SDK 数据广播，100Hz，无噪声），
            可选 vrpn（NOKOV 的 VRPN 输出，197Hz，但有死刚体警告噪声）。
   · udp  : 复刻 mocap_udp_bridge.py 的格式 (<7f: x,y,z,qw,qx,qy,qz @127.0.0.1:9000)，
            mocap 一跳走既有 mct+bridge（控制栈仍 100% cflib）。

运行（crazyplay-231 环境，独占 radio —— 期间不能开 crazyswarm2 server/cfclient）：
    conda activate crazyplay-231
    python3 0902ctbr.py --dry-run               # ① 纯轨迹自检(不连飞机)
    python3 0902ctbr.py                         # ② 影子=链路连通性影子(连接+参数+日志+预检, 不发指令不 ARM)
    python3 0902ctbr.py --fly                   # ③ 实飞（建议 --circle-z 0.35 先小高度）

安全：实飞请拆桨/护罩小步推进；Ctrl+C(一次)=紧急降落，Ctrl+C×2=硬退。
     ESTOP 后若 supervisor 被判 LOCKED/TUMBLED，需给飞机断电重上电。
"""

import argparse
import csv
import math
import os
import signal
import socket
import struct
import sys
import threading
import time
import traceback
from datetime import datetime

import numpy as np

# ---- 解释器/环境校验（cflib 必须在 conda crazyplay-231 里跑，system python 没装 cflib）----
try:
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crtp import init_drivers
except ImportError as _e:
    print("=" * 72)
    print("错误：当前环境没有 cflib，请用：")
    print("    conda activate crazyplay-231")
    print("    python3 0902ctbr.py")
    print(f"  ({_e})")
    print("=" * 72)
    sys.exit(1)

# ---- 复用 0901 几何控制器（纯 numpy/math，无 ROS 依赖）----
_SELF = os.path.dirname(os.path.abspath(__file__))
_CTBR_DIR = os.path.join(_SELF, "0901悬停测试", "ctbr")
if _CTBR_DIR not in sys.path:
    sys.path.insert(0, _CTBR_DIR)
try:
    from geometric import (ControllerConfig, GeometricCtbrController,  # noqa: E402
                           smoothstep5, quaternion_to_rotation,
                           rotation_yaw, rotation_tilt)
except ImportError as _e:
    print(f"错误：无法导入 0901 的几何控制器（{_CTBR_DIR}）：{_e}")
    sys.exit(1)

# ---- libmotioncapture 直连模块（mocap_direct/lib；导入失败时自动退回 UDP 桥）----
try:
    import motioncapture  # noqa: F401
    HAVE_LMC = True
except ImportError:
    HAVE_LMC = False


# ================================================================ 常量
MAX_LOG_BLOCK_B = 26          # 固件 log 单块上限
RAW_FULL_SCALE = 60000.0
LEGACY_RAW_FULL = 65535.0     # legacy RPYT thrust 16bit 满量程（发送换算用）

# supervisor 位域（固件源码 supervisor.c）
SUP_IS_ARMED = 0x0002
SUP_CAN_FLY = 0x0008
SUP_IS_TUMBLED = 0x0020
SUP_IS_LOCKED = 0x0040


def _smoothstep5_integral(u):
    """smoothstep5 的定积分 ∫₀^u: 2.5u⁴ − 3u⁵ + u⁶（相位斜坡用，∫₀¹=0.5）。"""
    u = min(max(float(u), 0.0), 1.0)
    return u ** 4 * (2.5 - 3.0 * u + u * u)


def quatdecompress(comp):
    """uint32 压缩四元数 → [x,y,z,w]（crazyflie_cpp quatdecompress 的 Python 移植，crtp.cpp:45）。

    29bit：最高 2bit=最大分量索引；其余 3 分量各 10bit(9bit 幅值+1bit 符号)，
    幅值按 SMALL_MAX=1/√2 归一；最大分量 = √(1−Σq_小²)。
    """
    SMALL_MAX = 1.0 / math.sqrt(2)
    mask = (1 << 9) - 1
    q = [0.0, 0.0, 0.0, 0.0]
    il = comp >> 30
    ss = 0.0
    for i in (3, 2, 1, 0):
        if i != il:
            mag = comp & mask
            negbit = (comp >> 9) & 0x1
            comp >>= 10
            q[i] = SMALL_MAX * mag / mask
            if negbit:
                q[i] = -q[i]
            ss += q[i] * q[i]
    q[il] = math.sqrt(max(0.0, 1.0 - ss))
    return q  # [x,y,z,w]


class ManualCtbrBridge:
    """几何外环 (p,q,r)[rad/s] + F[N] → legacy RPYT 的 (roll,pitch,yawrate,thrust_raw16)。

    符号映射（chan_compare 实测 + 推导，见文件头/尾注释）：
      roll=deg(p)、pitch=deg(q)、yawrate=-deg(r)；
    推力：F → raw(0..60000 满量程) 平方反解 × 电压 raw_scale → × 65535/60000 得 16bit。
    """

    def __init__(self, fmax_newton=1.234, curve_exponent=2.0, max_thrust_newton=0.80,
                 max_body_rate_radps=3.0, raw_scale_min=0.90, raw_scale_max=1.12):
        self.fmax = float(fmax_newton)
        self.exponent = float(curve_exponent)
        self.max_thrust = min(float(max_thrust_newton), self.fmax)
        self.max_body_rate = float(max_body_rate_radps)
        self.raw_scale_min = float(raw_scale_min)
        self.raw_scale_max = float(raw_scale_max)
        self.raw_scale = 1.0

    def set_raw_scale(self, scale):
        if not (self.raw_scale_min <= float(scale) <= self.raw_scale_max):
            return False
        self.raw_scale = float(scale)
        return True

    def newton_to_raw16(self, thrust_newton):
        """F[N] → legacy 16bit raw（0..65535），平方反解 × raw_scale × 满量程换算。"""
        thrust = min(max(0.0, float(thrust_newton)), self.max_thrust)
        ratio = thrust / self.fmax if self.fmax > 0.0 else 0.0
        raw_nominal = RAW_FULL_SCALE * math.pow(ratio, 1.0 / self.exponent)
        raw = raw_nominal * self.raw_scale                     # 0..60000 标度
        raw16 = raw * LEGACY_RAW_FULL / RAW_FULL_SCALE         # → legacy 16bit 满量程
        return int(min(max(raw16, 0.0), 65535.0))

    def to_legacy(self, p_rad, q_rad, r_rad, thrust_newton, signs):
        """返回 (roll_deg, pitch_deg, yawrate_deg, thrust_raw16)，signs=(rs,ps,ys) 最终翻转。"""
        lim = self.max_body_rate
        p = min(max(float(p_rad), -lim), lim)
        q = min(max(float(q_rad), -lim), lim)
        r = min(max(float(r_rad), -lim), lim)
        return (signs[0] * math.degrees(p),
                signs[1] * math.degrees(q),
                -signs[2] * math.degrees(r),          # legacy 净取负: api.yawrate = -r
                self.newton_to_raw16(thrust_newton))


# ================================================================ mocap 源
class MocapUdp:
    """UDP 位姿源（与 mocap_udp_bridge.py 同格式：<7f = x,y,z,qw,qx,qy,qz）。"""

    def __init__(self, host="127.0.0.1", port=9000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.lock = threading.Lock()
        self.pose = None          # (x,y,z,qw,qx,qy,qz)
        self.ts = 0.0
        self.name = f"udp:{host}:{port}"

    def poll(self):
        try:
            while True:
                data, _ = self.sock.recvfrom(64)
                if len(data) >= 28:
                    with self.lock:
                        self.pose = struct.unpack("<7f", data[:28])
                        self.ts = time.monotonic()   # 与主循环判定同基准
        except BlockingIOError:
            pass

    def get(self):
        with self.lock:
            return self.pose, self.ts


class MocapLmc:
    """libmotioncapture 直连（本地构建的 motioncapture 模块，无 ROS）。

    后台线程 waitForNextFrame（~200Hz），只保留目标刚体最新一帧；
    统一输出 (x,y,z,qw,qx,qy,qz) + 接收时刻。
    """

    def __init__(self, root, backend="vrpn", host="10.1.1.198", target="cf3"):
        import motioncapture
        if root not in sys.path:
            sys.path.insert(0, root)
        cfg = {"hostname": host}          # 绑定只接受 Dict[str,str]
        if backend == "nokov":
            cfg.update({"enableFrequency": "0", "updateFrequency": "100"})
        self.mc = motioncapture.connect(backend, cfg)
        self.target = target
        self.name = f"lmc({backend}):{target}@{host}"
        self.lock = threading.Lock()
        self.pose = None
        self.ts = 0.0
        # 诊断：线程帧计数（区分 线程卡死 / 刚体消失 / 主循环没读）
        self.frames = 0                    # 线程收到的总帧数
        self.hit_frames = 0                # 命中 target 的帧数
        self.last_frame_t = 0.0            # 线程最近收帧时刻
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while True:
            try:
                self.mc.waitForNextFrame()
                # ★ 全程统一 time.monotonic()：主循环的 age/超时判定都用单调钟，
                #   若这里用 time.time()（墙钟 epoch 秒）会让差值恒为负 → 误判断流
                self.last_frame_t = time.monotonic()
                with self.lock:
                    self.frames += 1
                for name, obj in self.mc.rigidBodies.items():
                    if name != self.target:
                        continue
                    p = obj.position
                    r = obj.rotation        # Eigen Quaternion: .w .x .y .z
                    with self.lock:
                        self.pose = (float(p[0]), float(p[1]), float(p[2]),
                                     float(r.w), float(r.x), float(r.y), float(r.z))
                        self.ts = time.monotonic()
                        self.hit_frames += 1
            except Exception:
                time.sleep(0.05)

    def poll(self):
        pass

    def get(self):
        with self.lock:
            return self.pose, self.ts

    def diag(self):
        """返回 (总帧数, 命中帧数, 线程最近帧年龄秒)。"""
        with self.lock:
            f, h = self.frames, self.hit_frames
        return f, h, (time.monotonic() - self.last_frame_t)


# ================================================================ 圆轨迹（全解析导数）
class CircleMission:
    """中心=原点的 R 圆任务：入场(中心→圆周) → 角速度斜坡 → 匀速圆周 → 减速停
    → 返回(圆周→原点) → 回点悬停。位置/速度/加速度全部闭式导数。

    相位积分(与 0901 Figure8Trajectory 同法)：角速度按 smoothstep5 0→ω→0，
    相位由 ∫ 给出 ⇒ 起终点角速度/角加速度为零，且总相位精确 = 2π·loops。
    yaw 全程锁定（几何外环 yaw 通道最弱）。"""

    def __init__(self, radius=1.0, omega=0.40, loops=1.0, height=0.15,
                 entry_t=2.5, ramp_t=1.5, return_t=2.5, hold_s=2.0):
        self.R = float(radius)
        self.omega = float(omega)
        self.loops = float(loops)
        self.z = float(height)
        self.entry_t = max(float(entry_t), 1e-3)
        self.ramp_t = max(float(ramp_t), 1e-3)
        self.return_t = max(float(return_t), 1e-3)
        self.hold_s = max(float(hold_s), 0.0)
        # 圆周相位预算：两段斜坡各贡献 ω·ramp_t/2，余下为匀速段
        total_angle = 2.0 * math.pi * self.loops
        ramp_angle = self.omega * self.ramp_t
        if total_angle <= ramp_angle:
            raise ValueError(f"ramp_t 过长：斜坡占 {ramp_angle:.2f} rad > 总相位 {total_angle:.2f} rad")
        self.cruise_angle = total_angle - ramp_angle
        self.cruise_t = self.cruise_angle / self.omega
        # 各段时间轴
        self.t_entry_end = self.entry_t
        self.t_ramp_end = self.t_entry_end + self.ramp_t
        self.t_cruise_end = self.t_ramp_end + self.cruise_t
        self.t_ret_end = self.t_cruise_end + self.ramp_t
        self.t_hold_end = self.t_ret_end + self.return_t
        self.duration = self.t_hold_end + self.hold_s
        # 圆周相位的 e_θ(θ0)；圆周起点 (R,0)，θ0=0
        self.theta0 = 0.0
        self.circle_start = np.array([self.R, 0.0, self.z])
        self.center = np.array([0.0, 0.0, self.z])

    # ---------- 相位 ----------
    def _phase(self, t):
        """返回 theta, thetad, thetadd（ramp 段用 smoothstep 斜坡，匀速段为常值）。"""
        w, R = self.omega, self.ramp_t
        if t <= self.t_entry_end:
            return 0.0, 0.0, 0.0
        if t <= self.t_ramp_end:                       # 加速斜坡
            u = (t - self.t_entry_end) / R
            s, ds, _ = smoothstep5(u)
            th = w * R * _smoothstep5_integral(u)
            return th, w * s, w * ds / R
        if t <= self.t_cruise_end:                     # 匀速
            th = w * R * 0.5 + w * (t - self.t_ramp_end)
            return th, w, 0.0
        if t <= self.t_ret_end:                        # 减速斜坡
            u = (t - self.t_cruise_end) / R
            s, ds, _ = smoothstep5(u)
            th = (w * R * 0.5 + w * self.cruise_t + w * R * (u - _smoothstep5_integral(u)))
            return th, w * (1.0 - s), -w * ds / R
        th = w * R + w * self.cruise_t                 # R 段结束相位= ω·(2·R/2 + cruise) = 2π·loops
        return th, 0.0, 0.0

    @staticmethod
    def _circle_kinematics(theta, thetad, thetadd, R, z):
        p = np.array([R * math.cos(theta), R * math.sin(theta), z])
        v = np.array([-R * thetad * math.sin(theta), R * thetad * math.cos(theta), 0.0])
        a = np.array([-R * (thetad * thetad * math.cos(theta) + thetadd * math.sin(theta)),
                      R * (-thetad * thetad * math.sin(theta) + thetadd * math.cos(theta)), 0.0])
        return p, v, a

    def evaluate(self, t):
        """返回 (target_dict, done)。target 键与 geometric.compute 一致（yaw 由调用方填）。"""
        t = float(t)
        if t <= self.t_entry_end:                      # 圆心 → 圆周起点 (R,0)
            u = t / self.entry_t
            s, ds, dds = smoothstep5(u)
            d = self.circle_start - self.center
            p = self.center + d * s
            v = d * ds / self.entry_t
            a = d * dds / (self.entry_t * self.entry_t)
            return self._target(p, v, a), False
        if t <= self.t_ramp_end:
            th, thd, thdd = self._phase(t)
            p, v, a = self._circle_kinematics(th, thd, thdd, self.R, self.z)
            return self._target(p, v, a), False
        if t <= self.t_cruise_end:
            th, thd, thdd = self._phase(t)
            p, v, a = self._circle_kinematics(th, thd, thdd, self.R, self.z)
            return self._target(p, v, a), False
        if t <= self.t_ret_end:                        # 减速斜坡 → 回圆周起点
            th, thd, thdd = self._phase(t)
            p, v, a = self._circle_kinematics(th, thd, thdd, self.R, self.z)
            return self._target(p, v, a), False
        if t <= self.t_hold_end:                       # 返回：圆周起点 → 圆心
            u = (t - self.t_ret_end) / self.return_t
            s, ds, dds = smoothstep5(u)
            d = self.center - self.circle_start
            p = self.circle_start + d * s
            v = d * ds / self.return_t
            a = d * dds / (self.return_t * self.return_t)
            return self._target(p, v, a), False
        if t <= self.duration:                         # 回点悬停
            p = self.center.copy()
            return self._target(p, np.zeros(3), np.zeros(3)), False
        return self._target(self.center.copy(), np.zeros(3), np.zeros(3)), True

    @staticmethod
    def _target(p, v, a):
        return {"position": p, "velocity": v, "acceleration": a}

    def peaks(self, gravity=9.80665, n=4000):
        """扫全轨：峰值速度/水平加速度/需求倾角（包线体检）。"""
        vmax = amax = 0.0
        for i in range(n + 1):
            tgt, _ = self.evaluate(self.duration * i / n)
            vmax = max(vmax, float(np.linalg.norm(tgt["velocity"])))
            amax = max(amax, float(np.linalg.norm(tgt["acceleration"][:2])))
        return {"v_max": vmax, "a_max": amax,
                "tilt_max_deg": math.degrees(math.atan2(amax, gravity))}


# ================================================================ 主节点
class CtbrCircleTest:
    """任务状态机（单线程 100Hz 主循环 + log/mocap 回调线程）。"""

    def __init__(self, args):
        self.args = args
        self.fly = bool(args.fly)
        self.shadow = bool(args.shadow)

        # ---- 控制内核（0901 几何外环 + manual 桥）----
        self.controller = GeometricCtbrController(ControllerConfig(
            mass=args.mass,
            gravity=9.80665,
            max_command_thrust=args.max_thrust,
            max_tilt_rad=math.radians(args.max_tilt_deg),
            max_body_rate=np.asarray(args.max_body_rate, dtype=float),
            position_gain=np.asarray(args.kp, dtype=float),
            velocity_gain=np.asarray(args.kv, dtype=float),
            integral_gain=np.array([0.05, 0.05, 0.02]),
            integral_limit=np.array([0.8, 0.8, 0.8]),
            attitude_gain=np.asarray(args.katt, dtype=float),
            attitude_integral_gain=np.zeros(3),
            attitude_integral_limit=np.array([0.8, 0.8, 0.8]),
            position_integral_c1=0.35,
            use_body_rate_feedforward=bool(args.ff),
        ))
        self.bridge = ManualCtbrBridge(
            fmax_newton=args.fmax, max_thrust_newton=args.max_thrust,
            max_body_rate_radps=max(args.max_body_rate))
        self.signs = (args.roll_sign, args.pitch_sign, args.yaw_sign)
        self.mission = CircleMission(
            radius=args.circle_radius, omega=args.circle_omega, loops=args.circle_loops,
            height=args.circle_z if args.circle_z is not None else args.goto_height,
            entry_t=args.entry_t, ramp_t=args.ramp_t, return_t=args.return_t,
            hold_s=args.final_hold_s)

        # ---- 状态（cflib log 回调线程写入）----
        self.state = None          # fused: {"position","velocity","rotation","t"}
        self._R = None             # att 块缓存: 当前旋转矩阵
        self._t_att = -1e9         # att 块最近到达时刻
        self.vbat = float("nan")
        self.ctrltarget = (float("nan"),) * 3      # ctrltarget.roll/pitch/yaw（legacy setpoint 层）
        self.ctrltarget_mode = (0, 0, 0)           # ctrltarget.mode_*：1=ABS, 2=Velocity(RATE)
        self.rate_setpoint = (float("nan"),) * 3   # controller.r_roll/r_pitch/r_yaw（诊断）
        self.cmd_rp = (float("nan"), float("nan")) # controller.cmd_roll/cmd_pitch（速率环输出）
        self.ctrl_epoch = 0

        # ---- mocap ----
        self.mocap = None
        self.mocap_last = 0.0
        self.inject_skips = 0
        self.inject_started = False

        # ---- 任务状态 ----
        self.phase = "init"
        self.phase_t0 = time.monotonic()
        self.t0 = self.phase_t0
        self.last_tick = self.phase_t0
        self.estop = False
        self.estop_count = 0
        self.abort_reason = ""
        self.finished = False
        self.armed = False
        self.hold_xy = None
        self.hold_yaw = 0.0
        self.climb_z0 = 0.0
        self.enter_z = None
        self.conv_samples = []
        self.conv_t0 = None
        self.volt_samples = []
        self.volt_t0 = None
        self.raw_scale = 1.0
        self.arrive_ticks = 0
        self.settle_since = None
        self.pending_hl = None      # (kind, t0) — hl 阶段的等待标记
        self.traj_start = None      # CTBR 轨迹计时起点

        # ---- CSV ----
        log_dir = args.log_dir or os.path.join(_SELF, "logs")
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, f"0902ctbr_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_f)
        self.csv.writerow(["t", "phase", "dt",
                           "px", "py", "pz", "vx", "vy", "vz",
                           "ex", "ey", "ez", "evx", "evy", "evz",
                           "rx", "ry", "rz",
                           "yaw", "tilt",
                           "gyro_x", "gyro_y", "gyro_z",
                           "cmd_p", "cmd_q", "cmd_r", "thrust_N", "thr_raw16",
                           "raw_scale",
                           "up_x", "up_y", "up_z", "up_yaw", "dup_xyz",
                           "ct_r_roll", "ct_r_pitch", "ct_r_yaw",
                           "vbat", "estop"])
        self._csv_cnt = 0

    # ============================================ 安全出口
    def install_signals(self):
        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)

    def _on_sigint(self, _s, _f):
        self.estop_count += 1
        if self.estop_count >= 2:
            sys.stderr.write("\n[急停] 第二次 Ctrl+C —— 立即退出\n")
            sys.stderr.flush()
            os._exit(1)
        sys.stderr.write("\n[急停] 收到 Ctrl+C —— 立即进入紧急降落(land)…\n")
        sys.stderr.flush()
        self.estop = True
        self.abort_reason = "用户 Ctrl+C 急停"

    def check_estop(self):
        if self.estop:
            self.emergency_land(self.abort_reason or "急停")
            return True
        return False

    def emergency_land(self, reason):
        """最高优先级急停（用户要求：Ctrl+C = 进入 land 指令）。

        顺序：1) 停止 CTBR 发包（最后指令会残留, 但优先级马上被 relax 覆盖）
              2) send_notify_setpoint_stop(0)  → commander 优先级降到 LOWEST
              3) enHighLevel=1 → hl.land(0.0, 1.0)（快降）
              4) 等落地确认 → disarm
        若 8s 内没落地 → 零推力兜底 + disarm 提示。
        """
        if self.phase == "abort_done":
            return
        self.phase = "abort_done"
        self.finished = True
        self.get_logger(f"[ESTOP] {reason} — 开始紧急降落序列")
        if not self.fly or self.cf is None:
            if self.cf is not None:
                self._safe_disarm()
            return
        try:
            try:
                self.cf.commander.send_notify_setpoint_stop(0)
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self.cf.param.set_value("commander.enHighLevel", 1)
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self.cf.high_level_commander.land(0.0, 1.0)
            except Exception:
                pass
            # 等落地：z(融合) < 0.08 或超时
            t_deadline = time.time() + self.args.land_timeout
            landed = False
            while time.time() < t_deadline:
                st = self.state
                if st is not None and float(st["position"][2]) < 0.08:
                    landed = True
                    break
                time.sleep(0.1)
            self.get_logger(f"[ESTOP] 落地确认={landed}")
            self._safe_disarm()
            time.sleep(0.3)
            if not landed:
                # 兜底：零推力 + 提示断电
                try:
                    self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                except Exception:
                    pass
                self.get_logger("[ESTOP] land 未确认 → 已发零推力兜底；若飞机仍在动请断开电源！")
        except Exception as e:
            self.get_logger(f"[ESTOP] 异常：{e}")
        finally:
            try:
                self.cf.close_link()
            except Exception:
                pass

    def _safe_disarm(self):
        try:
            self.cf.supervisor.send_arming_request(False)
            self.get_logger("[ESTOP] 已发送 DISARM")
        except Exception:
            pass

    def get_logger(self, msg):
        print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)

    # ============================================ 数据回调（cflib log 线程）
    def _make_log_cb(self):
        # ★ cflib 惯例坑：多个块共用一个回调时，data 只含【本块】的变量！
        #   "st" 块只有 stateEstimate.*（位置/速度）；"att" 块才有
        #   stateEstimateZ.quat + gyro.* —— 二者分开到达，必须各自缓存、
        #   都在新鲜窗口内才装配 self.state（不会丢帧: 两路同为 100Hz）。
        def cb(_ts, data, _lk):
            try:
                if "stateEstimateZ.quat" in data:                # ---- att 块
                    q = quatdecompress(int(data["stateEstimateZ.quat"]))
                    if np.all(np.isfinite(q)):
                        self._R = quaternion_to_rotation(q[0], q[1], q[2], q[3])
                        self._t_att = time.monotonic()
                    if "gyro.x" in data:
                        self.gyro = np.array([data["gyro.x"], data["gyro.y"], data["gyro.z"]],
                                             dtype=float) / 180.0 * math.pi
                if "stateEstimate.x" in data:                    # ---- st 块
                    p = np.array([data["stateEstimate.x"], data["stateEstimate.y"],
                                  data["stateEstimate.z"]], dtype=float)
                    v = np.array([data["stateEstimate.vx"], data["stateEstimate.vy"],
                                  data["stateEstimate.vz"]], dtype=float)
                    if (self._R is not None and np.all(np.isfinite(p)) and np.all(np.isfinite(v))
                            and np.max(np.abs(p)) < 10.0
                            and (time.monotonic() - self._t_att) <= self.args.state_timeout):
                        self.state = {"position": p, "velocity": v, "rotation": self._R.copy(),
                                      "t": time.monotonic()}
                if "pm.vbat" in data:                            # ---- bat 块
                    self.vbat = float(data["pm.vbat"])
                if "ctrltarget.roll" in data:                    # ---- ct 块（legacy setpoint 层）
                    self.ctrltarget = (float(data["ctrltarget.roll"]),
                                       float(data["ctrltarget.pitch"]),
                                       float(data["ctrltarget.yaw"]))
                    if "ctrltarget.mode_roll" in data:
                        self.ctrltarget_mode = (int(data["ctrltarget.mode_roll"]),
                                                int(data["ctrltarget.mode_pitch"]),
                                                int(data["ctrltarget.mode_yaw"]))
                    self.ctrl_epoch += 1
                if "controller.r_roll" in data:                  # ---- ctr 块（诊断）
                    self.rate_setpoint = (float(data["controller.r_roll"]),
                                          float(data["controller.r_pitch"]),
                                          float(data["controller.r_yaw"]))
                    if "controller.cmd_roll" in data:
                        self.cmd_rp = (float(data["controller.cmd_roll"]),
                                       float(data["controller.cmd_pitch"]))
            except (TypeError, ValueError):
                return
        return cb

    # ============================================ mocap
    def _mocap_get(self):
        self.mocap.poll()
        pose, ts = self.mocap.get()
        return pose, ts

    def _inject_once(self, pose):
        """把动捕位姿注入固件（[x,y,z]+[w,x,y,z]→ send_extpose(x,y,z,qx,qy,qz,qw)）。
        返回是否注入（带 0.5m 跳变闸门）。"""
        x, y, z, qw, qx, qy, qz = pose
        try:
            self.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
            return True
        except Exception:
            return False

    def _mocap_delta(self):
        """动捕与融合的 3D 差（NaN=mocap 不可用）。"""
        pose, _ = self._mocap_get()
        st = self.state
        if pose is None or st is None:
            return float("nan")
        return float(np.linalg.norm(np.array(pose[:3]) - st["position"]))

    # ============================================ 预检
    def _phase_elapsed(self):
        return time.monotonic() - self.phase_t0

    def _goto(self, phase):
        self.phase = phase
        self.phase_t0 = time.monotonic()
        self.settle_since = None
        self.arrive_ticks = 0

    def _envelope_check(self):
        """起飞前包线体检：轨迹峰值 vs 限幅（角度/速度/推力/半径）。"""
        pk = self.mission.peaks()
        thr_need = self.args.mass * 9.80665 / math.cos(math.radians(pk["tilt_max_deg"]))
        items = [("峰值速度", pk["v_max"], self.args.max_velocity, "m/s"),
                 ("需求倾角", pk["tilt_max_deg"], self.args.max_tilt_abort_deg, "°"),
                 ("需求推力", thr_need, self.args.max_thrust, "N"),
                 ("离中心半径", self.args.circle_radius, self.args.max_room - 0.2, "m")]
        self.get_logger("---- 包线体检 ----")
        ok = True
        for name, need, limit, unit in items:
            if need > limit:
                self.get_logger(f"  ✘ {name:8s} {need:7.2f} > {limit:.2f} {unit}  ← 拒飞")
                ok = False
            else:
                self.get_logger(f"  ✔ {name:8s} {need:7.2f} ≤ {limit:.2f} {unit}")
        if not ok:
            self._refuse(f"包线体检不通过")
        return ok

    def _refuse(self, reason):
        self.get_logger(f"[拒绝起飞] {reason}")
        self.finished = True
        self.phase = "done"

    # ============================================ 状态机（每拍）
    def tick(self):
        now = time.monotonic()
        dt = max(now - self.last_tick, 1.0 / self.args.rate)
        self.last_tick = now

        # 全局外部位姿注入（开盘后所有阶段都保持 EKF 与动捕同源）
        self._maybe_inject()

        if self.check_estop():
            return
        # 空中/地上都持续的电压监测（不满足即急停降落）
        if self.phase in ("ctbr", "hl_takeoff", "ctbr_enter", "hl_land") and \
                math.isfinite(self.vbat) and self.vbat < self.args.critical_voltage:
            self.estop = True
            self.abort_reason = f"低电量 {self.vbat:.2f}V < {self.args.critical_voltage}V"
            return

        try:
            handler = getattr(self, "_ph_" + self.phase)
        except AttributeError:
            handler = None
        if handler:
            try:
                handler()
            except Exception as e:
                self.get_logger(f"阶段 {self.phase} 异常：{e}\n{traceback.format_exc()}")
                self.estop = True
                self.abort_reason = f"主机代码异常：{e}"

        # ---- 逐拍 CSV（含控制律输出）----
        self._log_row(now, dt)

    # ------------------------------------------------------------- 阶段
    def _ph_init(self):
        """等融合状态到达（无 EKF 门控——刚复位未注入外部位姿时本就会漂，见 wait_mocap）。"""
        if self.state is None:
            return
        self._goto("wait_mocap")

    def _start_injection(self):
        """动捕确认后：复位 EKF 世界系 → 开启外部位姿注入（地面注入安全）。
        ★ 不注入的纯 IMU EKF 在复位后会因偏置未修正而漂移(|v|持续爬升, 位置跑飞)——
          老路线(mct+server 常驻注入)与实飞都没有这个问题；影子模式也要注入,
          否则 EKF 与动捕不是同一坐标系, 收敛/偏差门全部失真。"""
        if self.inject_started:
            return
        if self.cf is not None:
            try:
                self.cf.param.set_value("kalman.resetEstimation", 1)
                time.sleep(0.4)
            except Exception as e:
                self.get_logger(f"⚠ EKF 复位失败: {e}")
        self.inject_started = True
        self.get_logger(f"✓ 动捕在流（源={self.mocap.name}）→ EKF 已复位，外部位姿注入开启（闸门 0.5m）")

    def _maybe_inject(self):
        """全局每拍注入：最新动捕 → send_extpose；附 0.5m 跳变闸门 + 越界计数。"""
        if not self.inject_started or self.cf is None:
            return
        pose, pts = self._mocap_get()
        if pose is None:
            return
        self.mocap_last = pts
        st = self.state
        if st is None:
            return
        d = float(np.linalg.norm(np.array(pose[:3]) - st["position"]))
        if d < 0.5:
            self._inject_once(pose)
            self.inject_skips = 0
        else:
            self.inject_skips += 1

    def _ph_wait_mocap(self):
        pose, pts = self._mocap_get()
        if pose is None:
            if self._phase_elapsed() > 10.0:
                self._refuse("10s 未收到动捕位姿（检查 mocap 源与 --mocap-source）")
            return
        if not self.inject_started:
            self._start_injection()
            return
        if self._phase_elapsed() < 1.5:      # 注入后再等 EKF 收敛
            return
        st = self.state
        if st is None:
            return
        # EKF 收敛门控（现在 EKF 已融合动捕）：1s 位置散布 < 2cm、|v| < 0.05、动捕偏差 < 0.30
        if self.conv_t0 is None:
            self.conv_t0 = time.monotonic()
        self.conv_samples.append(st["position"].copy())
        if time.monotonic() - self.conv_t0 >= 1.0:
            arr = np.array(self.conv_samples[-100:])
            spread = float(np.max(np.ptp(arr, axis=0)))
            spd = float(np.linalg.norm(st["velocity"]))
            d = self._mocap_delta()
            ok = (spread < 0.02 and spd < 0.05 and math.isfinite(d) and d < 0.30)
            if ok:
                self.get_logger(f"✓ EKF 收敛（1s 散布 {spread*1000:.1f}mm, |v|={spd:.3f}, 动捕偏差 {d:.3f}m）")
                self._goto("preflight_voltage")
            else:
                self.get_logger(f"… 未收敛（散布 {spread*1000:.1f}mm, |v|={spd:.3f}, 动捕偏差 {d:.3f}m）—— 检查注入/刚体遮挡")
                self.conv_samples.clear()
                self.conv_t0 = None

    def _ph_preflight_voltage(self):
        if self.volt_t0 is None:
            self.volt_t0 = time.monotonic()
            self.volt_samples = []
            self.get_logger(f"电压预检：收集 {self.args.voltage_window:.1f}s 样本 (min {self.args.min_voltage}V)…")
        if math.isfinite(self.vbat) and self.vbat > 0:
            self.volt_samples.append(self.vbat)
        if time.monotonic() - self.volt_t0 < self.args.voltage_window:
            return
        if not self.volt_samples:
            self._refuse("无电压样本（pm.vbat 在发吗？）")
            return
        med = float(np.median(self.volt_samples))
        if med < self.args.min_voltage:
            self._refuse(f"电压 {med:.2f}V < {self.args.min_voltage}V")
            return
        scale = (self.args.voltage_ref / med) ** self.args.voltage_exponent
        if not self.bridge.set_raw_scale(scale):
            self._refuse(f"raw_scale {scale:.3f} 越界 [0.90,1.12]")
            return
        self.raw_scale = self.bridge.raw_scale
        self.get_logger(f"✓ 电压 {med:.2f}V，raw_scale={scale:.4f}")
        if self.shadow or not self.fly:
            self.get_logger("（影子：链路预检完成 —— 通道自检/ARM/HL 起飞/CTBR 均跳过，退出）")
            self.finished = True
            self._goto("done")
            return
        self._goto("arm")

    def _ph_channel_check(self):
        """ARM 后贯透自检：legacy RPYT 探针 → 回读 ctrltarget.*（setpoint 层）。

        ★ 必须在 ARM 之后：stabilizer.c:332-343 未 canFly 时 crtpCommanderBlock(true)
          且 setpoint 被强制清零 ⇒ 未 ARM 的 ctrltarget 回读恒为 0（2026-09-02 实测）。
        探针推力=0 ⇒ ARM idle 下电机不转，安全。
        实测判据（chan_compare 定案）：api(15,10,+5,0) → ctrltarget.roll≈+15、
        ctrltarget.yaw≈−5（legacy yaw 净取负）；pitch 是 legacy ABS 约定，不作判据。
        """
        if self.args.skip_channel_check:
            self._goto("hl_takeoff")
            return
        if self.ctrl_epoch == 0:
            if self._phase_elapsed() > 3.0:
                self.get_logger("✘ 无 ctrltarget 回读 —— 日志块未通，先排查再飞")
                self._refuse("通道自检失败（无回读）")
            return
        if not getattr(self, "_probe_sent", False):
            self.cf.commander.send_setpoint(15.0, 10.0, 5.0, 0)   # 推力 0（ARM idle，电机不转）
            self._probe_sent = True
            self._probe_t0 = time.monotonic()
            self.get_logger("通道自检（ARM 后）：发送 legacy setpoint(15°,10°,+5°/s,0) …")
            return
        if self._phase_elapsed() < 0.8:
            return
        tr, tp_, ty = self.ctrltarget
        mr, mp, my = self.ctrltarget_mode
        cr, cp = self.cmd_rp
        # ★ 判据（RATE 模式）：mode_roll/mode_pitch=2（速率态贯透）+ yaw≈−5（净取负）。
        #   ctrltarget.roll/pitch 是【角度态】RATE 下恒 0，不作判据。
        #   cmd_roll/cmd_pitch 也不作判据：controller_pid.c 里 thrust==0 时力矩输出被
        #   强制清零（"零推力=不输出力矩"安全设计），零推力探针下 cmd 恒 0。
        mode_ok = (mr == 2 and mp == 2)
        yaw_ok = abs(ty - (-5.0)) < 2.0
        ok = mode_ok and yaw_ok
        self.get_logger(f"通道自检回读：mode(r/p/y)=({mr},{mp},{my}) yaw={ty:+.1f} "
                        f"cmd_roll={cr:+.2f} cmd_pitch={cp:+.2f}（零推力下恒 0，不作判据）→ "
                        + ("✔ legacy RATE 贯透（yaw 净取负已验证）"
                           if ok else f"✘ 未贯透（mode_ok={mode_ok} yaw_ok={yaw_ok}）"))
        try:
            self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            time.sleep(0.3)
        except Exception:
            pass
        if not ok:
            self._refuse("通道自检失败（legacy RPYT 不通或符号异常，不要盲飞）")
            return
        # ★ 自检的 RPYT 探针把 commander 优先级顶到 CRTP=2；goto/takeoff 是 HIGHLEVEL=1，
        #   不 relax 会被 commanderSetSetpoint 的 `priority >= currentPriority` 丢弃。
        #   notify 是幂等元命令（降到 LOWEST）—— 连发 3 次防 RF 丢包（19:34 单发丢包后
        #   HL 被压 15s 不起飞的教训；20:03 复发）。
        try:
            for _ in range(3):
                self.cf.commander.send_notify_setpoint_stop(0)   # → commanderRelaxPriority
                time.sleep(0.15)
        except Exception as e:
            self.get_logger(f"⚠ relax 失败：{e}")
        self._goto("hl_takeoff")

    def _ph_arm(self):
        self.get_logger("[ARM] 发送 arming 请求…")
        try:
            self.cf.supervisor.send_arming_request(True)
        except Exception as e:
            self._refuse(f"ARM 发送失败：{e}")
            return
        self._goto("wait_armed")

    def _ph_wait_armed(self):
        try:
            bf = self.cf.supervisor.read_bitfield()
            if bf & SUP_IS_LOCKED:
                self._refuse(f"supervisor LOCKED (0x{bf:04X}) —— 请给飞机断电重上电")
                return
            if bf & SUP_IS_TUMBLED:
                self._refuse(f"supervisor TUMBLED (0x{bf:04X}) —— 飞机没放平？")
                return
            if self.cf.supervisor.is_armed and self.cf.supervisor.can_fly:
                self.armed = True
                self.get_logger(f"✓ ARM 确认 (0x{bf:04X})")
                self._goto("channel_check")
                return
        except Exception as e:
            if self._phase_elapsed() > self.args.arm_timeout:
                self._refuse(f"ARM 确认异常：{e}")
            return
        if self._phase_elapsed() > self.args.arm_timeout:
            self._refuse(f"ARM {self.args.arm_timeout:.0f}s 未确认 —— arm 只发不等，需等 supervisor ReadyToFly")
            return
        # 每 0.5s 打印一次（ARM 确认通常 1~2s，supervisor 状态机走完就过）
        if time.monotonic() - getattr(self, "_last_wait_log", -9) > 0.5:
            self._last_wait_log = time.monotonic()
            self.get_logger("… 等待 ARM 生效")
        return

    def _ph_hl_takeoff(self):
        """高层 takeoff 起飞到高度 H（不设 yaw —— HL 的 yaw 环在 EKF 坐标系下
        曾实测正反馈指数发散，19:34 起飞 3s 转到 127°；老路线同样结论）。
        位置只垂直上升：takeoff 保持当前航向不动，交棒 CTBR 时锁当前 yaw。"""
        st = self.state
        if st is None:
            if self._phase_elapsed() > 8.0:
                self.estop = True
                self.abort_reason = "HL 起飞段状态失效"
            return
        H = self.args.goto_height
        if not getattr(self, "_hl_started", False):
            self._hl_started = True
            self.hold_yaw = math.degrees(rotation_yaw(st["rotation"]))
            try:
                self.cf.param.set_value("commander.enHighLevel", 1)
                time.sleep(0.2)
                self.cf.high_level_commander.takeoff(H, self.args.goto_duration)
            except Exception as e:
                self.estop = True
                self.abort_reason = f"takeoff 发送失败：{e}"
            self.get_logger(f"HL takeoff 起飞 → 高度 {H}m（不设 yaw，保持当前航向 {self.hold_yaw:.1f}°，{self.args.goto_duration}s）")
            self._hl_t0 = time.monotonic()
            return
        # 到位判据：z 达标且速度小，连续 0.5s
        z = float(st["position"][2])
        v = np.linalg.norm(st["velocity"])
        if z >= H - 0.05 and v < 0.15:
            if self.settle_since is None:
                self.settle_since = time.monotonic()
            elif time.monotonic() - self.settle_since >= 0.5:
                self.get_logger(f"✓ HL 已到 (0,0,{z:.3f}) → 交棒 CTBR")
                self._enter_ctbr()
                return
        else:
            self.settle_since = None
        if time.monotonic() - getattr(self, "_last_goto_log", -9) > 0.5:
            self._last_goto_log = time.monotonic()
            self.get_logger(f"… goto 起飞中 z={z:.3f} |v|={v:.2f} (t={self._phase_elapsed():.1f}s)")
        if self._phase_elapsed() > self.args.goto_timeout:
            self.estop = True
            self.abort_reason = f"goto 超时（z={z:.3f}）"

    def _enter_ctbr(self):
        st = self.state
        self.hold_xy = st["position"][:2].copy()
        self.hold_yaw = math.degrees(rotation_yaw(st["rotation"]))
        self.enter_z = float(st["position"][2])
        self.controller.reset()
        # 交棒：relax 优先级（停高层规划器）→ 关高层 → 零包（惯例；manual 无推力锁）
        try:
            self.cf.commander.send_notify_setpoint_stop(0)
            self.cf.param.set_value("commander.enHighLevel", 0)
            time.sleep(0.05)
        except Exception as e:
            self.get_logger(f"⚠ 交棒告警：{e}")
        try:
            self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)   # legacy 推力锁解锁零包
        except Exception:
            pass
        self.traj_start = time.monotonic()
        self._goto("ctbr")
        self.get_logger(f"★ CTBR 接管：xy=({self.hold_xy[0]:.3f},{self.hold_xy[1]:.3f}) "
                        f"yaw={self.hold_yaw:.1f}° | 任务：R={self.args.circle_radius} 圆 "
                        f"@{self.mission.z:.2f}m，总时长 {self.mission.duration:.1f}s")

    def _ctbr_guard(self, target):
        """飞行包线闸门（对当前轨迹目标做跟踪误差, 不是对起飞点）。"""
        st = self.state
        a = self.args
        p = st["position"]
        track = float(np.linalg.norm(p[:2] - target["position"][:2]))
        if track > a.max_track_error:
            return f"水平跟踪误差 {track:.2f}m > {a.max_track_error:.2f}m"
        room = float(np.linalg.norm(p[:2]))
        if room > a.max_room:
            return f"离原点 {room:.2f}m > {a.max_room:.2f}m"
        if p[2] > a.goto_height + a.max_z_over:
            return f"高度 {p[2]:.2f}m > {a.goto_height + a.max_z_over:.2f}m"
        if p[2] < 0.04:
            return f"高度 {p[2]:.3f}m 过低"
        if rotation_tilt(st["rotation"]) > math.radians(a.max_tilt_abort_deg):
            return f"倾角 {math.degrees(rotation_tilt(st['rotation'])):.1f}° > {a.max_tilt_abort_deg}°"
        if float(np.linalg.norm(st["velocity"])) > a.max_velocity:
            return f"速度 {float(np.linalg.norm(st['velocity'])):.2f}m/s > {a.max_velocity}"
        if math.isfinite(self.vbat) and self.vbat < a.critical_voltage:
            return f"电压 {self.vbat:.2f}V < {a.critical_voltage}V"
        return None

    def _ph_ctbr(self):
        st = self.state
        if st is None:
            self.estop = True
            self.abort_reason = "CTBR 期间状态失效(>state_timeout)"
            self.get_logger(f"✗ {self.abort_reason}")
            return
        if time.monotonic() - st["t"] > self.args.state_timeout:
            self.estop = True
            self.abort_reason = f"状态超龄 {(time.monotonic()-st['t'])*1000:.0f}ms"
            self.get_logger(f"✗ {self.abort_reason}")
            return
        # 动捕断流 / 持续偏差门（注入已由 tick() 全局执行）
        if abs(time.monotonic() - self.mocap_last) > 0.5:
            f, h, age = self.mocap.diag()
            self.estop = True
            self.abort_reason = "动捕断流 > 0.5s"
            self.get_logger(f"✗ {self.abort_reason} | mocap线程: 总帧={f} 命中={h} "
                            f"线程帧龄={age*1000:.0f}ms | 主循环 tick 龄="
                            f"{(time.monotonic()-self.last_tick)*1000:.0f}ms")
            return
        if self.inject_skips >= 300:
            self.estop = True
            self.abort_reason = "动捕-融合偏差持续 > 0.5m"
            self.get_logger(f"✗ {self.abort_reason}")
            return
        t = time.monotonic() - self.traj_start
        target, done = self.mission.evaluate(t)
        target["yaw"] = math.radians(self.hold_yaw)
        bad = self._ctbr_guard(target)
        if bad:
            self.estop = True
            self.abort_reason = f"CTBR 越界：{bad}"
            return
        dt = 1.0 / self.args.rate
        cmd = self.controller.compute(st, target, dt)
        p, q, r = cmd["body_rate_command"]
        rs, ps, ys, raw16 = self.bridge.to_legacy(p, q, r, cmd["collective_thrust"], self.signs)
        if self.fly:
            self.cf.commander.send_setpoint(rs, ps, ys, raw16)
        self._last_cmd = (cmd, (rs, ps, ys, raw16), target)
        if done:
            self.get_logger("★ 任务（圆+回点）完成 → 交回高层降落")
            self._goto("relax_before_land")

    def _ph_relax_before_land(self):
        if getattr(self, "_relaxed", False):
            return
        self._relaxed = True
        try:
            self.cf.commander.send_notify_setpoint_stop(0)
            self.cf.param.set_value("commander.enHighLevel", 1)
        except Exception as e:
            self.estop = True
            self.abort_reason = f"relax 失败：{e}"
            return
        self._goto("hl_land")

    def _ph_hl_land(self):
        st = self.state
        if not getattr(self, "_land_sent", False):
            self._land_sent = True
            if self.fly:
                try:
                    self.cf.high_level_commander.land(self.args.land_height, self.args.land_rate)
                    self.get_logger(f"[land] height={self.args.land_height} rate={self.args.land_rate}")
                except Exception as e:
                    self.estop = True
                    self.abort_reason = f"land 发送失败：{e}"
            return
        if self.fly:
            if st is not None and float(st["position"][2]) <= self.args.land_height + 0.05:
                if self.settle_since is None:
                    self.settle_since = time.monotonic()
                elif time.monotonic() - self.settle_since >= 0.7:
                    self.get_logger(f"✓ 已落地 z={st['position'][2]:.3f}")
                    self._goto("disarm")
                    return
            else:
                self.settle_since = None
            if self._phase_elapsed() > self.args.land_timeout:
                self.get_logger(f"⚠ land 超时（z={st['position'][2] if st else '?'}），直接 disarm")
                self._goto("disarm")
        else:
            self._goto("disarm")

    def _ph_disarm(self):
        self.get_logger("[DISARM] …")
        try:
            self.cf.supervisor.send_arming_request(False)
        except Exception as e:
            self.get_logger(f"⚠ disarm 异常：{e}")
        self.armed = False
        self._goto("done")
        self.finished = True

    # ============================================ 日志
    def _log_row(self, now, dt):
        try:
            row = {
                "t": f"{now - self.t0:.3f}", "phase": self.phase, "dt": f"{dt:.4f}",
                "vbat": "%.3f" % self.vbat if math.isfinite(self.vbat) else "",
                "raw_scale": "%.4f" % self.bridge.raw_scale,
                "estop": int(self.estop),
            }
            st = self.state
            if st is not None:
                p, v = st["position"], st["velocity"]
                row.update({"px": f"{p[0]:.4f}", "py": f"{p[1]:.4f}", "pz": f"{p[2]:.4f}",
                            "vx": f"{v[0]:.3f}", "vy": f"{v[1]:.3f}", "vz": f"{v[2]:.3f}",
                            "yaw": f"{math.degrees(rotation_yaw(st['rotation'])):.2f}",
                            "tilt": f"{math.degrees(rotation_tilt(st['rotation'])):.2f}"})
                py_r = [f"{float(x):.2f}" for x in getattr(self, 'gyro', [0, 0, 0])]
                row.update({"gyro_x": py_r[0], "gyro_y": py_r[1], "gyro_z": py_r[2]})
            else:
                row.update({"px": "", "py": "", "pz": "", "tilt": ""})
            if getattr(self, "_last_cmd", None):
                cmd, man, tgt = self._last_cmd
                row["ex"] = f"{cmd['position_error'][0]:.4f}"
                row["ey"] = f"{cmd['position_error'][1]:.4f}"
                row["ez"] = f"{cmd['position_error'][2]:.4f}"
                row["evx"] = f"{cmd['velocity_error'][0]:.3f}"
                row["evy"] = f"{cmd['velocity_error'][1]:.3f}"
                row["evz"] = f"{cmd['velocity_error'][2]:.3f}"
                row["rx"], row["ry"], row["rz"] = (f"{float(x):.4f}" for x in tgt["position"])
                bc = cmd["body_rate_command"]
                row.update({"cmd_p": f"{math.degrees(bc[0]):.2f}", "cmd_q": f"{math.degrees(bc[1]):.2f}",
                            "cmd_r": f"{math.degrees(bc[2]):.2f}",
                            "thrust_N": f"{cmd['collective_thrust']:.4f}",
                            "thr_raw16": f"{man[3]}"})
            pose, _ = self._mocap_get()
            if pose is not None:
                x, y, z, qw, qx, qy, qz = pose
                up_yaw = math.degrees(math.atan2(
                    2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
                row.update({"up_x": f"{x:.4f}", "up_y": f"{y:.4f}", "up_z": f"{z:.4f}",
                            "up_yaw": f"{up_yaw:.2f}"})
                if st is not None:
                    row["dup_xyz"] = f"{np.linalg.norm(np.array(pose[:3]) - st['position']):.4f}"
            row.update({"ct_r_roll": f"{self.rate_setpoint[0]:.2f}",
                        "ct_r_pitch": f"{self.rate_setpoint[1]:.2f}",
                        "ct_r_yaw": f"{self.rate_setpoint[2]:.2f}"})
            self.csv.writerow([row.get(k, "") for k in ("t", "phase", "dt", "px", "py", "pz",
                               "vx", "vy", "vz", "ex", "ey", "ez", "evx", "evy", "evz",
                               "rx", "ry", "rz", "yaw", "tilt", "gyro_x", "gyro_y", "gyro_z",
                               "cmd_p", "cmd_q", "cmd_r", "thrust_N", "thr_raw16", "raw_scale",
                               "up_x", "up_y", "up_z", "up_yaw", "dup_xyz",
                               "ct_r_roll", "ct_r_pitch", "ct_r_yaw", "vbat", "estop")])
            self._csv_cnt += 1
            if self._csv_cnt % 25 == 0:
                self.csv_f.flush()
        except Exception:
            pass


# ================================================================ main
def build_parser():
    p = argparse.ArgumentParser(description="0902 CTBR 圆轨迹飞行（cflib legacy RPYT 通道 send_setpoint）")
    p.add_argument("--uri", default="radio://0/80/2M/E7E7E7E703")
    # ---- 模式 ----
    p.add_argument("--fly", action="store_true", help="实飞（默认影子：真实链路数据，不发指令不 ARM）")
    p.add_argument("--shadow", action="store_true", help="影子模式（同默认，显式写法）")
    p.add_argument("--dry-run", action="store_true", help="纯轨迹/包线自检，不连飞机")
    # ---- mocap ----
    p.add_argument("--mocap-source", choices=("auto", "lmc", "udp"), default="auto",
                   help="auto=优先 libmotioncapture 直连(lmc)否则 udp 桥")
    p.add_argument("--lmc-backend", choices=("vrpn", "nokov"), default="nokov",
                   help="lmc 后端：nokov=官方 SDK 广播(默认, 100Hz, 无噪声)；vrpn=NOKOV VRPN 输出(197Hz, 有死刚体警告噪声)")
    p.add_argument("--lmc-path", default=None, help="motioncapture 模块目录(默认仓库 mocap_direct/lib)")
    p.add_argument("--udp-host", default="127.0.0.1", help="UDP 桥地址（mocap_udp_bridge.py）")
    p.add_argument("--udp-port", type=int, default=9000)
    p.add_argument("--vrpn-host", default="10.1.1.198")
    p.add_argument("--vrpn-name", default="cf3")
    # ---- 任务 ----
    p.add_argument("--goto-height", type=float, default=0.15, help="HL goto 起飞高度 m")
    p.add_argument("--goto-duration", type=float, default=2.5)
    p.add_argument("--circle-radius", type=float, default=1.0, help="圆周半径 m")
    p.add_argument("--circle-omega", type=float, default=0.40, help="圆周角速度 rad/s(v=Rω)")
    p.add_argument("--circle-loops", type=float, default=1.0)
    p.add_argument("--circle-z", type=float, default=None, help="圆飞行高度(默认= goto 高度)")
    p.add_argument("--entry-t", type=float, default=2.5, help="入口(中心→圆周)时长 s")
    p.add_argument("--ramp-t", type=float, default=1.5, help="角速度斜坡时长 s")
    p.add_argument("--return-t", type=float, default=2.5, help="返回(圆周→原点)时长 s")
    p.add_argument("--final-hold-s", type=float, default=2.0, help="回点悬停 s")
    p.add_argument("--rate", type=float, default=100.0)
    # ---- 物理与增益（0901 定案默认）----
    p.add_argument("--mass", type=float, default=0.0434)
    p.add_argument("--fmax", type=float, default=1.234, help="满电静态总推力 N（悬停 raw 反推）")
    p.add_argument("--max-thrust", type=float, default=0.80)
    p.add_argument("--max-tilt-deg", type=float, default=10.0, help="控制器倾角上限")
    p.add_argument("--max-body-rate", type=float, nargs=3, default=[3.0, 3.0, 2.0])
    p.add_argument("--kp", type=float, nargs=3, default=[0.30, 0.30, 0.45])
    p.add_argument("--kv", type=float, nargs=3, default=[0.25, 0.25, 0.35])
    p.add_argument("--katt", type=float, nargs=3, default=[8.0, 8.0, 4.0])
    p.add_argument("--ff", action="store_true", help="姿态角速度前馈(高速机动建议开)")
    # ---- 符号（默认 = 推导的 manual 通道直通映射，见文件头）----
    p.add_argument("--roll-sign", type=float, default=1.0)
    p.add_argument("--pitch-sign", type=float, default=1.0)
    p.add_argument("--yaw-sign", type=float, default=1.0)
    # ---- 安全 ----
    p.add_argument("--state-timeout", type=float, default=0.30, help="融合状态最大年龄 s")
    p.add_argument("--max-track-error", type=float, default=0.50)
    p.add_argument("--max-room", type=float, default=1.70, help="离原点绝对半径上限")
    p.add_argument("--max-z-over", type=float, default=0.60)
    p.add_argument("--max-tilt-abort-deg", type=float, default=30.0)
    p.add_argument("--max-velocity", type=float, default=2.0)
    p.add_argument("--critical-voltage", type=float, default=3.60)
    p.add_argument("--arm-timeout", type=float, default=10.0)
    p.add_argument("--goto-timeout", type=float, default=15.0)
    p.add_argument("--land-timeout", type=float, default=8.0)
    p.add_argument("--land-height", type=float, default=0.03)
    p.add_argument("--land-rate", type=float, default=0.8, help="HL land 下降速度 m/s")
    # ---- 电压预检 ----
    p.add_argument("--min-voltage", type=float, default=3.80)
    p.add_argument("--voltage-ref", type=float, default=4.20)
    p.add_argument("--voltage-exponent", type=float, default=0.0, help="0=关闭电压补偿(0901 定案)")
    p.add_argument("--voltage-window", type=float, default=3.0)
    # ---- 其它 ----
    p.add_argument("--skip-channel-check", action="store_true")
    p.add_argument("--log-dir", default=None)
    return p


def main():
    args = build_parser().parse_args()
    if args.dry_run:
        # ---- 纯轨迹自检 ----
        mis = CircleMission(radius=args.circle_radius, omega=args.circle_omega,
                            loops=args.circle_loops,
                            height=args.circle_z if args.circle_z is not None else args.goto_height,
                            entry_t=args.entry_t, ramp_t=args.ramp_t, return_t=args.return_t,
                            hold_s=args.final_hold_s)
        pk = mis.peaks()
        print(f"圆轨迹自检 R={mis.R}m ω={mis.omega}rad/s → v=Rω={mis.R*mis.omega:.2f} m/s, "
              f"向心加速度={mis.R*mis.omega**2:.3f} m/s², 需求倾角={pk['tilt_max_deg']:.2f}°")
        vmax = pk["v_max"]
        need_tilt = pk["tilt_max_deg"]
        print(f"全程峰值: |v|={vmax:.2f} m/s, 最大倾角={need_tilt:.2f}°, 总时长={mis.duration:.1f}s")
        print(f"限幅参考: 速度<{args.max_velocity} 倾角<{args.max_tilt_abort_deg} 推力<{args.max_thrust}N")
        for i in range(int(mis.duration) + 1):
            tgt, _ = mis.evaluate(float(i))
            print(f"  t={i:3d}s  ref=({tgt['position'][0]:+.2f},{tgt['position'][1]:+.2f},"
                  f"{tgt['position'][2]:.2f})  |v|={np.linalg.norm(tgt['velocity']):.2f}")
        print("dry-run OK（不连电台）")
        return

    test = CtbrCircleTest(args)
    test.install_signals()

    # ---- mocap 源选择（优先 libmotioncapture 直连，其次 UDP 桥）----
    lm_lib = args.lmc_path or os.path.join(_SELF, "mocap_direct", "lib")
    sys.path.insert(0, lm_lib)
    try:
        import motioncapture  # noqa: F401
        have_lmc = True
    except ImportError:
        have_lmc = False
    if args.mocap_source == "lmc" and not have_lmc:
        print(f"✘ 找不到 motioncapture 模块（{lm_lib}）—— 先按 mocap_direct/README.md 构建/放置；"
              "或改用 --mocap-source udp")
        sys.exit(1)
    if args.mocap_source == "lmc" or (args.mocap_source == "auto" and have_lmc):
        test.mocap = MocapLmc(lm_lib, args.lmc_backend, args.vrpn_host, args.vrpn_name)
    else:
        test.mocap = MocapUdp(args.udp_host, args.udp_port)

    test.get_logger("=" * 66)
    test.get_logger(f"0902 CTBR 圆轨迹 | 模式={'实飞' if args.fly else '影子'} | "
                    f"mocap={test.mocap.name} | 日志={test.csv_path}")
    test.get_logger(f"圆: R={args.circle_radius}m ω={args.circle_omega}rad/s "
                    f"z={test.mission.z:.2f}m 时长{test.mission.duration:.1f}s")
    test.get_logger(f"Ctrl+C(1次)=紧急降落/land, Ctrl+C(2次)=硬退出 — 起飞前检查包线: --dry-run 先跑")
    test.get_logger("=" * 66)

    # ---- 连接（影子也连，为了拿真实数据）----
    init_drivers()
    cf = Crazyflie()
    test.cf = cf
    conn_evt = threading.Event()

    def on_connected(uri):
        test.get_logger(f"[+] 连接成功 {uri}")
        conn_evt.set()

    def on_disconnected(uri, _e=None):
        test.get_logger(f"[!] 连接断开 {uri}")

    cf.connected.add_callback(on_connected)
    cf.disconnected.add_callback(on_disconnected)

    for attempt in range(3):
        try:
            cf.open_link(args.uri)
        except Exception as e:
            test.get_logger(f"[WARN] open_link 异常：{e}")
        if conn_evt.wait(15.0):
            break
        test.get_logger(f"[WARN] 连接等待超时（第 {attempt+1}/3 次），重连…")
        try:
            cf.close_link()
        except Exception:
            pass
        time.sleep(1.0)
    if not conn_evt.is_set():
        test.get_logger("[FATAL] 连接失败：检查 radio 独占（server/cfclient 关掉？）、飞机上电、uri")
        sys.exit(1)
    time.sleep(1.5)   # 等完全连接(参数 ToC 完成)

    # ---- 参数（主线程；回调内禁用）----
    for k, v in [("stabilizer.estimator", 2),          # Kalman
                 ("stabilizer.controller", 1),         # PID（manual 速率环在 PID 里）
                 ("locSrv.extPosStdDev", 0.001),
                 ("locSrv.extQuatStdDev", 0.05),
                 ("kalman.resetEstimation", 1),        # 上电复位 EKF 世界系(==动捕系)
                 ("flightmode.stabModeRoll", 0),       # 与本次无关，保持一致
                 ("flightmode.stabModePitch", 0),
                 ("flightmode.stabModeYaw", 0),
                 ("commander.enHighLevel", 0)]:
        try:
            cf.param.set_value(k, v)
            time.sleep(0.15)
            got = cf.param.get_value(k)
            if abs(float(got) - float(v)) > 1e-6:
                raise ValueError(f"回读 {got} != {v}")
        except Exception as e:
            test.get_logger(f"[FATAL] 设参/回读失败 {k}: {e}")
            cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            cf.supervisor.send_arming_request(False)
            cf.close_link()
            sys.exit(1)
    test.get_logger("[+] 参数已设置并回读确认")

    # ---- 日志块（≤26B/块；period 单位=毫秒）----
    # ★ 必须等 TOC 就绪再配置：未就绪 add 会在固件端产生坏块（"Error no LogEntry
    #   id=255"），飞行中固件 log 任务流断（19:39/19:45/19:47 三次实测根因）。
    #   等 TOC 的对照实验（地面/ARM/实飞 takeoff）日志流全部完美。
    t_toc = time.time()
    while time.time() - t_toc < 20.0:
        try:
            toc_ready = (len(cf.log.toc.toc) > 0 and cf.param.is_updated)
        except Exception:
            toc_ready = False
        if toc_ready:
            break
        time.sleep(0.2)
    if not toc_ready:
        test.get_logger("[FATAL] 20s 未等到参数/日志 TOC 就绪，拒绝起飞")
        sys.stdout.flush()
        os._exit(1)
    test.get_logger(f"[+] TOC 就绪（log {len(cf.log.toc.toc)} 组）")
    # 频率分级（19:39 实测教训：诊断块也降频让路）
    period_ms = int(1000.0 / args.rate)          # 控制块
    diag_ms = 100                                # 诊断块 10Hz
    blocks = [
        ("st", ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz"],
         ("float",) * 6, period_ms),
        ("att", ["stateEstimateZ.quat", "gyro.x", "gyro.y", "gyro.z"],
         ("uint32_t", "float", "float", "float"), period_ms),   # cflib 类型名 uint32_t
        ("bat", ["pm.vbat"], ("float",), 1000),
        ("ct", ["ctrltarget.roll", "ctrltarget.pitch", "ctrltarget.yaw",
                "ctrltarget.mode_roll", "ctrltarget.mode_pitch", "ctrltarget.mode_yaw"],
         ("float", "float", "float", "uint8_t", "uint8_t", "uint8_t"), diag_ms),
        ("ctr", ["controller.r_roll", "controller.r_pitch", "controller.r_yaw",
                 "controller.cmd_roll", "controller.cmd_pitch"],
         ("float", "float", "float", "float", "float"), diag_ms),
    ]
    cb = test._make_log_cb()
    try:
        # ★ 先重置固件 log 系统：上一进程 os._exit 退出未停块 → 固件端块残留
        #   → 本轮收"未配置块"数据（Error no LogEntry id=255）且占用块槽。
        cf.log.reset()
        time.sleep(0.5)
        for lname, vs, fs, per in blocks:
            lc = LogConfig(lname, per)
            for v, tp in zip(vs, fs):
                lc.add_variable(v, tp)
            lc.data_received_cb.add_callback(cb)
            cf.log.add_config(lc)
            lc.start()
            time.sleep(0.3)     # 块间留出发送/确认时间
    except Exception as e:
        test.get_logger(f"[FATAL] 日志块配置失败：{e} —— 检查变量名/类型（建议参数已改后重跑）")
        sys.stdout.flush()
        os._exit(1)
    time.sleep(0.5)
    test.get_logger(f"[+] 日志开启 5 块（st/att@{args.rate:.0f}Hz, ct/ctr@10Hz, bat@1Hz）")

    # ---- 影子/实飞主循环 ----
    try:
        test._goto("init")
        period = 1.0 / args.rate
        while not test.finished and not test.estop:
            loop_start = time.monotonic()
            test.tick()
            sleep = period - (time.monotonic() - loop_start)
            if sleep > 0:
                time.sleep(sleep)
        # 正常结束后清理
        if not test.estop and test.phase != "done":
            pass
    except KeyboardInterrupt:
        test.estop = True
        test.abort_reason = "用户 Ctrl+C 急停"
    finally:
        if test.phase != "done" and test.phase != "abort_done":
            test.emergency_land("主循环退出兜底")
        try:
            test.csv_f.flush()
            test.csv_f.close()
        except Exception:
            pass
        try:
            cf.close_link()
        except Exception:
            pass
        # ★ 用 os._exit 收尾：cflib/crazyradio 的析构在解释器退出时会段错误(实测),
        #   跳过 Python 析构即绕开 (CSV 已落盘, print 已 flush)。
        sys.stdout.flush()
        os._exit(0)


if __name__ == "__main__":
    main()

"""
================= 符号/参数依据（本脚本发送链 = legacy RPYT） =================

1. cflib (0.1.33) send_setpoint 打包（commander.py）:
       pk.port = CRTP_PORT_SETPOINT(0x03); channel 0
       pk.data = struct.pack('<fffH', roll, -pitch, yawrate, thrust)   # 只翻转 pitch

2. 固件 crtp_commander_rpyt 解码（RATE 模式, flightmode.stabMode*=0）:
       attitudeRate.roll  = +wire.roll
       attitudeRate.pitch = +wire.pitch
       attitudeRate.yaw   = -wire.yaw      // "legacy rate input is inverted"

3. 固件 PID 速率内环（controller_pid.c）反馈量 = (gyro.x, -gyro.y, gyro.z)
   ⇒ 稳态：gyro.x = attitudeRate.roll,  -gyro.y = attitudeRate.pitch,  gyro.z = attitudeRate.yaw

4. 三轴汇合（p/q/r 为真实 FLU 体轴角速度 rad/s；wire=(api.roll, -api.pitch, api.yawrate)）：
   roll  : gyro.x = +wire.roll = +api.roll                        ⇒ api.roll  = +p
   pitch : -gyro.y = wire.pitch = -api.pitch ⇒ gyro.y = api.pitch ⇒ api.pitch = +q
   yaw   : gyro.z = -wire.yaw = -api.yawrate                       ⇒ api.yawrate = -r
   ⇒ send_setpoint(deg(p), deg(q), -deg(r), raw16)，pitch/yaw 与 0901 cmd_vel_legacy 桥
     (linear.x=+q°, angular.z=-r°) 完全同语义；roll 全链直通。
   2026-09-02 chan_compare 实测印证：api(20,0,+30,0) → ctrltarget.roll=+20, yaw=-30。

5. 推力：F → raw = 60000·(F/Fmax)^(1/2)·raw_scale（N→raw 平方反解，与 0901 同），
   再 × 65535/60000 换算 legacy 16bit 满量程。固件 legacy 推力锁：第一个非零推力包
   前必须先发零推力包（_enter_ctbr 已做）。

6. 2026-09-02 实测：本机 2026.08 固件对 TYPE_MANUAL(11)（send_setpoint_manual）无响应
   （ARM 前后 ctrltarget 均不更新），故本脚本采用 legacy RPYT 通道。

5. 状态回读变量（本机固件 source/stabilizer.c log 组）：
   stateEstimate.x/y/z      LOG_FLOAT  融合位置 (m, world)
   stateEstimate.vx/vy/vz   LOG_FLOAT  融合速度 (m/s, world)
   stateEstimateZ.quat      LOG_UINT32 压缩四元数（quatdecompress 解压 → 旋转矩阵）
   gyro.x/y/z               LOG_FLOAT  体轴角速度 (deg/s)
   pm.vbat                  LOG_FLOAT  电池电压
   controller.r_roll/r_pitch/r_yaw    闭环 rateDesired（贯透自检回读）
   （stateEstimateZ.x/y/z/z 是 LOG_INT16 毫米！不要用它做位置。）
"""
