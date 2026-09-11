#!/usr/bin/env python3
"""ARM/DISARM a brushless Crazyflie 2.1 around a crazyswarm flight.

无刷 Crazyflie 2.1 (CF21BL) 在收到明确的 ARM 请求之前不会转动电机。
crazyswarm_server（C++）本身不支持 ARM，因此 hover_swarm.launch 通过
cf_arm_wrapper.sh（launch-prefix）在本脚本发送 ARM 成功后才启动 server。
ARM 状态在 cflib 断开连接后仍然保持，server 连接后即可正常飞行；
server 退出（如 roslaunch Ctrl-C）后 wrapper 会自动再次 DISARM。

手动使用：:

    python3 cf_arm.py --arm
    python3 cf_arm.py --disarm
    python3 cf_arm.py --arm --require-mocap   # 同时要求 NOKOV 刚体位姿有效
"""

import argparse
import logging
import os
import sys
import threading
import time
import warnings

import yaml

from vehicle_config import load_vehicle_entry, select_vehicle_entry, vehicle_uri

# 抑制 cflib 内部异常堆栈噪音（例如找不到 Crazyradio 时的冗长 traceback）。
logging.getLogger("cflib").setLevel(logging.CRITICAL)
logging.getLogger("cflib").addHandler(logging.NullHandler())

# 旧固件（CRTP 协议 < 12）没有 supervisor 端口，cflib 会回退到 legacy platform
# 通道发送 ARM/DISARM，功能完全正常；忽略其“请升级固件”的警告以免误导。
warnings.filterwarnings(
    "ignore",
    message=r".*supervisor subsystem requires CRTP protocol version.*",
)


def _quiet_thread_excepthook(_args):
    """忽略 cflib 后台线程在连接失败时打印的堆栈（主线程错误仍会正常报出）。"""
    pass


if hasattr(threading, "excepthook"):
    threading.excepthook = _quiet_thread_excepthook

# 优先使用带 supervisor 模块的 cflib（>= 0.1.33）。pip 安装到 ~/.local 的
# cflib 0.1.27 没有 supervisor API。
_CFLIB_SRC = os.environ.get("CFLIB_PATH", "/home/nan/crazyflie-lib-python")
if os.path.isdir(os.path.join(_CFLIB_SRC, "cflib")):
    sys.path.insert(0, _CFLIB_SRC)

DEFAULT_CONFIG = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "launch", "crazyflies.yaml")
)
DEFAULT_SERVER_IP = "10.1.1.198"
INVALID_NOKOV_COORDINATE_MM = 1_000_000.0


# supervisor.info 位定义（固件 2023+）
BIT_CAN_BE_ARMED = 0
BIT_IS_ARMED = 1
BIT_IS_AUTO_ARMED = 2
BIT_CAN_FLY = 3
BIT_IS_FLYING = 4
BIT_IS_TUMBLED = 5
BIT_IS_LOCKED = 6
BIT_IS_CRASHED = 7
BIT_DECK_FAULT = 11


def import_cflib():
    """Import cflib modules with a helpful error message."""
    try:
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        from cflib.crazyflie.log import LogConfig
    except ImportError as exc:
        raise SystemExit(
            "无法加载 cflib（需要 >= 0.1.33，含 supervisor 模块）。\n"
            "可通过 CFLIB_PATH 指向 cflib 源码目录，例如：\n"
            "  CFLIB_PATH=/home/nan/crazyflie-lib-python python3 cf_arm.py --arm\n"
            f"原始错误：{exc}"
        )
    return cflib.crtp, Crazyflie, SyncCrazyflie, LogConfig


# ---------------------------------------------------------------------------
# NOKOV 预检查（可选，--require-mocap）
# ---------------------------------------------------------------------------

class NokovPoseCheck:
    """连接 NOKOV 并要求指定刚体出现一个有效位姿。"""

    def __init__(self, server_ip: str, rigid_body: str, timeout_s: float):
        self._server_ip = server_ip
        self._rigid_body = rigid_body
        self._timeout_s = timeout_s
        self._client = None
        self._body_id = None
        self._pose = None
        self._ready = threading.Event()

    def connect(self) -> None:
        try:
            from nokov.nokovsdk import (
                DataDescriptions, DataDescriptors, POINTER, PySDKClient,
            )
        except (ImportError, OSError) as exc:
            raise RuntimeError(
                f"无法加载 NOKOV SDK：{exc}；如不需要此检查请去掉 --require-mocap"
            )

        client = PySDKClient()
        if client.Initialize(self._server_ip.encode("utf-8")) != 0:
            raise RuntimeError(f"连接 NOKOV 服务器 {self._server_ip} 失败")
        client.PySetVerbosityLevel(0)

        descriptions = POINTER(DataDescriptions)()
        if client.PyGetDataDescriptions(descriptions) != 0 or not descriptions:
            raise RuntimeError("读取 NOKOV 刚体描述失败")

        body_id = None
        data = descriptions.contents
        for index in range(data.nDataDescriptions):
            item = data.arrDataDescriptions[index]
            if item.type != DataDescriptors.Descriptor_RigidBody.value:
                continue
            body = item.Data.RigidBodyDescription.contents
            if body.szName.decode("utf-8", errors="replace") == self._rigid_body:
                body_id = body.ID
                break
        if body_id is None:
            raise RuntimeError(f"NOKOV 中找不到刚体 {self._rigid_body!r}")

        self._client = client
        self._body_id = body_id
        client.PySetDataCallback(self._on_frame, None)

    def _on_frame(self, frame_pointer, _user_data) -> None:
        if not frame_pointer:
            return
        frame = frame_pointer.contents
        body = None
        for index in range(frame.nRigidBodies):
            candidate = frame.RigidBodies[index]
            if candidate.ID == self._body_id:
                body = candidate
                break
        if body is None:
            return
        if max(abs(body.x), abs(body.y), abs(body.z)) >= INVALID_NOKOV_COORDINATE_MM:
            return
        self._pose = (body.x / 1000.0, body.y / 1000.0, body.z / 1000.0)
        self._ready.set()

    def require_pose(self):
        if not self._ready.wait(self._timeout_s):
            raise RuntimeError(
                f"{self._timeout_s:.1f}s 内未收到刚体 {self._rigid_body!r} 的有效位姿"
            )
        return self._pose

    def close(self) -> None:
        if self._client is not None:
            # nokovpy 3.0.1 在析构函数中执行 Uninitialize 和 DestroyClient。
            del self._client
            self._client = None


# ---------------------------------------------------------------------------
# supervisor.info 状态确认
# ---------------------------------------------------------------------------

def _has_log_variable(crazyflie, name: str) -> bool:
    return crazyflie.log.toc.get_element_by_complete_name(name) is not None


def _critical_reason(info: int):
    reasons = []
    if info & (1 << BIT_IS_TUMBLED):
        reasons.append("飞机已翻倒")
    if info & (1 << BIT_IS_LOCKED):
        reasons.append("supervisor 已锁定，需重启飞机")
    if info & (1 << BIT_IS_CRASHED):
        reasons.append("检测到碰撞")
    if info & (1 << BIT_DECK_FAULT):
        reasons.append("扩展板故障")
    return "；".join(reasons) if reasons else None


class ArmStatus:
    """通过 supervisor.info 日志等待 ARM/DISARM 确认。"""

    def __init__(self, crazyflie, LogConfig):
        self._info = None
        self._error = None
        self._ready = threading.Event()
        config = LogConfig(name="arm check", period_in_ms=100)
        config.add_variable("supervisor.info")
        # 必须先注册到 Crazyflie 日志系统（add_config 会给 config.cf 赋值），
        # 否则 config.start() 会因 config.cf 为 None 报
        # 'NoneType' object has no attribute 'link'。
        crazyflie.log.add_config(config)
        config.data_received_cb.add_callback(self._on_data)
        config.error_cb.add_callback(self._on_error)
        config.start()
        self._config = config
        self._crazyflie = crazyflie

    def _on_data(self, _timestamp, data, _config):
        self._info = int(data.get("supervisor.info", 0))
        self._ready.set()

    def _on_error(self, *args):
        self._error = " ".join(str(value) for value in args)
        self._ready.set()

    def close(self) -> None:
        try:
            self._config.stop()
        except BaseException:
            pass
        try:
            self._crazyflie.log.remove_config(self._config)
        except BaseException:
            pass

    def wait_until(self, want_armed: bool, timeout_s: float) -> int:
        if not self._ready.wait(timeout_s):
            raise RuntimeError(f"{timeout_s:.1f}s 内未收到 supervisor.info 日志")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._error is not None:
                raise RuntimeError(f"supervisor.info 日志错误：{self._error}")
            if self._info is not None:
                if want_armed:
                    reason = _critical_reason(self._info)
                    if reason is not None:
                        raise RuntimeError(
                            f"ARM 被拒绝：{reason}；supervisor.info=0x{self._info:04x}"
                        )
                    if (
                        self._info & (1 << BIT_IS_ARMED)
                        and self._info & (1 << BIT_CAN_FLY)
                    ):
                        return self._info
                else:
                    if not (self._info & (1 << BIT_IS_ARMED)):
                        return self._info
            time.sleep(0.05)
        action = "ARM" if want_armed else "DISARM"
        raise RuntimeError(
            f"{timeout_s:.1f}s 内未确认{action}；supervisor.info=0x{self._info or 0:04x}"
        )


def send_and_confirm(crazyflie, arm: bool, timeout_s: float, LogConfig) -> int:
    supervisor = getattr(crazyflie, "supervisor", None)
    sender = getattr(supervisor, "send_arming_request", None)
    if not callable(sender):
        raise RuntimeError(
            "当前 cflib 不支持 supervisor ARM API，请使用 cflib >= 0.1.33"
        )
    if not _has_log_variable(crazyflie, "supervisor.info"):
        raise RuntimeError("固件缺少 supervisor.info 日志变量，无法确认 ARM 状态")

    status = ArmStatus(crazyflie, LogConfig)
    try:
        sender(arm)
        action = "ARM" if arm else "DISARM"
        print(f"已发送{action}请求，等待固件确认……", flush=True)
        info = status.wait_until(arm, timeout_s)
        print(f"固件已确认{action}：supervisor.info=0x{info:04x}", flush=True)
        return info
    finally:
        status.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="对无刷 Crazyflie 2.1 发送 ARM/DISARM 请求并等待固件确认",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--arm", dest="arm", action="store_true", default=argparse.SUPPRESS,
        help="发送 ARM 请求",
    )
    group.add_argument(
        "--disarm", dest="arm", action="store_false", default=argparse.SUPPRESS,
        help="发送 DISARM 请求",
    )
    parser.add_argument(
        "--config", default=DEFAULT_CONFIG,
        help="包含 crazyflies 列表的 YAML 参数文件",
    )
    parser.add_argument(
        "--cf-id", type=int, default=None,
        help="按 id 选择飞机；省略时选择唯一 ctbr_enabled=true 的飞机",
    )
    parser.add_argument(
        "--uri", default=None,
        help="覆盖参数文件中的 Crazyflie 无线 URI",
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="等待状态确认的最长时间 s")
    parser.add_argument(
        "--total-timeout", type=float, default=35.0,
        help="连接+发送+确认的总超时 s（防止无线丢包时无限重试）",
    )
    parser.add_argument(
        "--require-mocap", action="store_true",
        help="ARM 前要求 NOKOV 刚体位姿有效（可选安全检查）",
    )
    parser.add_argument("--server-ip", default=DEFAULT_SERVER_IP, help="NOKOV 服务器 IP")
    parser.add_argument(
        "--rigid-body", default=None,
        help="覆盖参数文件中的 NOKOV 刚体名称，默认 cf<ID>",
    )
    parser.add_argument("--mocap-timeout", type=float, default=10.0, help="等待首个位姿的最长时间 s")
    return parser


def _run_with_deadline(fn, timeout_s: float):
    """在守护线程中运行 fn；超过 timeout_s 则抛 TimeoutError。

    cflib 在无线丢包时会无限重试（send_packet 每 0.2s 重发），这里加总超时
    防止 wrapper 永久卡住。超时后线程仍可能在后台运行，但进程退出时会被
    系统清理（守护线程 + OS 释放 USB）。
    """
    box = {}

    def worker():
        try:
            box["value"] = fn()
            box["ok"] = True
        except BaseException as exc:  # noqa: BLE001 - 全部捕获并传递
            box["ok"] = False
            box["error"] = exc

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    thread.join(timeout_s)
    if thread.is_alive():
        raise TimeoutError(
            f"操作在 {timeout_s:.1f}s 内未完成（请检查飞机电源和无线连接）"
        )
    if not box.get("ok"):
        raise box["error"]
    return box["value"]


def _do_arm(args, crtp, SyncCrazyflie, LogConfig) -> int:
    crtp.init_drivers()
    print(f"连接 Crazyflie：{args.uri}", flush=True)
    with SyncCrazyflie(args.uri) as scf:
        return send_and_confirm(scf.cf, args.arm, args.timeout, LogConfig)


def main() -> int:
    args = build_parser().parse_args()
    try:
        vehicle = load_vehicle_entry(args.config, cf_id=args.cf_id)
    except (OSError, TypeError, ValueError) as exc:
        print(f"错误：读取飞机参数失败：{exc}", file=sys.stderr)
        return 1
    if args.uri is None:
        args.uri = str(vehicle["uri"])
    if args.rigid_body is None:
        args.rigid_body = str(vehicle.get("rigid_body", "cf%d" % int(vehicle["id"])))
    crtp, Crazyflie, SyncCrazyflie, LogConfig = import_cflib()

    mocap = None
    if args.require_mocap:
        mocap = NokovPoseCheck(args.server_ip, args.rigid_body, args.mocap_timeout)
        try:
            mocap.connect()
            x, y, z = mocap.require_pose()
            print(f"NOKOV 位姿有效：pos=({x:.3f},{y:.3f},{z:.3f})m", flush=True)
        finally:
            mocap.close()

    try:
        _run_with_deadline(
            lambda: _do_arm(args, crtp, SyncCrazyflie, LogConfig),
            args.total_timeout,
        )
    except BaseException as exc:
        # cflib 的异常消息里可能带完整 traceback 文本，只打印第一行。
        message = str(exc).splitlines()[0] if str(exc) else str(exc)
        print(f"错误：{message}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
