#!/usr/bin/env python3
"""Takeoff-hover-land for the swarm (brushless Crazyflie 2.1).

先启动 hover_swarm.launch —— 它会在 crazyswarm_server 连接飞机之前自动
发送 ARM 指令（cf_arm_wrapper.sh / cf_arm.py），server 退出时自动 DISARM。

用法：:

    source devel/setup.bash
    roslaunch crazyswarm hover_swarm.launch
    # 另开一个终端：
    python3 scripts/hover_swarm.py --height 0.8 --hold 5.0
"""

import argparse
import os

from pycrazyswarm import Crazyswarm

# pycrazyswarm 默认用相对路径 ../launch/crazyflies.yaml（相对当前工作目录），
# 这里改为相对本脚本的绝对路径，保证从任意目录运行都能找到。
CRAZYFLIES_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "launch", "crazyflies.yaml"
)


def main():
    parser = argparse.ArgumentParser(
        description="使用 crazyswarm 执行单机悬停任务（起飞-悬停-降落）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--height", type=float, default=0.8, help="悬停高度 m（相对起飞点）")
    parser.add_argument("--hold", type=float, default=5.0, help="悬停时间 s")
    parser.add_argument("--takeoff-duration", type=float, default=2.5, help="起飞轨迹时间 s")
    parser.add_argument("--land-duration", type=float, default=2.5, help="降落轨迹时间 s")
    parser.add_argument("--land-height", type=float, default=0.04, help="降落目标高度 m")
    parser.add_argument(
        "--cf-index", type=int, default=0,
        help="使用 allcfs.crazyflies 中的第几架飞机（下标）",
    )
    args = parser.parse_args()

    if args.height <= 0:
        raise SystemExit("--height 必须大于 0")
    if args.hold < 0:
        raise SystemExit("--hold 不能小于 0")
    if not (0.0 <= args.land_height <= 0.2):
        raise SystemExit("--land-height 必须在 0 到 0.2 m 之间")

    swarm = Crazyswarm(crazyflies_yaml=CRAZYFLIES_YAML)
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[args.cf_index]

    print(f"使用第 {args.cf_index} 架飞机（{cf.prefix}）执行悬停任务")
    print(f"起飞到 {args.height:.2f} m（{args.takeoff_duration:.1f} s）……")
    cf.takeoff(targetHeight=args.height, duration=args.takeoff_duration)
    timeHelper.sleep(args.takeoff_duration + 1.0)

    print(f"悬停 {args.hold:.1f} s……")
    timeHelper.sleep(args.hold)

    print(f"降落到 {args.land_height:.2f} m（{args.land_duration:.1f} s）……")
    cf.land(targetHeight=args.land_height, duration=args.land_duration)
    timeHelper.sleep(args.land_duration + 1.0)

    print("悬停任务完成")


if __name__ == "__main__":
    main()
