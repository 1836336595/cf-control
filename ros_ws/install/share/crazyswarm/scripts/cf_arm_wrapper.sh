#!/bin/bash
# launch-prefix wrapper for crazyswarm_server in hover_swarm.launch.
#
# 1) 先通过 cf_arm.py 对无刷 Crazyflie 发送 ARM（cflib 直连），ARM 成功后才
#    启动真正的节点命令（"$@" = crazyswarm_server 及其参数）；
#    ARM 失败则拒绝启动 server（安全）。
# 2) 把 SIGINT/SIGTERM 转发给子进程，保证 roslaunch Ctrl-C 时 server 正常退出。
# 3) server 退出（释放无线连接）后，best-effort 发送 DISARM。
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CF_ARM="$DIR/cf_arm.py"
CF_CONFIG="$DIR/../launch/crazyflies.yaml"
PYTHON="${PYTHON:-python3}"
export CFLIB_PATH="${CFLIB_PATH:-/home/nan/crazyflie-lib-python}"

if [ ! -f "$CF_CONFIG" ] && command -v rospack >/dev/null 2>&1; then
  PACKAGE_DIR="$(rospack find crazyswarm 2>/dev/null || true)"
  if [ -n "$PACKAGE_DIR" ]; then
    CF_CONFIG="$PACKAGE_DIR/launch/crazyflies.yaml"
  fi
fi

if ! "$PYTHON" "$CF_ARM" --config "$CF_CONFIG" --arm; then
  echo "[cf_arm_wrapper] ARM 失败，不启动 crazyswarm_server（请检查飞机电源和无线连接）" >&2
  exit 1
fi

"$@" &
CHILD=$!
trap 'kill -TERM "$CHILD" 2>/dev/null' INT TERM
wait "$CHILD"
RET=$?
trap - INT TERM

"$PYTHON" "$CF_ARM" --config "$CF_CONFIG" --disarm || true
exit $RET
