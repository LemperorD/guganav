#!/usr/bin/env bash
set -euo pipefail

# ────────────────────────────────────────────────────────────────
# 一键启动行为树可视化：假裁判 + 决策节点 + Web UI 三个进程
#
# 用法：
#   scripts/btview.sh                  # 端口 8080，tick 1 Hz，自动开浏览器
#   scripts/btview.sh --port 8090
#   scripts/btview.sh --hz 10          # 行为树 tick 频率
#   scripts/btview.sh --no-browser
#   scripts/btview.sh --keep-log       # 日志留在文件里，不删旧的
#
# Ctrl-C 会一并停掉三个进程。
#
# 直接跑 install 里的可执行文件，而不是 ros2 run：ros2 run 是 Python 包装，
# kill 它不会把真正的节点带走，很容易留下杀不干净的残留进程。
# ────────────────────────────────────────────────────────────────

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)

PORT=8080
TICK_HZ=1.0
OPEN_BROWSER=1
LOG_DIR="${WS}/log/btview"
BTLOG=/tmp/bt_trace.btlog

usage() { sed -n '4,18p' "$0"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)       PORT=${2:-}; shift 2 ;;
    --hz)         TICK_HZ=${2:-}; shift 2 ;;
    --no-browser) OPEN_BROWSER=0; shift ;;
    --log)        BTLOG=${2:-}; shift 2 ;;
    --keep-log)   KEEP_LOG=1; shift ;;
    -h|--help)    usage; exit 0 ;;
    *)            echo "未知参数: $1" >&2; usage; exit 1 ;;
  esac
done

if ! [[ "$PORT" =~ ^[0-9]+$ ]]; then
  echo "ERROR: --port 需要是端口号" >&2; exit 1
fi
if ! [[ "$TICK_HZ" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: --hz 需要是数字" >&2; exit 1
fi
# ROS 2 按字面量推断参数类型：写 2 会被当成整数，与声明为 double 的
# tick_rate_hz 冲突，抛 InvalidParameterTypeException。这里补上小数位。
case "$TICK_HZ" in
  *.*) ;;
  *) TICK_HZ="${TICK_HZ}.0" ;;
esac

source_setup() {
  local f=$1
  if [ -f "$f" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$f"
    set -u
  fi
}
[[ -n "${ROS_DISTRO:-}" ]] || source_setup /opt/ros/humble/setup.bash
source_setup "$WS/install/setup.bash"

REFEREE_BIN="$WS/install/fake_referee/lib/fake_referee/fake_referee_node"
STRATEGY_BIN="$WS/install/guga_rmul_strategy/lib/guga_rmul_strategy/rmul_strategy_node"
SERVER_PY="$WS/scripts/btview_server.py"
PAGE="$WS/scripts/btview.html"

for f in "$REFEREE_BIN" "$STRATEGY_BIN" "$SERVER_PY" "$PAGE"; do
  if [ ! -e "$f" ]; then
    echo "ERROR: 缺少 $f" >&2
    echo "      先构建：./scripts/colconBuild.sh --packages-select fake_referee guga_rmul_strategy" >&2
    exit 1
  fi
done

mkdir -p "$LOG_DIR"
[[ -n "${KEEP_LOG:-}" ]] || rm -f "$LOG_DIR"/*.log
rm -f "$BTLOG"

export ROS_LOG_DIR="${LOG_DIR}/ros"
mkdir -p "$ROS_LOG_DIR"

PIDS=()
cleanup() {
  echo
  echo "正在停止…"
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  echo "已停止。日志在 $LOG_DIR"
}
trap cleanup EXIT INT TERM

echo "启动假裁判…"
"$REFEREE_BIN" >"$LOG_DIR/referee.log" 2>&1 &
PIDS+=($!)

sleep 1

echo "启动决策节点（tick ${TICK_HZ} Hz，记录写入 $BTLOG）…"
"$STRATEGY_BIN" --ros-args -p tick_rate_hz:="$TICK_HZ" -p btlog_path:="$BTLOG" \
  >"$LOG_DIR/strategy.log" 2>&1 &
PIDS+=($!)

sleep 1

echo "启动 Web UI（端口 $PORT）…"
python3 "$SERVER_PY" --log "$BTLOG" --port "$PORT" >"$LOG_DIR/server.log" 2>&1 &
PIDS+=($!)

# 等页面能访问了再开浏览器，免得打开是空白的
for _ in $(seq 1 30); do
  if curl -fsS -o /dev/null "http://localhost:${PORT}/" 2>/dev/null; then
    break
  fi
  sleep 0.2
done

URL="http://localhost:${PORT}"
echo
echo "  可视化页面: $URL"
echo "  假裁判日志: $LOG_DIR/referee.log"
echo "  决策日志  : $LOG_DIR/strategy.log"
echo "  服务日志  : $LOG_DIR/server.log"
echo "  按 Ctrl-C 停止全部"
echo

if [[ "$OPEN_BROWSER" -eq 1 ]]; then
  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$URL" >/dev/null 2>&1 || true
  else
    echo "（没有 xdg-open，请手动打开 $URL）"
  fi
fi

wait
