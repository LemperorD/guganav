#!/usr/bin/env bash
set -euo pipefail

# ────────────────────────────────────────────────────────────────
# 一键启动行为树可视化：假裁判 + 决策节点 + Web UI 三个进程
#
# 用法：
#   scripts/btview.sh                  # rmul_decision，端口 8080，tick 1 Hz
#   scripts/btview.sh --simulation     # 接入已运行的 Gazebo/Nav2
#   scripts/btview.sh --simulation --namespace red_standard_robot1
#   scripts/btview.sh --decision guga_rmul_strategy
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
DECISION=rmul_decision
SIMULATION=0
SIM_NAMESPACE=red_standard_robot1
FAKE_REF_NAMESPACE=/fake_ref
LOG_DIR="${WS}/log/btview"
BTLOG=/tmp/bt_trace.btlog

usage() { sed -n '/^# 用法/,/^# Ctrl-C/p' "$0"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -port)       PORT=${2:-}; shift 2 ;;
    -hz)         TICK_HZ=${2:-}; shift 2 ;;
    -decision)   DECISION=${2:-}; shift 2 ;;
    -simulation) SIMULATION=1; shift ;;
    -sim)        SIMULATION=1; shift ;;
    -namespace)  SIM_NAMESPACE=${2:-}; shift 2 ;;
    -no-browser) OPEN_BROWSER=0; shift ;;
    -log)        BTLOG=${2:-}; shift 2 ;;
    -keep-log)   KEEP_LOG=1; shift ;;
    -h|-help)    usage; exit 0 ;;
    *)            echo "未知参数: $1" >&2; usage; exit 1 ;;
  esac
done

if ! [[ "$PORT" =~ ^[0-9]+$ ]]; then
  echo "ERROR: --port 需要是端口号" >&2; exit 1
fi
if ! [[ "$TICK_HZ" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: --hz 需要是数字" >&2; exit 1
fi
case "$DECISION" in
  rmul_decision|guga_rmul_strategy) ;;
  *)
    echo "ERROR: --decision 只支持 rmul_decision 或 guga_rmul_strategy" >&2
    exit 1
    ;;
esac

if [[ "$SIMULATION" -eq 1 ]]; then
  SIM_NAMESPACE=${SIM_NAMESPACE#/}
  SIM_NAMESPACE=${SIM_NAMESPACE%/}
  if [ -z "$SIM_NAMESPACE" ]; then
    echo "ERROR: --namespace 不能为空" >&2
    exit 1
  fi
  NAV_NAMESPACE="/$SIM_NAMESPACE"
  SOURCE_NODE="$FAKE_REF_NAMESPACE/fake_msg_source"
else
  SOURCE_NODE="/fake_msg_source"
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

SOURCE_BIN="$WS/install/fake_msg_source/lib/fake_msg_source/fake_msg_source_node"
case "$DECISION" in
  rmul_decision)
    DECISION_BIN="$WS/install/rmul_decision/lib/rmul_decision/rmul_decision"
    ;;
  guga_rmul_strategy)
    DECISION_BIN="$WS/install/guga_rmul_strategy/lib/guga_rmul_strategy/rmul_strategy_node"
    ;;
esac
SERVER_PY="$WS/scripts/btview/btview_server.py"
PAGE="$WS/scripts/btview/btview.html"

for f in "$SOURCE_BIN" "$DECISION_BIN" "$SERVER_PY" "$PAGE"; do
  if [ ! -e "$f" ]; then
    echo "ERROR: 缺少 $f" >&2
    echo "      先构建：./scripts/colconBuild.sh --packages-select fake_msg_source $DECISION" >&2
    exit 1
  fi
done

# 上一次 btview 如果没有正常退出，Web UI 会继续占着端口。这里只自动结束
# 明确属于本仓库 btview 的旧 server；其他程序占用时提示换端口，避免误杀。
ensure_port_available() {
  local -a pids=()
  local pid cmd
  mapfile -t pids < <(fuser -n tcp "$PORT" 2>/dev/null | tr ' ' '\n' | sed '/^$/d')
  [ "${#pids[@]}" -eq 0 ] && return 0

  local found_stale=0
  for pid in "${pids[@]}"; do
    [ -r "/proc/$pid/cmdline" ] || continue
    cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline")
    if [[ "$cmd" == *"$SERVER_PY"* ]]; then
      echo "检测到上次遗留的 btview Web UI（PID $pid），正在结束…"
      kill "$pid" 2>/dev/null || true
      found_stale=1
    fi
  done

  if [ "$found_stale" -eq 1 ]; then
    for _ in $(seq 1 30); do
      fuser -n tcp "$PORT" >/dev/null 2>&1 || return 0
      sleep 0.1
    done
  fi

  echo "ERROR: 端口 $PORT 已被其他程序占用。" >&2
  ss -ltnp "sport = :$PORT" 2>/dev/null >&2 || true
  echo "      可换端口启动：$0 --port $((PORT + 1))" >&2
  return 1
}

ensure_port_available || exit 1

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
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# 启动一个进程，稍后确认它还活着。崩了就把日志末尾打出来并退出，
# 而不是继续往下打印页面地址装作一切正常。
# 决策节点最容易这样死：树里用了没注册的节点名时，BT.CPP 在加载期就终止进程。
start_checked() {
  local label=$1 log=$2
  shift 2
  "$@" >"$log" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  sleep 1
  if ! kill -0 "$pid" 2>/dev/null; then
    {
      echo
      echo "ERROR: $label 启动后立即退出。$log 末尾："
      echo "────────────"
      tail -n 15 "$log"
      echo "────────────"
    } >&2
    exit 1
  fi
}

echo "启动数据源…"
if [[ "$SIMULATION" -eq 1 ]]; then
  echo "仿真模式：接入已运行的 Gazebo/Nav2（$NAV_NAMESPACE），不发布假位姿。"
  start_checked "假裁判数据源" "$LOG_DIR/source.log" \
    "$SOURCE_BIN" --ros-args \
    -r __ns:="$FAKE_REF_NAMESPACE" \
    -p use_sim_time:=true \
    -p publish_pose:=false \
    -p publish_map_to_odom:=false \
    -p vision_topic:="$NAV_NAMESPACE/vision/info"
else
  start_checked "数据源" "$LOG_DIR/source.log" "$SOURCE_BIN"
fi

echo "启动决策节点（tick ${TICK_HZ} Hz，记录写入 $BTLOG）…"
decision_args=(--ros-args)
if [[ "$SIMULATION" -eq 1 ]]; then
  decision_args+=(-r __ns:="$NAV_NAMESPACE")
  decision_args+=(-p use_sim_time:=true)
fi
decision_args+=(-p tick_rate_hz:="$TICK_HZ" -p btlog_path:="$BTLOG")
start_checked "决策节点" "$LOG_DIR/decision.log" \
  "$DECISION_BIN" "${decision_args[@]}"

echo "启动 Web UI（端口 $PORT）…"
start_checked "Web UI" "$LOG_DIR/server.log" \
  python3 "$SERVER_PY" --log "$BTLOG" --port "$PORT" --source "$SOURCE_NODE"

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
echo "  数据源日志: $LOG_DIR/source.log"
echo "  决策节点  : $DECISION"
echo "  决策日志  : $LOG_DIR/decision.log"
echo "  服务日志  : $LOG_DIR/server.log"
if [[ "$SIMULATION" -eq 1 ]]; then
  echo "  仿真命名空间: $NAV_NAMESPACE"
  echo "  假裁判调参: scripts/tune_fake_source.sh menu $SOURCE_NODE"
else
  echo "  假裁判调参: scripts/tune_fake_source.sh"
fi
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
