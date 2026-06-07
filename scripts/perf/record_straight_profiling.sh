#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)

SCENE="straight_profile_v0"
DURATION=30
OUTPUT_ROOT="$WS/rosbag/baseline"
PID=""
PROCESS_NAME=""
CALL_GRAPH="dwarf,8192"
PERF_EXTRA_ARGS=()
SYSTEM_SAMPLE=1
NOTES=""

usage() {
  cat <<'EOF'
用法:
  scripts/perf/record_straight_profiling.sh --pid PID [options]
  scripts/perf/record_straight_profiling.sh --process NAME [options]

选项:
  --pid PID             目标进程 id，例如 nav2_container 的 pid
  --process NAME        使用 pgrep -f NAME 匹配最新的目标进程
  --scene NAME          场景名称。默认: straight_profile_v0
  --duration SEC        profiling 时长，单位秒。默认: 30
  --output-dir DIR      输出根目录。默认: rosbag/baseline
  --call-graph MODE     perf 调用栈模式。默认: dwarf,8192
                        使用 RelWithDebInfo + -fno-omit-frame-pointer 后可改用 fp
  --event EVENT         追加 perf event，可重复传入
  --no-system-sample    即使 pidstat 可用，也不记录系统采样
  --notes TEXT          写入 metadata.yaml 的备注
  -h, --help            显示本帮助

本脚本只记录 profiling 数据。若需要干净的行为基线指标，不要和轻量行为
基线录包同时运行。
录制完成后会自动将 perf.data 权限设置为 777，便于 Hotspot 或其他用户打开。
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --pid)
      PID=$2
      shift 2
      ;;
    --pid=*)
      PID=${1#*=}
      shift
      ;;
    --process)
      PROCESS_NAME=$2
      shift 2
      ;;
    --process=*)
      PROCESS_NAME=${1#*=}
      shift
      ;;
    --scene)
      SCENE=$2
      shift 2
      ;;
    --scene=*)
      SCENE=${1#*=}
      shift
      ;;
    --duration)
      DURATION=$2
      shift 2
      ;;
    --duration=*)
      DURATION=${1#*=}
      shift
      ;;
    --output-dir)
      OUTPUT_ROOT=$2
      shift 2
      ;;
    --output-dir=*)
      OUTPUT_ROOT=${1#*=}
      shift
      ;;
    --call-graph)
      CALL_GRAPH=$2
      shift 2
      ;;
    --call-graph=*)
      CALL_GRAPH=${1#*=}
      shift
      ;;
    --event)
      PERF_EXTRA_ARGS+=(-e "$2")
      shift 2
      ;;
    --event=*)
      PERF_EXTRA_ARGS+=(-e "${1#*=}")
      shift
      ;;
    --no-system-sample)
      SYSTEM_SAMPLE=0
      shift
      ;;
    --notes)
      NOTES=$2
      shift 2
      ;;
    --notes=*)
      NOTES=${1#*=}
      shift
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$PID" ] && [ -z "$PROCESS_NAME" ]; then
  echo "Either --pid or --process is required" >&2
  usage >&2
  exit 2
fi

if [ -n "$PID" ] && [ -n "$PROCESS_NAME" ]; then
  echo "Use only one of --pid or --process" >&2
  exit 2
fi

if ! [[ "$DURATION" =~ ^[1-9][0-9]*$ ]]; then
  echo "--duration must be a positive integer" >&2
  exit 2
fi

if [ -n "$PROCESS_NAME" ]; then
  PID=$(pgrep -n -f "$PROCESS_NAME" || true)
  if [ -z "$PID" ]; then
    echo "No process matched: $PROCESS_NAME" >&2
    exit 1
  fi
fi

if ! [[ "$PID" =~ ^[0-9]+$ ]]; then
  echo "Invalid pid: $PID" >&2
  exit 2
fi

if ! kill -0 "$PID" 2>/dev/null; then
  echo "Process is not running: $PID" >&2
  exit 1
fi

if ! command -v perf >/dev/null 2>&1; then
  echo "perf is required but not found" >&2
  exit 1
fi

cd "$WS"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SCENE_SAFE=$(printf '%s' "$SCENE" | tr -c '[:alnum:]_.-' '_')
RUN_DIR="$OUTPUT_ROOT/${TIMESTAMP}_${SCENE_SAFE}"
mkdir -p "$RUN_DIR"

BRANCH=$(git branch --show-current 2>/dev/null || true)
COMMIT=$(git rev-parse HEAD 2>/dev/null || true)
DIRTY=false
if ! git diff --quiet --ignore-submodules -- 2>/dev/null \
  || ! git diff --cached --quiet --ignore-submodules -- 2>/dev/null; then
  DIRTY=true
fi

PERF_PARANOID=$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo unknown)
if [[ "$PERF_PARANOID" =~ ^-?[0-9]+$ ]] && [ "$PERF_PARANOID" -gt 1 ] \
  && [ "$(id -u)" -ne 0 ]; then
  echo "Warning: perf_event_paranoid=$PERF_PARANOID may block perf attach."
  echo "Temporary fix example: sudo sysctl kernel.perf_event_paranoid=1"
  echo
fi

cat > "$RUN_DIR/metadata.yaml" <<EOF
experiment: straight_profiling_v0
scene: "$SCENE"
mode: "profiling"
started_at: "$(date --iso-8601=seconds)"
workspace: "$WS"
branch: "$BRANCH"
commit: "$COMMIT"
dirty: $DIRTY
pid: $PID
process_name: "$PROCESS_NAME"
duration_sec: $DURATION
call_graph: "$CALL_GRAPH"
perf_event_paranoid: "$PERF_PARANOID"
perf_data: "$RUN_DIR/perf.data"
system_sample: "$([ "$SYSTEM_SAMPLE" -eq 1 ] && echo enabled || echo disabled)"
notes: "$NOTES"
launch_command: "fill manually"
params_file: "fill manually"
map: "fill manually"
prior_pcd: "fill manually"
operator: "fill manually"
result: "fill manually"
EOF

echo "=== Straight profiling recording ==="
echo "run dir:    $RUN_DIR"
echo "pid:        $PID"
echo "duration:   ${DURATION}s"
echo "call graph: $CALL_GRAPH"
echo
echo "Recommended build for Hotspot/perf:"
echo "  RelWithDebInfo with -O2 -g -fno-omit-frame-pointer"
echo

PIDSTAT_PID=""
if [ "$SYSTEM_SAMPLE" -eq 1 ] && command -v pidstat >/dev/null 2>&1; then
  pidstat -h -r -u -p "$PID" 1 "$DURATION" > "$RUN_DIR/system_pidstat.txt" &
  PIDSTAT_PID=$!
else
  echo "pidstat not available or disabled; skipping system sampling"
fi

PERF_CMD=(
  perf record
  -o "$RUN_DIR/perf.data"
  --call-graph "$CALL_GRAPH"
  "${PERF_EXTRA_ARGS[@]}"
  --pid "$PID"
)

printf '%q ' "${PERF_CMD[@]}" > "$RUN_DIR/perf_command.txt"
printf '\n' >> "$RUN_DIR/perf_command.txt"

set +e
timeout --foreground --signal=INT "$DURATION" "${PERF_CMD[@]}"
STATUS=$?
set -e

if [ -n "$PIDSTAT_PID" ]; then
  wait "$PIDSTAT_PID" 2>/dev/null || true
fi

if [ "$STATUS" -ne 0 ] && [ "$STATUS" -ne 124 ]; then
  echo "perf record failed with exit code $STATUS" >&2
  exit "$STATUS"
fi

if [ -f "$RUN_DIR/perf.data" ]; then
  chmod 777 "$RUN_DIR/perf.data"
fi

echo
echo "=== Profiling finished ==="
echo "metadata: $RUN_DIR/metadata.yaml"
echo "perf:     $RUN_DIR/perf.data"
if [ -f "$RUN_DIR/system_pidstat.txt" ]; then
  echo "system:   $RUN_DIR/system_pidstat.txt"
fi
echo
echo "Open perf.data with Hotspot or inspect with:"
echo "  perf report -i $RUN_DIR/perf.data"
