#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)

DURATION=30
WINDOW=10000
OUTPUT_ROOT="$WS/rosbag/topic_hz"
NAMESPACE=""
USE_SIM_TIME=0
WALL_TIME=1
CUSTOM_TOPICS=()

usage() {
  cat <<'EOF'
用法:
  scripts/perf/measure_hot_topic_hz.sh [options] [topic ...]

选项:
  --duration SEC        测量时长，单位秒。默认: 30
  --window N            ros2 topic hz 统计窗口。默认: 10000
  --output-dir DIR      输出根目录。默认: rosbag/topic_hz
  --simulation          仿真模式，默认给 topic 添加 /red_standard_robot1 命名空间
  --namespace NS        给默认 topic 添加命名空间，例如 /red_standard_robot1
  --use-sim-time        使用 ROS 仿真时间
  --ros-time            使用 ROS 时间统计频率。默认使用 wall time
  --wall-time           使用 wall time 统计频率。默认开启
  -h, --help            显示本帮助

不传 topic 时，默认测量导航热点 topic：
  /cmd_vel_controller /cmd_vel_nav2_result /cmd_vel
  /odometry /goal_pose /plan /local_plan
  /terrain_map /terrain_map_ext
  /local_costmap/costmap_raw /global_costmap/costmap_raw
  /cloud_registered /lidar_odometry /livox/lidar /velodyne_points

本脚本只订阅 topic 并统计接收频率，不启动导航、不发布消息。
EOF
}

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    source "$setup_file"
    set -u
  fi
}

normalize_namespace() {
  local ns=$1
  if [ -z "$ns" ]; then
    printf ''
    return
  fi
  ns="/${ns#/}"
  ns="${ns%/}"
  printf '%s' "$ns"
}

with_namespace() {
  local topic=$1
  if [ -z "$NAMESPACE" ]; then
    printf '%s' "$topic"
    return
  fi
  case "$topic" in
    /tf | /tf_static | /rosout)
      printf '%s' "$topic"
      ;;
    /*)
      printf '%s%s' "$NAMESPACE" "$topic"
      ;;
    *)
      printf '%s/%s' "$NAMESPACE" "$topic"
      ;;
  esac
}

topic_to_file_name() {
  local topic=$1
  local name
  name=$(printf '%s' "$topic" | sed 's#^/##; s#[^A-Za-z0-9_.-]#_#g')
  if [ -z "$name" ]; then
    name=root
  fi
  printf '%s' "$name"
}

while [ $# -gt 0 ]; do
  case "$1" in
    --duration)
      DURATION=$2
      shift 2
      ;;
    --duration=*)
      DURATION=${1#*=}
      shift
      ;;
    --window)
      WINDOW=$2
      shift 2
      ;;
    --window=*)
      WINDOW=${1#*=}
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
    --simulation)
      NAMESPACE="/red_standard_robot1"
      shift
      ;;
    --namespace)
      NAMESPACE=$(normalize_namespace "$2")
      shift 2
      ;;
    --namespace=*)
      NAMESPACE=$(normalize_namespace "${1#*=}")
      shift
      ;;
    --use-sim-time)
      USE_SIM_TIME=1
      shift
      ;;
    --ros-time)
      WALL_TIME=0
      shift
      ;;
    --wall-time)
      WALL_TIME=1
      shift
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    --)
      shift
      while [ $# -gt 0 ]; do
        CUSTOM_TOPICS+=("$1")
        shift
      done
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      CUSTOM_TOPICS+=("$1")
      shift
      ;;
  esac
done

if ! [[ "$DURATION" =~ ^[1-9][0-9]*$ ]]; then
  echo "--duration must be a positive integer" >&2
  exit 2
fi

if ! [[ "$WINDOW" =~ ^[1-9][0-9]*$ ]]; then
  echo "--window must be a positive integer" >&2
  exit 2
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  source_setup /opt/ros/humble/setup.bash
fi
source_setup "$HOME/nav2_ws/install/setup.bash"
source_setup "$WS/install/setup.bash"

cd "$WS"

DEFAULT_TOPICS=(
  /cmd_vel_controller
  /cmd_vel_nav2_result
  /cmd_vel
  /odometry
  /goal_pose
  /plan
  /local_plan
  /terrain_map
  /terrain_map_ext
  /local_costmap/costmap_raw
  /global_costmap/costmap_raw
  /cloud_registered
  /lidar_odometry
  /livox/lidar
  /velodyne_points
)

if [ "${#CUSTOM_TOPICS[@]}" -gt 0 ]; then
  TOPICS=("${CUSTOM_TOPICS[@]}")
else
  TOPICS=("${DEFAULT_TOPICS[@]}")
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RUN_DIR="$OUTPUT_ROOT/${TIMESTAMP}_topic_hz"
mkdir -p "$RUN_DIR"

BRANCH=$(git branch --show-current 2>/dev/null || true)
COMMIT=$(git rev-parse HEAD 2>/dev/null || true)

cat > "$RUN_DIR/metadata.yaml" <<EOF
experiment: hot_topic_hz
started_at: "$(date --iso-8601=seconds)"
workspace: "$WS"
branch: "$BRANCH"
commit: "$COMMIT"
duration_sec: $DURATION
window: $WINDOW
namespace: "$NAMESPACE"
time_source: "$([ "$WALL_TIME" -eq 1 ] && echo wall || echo ros)"
use_sim_time: "$([ "$USE_SIM_TIME" -eq 1 ] && echo true || echo false)"
EOF

PIDS=()
FILES=()
RESOLVED_TOPICS=()

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup INT TERM EXIT

echo "=== Hot topic frequency measurement ==="
echo "run dir:  $RUN_DIR"
echo "duration: ${DURATION}s"
echo "window:   $WINDOW"
echo "namespace:${NAMESPACE:- <none>}"
echo

for topic in "${TOPICS[@]}"; do
  resolved_topic=$(with_namespace "$topic")
  file="$RUN_DIR/$(topic_to_file_name "$resolved_topic").hz.txt"
  cmd=(ros2 topic hz "$resolved_topic" --window "$WINDOW" --spin-time 2)
  if [ "$WALL_TIME" -eq 1 ]; then
    cmd+=(--wall-time)
  fi
  if [ "$USE_SIM_TIME" -eq 1 ]; then
    cmd+=(--use-sim-time)
  fi

  printf '%q ' "${cmd[@]}" > "$file"
  printf '\n\n' >> "$file"
  "${cmd[@]}" >> "$file" 2>&1 &
  PIDS+=("$!")
  FILES+=("$file")
  RESOLVED_TOPICS+=("$resolved_topic")
  echo "measuring: $resolved_topic"
done

sleep "$DURATION"
cleanup
trap - INT TERM EXIT

for pid in "${PIDS[@]}"; do
  wait "$pid" 2>/dev/null || true
done

SUMMARY="$RUN_DIR/summary.txt"
SUMMARY_MD="$RUN_DIR/summary.md"
{
  echo "topic hz summary"
  echo "duration_sec: $DURATION"
  echo
  for i in "${!FILES[@]}"; do
    topic=${RESOLVED_TOPICS[$i]}
    file=${FILES[$i]}
    avg=$(grep -E 'average rate:' "$file" | tail -1 | awk '{print $3}')
    min=$(grep -E 'min:' "$file" | tail -1 | sed -E 's/.*min: ([^ ]+).*/\1/')
    max=$(grep -E 'max:' "$file" | tail -1 | sed -E 's/.*max: ([^ ]+).*/\1/')
    stddev=$(grep -E 'std dev:' "$file" | tail -1 | sed -E 's/.*std dev: ([^ ]+).*/\1/')
    window=$(grep -E 'window:' "$file" | tail -1 | sed -E 's/.*window: ([^ ]+).*/\1/')
    if [ -z "${avg:-}" ]; then
      avg="no_data"
    fi
    printf '%-48s avg_hz=%-10s min_delta=%-10s max_delta=%-10s stddev=%-10s window=%s\n' \
      "$topic" "$avg" "${min:-n/a}" "${max:-n/a}" "${stddev:-n/a}" "${window:-n/a}"
  done
} | tee "$SUMMARY"

{
  echo "# Topic 频率测量"
  echo
  echo "| 项目 | 值 |"
  echo "| --- | --- |"
  echo "| duration_sec | $DURATION |"
  echo "| window | $WINDOW |"
  echo "| namespace | ${NAMESPACE:-无} |"
  echo "| time_source | $([ "$WALL_TIME" -eq 1 ] && echo wall || echo ros) |"
  echo "| use_sim_time | $([ "$USE_SIM_TIME" -eq 1 ] && echo true || echo false) |"
  echo
  echo "| Topic | 平均频率(Hz) | 最小间隔(s) | 最大间隔(s) | 标准差(s) | Window |"
  echo "| --- | ---: | ---: | ---: | ---: | ---: |"
  for i in "${!FILES[@]}"; do
    topic=${RESOLVED_TOPICS[$i]}
    file=${FILES[$i]}
    avg=$(grep -E 'average rate:' "$file" | tail -1 | awk '{print $3}')
    min=$(grep -E 'min:' "$file" | tail -1 | sed -E 's/.*min: ([^ ]+).*/\1/')
    max=$(grep -E 'max:' "$file" | tail -1 | sed -E 's/.*max: ([^ ]+).*/\1/')
    stddev=$(grep -E 'std dev:' "$file" | tail -1 | sed -E 's/.*std dev: ([^ ]+).*/\1/')
    window=$(grep -E 'window:' "$file" | tail -1 | sed -E 's/.*window: ([^ ]+).*/\1/')
    if [ -z "${avg:-}" ]; then
      avg="no_data"
    fi
    printf '| `%s` | %s | %s | %s | %s | %s |\n' \
      "$topic" "$avg" "${min:-n/a}" "${max:-n/a}" "${stddev:-n/a}" "${window:-n/a}"
  done
} > "$SUMMARY_MD"

echo
echo "summary: $SUMMARY"
echo "summary md: $SUMMARY_MD"
