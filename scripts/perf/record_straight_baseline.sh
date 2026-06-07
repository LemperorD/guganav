#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)

SCENE="straight_v0"
DURATION=0
OUTPUT_ROOT="$WS/rosbag/baseline"
STORAGE_ID="mcap"
DIAGNOSTIC=0
NOTES=""
NAMESPACE=""

usage() {
  cat <<'EOF'
用法:
  scripts/perf/record_straight_baseline.sh [options]

选项:
  --scene NAME          场景名称。默认: straight_v0
  --duration SEC        录制时长。默认: 0，持续录制直到 Ctrl-C
  --output-dir DIR      输出根目录。默认: rosbag/baseline
  --storage ID          rosbag2 存储后端。默认: mcap
  --simulation          仿真模式，默认给 topic 添加 /red_standard_robot1 命名空间
  --namespace NS        给默认 topic 添加命名空间，例如 /red_standard_robot1
  --diagnostic          同时录制点云、terrain 和 costmap 诊断 topic
  --notes TEXT          写入 metadata.yaml 的备注
  -h, --help            显示本帮助

本脚本只记录基线数据，不启动导航、不发布 goal、不改变机器人行为。
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

while [ $# -gt 0 ]; do
  case "$1" in
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
    --storage)
      STORAGE_ID=$2
      shift 2
      ;;
    --storage=*)
      STORAGE_ID=${1#*=}
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
    --diagnostic)
      DIAGNOSTIC=1
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

if ! [[ "$DURATION" =~ ^[0-9]+$ ]]; then
  echo "--duration must be a non-negative integer" >&2
  exit 2
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  source_setup /opt/ros/humble/setup.bash
fi
source_setup "$HOME/nav2_ws/install/setup.bash"
source_setup "$WS/install/setup.bash"

cd "$WS"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SCENE_SAFE=$(printf '%s' "$SCENE" | tr -c '[:alnum:]_.-' '_')
RUN_DIR="$OUTPUT_ROOT/${TIMESTAMP}_${SCENE_SAFE}"
BAG_DIR="$RUN_DIR/nav"
mkdir -p "$RUN_DIR"

TOPICS=(
  /tf
  /tf_static
  /odometry
  /cmd_vel_controller
  /cmd_vel_nav2_result
  /cmd_vel
  /cmd_spin
  /chassis_mode
  /goal_pose
  /plan
  /local_plan
  /rosout
)

if [ "$DIAGNOSTIC" -eq 1 ]; then
  TOPICS+=(
    /terrain_map
    /terrain_map_ext
    /local_costmap/costmap_raw
    /global_costmap/costmap_raw
    /local_costmap/published_footprint
    /global_costmap/published_footprint
    /cloud_registered
    /lidar_odometry
    /livox/lidar
    /velodyne_points
  )
fi

RESOLVED_TOPICS=()
for topic in "${TOPICS[@]}"; do
  RESOLVED_TOPICS+=("$(with_namespace "$topic")")
done

printf '%s\n' "${RESOLVED_TOPICS[@]}" > "$RUN_DIR/topics.txt"

BRANCH=$(git branch --show-current 2>/dev/null || true)
COMMIT=$(git rev-parse HEAD 2>/dev/null || true)
DIRTY=false
if ! git diff --quiet --ignore-submodules -- 2>/dev/null \
  || ! git diff --cached --quiet --ignore-submodules -- 2>/dev/null; then
  DIRTY=true
fi

cat > "$RUN_DIR/metadata.yaml" <<EOF
experiment: straight_baseline_v0
scene: "$SCENE"
mode: "$([ "$DIAGNOSTIC" -eq 1 ] && echo diagnostic || echo behavior)"
started_at: "$(date --iso-8601=seconds)"
workspace: "$WS"
branch: "$BRANCH"
commit: "$COMMIT"
dirty: $DIRTY
storage_id: "$STORAGE_ID"
duration_sec: $DURATION
namespace: "$NAMESPACE"
bag_dir: "$BAG_DIR"
notes: "$NOTES"
launch_command: "fill manually"
params_file: "fill manually"
map: "fill manually"
prior_pcd: "fill manually"
start_pose: "fill manually"
goal_pose: "fill manually"
operator: "fill manually"
result: "fill manually"
EOF

echo "=== Straight baseline recording ==="
echo "run dir:  $RUN_DIR"
echo "bag dir:  $BAG_DIR"
echo "storage:  $STORAGE_ID"
echo "mode:     $([ "$DIAGNOSTIC" -eq 1 ] && echo diagnostic || echo behavior)"
echo "namespace:${NAMESPACE:- <none>}"
echo "duration: $([ "$DURATION" -eq 0 ] && echo 'until Ctrl-C' || echo "${DURATION}s")"
echo
echo "Start/keep navigation running, send the fixed straight-line goal, then let this recorder run."
echo "This script does not publish goals or change robot behavior."
echo

RECORD_CMD=(ros2 bag record -s "$STORAGE_ID" -o "$BAG_DIR" "${RESOLVED_TOPICS[@]}")
printf '%q ' "${RECORD_CMD[@]}" > "$RUN_DIR/record_command.txt"
printf '\n' >> "$RUN_DIR/record_command.txt"

set +e
if [ "$DURATION" -eq 0 ]; then
  "${RECORD_CMD[@]}"
  STATUS=$?
else
  timeout --foreground --signal=INT "$DURATION" "${RECORD_CMD[@]}"
  STATUS=$?
fi
set -e

if [ "$STATUS" -ne 0 ] && [ "$STATUS" -ne 124 ]; then
  echo "ros2 bag record failed with exit code $STATUS" >&2
  exit "$STATUS"
fi

echo
echo "=== Recording finished ==="
echo "metadata: $RUN_DIR/metadata.yaml"
echo "topics:   $RUN_DIR/topics.txt"
echo "bag:      $BAG_DIR"
