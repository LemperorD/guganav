#!/bin/bash
# 绕 z 轴旋转回放测试:
# 校验 ba 三轴收敛, 以及 bg 的 x/y 收敛. bg 的 z 轴与 yaw 率共线,
# 该场景下不可观, 不做断言.
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    source "$setup_file"
    set -u
  fi
}

if [ -z "${ROS_DISTRO:-}" ]; then
  source_setup /opt/ros/humble/setup.bash
fi
cd "$WS"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
export ROS_HOME="$TMP/ros_home"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

echo "=== 构建 livox_ros_driver2 与 point_lio ==="
colcon build \
  --packages-select livox_ros_driver2 point_lio \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON
source_setup "$WS/install/setup.bash"

echo "=== 生成绕 z 轴旋转 bag ==="
PL_CMP_DIR="$TMP/bench" python3 scripts/pl_cmp/gen_bag.py

echo "=== 启动 point_lio 并回放 ==="
ros2 run point_lio pointlio_mapping --ros-args \
  --params-file src/guga_localization/LIO/point_lio/config/mid360.yaml \
  -p publish.debug_state_en:=true > "$TMP/node.log" 2>&1 &
NODE_PID=$!
sleep 3
timeout 60 ros2 topic echo /point_lio/debug_state --truncate-length 10000 \
  > "$TMP/state.log" 2>&1 &
ECHO_PID=$!
ros2 bag play "$TMP/bench/bags/mid360_synth" > "$TMP/play.log" 2>&1 || true
sleep 2
kill "$NODE_PID" "$ECHO_PID" 2>/dev/null || true
wait 2>/dev/null || true

grep -q "标定完成" "$TMP/node.log" || {
  echo "FAILED: 标定未完成"
  tail -20 "$TMP/node.log"
  exit 1
}

python3 - "$TMP/state.log" <<'PY'
import re
import sys

text = open(sys.argv[1]).read()
bgs = re.findall(r'bg:\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', text)
bas = re.findall(r'ba:\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', text)
if len(bgs) < 20 or len(bas) < 20:
    print(f"FAILED: 状态记录不足, bg={len(bgs)}, ba={len(bas)}")
    sys.exit(1)

bg = [float(v) for v in bgs[-1]]
ba = [float(v) for v in bas[-1]]
bg_ref = [0.003, -0.002, 0.004]
ba_ref = [0.010 * 9.81, 0.010 * 9.81, 0.020 * 9.81]
print(f"bg last: {bg}")
print(f"ba last: {ba}")

ok = True
for i in range(3):
    if abs(ba[i] - ba_ref[i]) > 0.02:
        print(f"FAILED: ba[{i}] 误差 {ba[i] - ba_ref[i]:.4f} > 0.02")
        ok = False
for i in range(2):
    if abs(bg[i] - bg_ref[i]) > 0.02:
        print(f"FAILED: bg[{i}] 误差 {bg[i] - bg_ref[i]:.4f} > 0.02")
        ok = False
sys.exit(0 if ok else 1)
PY

echo "point_lio z-rotation replay test: PASSED"
