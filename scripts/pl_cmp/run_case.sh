#!/usr/bin/env bash
# 运行一个 point_lio 版本并把输出录成 bag, 同时记录 CPU 与峰值内存。
# 用法: run_case.sh <label> <setup.bash ...>
# 例:   run_case.sh refactored /home/rog/guganav/install/setup.bash
#
# 注意:
#   - 不能用 set -u, ROS 的 setup.bash 会读取未定义的 AMENT_TRACE_SETUP_FILES
#   - 不能用 /usr/bin/time 包装: 杀掉包装进程不会终止真正的节点, 残留节点会污染下一次录制
#   - 不能用 set -- 处理 /proc 字段, 会覆盖脚本自己的位置参数
set +u

LABEL="$1"
shift || true

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)
PL_DIR="${PL_CMP_DIR:-$WS/.pl_cmp_bench}"
BAG="$PL_DIR/bags/mid360_synth"
OUT="$PL_DIR/out/$LABEL"
mkdir -p "$OUT"
rm -rf "$OUT/bag" "$OUT"/*.log "$OUT"/*.txt

# 沙箱不允许写 ~/.ros, 否则 ros2 bag 会段错误
export ROS_HOME="$PL_DIR/roshome"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"

# 清掉任何残留的节点, 否则会混进本次录制
pkill -x pointlio_mapping 2>/dev/null
pkill -f "[m]onitor.py" 2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
for setup in "$@"; do
  # shellcheck disable=SC1090
  source "$setup"
done

PREFIX="$(ros2 pkg prefix point_lio)"
PARAMS="$PREFIX/share/point_lio/config/mid360.yaml"
EXE="$PREFIX/lib/point_lio/pointlio_mapping"
echo "[$LABEL] prefix=$PREFIX"
ls -l "$EXE" > "$OUT/exe.txt" 2>&1
sha256sum "$EXE" >> "$OUT/exe.txt" 2>&1

cpu_ticks() {   # $1 = pid -> "utime stime"
  awk '{ for (i = 1; i <= NF; i++) if ($i ~ /\)$/) { print $(i + 12), $(i + 13); exit } }' \
    "/proc/$1/stat" 2>/dev/null
}

# 用轻量监视器代替 rosbag2 录制: 重构版 500 Hz 重复发布整幅点云,
# 录成 bag 会在几十秒内写满 15 GB
python3 "$WS/scripts/pl_cmp/monitor.py" "$OUT" "${MONITOR_TOPICS:-odom+cloud+path}" > "$OUT/monitor.log" 2>&1 &
REC=$!
sleep 3

"$EXE" --ros-args -r __node:=point_lio --params-file "$PARAMS" \
  > "$OUT/node.log" 2>&1 &
NODE=$!
sleep 5

START=$(date +%s.%N)
ros2 bag play "$BAG" --rate 1.0 > "$OUT/play.log" 2>&1
END=$(date +%s.%N)
echo "[$LABEL] 回放耗时 $(echo "$END - $START" | bc) s" | tee "$OUT/play_time.txt"

# 回放结束后继续等待, 让节点把积压的帧处理完 (CPU 不再增长或到上限为止)
DRAIN_LIMIT=${DRAIN_LIMIT:-600}
DRAIN_START=$(date +%s)
PREV_CPU=""
while [ -d "/proc/$NODE" ]; do
  sleep 10
  read -r UT ST <<<"$(cpu_ticks "$NODE")"
  CUR="${UT:-},${ST:-}"
  ELAPSED=$(( $(date +%s) - DRAIN_START ))
  if [ "$CUR" = "$PREV_CPU" ]; then
    echo "[$LABEL] 排空结束: 等待 ${ELAPSED} s (CPU 不再增长)" | tee -a "$OUT/play_time.txt"
    break
  fi
  PREV_CPU="$CUR"
  if [ "$ELAPSED" -ge "$DRAIN_LIMIT" ]; then
    echo "[$LABEL] 排空结束: 达到上限 ${DRAIN_LIMIT} s, 可能仍在处理" | tee -a "$OUT/play_time.txt"
    break
  fi
done

# 采样 CPU 与峰值内存 (在终止之前)
read -r UT ST <<<"$(cpu_ticks "$NODE")"
HWM=$(grep VmHWM "/proc/$NODE/status" 2>/dev/null | awk '{print $2}')
if [ -n "${UT:-}" ]; then
  echo "[$LABEL] CPU: user=$(echo "scale=2; $UT/100" | bc) s sys=$(echo "scale=2; ${ST:-0}/100" | bc) s 峰值内存=${HWM:-?} kB" \
    | tee "$OUT/resource.txt"
fi

pkill -x pointlio_mapping 2>/dev/null
sleep 2
pkill -9 -x pointlio_mapping 2>/dev/null
pkill -TERM -f "[m]onitor.py" 2>/dev/null
sleep 3
pkill -9 -f "[m]onitor.py" 2>/dev/null
sleep 1

echo "[$LABEL] 完成, 输出在 $OUT"
