#!/usr/bin/env bash
# 验证"把 loam_interface 的职责搬进 point_lio"是否与原链路等价。
#
# A 阶段: point_lio(output_frame.enable=false) + loam_interface  -> 旧链路
# B 阶段: point_lio(output_frame.enable=true)                    -> 合入后
#
# 注意: loam_interface 包已随合并删除, 现在只有 B 阶段可以运行。
# 需要复现 A 阶段时, 从合并前的提交取回该包再编译, 例如:
#   git archive <合并前提交> src/guga_transform/loam_interface \
#     | tar -x -C /tmp/old_ws/src --strip-components=2
# 两阶段使用同一份 bag 与同一套静态 TF, 比较 /registered_scan 与 /lidar_odometry。
set +u

WS=/home/rog/guganav
WORK="${PL_CMP_DIR:-$WS/.pl_cmp_bench}"
BAG="$WORK/bags/mid360_synth"
export ROS_HOME="$WORK/roshome"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"

pkill -x pointlio_mapping 2>/dev/null
pkill -x loam_interface_node 2>/dev/null
pkill -f "[l]oam_interface_node" 2>/dev/null
pkill -x static_transform_publisher 2>/dev/null
pkill -f "[m]onitor_project.py" 2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# 与实车一致的静态 TF: base_footprint -> chassis -> front_mid360
# chassis -> front_mid360: xyz(0.225, 0, 0.107) rpy(-10deg, 0, -90deg)
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.123 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_footprint --child-frame-id chassis > "$WORK/tf1.log" 2>&1 &
TF1=$!
ros2 run tf2_ros static_transform_publisher \
  --x 0.225 --y 0 --z 0.107 --roll -0.17453292519943295 --pitch 0 \
  --yaw -1.5707963267948966 \
  --frame-id chassis --child-frame-id front_mid360 > "$WORK/tf2.log" 2>&1 &
TF2=$!
sleep 3

PARAMS=$(ros2 pkg prefix point_lio)/share/point_lio/config/mid360.yaml
EXE=$(ros2 pkg prefix point_lio)/lib/point_lio/pointlio_mapping
echo "params: $PARAMS"

run_phase() {
  local label="$1" enable="$2" with_loam="$3"
  local out="$WORK/verify/$label"
  mkdir -p "$out"
  rm -rf "$out"/*.csv "$out"/*.log

  python3 "$WS/scripts/pl_cmp/monitor_project.py" "$out" > "$out/monitor.log" 2>&1 &
  sleep 2
  "$EXE" --ros-args -r __node:=point_lio --params-file "$PARAMS" \
    -p "output_frame.enable:=$enable" > "$out/node.log" 2>&1 &
  if [ "$with_loam" = "yes" ]; then
    sleep 4
    ros2 run loam_interface loam_interface_node --ros-args \
      -p state_estimation_topic:=aft_mapped_to_init \
      -p registered_scan_topic:=cloud_registered \
      -p odom_frame:=odom -p base_frame:=base_footprint \
      -p lidar_frame:=front_mid360 > "$out/loam.log" 2>&1 &
  fi
  sleep 5
  ros2 bag play "$BAG" --rate 1.0 > "$out/play.log" 2>&1
  sleep 20   # 让节点处理完剩余帧

  pkill -x pointlio_mapping 2>/dev/null
  pkill -x loam_interface_node 2>/dev/null
pkill -f "[l]oam_interface_node" 2>/dev/null
  sleep 2
  pkill -9 -x pointlio_mapping 2>/dev/null
  pkill -9 -x loam_interface_node 2>/dev/null
  pkill -9 -f "[l]oam_interface_node" 2>/dev/null
  pkill -TERM -f "[m]onitor_project.py" 2>/dev/null
  sleep 2
  pkill -9 -f "[m]onitor_project.py" 2>/dev/null
  echo "[$label] 完成: $(wc -l < "$out/lidar_odometry.csv" 2>/dev/null) 条里程计, $(wc -l < "$out/registered_scan.csv" 2>/dev/null) 条点云"
}

PHASES="${1:-old_chain merged}"
for phase in $PHASES; do
  case "$phase" in
    old_chain) run_phase "old_chain" false yes ;;
    old_chain2) run_phase "old_chain2" false yes ;;
    merged)    run_phase "merged" true no ;;
    *) echo "未知阶段: $phase" ;;
  esac
done

kill $TF1 $TF2 2>/dev/null
pkill -x static_transform_publisher 2>/dev/null
echo "验证结束"
