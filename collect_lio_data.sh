#!/usr/bin/env bash
# small_point_lio 发散诊断采集脚本
NS=${1:-red_standard_robot1}
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/guganav/install/setup.bash 2>/dev/null
export ROS_LOG_DIR=~/guganav/log

echo "=== [1/5] 静止基线 lidar_odometry ==="
sleep 3
timeout 5 ros2 topic echo /$NS/lidar_odometry --once 2>/dev/null | grep -E "frame_id|child_frame|position|orientation" | head -8

echo ""
echo "=== [2/5] IMU 静止数据 ==="
timeout 5 ros2 topic echo /$NS/livox/imu --once 2>/dev/null | grep -E "linear_acceleration|angular_velocity" -A 4 | head -12

echo ""
echo "=== [3/5] 点云话题检查 ==="
timeout 5 ros2 topic info /$NS/velodyne_points 2>/dev/null | grep -E "Type|Publisher|Subscriber" | head -4

echo ""
echo "=== [4/5] 请让小车移动 5 秒! ==="
echo ">>> 移动中采集..."
sleep 3
timeout 5 ros2 topic echo /$NS/lidar_odometry --once 2>/dev/null | grep -E "frame_id|child_frame|position|orientation" | head -8

echo ""
echo "=== [5/5] 移动后再次采集 ==="
sleep 2
timeout 5 ros2 topic echo /$NS/lidar_odometry --once 2>/dev/null | grep -E "position|orientation" | head -6

echo "=== 采集完成 ==="
