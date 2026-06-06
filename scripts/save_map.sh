#!/bin/bash
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source ./install/setup.bash

mkdir -p src/guga_nav_bringup/map/reality
cd src/guga_nav_bringup/map/reality

TIME_STR=$(date +"%Y-%m-%d_%H-%M-%S")
MAP_NAME="map_real_${TIME_STR}"

echo "保存 2D 栅格地图到 $(pwd)/${MAP_NAME}.*"
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"
