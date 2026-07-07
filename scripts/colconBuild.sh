#!/bin/bash
# 咕嘎导航构建脚本
# 用法: bash scripts/colconBuild.sh [--test|-t]
#   --test, -t  启用测试 (传递 -DBUILD_TESTING=ON 给 CMake)

set -e

# 解析可选参数
TESTING_FLAG=""
for arg in "$@"; do
  if [ "$arg" = "--test" ] || [ "$arg" = "-t" ]; then
    TESTING_FLAG="-DBUILD_TESTING=ON"
  fi
done

cd ~/guganav
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  ${TESTING_FLAG:+"$TESTING_FLAG"}