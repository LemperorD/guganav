#!/usr/bin/env bash
# 编译 MID-360 IMU 读取工具。产物是当前目录下的 imu_lidar。
set -euo pipefail

cd "$(dirname "$0")"

SDK="$(cd ../../src/guga_driver/livox_ros_driver2/Livox-SDK2 && pwd)"

case "$(uname -m)" in
  x86_64) LIB_ARCH=amd64 ;;
  aarch64 | arm64) LIB_ARCH=arm64 ;;
  riscv64) LIB_ARCH=riscv64 ;;
  *)
    echo "不支持的架构: $(uname -m)" >&2
    exit 1
    ;;
esac

g++ -O2 -std=c++17 -Wall -Wextra -pthread \
    -I"$SDK/include" \
    -o imu_lidar imu_lidar.cpp \
    -L"$SDK/lib/$LIB_ARCH" \
    -llivox_lidar_sdk_shared \
    -Wl,-rpath,"$SDK/lib/$LIB_ARCH"

echo "编译完成: $(pwd)/imu_lidar"
