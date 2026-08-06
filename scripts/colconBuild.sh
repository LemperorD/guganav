#!/usr/bin/env bash
set -euo pipefail

# guganav构建脚本

show_help() {
cat << EOF
Usage:
    <workspace>/scripts/colconBuild.sh [OPTIONS]

Options:
    -h / --help          显示帮助
    -c / --clean         清理build目录
    -r / --release       Release编译
    -d / --debug         Debug编译
EOF
}

# 获取当前脚本所在的工作空间路径
WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)
HIK_MVS_ROOT="${HIK_MVS_ROOT:-/opt/MVS}"

# 默认参数
CLEAN=false
BUILD_TYPE=Release

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h | --help)
            show_help
            exit 0
            ;;
        -c | --clean)
            CLEAN=true
            shift
            ;;
        -r | --release)
            BUILD_TYPE=Release
            shift
            ;;
        -d | --debug)
            BUILD_TYPE=Debug
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage."
            exit 1
            ;;
    esac
done

set +u
# shellcheck source=/dev/null
source "$WS/scripts/ci/env_humble.sh"
set -u

echo "Workspace  : $WS"
echo "Build type : $BUILD_TYPE"
echo "Clean      : $CLEAN"

if $CLEAN; then
  echo "[Warn] maybe need your password"
  sudo rm -rf "$WS/build" "$WS/log" "$WS/install"
  echo "Cleaning..."
fi

echo "Building..."

cd "$WS"
export HIK_MVS_ROOT
BUILD_ARGS=(--symlink-install)
HIK_MVS_LIBRARIES=(MvCameraControl FormatConversion MediaProcess MVRender MvUsb3vTL)
HIK_MVS_AVAILABLE=true
if [ ! -f "$HIK_MVS_ROOT/include/MvCameraControl.h" ]; then
  HIK_MVS_AVAILABLE=false
fi
for library in "${HIK_MVS_LIBRARIES[@]}"; do
  if [ ! -e "$HIK_MVS_ROOT/lib/64/lib${library}.so" ]; then
    HIK_MVS_AVAILABLE=false
  fi
done

if $HIK_MVS_AVAILABLE; then
  echo "Hik MVS    : enabled ($HIK_MVS_ROOT)"
else
  echo "Hik MVS    : unavailable; skipping hik_driver"
  BUILD_ARGS+=(--packages-skip hik_driver)
fi

colcon build "${BUILD_ARGS[@]}" --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
if [ ! -L "$WS/compile_commands.json" ]; then
  ln -sf "$WS/build/compile_commands.json" "$WS/compile_commands.json"
  echo "已创建compile_commands.json的软链接, 请设置vscode的c_cpp_properties.json以启用代码补全和跳转功能。"
fi
