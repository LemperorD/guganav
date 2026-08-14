#!/usr/bin/env bash
set -euo pipefail

# guganav构建脚本

show_help() {
cat << EOF
Usage:
    <workspace>/scripts/colconBuild.sh [OPTIONS]

Options:
    -h  / --help          显示帮助
    -c  / --clean         清理build目录
    -r  / --release       Release编译
    -d  / --debug         Debug编译
    -fm / --force-model   强制重编译控制器模型
EOF
}

# 获取当前脚本所在的工作空间路径
WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)
HIK_MVS_ROOT="${HIK_MVS_ROOT:-/opt/MVS}"

# 默认参数
CLEAN=false
BUILD_TYPE=Release

# 是否重新编译MPC控制器模型
FORCE_MODEL=false

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
        -fm | --force-model)
            FORCE_MODEL=true
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


if $FORCE_MODEL; then
  echo "正在强制编译控制器模型..."
  bash "$WS/scripts/ci/generate_acados_mpc.sh"
  if [ $? -ne 0 ]; then
    echo "控制器模型编译失败，请检查错误信息。"
    exit 1
  fi
else
  if [ ! -f "$WS/src/guga_controller/mpc_controller/generated/omni/omni_ocp/acados_solver_omni.h" ]; then
    echo "控制器模型不存在，正在编译..."
    bash "$WS/scripts/ci/generate_acados_mpc.sh"
    if [ $? -ne 0 ]; then
      echo "控制器模型编译失败，请检查错误信息。"
      exit 1
    fi
  else 
    echo "控制器模型已存在，跳过编译。"
  fi
fi

# source "$WS/install/setup.bash"
colcon build "${BUILD_ARGS[@]}" --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
if [ ! -L "$WS/compile_commands.json" ]; then
  ln -sf "$WS/build/compile_commands.json" "$WS/compile_commands.json"
  echo "已创建compile_commands.json的软链接, 请设置vscode的c_cpp_properties.json以启用代码补全和跳转功能。"
fi