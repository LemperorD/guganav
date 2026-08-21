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
    -m / --model         强制重编译控制器模型
    -p / --packages      仅指定安全编译的包(空格分隔多个包名)
    -t / --tes           安全编译TES相关包(内存限制+单线程,防死机)
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

# 阶段一(安全模式)编译的包; 阶段二总是全量编译, 保证 compile_commands.json 含全部包
PACKAGES_SELECT="livox_ros_driver2 point_lio nav2_mppi_controller nonrotating_vel_transform"

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
        -p | --packages)
            PACKAGES_SELECT="$2"
            shift 2
            ;;
        -t | --tes)
            # point_lio 依赖 livox_ros_driver2，清空 install/ 后必须一起构建
            PACKAGES_SELECT="livox_ros_driver2 point_lio nav2_mppi_controller nonrotating_vel_transform"
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

if [ -n "$PACKAGES_SELECT" ]; then
  echo "Packages   : $PACKAGES_SELECT"
  # shellcheck disable=SC2086
  BUILD_ARGS+=(--packages-select $PACKAGES_SELECT)
fi

# ---- 阶段一: 安全编译指定包 (内存限制+单线程, 防死机) ----
echo "Safe build : systemd-run MemoryMax=6G, -j1, sequential executor"
if ! systemd-run --user --scope \
  -p MemoryHigh=5G \
  -p MemoryMax=6G \
  bash -lc "cd '$WS' && export MAKEFLAGS='-j1 -l1' && colcon build ${BUILD_ARGS[*]} --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"; then
  echo "Build failed, please check the error messages."
  exit 1
fi

# ---- 阶段二: 全量编译所有包 (阶段一的包已 up-to-date 会跳过) ----
# 目的: 让根 compile_commands.json 包含全部包的条目, clangd 才能索引所有头文件
echo "Full build : 全量编译所有包"
export MAKEFLAGS='-j1 -l1'
if ! colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_EXPORT_COMPILE_COMMANDS=ON; then
  echo "Build failed, please check the error messages."
  exit 1
fi

# 无条件刷新软链接, 保证指向本次构建生成的 compile_commands.json
ln -sf "$WS/build/compile_commands.json" "$WS/compile_commands.json"
echo "已创建compile_commands.json的软链接, 请设置vscode的c_cpp_properties.json以启用代码补全和跳转功能。"
