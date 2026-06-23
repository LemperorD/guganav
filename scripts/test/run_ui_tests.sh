#!/bin/bash
# guga_ui 测试运行脚本
# 用法: bash scripts/run_ui_tests.sh
set -euo pipefail

WS="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"

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
source_setup "$WS/install/setup.bash"
cd "$WS"

# 构建（带测试）
echo ">>> Building guga_ui with tests ..."
colcon build --symlink-install --packages-select guga_ui_common guga_ui_pangolin \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source_setup "$WS/install/setup.bash"

FAILED=0

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║           guga_ui 测试报告                           ║"
echo "╚══════════════════════════════════════════════════════╝"

# ─── guga_ui_common ───
echo ""
echo "────────────── guga_ui_common (3 suites) ──────────────"
cd "$WS/build/guga_ui_common"
if ! ctest --output-on-failure; then
  FAILED=1
fi

# ─── guga_ui_pangolin ───
echo ""
echo "───────────── guga_ui_pangolin (2 suites) ─────────────"
cd "$WS/build/guga_ui_pangolin"
if ! ctest --output-on-failure; then
  FAILED=1
fi

echo ""
if [ $FAILED -eq 0 ]; then
  echo "✅ guga_ui: all tests PASSED"
else
  echo "❌ guga_ui: some tests FAILED"
  exit 1
fi
