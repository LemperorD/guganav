#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)
BASE_SCRIPT="$WS/scripts/perf/record_straight_baseline.sh"

usage() {
  cat <<'EOF'
用法:
  scripts/perf/record_straight_diagnostic.sh [baseline options]

录制直线诊断基线。本脚本是下面命令的轻量包装：

  scripts/perf/record_straight_baseline.sh --diagnostic

建议在轻量行为基线暴露问题后再使用。诊断模式会额外录制点云、terrain
和 costmap topic，因此不要把它的运行开销和轻量行为 bag 直接对比。
EOF
}

if [ $# -gt 0 ]; then
  case "$1" in
    -h | --help)
      usage
      exit 0
      ;;
  esac
fi

exec "$BASE_SCRIPT" --diagnostic "$@"
