#!/usr/bin/env bash
set -euo pipefail

# ────────────────────────────────────────────────────────────────
# 动态调参 Web UI 一键启动
#
# 用法：
#   scripts/paramview.sh                                  # 自动识别仿真/实车命名空间、控制器（mppi/pid/mpc）与规划器
#   scripts/paramview.sh --profile pid                    # 显式指定 pid 参数表与控制器基线
#   scripts/paramview.sh --target planner_server:GridBased --profile mppi
#   scripts/paramview.sh --mode simulation --planner smac2d
#   scripts/paramview.sh --port 8091 --no-browser
#   scripts/paramview.sh --read-only                      # 只调不存，禁止写回配置文件
#
# 参数表取自 scripts/paramview/params_list/<profile>_para.txt（名称|说明）；基线取自
# scripts/paramview/param_merge.py 合并后的 yaml，用来在界面上标出"当前值已经偏离配置值"。
#
# 界面上的"保存到配置"把当前与配置不同的参数写回三层 yaml 里最后出现该参数的
# 那个文件（base → controller → planner，与 param_merge.py 的合并顺序一致），
# 保留原有注释与排版，写前备份到 log/paramview_backup/<时间戳>/，下次启动生效。
#
# Ctrl-C 结束。
# ────────────────────────────────────────────────────────────────

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)
GUGANAV_MODE="paramview"
source "$WS/scripts/lib/common.sh"

PROFILE=""
TARGET=""
PORT=8090
HOST=127.0.0.1
HZ=2.0
OPEN_BROWSER=1
READ_ONLY=0
BASELINE_MODE="auto"
BASELINE_CONTROLLER=""
BASELINE_PLANNER="auto"

usage() { sed -n '4,23p' "$0"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h | --help) usage; exit 0 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --target) TARGET="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --hz) HZ="$2"; shift 2 ;;
    --mode) BASELINE_MODE="$2"; shift 2 ;;
    --planner) BASELINE_PLANNER="$2"; shift 2 ;;
    --read-only) READ_ONLY=1; shift ;;
    --no-browser) OPEN_BROWSER=0; shift ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
done

LIST_FILE=""
if [ -n "$PROFILE" ]; then
  LIST_FILE="$WS/scripts/paramview/params_list/${PROFILE}_para.txt"
  if [ ! -f "$LIST_FILE" ]; then
    echo "缺少参数表：$LIST_FILE" >&2
    echo "可用的 profile：$(ls "$WS/scripts/paramview/params_list" | sed 's/_para.txt//' | tr '\n' ' ')" >&2
    exit 1
  fi
  BASELINE_CONTROLLER="${BASELINE_CONTROLLER:-$PROFILE}"
else
  BASELINE_CONTROLLER="${BASELINE_CONTROLLER:-auto}"
fi

# 目标默认按 controller_server 的 FollowPath 匹配（服务端会自动解析仿真命名空间）
if [ -z "$TARGET" ]; then
  TARGET="controller_server:FollowPath"
fi

require_workspace_setup
export ROS_LOG_DIR="${WS}/log/ros"
mkdir -p "$ROS_LOG_DIR"

ARGS=(
  --host "$HOST"
  --port "$PORT"
  --hz "$HZ"
  --baseline-mode "$BASELINE_MODE"
  --baseline-controller "$BASELINE_CONTROLLER"
  --baseline-planner "$BASELINE_PLANNER"
)
if [ -n "$LIST_FILE" ]; then
  ARGS+=(--list "$LIST_FILE")
fi
if [ "$READ_ONLY" -eq 1 ]; then
  ARGS+=(--no-save)
fi
for target in $TARGET; do
  ARGS+=(--target "$target")
done

URL="http://${HOST}:${PORT}"
if [ -n "$LIST_FILE" ]; then
  echo "参数表: $LIST_FILE"
else
  echo "参数表: 自动识别（按运行中的 FollowPath.plugin 匹配 mppi/pid/mpc）"
fi
echo "目标  : $TARGET（自动适配仿真/实车命名空间）"
echo "地址  : $URL"
if [ "$READ_ONLY" -eq 1 ]; then
  echo "保存  : 已禁用（--read-only），参数不会写回配置文件"
elif [ "$BASELINE_MODE" = "auto" ] || [ "$BASELINE_CONTROLLER" = "auto" ] || [ "$BASELINE_PLANNER" = "auto" ]; then
  echo "保存  : 自动识别 mode=${BASELINE_MODE} controller=${BASELINE_CONTROLLER} planner=${BASELINE_PLANNER}"
else
  echo "保存  : 写回 src/guga_bringup/config/${BASELINE_MODE}/ 下的 base.yaml 与"
  echo "        controller/${BASELINE_CONTROLLER}.yaml、planner/${BASELINE_PLANNER}.yaml（下次启动生效）"
fi

if [ "$OPEN_BROWSER" -eq 1 ] && command -v xdg-open >/dev/null 2>&1; then
  (sleep 1.5 && xdg-open "$URL" >/dev/null 2>&1) &
fi

exec python3 "$WS/scripts/paramview/paramview_server.py" "${ARGS[@]}"
