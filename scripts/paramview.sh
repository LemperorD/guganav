#!/usr/bin/env bash
set -euo pipefail

# ────────────────────────────────────────────────────────────────
# 动态调参 Web UI 一键启动
#
# 用法：
#   scripts/paramview.sh                                  # 默认调 controller_server 的 FollowPath（mppi 参数表）
#   scripts/paramview.sh --profile pid                    # 换用 pid 参数表
#   scripts/paramview.sh --target planner_server:GridBased --profile mppi
#   scripts/paramview.sh --mode simulation --planner smac2d
#   scripts/paramview.sh --port 8091 --no-browser
#   scripts/paramview.sh --read-only                      # 只调不存，禁止写回配置文件
#
# 参数表取自 scripts/params_list/<profile>_para.txt（名称|说明）；基线取自
# scripts/param_merge.py 合并后的 yaml，用来在界面上标出"当前值已经偏离配置值"。
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

PROFILE="mppi"
TARGET=""
PORT=8090
HOST=127.0.0.1
HZ=2.0
OPEN_BROWSER=1
READ_ONLY=0
BASELINE_MODE="reality"
BASELINE_CONTROLLER=""
BASELINE_PLANNER="jps"

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

LIST_FILE="$WS/scripts/params_list/${PROFILE}_para.txt"
if [ ! -f "$LIST_FILE" ]; then
  echo "缺少参数表：$LIST_FILE" >&2
  echo "可用的 profile：$(ls "$WS/scripts/params_list" | sed 's/_para.txt//' | tr '\n' ' ')" >&2
  exit 1
fi

# 目标默认按 profile 推断：控制器参数都在 controller_server 的 FollowPath 下
if [ -z "$TARGET" ]; then
  TARGET="controller_server:FollowPath"
fi

if [ "$BASELINE_MODE" = "simulation" ]; then
  BASELINE_CONTROLLER="${BASELINE_CONTROLLER:-$PROFILE}"
else
  BASELINE_CONTROLLER="${BASELINE_CONTROLLER:-$PROFILE}"
fi

require_workspace_setup
export ROS_LOG_DIR="${WS}/log/ros"
mkdir -p "$ROS_LOG_DIR"

ARGS=(
  --list "$LIST_FILE"
  --host "$HOST"
  --port "$PORT"
  --hz "$HZ"
  --baseline-mode "$BASELINE_MODE"
  --baseline-controller "$BASELINE_CONTROLLER"
  --baseline-planner "$BASELINE_PLANNER"
)
if [ "$READ_ONLY" -eq 1 ]; then
  ARGS+=(--no-save)
fi
for target in $TARGET; do
  ARGS+=(--target "$target")
done

URL="http://${HOST}:${PORT}"
echo "参数表: $LIST_FILE"
echo "目标  : $TARGET"
echo "地址  : $URL"
if [ "$READ_ONLY" -eq 1 ]; then
  echo "保存  : 已禁用（--read-only），参数不会写回配置文件"
else
  echo "保存  : 写回 src/guga_bringup/config/${BASELINE_MODE}/ 下的 base.yaml 与"
  echo "        controller/${BASELINE_CONTROLLER}.yaml、planner/${BASELINE_PLANNER}.yaml（下次启动生效）"
fi

if [ "$OPEN_BROWSER" -eq 1 ] && command -v xdg-open >/dev/null 2>&1; then
  (sleep 1.5 && xdg-open "$URL" >/dev/null 2>&1) &
fi

exec python3 "$WS/scripts/paramview/paramview_server.py" "${ARGS[@]}"
