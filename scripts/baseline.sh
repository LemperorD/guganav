#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)

usage() {
  cat <<'EOF'
用法:
  scripts/baseline.sh

交互式直线基线入口。可选择：
  1. 轻量行为录包
  2. 诊断录包
  3. 热点 topic 频率测量
  4. perf / Hotspot profiling
  5. 往返发布 goal_pose

除第 5 项外，本脚本只调用 scripts/perf/ 下的具体工具，不启动导航、不发布 goal。
EOF
}

prompt_default() {
  local prompt=$1
  local default_value=$2
  local value
  read -r -p "$prompt [$default_value]: " value
  printf '%s' "${value:-$default_value}"
}

prompt_optional() {
  local prompt=$1
  local value
  read -r -p "$prompt: " value
  printf '%s' "$value"
}

confirm() {
  local prompt=$1
  local value
  read -r -p "$prompt [Y/n]: " value
  case "$value" in
    n | N | no | NO)
      return 1
      ;;
    *)
      return 0
      ;;
  esac
}

select_environment_args() {
  local env_choice namespace
  echo
  echo "选择环境:"
  echo "  1. 实车"
  echo "  2. 仿真(/red_standard_robot1)"
  echo "  3. 自定义 namespace"
  read -r -p "环境 [1]: " env_choice
  case "${env_choice:-1}" in
    1 | real | reality)
      ENV_ARGS=()
      ;;
    2 | sim | simulation)
      ENV_ARGS=(--simulation)
      ;;
    3 | ns | namespace)
      namespace=$(prompt_default "namespace" "/red_standard_robot1")
      ENV_ARGS=(--namespace "$namespace")
      ;;
    *)
      echo "未知环境选项: $env_choice" >&2
      exit 2
      ;;
  esac
}

run_command() {
  local cmd=("$@")
  echo
  echo "将执行:"
  printf '  %q' "${cmd[@]}"
  echo
  if confirm "确认执行"; then
    exec "${cmd[@]}"
  fi
  echo "已取消"
}

if [ $# -gt 0 ]; then
  case "$1" in
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

echo "=== 直线基线工具 ==="
echo "  1. 轻量行为录包"
echo "  2. 诊断录包"
echo "  3. 热点 topic 频率测量"
echo "  4. perf / Hotspot profiling"
echo "  5. 往返发布 goal_pose"
read -r -p "选择模式 [1]: " mode
mode=${mode:-1}

case "$mode" in
  1 | behavior | bag)
    select_environment_args
    scene=$(prompt_default "场景名" "straight_v0")
    duration=$(prompt_default "录制时长秒数，0 表示 Ctrl-C 停止" "120")
    notes=$(prompt_optional "备注，可留空")
    cmd=("$WS/scripts/perf/record_straight_baseline.sh" "${ENV_ARGS[@]}" --scene "$scene" --duration "$duration")
    if [ -n "$notes" ]; then
      cmd+=(--notes "$notes")
    fi
    run_command "${cmd[@]}"
    ;;
  2 | diagnostic | diag)
    select_environment_args
    scene=$(prompt_default "场景名" "straight_diag_v0")
    duration=$(prompt_default "录制时长秒数，0 表示 Ctrl-C 停止" "120")
    notes=$(prompt_optional "备注，可留空")
    cmd=("$WS/scripts/perf/record_straight_diagnostic.sh" "${ENV_ARGS[@]}" --scene "$scene" --duration "$duration")
    if [ -n "$notes" ]; then
      cmd+=(--notes "$notes")
    fi
    run_command "${cmd[@]}"
    ;;
  3 | hz | topic)
    select_environment_args
    duration=$(prompt_default "测量时长秒数" "30")
    custom_topics=$(prompt_optional "自定义 topic 列表，空格分隔；留空使用默认热点 topic")
    cmd=("$WS/scripts/perf/measure_hot_topic_hz.sh" "${ENV_ARGS[@]}" --duration "$duration")
    if [ -n "$custom_topics" ]; then
      # shellcheck disable=SC2206
      topics=($custom_topics)
      cmd+=("${topics[@]}")
    fi
    run_command "${cmd[@]}"
    ;;
  4 | perf | profile | profiling)
    scene=$(prompt_default "场景名" "straight_profile_v0")
    duration=$(prompt_default "profiling 时长秒数" "30")
    target=$(prompt_default "目标进程名或 PID" "nav2_container")
    call_graph=$(prompt_default "调用栈模式(fp/dwarf,8192)" "fp")
    notes=$(prompt_optional "备注，可留空")
    if [[ "$target" =~ ^[0-9]+$ ]]; then
      cmd=("$WS/scripts/perf/record_straight_profiling.sh" --pid "$target")
    else
      cmd=("$WS/scripts/perf/record_straight_profiling.sh" --process "$target")
    fi
    cmd+=(--scene "$scene" --duration "$duration" --call-graph "$call_graph" --event cycles:u)
    if [ -n "$notes" ]; then
      cmd+=(--notes "$notes")
    fi
    run_command "${cmd[@]}"
    ;;
  5 | goal | roundtrip)
    select_environment_args
    interval=$(prompt_default "发布间隔秒数" "30")
    count=$(prompt_default "发布次数，0 表示无限循环" "0")
    start=$(prompt_default "起点 x y yaw" "0 0 0")
    end=$(prompt_default "终点 x y yaw" "9 -5.5 0")
    cmd=("$WS/scripts/perf/publish_roundtrip_goal.py" "${ENV_ARGS[@]}" --interval "$interval" --count "$count")
    # shellcheck disable=SC2206
    start_values=($start)
    # shellcheck disable=SC2206
    end_values=($end)
    if [ "${#start_values[@]}" -ne 3 ] || [ "${#end_values[@]}" -ne 3 ]; then
      echo "起点和终点都必须是 3 个数: x y yaw" >&2
      exit 2
    fi
    cmd+=(--start "${start_values[@]}" --end "${end_values[@]}")
    run_command "${cmd[@]}"
    ;;
  *)
    echo "未知模式: $mode" >&2
    exit 2
    ;;
esac
