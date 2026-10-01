#!/usr/bin/env bash
set -euo pipefail

# ────────────────────────────────────────────────────────────────
# 假数据源动态调参工具（运行时调整，无需重启节点）
#
# fake_msg_source 的所有字段都注册为 ROS 2 参数，publish_status() 每帧重新读取，
# 所以改完下一帧（20 Hz，约 50 ms）即生效。
#
# 用法：
#   scripts/tune_fake_source.sh                              # 交互菜单
#   scripts/tune_fake_source.sh set <param> <value> [node]   # 改单个参数
#   scripts/tune_fake_source.sh hp <value> [node]            # 快捷：改当前血量
#   scripts/tune_fake_source.sh ammo <value> [node]          # 快捷：改允许发弹量
#   scripts/tune_fake_source.sh heat <value> [node]          # 快捷：改当前枪管热量
#   scripts/tune_fake_source.sh hit <damage> [node]          # 在现有血量上扣血
#   scripts/tune_fake_source.sh show [node]                  # 显示当前取值
#   scripts/tune_fake_source.sh dump [node]                  # 导出全部参数
#   scripts/tune_fake_source.sh save <file> [node]           # 保存到文件
#   scripts/tune_fake_source.sh restore <file> [node]        # 从文件恢复
#
# 节点名默认 /fake_msg_source；若数据源带命名空间启动（例如仿真里的
# /red_standard_robot1/fake_msg_source），把完整节点名作为最后一个参数传入。
#
# ros2 param 依赖 node graph 查询，某些环境会报 "Node not found"。
# 本脚本在 param 命令失败时回退到直接调用 set_parameters / get_parameters
# 服务，那条路只依赖服务发现。
# ────────────────────────────────────────────────────────────────

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
  fi
}

if [[ -z "${ROS_DISTRO:-}" ]]; then
  source_setup /opt/ros/humble/setup.bash
fi
source_setup "$WS/install/setup.bash"

DEFAULT_NODE="/fake_msg_source"

# 参数表：名称|类型|是否即时生效|说明
PARAM_TABLE=(
  "current_hp|integer|yes|当前血量"
  "maximum_hp|integer|yes|血量上限（RMUL 哨兵为 400）"
  "robot_id|integer|yes|机器人 ID（哨兵为 7）"
  "robot_level|integer|yes|机器人等级"
  "shooter_barrel_cooling_value|integer|yes|枪管冷却（每秒）"
  "shooter_barrel_heat_limit|integer|yes|枪管热量上限"
  "shooter_17mm_1_barrel_heat|integer|yes|当前枪管热量"
  "projectile_allowance_17mm|integer|yes|允许发弹量"
  "remaining_gold_coin|integer|yes|剩余金币"
  "publish_rate|double|no|发布频率（改动需重启节点）"
  "enemy_count|integer|yes|敌人数量（>0 即有敌）"
  "vision_rate|double|no|视觉发布频率（改动需重启节点）"
)

param_names() {
  local entry
  for entry in "${PARAM_TABLE[@]}"; do
    printf '%s\n' "${entry%%|*}"
  done
}

param_field() {
  local want=$1 field=$2 entry
  for entry in "${PARAM_TABLE[@]}"; do
    if [ "${entry%%|*}" = "$want" ]; then
      printf '%s\n' "$(cut -d'|' -f"$field" <<<"$entry")"
      return 0
    fi
  done
  return 1
}

param_type() { param_field "$1" 2 2>/dev/null || printf 'integer\n'; }
param_desc() { param_field "$1" 4 2>/dev/null || printf '\n'; }
param_immediate() { [ "$(param_field "$1" 3 2>/dev/null || printf yes)" = "yes" ]; }

# ── 读参数：先 ros2 param get，失败回退到 get_parameters 服务 ──
param_value() {
  local node=$1 param=$2 out raw

  if out=$(timeout 2 ros2 param get "$node" "$param" 2>/dev/null); then
    out=$(printf '%s\n' "$out" | tail -n 1 | sed 's/^.*is: //')
    if [ -n "$out" ]; then
      printf '%s\n' "$out"
      return 0
    fi
  fi

  # ParameterValue 的字段顺序是 type/bool_value/integer_value/double_value…，
  # bool_value 排在整型前面，所以不能把两类放在同一个候选列表里取第一个匹配，
  # 否则整数会被读成 False。先找整型，再找浮点。
  raw=$(timeout 3 ros2 service call "${node%/}/get_parameters" \
      rcl_interfaces/srv/GetParameters "{names: ['$param']}" 2>/dev/null)
  out=$(printf '%s' "$raw" | grep -oE "integer_value=[^,)]*" | head -n 1 | cut -d= -f2)
  if [ -z "$out" ]; then
    out=$(printf '%s' "$raw" | grep -oE "double_value=[^,)]*" | head -n 1 | cut -d= -f2)
  fi
  if [ -n "$out" ]; then
    printf '%s\n' "$out"
  else
    printf '?\n'
  fi
}

# ── 写参数：先 ros2 param set，失败回退到 set_parameters 服务 ──
# 参数顺序：node param type value
set_param() {
  local node=$1 param=$2 type=$3 value=$4

  # 加 timeout：param 查询不可用时这条命令可能长时间挂住
  if timeout 5 ros2 param set "$node" "$param" "$value" >/dev/null 2>&1; then
    return 0
  fi

  local req
  case "$type" in
    double)
      req="{parameters: [{name: $param, value: {type: 3, double_value: $value}}]}"
      ;;
    *)
      req="{parameters: [{name: $param, value: {type: 2, integer_value: $value}}]}"
      ;;
  esac

  timeout 8 ros2 service call "${node%/}/set_parameters" \
    rcl_interfaces/srv/SetParameters "$req" 2>/dev/null | grep -q "successful=True"
}

# 先确认节点可达，否则逐项重试会把时间耗光
probe_node() {
  local node=$1
  [ "$(param_value "$node" current_hp)" != "?" ] && return 0
  {
    echo "ERROR: 读不到 $node 的参数。常见原因："
    echo "       - 节点没启动，或名字不对。用 ros2 node list 确认；"
    echo "         带命名空间启动时要传完整名，例如"
    echo "         $0 show /red_standard_robot1/fake_msg_source"
    echo "       - param 查询暂时不可用（服务发现未完成），稍后重试。"
  } >&2
  return 1
}

require_number() {
  local value=$1 label=$2
  if ! [[ "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "ERROR: $label 需要是数字，收到: $value" >&2
    exit 1
  fi
}

dump_params() {
  local node=$1 p v
  while read -r p; do
    v=$(param_value "$node" "$p")
    [ "$v" = "?" ] && continue
    printf '%s: %s\n' "$p" "$v"
  done < <(param_names)
}

# ── 交互菜单：列出参数 → 选择 → 输入新值 → 应用 ──
interactive_menu() {
  local node=${1:-$DEFAULT_NODE}
  local -a names=()
  local -A values=()
  local -A changed=()

  probe_node "$node" || exit 1
  mapfile -t names < <(param_names)

  clear 2>/dev/null || true
  echo "== 正在读取参数（${#names[@]} 项，节点 $node）... =="
  local i=1 p
  for p in "${names[@]}"; do
    values[$p]=$(param_value "$node" "$p")
    printf "  %2d) %-34s [%s]\n" "$i" "$p" "${values[$p]}"
    i=$((i + 1))
  done

  while true; do
    clear 2>/dev/null || true

    # 只重取上次为 ? 的，以及刚改过需要确认的
    local -a refetch=()
    for p in "${names[@]}"; do
      if [ "${values[$p]}" = "?" ] || [ "${changed[$p]:-0}" = "1" ]; then
        refetch+=("$p")
      fi
    done
    if [ "${#refetch[@]}" -gt 0 ]; then
      echo "== 重新获取 ${#refetch[@]} 项（? 重试 + 更改确认）... =="
      for p in "${refetch[@]}"; do
        values[$p]=$(param_value "$node" "$p")
        [ "${values[$p]}" != "?" ] && changed[$p]=0
      done
    fi

    echo
    echo "== 假裁判动态调参（$node，${#names[@]} 项）=="
    echo "   ? = 读取超时或参数未声明；* = 上次改过，本循环已复查"
    i=1
    local mark
    for p in "${names[@]}"; do
      mark=""
      [ "${changed[$p]:-0}" = "1" ] && mark=" *"
      printf "  %2d) %-34s [%s]%s  %s\n" "$i" "$p" "${values[$p]}" "$mark" "$(param_desc "$p")"
      i=$((i + 1))
    done
    echo "  0) 退出"
    printf "选择 [0-%d]（s=保存当前参数）: " "${#names[@]}"
    read -r choice || break
    [ -z "$choice" ] && continue

    if [ "$choice" = "0" ]; then
      echo "退出。"
      break
    fi

    if [ "$choice" = "s" ] || [ "$choice" = "S" ]; then
      local save_file
      printf "  保存到文件 [默认 /tmp/fake_msg_source_params.yaml]: "
      read -r save_file || break
      [ -z "$save_file" ] && save_file="/tmp/fake_msg_source_params.yaml"
      dump_params "$node" >"$save_file"
      echo "  已保存 $(grep -c . "$save_file" 2>/dev/null || echo 0) 项 → $save_file"
      printf "  按回车继续..."
      read -r _ || true
      continue
    fi

    if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#names[@]}" ]; then
      echo "无效选择: $choice" >&2
      continue
    fi

    local param=${names[$((choice - 1))]}
    printf "  %s（%s）当前值: %s\n  新值: " "$param" "$(param_desc "$param")" "${values[$param]}"
    local value
    read -r value || break
    [ -z "$value" ] && continue

    if set_param "$node" "$param" "$(param_type "$param")" "$value"; then
      values[$param]=$value
      changed[$param]=1
      param_immediate "$param" || echo "  注意：该项改动需要重启节点才生效"
    else
      echo "  设置失败: $param = $value" >&2
      printf "  按回车继续..."
      read -r _ || true
    fi
  done
}

cmd=${1:-}
shift || true

case "$cmd" in
  set)
    # tune_fake_source.sh set <param> <value> [node]
    param=${1:-}
    value=${2:-}
    node=${3:-$DEFAULT_NODE}
    [ -n "$param" ] && [ -n "$value" ] || { sed -n '4,27p' "$0"; exit 1; }
    require_number "$value" "$param"
    if set_param "$node" "$param" "$(param_type "$param")" "$value"; then
      echo "==> $node $param = $value"
      param_immediate "$param" || echo "注意：该项改动需要重启节点才生效"
    else
      echo "设置失败: $node $param = $value" >&2
      exit 1
    fi
    ;;

  hp | ammo | heat | enemy)
    # 快捷改常用字段：tune_fake_source.sh <cmd> <value> [node]
    value=${1:-}
    node=${2:-$DEFAULT_NODE}
    [ -n "$value" ] || { echo "用法: $0 $cmd <value> [node]" >&2; exit 1; }
    require_number "$value" "$cmd"
    case "$cmd" in
      hp) param=current_hp ;;
      ammo) param=projectile_allowance_17mm ;;
      heat) param=shooter_17mm_1_barrel_heat ;;
      enemy) param=enemy_count ;;
    esac
    if set_param "$node" "$param" "$(param_type "$param")" "$value"; then
      echo "==> $node $param = $value"
    else
      echo "设置失败: $node $param = $value" >&2
      exit 1
    fi
    ;;

  hit)
    # 在现有血量上扣血，用来模拟受击
    damage=${1:-}
    node=${2:-$DEFAULT_NODE}
    [ -n "$damage" ] || { echo "用法: $0 hit <damage> [node]" >&2; exit 1; }
    require_number "$damage" "$cmd"
    current=$(param_value "$node" current_hp)
    if [ "$current" = "?" ]; then
      echo "读不到 current_hp，无法扣血：节点未启动或不可达。" >&2
      exit 1
    fi
    remaining=$((current - damage))
    [ "$remaining" -lt 0 ] && remaining=0
    if set_param "$node" current_hp integer "$remaining"; then
      echo "==> current_hp: $current - $damage = $remaining"
    else
      echo "设置失败: current_hp = $remaining" >&2
      exit 1
    fi
    ;;

  show)
    node=${1:-$DEFAULT_NODE}
    probe_node "$node" || exit 1
    echo "== 假裁判当前取值（$node）=="
    while read -r p; do
      printf "  %-34s %-10s %s\n" "$p" "$(param_value "$node" "$p")" "$(param_desc "$p")"
    done < <(param_names)
    ;;

  dump)
    node=${1:-$DEFAULT_NODE}
    probe_node "$node" || exit 1
    dump_params "$node"
    ;;

  save)
    # tune_fake_source.sh save <file> [node]
    file=${1:-}
    node=${2:-$DEFAULT_NODE}
    [ -n "$file" ] || { echo "用法: $0 save <file> [node]" >&2; exit 1; }
    probe_node "$node" || exit 1
    dump_params "$node" >"$file"
    echo "已保存到 $file（$(grep -c . "$file" 2>/dev/null || echo 0) 项）"
    ;;

  restore)
    # tune_fake_source.sh restore <file> [node]
    file=${1:-}
    node=${2:-$DEFAULT_NODE}
    [ -n "$file" ] || { echo "用法: $0 restore <file> [node]" >&2; exit 1; }
    [ -f "$file" ] || { echo "文件不存在: $file" >&2; exit 1; }
    count=0
    while IFS= read -r line; do
      [ -z "$line" ] && continue
      name=${line%%:*}
      value=${line#*: }
      if set_param "$node" "$name" "$(param_type "$name")" "$value"; then
        count=$((count + 1))
      else
        echo "  ⚠️ 设置失败: $name" >&2
      fi
    done <"$file"
    echo "已恢复 $count 项（$node）"
    ;;

  menu | interactive | "")
    interactive_menu "${1:-$DEFAULT_NODE}"
    ;;

  *)
    sed -n '4,27p' "$0"
    exit 1
    ;;
esac
