#!/usr/bin/env bash
set -euo pipefail

# ────────────────────────────────────────────────────────────────
# Controller 动态调参工具（运行时调整，无需重启节点）
#
# Controller 的所有参数（critics 权重/速度限制/采样参数）注册为 Dynamic 类型，
# 通过 ros2 param set 即时生效（parameters_handler 的 on_set 回调）。
#
# 用法：
#   scripts/tune_controller.sh                  # 交互菜单（列表选择参数 → 输入新值）
#   scripts/tune_controller.sh set <param> <value> [node]   # 命令行改单个参数
#   scripts/tune_controller.sh show [node]                  # 显示当前关键参数
#   scripts/tune_controller.sh dump [node]                  # 导出全部 Controller 参数
#   scripts/tune_controller.sh save <file> [node]           # 保存当前参数到文件
#   scripts/tune_controller.sh restore <file> [node]        # 从文件恢复
#
# 参数名省略 FollowPath. 前缀（自动补全）；节点名默认
# /red_standard_robot1/controller_server，可传参数覆盖。
# ────────────────────────────────────────────────────────────────

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    source "$setup_file"
    set -u
  fi
}

if [[ -z "${ROS_DISTRO:-}" ]]; then
  source_setup /opt/ros/humble/setup.bash
fi
source_setup "$WS/install/setup.bash"

DEFAULT_NODE="/red_standard_robot1/controller_server"
# 常用关键参数（show 时展示）
echo "默认节点: $DEFAULT_NODE"
set +e
CONTROLLER_TYPE=$(ros2 param get "$DEFAULT_NODE" "FollowPath.plugin" 2>/dev/null |
 awk '{print tolower($4)}' )
set -e
if [[ -z "$CONTROLLER_TYPE" ]]; then
  echo "⚠️ 未获取到 FollowPath.controller 参数，可能节点未启动或参数未声明"
  CONTROLLER_TYPE="unknown"
elif [[ "$CONTROLLER_TYPE" = *"mppi"* ]]; then
  	echo "控制器类型:MPPI"
  	CONTROLLER_TYPE='mppi'
elif [[ "$CONTROLLER_TYPE" = *"mpc"* ]]; then
	echo "控制器类型:MPC"
	CONTROLLER_TYPE='mpc'
elif [[ "$CONTROLLER_TYPE" = *"pid"* ]]; then
	echo "控制器类型:PID"
	CONTROLLER_TYPE='pid'
else
	echo "非法的控制器类型⚠️"
 	CONTROLLER_TYPE="unknown"
	exit 1;
fi
read -p "按任意键继续"

CONFIG_FILE="${WS}/scripts/params_list/${CONTROLLER_TYPE}_para.txt"
KEY_PARAMS=()
PARAM_LIST=()
if [[ ! -f "$CONFIG_FILE" ]]; then
	echo "ERROR:config file $CONFIG_FILE is invalid"
	exit 1;
fi

while read -r line; do 
   line=${line%$'\r'}
   [[ -z "$line" ]] && continue
   KEY_PARAMS+=("$line")
   PARAM_LIST+=("$line")
 done< $CONFIG_FILE

# 显示当前值（逐行实时获取，带 timeout 防节点忙时挂死）
param_value() {
  local node=$1
  local param=$2
  local output

  if ! output=$(timeout 3 ros2 param get "$node" "$param" 2>&1); then
    echo "获取失败 [$param#FollowPath.]:" >&2
    echo "$output" >&2
    printf '?\n'
    return 0
  fi

  output=$(printf '%s\n' "$output" |
    tail -n 1 |
    sed 's/^.*is: //')

  if [ -z "$output" ]; then
    printf '?\n'
  else
    printf '%s\n' "$output"
  fi
}

# ── 交互菜单：列出参数 → 选择 → 输入新值 → 应用 ──
interactive_menu() {
  local node=${1:-$DEFAULT_NODE}
  local choice value full current
  local -A values=()
  local -a names=()

  # ── 固定参数表（PARAM_LIST）──
  local -a names=()
  for entry in "${PARAM_LIST[@]}"; do
    names+=("${entry%%|*}")
  done

  # ── 首次进入：边获取边逐行打印（一行一行输出）──
  clear 2>/dev/null || true   # 非 tty/TERM=dumb 时 clear 失败不影响
  echo "== 正在读取参数（${#names[@]} 项）... =="
  local i=1
  for p in "${names[@]}"; do
    values[$p]=$(param_value "$node" "$p")
    p_print="${p#FollowPath.}"   # 显示时去掉前缀
    printf "  %2d) %-38s [%s]\n" "$i" "$p_print" "${values[$p]:-?}"
    i=$((i+1))
  done
  # changed[p]=1 标记本参数刚被更改（下次循环重取确认）；取到值后清除标记
  local -A changed=()

  while true; do
    clear 2>/dev/null || true   # 每次输出前清除上一次的输出
    # ── 每次循环只重取两类：上次为 ? 的（重试） + 上次更改过的（确认生效值）──
    local -a refetch=()
    for p in "${names[@]}"; do
      if [ "${values[$p]:-?}" = "?" ] || [ "${changed[$p]:-0}" = "1" ]; then
        refetch+=("$p")
      fi
    done
    if [ "${#refetch[@]}" -gt 0 ]; then
      echo "== 重新获取 ${#refetch[@]} 项（? 重试 + 已更改确认）... =="
      for p in "${refetch[@]}"; do
        local newval
        newval=$(param_value "$node" "$p")
        values[$p]=$newval
        [ "$newval" != "?" ] && changed[$p]=0   # 取到真实值后不再重试
      done
    fi

    echo
    echo "== ${CONTROLLER_TYPE} 动态调参（$node,${#names[@]} 项）=="
    echo "   (? = 获取超时或参数未声明；本循环只重取 ? 项与已更改项),  * = 上次更改失败"
    local i=1
    for p in "${names[@]}"; do
      local mark=""
      p_print="${p#FollowPath.}"   # 显示时去掉前缀
      [ "${changed[$p]:-0}" = "1" ] && mark=" *"   # * = 上次更改，本循环已重取确认
      printf "  %2d) %-38s [%s]%s\n" "$i" "${p_print}" "${values[$p]:-?}" "$mark"
      i=$((i+1))
    done
    echo "  0) 退出"
    printf "选择 [0-%d]: " "${#names[@]}"
    read -r choice || break
    [ -z "$choice" ] && continue
    if [ "$choice" = "0" ]; then
      echo "再见了，${USER}小弟🐧🐧🐧"
      break
    fi
    if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#names[@]}" ]; then
      echo "无效选择: $choice" >&2
      continue
    fi
    local idx=$((choice-1))
    local param=${names[$idx]}
    full="$param"
    current=${values[$param]:-?}
    printf "  %s 当前值: %s\n  新值: " "$param" "$current"
    read -r value || break
    [ -z "$value" ] && continue
    echo "==> $node $full = $value"
    ros2 param set "$node" "$full" "$value"
    values[$param]=$value    # 先显示输入值
    changed[$param]=1        # 标记：下次循环重取确认实际生效值
  done
}

cmd=${1:-}
shift || true

case "$cmd" in
  set)
    # tune_controller.sh set <param> <value> [node]
    param=$1; value=$2; node=${3:-$DEFAULT_NODE}
    case "$param" in
      FollowPath.*) full=$param ;;
      *) full="FollowPath.$param" ;;
    esac
    echo "==> $node $full = $value"
    ros2 param set "$node" "$full" "$value"
    ;;
  show)
    node=${1:-$DEFAULT_NODE}
    echo "== 当前 ${CONTROLLER_TYPE} 关键参数（$node）=="
    for p in "${KEY_PARAMS[@]}"; do
      v=$(ros2 param get "$node" "$p" 2>/dev/null | tail -1 | sed 's/^.*is: //')
      printf "  %-45s %s\n" "$p" "${v:-<未设置>}"
    done
    ;;
  dump)
    node=${1:-$DEFAULT_NODE}
    ros2 param dump "$node" 2>/dev/null | grep -E "FollowPath\." || true
    ;;
  save)
    # tune_controller.sh save <file> [node]
    file=$1; node=${2:-$DEFAULT_NODE}
    ros2 param dump "$node" 2>/dev/null | grep -E "FollowPath\." > "$file" || true
    echo "已保存 ${CONTROLLER_TYPE} 参数到 $file($(grep -c . "$file" || echo 0) 项）"
    ;;
  restore)
    # tune_controller.sh restore <file> [node]
    file=$1; node=${2:-$DEFAULT_NODE}
    [ -f "$file" ] || { echo "文件不存在: $file" >&2; exit 1; }
    count=0
    while IFS= read -r line; do
      # ros2 param dump 格式: name: value
      name=${line%%:*}
      value=${line#*: }
      case "$name" in
        /red_standard_robot1/*) name=${name#/red_standard_robot1/} ;;
      esac
      ros2 param set "$node" "$name" "$value" >/dev/null 2>&1 && count=$((count+1)) || \
        echo "  ⚠️ 设置失败: $name" >&2
    done < "$file"
    echo "已恢复 $count 项参数（$node)"
    ;;
  menu | interactive | "")
    interactive_menu "${1:-$DEFAULT_NODE}"
    ;;
  *)
    sed -n '3,24p' "$0"
    exit 1
    ;;
esac
