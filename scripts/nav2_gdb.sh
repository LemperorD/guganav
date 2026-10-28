#!/usr/bin/env bash
# Capture the original SIGSEGV before Magick turns it into SIGABRT.
set -euo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo "用法: scripts/nav2_gdb.sh [容器PID]"
  echo "正常启动导航后执行；自动捕获崩溃栈到 /tmp/nav2_gdb_时间_PID.txt。"
  exit 0
fi
if (( $# > 1 )) || [[ -n ${1:-} && ! ${1:-} =~ ^[0-9]+$ ]]; then
  echo "参数必须是一个容器 PID，或不传参数自动查找。" >&2
  exit 2
fi
command -v gdb >/dev/null || { echo "请先安装: sudo apt install gdb" >&2; exit 1; }
if (( EUID != 0 )); then
  exec sudo -- bash "$(readlink -f "${BASH_SOURCE[0]}")" "$@"
fi

if [[ -n ${1:-} ]]; then
  nav2_pid=$1
else
  mapfile -t nav2_pids < <(pgrep -f '^/[^ ]*/component_container_isolated .*__node:=nav2_container' || true)
  if (( ${#nav2_pids[@]} != 1 )); then
    echo "找到 ${#nav2_pids[@]} 个 Nav2 容器。请先启动导航；多个容器时手动指定 PID：" >&2
    pgrep -af '^/[^ ]*/component_container_isolated' >&2 || true
    exit 1
  fi
  nav2_pid=${nav2_pids[0]}
fi
if [[ ! -r /proc/$nav2_pid/cmdline ]] || ! tr '\0' '\n' < "/proc/$nav2_pid/cmdline" | grep -qx '__node:=nav2_container'; then
  echo "PID $nav2_pid 不是 nav2_container，或已退出。" >&2
  exit 1
fi

nav2_log=$(mktemp "/tmp/nav2_gdb_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_${nav2_pid}_XXXXXX.txt")
chmod 644 "$nav2_log"
nav2_commands=$(mktemp /tmp/nav2_gdb_commands_XXXXXX)
trap 'rm -f "$nav2_commands"' EXIT
cat > "$nav2_commands" <<EOF
set pagination off
set confirm off
set print frame-arguments all
set logging file $nav2_log
set logging overwrite on
set logging redirect off
set logging on
handle SIGSEGV stop print nopass
handle SIGABRT stop print nopass
continue
echo \n========== STOP LOCATION ==========\n
info program
bt full
echo \n========== ALL THREADS ==========\n
thread apply all bt full
set logging off
echo \n调用栈已保存。现场保持暂停；输入 quit 退出调试并脱离进程。\n
EOF
echo "附加 Nav2 PID: $nav2_pid"
echo "日志: $nav2_log"
echo "附加完成后在 RViz 复现问题；捕获后自动保存栈并停在 GDB。"
echo "实车请架空轮子，调试暂停时导航也会暂停。"
gdb -q -p "$nav2_pid" -x "$nav2_commands"
