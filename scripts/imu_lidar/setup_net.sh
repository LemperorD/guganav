#!/usr/bin/env bash
# 给连接 MID-360 的网口配置静态地址，并检查能否与雷达通信。
#
# 用法:
#   sudo scripts/imu_lidar/setup_net.sh                 # 自动选网口，配置并测试
#   sudo scripts/imu_lidar/setup_net.sh --iface enp108s0
#   sudo scripts/imu_lidar/setup_net.sh --remove        # 移除本脚本添加的配置
#
# 配置方式优先使用 NetworkManager（重启后仍然有效），没有 nmcli 时退回 ip 命令
# （只在本次开机内有效）。
#
# 默认值与 src/guga_bringup/config/reality/mid360_user_config.json 保持一致：
# 主机 192.168.11.50/24，雷达 192.168.11.3。

set -euo pipefail

IFACE=""
HOST_IP="192.168.11.50"
PREFIX=24
LIDAR_IP="192.168.11.3"
CONNECTION_NAME="mid360"
REMOVE=0

usage() {
  awk 'NR == 1 { next }
       /^#/ { sub(/^# ?/, ""); print; next }
       { exit }' "$0"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface)
      IFACE="${2:?--iface 需要参数}"
      shift 2
      ;;
    --host)
      HOST_IP="${2:?--host 需要参数}"
      shift 2
      ;;
    --lidar)
      LIDAR_IP="${2:?--lidar 需要参数}"
      shift 2
      ;;
    --connection)
      CONNECTION_NAME="${2:?--connection 需要参数}"
      shift 2
      ;;
    --remove)
      REMOVE=1
      shift
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ $EUID -ne 0 ]]; then
  echo "这个脚本要改网口配置，请用 sudo 运行。" >&2
  exit 1
fi

if [[ -z "$IFACE" ]]; then
  for dev in /sys/class/net/*; do
    name="$(basename "$dev")"
    case "$name" in
      lo | wl* | docker* | veth* | br-*) continue ;;
    esac
    if [[ -f "$dev/carrier" ]] && [[ "$(cat "$dev/carrier" 2>/dev/null)" == "1" ]]; then
      IFACE="$name"
      break
    fi
  done
fi

if [[ -z "$IFACE" ]]; then
  echo "没有找到有载波的有线网口。" >&2
  echo "检查网线是否插在雷达与电脑之间，以及雷达是否上电。" >&2
  exit 1
fi

if ! [[ -d "/sys/class/net/$IFACE" ]]; then
  echo "网口不存在: $IFACE" >&2
  exit 1
fi

if [[ "$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null)" != "1" ]]; then
  echo "网口 $IFACE 没有载波，检查网线与雷达电源。" >&2
  exit 1
fi

if [[ "$REMOVE" == "1" ]]; then
  if command -v nmcli >/dev/null 2>&1 \
      && nmcli -t -f NAME connection show | grep -qx "$CONNECTION_NAME"; then
    nmcli connection delete "$CONNECTION_NAME"
    echo "已删除 NetworkManager 连接 $CONNECTION_NAME"
  else
    ip addr del "$HOST_IP/$PREFIX" dev "$IFACE" 2>/dev/null || true
    echo "已从 $IFACE 移除 $HOST_IP/$PREFIX"
  fi
  exit 0
fi

echo "网口: $IFACE"
echo "主机地址: $HOST_IP/$PREFIX"
echo "雷达地址: $LIDAR_IP"

if command -v nmcli >/dev/null 2>&1 \
    && nmcli -g DEVICE device status 2>/dev/null | grep -qx "$IFACE"; then
  # NetworkManager 管理这个网口时用连接配置，重启后仍然有效。
  if nmcli -t -f NAME connection show | grep -qx "$CONNECTION_NAME"; then
    nmcli connection modify "$CONNECTION_NAME" \
      ipv4.method manual \
      ipv4.addresses "$HOST_IP/$PREFIX" \
      ipv4.never-default yes \
      connection.autoconnect yes \
      connection.autoconnect-priority 10
  else
    nmcli connection add type ethernet ifname "$IFACE" con-name "$CONNECTION_NAME" \
      ipv4.method manual \
      ipv4.addresses "$HOST_IP/$PREFIX" \
      ipv4.never-default yes \
      connection.autoconnect yes \
      connection.autoconnect-priority 10 >/dev/null
  fi
  if ! nmcli connection up "$CONNECTION_NAME" >/dev/null 2>&1; then
    # 设备可能正处在自动连接的激活过程中，先断开再激活
    nmcli device disconnect "$IFACE" >/dev/null 2>&1 || true
    nmcli connection up "$CONNECTION_NAME" >/dev/null
  fi
  echo "已通过 NetworkManager 配置连接 $CONNECTION_NAME（重启后保留）"

  # NetworkManager 激活是异步的，等地址真正出现在网口上
  for _ in $(seq 1 20); do
    if ip -4 addr show dev "$IFACE" | grep -q "inet $HOST_IP/"; then
      break
    fi
    sleep 0.25
  done
  if ! ip -4 addr show dev "$IFACE" | grep -q "inet $HOST_IP/"; then
    echo "连接已配置，但网口上没有出现 $HOST_IP，激活可能失败:" >&2
    nmcli -f GENERAL.STATE,IP4.ADDRESS connection show "$CONNECTION_NAME" >&2 || true
    exit 1
  fi
else
  ip link set "$IFACE" up
  if ip -4 addr show dev "$IFACE" | grep -q "inet $HOST_IP/"; then
    echo "地址已存在，跳过添加"
  else
    ip addr add "$HOST_IP/$PREFIX" dev "$IFACE"
    echo "已添加地址（只在本次开机内有效）"
  fi
  if command -v nmcli >/dev/null 2>&1 \
      && nmcli -g DEVICE device status 2>/dev/null | grep -qx "$IFACE"; then
    echo "注意: NetworkManager 正在管理 $IFACE，DHCP 重试时可能清掉这个地址。"
    echo "若地址消失，先执行 nmcli device set $IFACE managed no 再重试。"
  fi
fi

echo ""
echo "测试雷达 $LIDAR_IP ..."
if ping -c 3 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
  echo "雷达可达，可以启动读取程序:"
  echo "  scripts/imu_lidar/imu_lidar"
  exit 0
fi

echo "雷达 $LIDAR_IP 不可达。扫描本网段，看看到底哪个地址有响应："
SUBNET="${HOST_IP%.*}"
for i in $(seq 1 254); do
  ping -c 1 -W 1 "$SUBNET.$i" >/dev/null 2>&1 &
done
wait
FOUND="$(ip neigh show dev "$IFACE" | grep -v FAILED || true)"
if [[ -n "$FOUND" ]]; then
  echo "$FOUND"
  echo ""
  echo "若上面的地址不是 $LIDAR_IP，说明雷达 IP 与配置文件不一致。"
  echo "用 Livox Viewer 把雷达改成 $LIDAR_IP，或者把配置文件里的地址改成实际地址。"
else
  echo "本网段没有任何响应。可能的原因:"
  echo "  1. 雷达没有上电，或网线没有接到 $IFACE；"
  echo "  2. 雷达的 IP 不在 $SUBNET.0/24 网段，需要先用 Livox Viewer 查看。"
fi
exit 1
