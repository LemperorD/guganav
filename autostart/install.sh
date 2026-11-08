#!/bin/bash

SERVICE_NAME="nav_auto_launch.service"
TARGET_PATH="/etc/systemd/system/$SERVICE_NAME"
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

echo "Installing $SERVICE_NAME ..."

# 必须 root 权限
if [ "$EUID" -ne 0 ]; then
  echo "Please run with sudo."
  exit 1
fi

# 检查脚本所在目录是否存在 service 文件（支持从任意目录执行）
if [ ! -f "$SCRIPT_DIR/$SERVICE_NAME" ]; then
  echo "$SERVICE_NAME not found in $SCRIPT_DIR."
  exit 1
fi

# 先停掉并 disable 旧实例：新 unit 的 WantedBy 从 multi-user.target 改成了
# graphical.target，不先 disable 的话 multi-user.target.wants 下会残留旧链接，
# 服务仍然在登录前被拉起（那时还没有 DISPLAY）。
systemctl stop "$SERVICE_NAME" >/dev/null 2>&1 || true
systemctl disable "$SERVICE_NAME" >/dev/null 2>&1 || true

# 复制（0644，root 所有）
install -m 0644 "$SCRIPT_DIR/$SERVICE_NAME" "$TARGET_PATH"

# 重新加载
systemctl daemon-reload

# 开机自启
systemctl enable "$SERVICE_NAME"

# 立即启动
systemctl start "$SERVICE_NAME"

echo "Installation complete."
systemctl status "$SERVICE_NAME" --no-pager || true
