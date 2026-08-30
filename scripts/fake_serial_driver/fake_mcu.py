#!/usr/bin/env python3
"""
假下位机：创建虚拟串口，向 serial_driver 持续发送 BR 协议帧。
用于无实车时测试 UI 标签更新，不修改任何项目代码。

详细说明见 scripts/fake_serial_test.md。

用法：
  1. 运行本脚本，记下输出的虚拟串口路径（如 /dev/pts/5）
  2. 另开终端：ros2 run serial_driver serial_driver_node_exe \
        --ros-args -p port_name:=/dev/pts/5
  3. 再开终端启动 UI，即可看到 Yaw 标签随本脚本输出的值变化
"""

import math
import os
import pty
import struct
import termios
import time

CMD_MOTION = 0xCD    # 运动控制帧（PC <-> MCU）
CMD_REFEREE = 0xD1   # 裁判系统帧（MCU -> PC）


def _crc8_table() -> list:
    """CRC-8/MAXIM 查表（反射多项式 0x8C），与 serial_driver 的查表一致。"""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
        table.append(crc & 0xFF)
    return table


_CRC8_TABLE = _crc8_table()


def crc8(data: bytes) -> int:
    """BR 协议 CRC8：初值 0xFF，逐字节查表。"""
    crc = 0xFF
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def frame(cmd: int, payload: bytes) -> bytes:
    head = bytes([0x42, 0x52, cmd, len(payload)])
    return head + payload + bytes([crc8(head + payload)])


def motion(yaw: float, ex: float = 0.0, ey: float = 0.0) -> bytes:
    """运动帧：payload[0:4]=yaw 角差(度), [4:8]=敌方x, [8:12]=敌方y。"""
    payload = struct.pack("<fff", yaw, ex, ey) + b"\x00" * 5
    return frame(CMD_MOTION, payload)


def referee(progress: int = 4) -> bytes:
    """裁判帧：高4位比赛阶段(4=比赛中)，带血量/弹丸/热量/RFID。"""
    payload = struct.pack("<BHHHHI", progress << 4, 300, 50, 10, 0, 0)
    return frame(CMD_REFEREE, payload)


def main() -> None:
    master, slave = pty.openpty()

    # 把从端设为 raw 模式，避免行规程吞掉二进制帧
    attrs = termios.tcgetattr(slave)
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                  termios.ISTRIP | termios.INLCR | termios.IGNCR |
                  termios.ICRNL | termios.IXON)
    attrs[1] &= ~termios.OPOST
    attrs[2] &= ~(termios.CSIZE | termios.PARENB)
    attrs[2] |= termios.CS8
    attrs[3] &= ~(termios.ECHONL | termios.ICANON | termios.ISIG |
                  termios.IEXTEN)
    termios.tcsetattr(slave, termios.TCSANOW, attrs)

    port = os.ttyname(slave)
    print(f"[fake_mcu] 虚拟串口: {port}", flush=True)
    print(f"[fake_mcu] serial_driver 启动参数: "
          f"--ros-args -p port_name:={port}", flush=True)

    t0 = time.time()
    last_referee = 0.0
    try:
        while True:
            t = time.time() - t0
            # 下位机上报单位为度；serial_driver 内部 /180*pi 转弧度，
            # 因此发送 0.8rad 的度数，UI 显示约 -0.8 ~ +0.8 rad
            yaw = math.degrees(0.8 * math.sin(0.5 * t))
            os.write(master, motion(yaw, ex=1.5, ey=-0.5))
            if t - last_referee >= 1.0:
                last_referee = t
                os.write(master, referee(progress=4))
            time.sleep(0.02)  # 50Hz
    except KeyboardInterrupt:
        pass
    finally:
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
