#!/usr/bin/env python3
"""
读取 guga_shm 的 YAW 槽位，打印 seq 与 yaw_diff，用于排查 UI 不更新。

判断方法：
  - seq 每轮递增、yaw_diff 在变 → 写端正常，问题在 UI 进程（未重启/旧二进制）；
  - seq 恒为 0 → serial_driver 没在写 YAW 槽（未编译新代码/未重启/串口没连）。

用法：
  python3 scripts/shm_yaw_probe.py [秒数]
"""

import mmap
import struct
import sys
import time

SHM = "/dev/shm/guga_shm"
YAW = 6
SLOT_META = 64 + YAW * 64       # ShmHeader 64B + slot 元数据表
DATA_OFF = 64 + 16 * 64 + YAW * 4224


def main() -> None:
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    with open(SHM, "rb") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        for _ in range(rounds):
            seq = struct.unpack_from("<Q", mm, SLOT_META + 16)[0]
            yaw_diff = struct.unpack_from("<d", mm, DATA_OFF)[0]
            print(f"YAW seq={seq} yaw_diff={yaw_diff:.4f}", flush=True)
            time.sleep(1)


if __name__ == "__main__":
    main()
