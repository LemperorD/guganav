#!/usr/bin/env python3
"""用合成数据验证 imu_lidar 的六面采集与 imu_calib 的求解链路。

不需要 MID-360 硬件：imu_lidar --simulate 内部按已知的 M、b 生成 200 Hz 样本，
本脚本负责按提示切换六个朝向，然后调用求解器检查参数是否恢复。

运行: python3 scripts/imu_lidar/test_sim.py
"""

import csv
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BINARY = os.path.join(HERE, "imu_lidar")
CALIB = os.path.join(ROOT, "scripts", "imu_calib", "imu_calib.py")

# 与 imu_lidar.cpp 里 Simulator 的固定参数保持一致
TRUE_M = np.array(
    [
        [1.021, 0.006, -0.004],
        [0.003, 0.985, 0.005],
        [-0.005, 0.004, 1.013],
    ]
)
TRUE_BIAS = np.array([0.15, -0.21, 0.09])
TRUE_GYRO_BIAS = np.array([0.010, -0.020, 0.005])


def run():
    if not os.path.exists(BINARY):
        raise SystemExit(f"找不到 {BINARY}，先运行 scripts/imu_lidar/build.sh")

    output_dir = tempfile.mkdtemp(prefix="imu_lidar_test_")
    csv_path = os.path.join(output_dir, "six_face.csv")
    json_path = os.path.join(output_dir, "calib.json")

    print("运行六面采集（模拟数据）...")
    process = subprocess.run(
        [
            BINARY,
            "--simulate",
            "--six-face",
            "--duration", "2",
            "--out", csv_path,
        ],
        input="\n" * 6,
        capture_output=True,
        text=True,
        timeout=120,
    )
    if process.returncode != 0:
        print(process.stdout)
        print(process.stderr)
        print(f"imu_lidar 退出码 {process.returncode}")
        return 1

    with open(csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    faces = sorted({row["face"] for row in rows})
    print(f"CSV 共 {len(rows)} 行，朝向 {faces}")
    if set(faces) != {"+x", "-x", "+y", "-y", "+z", "-z"}:
        print("朝向不完整")
        return 1

    print("调用求解器...")
    result = subprocess.run(
        [sys.executable, CALIB, "solve", "--csv", csv_path, "--out", json_path],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr)
        print(f"求解器退出码 {result.returncode}")
        return 1

    with open(json_path, encoding="utf-8") as handle:
        payload = json.load(handle)
    M = np.array(payload["accelerometer"]["M"])
    bias = np.array(payload["accelerometer"]["bias"])
    gyro = np.array(payload["gyroscope"]["bias"])

    m_error = float(np.max(np.abs(M - TRUE_M)))
    bias_error = float(np.max(np.abs(bias - TRUE_BIAS)))
    gyro_error = float(np.max(np.abs(gyro - TRUE_GYRO_BIAS)))

    print("与合成真值对比")
    print(f"  M 最大偏差 {m_error:.5f}（容差 0.02）")
    print(f"  零偏最大偏差 {bias_error:.4f} m/s^2（容差 0.03）")
    print(f"  陀螺零偏最大偏差 {gyro_error:.5f} rad/s（容差 0.003）")

    ok = m_error <= 0.02 and bias_error <= 0.03 and gyro_error <= 0.003
    print("测试通过" if ok else "测试未通过")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(run())
