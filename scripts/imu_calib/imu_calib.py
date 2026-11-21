#!/usr/bin/env python3
"""IMU 六面标定求解工具的入口。

子命令:
    solve     由 CSV 求解零偏、标度因子与交叉耦合，输出 JSON 与文本报告
    apply     用标定结果修正一份 CSV，检查修正前后的模长误差
    selftest  用合成数据验证算法

采集由 scripts/imu_lidar 完成，本工具整条链路不依赖 ROS。
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile

import numpy as np

import calib_core as core


def _run_solve(csv_path, unit, model, out_path, gravity, verbose=True):
    stats = core.load_csv(csv_path)
    try:
        inferred, median = core.infer_unit(stats)
    except ValueError:
        inferred, median = None, float("nan")
    if unit == "auto":
        if inferred is None:
            raise ValueError(
                "无法判断加速度单位，请用 --unit g 或 --unit mps2 指定"
            )
        unit = inferred
        if verbose:
            print(f"加速度单位自动判断为 {unit}（六面模长中位数 {median:.4f}）")
    elif inferred is not None and unit != inferred and verbose:
        print(
            f"注意: 六面模长中位数 {median:.4f} 更接近 {inferred}，"
            f"但你指定的是 {unit}，请确认数据单位"
        )
    stats_si = core.to_si(stats, unit)

    solution = core.solve(stats_si, gravity=gravity, model=model)
    rows = core.evaluate(stats_si, solution["M"], solution["bias"], gravity=gravity)
    gyro = core.gyro_bias(stats_si)
    errors, warnings = core.check_quality(stats_si, rows, solution["M"])

    if verbose:
        print(core.format_report(stats_si, unit, solution, rows, gyro, gravity))
        for warning in warnings:
            print(f"注意: {warning}")
        for error in errors:
            print(f"不合格: {error}")

    payload = core.build_payload(
        stats_si, unit, solution, rows, gyro, errors, warnings,
        gravity=gravity, model=model,
    )
    core.save_payload(out_path, payload)
    if verbose:
        print(f"标定结果已写入 {out_path}")
    return payload, errors


def _cmd_solve(args):
    _, errors = _run_solve(
        args.csv, args.unit, args.model, args.out, args.gravity
    )
    return 2 if errors else 0


def _cmd_apply(args):
    M_inv, bias, unit = core.load_calibration(args.calib)
    rows = core.read_raw_csv(args.csv)
    corrected = core.apply_rows(rows, M_inv, bias, unit)
    fieldnames = list(rows[0].keys())
    for name in ("norm_before", "norm_after"):
        if name not in fieldnames:
            fieldnames.append(name)
    core.write_rows(args.out, corrected, fieldnames)

    to_si = core.G_STANDARD if unit == "g" else 1.0
    before = np.array([row["norm_before"] for row in corrected]) * to_si
    after = np.array([row["norm_after"] for row in corrected]) * to_si
    print(f"修正后数据写入 {args.out}，加速度单位保持为 {unit}")
    print(
        f"模长均值: 修正前 {before.mean():.4f}，修正后 {after.mean():.4f} m/s^2"
    )
    print(
        f"模长标准差: 修正前 {before.std():.4f}，修正后 {after.std():.4f} m/s^2"
    )
    if "face" in fieldnames:
        faces = sorted({row["face"] for row in corrected})
        for face in faces:
            subset = [row for row in corrected if row["face"] == face]
            face_before = np.mean([r["norm_before"] for r in subset]) * to_si
            face_after = np.mean([r["norm_after"] for r in subset]) * to_si
            print(
                f"  {face}: 修正前 {face_before:.4f}，"
                f"修正后 {face_after:.4f} m/s^2"
            )
    return 0


def _cmd_selftest(args):
    rows, truth = core.synthesize()
    if args.out:
        path = args.out
    else:
        path = os.path.join(tempfile.gettempdir(), "imu_six_face_synthetic.csv")
    core.write_rows(path, rows)
    print(f"合成数据写入 {path}")

    payload, errors = _run_solve(
        path, "auto", "full",
        os.path.join(tempfile.gettempdir(), "imu_calibration_synthetic.json"),
        core.G_STANDARD,
    )

    M = np.array(payload["accelerometer"]["M"])
    bias = np.array(payload["accelerometer"]["bias"])
    gyro = np.array(payload["gyroscope"]["bias"])
    M_error = float(np.max(np.abs(M - truth["M"])))
    bias_error = float(np.max(np.abs(bias - truth["bias"])))
    gyro_error = float(np.max(np.abs(gyro - truth["gyro_bias"])))

    print("与合成真值对比")
    print(f"  M 最大偏差 {M_error:.5f}（容差 0.02）")
    print(f"  零偏最大偏差 {bias_error:.4f} m/s^2（容差 0.02）")
    print(f"  陀螺零偏最大偏差 {gyro_error:.5f} rad/s（容差 0.002）")

    ok = (
        M_error <= 0.02
        and bias_error <= 0.02
        and gyro_error <= 0.002
        and not errors
    )
    if ok:
        print("自检通过")
        return 0
    print("自检未通过")
    return 1


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="IMU 六面标定：采集静态数据并求解零偏、标度因子与交叉耦合"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    solve = sub.add_parser("solve", help="由 CSV 求解标定参数")
    solve.add_argument("--csv", required=True, help="带 face 列的 CSV")
    solve.add_argument("--unit", choices=("auto", "g", "mps2"), default="auto",
                       help="加速度单位，默认按模长自动判断")
    solve.add_argument("--model", choices=("full", "diag"), default="full",
                       help="full 解 3x3 矩阵，diag 只解标度与零偏")
    solve.add_argument("--gravity", type=float, default=core.G_STANDARD,
                       help="当地重力加速度，单位 m/s^2")
    solve.add_argument("--out", default="imu_calibration.json",
                       help="输出 JSON 路径")
    solve.set_defaults(func=_cmd_solve)

    apply = sub.add_parser("apply", help="用标定结果修正数据")
    apply.add_argument("--csv", required=True, help="原始 CSV")
    apply.add_argument("--calib", required=True, help="solve 输出的 JSON")
    apply.add_argument("--out", default="imu_corrected.csv", help="输出 CSV")
    apply.set_defaults(func=_cmd_apply)

    selftest = sub.add_parser("selftest", help="用合成数据验证算法")
    selftest.add_argument("--out", default=None, help="合成 CSV 的保存路径")
    selftest.set_defaults(func=_cmd_selftest)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except KeyboardInterrupt:
        print("\n已中断，未写入标定结果。")
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
