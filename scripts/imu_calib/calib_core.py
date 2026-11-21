"""IMU 六面标定核心算法。

本模块只依赖 numpy 与 Python 标准库，不依赖 ROS，便于离线分析和单元测试。

测量模型
--------
加速度计:

    a_meas = M @ a_true + b

其中 a_meas 是传感器读数（m/s^2），a_true 是真实比力（m/s^2），M 是 3x3 的
标度与交叉耦合矩阵，b 是零偏（m/s^2）。

六面静态采集时，每一面让一个坐标轴竖直朝上（或朝下），此时

    a_true = sign * g * e_axis

g 取 9.80665 m/s^2，sign 为 +1 表示该轴朝上，-1 表示该轴朝下。六个位置给出
18 个方程，M 与 b 共 12 个未知量，按行解最小二乘即可。

陀螺仪在六面静态法里只能标定零偏：
陀螺零偏 = 六个位置静止读数的加权平均（rad/s）。角速度的标度因数需要转台或
已知角速度激励，六面静态法无法给出。
"""

from __future__ import annotations

import csv
import json
import math

import numpy as np

G_STANDARD = 9.80665

FACES = ("+x", "-x", "+y", "-y", "+z", "-z")

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
FACE_AXIS = {face: _AXIS_INDEX[face[1]] for face in FACES}
FACE_SIGN = {face: (1.0 if face[0] == "+" else -1.0) for face in FACES}

_CSV_FIELDS = ("t", "ax", "ay", "az", "wx", "wy", "wz", "face")


class FaceStats:
    """单个朝向的静止统计量。"""

    __slots__ = (
        "face",
        "samples",
        "duration",
        "acc_mean",
        "acc_std",
        "gyro_mean",
        "gyro_std",
        "rejected",
    )

    def __init__(self, face, samples, duration, acc_mean, acc_std,
                 gyro_mean, gyro_std, rejected):
        self.face = face
        self.samples = samples
        self.duration = duration
        self.acc_mean = np.asarray(acc_mean, dtype=float)
        self.acc_std = np.asarray(acc_std, dtype=float)
        self.gyro_mean = np.asarray(gyro_mean, dtype=float)
        self.gyro_std = np.asarray(gyro_std, dtype=float)
        self.rejected = float(rejected)

    @property
    def rate(self):
        if not np.isfinite(self.duration) or self.duration <= 0.0:
            return float("nan")
        return self.samples / self.duration

    def scaled(self, factor):
        return FaceStats(
            self.face,
            self.samples,
            self.duration,
            self.acc_mean * factor,
            self.acc_std * factor,
            self.gyro_mean,
            self.gyro_std,
            self.rejected,
        )


def robust_mean(samples, k=3.0, iterations=5):
    """按 3 sigma 迭代剔除离群点后求均值，返回 (均值, 标准差, 剔除比例)。

    静止数据里偶尔会有碰撞或振动造成的尖峰，直接求均值会被拉偏，因此先剔除
    偏离均值超过 k 倍标准差的样本。保留不足一成样本时停止剔除，避免把真实
    漂移当成离群点删掉。
    """
    x = np.asarray(samples, dtype=float)
    if x.ndim != 2 or x.shape[0] == 0:
        raise ValueError("样本数组必须是非空的二维数组")

    keep = np.ones(x.shape[0], dtype=bool)
    for _ in range(iterations):
        sub = x[keep]
        mean = sub.mean(axis=0)
        std = np.maximum(sub.std(axis=0), 1e-9)
        new_keep = np.all(np.abs(x - mean) <= k * std, axis=1)
        if new_keep.sum() < max(10, x.shape[0] // 10):
            break
        if np.array_equal(new_keep, keep):
            break
        keep = new_keep

    sub = x[keep]
    return sub.mean(axis=0), sub.std(axis=0), 1.0 - keep.mean()


def load_csv(path):
    """读取带 face 列的 CSV，返回按 FACES 顺序排列的 FaceStats 列表。"""
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)
        fields = reader.fieldnames or []

    missing = [name for name in _CSV_FIELDS if name != "t" and name not in fields]
    if missing:
        raise ValueError(f"{path} 缺少列: {', '.join(missing)}")
    has_time = "t" in fields

    grouped = {face: [] for face in FACES}
    unknown = set()
    for row in rows:
        face = (row.get("face") or "").strip().lower()
        if not face:
            continue
        if face not in grouped:
            unknown.add(face)
            continue
        try:
            acc = [float(row["ax"]), float(row["ay"]), float(row["az"])]
            gyro = [float(row["wx"]), float(row["wy"]), float(row["wz"])]
        except (TypeError, ValueError):
            continue
        stamp = float("nan")
        if has_time and row.get("t"):
            try:
                stamp = float(row["t"])
            except ValueError:
                stamp = float("nan")
        grouped[face].append((stamp, acc, gyro))

    if unknown:
        raise ValueError(f"未知的 face 取值: {', '.join(sorted(unknown))}")

    stats = []
    for face in FACES:
        items = grouped[face]
        if len(items) < 2:
            raise ValueError(f"缺少 {face} 面的数据，只读到 {len(items)} 个样本")
        acc = np.array([item[1] for item in items], dtype=float)
        gyro = np.array([item[2] for item in items], dtype=float)
        stamps = np.array([item[0] for item in items], dtype=float)

        acc_mean, acc_std, rejected = robust_mean(acc)
        gyro_mean, gyro_std, _ = robust_mean(gyro)

        if np.isfinite(stamps).all():
            duration = float(stamps.max() - stamps.min())
        else:
            duration = float("nan")

        stats.append(
            FaceStats(face, len(items), duration, acc_mean, acc_std,
                      gyro_mean, gyro_std, rejected)
        )
    return stats


def infer_unit(stats):
    """根据六面读数的平均模长判断加速度单位，返回 (单位, 模长中位数)。

    Livox 驱动原样转发 SDK 的加速度，单位是 g，不是 m/s^2。六面标定里每个面
    的模长都接近 1 g，因此用中位数判断最稳妥。
    """
    norms = [float(np.linalg.norm(s.acc_mean)) for s in stats]
    median = float(np.median(norms))
    if 0.5 <= median <= 2.5:
        return "g", median
    if 5.0 <= median <= 15.0:
        return "mps2", median
    raise ValueError(
        f"无法判断加速度单位: 六面读数模长的中位数是 {median:.3f}，"
        "既不像 1 g 也不像 9.8 m/s^2，请检查数据或显式指定 --unit"
    )


def to_si(stats, unit):
    """把加速度统一换算成 m/s^2。单位是 g 时乘以标准重力。"""
    factors = {"g": G_STANDARD, "mps2": 1.0}
    if unit not in factors:
        raise ValueError(f"未知单位: {unit}")
    factor = factors[unit]
    if factor == 1.0:
        return list(stats)
    return [s.scaled(factor) for s in stats]


def solve_means(means, gravity=G_STANDARD, model="full"):
    """由六个面的加速度均值求解 M 与 b。

    means: dict, 键为 FACES 中的六个朝向，值为该朝向的加速度均值 (m/s^2)。
    model: "full" 解 3x3 矩阵与零偏，"diag" 只解三个标度因子与零偏。

    返回 {"M": 3x3 ndarray, "bias": 3 维 ndarray}。
    """
    missing = [face for face in FACES if face not in means]
    if missing:
        raise ValueError(f"缺少朝向: {', '.join(missing)}")

    if model == "diag":
        M = np.zeros((3, 3))
        bias = np.zeros(3)
        for axis, name in enumerate("xyz"):
            up = np.asarray(means["+" + name], dtype=float)
            down = np.asarray(means["-" + name], dtype=float)
            M[axis, axis] = (up[axis] - down[axis]) / (2.0 * gravity)
            bias[axis] = (up[axis] + down[axis]) / 2.0
        return {"M": M, "bias": bias}

    if model != "full":
        raise ValueError(f"未知模型: {model}")

    M = np.zeros((3, 3))
    bias = np.zeros(3)
    for row in range(3):
        A = np.zeros((6, 4))
        y = np.zeros(6)
        for index, face in enumerate(FACES):
            axis = FACE_AXIS[face]
            A[index, axis] = FACE_SIGN[face] * gravity
            A[index, 3] = 1.0
            y[index] = means[face][row]
        solution, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
        M[row] = solution[:3]
        bias[row] = solution[3]
    return {"M": M, "bias": bias}


def solve(stats, gravity=G_STANDARD, model="full"):
    means = {s.face: s.acc_mean for s in stats}
    return solve_means(means, gravity=gravity, model=model)


def evaluate(stats, M, bias, gravity=G_STANDARD):
    """逐面计算校准前后的残差，返回每面的明细字典列表。"""
    rows = []
    for s in stats:
        axis = FACE_AXIS[s.face]
        sign = FACE_SIGN[s.face]

        true_acc = np.zeros(3)
        true_acc[axis] = sign * gravity

        predicted = M @ true_acc + bias
        residual = s.acc_mean - predicted
        corrected = np.linalg.solve(M, s.acc_mean - bias)

        corrected_norm = float(np.linalg.norm(corrected))
        if corrected_norm > 0.0:
            cosine = float(np.dot(corrected, true_acc)) / (corrected_norm * gravity)
            angle = math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))
        else:
            angle = float("nan")

        axis_read = float(s.acc_mean[axis])
        rows.append(
            {
                "face": s.face,
                "measured": s.acc_mean.copy(),
                "predicted": predicted,
                "residual": residual,
                "residual_norm": float(np.linalg.norm(residual)),
                "corrected": corrected,
                "norm_error": corrected_norm - gravity,
                "angle_deg": angle,
                "raw_norm_error": float(np.linalg.norm(s.acc_mean) - gravity),
                "axis_read": axis_read,
                "sign_ok": bool(axis_read * sign > 0.3 * gravity),
            }
        )
    return rows


def gyro_bias(stats):
    """六个位置静止读数的加权平均，作为陀螺零偏 (rad/s)。"""
    weights = np.array([s.samples for s in stats], dtype=float)
    means = np.array([s.gyro_mean for s in stats], dtype=float)
    return (means * weights[:, None]).sum(axis=0) / weights.sum()


def check_quality(stats, rows, M):
    """检查采集与拟合质量，返回 (errors, warnings) 两个字符串列表。

    errors 表示这组数据不足以给出可信结果，调用方应让程序以非零状态退出；
    warnings 表示结果可用，但某些环节需要人工确认。
    """
    errors = []
    warnings = []

    for s in stats:
        if s.samples < 200:
            errors.append(
                f"{s.face} 面只有 {s.samples} 个样本，至少需要 200 个"
            )
        if np.isfinite(s.duration) and s.duration < 1.0:
            errors.append(
                f"{s.face} 面采样时长只有 {s.duration:.2f} s，至少需要 1 s"
            )
        max_gyro_std = float(np.max(s.gyro_std))
        if max_gyro_std > 0.02:
            warnings.append(
                f"{s.face} 面角速度标准差 {max_gyro_std:.4f} rad/s 偏大，"
                "采集期间可能没有完全静止"
            )
        if s.rejected > 0.05:
            warnings.append(
                f"{s.face} 面剔除了 {s.rejected * 100.0:.1f}% 的样本，"
                "可能存在振动或碰撞"
            )

    for row in rows:
        if not row["sign_ok"]:
            errors.append(
                f"{row['face']} 面主轴读数为 {row['axis_read']:.3f} m/s^2，"
                "符号与预期不符，检查朝向是否放反或轴向定义是否一致"
            )

    scale = np.diag(M)
    for axis, name in enumerate("xyz"):
        if abs(scale[axis] - 1.0) > 0.05:
            warnings.append(
                f"{name} 轴标度因子 {scale[axis]:.4f} 偏离 1 超过 5%，"
                "检查数据或确认这颗 IMU 的标度误差确实较大"
            )

    max_norm_error = max(abs(row["norm_error"]) for row in rows)
    if max_norm_error > 0.3:
        errors.append(
            f"校准后最大模长误差 {max_norm_error:.3f} m/s^2 超过 0.3，"
            "数据可能不满足静态假设"
        )
    elif max_norm_error > 0.1:
        warnings.append(
            f"校准后最大模长误差 {max_norm_error:.3f} m/s^2 超过 0.1，"
            "检查安装面是否水平"
        )

    return errors, warnings


def format_report(stats, unit, solution, rows, gyro, gravity=G_STANDARD):
    """把标定结果整理成可直接打印的文本。"""
    M = solution["M"]
    bias = solution["bias"]
    total = sum(s.samples for s in stats)

    lines = []
    lines.append("")
    lines.append("六面标定结果")
    lines.append("=" * 60)
    if unit == "g":
        lines.append(f"输入 {total} 个样本，加速度原始单位 g，已换算为 m/s^2")
    else:
        lines.append(f"输入 {total} 个样本，加速度原始单位 m/s^2")
    lines.append("")

    lines.append("采集质量")
    for s in stats:
        duration = (
            f"{s.duration:.2f} s" if np.isfinite(s.duration) else "时长未知"
        )
        rate = (
            f"{s.rate:.1f} Hz" if np.isfinite(s.rate) else "采样率未知"
        )
        lines.append(
            f"  {s.face:<3} {s.samples:>6} 点  {duration}  {rate}  "
            f"角速度标准差 {np.max(s.gyro_std):.4f} rad/s  "
            f"剔除 {s.rejected * 100.0:.1f}%"
        )
    lines.append("")

    lines.append("加速度计参数")
    lines.append(
        "零偏 b = "
        f"[{bias[0]: .4f}, {bias[1]: .4f}, {bias[2]: .4f}] m/s^2"
    )
    scale = np.diag(M)
    lines.append(
        "标度因子 = "
        f"[{scale[0]:.4f}, {scale[1]:.4f}, {scale[2]:.4f}]"
    )
    lines.append(
        "标度误差 = "
        f"[{(scale[0] - 1.0) * 100.0: .2f}, "
        f"{(scale[1] - 1.0) * 100.0: .2f}, "
        f"{(scale[2] - 1.0) * 100.0: .2f}] %"
    )
    lines.append("交叉耦合矩阵 M（行是读数轴，列是真实轴）:")
    for row in M:
        lines.append(
            f"  [{row[0]: .5f} {row[1]: .5f} {row[2]: .5f}]"
        )
    lines.append("")

    lines.append("陀螺仪零偏")
    lines.append(
        f"b_g = [{gyro[0]: .5f}, {gyro[1]: .5f}, {gyro[2]: .5f}] rad/s"
    )
    lines.append("六面静态法只能给出零偏，角速度标度因数需要转台或已知角速度。")
    lines.append("")

    lines.append("逐面残差")
    lines.append("校准前误差与校准后误差都是模长与 g 的差，单位 m/s^2")
    for row in rows:
        lines.append(
            f"  {row['face']:<3} 主轴读数 {row['axis_read']: .4f}  "
            f"校准前 {row['raw_norm_error']: .4f}  "
            f"校准后 {row['norm_error']: .4f}  "
            f"与理想轴夹角 {row['angle_deg']:.2f}°"
        )
    lines.append("")

    before = max(abs(row["raw_norm_error"]) for row in rows)
    after = max(abs(row["norm_error"]) for row in rows)
    lines.append(
        f"模长误差最大值: 校准前 {before:.4f} m/s^2，校准后 {after:.4f} m/s^2"
    )
    lines.append(
        "标定后把读数按 a_true = M^-1 (a_meas - b) 修正，再送入定位算法。"
    )
    lines.append("")
    return "\n".join(lines)


def build_payload(stats, unit, solution, rows, gyro, errors, warnings,
                  gravity=G_STANDARD, model="full"):
    M = solution["M"]
    bias = solution["bias"]
    return {
        "model": "a_meas = M @ a_true + bias",
        "gravity": gravity,
        "input_acc_unit": unit,
        "fit_model": model,
        "accelerometer": {
            "M": M.tolist(),
            "bias": bias.tolist(),
            "scale": np.diag(M).tolist(),
            "scale_error_percent": ((np.diag(M) - 1.0) * 100.0).tolist(),
            "cross_coupling": (M - np.diag(np.diag(M))).tolist(),
            "correction": {
                "M_inv": np.linalg.inv(M).tolist(),
                "bias": bias.tolist(),
            },
        },
        "gyroscope": {"bias": np.asarray(gyro).tolist()},
        "quality": {
            "faces": [
                {
                    "face": s.face,
                    "samples": s.samples,
                    "duration": s.duration if np.isfinite(s.duration) else None,
                    "rate": s.rate if np.isfinite(s.rate) else None,
                    "gyro_std_max": float(np.max(s.gyro_std)),
                    "rejected": s.rejected,
                }
                for s in stats
            ],
            "residuals": [
                {
                    "face": row["face"],
                    "raw_norm_error": row["raw_norm_error"],
                    "norm_error": row["norm_error"],
                    "angle_deg": row["angle_deg"],
                }
                for row in rows
            ],
            "errors": errors,
            "warnings": warnings,
        },
    }


def save_payload(path, payload):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)
        fh.write("\n")


def load_calibration(path):
    with open(path, encoding="utf-8") as fh:
        payload = json.load(fh)
    acc = payload["accelerometer"]
    M_inv = np.array(acc["correction"]["M_inv"], dtype=float)
    bias = np.array(acc["correction"]["bias"], dtype=float)
    unit = payload.get("input_acc_unit", "mps2")
    return M_inv, bias, unit


def apply_rows(rows, M_inv, bias, unit="mps2"):
    """对逐行读数做修正。

    标定参数以 m/s^2 为基准，所以先把读数换算到 m/s^2，修正后再换回输入单位，
    这样输出的 CSV 与原始数据使用同一个单位。norm_before 与 norm_after 也按
    输入单位给出。
    """
    factor = G_STANDARD if unit == "g" else 1.0
    if unit not in ("g", "mps2"):
        raise ValueError(f"未知单位: {unit}")

    output = []
    for row in rows:
        acc = np.array([row["ax"], row["ay"], row["az"]], dtype=float)
        corrected = M_inv @ (acc * factor - bias) / factor
        new_row = dict(row)
        new_row["ax"] = float(corrected[0])
        new_row["ay"] = float(corrected[1])
        new_row["az"] = float(corrected[2])
        new_row["norm_before"] = float(np.linalg.norm(acc))
        new_row["norm_after"] = float(np.linalg.norm(corrected))
        output.append(new_row)
    return output


def read_raw_csv(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def write_rows(path, rows, fieldnames=None):
    if fieldnames is None:
        fieldnames = list(rows[0].keys()) if rows else ["t"]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def synthesize(seed=7, samples_per_face=800, rate=200.0,
               noise_acc=0.012, noise_gyro=0.0015, unit="mps2"):
    """生成一组已知真值的六面合成数据，用于自检与单元测试。

    unit 为 "g" 时把加速度除以标准重力，模拟 Livox 驱动的输出单位。
    返回 (rows, truth)。rows 的元素是可直接写 CSV 的字典。
    """
    rng = np.random.default_rng(seed)
    true_M = np.array(
        [
            [1.021, 0.006, -0.004],
            [0.003, 0.985, 0.005],
            [-0.005, 0.004, 1.013],
        ]
    )
    true_bias = np.array([0.15, -0.21, 0.09])
    true_gyro_bias = np.array([0.010, -0.020, 0.005])

    rows = []
    stamp = 0.0
    for face in FACES:
        axis = FACE_AXIS[face]
        sign = FACE_SIGN[face]
        true_acc = np.zeros(3)
        true_acc[axis] = sign * G_STANDARD

        acc = true_M @ true_acc + true_bias + rng.normal(
            0.0, noise_acc, (samples_per_face, 3)
        )
        gyro = true_gyro_bias + rng.normal(
            0.0, noise_gyro, (samples_per_face, 3)
        )
        if unit == "g":
            acc = acc / G_STANDARD
        elif unit != "mps2":
            raise ValueError(f"未知单位: {unit}")
        for index in range(samples_per_face):
            rows.append(
                {
                    "t": f"{stamp:.6f}",
                    "ax": f"{acc[index, 0]:.6f}",
                    "ay": f"{acc[index, 1]:.6f}",
                    "az": f"{acc[index, 2]:.6f}",
                    "wx": f"{gyro[index, 0]:.6f}",
                    "wy": f"{gyro[index, 1]:.6f}",
                    "wz": f"{gyro[index, 2]:.6f}",
                    "face": face,
                }
            )
            stamp += 1.0 / rate

    truth = {"M": true_M, "bias": true_bias, "gyro_bias": true_gyro_bias}
    return rows, truth
