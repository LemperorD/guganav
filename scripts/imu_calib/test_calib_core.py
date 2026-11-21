"""六面标定核心算法的单元测试。

运行方式:
    cd scripts/imu_calib && python3 -m unittest -v test_calib_core
"""

import os
import tempfile
import unittest

import numpy as np

import calib_core as core


def _exact_means(M, bias):
    means = {}
    for face in core.FACES:
        axis = core.FACE_AXIS[face]
        sign = core.FACE_SIGN[face]
        true_acc = np.zeros(3)
        true_acc[axis] = sign * core.G_STANDARD
        means[face] = M @ true_acc + bias
    return means


TRUE_M = np.array(
    [
        [1.021, 0.006, -0.004],
        [0.003, 0.985, 0.005],
        [-0.005, 0.004, 1.013],
    ]
)
TRUE_BIAS = np.array([0.15, -0.21, 0.09])


class TestRobustMean(unittest.TestCase):
    def test_rejects_spike(self):
        rng = np.random.default_rng(1)
        data = rng.normal(0.0, 0.01, (1000, 3))
        data[500] = [5.0, 5.0, 5.0]
        mean, std, rejected = core.robust_mean(data)
        self.assertLess(np.max(np.abs(mean)), 0.002)
        self.assertGreater(rejected, 0.0)

    def test_keeps_clean_data(self):
        rng = np.random.default_rng(2)
        data = rng.normal(1.0, 0.01, (500, 3))
        mean, _, rejected = core.robust_mean(data)
        # 3 sigma 截断在高斯噪声上本来就会剔除千分之几的样本
        self.assertLess(rejected, 0.02)
        self.assertLess(np.max(np.abs(mean - 1.0)), 0.002)


class TestSolve(unittest.TestCase):
    def test_full_model_recovers_exact_parameters(self):
        means = _exact_means(TRUE_M, TRUE_BIAS)
        solution = core.solve_means(means, model="full")
        self.assertLess(np.max(np.abs(solution["M"] - TRUE_M)), 1e-9)
        self.assertLess(np.max(np.abs(solution["bias"] - TRUE_BIAS)), 1e-9)

    def test_diag_model_matches_pair_formula(self):
        means = _exact_means(TRUE_M, TRUE_BIAS)
        solution = core.solve_means(means, model="diag")
        for axis in range(3):
            name = "xyz"[axis]
            up = means["+" + name][axis]
            down = means["-" + name][axis]
            self.assertAlmostEqual(solution["bias"][axis], (up + down) / 2.0)
            self.assertAlmostEqual(
                solution["M"][axis, axis],
                (up - down) / (2.0 * core.G_STANDARD),
            )

    def test_missing_face_raises(self):
        means = _exact_means(TRUE_M, TRUE_BIAS)
        del means["+z"]
        with self.assertRaises(ValueError):
            core.solve_means(means)

    def test_noisy_recovery(self):
        rng = np.random.default_rng(3)
        means = _exact_means(TRUE_M, TRUE_BIAS)
        means = {k: v + rng.normal(0.0, 0.01, 3) for k, v in means.items()}
        solution = core.solve_means(means, model="full")
        self.assertLess(np.max(np.abs(solution["M"] - TRUE_M)), 5e-3)
        self.assertLess(np.max(np.abs(solution["bias"] - TRUE_BIAS)), 3e-2)


class TestEvaluate(unittest.TestCase):
    def test_exact_data_has_zero_residual(self):
        means = _exact_means(TRUE_M, TRUE_BIAS)
        stats = [
            core.FaceStats(face, 500, 2.5, means[face], np.zeros(3),
                           np.zeros(3), np.zeros(3), 0.0)
            for face in core.FACES
        ]
        solution = core.solve_means(means, model="full")
        rows = core.evaluate(stats, solution["M"], solution["bias"])
        for row in rows:
            self.assertLess(abs(row["norm_error"]), 1e-9)
            self.assertLess(row["angle_deg"], 1e-6)
            self.assertTrue(row["sign_ok"])

    def test_wrong_face_is_detected(self):
        means = _exact_means(TRUE_M, TRUE_BIAS)
        # 把 +x 面的数据换成 -x 面的读数，模拟朝向放反
        means["+x"] = means["-x"].copy()
        stats = [
            core.FaceStats(face, 500, 2.5, means[face], np.zeros(3),
                           np.zeros(3), np.zeros(3), 0.0)
            for face in core.FACES
        ]
        solution = core.solve_means(means, model="full")
        rows = core.evaluate(stats, solution["M"], solution["bias"])
        bad = [row for row in rows if row["face"] == "+x"]
        self.assertFalse(bad[0]["sign_ok"])
        errors, _ = core.check_quality(stats, rows, solution["M"])
        self.assertTrue(any("+x" in message for message in errors))


class TestCsvAndApply(unittest.TestCase):
    def test_csv_roundtrip(self):
        rows, truth = core.synthesize(samples_per_face=300)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "six_face.csv")
            core.write_rows(path, rows)
            stats = core.load_csv(path)
            self.assertEqual([s.face for s in stats], list(core.FACES))
            unit, _ = core.infer_unit(stats)
            self.assertEqual(unit, "mps2")
            solution = core.solve(core.to_si(stats, unit), model="full")
            self.assertLess(np.max(np.abs(solution["M"] - truth["M"])), 0.01)

    def test_g_unit_input_converted_to_si(self):
        rows, truth = core.synthesize(samples_per_face=300, unit="g")
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "six_face_g.csv")
            core.write_rows(path, rows)
            stats = core.load_csv(path)
            unit, median = core.infer_unit(stats)
            self.assertEqual(unit, "g")
            self.assertAlmostEqual(median, 1.0, delta=0.05)
            solution = core.solve(core.to_si(stats, "g"), model="full")
            self.assertLess(np.max(np.abs(solution["M"] - truth["M"])), 0.01)
            self.assertLess(
                np.max(np.abs(solution["bias"] - truth["bias"])), 0.03
            )

    def test_apply_reduces_norm_error(self):
        rows, _ = core.synthesize(samples_per_face=200)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "six_face.csv")
            core.write_rows(path, rows)
            stats = core.load_csv(path)
            solution = core.solve(stats, model="full")
            payload = core.build_payload(
                stats, "mps2", solution,
                core.evaluate(stats, solution["M"], solution["bias"]),
                core.gyro_bias(stats), [], [], model="full",
            )
            calib_path = os.path.join(directory, "calib.json")
            core.save_payload(calib_path, payload)

            M_inv, bias, unit = core.load_calibration(calib_path)
            corrected = core.apply_rows(
                core.read_raw_csv(path), M_inv, bias, unit
            )
            before = np.array([row["norm_before"] for row in corrected])
            after = np.array([row["norm_after"] for row in corrected])
            g = core.G_STANDARD
            self.assertLess(np.max(np.abs(after - g)), np.max(np.abs(before - g)))

    def test_apply_keeps_g_unit(self):
        rows, _ = core.synthesize(samples_per_face=200, unit="g")
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "six_face_g.csv")
            core.write_rows(path, rows)
            stats = core.to_si(core.load_csv(path), "g")
            solution = core.solve(stats, model="full")
            corrected = core.apply_rows(
                core.read_raw_csv(path),
                np.linalg.inv(solution["M"]),
                solution["bias"],
                "g",
            )
            after = np.array([row["norm_after"] for row in corrected])
            self.assertAlmostEqual(float(after.mean()), 1.0, delta=0.01)


if __name__ == "__main__":
    unittest.main()
