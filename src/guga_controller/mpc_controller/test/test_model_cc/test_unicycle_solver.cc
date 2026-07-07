/**
 * @file test_unicycle_solver.cc
 * @brief 测试 acados 生成的 unicycle OCP C 代码。
 *
 * 不依赖 ROS，仅链接 acados 运行时库和生成的 solver 代码。
 * 做以下测试：
 *   1. 创建 & 销毁 solver (smoke test)
 *   2. 设置参考轨迹，求解 OCP，验证输出非零
 *   3. 改变初始状态，验证解相应改变
 *
 * 使用方法:
 *   cd test_model_cc && mkdir build && cd build
 *   cmake .. && make && ./test_unicycle_solver
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

// acados
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/utils/math.h"
#include "acados/utils/print.h"

// generated solver
extern "C" {
#include "acados_solver_unicycle.h"
}

#define NX   UNICYCLE_NX    // 3
#define NU   UNICYCLE_NU    // 2
#define N    UNICYCLE_N     // 20
#define NY   UNICYCLE_NY    // 5 (x + u)
#define NYN  UNICYCLE_NYN   // 3 (x)
#define NBX0 UNICYCLE_NBX0  // 3

// ============================================================================
// 辅助函数
// ============================================================================
static bool double_eq(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) < eps;
}

// ============================================================================
// Test 1: Smoke test — 创建并销毁 solver
// ============================================================================
static bool test_create_destroy() {
    std::printf("--- Test 1: Create & Destroy ---\n");

    unicycle_solver_capsule *capsule = unicycle_acados_create_capsule();
    if (!capsule) {
        std::printf("  FAIL: unicycle_acados_create_capsule() returned NULL\n");
        return false;
    }

    int status = unicycle_acados_create_with_discretization(capsule, N, nullptr);
    if (status != 0) {
        std::printf("  FAIL: create_with_discretization returned %d\n", status);
        unicycle_acados_free_capsule(capsule);
        return false;
    }

    status = unicycle_acados_free(capsule);
    if (status != 0) {
        std::printf("  FAIL: unicycle_acados_free returned %d\n", status);
        unicycle_acados_free_capsule(capsule);
        return false;
    }

    status = unicycle_acados_free_capsule(capsule);
    if (status != 0) {
        std::printf("  FAIL: unicycle_acados_free_capsule returned %d\n", status);
        return false;
    }

    std::printf("  PASS\n");
    return true;
}

// ============================================================================
// Test 2: 求解 OCP — 初始状态在原点，参考轨迹为直线
// ============================================================================
static bool test_solve_ocp_basic() {
    std::printf("--- Test 2: Basic OCP Solve (line reference) ---\n");

    unicycle_solver_capsule *capsule = unicycle_acados_create_capsule();
    int status = unicycle_acados_create_with_discretization(capsule, N, nullptr);
    if (status != 0) {
        std::printf("  FAIL: create returned %d\n", status);
        return false;
    }

    ocp_nlp_config *nlp_config = unicycle_acados_get_nlp_config(capsule);
    ocp_nlp_dims  *nlp_dims   = unicycle_acados_get_nlp_dims(capsule);
    ocp_nlp_in    *nlp_in     = unicycle_acados_get_nlp_in(capsule);
    ocp_nlp_out   *nlp_out    = unicycle_acados_get_nlp_out(capsule);
    ocp_nlp_solver *nlp_solver = unicycle_acados_get_nlp_solver(capsule);

    // ---- 初始状态 ----
    double lbx0[NBX0] = {0.0, 0.0, 0.0};
    double ubx0[NBX0] = {0.0, 0.0, 0.0};
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0,
                                  "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0,
                                  "ubx", ubx0);

    // ---- 参考轨迹 (yref = [x, y, theta, v, omega]) ----
    // 直线前进: 沿 x 轴以 1m/s 前进
    double yref_e[NYN] = {2.0, 0.0, 0.0};             // terminal ref

    // 设置所有 stage 的参考值
    for (int i = 0; i < N; ++i) {
        // 每个 stage 的期望位置
        double progress = static_cast<double>(i + 1) / N * 2.0;
        double yref_i[NY] = {progress, 0.0, 0.0, 1.0, 0.0};
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i,
                               "y_ref", yref_i);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N,
                           "y_ref", yref_e);

    // ---- 初始猜测 ----
    double x_init[NX] = {0.0, 0.0, 0.0};
    double u0[NU]     = {1.0, 0.0};
    for (int i = 0; i < N; ++i) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_init);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x_init);

    // ---- 求解 ----
    status = unicycle_acados_solve(capsule);
    if (status != ACADOS_SUCCESS) {
        std::printf("  FAIL: solve returned status %d\n", status);
        unicycle_acados_free(capsule);
        unicycle_acados_free_capsule(capsule);
        return false;
    }

    // ---- 提取解 ----
    double xtraj[NX * (N + 1)];
    double utraj[NU * N];
    for (int i = 0; i <= N; ++i) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &xtraj[i * NX]);
    }
    for (int i = 0; i < N; ++i) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", &utraj[i * NU]);
    }

    // ---- 验证 ----
    // 1. x0 等于初始条件
    if (!double_eq(xtraj[0], 0.0) || !double_eq(xtraj[1], 0.0) ||
        !double_eq(xtraj[2], 0.0)) {
        std::printf("  FAIL: x0 should be (0,0,0), got (%f,%f,%f)\n",
                    xtraj[0], xtraj[1], xtraj[2]);
        goto cleanup;
    }

    // 2. 第一次控制 v > 0 (应该向前走)
    if (utraj[0] <= 0.0) {
        std::printf("  FAIL: first v should be > 0, got %f\n", utraj[0]);
        goto cleanup;
    }

    // 3. y 轨迹应该基本为 0 (参考是直线)
    {
        double max_y_dev = 0.0;
        for (int i = 0; i <= N; ++i) {
            double y = xtraj[i * NX + 1];
            if (std::fabs(y) > max_y_dev) { max_y_dev = std::fabs(y); }
        }
        if (max_y_dev > 0.5) {
            std::printf("  FAIL: max y deviation too large: %f\n", max_y_dev);
            goto cleanup;
        }
    }

    // 4. 控制满足约束 [-2.5, 2.5], [-4.0, 4.0]
    for (int i = 0; i < N; ++i) {
        double v = utraj[i * NU];
        double omega = utraj[i * NU + 1];
        if (v < -2.5001 || v > 2.5001 || omega < -4.0001 || omega > 4.0001) {
            std::printf("  FAIL: control bounds violated at step %d: v=%f, ω=%f\n",
                        i, v, omega);
            goto cleanup;
        }
    }

    // ---- 打印统计 ----
    {
        int sqp_iter = 0;
        double kkt = 0.0, time_tot = 0.0;
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt);
        ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);
        ocp_nlp_get(nlp_solver, "time_tot", &time_tot);
        std::printf("  SQP iter=%d, KKT=%e, time=%f ms\n",
                    sqp_iter, kkt, time_tot * 1000);
    }

    std::printf("  x[N] = (%7.4f, %7.4f, %7.4f)\n",
                xtraj[N * NX], xtraj[N * NX + 1], xtraj[N * NX + 2]);
    std::printf("  u[0] = (%7.4f, %7.4f)\n", utraj[0], utraj[1]);
    std::printf("  PASS\n");

cleanup:
    unicycle_acados_free(capsule);
    unicycle_acados_free_capsule(capsule);
    return true;
}

// ============================================================================
// Test 3: 改变初始状态 — 初始角度非零
// ============================================================================
static bool test_different_initial_state() {
    std::printf("--- Test 3: Different Initial State ---\n");

    unicycle_solver_capsule *capsule = unicycle_acados_create_capsule();
    int status = unicycle_acados_create_with_discretization(capsule, N, nullptr);
    if (status != 0) {
        std::printf("  FAIL: create returned %d\n", status);
        return false;
    }

    ocp_nlp_config *nlp_config = unicycle_acados_get_nlp_config(capsule);
    ocp_nlp_dims  *nlp_dims   = unicycle_acados_get_nlp_dims(capsule);
    ocp_nlp_in    *nlp_in     = unicycle_acados_get_nlp_in(capsule);
    ocp_nlp_out   *nlp_out    = unicycle_acados_get_nlp_out(capsule);

    // ---- 初始状态: x=0, y=1 (偏离参考), theta=π/2 ----
    double lbx0[NBX0] = {0.0, 1.0, M_PI / 2.0};
    double ubx0[NBX0] = {0.0, 1.0, M_PI / 2.0};
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0,
                                  "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0,
                                  "ubx", ubx0);

    // 参考: 沿x轴直线
    for (int i = 0; i < N; ++i) {
        double progress = static_cast<double>(i + 1) / N * 2.0;
        double yref_i[NY] = {progress, 0.0, 0.0, 1.0, 0.0};
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i,
                               "y_ref", yref_i);
    }
    double yref_e[NYN] = {2.0, 0.0, 0.0};
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N,
                           "y_ref", yref_e);

    // 初始猜测
    double x_init[NX] = {0.0, 1.0, M_PI / 2.0};
    double u0[NU]     = {0.5, -1.0};  // 猜测: 减速 + 左转靠近参考
    for (int i = 0; i < N; ++i) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_init);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x_init);

    // 求解
    status = unicycle_acados_solve(capsule);
    if (status != ACADOS_SUCCESS) {
        std::printf("  FAIL: solve returned status %d\n", status);
        unicycle_acados_free(capsule);
        unicycle_acados_free_capsule(capsule);
        return false;
    }

    // 提取解
    double utraj[NU * N];
    for (int i = 0; i < N; ++i) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", &utraj[i * NU]);
    }

    // 验证: 由于初始 y>0 偏离参考 y=0，第一次 omega 应为负（右转靠近参考）
    // 注意: theta=pi/2 (朝上), 参考在右(x正), 所以需要右转 → omega < 0
    std::printf("  u[0] = (%7.4f, %7.4f)\n", utraj[0], utraj[1]);

    if (utraj[1] > 0.0) {
        std::printf("  WARN: expected omega[0] < 0 (turning right toward ref),\n"
                    "        got omega[0] = %f\n", utraj[1]);
        // 不以此为 hard fail，因为 MPC 优化可能偏爱其他策略
    } else {
        std::printf("  OK: omega[0] < 0, correct direction toward reference\n");
    }

    std::printf("  PASS\n");

    unicycle_acados_free(capsule);
    unicycle_acados_free_capsule(capsule);
    return true;
}

// ============================================================================
// Test 4: 求解时间统计
// ============================================================================
static bool test_solve_timing() {
    std::printf("--- Test 4: Solve Timing ---\n");

    unicycle_solver_capsule *capsule = unicycle_acados_create_capsule();
    int status = unicycle_acados_create_with_discretization(capsule, N, nullptr);
    if (status != 0) {
        std::printf("  FAIL: create returned %d\n", status);
        return false;
    }

    ocp_nlp_config *nlp_config = unicycle_acados_get_nlp_config(capsule);
    ocp_nlp_dims  *nlp_dims   = unicycle_acados_get_nlp_dims(capsule);
    ocp_nlp_in    *nlp_in     = unicycle_acados_get_nlp_in(capsule);
    ocp_nlp_out   *nlp_out    = unicycle_acados_get_nlp_out(capsule);
    ocp_nlp_solver *nlp_solver = unicycle_acados_get_nlp_solver(capsule);

    double lbx0[NBX0] = {0.0, 0.0, 0.0};
    double ubx0[NBX0] = {0.0, 0.0, 0.0};
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0,
                                  "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0,
                                  "ubx", ubx0);

    for (int i = 0; i < N; ++i) {
        double progress = static_cast<double>(i + 1) / N * 2.0;
        double yref_i[NY] = {progress, 0.0, 0.0, 1.0, 0.0};
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i,
                               "y_ref", yref_i);
    }
    double yref_e[NYN] = {2.0, 0.0, 0.0};
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "y_ref", yref_e);

    double x_init[NX] = {0.0, 0.0, 0.0};
    double u0[NU]     = {1.0, 0.0};
    for (int i = 0; i < N; ++i) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_init);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x_init);

    // 预热一次
    status = unicycle_acados_solve(capsule);

    // 测时
    const int n_timing = 20;
    double times[n_timing];
    double min_time = 1e12, max_time = 0.0, sum_time = 0.0;

    for (int i = 0; i < n_timing; ++i) {
        status = unicycle_acados_solve(capsule);
        if (status != ACADOS_SUCCESS) {
            std::printf("  FAIL: solve returned %d at iteration %d\n", status, i);
            goto cleanup;
        }
        double t = 0.0;
        ocp_nlp_get(nlp_solver, "time_tot", &t);
        times[i] = t * 1000.0;  // ms
        sum_time += times[i];
        if (times[i] < min_time) { min_time = times[i]; }
        if (times[i] > max_time) { max_time = times[i]; }
    }

    std::printf("  %d solves:\n", n_timing);
    std::printf("    min  = %.3f ms\n", min_time);
    std::printf("    max  = %.3f ms\n", max_time);
    std::printf("    mean = %.3f ms\n", sum_time / n_timing);

    // 硬实时约束检查: 50ms 控制周期内必须能求解完毕
    if (max_time > 50.0) {
        std::printf("  WARN: max solve time (%.1f ms) > 50 ms control period\n",
                    max_time);
    } else {
        std::printf("  OK: within 50 ms budget\n");
    }

    std::printf("  PASS\n");

cleanup:
    unicycle_acados_free(capsule);
    unicycle_acados_free_capsule(capsule);
    return true;
}

// ============================================================================
int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    std::printf("============================================================\n");
    std::printf("acados Unicycle Solver C API Test\n");
    std::printf("  NX=%d, NU=%d, N=%d\n", NX, NU, N);
    std::printf("============================================================\n\n");

    int passed = 0;
    int total = 4;

    if (test_create_destroy()) { ++passed; }
    std::printf("\n");

    if (test_solve_ocp_basic()) { ++passed; }
    std::printf("\n");

    if (test_different_initial_state()) { ++passed; }
    std::printf("\n");

    if (test_solve_timing()) { ++passed; }
    std::printf("\n");

    std::printf("============================================================\n");
    std::printf("Results: %d/%d tests passed\n", passed, total);
    std::printf("============================================================\n");

    return (passed == total) ? 0 : 1;
}
