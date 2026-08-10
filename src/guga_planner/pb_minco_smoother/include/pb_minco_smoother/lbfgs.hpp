/**
 * @file lbfgs.hpp
 * @brief L-BFGS 优化器
 *
 * 基于 GCOPTER (ZJU-FAST-Lab) 的 lbfgs.hpp
 * 用于 MINCO 轨迹优化的无约束优化
 *
 * L-BFGS (Limited-memory BFGS) 是一种拟牛顿法，
 * 适用于大规模优化问题，内存开销低。
 *
 * MIT License
 * Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)
 * Modified for pb_minco_smoother by Jinbo Liu, 2025
 */

#ifndef PB_MINCO_SMOOTHER__LBFGS_HPP_
#define PB_MINCO_SMOOTHER__LBFGS_HPP_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>

namespace pb_minco {
namespace lbfgs {

/**
 * @brief L-BFGS 优化参数
 */
struct LbfgsParams {
    /// 用于近似逆 Hessian 的历史步数，默认 8
    int mem_size = 8;

    /// 梯度收敛阈值，||g||_inf / max(1, ||x||_inf) < g_epsilon
    double g_epsilon = 1.0e-5;

    /// 增量收敛测试的历史距离
    int past = 3;

    /// 增量收敛阈值，|f_past - f| / max(1, |f|) < delta
    double delta = 1.0e-6;

    /// 最大迭代次数，0 表示无限制
    int max_iterations = 0;

    /// 线搜索最大次数
    int max_linesearch = 64;

    /// 最小步长
    double min_step = 1.0e-20;

    /// 最大步长
    double max_step = 1.0e+20;

    /// Armijo 条件参数
    double f_dec_coeff = 1.0e-4;

    /// Wolfe 条件参数
    double s_curv_coeff = 0.9;

    /// 谨慎更新因子
    double cautious_factor = 1.0e-6;

    /// 机器精度
    double machine_prec = 1.0e-16;
};

/**
 * @brief L-BFGS 返回值
 */
enum LbfgsReturn {
    LBFGS_CONVERGENCE = 0,  ///< 收敛
    LBFGS_STOP,             ///< 满足停止条件
    LBFGS_CANCELED,         ///< 被回调取消
    LBFGSERR_UNKNOWNERROR = -1024,
    LBFGSERR_INVALID_N,
    LBFGSERR_INVALID_MEMSIZE,
    LBFGSERR_INVALID_GEPSILON,
    LBFGSERR_INVALID_TESTPERIOD,
    LBFGSERR_INVALID_DELTA,
    LBFGSERR_INVALID_MINSTEP,
    LBFGSERR_INVALID_MAXSTEP,
    LBFGSERR_INVALID_FDECCOEFF,
    LBFGSERR_INVALID_SCURVCOEFF,
    LBFGSERR_INVALID_MACHINEPREC,
    LBFGSERR_INVALID_MAXLINESEARCH,
    LBFGSERR_INVALID_FUNCVAL,
    LBFGSERR_MINIMUMSTEP,
    LBFGSERR_MAXIMUMSTEP,
    LBFGSERR_MAXIMUMLINESEARCH,
    LBFGSERR_MAXIMUMITERATION,
    LBFGSERR_WIDTHTOOSMALL,
    LBFGSERR_INVALIDPARAMETERS,
    LBFGSERR_INCREASEGRADIENT,
};

/**
 * @brief 目标函数类型
 *
 * @param instance 用户数据指针
 * @param x 当前变量值
 * @param g 输出：梯度向量
 * @return 函数值
 */
using EvaluateFunc = double (*)(void *instance, const Eigen::VectorXd &x,
                                Eigen::VectorXd &g);

/**
 * @brief 步长上界函数类型（可选）
 */
using StepBoundFunc = double (*)(void *instance, const Eigen::VectorXd &xp,
                                 const Eigen::VectorXd &d);

/**
 * @brief 进度回调函数类型（可选）
 */
using ProgressFunc = int (*)(void *instance, const Eigen::VectorXd &x,
                             const Eigen::VectorXd &g, double fx, double step,
                             int k, int ls);

/**
 * @brief 回调数据结构
 */
struct CallbackData {
    void *instance = nullptr;
    EvaluateFunc proc_evaluate = nullptr;
    StepBoundFunc proc_stepbound = nullptr;
    ProgressFunc proc_progress = nullptr;
};

/**
 * @brief Lewis-Overton 线搜索
 *
 * 适用于光滑和非光滑函数的线搜索方法
 */
inline int lineSearchLewisOverton(Eigen::VectorXd &x, double &f,
                                  Eigen::VectorXd &g, double &stp,
                                  const Eigen::VectorXd &s,
                                  const Eigen::VectorXd &xp,
                                  const Eigen::VectorXd &gp, double stpmin,
                                  double stpmax, const CallbackData &cd,
                                  const LbfgsParams &param) {
    int count = 0;
    bool brackt = false, touched = false;
    double finit, dginit, dgtest, dstest;
    double mu = 0.0, nu = stpmax;

    if (!(stp > 0.0)) {
        return LBFGSERR_INVALIDPARAMETERS;
    }

    // 计算初始方向导数
    dginit = gp.dot(s);

    // 检查是否为下降方向
    if (0.0 < dginit) {
        return LBFGSERR_INCREASEGRADIENT;
    }

    finit = f;
    dgtest = param.f_dec_coeff * dginit;
    dstest = param.s_curv_coeff * dginit;

    while (true) {
        x = xp + stp * s;
        f = cd.proc_evaluate(cd.instance, x, g);
        ++count;

        if (std::isinf(f) || std::isnan(f)) {
            return LBFGSERR_INVALID_FUNCVAL;
        }
        if (param.past > 0 &&
            fabs(finit - f) / (fabs(finit) + 1.0) < param.delta / param.past) {
            return count;
        }
        // Armijo 条件
        if (f > finit + stp * dgtest) {
            nu = stp;
            brackt = true;
        } else {
            // Wolfe 条件
            if (g.dot(s) < dstest) {
                mu = stp;
            } else {
                return count;
            }
        }

        if (param.max_linesearch <= count) {
            return LBFGSERR_MAXIMUMLINESEARCH;
        }

        if (brackt && (nu - mu) < param.machine_prec * nu) {
            return LBFGSERR_WIDTHTOOSMALL;
        }

        if (brackt) {
            stp = 0.5 * (mu + nu);
        } else {
            stp *= 2.0;
        }

        if (stp < stpmin) {
            return LBFGSERR_MINIMUMSTEP;
        }

        if (stp > stpmax) {
            if (touched) {
                return LBFGSERR_MAXIMUMSTEP;
            } else {
                touched = true;
                stp = stpmax;
            }
        }
    }
}

/**
 * @brief L-BFGS 优化主函数
 *
 * @param x 变量向量（输入为初始值，输出为最优解）
 * @param f 最终函数值
 * @param proc_evaluate 目标函数
 * @param proc_stepbound 步长上界函数（可为 nullptr）
 * @param proc_progress 进度回调（可为 nullptr）
 * @param instance 用户数据指针
 * @param param 优化参数
 * @return 返回状态码
 */
inline int lbfgsOptimize(Eigen::VectorXd &x, double &f,
                         EvaluateFunc proc_evaluate,
                         StepBoundFunc proc_stepbound,
                         ProgressFunc proc_progress, void *instance,
                         const LbfgsParams &param) {
    int ret, i, j, k, ls, end, bound;
    double step, step_min, step_max, fx, ys, yy;
    double gnorm_inf, xnorm_inf, beta, rate, cau;

    const int n = x.size();
    const int m = param.mem_size;

    // 参数检查
    if (n <= 0) return LBFGSERR_INVALID_N;
    if (m <= 0) return LBFGSERR_INVALID_MEMSIZE;
    if (param.g_epsilon < 0.0) return LBFGSERR_INVALID_GEPSILON;
    if (param.past < 0) return LBFGSERR_INVALID_TESTPERIOD;
    if (param.delta < 0.0) return LBFGSERR_INVALID_DELTA;
    if (param.min_step < 0.0) return LBFGSERR_INVALID_MINSTEP;
    if (param.max_step < param.min_step) return LBFGSERR_INVALID_MAXSTEP;
    if (!(param.f_dec_coeff > 0.0 && param.f_dec_coeff < 1.0))
        return LBFGSERR_INVALID_FDECCOEFF;
    if (!(param.s_curv_coeff < 1.0 && param.s_curv_coeff > param.f_dec_coeff))
        return LBFGSERR_INVALID_SCURVCOEFF;
    if (!(param.machine_prec > 0.0)) return LBFGSERR_INVALID_MACHINEPREC;
    if (param.max_linesearch <= 0) return LBFGSERR_INVALID_MAXLINESEARCH;

    // 准备中间变量
    Eigen::VectorXd xp(n);
    Eigen::VectorXd g(n);
    Eigen::VectorXd gp(n);
    Eigen::VectorXd d(n);
    Eigen::VectorXd pf(std::max(1, param.past));

    // 初始化 L-BFGS 记忆
    Eigen::VectorXd lm_alpha = Eigen::VectorXd::Zero(m);
    Eigen::MatrixXd lm_s = Eigen::MatrixXd::Zero(n, m);
    Eigen::MatrixXd lm_y = Eigen::MatrixXd::Zero(n, m);
    Eigen::VectorXd lm_ys = Eigen::VectorXd::Zero(m);

    // 构造回调数据
    CallbackData cd;
    cd.instance = instance;
    cd.proc_evaluate = proc_evaluate;
    cd.proc_stepbound = proc_stepbound;
    cd.proc_progress = proc_progress;

    // 计算初始函数值和梯度
    fx = cd.proc_evaluate(cd.instance, x, g);
    pf(0) = fx;

    // 初始搜索方向
    d = -g;

    // 检查初始点是否已是驻点
    gnorm_inf = g.cwiseAbs().maxCoeff();
    xnorm_inf = x.cwiseAbs().maxCoeff();

    if (gnorm_inf / std::max(1.0, xnorm_inf) < param.g_epsilon) {
        ret = LBFGS_CONVERGENCE;
    } else {
        // 初始步长
        step = 1.0 / d.norm();

        k = 1;
        end = 0;
        bound = 0;

        while (true) {
            // 保存当前点
            xp = x;
            gp = g;

            // 确定步长范围
            step_min = param.min_step;
            step_max = param.max_step;
            if (cd.proc_stepbound) {
                step_max = cd.proc_stepbound(cd.instance, xp, d);
                step_max = std::min(step_max, param.max_step);
                step = std::min(step, 0.5 * step_max);
            }

            // 线搜索
            ls = lineSearchLewisOverton(x, fx, g, step, d, xp, gp, step_min,
                                        step_max, cd, param);

            if (ls < 0) {
                x = xp;
                g = gp;
                ret = ls;
                break;
            }

            // 进度回调
            if (cd.proc_progress) {
                if (cd.proc_progress(cd.instance, x, g, fx, step, k, ls)) {
                    ret = LBFGS_CANCELED;
                    break;
                }
            }

            // 梯度收敛测试
            gnorm_inf = g.cwiseAbs().maxCoeff();
            xnorm_inf = x.cwiseAbs().maxCoeff();
            if (gnorm_inf / std::max(1.0, xnorm_inf) < param.g_epsilon) {
                ret = LBFGS_CONVERGENCE;
                break;
            }

            // 增量收敛测试
            if (0 < param.past) {
                if (param.past <= k) {
                    rate = std::fabs(pf(k % param.past) - fx) /
                           std::max(1.0, std::fabs(fx));
                    if (rate < param.delta) {
                        ret = LBFGS_STOP;
                        break;
                    }
                }
                pf(k % param.past) = fx;
            }

            // 最大迭代检查
            if (param.max_iterations != 0 && param.max_iterations <= k) {
                ret = LBFGSERR_MAXIMUMITERATION;
                break;
            }

            ++k;

            // 更新 L-BFGS 记忆
            lm_s.col(end) = x - xp;
            lm_y.col(end) = g - gp;
            ys = lm_y.col(end).dot(lm_s.col(end));
            yy = lm_y.col(end).squaredNorm();
            lm_ys(end) = ys;

            // 初始搜索方向
            d = -g;

            // 谨慎更新检查
            cau =
                lm_s.col(end).squaredNorm() * gp.norm() * param.cautious_factor;

            if (ys > cau) {
                // 双循环递归
                ++bound;
                bound = std::min(m, bound);
                end = (end + 1) % m;

                j = end;
                for (i = 0; i < bound; ++i) {
                    j = (j + m - 1) % m;
                    lm_alpha(j) = lm_s.col(j).dot(d) / lm_ys(j);
                    d += (-lm_alpha(j)) * lm_y.col(j);
                }

                d *= ys / yy;

                for (i = 0; i < bound; ++i) {
                    beta = lm_y.col(j).dot(d) / lm_ys(j);
                    d += (lm_alpha(j) - beta) * lm_s.col(j);
                    j = (j + 1) % m;
                }
            }

            step = 1.0;
        }
    }

    f = fx;
    return ret;
}

/**
 * @brief 获取返回状态的描述字符串
 */
inline const char *lbfgsStrerror(int err) {
    switch (err) {
        case LBFGS_CONVERGENCE:
            return "收敛：达到梯度收敛条件";
        case LBFGS_STOP:
            return "收敛：达到增量收敛条件";
        case LBFGS_CANCELED:
            return "优化被用户回调取消";
        case LBFGSERR_UNKNOWNERROR:
            return "未知错误";
        case LBFGSERR_INVALID_N:
            return "无效的变量数量";
        case LBFGSERR_INVALID_MEMSIZE:
            return "无效的记忆大小参数";
        case LBFGSERR_INVALID_GEPSILON:
            return "无效的梯度收敛阈值";
        case LBFGSERR_INVALID_TESTPERIOD:
            return "无效的测试周期参数";
        case LBFGSERR_INVALID_DELTA:
            return "无效的增量收敛阈值";
        case LBFGSERR_INVALID_MINSTEP:
            return "无效的最小步长";
        case LBFGSERR_INVALID_MAXSTEP:
            return "无效的最大步长";
        case LBFGSERR_INVALID_FDECCOEFF:
            return "无效的 Armijo 参数";
        case LBFGSERR_INVALID_SCURVCOEFF:
            return "无效的 Wolfe 参数";
        case LBFGSERR_INVALID_MACHINEPREC:
            return "无效的机器精度";
        case LBFGSERR_INVALID_MAXLINESEARCH:
            return "无效的最大线搜索次数";
        case LBFGSERR_INVALID_FUNCVAL:
            return "函数值为 NaN 或 Inf";
        case LBFGSERR_MINIMUMSTEP:
            return "步长小于最小值";
        case LBFGSERR_MAXIMUMSTEP:
            return "步长大于最大值";
        case LBFGSERR_MAXIMUMLINESEARCH:
            return "线搜索达到最大次数";
        case LBFGSERR_MAXIMUMITERATION:
            return "迭代达到最大次数";
        case LBFGSERR_WIDTHTOOSMALL:
            return "搜索区间过窄";
        case LBFGSERR_INVALIDPARAMETERS:
            return "逻辑错误（负步长）";
        case LBFGSERR_INCREASEGRADIENT:
            return "搜索方向增加函数值";
        default:
            return "未知状态";
    }
}

}  // namespace lbfgs
}  // namespace pb_minco

#endif  // PB_MINCO_SMOOTHER__LBFGS_HPP_
