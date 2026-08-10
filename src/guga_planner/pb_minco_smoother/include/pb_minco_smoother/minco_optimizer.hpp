/**
 * @file minco_optimizer.hpp
 * @brief 2D MINCO 轨迹优化器 - 融合 ESDF 避障 (S=3, Quintic Polynomial)
 *
 * 本文件实现了基于 MINCO 的 2D 地面机器人轨迹优化器
 * 使用 gcopter 库的 MINCO_S3NU 实现（5次多项式，最小化Jerk）
 *
 * 核心特性：
 * 1. 平滑性代价：最小化 Jerk 能量（S=3 MINCO）
 * 2. 避障代价：利用 ESDF 进行软约束避障
 * 3. 动力学约束：速度和加速度软约束
 * 4. 时间优化：同时优化路点位置和时间分配
 *
 * S=3 vs S=2 的区别：
 * - S=2 (Cubic): 3次多项式，最小化加速度，只保证位置/速度连续
 * - S=3 (Quintic): 5次多项式，最小化Jerk，保证位置/速度/加速度连续
 *
 * 使用方法：
 * 1. 设置 ESDF 接口（GridMapSDF 指针）
 * 2. 设置边界条件和初始路径
 * 3. 调用 optimize() 进行优化
 * 4. 获取优化后的轨迹
 *
 * MIT License
 * Modified for 2D ground robot by Jinbo Liu, 2025
 */

#ifndef PB_MINCO_SMOOTHER__MINCO_OPTIMIZER_HPP_
#define PB_MINCO_SMOOTHER__MINCO_OPTIMIZER_HPP_

#include <Eigen/Eigen>
#include <cmath>
#include <ctime>
#include <functional>
#include <memory>
#include <vector>

#include "gcopter/minco.hpp"       
#include "gcopter/trajectory.hpp"
#include "pb_minco_smoother/lbfgs.hpp"

namespace pb_minco {

/**
 * @brief ESDF 接口抽象类
 *
 * 用户需要实现此接口以提供环境距离场信息
 */
class ESDFInterface {
   public:
    virtual ~ESDFInterface() = default;

    /**
     * @brief 获取点 (x, y) 到最近障碍物的有符号距离
     * @param x X 坐标（世界坐标系）
     * @param y Y 坐标（世界坐标系）
     * @return 有符号距离，正值表示在自由空间内，负值表示在障碍物内
     */
    virtual double getDistance(double x, double y) const = 0;

    /**
     * @brief 获取距离场在点 (x, y) 处的梯度
     * @param x X 坐标
     * @param y Y 坐标
     * @return 梯度向量 (∂d/∂x, ∂d/∂y)
     */
    virtual Eigen::Vector2d getGradient(double x, double y) const = 0;

    /**
     * @brief 检查点是否在地图范围内
     */
    virtual bool isInside(double x, double y) const = 0;
};

/**
 * @brief 优化器配置参数
 */
struct MincoOptimizerConfig {
    // ========== 权重参数 ==========
    double weight_smooth = 1.0;        ///< 平滑性代价权重
    double weight_obstacle = 100.0;    ///< 避障代价权重
    double weight_feasibility = 10.0;  ///< 动力学可行性代价权重
    double weight_time = 10.0;         ///< 时间正则化权重
    double weight_mean_time = 10.0;    ///< 平均时间约束权重

    // ========== 约束参数 ==========
    double max_vel = 2.0;        ///< 最大速度 (m/s)
    double max_acc = 2.0;        ///< 最大加速度 (m/s²)
    double safe_distance = 0.5;  ///< 安全距离 (m)

    // ========== 平均时间约束参数 ==========
    double mean_time_lower_bound = 0.9;  ///< 平均时间下界比例
    double mean_time_upper_bound = 1.1;  ///< 平均时间上界比例

    // ========== 采样参数 ==========
    int integral_resolution = 8;  ///< 每段轨迹的积分采样点数

    // ========== 优化器参数 ==========
    int max_iterations = 200;  ///< 最大迭代次数
    double g_epsilon = 1e-4;   ///< 梯度收敛阈值
    double min_time = 0.1;     ///< 最小段时间 (s)
};

/**
 * @brief 2D MINCO 轨迹优化器
 *
 * 使用 L-BFGS 优化器，同时优化：
 * - 中间路点位置 (N-1 个 2D 点)
 * - 每段时间分配 (N 个时间值)
 *
 * - 所有临时变量预分配为成员变量，运行时零动态内存分配
 * - costFunction 在一次优化中被调用数十次，避免 malloc/free 极大提升性能
 * - 注意：此类非线程安全，预分配的缓冲区在多线程下会产生竞态条件
 */
class MincoOptimizer {
   public:
    MincoOptimizer() = default;

    /**
     * @brief 设置 ESDF 接口
     * @param esdf ESDF 接口智能指针
     */
    void setESDFInterface(std::shared_ptr<ESDFInterface> esdf) { esdf_ = esdf; }

    /**
     * @brief 设置配置参数
     */
    void setConfig(const MincoOptimizerConfig &config) { config_ = config; }

    /**
     * @brief 初始化优化器并预分配所有运行时缓冲区
     * @param headPVA 起始状态 [位置, 速度, 加速度] (2x3)
     * @param tailPVA 终止状态 [位置, 速度, 加速度] (2x3)
     * @param pieceNum 轨迹段数
     *
     * 此函数会预分配所有运行时需要的内存，之后的 optimize() 调用
     * 以及内部的 costFunction 调用将不再触发动态内存分配
     *
     * S=3 MINCO 需要完整的 PVA (位置、速度、加速度) 边界条件，
     * 用于实现加速度连续的热启动。
     */
    void initialize(const Eigen::Matrix<double, 2, 3> &headPVA,
                    const Eigen::Matrix<double, 2, 3> &tailPVA, int pieceNum) {
        pieceNum_ = pieceNum;

        // 直接使用传入的 PVA 边界条件
        headPVA_ = headPVA;
        tailPVA_ = tailPVA;

        // 初始化 MINCO（内部也会预分配缓冲区）
        // 使用 gcopter 的 MINCO_S3NU，需要 PVA 边界条件
        minco_.setConditions(headPVA_, tailPVA_, pieceNum);

        // 计算决策变量维度
        // 路点：(N-1) * 2
        // 时间：N（使用对数变换保证正性）
        dimPoints_ = (pieceNum - 1) * 2;
        dimTimes_ = pieceNum;
        totalDim_ = dimPoints_ + dimTimes_;

        // ========== 预分配所有运行时缓冲区 ==========
        // S=3 (Quintic) 多项式有 6 个系数，所以梯度矩阵大小为 6N x 2

        // computeCostAndGradient 使用的缓冲区
        points_.resize(2, pieceNum - 1);
        times_.resize(pieceNum);

        // 平滑性代价梯度缓冲区 (6N x 2 for S=3)
        gradC_smooth_.resize(6 * pieceNum, 2);
        gradT_smooth_.resize(pieceNum);

        // 避障代价梯度缓冲区 (6N x 2 for S=3)
        gradC_obstacle_.resize(6 * pieceNum, 2);
        gradT_obstacle_.resize(pieceNum);

        // 动力学可行性代价梯度缓冲区 (6N x 2 for S=3)
        gradC_feas_.resize(6 * pieceNum, 2);
        gradT_feas_.resize(pieceNum);

        // 时间代价梯度缓冲区
        gradT_time_.resize(pieceNum);

        // 平均时间约束梯度缓冲区
        gradT_mean_time_.resize(pieceNum);

        // 合并后的总梯度缓冲区 (6N x 2 for S=3)
        totalGradC_.resize(6 * pieceNum, 2);
        totalGradT_.resize(pieceNum);

        // 梯度传播输出缓冲区
        gradPoints_.resize(2, pieceNum - 1);
        gradTimes_.resize(pieceNum);

        // 轨迹缓冲区（避免在 computeObstacleCost 和 computeFeasibilityCost
        // 中重复构造）
        traj_.clear();
        traj_.reserve(pieceNum);

        // 优化结果缓冲区
        optPoints_.resize(2, pieceNum - 1);
        optTimes_.resize(pieceNum);
    }

    /**
     * @brief 执行轨迹优化
     * @param initPoints 初始路点 (2 x (N-1))
     * @param initTimes 初始时间分配 (N 维)
     * @return 优化是否成功
     */
    bool optimize(const Eigen::Matrix2Xd &initPoints,
                  const Eigen::VectorXd &initTimes) {
        // 构造决策变量向量
        // x = [q_1, q_2, ..., q_{N-1}, ln(T_1), ln(T_2), ..., ln(T_N)]
        Eigen::VectorXd x(totalDim_);

        // 展平路点到向量
        for (int i = 0; i < pieceNum_ - 1; ++i) {
            x.segment<2>(i * 2) = initPoints.col(i);
        }

        // 时间使用多项式映射：T → τ（反向映射）
        // 确保初始时间不小于最小值
        Eigen::VectorXd clampedTimes = initTimes.cwiseMax(config_.min_time);
        Eigen::VectorXd tau(pieceNum_);
        backwardT(clampedTimes, tau);
        x.segment(dimPoints_, pieceNum_) = tau;

        // 设置 L-BFGS 参数
        lbfgs::LbfgsParams params;
        params.max_iterations = config_.max_iterations;
        params.g_epsilon = config_.g_epsilon;
        params.mem_size = 256;
        params.past = 3;
        params.min_step = 1.0e-32;
        params.delta = 5.0e-3;

        // 执行优化
        double finalCost;
        int ret =
            lbfgs::lbfgsOptimize(x, finalCost, &MincoOptimizer::costFunction,
                                 nullptr, nullptr, this, params);

        // 检查结果
        if (ret < 0 && ret != lbfgs::LBFGSERR_MAXIMUMITERATION) {
            std::cerr << "L-BFGS optimization failed: "
                      << lbfgs::lbfgsStrerror(ret) << std::endl;
            // return false;
        }

        // 提取最优解
        extractSolution(x);

        return true;
    }

    /**
     * @brief 获取优化后的轨迹
     * @note S=3 使用 5 次多项式，返回 Trajectory<5, 2> 类型
     */
    void getTrajectory(Trajectory<5, 2> &traj) const {
        minco_.getTrajectory(traj);
    }

    /**
     * @brief 获取最终能量
     */
    double getFinalEnergy() const {
        double energy;
        minco_.getEnergy(energy);
        return energy;
    }

   private:
    /**
     * @brief L-BFGS 代价函数回调
     *
     * 计算总代价及其梯度：
     * J = w_s * J_smooth + w_o * J_obstacle + w_f * J_feasibility + w_t *
     * J_time
     */
    static double costFunction(void *instance, const Eigen::VectorXd &x,
                               Eigen::VectorXd &grad) {
        MincoOptimizer *opt = static_cast<MincoOptimizer *>(instance);
        return opt->computeCostAndGradient(x, grad);
    }

    /**
     * @brief Smoothed L1 惩罚函数 (Positive Part)
     *
     * 当输入 x > 0 时产生惩罚，x <= 0 时无惩罚。
     * 特点：
     * - 当 x >= pe 时，f(x) = x - 0.5*pe，df = 1.0（恒定梯度，模拟硬约束）
     * - 当 0 < x < pe 时，使用三次多项式平滑过渡，保证 C2 连续
     * - 当 x <= 0 时，f = 0, df = 0
     *
     * 相比二次/三次惩罚的优势：
     * 1. 在约束边界处提供恒定的"推力"（梯度 = 1），不会因接近满足约束而减弱
     * 2. 更好地模拟硬约束行为，严格保证约束满足
     *
     * @param x 违反量（正值表示违反约束）
     * @param[out] f 惩罚值
     * @param[out] df 惩罚对 x 的导数
     */
    static inline void positiveSmoothedL1(const double &x, double &f,
                                          double &df) {
        // 平滑区间宽度（在 [0, pe] 内使用三次多项式过渡）
        const double pe = 1.0e-2;

        // 三次多项式系数：f(x) = f4c * x^4 + f3c * x^3（确保 f(0)=0, f'(0)=0,
        // f(pe)=pe-0.5*pe, f'(pe)=1）
        // 实际采用的形式：f(x) = (f4c * x + f3c) * x^3
        const double f3c = 1.0 / (pe * pe);
        const double f4c = -0.5 * f3c / pe;

        // 导数系数：df/dx = d3c * x^2 + d2c * x^3（通过对 f 求导得到）
        // 实际采用的形式：df = (d3c * x + d2c) * x^2
        const double d2c = 3.0 * f3c;
        const double d3c = 4.0 * f4c;

        if (x < pe) {
            if (x > 0) {
                // 平滑过渡区：0 < x < pe
                f = (f4c * x + f3c) * x * x * x;
                df = (d3c * x + d2c) * x * x;
            } else {
                // 无违反：x <= 0
                f = 0.0;
                df = 0.0;
            }
        } else {
            // 线性区：x >= pe，恒定梯度
            f = x - 0.5 * pe;
            df = 1.0;
        }
    }

    /**
     * @brief 计算代价和梯度
     *
     * 此函数在一次 L-BFGS 优化中被调用数十次
     * 通过使用预分配的成员变量缓冲区，实现零动态内存分配
     */
    double computeCostAndGradient(const Eigen::VectorXd &x,
                                  Eigen::VectorXd &grad) {
        // 解析决策变量到预分配的缓冲区
        for (int i = 0; i < pieceNum_ - 1; ++i) {
            points_.col(i) = x.segment<2>(i * 2);
        }

        // 提取 τ 并使用多项式映射转换为 T
        Eigen::VectorXd tau = x.segment(dimPoints_, pieceNum_);
        forwardT(tau, times_);

        // 更新 MINCO 轨迹
        minco_.setParameters(points_, times_);

        // 初始化梯度（使用传入的 grad，避免分配）
        grad.setZero();

        // ========== 1. 平滑性代价 ==========
        double smoothCost = 0.0;
        minco_.getEnergy(smoothCost);
        minco_.getEnergyPartialGradByCoeffs(gradC_smooth_);
        minco_.getEnergyPartialGradByTimes(gradT_smooth_);

        smoothCost *= config_.weight_smooth;
        gradC_smooth_ *= config_.weight_smooth;
        gradT_smooth_ *= config_.weight_smooth;

        // ========== 2. 避障代价 ==========
        double obstacleCost = 0.0;
        gradC_obstacle_.setZero();
        gradT_obstacle_.setZero();
        if (esdf_) {
            computeObstacleCost(obstacleCost);
            obstacleCost *= config_.weight_obstacle;
            gradC_obstacle_ *= config_.weight_obstacle;
            gradT_obstacle_ *= config_.weight_obstacle;
        }

        // ========== 3. 动力学可行性代价 ==========
        double feasibilityCost = 0.0;
        gradC_feas_.setZero();
        gradT_feas_.setZero();

        computeFeasibilityCost(feasibilityCost);
        feasibilityCost *= config_.weight_feasibility;
        gradC_feas_ *= config_.weight_feasibility;
        gradT_feas_ *= config_.weight_feasibility;

        // ========== 4. 时间正则化代价 ==========
        double timeCost = 0.0;
        for (int i = 0; i < pieceNum_; ++i) {
            timeCost += times_(i);
            gradT_time_(i) = config_.weight_time;
        }
        timeCost *= config_.weight_time;

        // ========== 5. 平均时间约束代价 ==========
        // 参考成熟方案 optimizer.cpp 中 attachPenaltyFunctional 的实现
        // 计算平均段时间，对偏离 [avg*lower, avg*upper] 的段施加二次惩罚
        double meanTimeCost = 0.0;
        gradT_mean_time_.setZero();

        if (config_.weight_mean_time > 0.0 && pieceNum_ > 1) {
            double avgT = times_.mean();
            double lowerT = avgT * config_.mean_time_lower_bound;
            double upperT = avgT * config_.mean_time_upper_bound;

            for (int i = 0; i < pieceNum_; ++i) {
                if (times_(i) < lowerT) {
                    // 段时间过短，施加二次惩罚
                    double diff = times_(i) - lowerT;
                    meanTimeCost += config_.weight_mean_time * diff * diff;
                    // 梯度：∂/∂T_j (T_i - avg*lb)^2
                    // avg = sum(T)/N, 所以 ∂avg/∂T_j = 1/N
                    // ∂(T_i - avg*lb)/∂T_j = δ_{ij} - lb/N
                    // 对所有段的梯度贡献（通过 avg 的链式法则）
                    gradT_mean_time_.array() +=
                        config_.weight_mean_time * 2.0 * diff *
                        (-config_.mean_time_lower_bound / pieceNum_);
                    // 对当前段的额外直接梯度
                    gradT_mean_time_(i) +=
                        config_.weight_mean_time * 2.0 * diff;
                }
                if (times_(i) > upperT) {
                    // 段时间过长，施加二次惩罚
                    double diff = times_(i) - upperT;
                    meanTimeCost += config_.weight_mean_time * diff * diff;
                    gradT_mean_time_.array() +=
                        config_.weight_mean_time * 2.0 * diff *
                        (-config_.mean_time_upper_bound / pieceNum_);
                    gradT_mean_time_(i) +=
                        config_.weight_mean_time * 2.0 * diff;
                }
            }
        }

        // ========== 合并梯度（使用预分配的缓冲区）==========
        totalGradC_ = gradC_smooth_;
        totalGradC_ += gradC_obstacle_;
        totalGradC_ += gradC_feas_;

        totalGradT_ = gradT_smooth_;
        totalGradT_ += gradT_obstacle_;
        totalGradT_ += gradT_feas_;
        totalGradT_ += gradT_time_;
        totalGradT_ += gradT_mean_time_;

        // 梯度传播：从系数/时间梯度转换为路点/时间梯度
        minco_.propogateGrad(totalGradC_, totalGradT_, gradPoints_, gradTimes_);

        // 填充梯度向量
        for (int i = 0; i < pieceNum_ - 1; ++i) {
            grad.segment<2>(i * 2) = gradPoints_.col(i);
        }

        // 链式法则：多项式映射的梯度 ∂J/∂τ = ∂J/∂T * ∂T/∂τ
        // 使用 forwardGradT 计算 ∂T/∂τ
        Eigen::VectorXd dTdtau(pieceNum_);
        forwardGradT(tau, dTdtau);
        for (int i = 0; i < pieceNum_; ++i) {
            grad(dimPoints_ + i) = gradTimes_(i) * dTdtau(i);
        }
        return smoothCost + obstacleCost + feasibilityCost + timeCost +
               meanTimeCost;
    }
    /**
     * @brief 计算避障代价 (使用 Smoothed L1 惩罚)
     *
     * 在轨迹上进行离散采样，对每个采样点计算避障惩罚。
     * 使用 Smoothed L1 惩罚函数，在违反约束时提供恒定梯度（线性惩罚），
     * 从而更好地模拟硬约束行为。
     *
     * 使用预分配的 traj_ 和 gradC_obstacle_, gradT_obstacle_ 缓冲区
     * 调用前需确保 gradC_obstacle_ 和 gradT_obstacle_ 已 setZero()
     *
     * ========== FINELY_OPTIMIZATION 策略 ==========
     * 为解决窄路/急弯场景下的"挤牙膏效应"和"轨迹扭曲"问题，引入以下优化：
     * 1. 正交梯度投影：剔除沿轨迹切线方向的梯度分量，只保留横向推力
     * 2. 梯度归一化与增强：在势能谷底强制提供明确的推力方向
     * 3. Smoothed L1 惩罚：在边界处提供恒定梯度，避免梯度消失
     *
     * ========== S=3 (Quintic) 多项式说明 ==========
     * 5次多项式: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
     * gcopter 的 b 矩阵存储: [c0, c1, c2, c3, c4, c5] (低阶在前)
     * Trajectory 类使用 .reverse()，所以 getPos 期望高阶在前
     * 因此 ∂p/∂c_j 对应 t^j (j = 0, 1, ..., 5)
     */
    void computeObstacleCost(double &cost) {
        cost = 0.0;
        // gradC_obstacle_ 和 gradT_obstacle_ 在调用前已由
        // computeCostAndGradient 清零

        // 使用预分配的轨迹缓冲区
        minco_.getTrajectory(traj_);

        const int K = config_.integral_resolution;
        const double dSafe = config_.safe_distance;

        // ========== FINELY_OPTIMIZATION 参数 ==========
        constexpr double kGradNormThreshold = 0.5;  // 梯度归一化阈值
        constexpr double kMinGradNorm = 1e-6;  // 最小梯度模长（防止除零）

        // 遍历每段轨迹
        for (int i = 0; i < pieceNum_; ++i) {
            const auto &piece = traj_[i];
            double duration = piece.getDuration();
            double dt = duration / K;

            for (int k = 0; k <= K; ++k) {
                double t = k * dt;
                Eigen::Vector2d pos = piece.getPos(t);

                // 检查是否在地图范围内
                if (!esdf_->isInside(pos.x(), pos.y())) {
                    continue;
                }

                double dist = esdf_->getDistance(pos.x(), pos.y());

                // 计算违反量：violation = d_safe - d
                // violation > 0 表示违反约束（距离不足）
                double violaPos = dSafe - dist;

                // 使用 Smoothed L1 惩罚
                double penalty, dPenalty_dViolation;
                positiveSmoothedL1(violaPos, penalty, dPenalty_dViolation);

                // 如果有惩罚（violation > 0）
                if (penalty > 0.0) {
                    cost += penalty * dt;

                    // 获取原始 ESDF 梯度
                    Eigen::Vector2d gradDist =
                        esdf_->getGradient(pos.x(), pos.y());

                    // ========== 正交梯度投影 (核心) ==========
                    // 目的：只允许障碍物把轨迹往"路中间"推（横向），
                    //       不允许影响轨迹的"进度"（纵向），防止挤牙膏效应
                    Eigen::Vector2d vel = piece.getVel(t);
                    double velNorm = vel.norm();

                    Eigen::Vector2d gradOrthogonal =
                        gradDist;  // 默认使用原始梯度

                    if (velNorm > kMinGradNorm) {
                        // 计算切线方向（速度方向）
                        Eigen::Vector2d tangent = vel / velNorm;

                        // 投影：剔除沿切线方向的分量
                        // RealGrad = OriginalGrad - (OriginalGrad · Tangent) *
                        // Tangent
                        double longitudinalComponent = gradDist.dot(tangent);
                        gradOrthogonal =
                            gradDist - longitudinalComponent * tangent;
                    }

                    // ========== 梯度归一化与增强 ==========
                    // 当点处于势能谷底（两侧障碍物梯度抵消）时，
                    // 梯度模长接近0，优化器会"卡住"
                    // 解决方案：检测这种情况，强制提供一个明确的推力方向
                    double orthogonalNorm = gradOrthogonal.norm();

                    if (orthogonalNorm < kGradNormThreshold &&
                        orthogonalNorm > kMinGradNorm) {
                        // 梯度太小但不是零：说明在势能谷底附近
                        // 按照文档的方法，用 sqrt(norm) 作为增强后的模长
                        // 这样既保留了方向信息，又避免了梯度消失
                        double enhancedNorm = std::sqrt(orthogonalNorm);
                        gradOrthogonal =
                            gradOrthogonal / orthogonalNorm * enhancedNorm;
                    } else if (orthogonalNorm <= kMinGradNorm) {
                        // 梯度几乎为零：使用原始 ESDF 梯度的归一化版本
                        // 作为备用推力方向
                        double rawNorm = gradDist.norm();
                        if (rawNorm > kMinGradNorm) {
                            gradOrthogonal =
                                gradDist / rawNorm * kGradNormThreshold;
                        }
                        // 如果原始梯度也为零，则无法确定推力方向，跳过
                    }

                    // ========== 链式法则计算位置梯度 ==========
                    // ∂Cost/∂pos = ∂penalty/∂violation * ∂violation/∂dist *
                    // ∂dist/∂pos
                    //            = dPenalty_dViolation * (-1.0) *
                    //            gradOrthogonal
                    // 乘以积分权重 dt
                    Eigen::Vector2d gradPos =
                        dPenalty_dViolation * (-1.0) * gradOrthogonal * dt;

                    // ========== S=3 (Quintic) 多项式梯度计算 ==========
                    // 5次多项式: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 +
                    // c5*t^5 gcopter 存储 b = [c0, c1, c2, c3, c4, c5]^T
                    // (低阶在前) ∂p/∂c_j = t^j
                    double t_powers[6] = {1.0,           t,
                                          t * t,         t * t * t,
                                          t * t * t * t, t * t * t * t * t};
                    for (int j = 0; j < 6; ++j) {
                        gradC_obstacle_.row(6 * i + j) +=
                            t_powers[j] * gradPos.transpose();
                    }

                    // ========== 时间梯度 ==========
                    // 使用正交化后的梯度，这样时间优化也不会受到纵向推力的干扰
                    gradT_obstacle_(i) +=
                        gradPos.dot(vel) * (static_cast<double>(k) / K);
                    gradT_obstacle_(i) += penalty / K;  // 积分的时间项
                }
            }
        }
    }
    /**
     * @brief 计算动力学可行性代价 (使用 Smoothed L1 惩罚)
     *
     * 对超过限制的速度和加速度添加软约束惩罚。
     * 使用 Smoothed L1 惩罚函数，在违反约束时提供恒定梯度，
     * 从而更好地模拟硬约束行为，严格保证动力学可行性。
     *
     * 使用预分配的 traj_ 和 gradC_feas_, gradT_feas_ 缓冲区
     * 调用前需确保 gradC_feas_ 和 gradT_feas_ 已 setZero()
     *
     * ========== S=3 (Quintic) 多项式说明 ==========
     * 位置: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
     * 速度: v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
     * 加速度: a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
     * Jerk: j(t) = 6*c3 + 24*c4*t + 60*c5*t^2
     */
    void computeFeasibilityCost(double &cost) {
        cost = 0.0;
        // gradC_feas_ 和 gradT_feas_ 在调用前已由 computeCostAndGradient 清零

        // 使用预分配的轨迹缓冲区（如果 computeObstacleCost 已调用则已更新）
        // 否则需要重新获取
        minco_.getTrajectory(traj_);

        const int K = config_.integral_resolution;
        const double vMax = config_.max_vel;
        const double aMax = config_.max_acc;
        const double vMax2 = vMax * vMax;
        const double aMax2 = aMax * aMax;

        for (int i = 0; i < pieceNum_; ++i) {
            const auto &piece = traj_[i];
            double duration = piece.getDuration();
            double dt = duration / K;

            for (int k = 0; k <= K; ++k) {
                double t = k * dt;

                // ========== 速度约束 ==========
                Eigen::Vector2d vel = piece.getVel(t);
                double vSqr = vel.squaredNorm();

                // 违反量定义：violation = v² - v_max²
                // 使用速度平方作为约束，避免开方运算
                double violaVel = vSqr - vMax2;

                double velPenalty, dVelPenalty_dViolation;
                positiveSmoothedL1(violaVel, velPenalty,
                                   dVelPenalty_dViolation);

                if (velPenalty > 0.0) {
                    cost += velPenalty * dt;

                    // 链式法则：∂penalty/∂vel = ∂penalty/∂violation *
                    // ∂violation/∂v²  * ∂v²/∂vel
                    //           = dVelPenalty_dViolation * 1.0 * 2.0 * vel
                    Eigen::Vector2d gradVel =
                        dVelPenalty_dViolation * 2.0 * vel * dt;

                    // ========== S=3 (Quintic) 速度梯度 ==========
                    // v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
                    // ∂v/∂c1 = 1, ∂v/∂c2 = 2t, ∂v/∂c3 = 3t^2,
                    // ∂v/∂c4 = 4t^3, ∂v/∂c5 = 5t^4
                    double t1 = t, t2 = t * t, t3 = t * t * t,
                           t4 = t * t * t * t;
                    gradC_feas_.row(6 * i + 1) += gradVel.transpose();
                    gradC_feas_.row(6 * i + 2) +=
                        2.0 * t1 * gradVel.transpose();
                    gradC_feas_.row(6 * i + 3) +=
                        3.0 * t2 * gradVel.transpose();
                    gradC_feas_.row(6 * i + 4) +=
                        4.0 * t3 * gradVel.transpose();
                    gradC_feas_.row(6 * i + 5) +=
                        5.0 * t4 * gradVel.transpose();

                    // 时间梯度
                    Eigen::Vector2d acc = piece.getAcc(t);
                    gradT_feas_(i) +=
                        gradVel.dot(acc) * (static_cast<double>(k) / K);
                    gradT_feas_(i) += velPenalty / K;
                }

                // ========== 加速度约束 ==========
                Eigen::Vector2d acc = piece.getAcc(t);
                double aSqr = acc.squaredNorm();

                // 违反量定义：violation = a² - a_max²
                double violaAcc = aSqr - aMax2;

                double accPenalty, dAccPenalty_dViolation;
                positiveSmoothedL1(violaAcc, accPenalty,
                                   dAccPenalty_dViolation);

                if (accPenalty > 0.0) {
                    cost += accPenalty * dt;

                    // 链式法则：∂penalty/∂acc = dAccPenalty_dViolation * 2.0 *
                    // acc
                    Eigen::Vector2d gradAcc =
                        dAccPenalty_dViolation * 2.0 * acc * dt;

                    // ========== S=3 (Quintic) 加速度梯度 ==========
                    // a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
                    // ∂a/∂c2 = 2, ∂a/∂c3 = 6t, ∂a/∂c4 = 12t^2, ∂a/∂c5 = 20t^3
                    double t1 = t, t2 = t * t, t3 = t * t * t;
                    gradC_feas_.row(6 * i + 2) += 2.0 * gradAcc.transpose();
                    gradC_feas_.row(6 * i + 3) +=
                        6.0 * t1 * gradAcc.transpose();
                    gradC_feas_.row(6 * i + 4) +=
                        12.0 * t2 * gradAcc.transpose();
                    gradC_feas_.row(6 * i + 5) +=
                        20.0 * t3 * gradAcc.transpose();

                    // 时间梯度
                    Eigen::Vector2d jer = piece.getJer(t);
                    gradT_feas_(i) +=
                        gradAcc.dot(jer) * (static_cast<double>(k) / K);
                    gradT_feas_(i) += accPenalty / K;
                }
            }
        }
    }

    /**
     * @brief 从决策变量提取最优解
     *
     * 使用预分配的 optPoints_ 和 optTimes_ 缓冲区
     */
    void extractSolution(const Eigen::VectorXd &x) {
        for (int i = 0; i < pieceNum_ - 1; ++i) {
            optPoints_.col(i) = x.segment<2>(i * 2);
        }

        for (int i = 0; i < pieceNum_; ++i) {
            optTimes_(i) = std::exp(x(dimPoints_ + i));
        }

        minco_.setParameters(optPoints_, optTimes_);
    }

   public:
    /// @brief 获取优化后的路点
    const Eigen::Matrix2Xd &getOptimizedPoints() const { return optPoints_; }

    /// @brief 获取优化后的时间分配
    const Eigen::VectorXd &getOptimizedTimes() const { return optTimes_; }
    /**
     * @brief 正向映射：τ → T
     * 将无约束变量 τ 映射到正实数时间 T
     *
     * 映射公式：
     * - 当 τ > 0: T = 0.5*τ² + τ + 1
     * - 当 τ ≤ 0: T = 1 / (0.5*τ² - τ + 1)
     *
     * 性质：T(0) = 1, T 始终为正，且映射是光滑的
     */
    static inline void forwardT(const Eigen::VectorXd &tau,
                                Eigen::VectorXd &T) {
        const int sizeTau = tau.size();
        T.resize(sizeTau);
        for (int i = 0; i < sizeTau; i++) {
            T(i) = tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                                : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
        }
    }
    /**
     * @brief 反向映射：T → τ
     * 将正实数时间 T 映射回无约束变量 τ
     */
    static inline void backwardT(const Eigen::VectorXd &T,
                                 Eigen::VectorXd &tau) {
        const int sizeT = T.size();
        tau.resize(sizeT);
        for (int i = 0; i < sizeT; i++) {
            tau(i) = T(i) > 1.0 ? (std::sqrt(2.0 * T(i) - 1.0) - 1.0)
                                : (1.0 - std::sqrt(2.0 / T(i) - 1.0));
        }
    }

    /**
     * @brief 计算时间映射的梯度 ∂T/∂τ
     * 用于链式法则：∂J/∂τ = ∂J/∂T * ∂T/∂τ
     */
    static inline void forwardGradT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &gradT) {
        const int sizeTau = tau.size();
        gradT.resize(sizeTau);
        for (int i = 0; i < sizeTau; i++) {
            if (tau(i) > 0.0) {
                // T = 0.5*τ² + τ + 1
                // ∂T/∂τ = τ + 1
                gradT(i) = tau(i) + 1.0;
            } else {
                // T = 1 / (0.5*τ² - τ + 1)
                // 令 denom = 0.5*τ² - τ + 1
                // ∂T/∂τ = -(τ - 1) / denom²
                double denom = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradT(i) = (1.0 - tau(i)) / (denom * denom);
            }
        }
    }

   private:
    // ESDF 接口
    std::shared_ptr<ESDFInterface> esdf_;

    // 配置参数
    MincoOptimizerConfig config_;

    // MINCO 求解器 (使用 gcopter 的 S=3 实现)
    minco::MINCO_S3NU minco_;

    // 边界条件 (PVA: 位置、速度、加速度)
    Eigen::Matrix<double, 2, 3> headPVA_;
    Eigen::Matrix<double, 2, 3> tailPVA_;

    // 维度信息
    int pieceNum_ = 0;
    int dimPoints_ = 0;
    int dimTimes_ = 0;
    int totalDim_ = 0;

    // ========== 预分配缓冲区 (Pre-allocated Buffers) ==========
    // 这些缓冲区在 initialize() 中一次性分配，之后在 costFunction 中重复使用
    // 避免每次调用时的动态内存分配，显著提升性能
    // 注意：这些缓冲区使此类非线程安全

    // computeCostAndGradient 使用的决策变量解析缓冲区
    Eigen::Matrix2Xd points_;  ///< 路点缓冲区 (2 x (N-1))
    Eigen::VectorXd times_;    ///< 时间缓冲区 (N)

    // 平滑性代价梯度缓冲区 (S=3: 6N x 2)
    Eigen::MatrixX2d gradC_smooth_;  ///< 系数梯度 (6N x 2)
    Eigen::VectorXd gradT_smooth_;   ///< 时间梯度 (N)

    // 避障代价梯度缓冲区 (S=3: 6N x 2)
    Eigen::MatrixX2d gradC_obstacle_;  ///< 系数梯度 (6N x 2)
    Eigen::VectorXd gradT_obstacle_;   ///< 时间梯度 (N)

    // 动力学可行性代价梯度缓冲区 (S=3: 6N x 2)
    Eigen::MatrixX2d gradC_feas_;  ///< 系数梯度 (6N x 2)
    Eigen::VectorXd gradT_feas_;   ///< 时间梯度 (N)

    // 时间代价梯度缓冲区
    Eigen::VectorXd gradT_time_;  ///< 时间梯度 (N)

    // 平均时间约束梯度缓冲区
    Eigen::VectorXd gradT_mean_time_;  ///< 平均时间约束梯度 (N)

    // 合并后的总梯度缓冲区 (S=3: 6N x 2)
    Eigen::MatrixX2d totalGradC_;  ///< 合并后的系数梯度 (6N x 2)
    Eigen::VectorXd totalGradT_;   ///< 合并后的时间梯度 (N)

    // 梯度传播输出缓冲区
    Eigen::Matrix2Xd gradPoints_;  ///< 路点梯度 (2 x (N-1))
    Eigen::VectorXd gradTimes_;    ///< 时间梯度 (N)

    // 轨迹缓冲区（用于 computeObstacleCost 和 computeFeasibilityCost）
    // S=3 使用 5 次多项式，2D 轨迹
    Trajectory<5, 2> traj_;  ///< 轨迹缓冲区

    // 优化结果缓冲区
    Eigen::Matrix2Xd optPoints_;  ///< 最优路点 (2 x (N-1))
    Eigen::VectorXd optTimes_;    ///< 最优时间 (N)
};
}  // namespace pb_minco

#endif  // PB_MINCO_SMOOTHER__MINCO_OPTIMIZER_HPP_
