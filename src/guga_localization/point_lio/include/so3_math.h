/**
 * @file so3_math.h
 * @brief SO(3) 旋转群上的数学工具函数
 * @author HKU-MARS
 *
 * 提供李群 SO(3) 与李代数 so(3) 之间的映射:
 * - Exp: 轴角 → 旋转矩阵 (Rodrigues 公式)
 * - Log: 旋转矩阵 → 轴角
 * - 反对称矩阵 (skew-symmetric) 构造
 * - 旋转矩阵 → 欧拉角
 * - SO(3) 右雅可比逆
 *
 * 所有函数均为模板，支持 float/double 类型。
 */

#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>
#include <Eigen/Core>

/**
 * @def SKEW_SYM_MATRX(v)
 * @brief 生成 3维向量 v 的反对称矩阵的 9 个元素序列
 *
 * 反对称矩阵 [v]×:
 *   |  0  -v2   v1 |
 *   |  v2   0  -v0 |
 *   | -v1   v0   0 |
 *
 * 用于 Eigen 的逗号初始化: K << SKEW_SYM_MATRX(vec);
 */
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

/**
 * @brief 构造 3维向量的反对称矩阵
 * @tparam T 标量类型
 * @param v 3维向量
 * @return 3x3 反对称矩阵 [v]×
 */
template<typename T>
Eigen::Matrix<T, 3, 3> skew_sym_mat(const Eigen::Matrix<T, 3, 1> &v)
{
    Eigen::Matrix<T, 3, 3> skew_sym_mat;
    skew_sym_mat<<SKEW_SYM_MATRX(v);
    return skew_sym_mat;
}

/**
 * @brief SO(3) 指数映射: 轴角向量 → 旋转矩阵 (Rodrigues 公式)
 *
 * R = I + sin(θ)·K + (1-cos(θ))·K²
 * 其中 θ = |ang|, K = [ang/θ]×
 *
 * @tparam T 标量类型
 * @param ang 轴角向量 (方向=旋转轴, 模=旋转角 rad)
 * @return 3x3 旋转矩阵 R ∈ SO(3)
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang)
{
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_axis);
        /// Rodrigues 变换
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        // 角度趋零时直接返回单位矩阵
        return Eye3;
    }
}

/**
 * @brief SO(3) 指数映射 (带时间步长): 角速度 × 时间 → 旋转矩阵
 *
 * 用于 IMU 预积分中的旋转增量:
 * R(Δt) = Exp(ω·Δt)
 *
 * @tparam T 标量类型 (角速度)
 * @tparam Ts 时间标量类型
 * @param ang_vel 角速度向量 (rad/s)
 * @param dt 时间步长 (s)
 * @return 3x3 旋转矩阵 R(Δt)
 */
template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Rodrigues 变换
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/**
 * @brief SO(3) 指数映射 (标量版本): 三个轴角分量 → 旋转矩阵
 *
 * 等价于 Exp([v1, v2, v3]^T)
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_ang);

        /// Rodrigues 变换
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/**
 * @brief SO(3) 对数映射: 旋转矩阵 → 轴角向量
 *
 * 通过旋转矩阵的迹计算旋转角 θ:
 * θ = arccos((tr(R)-1)/2)
 * 轴 = (R - R^T)^∨ / (2sinθ)
 *
 * @tparam T 标量类型
 * @param R 3x3 旋转矩阵
 * @return 3维轴角向量 (方向=旋转轴, 模=旋转角)
 */
template<typename T>
Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T, 3, 3> R)
{
    // 通过迹计算旋转角: tr(R) = 1 + 2cos(θ)
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    // 从反对称部分提取旋转轴: (R - R^T)^∨
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    // 小角度时退化为 0.5·K (sin(θ)/θ → 1)
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

/**
 * @brief 旋转矩阵 → ZYX 欧拉角 (yaw-pitch-roll)
 *
 * 解算顺序: R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * 返回 [roll, pitch, yaw]^T
 *
 * @tparam T 标量类型
 * @param rot 3x3 旋转矩阵
 * @return 3维欧拉角 (roll, pitch, yaw) [rad]
 */
template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 1> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;  // 万向节死锁判断
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    }
    else
    {
        // 万向节死锁: 设 yaw = 0
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

/**
 * @brief SO(3) 右雅可比逆矩阵
 *
 * 用途: 在流形上的 EKF 更新中，将 tangent space 的增量映射回 SO(3)
 * J_r^{-1}(φ) = I + 0.5·[φ]× + (1 - θ·cos(θ/2)/(2·sin(θ/2)))·[φ]×²/θ²
 *
 * @param vec 轴角向量 φ
 * @return 3x3 右雅可比逆矩阵 J_r^{-1}(φ)
 */
template<typename T>
Eigen::Matrix3d Jacob_right_inv(Eigen::Vector3d &vec){
    Eigen::Matrix3d hat_v, res;
    hat_v << SKEW_SYM_MATRX(vec);
    if(vec.norm() > 1e-6)
    {
        res = Eigen::Matrix<double, 3, 3>::Identity() + 0.5 * hat_v + (1 - vec.norm() * std::cos(vec.norm() / 2) / 2 / std::sin(vec.norm() / 2)) * hat_v * hat_v / vec.squaredNorm();
    }
    else
    {
        // 角度趋零时退化为单位矩阵
        res = Eigen::Matrix<double, 3, 3>::Identity();
    }
    return res;
}

#endif