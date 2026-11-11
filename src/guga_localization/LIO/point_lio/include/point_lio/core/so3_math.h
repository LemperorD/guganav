#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

/// @brief 构造三维向量的反对称矩阵, 满足 Skew(v) * u == v.cross(u).
template <typename T>
Eigen::Matrix<T, 3, 3> Hat(const Eigen::Matrix<T, 3, 1>& v) {
  Eigen::Matrix<T, 3, 3> m;
  m << T(0), -v(2), v(1), v(2), T(0), -v(0), -v(1), v(0), T(0);
  return m;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3) {
  const T norm = std::sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  const Eigen::Matrix<T, 3, 3> identity = Eigen::Matrix<T, 3, 3>::Identity();
  if (norm <= 0.00001) {
    return identity;
  }

  const Eigen::Matrix<T, 3, 1> axis(v1 / norm, v2 / norm, v3 / norm);
  const Eigen::Matrix<T, 3, 3> skew = Hat(axis);
  return identity + std::sin(norm) * skew
         + (1.0 - std::cos(norm)) * skew * skew;
}

inline Eigen::Matrix3d Exp(const Eigen::Vector3d& v) {
  return Exp(v(0), v(1), v(2));
}

/// @brief 旋转矩阵转旋转向量, Exp 的逆.
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> Log(
    const Eigen::MatrixBase<Derived>& R) {
  using T = typename Derived::Scalar;
  const T cos_theta =
      std::clamp((R.trace() - T(1)) * T(0.5), T(-1), T(1));
  const T theta = std::acos(cos_theta);
  const Eigen::Matrix<T, 3, 1> vee(R(2, 1), R(0, 2), R(1, 0));
  if (theta < T(1e-8)) {
    return T(0.5) * vee;
  }
  return theta / (T(2) * std::sin(theta)) * vee;
}
