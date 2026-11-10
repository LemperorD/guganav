#pragma once

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
