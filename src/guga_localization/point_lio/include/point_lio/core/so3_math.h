#pragma once

#include <cmath>

#include <Eigen/Core>

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3) {
  const T norm = std::sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  const Eigen::Matrix<T, 3, 3> identity =
      Eigen::Matrix<T, 3, 3>::Identity();
  if (norm <= 0.00001) {
    return identity;
  }

  const T axis[3] = {v1 / norm, v2 / norm, v3 / norm};
  Eigen::Matrix<T, 3, 3> skew;
  skew << SKEW_SYM_MATRX(axis);
  return identity + std::sin(norm) * skew
         + (1.0 - std::cos(norm)) * skew * skew;
}
