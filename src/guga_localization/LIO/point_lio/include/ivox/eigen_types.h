//
// Created by xiang on 2021/7/16.
//

#ifndef FASTER_LIO_EIGEN_TYPES_H
#define FASTER_LIO_EIGEN_TYPES_H

#include <cstddef>

#include <Eigen/Core>

namespace faster_lio {

  /// hash of vector
  template <int N>
  struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
  };

  /// vec 2 hash
  /// @see Optimized Spatial Hashing for Collision Detection of Deformable
  /// Objects, Matthias Teschner et. al., VMV 2003
  template <>
  inline size_t hash_vec<2>::operator()(
      const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943)) % 10000000;
  }

  /// vec 3 hash
  template <>
  inline size_t hash_vec<3>::operator()(
      const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791))
           % 10000000;
  }
}  // namespace faster_lio

#endif
