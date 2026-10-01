// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__POSE_PENALTY_FACTOR_HPP_
#define SMALL_GICP_RELOCALIZATION__POSE_PENALTY_FACTOR_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace small_gicp_relocalization
{

/// @brief In-iteration regularizer for ground-robot relocalization.
///
/// Two soft constraints are injected into the linearized system:
///  1. A DoF restriction (large weight on masked axes, e.g. tz / roll / pitch)
///     that keeps those axes at the value of the reference pose.
///  2. A pose penalty that pulls the estimate back toward the reference pose
///     (the last trusted pose), so a degenerate solve cannot drift away.
///
/// The penalty is applied to H and b only; `update_error` stays a no-op so the
/// LM step validation keeps comparing the pure GICP error, matching the
/// behavior of small_gicp::RestrictDoFFactor.
struct PosePenaltyFactor
{
  PosePenaltyFactor() = default;

  void set_reference(const Eigen::Isometry3d & reference) { reference_ = reference; }
  void set_dof_restriction_mask(const Eigen::Matrix<double, 6, 1> & mask)
  {
    dof_restriction_mask_ = mask;
  }
  void set_dof_restriction_weight(double weight) { dof_restriction_weight_ = weight; }
  void set_pose_penalty_weight(double weight) { pose_penalty_weight_ = weight; }

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree>
  void update_linearized_system(
    const TargetPointCloud & target, const SourcePointCloud & source,
    const TargetTree & target_tree, const Eigen::Isometry3d & T, Eigen::Matrix<double, 6, 6> * H,
    Eigen::Matrix<double, 6, 1> * b, double * e) const
  {
    (void)target;
    (void)source;
    (void)target_tree;
    (void)e;

    if (dof_restriction_weight_ > 0.0) {
      (*H) += dof_restriction_weight_ * dof_restriction_mask_.asDiagonal();
    }

    if (pose_penalty_weight_ > 0.0) {
      (*H) += pose_penalty_weight_ * Eigen::Matrix<double, 6, 6>::Identity();
      (*b) += pose_penalty_weight_ * deviation(reference_, T);
    }
  }

  template <typename TargetPointCloud, typename SourcePointCloud>
  void update_error(
    const TargetPointCloud & target, const SourcePointCloud & source, const Eigen::Isometry3d & T,
    double * e) const
  {
    (void)target;
    (void)source;
    (void)T;
    (void)e;
  }

private:
  /// @brief Pose deviation [rx, ry, rz, tx, ty, tz] of T with respect to reference.
  static Eigen::Matrix<double, 6, 1> deviation(
    const Eigen::Isometry3d & reference, const Eigen::Isometry3d & T)
  {
    Eigen::Matrix<double, 6, 1> residual;
    const Eigen::AngleAxisd rotation_error(reference.rotation().transpose() * T.rotation());
    residual.head<3>() = rotation_error.angle() * rotation_error.axis();
    residual.tail<3>() = T.translation() - reference.translation();
    return residual;
  }

  Eigen::Isometry3d reference_{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 1> dof_restriction_mask_{Eigen::Matrix<double, 6, 1>::Zero()};
  double dof_restriction_weight_{0.0};
  double pose_penalty_weight_{0.0};
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__POSE_PENALTY_FACTOR_HPP_
