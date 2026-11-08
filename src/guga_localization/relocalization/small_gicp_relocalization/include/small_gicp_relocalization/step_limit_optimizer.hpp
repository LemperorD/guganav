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

#ifndef SMALL_GICP_RELOCALIZATION__STEP_LIMIT_OPTIMIZER_HPP_
#define SMALL_GICP_RELOCALIZATION__STEP_LIMIT_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <small_gicp/registration/registration_result.hpp>
#include <small_gicp/util/lie.hpp>
#include <vector>

namespace small_gicp_relocalization
{

/// @brief Levenberg-Marquardt optimizer that clamps the update of each iteration.
///
/// The update is delta = [rx, ry, rz, tx, ty, tz] (rotation-first, applied as
/// T <- T * exp(delta)). Before it is applied:
///  - the roll/pitch part (rx, ry) is capped by its 2-D norm, and
///  - tz is capped per component.
/// This bounds how far a single iteration may tilt or lift the pose. The clamp
/// only limits the *step*, so the reachable total is at most max_iterations times
/// the cap; set the caps to 0 to disable.
struct StepLimitOptimizer
{
  StepLimitOptimizer()
  : max_iterations(100),
    max_inner_iterations(10),
    init_lambda(1e-3),
    lambda_factor(10.0),
    max_roll_pitch_step(0.05),
    max_tz_step(0.02)
  {
  }

  template <
    typename TargetPointCloud, typename SourcePointCloud, typename TargetTree,
    typename CorrespondenceRejector, typename TerminationCriteria, typename Reduction,
    typename Factor, typename GeneralFactor>
  small_gicp::RegistrationResult optimize(
    const TargetPointCloud & target, const SourcePointCloud & source,
    const TargetTree & target_tree, const CorrespondenceRejector & rejector,
    const TerminationCriteria & criteria, Reduction & reduction, const Eigen::Isometry3d & init_T,
    std::vector<Factor> & factors, GeneralFactor & general_factor) const
  {
    double lambda = init_lambda;
    small_gicp::RegistrationResult result(init_T);

    for (int i = 0; i < max_iterations && !result.converged; i++) {
      auto [H, b, e] =
        reduction.linearize(target, source, target_tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(
        target, source, target_tree, result.T_target_source, &H, &b, &e);

      bool success = false;
      for (int j = 0; j < max_inner_iterations; j++) {
        Eigen::Matrix<double, 6, 1> delta =
          (H + lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(-b);
        clampStep(delta);

        const Eigen::Isometry3d new_T = result.T_target_source * small_gicp::se3_exp(delta);
        const double new_e = reduction.error(target, source, new_T, factors);
        general_factor.update_error(target, source, new_T, &e);

        if (new_e <= e) {
          result.converged = criteria.converged(delta);
          result.T_target_source = new_T;
          lambda /= lambda_factor;
          success = true;
          e = new_e;
          break;
        }
        lambda *= lambda_factor;
      }

      result.iterations = i;
      result.H = H;
      result.b = b;
      result.error = e;

      if (!success) {
        break;
      }
    }

    result.num_inliers = std::count_if(
      factors.begin(), factors.end(), [](const auto & factor) { return factor.inlier(); });
    return result;
  }

  int max_iterations;          ///< Max number of outer iterations
  int max_inner_iterations;    ///< Max number of lambda-trial iterations
  double init_lambda;          ///< Initial damping
  double lambda_factor;        ///< Damping increase factor
  double max_roll_pitch_step;  ///< Max norm of (rx, ry) per iteration [rad], 0 disables
  double max_tz_step;          ///< Max |tz| per iteration [m], 0 disables

private:
  void clampStep(Eigen::Matrix<double, 6, 1> & delta) const
  {
    if (max_roll_pitch_step > 0.0) {
      const double roll_pitch_norm = delta.head<2>().norm();
      if (roll_pitch_norm > max_roll_pitch_step) {
        delta.head<2>() *= max_roll_pitch_step / roll_pitch_norm;
      }
    }

    if (max_tz_step > 0.0) {
      delta[5] = std::min(max_tz_step, std::max(-max_tz_step, delta[5]));
    }
  }
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__STEP_LIMIT_OPTIMIZER_HPP_
