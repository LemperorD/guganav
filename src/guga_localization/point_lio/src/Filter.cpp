#include "point_lio/Filter.h"

#include "point_lio/parameters.h"

void Filter::initialize(Estimator& estimator) {
  if (initialized_) {
    return;
  }

  kf_input.init_dyn_share_modified_2h(
      [&estimator](state_input& state, const input_ikfom& input) {
        return estimator.getFInput(state, input);
      },
      [&estimator](state_input& state, const input_ikfom& input) {
        return estimator.dfDxInput(state, input);
      },
      [&estimator](state_input& state, Eigen::Matrix3d cov_p,
                   Eigen::Matrix3d cov_R,
                   esekfom::dyn_share_modified<double>& data) {
        estimator.hModelInput(state, cov_p, cov_R, data);
      });

  kf_output.init_dyn_share_modified_3h(
      [&estimator](state_output& state, const input_ikfom& input) {
        return estimator.getFOutput(state, input);
      },
      [&estimator](state_output& state, const input_ikfom& input) {
        return estimator.dfDxOutput(state, input);
      },
      [&estimator](state_output& state, Eigen::Matrix3d cov_p,
                   Eigen::Matrix3d cov_R,
                   esekfom::dyn_share_modified<double>& data) {
        estimator.hModelOutput(state, cov_p, cov_R, data);
      },
      [&estimator](state_output& state,
                   esekfom::dyn_share_modified<double>& data) {
        estimator.hModelImuOutput(state, data);
      });

  Eigen::Matrix<double, 24, 24> input_covariance;
  reset_cov(input_covariance);
  kf_input.change_P(input_covariance);
  Eigen::Matrix<double, 30, 30> output_covariance;
  reset_cov_output(output_covariance);
  kf_output.change_P(output_covariance);
  input_noise_ = estimator.processNoiseCovInput();
  output_noise_ = estimator.processNoiseCovOutput();
  initialized_ = true;
}
