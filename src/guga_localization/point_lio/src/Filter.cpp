#include "point_lio/Filter.h"

#include "point_lio/Imu.h"
#include "point_lio/Lidar.h"

void EskfProcessModel::configure(const FilterParams& params) {
  params_ = params;
}

Eigen::Matrix<double, 24, 24> EskfProcessModel::inputNoise() const {
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  cov.block<3, 3>(3, 3).diagonal().setConstant(params_.gyr_cov_input);
  cov.block<3, 3>(12, 12).diagonal().setConstant(params_.acc_cov_input);
  cov.block<3, 3>(15, 15).diagonal().setConstant(params_.b_gyr_cov);
  cov.block<3, 3>(18, 18).diagonal().setConstant(params_.b_acc_cov);
  return cov;
}

Eigen::Matrix<double, 30, 30> EskfProcessModel::outputNoise() const {
  Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
  cov.block<3, 3>(12, 12).diagonal().setConstant(params_.vel_cov);
  cov.block<3, 3>(15, 15).diagonal().setConstant(params_.gyr_cov_output);
  cov.block<3, 3>(18, 18).diagonal().setConstant(params_.acc_cov_output);
  cov.block<3, 3>(24, 24).diagonal().setConstant(params_.b_gyr_cov);
  cov.block<3, 3>(27, 27).diagonal().setConstant(params_.b_acc_cov);
  return cov;
}

Eigen::Matrix<double, 24, 1> EskfProcessModel::getFInput(
    state_input& state, const input_ikfom& input) const {
  Eigen::Matrix<double, 24, 1> result =
      Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  input.gyro.boxminus(omega, state.bg);
  const vect3 acceleration = state.rot * (input.acc - state.ba);
  for (int i = 0; i < 3; ++i) {
    result(i) = state.vel[i];
    result(i + 3) = omega[i];
    result(i + 12) = acceleration[i] + state.gravity[i];
  }
  return result;
}

Eigen::Matrix<double, 30, 1> EskfProcessModel::getFOutput(
    state_output& state, const input_ikfom&) const {
  Eigen::Matrix<double, 30, 1> result =
      Eigen::Matrix<double, 30, 1>::Zero();
  const vect3 acceleration = state.rot * state.acc;
  for (int i = 0; i < 3; ++i) {
    result(i) = state.vel[i];
    result(i + 3) = state.omg[i];
    result(i + 12) = acceleration[i] + state.gravity[i];
  }
  return result;
}

Eigen::Matrix<double, 24, 24> EskfProcessModel::dfDxInput(
    state_input& state, const input_ikfom& input) const {
  Eigen::Matrix<double, 24, 24> result =
      Eigen::Matrix<double, 24, 24>::Zero();
  vect3 acceleration;
  input.acc.boxminus(acceleration, state.ba);
  result.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  result.block<3, 3>(12, 3) = -state.rot * MTK::hat(acceleration);
  result.block<3, 3>(12, 18) = -state.rot;
  result.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
  result.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return result;
}

Eigen::Matrix<double, 30, 30> EskfProcessModel::dfDxOutput(
    state_output& state, const input_ikfom&) const {
  Eigen::Matrix<double, 30, 30> result =
      Eigen::Matrix<double, 30, 30>::Zero();
  result.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  result.block<3, 3>(12, 3) = -state.rot * MTK::hat(state.acc);
  result.block<3, 3>(12, 18) = state.rot;
  result.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
  result.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
  return result;
}

void Filter::configure(const FilterParams& params) {
  process_model_.configure(params);
  input_noise_ = process_model_.inputNoise();
  output_noise_ = process_model_.outputNoise();
}

void Filter::initialize(LidarMeasurementModel& lidar_model,
                        ImuMeasurementModel& imu_model) {
  input_ = InputFilter{};
  output_ = OutputFilter{};

  input_.init_dyn_share_modified_2h(
      [this](state_input& state, const input_ikfom& input) {
        return process_model_.getFInput(state, input);
      },
      [this](state_input& state, const input_ikfom& input) {
        return process_model_.dfDxInput(state, input);
      },
      [model = &lidar_model](
          state_input& state, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
          esekfom::dyn_share_modified<double>& data) {
        model->hModelInput(state, cov_p, cov_R, data);
      });

  output_.init_dyn_share_modified_3h(
      [this](state_output& state, const input_ikfom& input) {
        return process_model_.getFOutput(state, input);
      },
      [this](state_output& state, const input_ikfom& input) {
        return process_model_.dfDxOutput(state, input);
      },
      [model = &lidar_model](
          state_output& state, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
          esekfom::dyn_share_modified<double>& data) {
        model->hModelOutput(state, cov_p, cov_R, data);
      },
      [model = &imu_model](
          state_output& state,
          esekfom::dyn_share_modified<double>& data) {
        model->hModelOutput(state, data);
      });

  InputNoise input_covariance = InputNoise::Identity() * 0.1;
  input_covariance.block<3, 3>(21, 21) =
      Eigen::Matrix3d::Identity() * 0.0001;
  input_covariance.block<6, 6>(15, 15) =
      Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
  input_.change_P(input_covariance);

  OutputNoise output_covariance = OutputNoise::Identity() * 0.01;
  output_covariance.block<3, 3>(21, 21) =
      Eigen::Matrix3d::Identity() * 0.0001;
  output_covariance.block<6, 6>(24, 24) =
      Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
  output_.change_P(output_covariance);
}
