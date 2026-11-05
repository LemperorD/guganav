#pragma once

#include "point_lio/core/common_lib.h"
#include "point_lio/core/parameters.h"

class ImuMeasurementModel;
class LidarMeasurementModel;

class EskfProcessModel {
public:
  void configure(const FilterParams& params);
  [[nodiscard]] Eigen::Matrix<double, 24, 24> inputNoise() const;
  [[nodiscard]] Eigen::Matrix<double, 30, 30> outputNoise() const;
  [[nodiscard]] Eigen::Matrix<double, 24, 1> getFInput(
      state_input& state, const input_ikfom& input) const;
  [[nodiscard]] Eigen::Matrix<double, 30, 1> getFOutput(
      state_output& state, const input_ikfom& input) const;
  [[nodiscard]] Eigen::Matrix<double, 24, 24> dfDxInput(
      state_input& state, const input_ikfom& input) const;
  [[nodiscard]] Eigen::Matrix<double, 30, 30> dfDxOutput(
      state_output& state, const input_ikfom& input) const;

private:
  FilterParams params_;
};

class Filter {
public:
  using InputFilter = esekfom::esekf<state_input, 24, input_ikfom>;
  using OutputFilter = esekfom::esekf<state_output, 30, input_ikfom>;
  using InputNoise = Eigen::Matrix<double, 24, 24>;
  using OutputNoise = Eigen::Matrix<double, 30, 30>;

  void configure(const FilterParams& params);
  void initialize(LidarMeasurementModel& lidar_model,
                  ImuMeasurementModel& imu_model);

  [[nodiscard]] InputFilter& input() {
    return input_;
  }
  [[nodiscard]] OutputFilter& output() {
    return output_;
  }
  [[nodiscard]] const InputFilter& input() const {
    return input_;
  }
  [[nodiscard]] const OutputFilter& output() const {
    return output_;
  }
  [[nodiscard]] auto& inputNoise() {
    return input_noise_;
  }
  [[nodiscard]] auto& outputNoise() {
    return output_noise_;
  }
  void updateOutputImu() {
    output_.update_iterated_dyn_share_IMU();
  }
  void predict(InputFilter& filter, double dt, InputNoise& noise,
               const input_ikfom& input, bool update_covariance,
               bool propagate) {
    filter.predict(dt, noise, input, update_covariance, propagate);
  }
  void predict(OutputFilter& filter, double dt, OutputNoise& noise,
               const input_ikfom& input, bool update_covariance,
               bool propagate) {
    filter.predict(dt, noise, input, update_covariance, propagate);
  }
  bool updateLidar(InputFilter& filter) {
    return filter.update_iterated_dyn_share_modified();
  }
  bool updateLidar(OutputFilter& filter) {
    return filter.update_iterated_dyn_share_modified();
  }

private:
  InputFilter input_;
  OutputFilter output_;
  EskfProcessModel process_model_;
  InputNoise input_noise_;
  OutputNoise output_noise_;
};
