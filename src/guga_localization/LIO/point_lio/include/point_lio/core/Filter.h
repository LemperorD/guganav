#pragma once

#include "point_lio/core/common_lib.h"
#include "point_lio/core/parameters.h"

class ImuMeasurementModel;
class LidarMeasurementModel;

class EskfProcessModel {
public:
  void configure(const FilterParams& params);
  [[nodiscard]] Eigen::Matrix<double, 30, 30> outputNoise() const;
  [[nodiscard]] Eigen::Matrix<double, 30, 1> getFOutput(
      state_output& state, const input_ikfom& input) const;
  [[nodiscard]] Eigen::Matrix<double, 30, 30> dfDxOutput(
      state_output& state, const input_ikfom& input) const;

private:
  FilterParams params_;
};

class Filter {
public:
  using OutputFilter = esekfom::esekf<state_output, 30, input_ikfom>;
  using OutputNoise = Eigen::Matrix<double, 30, 30>;

  void configure(const FilterParams& params);
  void initialize(LidarMeasurementModel& lidar_model,
                  ImuMeasurementModel& imu_model);

  [[nodiscard]] OutputFilter& output() {
    return output_;
  }
  [[nodiscard]] const OutputFilter& output() const {
    return output_;
  }
  [[nodiscard]] auto& outputNoise() {
    return output_noise_;
  }
  void updateOutputImu() {
    output_.update_iterated_dyn_share_IMU();
  }
  void predict(OutputFilter& filter, double dt, OutputNoise& noise,
               const input_ikfom& input, bool update_covariance,
               bool propagate) {
    filter.predict(dt, noise, input, update_covariance, propagate);
  }
  bool updateLidar(OutputFilter& filter) {
    return filter.update_iterated_dyn_share_modified();
  }

private:
  OutputFilter output_;
  EskfProcessModel process_model_;
  OutputNoise output_noise_;
};
