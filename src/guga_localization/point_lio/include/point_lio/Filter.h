#pragma once

#include "point_lio/Estimator.h"

class Filter {
public:
  void initialize(Estimator& estimator);

  [[nodiscard]] auto& input() { return kf_input; }
  [[nodiscard]] auto& output() { return kf_output; }
  [[nodiscard]] const auto& input() const { return kf_input; }
  [[nodiscard]] const auto& output() const { return kf_output; }
  [[nodiscard]] auto& inputNoise() { return input_noise_; }
  [[nodiscard]] auto& outputNoise() { return output_noise_; }

private:
  Eigen::Matrix<double, 24, 24> input_noise_;
  Eigen::Matrix<double, 30, 30> output_noise_;
  bool initialized_{false};
};
