#pragma once

#include "point_lio/Estimator.h"

class Filter {
public:
  using InputFilter = esekfom::esekf<state_input, 24, input_ikfom>;
  using OutputFilter = esekfom::esekf<state_output, 30, input_ikfom>;

  void initialize(Estimator& estimator);

  [[nodiscard]] InputFilter& input() { return input_; }
  [[nodiscard]] OutputFilter& output() { return output_; }
  [[nodiscard]] const InputFilter& input() const { return input_; }
  [[nodiscard]] const OutputFilter& output() const { return output_; }
  [[nodiscard]] auto& inputNoise() { return input_noise_; }
  [[nodiscard]] auto& outputNoise() { return output_noise_; }

private:
  InputFilter input_;
  OutputFilter output_;
  Eigen::Matrix<double, 24, 24> input_noise_;
  Eigen::Matrix<double, 30, 30> output_noise_;
  bool initialized_{false};
};
