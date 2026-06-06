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

#pragma once

namespace pb_omni_pid_pursuit_controller {

  class PID {
  public:
    PID(double dt, double max, double min, double kp, double kd, double ki,
        double min_max_sum_error = 1.0);

    // Returns the manipulated variable given a set_point and current process
    // value
    [[nodiscard]] double calculate(double set_point, double pv);
    void setSumError(double sum_error);
    ~PID() = default;

    PID(const PID&) = delete;
    PID& operator=(const PID&) = delete;
    PID(PID&&) = delete;
    PID& operator=(PID&&) = delete;

  private:
    double dt_;
    double max_;
    double min_;
    double kp_;
    double kd_;
    double ki_;
    double pre_error_;
    double integral_;
    double min_max_sum_error_;
  };

}  // namespace pb_omni_pid_pursuit_controller
