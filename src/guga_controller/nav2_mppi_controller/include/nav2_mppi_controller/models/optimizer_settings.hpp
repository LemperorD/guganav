// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_

#include <cstddef>
#include "nav2_mppi_controller/models/constraints.hpp"

namespace mppi::models
{

/**
 * @struct mppi::models::OptimizerSettings
 * @brief 优化器使用的约束、采样规模和迭代设置
 */
struct OptimizerSettings
{
  // 参数配置的原始速度约束，不受运行时限速影响。
  models::ControlConstraints base_constraints{0, 0, 0, 0};
  // 当前生效的速度约束，可能被运行时限速修改。
  models::ControlConstraints constraints{0, 0, 0, 0};
  // 三个控制维度的采样噪声标准差。
  models::SamplingStd sampling_std{0, 0, 0};
  // 相邻预测状态之间的积分时间间隔。
  float model_dt{0};
  // 轨迹成本转换为权重时使用的温度参数。
  float temperature{0};
  // 控制扰动成本的正则化系数。
  float gamma{0};
  // 每轮优化生成的候选轨迹数量。
  unsigned int batch_size{0};
  // 每条候选轨迹包含的预测时间步数。
  unsigned int time_steps{0};
  // 每个控制周期执行的优化迭代次数。
  unsigned int iteration_count{0};
  // 是否在控制周期结束后将最优控制序列前移一位。
  bool shift_control_sequence{false};
  // 全部候选轨迹失败时允许重新采样的次数。
  size_t retry_attempt_limit{0};
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
