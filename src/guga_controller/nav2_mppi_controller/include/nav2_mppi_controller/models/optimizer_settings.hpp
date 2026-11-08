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
 * @struct mppi::models::AccelDeltas
 * @brief 加速度约束换算成每个时间步允许的速度增量（参数或控制周期变化时算好）
 *
 * 两套值：`*_first` 用真实控制周期（t=0 的可行性），`*_model` 用预测步长
 * model_dt（t>=1）。某侧留 0 时对应值为 ±inf，表示该侧不限制。
 */
struct AccelDeltas
{
  // 第 0 步（真实控制周期）允许的纵向速度增量。
  float vx_max_first{0.0f};
  float vx_min_first{0.0f};
  // 第 0 步允许的横向速度增量（仅全向底盘使用）。
  float vy_max_first{0.0f};
  float vy_min_first{0.0f};
  // 第 0 步允许的偏航角速度增量。
  float wz_max_first{0.0f};
  // 第 1 步起（预测步长 model_dt）允许的纵向速度增量。
  float vx_max_model{0.0f};
  float vx_min_model{0.0f};
  // 第 1 步起允许的横向速度增量。
  float vy_max_model{0.0f};
  float vy_min_model{0.0f};
  // 第 1 步起允许的偏航角速度增量。
  float wz_max_model{0.0f};
};

/**
 * @struct mppi::models::OptimizerSettings
 * @brief 优化器使用的约束、采样规模和迭代设置
 */
struct OptimizerSettings
{
  // 参数配置的原始速度/加速度约束，不受运行时限速影响。
  models::ControlConstraints base_constraints{0, 0, 0, 0, 0, 0, 0, 0, 0};
  // 当前生效的约束，可能被运行时限速修改。
  models::ControlConstraints constraints{0, 0, 0, 0, 0, 0, 0, 0, 0};
  // 由 constraints 与控制周期预先换算好的每步速度增量上限。
  models::AccelDeltas accel_deltas{};
  // 三个控制维度的采样噪声标准差。
  models::SamplingStd sampling_std{0, 0, 0};
  // 相邻预测状态之间的积分时间间隔。
  float model_dt{0};
  // 控制周期 1/controller_frequency；t=0 的加速度可行性用它而不是 model_dt。
  float controller_period{0};
  // Savitzky-Golay 滤波阶数：1 = 9 点滑动均值（更平滑），2 = 二次九点（默认）。
  unsigned int sgf_order{2u};
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
