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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_

namespace mppi::models
{

/**
 * @struct mppi::models::ControlConstraints
 * @brief 控制量的速度、角速度与加速度约束
 *
 * 加速度字段移植自上游 nav2（PR #4352 引入、#6072 补齐 ay_min 与 controller_period
 * 的用法）：
 *   ax_max/ax_min —— 纵向加/减速上限（m/s²，ax_min 必须为负）
 *   ay_max/ay_min —— 横向加/减速上限（m/s²，仅全向底盘使用）
 *   az_max        —— 偏航角加速度上限（rad/s²，对称）
 * 全部为 0 表示"不启用加速度约束"，与移植前的行为完全一致。
 */
struct ControlConstraints
{
  // 纵向速度上限。
  float vx_max;
  // 纵向速度下限，负值表示允许倒车。
  float vx_min;
  // 横向速度绝对值上限。
  float vy;
  // 偏航角速度绝对值上限。
  float wz;
  // 纵向加速度上限（m/s²）。
  float ax_max;
  // 纵向减速度下限（m/s²，负值）。
  float ax_min;
  // 横向加速度上限（m/s²）。
  float ay_max;
  // 横向减速度下限（m/s²，负值）。
  float ay_min;
  // 偏航角加速度上限（rad/s²）。
  float az_max;
};

/**
 * @struct mppi::models::SamplingStd
 * @brief 轨迹采样噪声的标准差参数
 */
struct SamplingStd
{
  // 纵向速度采样噪声的标准差。
  float vx;
  // 横向速度采样噪声的标准差。
  float vy;
  // 偏航角速度采样噪声的标准差。
  float wz;
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
