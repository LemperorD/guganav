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
 * @brief 控制量的速度与角速度约束
 */
struct ControlConstraints
{
  float vx_max;
  float vx_min;
  float vy;
  float wz;
};

/**
 * @struct mppi::models::SamplingStd
 * @brief 轨迹采样噪声的标准差参数
 */
struct SamplingStd
{
  float vx;
  float vy;
  float wz;
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
