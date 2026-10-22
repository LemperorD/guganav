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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__ACCEL_LIMITS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__ACCEL_LIMITS_HPP_

#include <algorithm>
#include <limits>

/**
 * @brief 加速度限幅的公共实现
 *
 * 单独成头文件的原因：`motion_models.hpp`（预测里逐样本夹紧状态速度）和
 * `optimizer.cpp`（夹紧控制序列、odom 滞后补偿）都必须用**同一套**语义，
 * 而 `tools/utils.hpp` 依赖 CriticData，不能从 motion_models.hpp 里包含。
 * 语义在这里只实现一次，两处共用，避免出现"预测与执行方向不对称"的偏差。
 */
namespace mppi::utils
{

/**
 * @brief 把输入夹到 [lower_bound, upper_bound]
 */
inline float clamp(const float lower_bound, const float upper_bound, const float input)
{
  return std::min(upper_bound, std::max(input, lower_bound));
}

/**
 * @brief 按加速度上限夹紧速度（移植自上游 nav2 PR #6072）
 *
 * 前进（last_vel >= 0）时用 max_delta 限制加速、min_delta 限制减速；
 * 倒车（last_vel < 0）时把两个方向对调，这样"倒车时继续后退"用的是
 * ax_max、"倒车时刹停"用的是 ax_min，符合非对称加减速的物理含义。
 * 因此**前进与倒车行为镜像对称**：|v| 增大受加速度上限约束、减小不受限。
 *
 * @param last_vel: 上一步速度
 * @param curr_vel: 本步期望速度
 * @param min_delta: 允许的最大速度变化量（减速侧，通常为负；未设置时传 -inf）
 * @param max_delta: 允许的最大速度变化量（加速侧，通常为正；未设置时传 +inf）
 * @return 返回值: 夹紧后的速度
 */
inline float clampVelocityByAccel(
  const float last_vel, const float curr_vel,
  const float min_delta, const float max_delta)
{
  const float v = (last_vel >= 0)
    ? clamp(last_vel + min_delta, last_vel + max_delta, curr_vel)
    : clamp(last_vel - max_delta, last_vel - min_delta, curr_vel);

  // 换向（结果穿过 0）：刹车侧不设限时上面的公式会让速度一步跳到另一侧，但"在另一侧
  // 加速"仍然受加速度上限约束——否则模型允许瞬间反向、执行端做不到，表现为反向一卡
  // 一卡。两侧都给了限值时不触发（此时换向本就被刹车侧限住）。
  if ((last_vel >= 0.0f && v < 0.0f) || (last_vel < 0.0f && v > 0.0f)) {
    return clamp(-max_delta, max_delta, curr_vel);
  }
  return v;
}

/**
 * @brief 加速侧上限 → 单步允许的速度增量
 *
 * 上限留 0 表示这一侧"不做限制"，返回 +inf。不能返回 0：那会被
 * clampVelocityByAccel 读成"这一侧不允许变化"（只填 ax_max 就永远不能减速）。
 *
 * @param limit: 加速侧上限（ax_max/ay_max，正常为正；az_max 用于对称的 wz）
 * @param dt: 步长（控制周期或预测步长）
 * @return 返回值: dt 内允许的正向速度增量
 */
inline float maxAccelDelta(const float limit, const float dt)
{
  return limit > 0.0f ? limit * dt : std::numeric_limits<float>::infinity();
}

/**
 * @brief 减速侧下限 → 单步允许的速度增量（负值）
 * @param limit: 减速侧下限（ax_min/ay_min，正常为负）
 * @param dt: 步长
 * @return 返回值: dt 内允许的负向速度增量；未设置时返回 -inf
 */
inline float minAccelDelta(const float limit, const float dt)
{
  return limit < 0.0f ? limit * dt : -std::numeric_limits<float>::infinity();
}

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__ACCEL_LIMITS_HPP_
