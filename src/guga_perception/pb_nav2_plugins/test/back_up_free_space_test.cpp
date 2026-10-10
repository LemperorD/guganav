// Copyright 2026 HaoZhen Liu
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

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"

namespace pb_nav2_behaviors
{

class TestBackUpFreeSpace : public BackUpFreeSpace
{
public:
  using BackUpFreeSpace::EscapeDirection;
  using BackUpFreeSpace::findBestDirection;

  void setEscapeParameters(double max_escape_distance, double escape_clearance)
  {
    max_escape_distance_ = max_escape_distance;
    escape_clearance_ = escape_clearance;
  }

  void setTraverseCostThreshold(int threshold) { traverse_cost_threshold_ = threshold; }

  void setRelaxedEscapeEnabled(bool enabled) { relaxed_escape_enabled_ = enabled; }
};

nav2_msgs::msg::Costmap makeCostmap()
{
  nav2_msgs::msg::Costmap costmap;
  costmap.metadata.resolution = 0.05;
  costmap.metadata.size_x = 41;
  costmap.metadata.size_y = 41;
  costmap.metadata.origin.position.x = -1.025;
  costmap.metadata.origin.position.y = -1.025;
  costmap.data.assign(costmap.metadata.size_x * costmap.metadata.size_y, 0);
  return costmap;
}

void setCost(nav2_msgs::msg::Costmap & costmap, double x, double y, uint8_t cost)
{
  const int i = static_cast<int>(
    std::floor((x - costmap.metadata.origin.position.x) / costmap.metadata.resolution));
  const int j = static_cast<int>(
    std::floor((y - costmap.metadata.origin.position.y) / costmap.metadata.resolution));
  ASSERT_GE(i, 0);
  ASSERT_GE(j, 0);
  ASSERT_LT(i, static_cast<int>(costmap.metadata.size_x));
  ASSERT_LT(j, static_cast<int>(costmap.metadata.size_y));
  costmap.data[static_cast<size_t>(i) + static_cast<size_t>(j) * costmap.metadata.size_x] = cost;
}

void fillDisk(nav2_msgs::msg::Costmap & costmap, double radius, uint8_t cost)
{
  for (unsigned int j = 0; j < costmap.metadata.size_y; ++j) {
    for (unsigned int i = 0; i < costmap.metadata.size_x; ++i) {
      const double x = costmap.metadata.origin.position.x +
                       (static_cast<double>(i) + 0.5) * costmap.metadata.resolution;
      const double y = costmap.metadata.origin.position.y +
                       (static_cast<double>(j) + 0.5) * costmap.metadata.resolution;
      if (std::hypot(x, y) <= radius) {
        costmap.data[static_cast<size_t>(i) + static_cast<size_t>(j) * costmap.metadata.size_x] =
          cost;
      }
    }
  }
}

TEST(BackUpFreeSpaceTest, EscapesInitialLethalCell)
{
  auto costmap = makeCostmap();
  setCost(costmap, 0.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  for (double x = 0.05; x <= 0.30; x += 0.05) {
    setCost(costmap, x, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  TestBackUpFreeSpace behavior;
  behavior.setEscapeParameters(0.5, 0.1);
  geometry_msgs::msg::Pose2D pose;
  const auto result = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.8, M_PI / 32.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->started_in_high_cost);
  EXPECT_LE(result->escape_distance, 0.10);
  EXPECT_LE(result->command_distance, 0.20);
  EXPECT_GT(std::fabs(result->angle), M_PI / 4.0);
}

TEST(BackUpFreeSpaceTest, EscapesInitialInscribedCell)
{
  auto costmap = makeCostmap();
  setCost(costmap, 0.0, 0.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  TestBackUpFreeSpace behavior;
  behavior.setEscapeParameters(0.5, 0.1);
  geometry_msgs::msg::Pose2D pose;
  const auto result = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.5, M_PI / 32.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->started_in_high_cost);
  EXPECT_LE(result->escape_distance, 0.10);
}

TEST(BackUpFreeSpaceTest, FreeStartKeepsRequestedDistance)
{
  const auto costmap = makeCostmap();
  TestBackUpFreeSpace behavior;
  behavior.setEscapeParameters(0.5, 0.1);
  geometry_msgs::msg::Pose2D pose;
  const auto result = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.4, M_PI / 32.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->started_in_high_cost);
  EXPECT_FLOAT_EQ(result->escape_distance, 0.0);
  EXPECT_FLOAT_EQ(result->command_distance, 0.4);
}

TEST(BackUpFreeSpaceTest, RejectsHighCostRegionWiderThanEscapeLimit)
{
  auto costmap = makeCostmap();
  fillDisk(costmap, 0.60, nav2_costmap_2d::LETHAL_OBSTACLE);

  TestBackUpFreeSpace behavior;
  behavior.setEscapeParameters(0.5, 0.1);
  geometry_msgs::msg::Pose2D pose;
  const auto result = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.8, M_PI / 32.0);

  EXPECT_FALSE(result.has_value());
}

TEST(BackUpFreeSpaceTest, RejectsObstacleAfterLeavingInitialRegion)
{
  auto costmap = makeCostmap();
  setCost(costmap, 0.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  for (double angle = -M_PI; angle < M_PI; angle += M_PI / 64.0) {
    setCost(
      costmap, 0.15 * std::cos(angle), 0.15 * std::sin(angle),
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }

  TestBackUpFreeSpace behavior;
  behavior.setEscapeParameters(0.5, 0.1);
  geometry_msgs::msg::Pose2D pose;
  const auto result = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.4, M_PI / 32.0);

  EXPECT_FALSE(result.has_value());
}

/**
 * 允许穿越 253 的膨胀环：起点在环内，环外是自由空间。
 * 旧实现把 253 当障碍，任何穿过环的射线都作废，于是返回无解；
 * 现在只挡 >= traverse_cost_threshold（默认 254），因此能选出一个穿环方向。
 */
TEST(BackUpFreeSpaceTest, CrossesInscribedRingToReachFreeSpace)
{
  auto costmap = makeCostmap();
  fillDisk(costmap, 0.20, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  for (double x = -0.10; x <= 0.10; x += 0.05) {
    for (double y = -0.10; y <= 0.10; y += 0.05) {
      if (std::hypot(x, y) < 0.10) {
        setCost(costmap, x, y, nav2_costmap_2d::FREE_SPACE);
      }
    }
  }

  TestBackUpFreeSpace behavior;
  behavior.setEscapeParameters(0.5, 0.1);
  geometry_msgs::msg::Pose2D pose;
  const auto result = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.5, M_PI / 32.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->started_in_high_cost);
  EXPECT_FLOAT_EQ(result->command_distance, 0.5f);

  // 把阈值退回 253（旧语义）后同一个方向应当被否掉
  behavior.setTraverseCostThreshold(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const auto strict = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.5, M_PI / 32.0);
  EXPECT_FALSE(strict.has_value());
}

/**
 * 高代价区半径大于 max_escape_distance：严格条件下无解，放宽到请求距离后可以脱出。
 */
TEST(BackUpFreeSpaceTest, RelaxedEscapeDistanceFindsDirection)
{
  auto costmap = makeCostmap();
  fillDisk(costmap, 0.70, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  TestBackUpFreeSpace behavior;
  behavior.setEscapeParameters(0.5, 0.1);
  geometry_msgs::msg::Pose2D pose;
  const auto relaxed = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.9, M_PI / 32.0);

  ASSERT_TRUE(relaxed.has_value());
  EXPECT_TRUE(relaxed->started_in_high_cost);
  EXPECT_GT(relaxed->escape_distance, 0.5);
  EXPECT_NEAR(relaxed->escape_distance, 0.75, 0.02);

  // 关掉放宽后应当回到"无解"
  behavior.setRelaxedEscapeEnabled(false);
  const auto strict = behavior.findBestDirection(costmap, pose, -M_PI, M_PI, 0.9, M_PI / 32.0);
  EXPECT_FALSE(strict.has_value());
}

}  // namespace pb_nav2_behaviors
