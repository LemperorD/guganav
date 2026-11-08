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

#include "pb_nav2_plugins/behaviors/escape_obstacle_layer.hpp"

#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"

namespace pb_nav2_behaviors
{

namespace
{

nav2_msgs::msg::Costmap makeCostmap()
{
  nav2_msgs::msg::Costmap costmap;
  costmap.metadata.resolution = 0.05F;
  costmap.metadata.size_x = 81;
  costmap.metadata.size_y = 81;
  costmap.metadata.origin.position.x = -2.025;
  costmap.metadata.origin.position.y = -2.025;
  costmap.data.assign(static_cast<size_t>(costmap.metadata.size_x) * costmap.metadata.size_y, 0);
  return costmap;
}

void setCost(nav2_msgs::msg::Costmap & costmap, double x, double y, uint8_t cost)
{
  const int i = static_cast<int>(
    std::floor((x - costmap.metadata.origin.position.x) / costmap.metadata.resolution));
  const int j = static_cast<int>(
    std::floor((y - costmap.metadata.origin.position.y) / costmap.metadata.resolution));
  if (
    i < 0 || i >= static_cast<int>(costmap.metadata.size_x) || j < 0 ||
    j >= static_cast<int>(costmap.metadata.size_y)) {
    return;
  }
  costmap.data[static_cast<size_t>(i) + static_cast<size_t>(j) * costmap.metadata.size_x] = cost;
}

/// 在 (cx, cy) 处画一个半径为 radius 的实心圆
void fillDisk(nav2_msgs::msg::Costmap & costmap, double cx, double cy, double radius, uint8_t cost)
{
  const double resolution = costmap.metadata.resolution;
  for (double x = -2.0; x <= 2.0; x += resolution) {
    for (double y = -2.0; y <= 2.0; y += resolution) {
      const double dx = x - cx;
      const double dy = y - cy;
      if (std::hypot(dx, dy) <= radius) {
        setCost(costmap, x, y, cost);
      }
    }
  }
}

geometry_msgs::msg::Pose2D makePose(double x, double y)
{
  geometry_msgs::msg::Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = 0.0;
  return pose;
}

EscapeObstacleLayer::EscapeConfig makeConfig()
{
  EscapeObstacleLayer::EscapeConfig config;
  config.max_distance = 1.2;
  config.exit_clearance = 0.1;
  config.lethal_traverse_distance = 0.5;
  config.footprint_radius = 0.4;
  config.angle_step = M_PI / 32.0;
  return config;
}

}  // namespace

TEST(EscapeObstacleLayerTest, EscapesInscribedDiskWithMinimalDistance)
{
  auto costmap = makeCostmap();
  fillDisk(costmap, 0.0, 0.0, 0.30, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  const auto direction =
    EscapeObstacleLayer::findEscapeDirection(costmap, makePose(0.0, 0.0), makeConfig());

  ASSERT_TRUE(direction.has_value());
  // 圆盘半径 0.30 + 栅格量化，出口应当在 0.30 附近，且命令距离只多一个 clearance
  EXPECT_NEAR(direction->exit_distance, 0.30, 0.06);
  EXPECT_NEAR(direction->command_distance, direction->exit_distance + 0.1, 1e-6);
  EXPECT_LE(direction->command_distance, 1.2);
}

TEST(EscapeObstacleLayerTest, AvoidsLethalObstacleOnOneSide)
{
  auto costmap = makeCostmap();
  fillDisk(costmap, 0.0, 0.0, 0.30, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  // +x 半边是真障碍：朝 +x 逃会撞墙，方向必须绕开
  for (double x = 0.35; x <= 2.0; x += 0.05) {
    for (double y = -2.0; y <= 2.0; y += 0.05) {
      setCost(costmap, x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  const auto direction =
    EscapeObstacleLayer::findEscapeDirection(costmap, makePose(0.0, 0.0), makeConfig());

  ASSERT_TRUE(direction.has_value());
  EXPECT_LT(std::cos(direction->angle), 0.0) << "escape direction " << direction->angle;
}

TEST(EscapeObstacleLayerTest, NoDirectionWhenHighCostRegionIsTooWide)
{
  auto costmap = makeCostmap();
  // 整张图都是膨胀硬区：任何方向都走不出去
  for (auto & cell : costmap.data) {
    cell = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  const auto direction =
    EscapeObstacleLayer::findEscapeDirection(costmap, makePose(0.0, 0.0), makeConfig());

  EXPECT_FALSE(direction.has_value());
}

TEST(EscapeObstacleLayerTest, ReturnsNulloptWhenAlreadyFree)
{
  auto costmap = makeCostmap();

  const auto direction =
    EscapeObstacleLayer::findEscapeDirection(costmap, makePose(0.0, 0.0), makeConfig());

  EXPECT_FALSE(direction.has_value());
}

TEST(EscapeObstacleLayerTest, EscapesLethalStartWithinTraverseLimit)
{
  auto costmap = makeCostmap();
  fillDisk(costmap, 0.0, 0.0, 0.25, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto direction =
    EscapeObstacleLayer::findEscapeDirection(costmap, makePose(0.0, 0.0), makeConfig());

  ASSERT_TRUE(direction.has_value());
  EXPECT_LE(direction->exit_distance, 0.5);
}

}  // namespace pb_nav2_behaviors
