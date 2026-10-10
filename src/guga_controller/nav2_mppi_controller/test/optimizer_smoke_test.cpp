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

#include "gtest/gtest.h"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/motion_models.hpp"

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

// Smoke tests the optimizer

class OptimizerSuite : public ::testing::TestWithParam<std::tuple<std::string,
    std::vector<std::string>, bool>> {};

TEST_P(OptimizerSuite, OptimizerTest) {
  auto [motion_model, critics, consider_footprint] = GetParam();

  int batch_size = 400;
  int time_steps = 15;
  unsigned int path_points = 50u;
  int iteration_count = 1;
  double lookahead_distance = 10.0;

  TestCostmapSettings costmap_settings{};
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  auto costmap = costmap_ros->getCostmap();

  TestPose start_pose = costmap_settings.getCenterPose();
  double path_step = costmap_settings.resolution;

  TestPathSettings path_settings{start_pose, path_points, path_step, path_step};
  TestOptimizerSettings optimizer_settings{batch_size, time_steps, iteration_count,
    lookahead_distance, motion_model, consider_footprint};

  unsigned int offset = 4;
  unsigned int obstacle_size = offset * 2;

  unsigned char obstacle_cost = 250;

  auto [obst_x, obst_y] = costmap_settings.getCenterIJ();

  obst_x = obst_x - offset;
  obst_y = obst_y - offset;
  addObstacle(costmap, {obst_x, obst_y, obstacle_size, obstacle_cost});

  printInfo(optimizer_settings, path_settings, critics);
  auto node = getDummyNode(optimizer_settings, critics);
  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
  auto optimizer = getDummyOptimizer(node, costmap_ros, parameters_handler.get());

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);
  nav2_core::GoalChecker * dummy_goal_checker{nullptr};

  EXPECT_NO_THROW(optimizer->evalControl(pose, velocity, path, dummy_goal_checker));
}

INSTANTIATE_TEST_SUITE_P(
  OptimizerTests,
  OptimizerSuite,
  ::testing::Values(
    std::make_tuple(
      "Omni",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"}, {"PathAlignCritic"},
          {"TwirlingCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true),
    std::make_tuple(
      "DiffDrive",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"CostCritic"},
          {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true),
    std::make_tuple(
      "Ackermann",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
          {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true))
);


// 性能归因基准：整周期 evalControl（含全部 critics）在"加速度约束开/关"下的耗时。
// 规模与 critics 对齐 simulation profile（batch 1800 × time_steps 56、30 Hz、
// Constraint/Cost/Goal/PathAlign/PathFollow），代价地图里铺障碍 + 253 环带，
// 让大量轨迹点落在代价区，接近真实仿真的负载分布。
TEST(OptimizerBenchmark, FullCycleAccelCost)
{
  const int batch_size = 1800;
  const int time_steps = 56;
  const unsigned int path_points = 100u;
  const double lookahead = 10.0;
  const std::vector<std::string> critics = {
    "ConstraintCritic", "CostCritic", "GoalCritic", "PathAlignCritic", "PathFollowCritic"};

  TestCostmapSettings costmap_settings{};
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  auto costmap = costmap_ros->getCostmap();

  // 每 10 格放一个 2x2 障碍，并在周围铺 253 环带（等效 inflation 后的内切带）
  for (unsigned int i = 6; i + 5 < costmap_settings.cells_x; i += 10) {
    for (unsigned int j = 6; j + 5 < costmap_settings.cells_y; j += 10) {
      for (int di = 0; di < 2; ++di) {
        for (int dj = 0; dj < 2; ++dj) {
          costmap->setCost(i + di, j + dj, 254);
        }
      }
      for (int di = -2; di < 4; ++di) {
        for (int dj = -2; dj < 4; ++dj) {
          const int ci = static_cast<int>(i) + di;
          const int cj = static_cast<int>(j) + dj;
          if (ci < 0 || cj < 0 ||
            ci >= static_cast<int>(costmap_settings.cells_x) ||
            cj >= static_cast<int>(costmap_settings.cells_y))
          {
            continue;
          }
          if (costmap->getCost(ci, cj) == 0) {
            costmap->setCost(ci, cj, 253);
          }
        }
      }
    }
  }

  auto run = [&](bool accel_on) {
      std::vector<rclcpp::Parameter> params;
      params.emplace_back("dummy.batch_size", batch_size);
      params.emplace_back("dummy.time_steps", time_steps);
      params.emplace_back("dummy.iteration_count", 1);
      params.emplace_back("dummy.lookahead_dist", lookahead);
      params.emplace_back("dummy.motion_model", std::string("Omni"));
      params.emplace_back("dummy.critics", critics);
      params.emplace_back("dummy.model_dt", 0.033333333);
      params.emplace_back("controller_frequency", 30.0);
      params.emplace_back("dummy.CostCritic.consider_footprint", false);
      params.emplace_back("dummy.CostCritic.cost_weight", 4.0);
      params.emplace_back("dummy.CostCritic.critical_cost", 300.0);
      params.emplace_back("dummy.PathAlignCritic.cost_weight", 3.0);
      params.emplace_back("dummy.PathFollowCritic.cost_weight", 5.0);
      params.emplace_back("dummy.GoalCritic.cost_weight", 5.0);
      params.emplace_back("dummy.ConstraintCritic.cost_weight", 4.0);
      const double a = accel_on ? 4.5 : 0.0;
      const double ay = accel_on ? 3.0 : 0.0;
      params.emplace_back("dummy.ax_max", a);
      params.emplace_back("dummy.ax_min", -a);
      params.emplace_back("dummy.ay_max", ay);
      params.emplace_back("dummy.ay_min", -ay);
      params.emplace_back("dummy.az_max", 0.0);

      auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "bench_node", rclcpp::NodeOptions().parameter_overrides(params));
      auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
      auto optimizer = getDummyOptimizer(node, costmap_ros, parameters_handler.get());

      TestPose start_pose = costmap_settings.getCenterPose();
      TestPathSettings path_settings{start_pose, path_points, costmap_settings.resolution,
        costmap_settings.resolution};
      auto pose = getDummyPointStamped(node, start_pose);
      auto velocity = getDummyTwist();
      auto path = getIncrementalDummyPath(node, path_settings);
      nav2_core::GoalChecker * goal_checker{nullptr};

      int throws = 0;
      for (int i = 0; i < 5; ++i) {
        try {
          optimizer->evalControl(pose, velocity, path, goal_checker);
        } catch (const std::exception &) {
          ++throws;
        }
      }
      const int iters = 20;
      const auto t0 = std::chrono::steady_clock::now();
      for (int i = 0; i < iters; ++i) {
        try {
          optimizer->evalControl(pose, velocity, path, goal_checker);
        } catch (const std::exception &) {
          ++throws;
        }
      }
      const auto t1 = std::chrono::steady_clock::now();
      optimizer->shutdown();
      return std::make_pair(
        std::chrono::duration<double, std::milli>(t1 - t0).count() / iters, throws);
    };

  const auto off = run(false);
  const auto on = run(true);
  std::cout << "[bench] 整周期 evalControl(batch=" << batch_size << ", steps=" << time_steps
            << "): 加速度关闭 " << off.first << " ms/次（异常 " << off.second
            << " 次），开启 " << on.first << " ms/次（异常 " << on.second
            << " 次），比值 " << (on.first / off.first) << "x" << std::endl;

  EXPECT_GT(off.first, 0.0);
  EXPECT_GT(on.first, 0.0);
}
