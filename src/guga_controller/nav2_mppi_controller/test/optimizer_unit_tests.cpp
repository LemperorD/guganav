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

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/optimizer.hpp"

// Tests main optimizer functions

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi;  // NOLINT
using namespace mppi::critics;  // NOLINT
using namespace mppi::utils;  // NOLINT
using xt::evaluation_strategy::immediate;

class OptimizerTester : public Optimizer
{
public:
  OptimizerTester()
  : Optimizer() {}

  void testSetDiffModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("DiffDrive"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<DiffDriveMotionModel *>(motion_model_.get()));
    EXPECT_FALSE(isHolonomic());
  }

  void testSetOmniModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("Omni"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<OmniMotionModel *>(motion_model_.get()));
    EXPECT_TRUE(isHolonomic());
  }

  void testSetAckModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("Ackermann"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<AckermannMotionModel *>(motion_model_.get()));
    EXPECT_FALSE(isHolonomic());
  }

  void testSetRandModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    try {
      setMotionModel("Random");
      FAIL();
    } catch (...) {
      SUCCEED();
    }
    EXPECT_EQ(motion_model_.get(), nullptr);
  }

  void resetMotionModel()
  {
    motion_model_.reset();
  }

  void setOffsetWrapper(const double freq)
  {
    return setOffset(freq);
  }

  bool getShiftControlSequence()
  {
    return settings_.shift_control_sequence;
  }

  void fillOptimizerWithGarbage()
  {
    state_.vx = 0.43432 * xt::ones<float>({1000, 10});
    control_sequence_.vx = 342.0 * xt::ones<float>({30});
    control_history_[0] = {43, 5646, 32432};
    costs_ = 5.32 * xt::ones<float>({56453});
    generated_trajectories_.x = 432.234 * xt::ones<float>({7865, 1});
  }

  void testReset()
  {
    reset();

    EXPECT_EQ(state_.vx, xt::zeros<float>({1000, 50}));
    EXPECT_EQ(control_sequence_.vx, xt::zeros<float>({50}));
    EXPECT_EQ(control_history_[0].vx, 0.0);
    EXPECT_EQ(control_history_[0].vy, 0.0);
    EXPECT_NEAR(xt::sum(costs_, immediate)(), 0, 1e-6);
    EXPECT_EQ(generated_trajectories_.x, xt::zeros<float>({1000, 50}));
  }

  bool fallbackWrapper(bool fail)
  {
    return fallback(fail);
  }

  void testPrepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
  {
    prepare(robot_pose, robot_speed, plan, goal_checker);

    EXPECT_EQ(critics_data_.goal_checker, nullptr);
    EXPECT_NEAR(xt::sum(costs_, immediate)(), 0, 1e-6);  // should be reset
    EXPECT_FALSE(critics_data_.fail_flag);  // should be reset
    EXPECT_FALSE(critics_data_.motion_model->isHolonomic());  // object is valid + diff drive
    EXPECT_FALSE(critics_data_.furthest_reached_path_point.has_value());  // val is not set
    EXPECT_FALSE(critics_data_.path_pts_valid.has_value());  // val is not set
    EXPECT_EQ(state_.pose.pose.position.x, 999);
    EXPECT_EQ(state_.speed.linear.y, 4.0);
    EXPECT_EQ(path_.x.shape(0), 17u);
  }

  void shiftControlSequenceWrapper()
  {
    return shiftControlSequence();
  }

  std::pair<double, double> getVelLimits()
  {
    auto & s = settings_;
    return {s.constraints.vx_min, s.constraints.vx_max};
  }

  void applyControlSequenceConstraintsWrapper()
  {
    return applyControlSequenceConstraints();
  }

  void applyControlSequenceInterIterationConstraintsWrapper()
  {
    return applyControlSequenceInterIterationConstraints();
  }

  void propagateStateVelocitiesFromInitialsWrapper(models::State & state)
  {
    return propagateStateVelocitiesFromInitials(state);
  }

  models::OptimizerSettings & grabSettings()
  {
    return settings_;
  }

  models::State & grabState()
  {
    return state_;
  }

  void setLastCommandVel(const double vx, const double vy, const double wz)
  {
    last_command_vel_.linear.x = vx;
    last_command_vel_.linear.y = vy;
    last_command_vel_.angular.z = wz;
  }

  void prepareWrapper(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & plan)
  {
    prepare(pose, speed, plan, nullptr);
  }

  models::ControlSequence & grabControlSequence()
  {
    return control_sequence_;
  }

  void testupdateStateVels()
  {
    // updateInitialStateVelocities
    models::State state;
    state.reset(1000, 50);
    state.speed.linear.x = 5.0;
    state.speed.linear.y = 1.0;
    state.speed.angular.z = 6.0;
    state.cvx = 0.75 * xt::ones<float>({1000, 50});
    state.cvy = 0.5 * xt::ones<float>({1000, 50});
    state.cwz = 0.1 * xt::ones<float>({1000, 50});
    updateInitialStateVelocities(state);
    EXPECT_NEAR(state.vx(0, 0), 5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), 1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), 6.0, 1e-6);

    // propagateStateVelocitiesFromInitials
    propagateStateVelocitiesFromInitials(state);
    EXPECT_NEAR(state.vx(0, 0), 5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), 1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), 6.0, 1e-6);
    EXPECT_NEAR(state.vx(0, 1), 0.75, 1e-6);
    EXPECT_NEAR(state.vy(0, 1), 0.5, 1e-6);
    EXPECT_NEAR(state.wz(0, 1), 0.1, 1e-6);

    // Putting them together: updateStateVelocities
    state.reset(1000, 50);
    state.speed.linear.x = -5.0;
    state.speed.linear.y = -1.0;
    state.speed.angular.z = -6.0;
    state.cvx = -0.75 * xt::ones<float>({1000, 50});
    state.cvy = -0.5 * xt::ones<float>({1000, 50});
    state.cwz = -0.1 * xt::ones<float>({1000, 50});
    updateStateVelocities(state);
    EXPECT_NEAR(state.vx(0, 0), -5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), -1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), -6.0, 1e-6);
    EXPECT_NEAR(state.vx(0, 1), -0.75, 1e-6);
    EXPECT_NEAR(state.vy(0, 1), -0.5, 1e-6);
    EXPECT_NEAR(state.wz(0, 1), -0.1, 1e-6);
  }

  geometry_msgs::msg::TwistStamped getControlFromSequenceAsTwistWrapper()
  {
    builtin_interfaces::msg::Time stamp;
    return getControlFromSequenceAsTwist(stamp);
  }

  void integrateStateVelocitiesWrapper(
    models::Trajectories & traj,
    const models::State & state)
  {
    return integrateStateVelocities(traj, state);
  }
};

TEST(OptimizerTests, BasicInitializedFunctions)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Should be empty of size batches x time steps
  // and tests getting set params: time_steps, batch_size, controller_frequency
  auto trajs = optimizer_tester.getGeneratedTrajectories();
  EXPECT_EQ(trajs.x.shape(0), 1000u);
  EXPECT_EQ(trajs.x.shape(1), 50u);
  EXPECT_EQ(trajs.x, xt::zeros<float>({1000, 50}));

  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto traj = optimizer_tester.getOptimizedTrajectory();
  EXPECT_EQ(traj(5, 0), 0.0);  // x
  EXPECT_EQ(traj(5, 1), 0.0);  // y
  EXPECT_EQ(traj(5, 2), 0.0);  // yaw
  EXPECT_EQ(traj.shape(0), 50u);
  EXPECT_EQ(traj.shape(1), 3u);

  optimizer_tester.reset();
  optimizer_tester.shutdown();
}

TEST(OptimizerTests, TestOptimizerMotionModels)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Diff Drive should be non-holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetDiffModel();

  // Omni Drive should be holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  // // Ackermann should be non-holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetAckModel();

  // // Rand should fail
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetRandModel();
}

TEST(OptimizerTests, setOffsetTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test offsets are properly set based on relationship of model_dt and controller frequency
  // Also tests getting set model_dt parameter.
  EXPECT_THROW(optimizer_tester.setOffsetWrapper(1.0), std::runtime_error);
  EXPECT_NO_THROW(optimizer_tester.setOffsetWrapper(30.0));
  EXPECT_FALSE(optimizer_tester.getShiftControlSequence());
  EXPECT_NO_THROW(optimizer_tester.setOffsetWrapper(10.0));
  EXPECT_TRUE(optimizer_tester.getShiftControlSequence());
}

TEST(OptimizerTests, resetTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Tests resetting the full state of all the functions after filling with garbage
  optimizer_tester.fillOptimizerWithGarbage();
  optimizer_tester.testReset();
}

TEST(OptimizerTests, FallbackTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test fallback logic, also tests getting set param retry_attempt_limit
  // Because retry set to 2, it should attempt soft resets 2x before throwing exception
  // for hard reset
  EXPECT_FALSE(optimizer_tester.fallbackWrapper(false));
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  EXPECT_THROW(optimizer_tester.fallbackWrapper(true), std::runtime_error);
}

TEST(OptimizerTests, PrepareTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test Prepare function to set the state of the robot pose/speed on new cycle
  // Populate the contents with things easily identifiable if correct
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 999;
  geometry_msgs::msg::Twist speed;
  speed.linear.y = 4.0;
  nav_msgs::msg::Path path;
  path.poses.resize(17);

  optimizer_tester.testPrepare(pose, speed, path, nullptr);
}

TEST(OptimizerTests, shiftControlSequenceTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test shiftControlSequence by setting the 2nd value to something unique to neighbors
  auto & sequence = optimizer_tester.grabControlSequence();
  sequence.reset(100);
  sequence.vx(0) = 9999;
  sequence.vx(1) = 6;
  sequence.vx(2) = 888;
  sequence.vy(0) = 9999;
  sequence.vy(1) = 6;
  sequence.vy(2) = 888;
  sequence.wz(0) = 9999;
  sequence.wz(1) = 6;
  sequence.wz(2) = 888;

  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  optimizer_tester.shiftControlSequenceWrapper();

  EXPECT_EQ(sequence.vx(0), 6);
  EXPECT_EQ(sequence.vy(0), 6);
  EXPECT_EQ(sequence.wz(0), 6);
  EXPECT_EQ(sequence.vx(1), 888);
  EXPECT_EQ(sequence.vy(1), 888);
  EXPECT_EQ(sequence.wz(1), 888);
  EXPECT_EQ(sequence.vx(2), 0);
  EXPECT_EQ(sequence.vy(2), 0);
  EXPECT_EQ(sequence.wz(2), 0);
}

TEST(OptimizerTests, SpeedLimitTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test Speed limits API
  auto [v_min, v_max] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max, 0.5f);
  EXPECT_EQ(v_min, -0.35f);
  optimizer_tester.setSpeedLimit(0, false);
  auto [v_min2, v_max2] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max2, 0.5f);
  EXPECT_EQ(v_min2, -0.35f);
  optimizer_tester.setSpeedLimit(50.0, true);
  auto [v_min3, v_max3] = optimizer_tester.getVelLimits();
  EXPECT_NEAR(v_max3, 0.5 / 2.0, 1e-3);
  EXPECT_NEAR(v_min3, -0.35 / 2.0, 1e-3);
  optimizer_tester.setSpeedLimit(0, true);
  auto [v_min4, v_max4] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max4, 0.5f);
  EXPECT_EQ(v_min4, -0.35f);
  optimizer_tester.setSpeedLimit(0.75, false);
  auto [v_min5, v_max5] = optimizer_tester.getVelLimits();
  EXPECT_NEAR(v_max5, 0.75, 1e-3);
  EXPECT_NEAR(v_min5, -0.5249, 1e-2);
}

TEST(OptimizerTests, applyControlSequenceConstraintsTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.75));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test constraints being applied to ensure feasibility of trajectories
  // Also tests param get of set vx/vy/wz min/maxes

  // Set model to omni to consider holonomic vy elements
  // Ack is not tested here because `applyConstraints` is covered in detail
  // in motion_models_test.cpp
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto & sequence = optimizer_tester.grabControlSequence();

  // Test boundary of limits
  sequence.vx = xt::ones<float>({50});
  sequence.vy = 0.75 * xt::ones<float>({50});
  sequence.wz = 2.0 * xt::ones<float>({50});
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_EQ(sequence.vx, xt::ones<float>({50}));
  EXPECT_EQ(sequence.vy, 0.75 * xt::ones<float>({50}));
  EXPECT_EQ(sequence.wz, 2.0 * xt::ones<float>({50}));

  // Test breaking limits sets to maximum
  sequence.vx = 5.0 * xt::ones<float>({50});
  sequence.vy = 5.0 * xt::ones<float>({50});
  sequence.wz = 5.0 * xt::ones<float>({50});
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_EQ(sequence.vx, xt::ones<float>({50}));
  EXPECT_EQ(sequence.vy, 0.75 * xt::ones<float>({50}));
  EXPECT_EQ(sequence.wz, 2.0 * xt::ones<float>({50}));

  // Test breaking limits sets to minimum
  sequence.vx = -5.0 * xt::ones<float>({50});
  sequence.vy = -5.0 * xt::ones<float>({50});
  sequence.wz = -5.0 * xt::ones<float>({50});
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_EQ(sequence.vx, -1.0 * xt::ones<float>({50}));
  EXPECT_EQ(sequence.vy, -0.75 * xt::ones<float>({50}));
  EXPECT_EQ(sequence.wz, -2.0 * xt::ones<float>({50}));
}

TEST(OptimizerTests, updateStateVelocitiesTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.60));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test settings of the state to the initial robot speed to start rollout
  // Set model to omni to consider holonomic vy elements
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  optimizer_tester.testupdateStateVels();
}

TEST(OptimizerTests, getControlFromSequenceAsTwistTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.60));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test conversion of control sequence into a Twist command to execute
  auto & sequence = optimizer_tester.grabControlSequence();
  sequence.vx = 0.25 * xt::ones<float>({10});
  sequence.vy = 0.5 * xt::ones<float>({10});
  sequence.wz = 0.1 * xt::ones<float>({10});

  auto diff_t = optimizer_tester.getControlFromSequenceAsTwistWrapper();
  EXPECT_NEAR(diff_t.twist.linear.x, 0.25, 1e-6);
  EXPECT_NEAR(diff_t.twist.linear.y, 0.0, 1e-6);  // Y should not be populated
  EXPECT_NEAR(diff_t.twist.angular.z, 0.1, 1e-6);

  // Set model to omni to consider holonomic vy elements
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto omni_t = optimizer_tester.getControlFromSequenceAsTwistWrapper();
  EXPECT_NEAR(omni_t.twist.linear.x, 0.25, 1e-6);
  EXPECT_NEAR(omni_t.twist.linear.y, 0.5, 1e-6);  // Now it should be
  EXPECT_NEAR(omni_t.twist.angular.z, 0.1, 1e-6);
}

TEST(OptimizerTests, integrateStateVelocitiesTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  // Test integration of velocities for trajectory rollout poses

  // Give it a couple of easy const traj and check rollout, start from 0
  models::State state;
  state.reset(1000, 50);
  models::Trajectories traj;
  state.vx = 0.1 * xt::ones<float>({1000, 50});
  xt::view(state.vx, xt::all(), 0) = xt::zeros<float>({1000});
  state.vy = xt::zeros<float>({1000, 50});
  state.wz = xt::zeros<float>({1000, 50});

  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);
  EXPECT_EQ(traj.y, xt::zeros<float>({1000, 50}));
  EXPECT_EQ(traj.yaws, xt::zeros<float>({1000, 50}));
  for (unsigned int i = 0; i != traj.x.shape(1); i++) {
    EXPECT_NEAR(traj.x(1, i), i * 0.1 /*vel*/ * 0.1 /*dt*/, 1e-3);
  }

  // Give it a bit of a more complex trajectory to crunch
  state.vy = 0.2 * xt::ones<float>({1000, 50});
  xt::view(state.vy, xt::all(), 0) = xt::zeros<float>({1000});
  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);

  EXPECT_EQ(traj.yaws, xt::zeros<float>({1000, 50}));
  for (unsigned int i = 0; i != traj.x.shape(1); i++) {
    EXPECT_NEAR(traj.x(1, i), i * 0.1 /*vel*/ * 0.1 /*dt*/, 1e-3);
    EXPECT_NEAR(traj.y(1, i), i * 0.2 /*vel*/ * 0.1 /*dt*/, 1e-3);
  }

  // Lets add some angular motion to the mix
  state.vy = xt::zeros<float>({1000, 50});
  state.wz = 0.2 * xt::ones<float>({1000, 50});
  xt::view(state.wz, xt::all(), 0) = xt::zeros<float>({1000});
  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);

  float x = 0;
  float y = 0;
  for (unsigned int i = 1; i != traj.x.shape(1); i++) {
    std::cout << i << std::endl;
    x += (0.1 /*vx*/ * cos(0.2 /*wz*/ * 0.1 /*model_dt*/ * (i - 1))) * 0.1 /*model_dt*/;
    y += (0.1 /*vx*/ * sin(0.2 /*wz*/ * 0.1 /*model_dt*/ * (i - 1))) * 0.1 /*model_dt*/;

    EXPECT_NEAR(traj.x(1, i), x, 1e-6);
    EXPECT_NEAR(traj.y(1, i), y, 1e-6);
  }
}

// 加速度约束默认关闭：不配 ax/ay/az 时必须与移植前逐位一致
TEST(OptimizerTests, AccelConstraintsDisabledByDefault)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("accel_default_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.05));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap_accel_default", "", "dummy_costmap_accel_default");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  auto & settings = optimizer_tester.grabSettings();
  EXPECT_FLOAT_EQ(settings.base_constraints.ax_max, 0.0f);
  EXPECT_FLOAT_EQ(settings.base_constraints.ax_min, 0.0f);
  EXPECT_FLOAT_EQ(settings.base_constraints.ay_max, 0.0f);
  EXPECT_FLOAT_EQ(settings.base_constraints.az_max, 0.0f);
  EXPECT_EQ(settings.sgf_order, 2u);

  // 未启用加速度约束时，状态速度还是"采样控制量直接前移一位"
  models::State state;
  state.reset(1, 5);
  state.cvx = 5.0 * xt::ones<float>({1, 5});
  state.cvy = 3.0 * xt::ones<float>({1, 5});
  state.cwz = 2.0 * xt::ones<float>({1, 5});
  optimizer_tester.propagateStateVelocitiesFromInitialsWrapper(state);
  EXPECT_NEAR(state.vx(0, 1), 5.0, 1e-6);
  EXPECT_NEAR(state.vx(0, 4), 5.0, 1e-6);
  EXPECT_NEAR(state.vy(0, 1), 3.0, 1e-6);
  EXPECT_NEAR(state.wz(0, 4), 2.0, 1e-6);

  optimizer_tester.shutdown();
}

// 迭代间加速度可行性（移植自上游 PR #6072）
TEST(OptimizerTests, InterIterationAccelConstraintsTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("accel_inter_iter_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.05));
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.ay_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter("mppic.ay_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.az_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap_accel_inter_iter", "", "dummy_costmap_accel_inter_iter");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  auto & settings = optimizer_tester.grabSettings();
  auto & state = optimizer_tester.grabState();
  auto & seq = optimizer_tester.grabControlSequence();

  // 20 Hz + model_dt 0.05 → 控制序列前移开启，controller_period = 0.05
  EXPECT_TRUE(settings.shift_control_sequence);
  EXPECT_NEAR(settings.controller_period, 0.05f, 1e-6);
  EXPECT_FLOAT_EQ(settings.constraints.ax_max, 2.0f);
  EXPECT_FLOAT_EQ(settings.constraints.ax_min, -1.0f);

  // 前移模式下 index 0 代表"现在"，应被钉在当前实测速度上
  state.speed.linear.x = 0.1;
  state.speed.linear.y = 0.3;
  state.speed.angular.z = 0.2;
  seq.reset(10);
  seq.vx(0) = 5.0;
  seq.vy(0) = 5.0;
  seq.wz(0) = 5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  EXPECT_NEAR(seq.vx(0), 0.1, 1e-6);
  EXPECT_NEAR(seq.vy(0), 0.3, 1e-6);
  EXPECT_NEAR(seq.wz(0), 0.2, 1e-6);

  // 非前移模式（控制器 40 Hz）：第 0 项按 controller_period 夹到一步可达范围
  settings.shift_control_sequence = false;
  settings.controller_period = 0.025f;
  state.speed.linear.x = 0.0;
  state.speed.linear.y = 0.0;
  state.speed.angular.z = 0.0;
  seq.reset(10);
  seq.vx(0) = 5.0;
  seq.vy(0) = 5.0;
  seq.wz(0) = 5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  EXPECT_NEAR(seq.vx(0), 0.05, 1e-6);  // 0.025 * ax_max(2.0)
  EXPECT_NEAR(seq.vy(0), 0.05, 1e-6);  // 0.025 * ay_max(2.0)
  EXPECT_NEAR(seq.wz(0), 0.05, 1e-6);  // 0.025 * az_max(2.0)

  // 非对称：从 +0.5 往前，硬刹到 -5 应被夹到 0.5 + 0.025*ax_min = 0.475
  state.speed.linear.x = 0.5;
  seq.reset(10);
  seq.vx(0) = -5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  EXPECT_NEAR(seq.vx(0), 0.475, 1e-6);

  // 加速侧：0.5 + 0.025*ax_max = 0.55
  seq.vx(0) = 5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  EXPECT_NEAR(seq.vx(0), 0.55, 1e-6);

  optimizer_tester.shutdown();
}

// 运动模型：加速度约束作用在状态速度上，采样控制量保持原始（PR #5266 语义）
TEST(OptimizerTests, MotionModelAccelClampOnStateVelocities)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("accel_motion_model_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(5));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.ay_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.ay_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.az_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(10.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap_accel_model", "", "dummy_costmap_accel_model");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  // 起步速度 0，控制量全是 5 m/s；model_dt 0.1 与 a_max 1.0 → 每步最多 +0.1
  models::State state;
  state.reset(1, 5);
  state.cvx = 5.0 * xt::ones<float>({1, 5});
  state.cvy = 5.0 * xt::ones<float>({1, 5});
  state.cwz = 5.0 * xt::ones<float>({1, 5});
  optimizer_tester.propagateStateVelocitiesFromInitialsWrapper(state);
  EXPECT_NEAR(state.vx(0, 1), 0.1, 1e-6);
  EXPECT_NEAR(state.vx(0, 2), 0.2, 1e-6);
  EXPECT_NEAR(state.vx(0, 4), 0.4, 1e-6);
  EXPECT_NEAR(state.vy(0, 4), 0.4, 1e-6);
  EXPECT_NEAR(state.wz(0, 4), 0.4, 1e-6);
  // 采样控制量本身不被夹紧，softmax 才能分辨超限程度
  EXPECT_NEAR(state.cvx(0, 0), 5.0, 1e-6);
  EXPECT_NEAR(state.cwz(0, 4), 5.0, 1e-6);

  // 反向：从 0 起步给 -5，每步最多 -0.1
  state.reset(1, 5);
  state.cvx = -5.0 * xt::ones<float>({1, 5});
  state.cwz = -5.0 * xt::ones<float>({1, 5});
  optimizer_tester.propagateStateVelocitiesFromInitialsWrapper(state);
  EXPECT_NEAR(state.vx(0, 1), -0.1, 1e-6);
  EXPECT_NEAR(state.vx(0, 4), -0.4, 1e-6);

  optimizer_tester.shutdown();
}

// 逐轴独立：上限留 0 表示该轴不限制，而不是"不允许变化"
TEST(OptimizerTests, AccelConstraintsArePerAxis)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("accel_per_axis_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(5));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  // 只给纵向加速度上限，az/ay 保持 0
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(10.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap_accel_per_axis", "", "dummy_costmap_accel_per_axis");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  models::State state;
  state.reset(1, 5);
  state.cvx = 5.0 * xt::ones<float>({1, 5});
  state.cvy = 5.0 * xt::ones<float>({1, 5});
  state.cwz = 2.0 * xt::ones<float>({1, 5});
  optimizer_tester.propagateStateVelocitiesFromInitialsWrapper(state);

  // 纵向被夹到一步 0.1
  EXPECT_NEAR(state.vx(0, 1), 0.1, 1e-6);
  EXPECT_NEAR(state.vx(0, 4), 0.4, 1e-6);
  // wz / vy 未设上限 → 与控制量直通，不能被冻结成 0
  EXPECT_NEAR(state.wz(0, 1), 2.0, 1e-6);
  EXPECT_NEAR(state.wz(0, 4), 2.0, 1e-6);
  EXPECT_NEAR(state.vy(0, 4), 5.0, 1e-6);

  optimizer_tester.shutdown();
}

// 测量滞后补偿（上游 PR #6072）：启用加速度约束时，初始速度按"上一帧指令"往前推
// 一个控制周期，并夹在加速度可达范围内；未启用时严格等于实测速度。
TEST(OptimizerTests, PrepareLatencyCompensationTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("accel_latency_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(5));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.05));
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(-2.0));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap_accel_latency", "", "dummy_costmap_accel_latency");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetDiffModel();

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;  // 实测速度 0，yaw 0.7
  speed.angular.z = 0.7;
  nav_msgs::msg::Path plan;
  plan.poses.resize(17);

  // 上一帧指令 5.0 m/s，但一个控制周期（0.05 s × 2.0 m/s²）只能到 0.1
  optimizer_tester.setLastCommandVel(5.0, 0.0, 0.0);
  optimizer_tester.prepareWrapper(pose, speed, plan);
  auto & state = optimizer_tester.grabState();
  EXPECT_NEAR(state.speed.linear.x, 0.1, 1e-6);
  // az_max 为 0 → yaw 不做补偿，保持实测值
  EXPECT_NEAR(state.speed.angular.z, 0.7, 1e-6);

  optimizer_tester.shutdown();
}
