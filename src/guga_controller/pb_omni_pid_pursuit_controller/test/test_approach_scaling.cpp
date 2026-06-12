#include "gtest/gtest.h"

#include "pb_omni_pid_pursuit_controller/node/omni_pid_pursuit_controller_node.hpp"
#include "pb_omni_pid_pursuit_controller/core/pathhandler.hpp"
#include "pb_omni_pid_pursuit_controller/core/types.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include <memory>

namespace pb_omni_pid_pursuit_controller {
  namespace {

    [[nodiscard]] nav_msgs::msg::Path makePath(double x_distance) {
      nav_msgs::msg::Path path;
      path.header.frame_id = "odom";
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "odom";
      ps.pose.position.y = 0.0;
      if (x_distance <= 0.0) {
        path.poses.push_back(ps);
        return path;
      }
      int steps = std::max(10, static_cast<int>(x_distance / 0.01));
      double step = x_distance / steps;
      for (int i = 0; i <= steps; ++i) {
        ps.pose.position.x = i * step;
        path.poses.push_back(ps);
      }
      return path;
    }

  }  // namespace

  class ApproachScalingTest : public ::testing::Test {
  protected:
    static void SetUpTestSuite() {
      rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
      rclcpp::shutdown();
    }

    void SetUp() override {
      parent_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
      tf_ = std::make_shared<tf2_ros::Buffer>(parent_->get_clock());
      costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
          "test_costmap");
      costmap_->on_configure(rclcpp_lifecycle::State{});
      controller_.configure(parent_, "test", tf_, costmap_);
    }

    OmniPidPursuitControllerNode controller_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  };

  TEST_F(ApproachScalingTest, ScalingFactorFarFromGoal) {
    auto path = makePath(10.0);
    double factor = controller_.approachVelocityScalingFactor(path);
    EXPECT_DOUBLE_EQ(factor, 1.0);
  }

  TEST_F(ApproachScalingTest, ScalingFactorNearGoal) {
    controller_.config_.approach_velocity_scaling_dist = 1.0;
    auto path = makePath(0.5);
    double factor = controller_.approachVelocityScalingFactor(path);
    EXPECT_DOUBLE_EQ(factor, 0.5);
  }

  TEST_F(ApproachScalingTest, FloorEnforcedWhenApproachReducesBelowMin) {
    controller_.config_.approach_velocity_scaling_dist = 1.0;
    controller_.config_.min_approach_linear_velocity = 0.05;
    auto path = makePath(0.1);
    double lin_vel = 0.4;
    controller_.applyApproachVelocityScaling(path, lin_vel);
    double scaled = 0.4 * 0.1;  // 0.04 < 0.05
    EXPECT_LT(scaled, 0.05);
    EXPECT_DOUBLE_EQ(lin_vel, 0.05);  // floor enforced
  }
}  // namespace pb_omni_pid_pursuit_controller
