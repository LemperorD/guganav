#include "pb_omni_pid_pursuit_controller/core/pathhandler.hpp"

#include "tf2_ros/buffer.h"

using pb_omni_pid_pursuit_controller::PathHandler;
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

class PathHandlerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(PathHandlerTest, EmptyPlanThrows) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto pub = node->create_publisher<nav_msgs::msg::Path>("test_plan", 1);
  auto costmap =
      std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap->on_configure(rclcpp_lifecycle::State{});

  PathHandler handler(tf, costmap, pub);
  nav_msgs::msg::Path empty_plan;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  EXPECT_THROW(handler.transformGlobalPlan(pose, empty_plan),
               nav2_core::PlannerException);
}
