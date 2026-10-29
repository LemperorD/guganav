// Copyright 2025 Lihan Chen
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
#include <cmath>
#include <limits>

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "small_gicp_relocalization/small_gicp_relocalization.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: LifecycleNode("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity()),
  baseline_tf_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("max_iterations", 100);
  this->declare_parameter("max_roll_pitch_step", 0.05);
  this->declare_parameter("max_tz_step", 0.02);
  this->declare_parameter("error_max", 10.0);
  this->declare_parameter("good_error_max", 0.5);
  this->declare_parameter("good_inlier_ratio", 0.5);
  this->declare_parameter("baseline_translation_tolerance", 1.0);
  this->declare_parameter("baseline_update_translation_threshold", 0.01);
  this->declare_parameter("baseline_update_yaw_threshold", 0.01);
  this->declare_parameter("initial_state_timeout_sec", 10.0);
  this->declare_parameter("suspect_state_timeout_sec", 5.0);
  this->declare_parameter("map_boundary_margin", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter<std::vector<double>>("body_to_base_t", std::vector<double>{});
  this->declare_parameter<std::vector<double>>("body_to_base_r", std::vector<double>{});
  this->declare_parameter<std::vector<double>>("lidar_to_base_t", std::vector<double>{});
  this->declare_parameter<std::vector<double>>("lidar_to_base_r", std::vector<double>{});
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("publish_prior_pcd", true);
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  initial_result_t_ = result_t_;
  previous_result_t_ = result_t_;
  RCLCPP_INFO(
    this->get_logger(), "initial_tf:%f %f %f", init_pose_[0], init_pose_[1], init_pose_[2]);

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

SmallGicpRelocalizationNode::CallbackReturn SmallGicpRelocalizationNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring small_gicp_relocalization");

  // 1. 读取参数
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("publish_prior_pcd", publish_prior_pcd_);
  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("max_roll_pitch_step", max_roll_pitch_step_);
  this->get_parameter("max_tz_step", max_tz_step_);
  this->get_parameter("error_max", error_max_);
  this->get_parameter("good_error_max", good_error_max_);
  this->get_parameter("good_inlier_ratio", good_inlier_ratio_);
  this->get_parameter("baseline_translation_tolerance", baseline_translation_tolerance_);
  this->get_parameter(
    "baseline_update_translation_threshold", baseline_update_translation_threshold_);
  this->get_parameter("baseline_update_yaw_threshold", baseline_update_yaw_threshold_);
  this->get_parameter("initial_state_timeout_sec", initial_state_timeout_sec_);
  this->get_parameter("suspect_state_timeout_sec", suspect_state_timeout_sec_);
  this->get_parameter("map_boundary_margin", map_boundary_margin_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("body_to_base_t", body_to_base_t_);
  this->get_parameter("body_to_base_r", body_to_base_r_);
  this->get_parameter("lidar_to_base_t", lidar_to_base_t_);
  this->get_parameter("lidar_to_base_r", lidar_to_base_r_);
  this->get_parameter("init_pose", init_pose_);

  // 2. 加载全局地图
  global_map_->clear();
  loadGlobalMap(prior_pcd_file_);

  if (global_map_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load global map: %s", prior_pcd_file_.c_str());

    return CallbackReturn::FAILURE;
  }

  // 3. 创建 GICP 配准器
  register_ = std::make_shared<small_gicp::Registration<
    small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::NullFactor,
    small_gicp::DistanceRejector, StepLimitOptimizer>>();

  // 4. 创建 TF 相关对象
  // 如果这些对象已经在构造函数中创建，这里不要重复创建。
  //
  // tf_buffer_ = ...
  // tf_listener_ = ...
  // tf_broadcaster_ = ...

  // 5. 创建发布器
  lost_pub_ = this->create_publisher<std_msgs::msg::Bool>("gicp_lost", 1);

  if (publish_prior_pcd_) {
    prior_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_pcd", 1);
    prior_pcd_timer_ =
      this->create_wall_timer(std::chrono::seconds(1), [this]() { publishPriorPcd(); });
  }

  // 6. 重置定位状态
  accumulated_cloud_->clear();

  global_map_ready_ = false;
  has_map_bounds_ = false;

  global_state_ = State::INITIAL;

  RCLCPP_INFO(this->get_logger(), "small_gicp_relocalization configured");

  return CallbackReturn::SUCCESS;
}

SmallGicpRelocalizationNode::CallbackReturn SmallGicpRelocalizationNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating small_gicp_relocalization");

  // 1. 重置上一轮积累的点云
  accumulated_cloud_->clear();

  // 2. 创建点云订阅
  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { registeredPcdCallback(msg); });

  // 3. 创建手动初始位姿订阅
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      initialPoseCallback(msg);
    });

  // 4. 创建定位定时器
  register_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
    if (global_map_ready_) {
      performRegistration();
    }
  });

  transform_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(50), [this]() { publishTransform(); });

  state_check_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { checkStateTime(); });

  map_initialization_timer_ =
    this->create_wall_timer(std::chrono::seconds(1), [this]() { initializeGlobalMap(); });

  RCLCPP_INFO(this->get_logger(), "small_gicp_relocalization activated");

  return CallbackReturn::SUCCESS;
}

SmallGicpRelocalizationNode::CallbackReturn SmallGicpRelocalizationNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating small_gicp_relocalization");

  // 1. 停止所有定时器
  if (register_timer_) {
    register_timer_->cancel();
  }

  if (transform_timer_) {
    transform_timer_->cancel();
  }

  if (state_check_timer_) {
    state_check_timer_->cancel();
  }

  if (map_initialization_timer_) {
    map_initialization_timer_->cancel();
  }

  if (prior_pcd_timer_) {
    prior_pcd_timer_->cancel();
  }

  // 2. 销毁订阅，避免停用期间继续接收消息
  pcd_sub_.reset();
  initial_pose_sub_.reset();

  // 3. 清理尚未处理的点云
  accumulated_cloud_->clear();

  RCLCPP_INFO(this->get_logger(), "small_gicp_relocalization deactivated");

  return CallbackReturn::SUCCESS;
}

SmallGicpRelocalizationNode::CallbackReturn SmallGicpRelocalizationNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up small_gicp_relocalization");

  // 清理点云
  if (accumulated_cloud_) {
    accumulated_cloud_->clear();
  }

  if (global_map_) {
    global_map_->clear();
  }

  // 释放配准对象
  source_.reset();
  target_.reset();

  source_tree_.reset();
  target_tree_.reset();

  register_.reset();

  // 清理地图状态
  global_map_ready_ = false;
  has_map_bounds_ = false;

  // 清理定位状态
  global_state_ = State::INITIAL;
  accumulation_reset_ = false;

  // 释放发布器
  lost_pub_.reset();
  prior_pcd_pub_.reset();

  RCLCPP_INFO(this->get_logger(), "small_gicp_relocalization cleaned up");

  return CallbackReturn::SUCCESS;
}

SmallGicpRelocalizationNode::CallbackReturn SmallGicpRelocalizationNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down small_gicp_relocalization");

  // 停止后续任务
  if (register_timer_) {
    register_timer_->cancel();
  }

  if (transform_timer_) {
    transform_timer_->cancel();
  }

  if (state_check_timer_) {
    state_check_timer_->cancel();
  }

  if (map_initialization_timer_) {
    map_initialization_timer_->cancel();
  }

  if (prior_pcd_timer_) {
    prior_pcd_timer_->cancel();
  }

  // 销毁订阅
  pcd_sub_.reset();
  initial_pose_sub_.reset();

  RCLCPP_INFO(this->get_logger(), "small_gicp_relocalization shutdown complete");

  return CallbackReturn::SUCCESS;
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
