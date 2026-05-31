// Copyright 2025 LemperorD
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

#include "buaa_sentry_relocalization/buaa_sentry_relocalizaiton.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace buaa_sentry_relocalization
{

BuaaSentryRelocalizationNode::BuaaSentryRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("buaa_sentry_relocalization", options),
  gicp_aligned_(false),
  coarse_result_(Eigen::Isometry3d::Identity()),
  gicp_result_(Eigen::Isometry3d::Identity())
{
  // basic parameter
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  // kiss_matcher param
  this->declare_parameter("resolution", 0.2);
  this->declare_parameter("use_quatro", true);
  // small_gicp param
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("use_quatro", use_quatro_);
  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);

  registered_scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "registered_scan", 5, std::bind(&BuaaSentryRelocalizationNode::pcdCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);
  pcl::removeNaNFromPointCloud(*global_map_, *global_map_, tgt_indices);

  // Init KISS
  kiss_config_ = kiss_matcher::KISSMatcherConfig(resolution_);
  kiss_config_.use_quatro_ = use_quatro_;
  matcher_ = std::make_unique<kiss_matcher::KISSMatcher>(kiss_config_);
  target_vec_ = convertCloudToVec(*global_map_);

  // Init small_gicp
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  coarse_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(100),  // 粗配准：10Hz
  std::bind(&BuaaSentryRelocalizationNode::coarseAlign, this));

  gicp_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(500),  // 精配准：2Hz
  std::bind(&BuaaSentryRelocalizationNode::smallGicpAlign, this));

  transform_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(50),  // 20Hz
  std::bind(&BuaaSentryRelocalizationNode::publishTransform, this));
}

void BuaaSentryRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void BuaaSentryRelocalizationNode::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);

  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *registered_scan_ += *scan;
}

std::vector<Eigen::Vector3f> BuaaSentryRelocalizationNode::convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3f> vec;
  vec.reserve(cloud.size());
  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    vec.emplace_back(pt.x, pt.y, pt.z);
  }
  return vec;
}

void BuaaSentryRelocalizationNode::publishTransform()
{
  if (!gicp_aligned_) return;
  
  if (gicp_result_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = gicp_result_.translation();
  const Eigen::Quaterniond rotation(gicp_result_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void BuaaSentryRelocalizationNode::coarseAlign()
{
  if (registered_scan_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  pcl::removeNaNFromPointCloud(*registered_scan_, *registered_scan_, src_indices);
  source_vec_ = convertCloudToVec(*registered_scan_);
                      
  solution_ = matcher_->estimate(source_vec_, target_vec_);

  if (solution_.valid && matcher_->getNumFinalInliers() > 5 && !gicp_aligned_) {
    coarse_result_.translation() = solution_.translation;
    coarse_result_.linear() = solution_.rotation;
  }
  
  return;
}

void BuaaSentryRelocalizationNode::smallGicpAlign()
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);

  if (registered_scan_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *registered_scan_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  auto result = register_->align(*target_, *source_, *target_tree_, coarse_result_);

  if (result.converged) {
    gicp_result_ = coarse_result_ = result.T_target_source;
    gicp_aligned_ = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
    gicp_aligned_ = false;
    return;
  }

  registered_scan_->clear();
  return;
}

}  // namespace buaa_sentry_relocalization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(buaa_sentry_relocalization::BuaaSentryRelocalizationNode)
