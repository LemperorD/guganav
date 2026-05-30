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

#include "kiss_matcher_relocalization/kiss_matcher_relocalization.hpp"

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/filter.h"

namespace kiss_matcher_relocalization
{

KissMatcherRelocalizationNode::KissMatcherRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("kiss_matcher_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("resolution", 0.2);
  this->declare_parameter("use_quatro", true);

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("use_quatro", use_quatro_);

  registered_scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  pcl::removeNaNFromPointCloud(*global_map_, *global_map_, tgt_indices);
  target_vec = convertCloudToVec(*global_map_);

  config_ = kiss_matcher::KISSMatcherConfig(resolution_);
  config_.use_quatro_ = use_quatro_;
  matcher_ = std::make_unique<kiss_matcher::KISSMatcher>(config_);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  "registered_scan", 10,
  std::bind(&KissMatcherRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(100),  // 10 Hz
  std::bind(&KissMatcherRelocalizationNode::performMatcher, this));

  transform_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(50),  // 20 Hz
  std::bind(&KissMatcherRelocalizationNode::publishTransform, this));
}

void KissMatcherRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *registered_scan_ += *scan;
}

void KissMatcherRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
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

std::vector<Eigen::Vector3f> KissMatcherRelocalizationNode::convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3f> vec;
  vec.reserve(cloud.size());
  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    vec.emplace_back(pt.x, pt.y, pt.z);
  }
  return vec;
}

void KissMatcherRelocalizationNode::performMatcher()
{
  if (registered_scan_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  pcl::removeNaNFromPointCloud(*registered_scan_, *registered_scan_, src_indices);
  source_vec = convertCloudToVec(*registered_scan_);
  
  solution_ = matcher_->estimate(source_vec, target_vec);

  matcher_->print();
  euler_ = solution_.rotation.eulerAngles(2, 1, 0);
  time_ = getTime(*matcher_);
  RCLCPP_INFO(this->get_logger(), "Time: %f", time_);

  if (solution_.valid && matcher_->getNumFinalInliers() > 5) {
    result_t_.translation() = solution_.translation;
    result_t_.linear() = solution_.rotation;
  }

  registered_scan_->clear();
}

void KissMatcherRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

double KissMatcherRelocalizationNode::getTime(kiss_matcher::KISSMatcher& m)
{
  double t_p = m.getProcessingTime();
  double t_e = m.getExtractionTime();
  double t_r = m.getRejectionTime();
  double t_m = m.getMatchingTime();
  double t_s = m.getSolverTime();

  return t_p + t_e + t_r + t_m + t_s;
}

} // namespace kiss_matcher_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiss_matcher_relocalization::KissMatcherRelocalizationNode)