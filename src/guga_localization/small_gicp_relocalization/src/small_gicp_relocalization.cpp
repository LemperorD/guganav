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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("max_iterations", 100);
  this->declare_parameter("max_roll_pitch_step", 0.0);
  this->declare_parameter("max_tz_step", 0.0);
  this->declare_parameter("error_max",10.0);
  this->declare_parameter("tz_max_",0.5);
  this->declare_parameter("roll_max_",0.001);
  this->declare_parameter("pitch_max_",0.001);
  this->declare_parameter("map_boundary_margin", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("max_roll_pitch_step", max_roll_pitch_step_);
  this->get_parameter("max_tz_step", max_tz_step_);
  this->get_parameter("error_max",error_max_);
  this->get_parameter("tz_max_",tz_max_);
  this->get_parameter("roll_max_",roll_max_);
  this->get_parameter("pitch_max_",pitch_max_);
  this->get_parameter("map_boundary_margin", map_boundary_margin_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  initial_result_t_ = result_t_;
  RCLCPP_INFO(this->get_logger(), "initial_tf:%f %f %f",init_pose_[0], init_pose_[1], init_pose_[2]);
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<small_gicp::Registration<
    small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::NullFactor,
    small_gicp::DistanceRejector, StepLimitOptimizer>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    [this] (const sensor_msgs::msg::PointCloud2::SharedPtr msg){registeredPcdCallback(msg);});

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    [this] (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){initialPoseCallback(msg);});

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    [this] () {performRegistration();});

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    [this] (){publishTransform();});

  // A component constructor runs inside the container's load_node service.
  // Waiting for TF here blocks every component queued behind this node,
  // including the Nav2 servers that own the costmaps.
  map_initialization_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), [this] (){initializeGlobalMap();});
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());
}

void SmallGicpRelocalizationNode::initializeGlobalMap()
{
  if (global_map_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot initialize an empty global map");
    map_initialization_timer_->cancel();
    return;
  }

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  try {
    const auto tf_stamped =
      tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
    odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
    return;
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(), "odom_to_lidar_odom: translation = "
                          << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                          << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);

  // Map footprint used to detect a transform that pushes the robot off the map
  pcl::PointXYZ min_bound;
  pcl::PointXYZ max_bound;
  pcl::getMinMax3D(*global_map_, min_bound, max_bound);
  map_min_bound_ = min_bound.getVector3fMap().cast<double>();
  map_max_bound_ = max_bound.getVector3fMap().cast<double>();
  has_map_bounds_ = true;
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Map bounds: min = " << map_min_bound_.transpose() << ", max = " << map_max_bound_.transpose());

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  global_map_ready_ = true;
  map_initialization_timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Global map registration target is ready");
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (!global_map_ready_) {
    return;
  }

  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_roll_pitch_step = max_roll_pitch_step_;
  register_->optimizer.max_tz_step = max_tz_step_;
  if (previous_error_ > error_max_) {
    register_->optimizer.max_iterations = max_iterations_;
  } 
  else {
    register_->optimizer.max_iterations = 10;
  } 
  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);
    previous_error_ = result.error;
  if(!registration_initial_){
    initial_result_t_=result.T_target_source;
    registration_initial_=true;
  }
  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
  }

  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::checkRegistration(Eigen::Isometry3d& result_t,
    double tz_max_,double roll_max_,double pitch_max_)
{
    Eigen::Isometry3d& T=result_t;
    Eigen::Vector3d initial_rpy=initial_result_t_.linear().eulerAngles(0, 1, 2);
    Eigen::Vector3d rpy = T.linear().eulerAngles(0, 1, 2);
    if(fabs(rpy[2]-initial_rpy[2])>=0.78){
      
      rpy[2]=initial_rpy[2];
    }
    T.linear()=
    Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) 
    .toRotationMatrix();
  }

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }
  checkRegistration(result_t_,tz_max_,roll_max_,pitch_max_);
  
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

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
