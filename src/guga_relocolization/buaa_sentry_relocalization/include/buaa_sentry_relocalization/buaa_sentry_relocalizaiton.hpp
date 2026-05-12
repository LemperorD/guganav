#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <atomic>
#include <memory>

// external libs
#include <kiss_matcher/KISSMatcher.hpp>
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"

namespace buaa_sentry_relocalization
{

class BuaaSentryRelocalizationNode : public rclcpp::Node
{
public:
  explicit BuaaSentryRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  // ==== Callbacks ====
  void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // ==== Helper ====
  void loadGlobalMap(const std::string & file_name);
  std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void publishTransform();
  void coarseAlign();
  void smallGicpAlign();

  // ==== ROS Interfaces ====
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ==== basic variable ====
  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  bool gicp_aligned_; // flag
 
  // ==== timer ====
  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr coarse_timer_;
  rclcpp::TimerBase::SharedPtr gicp_timer_;
  
  // ==== kiss_matcher variable ====
  float resolution_;
  bool use_quatro_;
  std::vector<Eigen::Vector3f> target_vec_;
  std::vector<Eigen::Vector3f> source_vec_;
  std::vector<int> src_indices;
  std::vector<int> tgt_indices;
  kiss_matcher::KISSMatcherConfig kiss_config_;
  std::unique_ptr<kiss_matcher::KISSMatcher> matcher_;
  kiss_matcher::RegistrationSolution solution_;
  Eigen::Isometry3d coarse_result_;

  // ==== small_gicp variable ====
  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;
  Eigen::Isometry3d gicp_result_;

  // ==== Runtime ====
  std::mutex cloud_mutex_;
};

}  // namespace buaa_sentry_relocalization