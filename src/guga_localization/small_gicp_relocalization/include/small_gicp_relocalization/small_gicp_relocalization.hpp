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

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp_relocalization/step_limit_optimizer.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void initializeGlobalMap();
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void regulateRegistration(Eigen::Isometry3d& result_t);
  /// @brief 惯性约束: 候选 tf 相对基准 tf 的平移偏差过大时, 舍弃本次变换并回到基准;
  ///        good 为真(配准质量达标)的一次重定位则被采纳为新的基准 tf。
  ///        基准若来自手动定位(baseline_is_manual_), 则不设漂移上限, 允许注册把它修正回来。
  ///        会直接更新 result_t_ / previous_result_t_。
  /// @param good 配准质量是否达标(RMSE + 内点率 + 未越界, 由调用方判定)
  /// @return true 表示采纳本次结果, false 表示已回退到基准
  bool applyInertialConstraint(const Eigen::Isometry3d & candidate, bool good);
  /// @brief 本次变换是否把机器人推出先验地图外接框; 越界时 robot_in_map 为机器人在地图中的位置
  bool isOutOfMap(const Eigen::Isometry3d & T_map_odom, Eigen::Vector3d & robot_in_map) const;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  bool registration_initial_=false;
  bool accumulation_reset_=false;
  /// @brief RViz 手动给的初始位姿: 只作种子, 等下一次注册通过基准残差检查后再决定是否升级为基准;
  ///        连收敛都做不到时直接丢弃并回退, 不参与每轮重试
  bool pending_initial_pose_{false};
  int num_threads_;
  int num_neighbors_;
  int max_iterations_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  double max_roll_pitch_step_;
  double max_tz_step_;
  double error_max_;
  double good_error_max_;
  double good_inlier_ratio_;
  double baseline_translation_tolerance_;
  double map_boundary_margin_;
  double previous_rmse_{0.0};  ///< 上一次注册的 RMSE (sqrt(error / num_inliers))
  /// @brief 上一次注册的内点率 (num_inliers / 源点数); 与 previous_rmse_ 一起决定迭代脉冲是否拉满
  double previous_inlier_ratio_{0.0};
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d initial_result_t_;
  Eigen::Isometry3d previous_result_t_;
  /// @brief 唯一的基准 tf: 最近一次"残差很小且未越界"的重定位结果,
  ///        既是惯性约束的比较对象, 也是越界/惯性回退的目标
  Eigen::Isometry3d baseline_tf_;
  bool global_map_ready_{false};
  bool has_map_bounds_{false};
  /// @brief 当前基准是否来自手动定位(未过质量门)。为真时跳过惯性漂移检查,
  ///        因为手动基准本身就是"待修正"的, 一旦 GICP 收敛出合格结果就会被替换掉。
  bool baseline_is_manual_{false};
  Eigen::Vector3d map_min_bound_;
  Eigen::Vector3d map_max_bound_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<small_gicp::Registration<
    small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::NullFactor,
    small_gicp::DistanceRejector, StepLimitOptimizer>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  rclcpp::TimerBase::SharedPtr map_initialization_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
