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

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {  //语义不明确
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

  Eigen::Affine3d odom_to_lidar_odom;
  const bool has_body_transform = body_to_base_t_.size() == 3 && body_to_base_r_.size() == 9;
  const bool has_lidar_transform = lidar_to_base_t_.size() == 3 && lidar_to_base_r_.size() == 9;
  if (has_body_transform || has_lidar_transform) {
    const auto & translation = has_body_transform ? body_to_base_t_ : lidar_to_base_t_;
    const auto & rotation_values = has_body_transform ? body_to_base_r_ : lidar_to_base_r_;
    Eigen::Matrix3d rotation;
    rotation << rotation_values[0], rotation_values[1], rotation_values[2], rotation_values[3],
      rotation_values[4], rotation_values[5], rotation_values[6], rotation_values[7],
      rotation_values[8];
    odom_to_lidar_odom = Eigen::Affine3d::Identity();
    odom_to_lidar_odom.linear() = rotation;
    odom_to_lidar_odom.translation() << translation[0], translation[1], translation[2];
    RCLCPP_INFO(
      this->get_logger(), has_body_transform
                            ? "Using explicit body_to_base transform for prior PCD"
                            : "Using explicit lidar_to_base transform for prior PCD");
  } else {
    try {
      const auto tf_stamped =
        tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      return;
    }
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
  map_max_bound_ = max_bound.getVector3fMap().cast<double>();  //能否一步到位
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

void SmallGicpRelocalizationNode::publishPriorPcd()
{
  if (!global_map_ready_ || global_map_->empty() || !prior_pcd_pub_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*global_map_, message);
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = odom_frame_;
  prior_pcd_pub_->publish(message);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  if (global_state_ == State::LOST && accumulation_reset_) {
    accumulated_cloud_->clear();
    accumulation_reset_ = true;
  }  //如果配准初始化没有成功找到基准tf,意味着机器人发生大幅漂移，需要重置历史点云
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

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);  //考虑突变后reset

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }
  //benchmark是否可用
  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_roll_pitch_step = max_roll_pitch_step_;
  register_->optimizer.max_tz_step = max_tz_step_;
  // 迭代上限(频率脉冲)的触发条件与配准质量判定统一: 上一帧 RMSE 和内点率都达标才走
  // 10 次的快路径, 只要任一不达标, 本次就把迭代上限拉满重新收敛。
  // previous_* 初值为 0, 因此首帧必然走拉满这条路, 不会用 10 次去初始化基准。
  const bool previous_good =
    previous_rmse_ <= error_max_ && previous_inlier_ratio_ >= good_inlier_ratio_;
  register_->optimizer.max_iterations = previous_good ? 10 : max_iterations_;
  register_->optimizer.max_iterations =
    global_state_ == State::LOST ? 0 : register_->optimizer.max_iterations;
  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);
  // RMSE = sqrt(总误差 / 内点数); error 是各内点 error 之和(每个已是平方量), 开方后与内点数无关,
  // 便于跨帧/跨块比较, 也便于按"残差多大"直观设阈值
  const double rmse = result.num_inliers > 0
                        ? std::sqrt(result.error / static_cast<double>(result.num_inliers))
                        : std::numeric_limits<double>::infinity();
  // 内点率 = 有有效对应的源点数 / 源点总数; 只靠 RMSE 容易被"只匹配上少数点"的退化解骗过
  const double inlier_ratio = source_->empty() ? 0.0
                                               : static_cast<double>(result.num_inliers) /
                                                   static_cast<double>(source_->size());
  previous_rmse_ = rmse;
  previous_inlier_ratio_ = inlier_ratio;
  auto to_string = [this]() {
    if (global_state_ == State::INITIAL) {
      return "INITIAL";
    }
    if (global_state_ == State::FINE) {
      return "FINE";
    }
    if (global_state_ == State::SUSPECT) {
      return "SUSPECT";
    }
    return "LOST";
  };
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "GICP RMSE: %.4f, inlier ratio: %.2f (inliers: %zu, converged: %d),GICP STATE: %s ", rmse,
    inlier_ratio, result.num_inliers, static_cast<int>(result.converged), to_string());

  if (!result.converged && global_state_ == State::INITIAL) {
    // 手动给的种子连收敛都做不到, 就必然过不了 good 这道质量门, 直接丢弃:
    // 清掉 pending 并把种子复位到基准, 免得每一轮都从同一个错误位姿重新迭代。
    // (走到这里必然已有基准: 没有基准的手动位姿在 initialPoseCallback 里就直接生效了。)
    result_t_ = previous_result_t_ = initial_result_t_;
    RCLCPP_WARN(
      this->get_logger(),
      "Initial pose rejected: GICP did not converge, falling back to the previous tf");
    accumulated_cloud_->clear();
    return;
  }

  const Eigen::Isometry3d candidate = result.T_target_source;
  Eigen::Vector3d robot_in_map;
  // 配准质量 = RMSE 足够小 + 内点率足够高 + 未越界
  const bool out_of_map = isOutOfMap(candidate, robot_in_map);
  const bool good = rmse < good_error_max_ && inlier_ratio > good_inlier_ratio_ && !out_of_map;
  // 还没有基准时, 不合格的候选既不能冻结成基准, 也不往外发布;
  // 只要没越界就留作下一轮 GICP 的种子, 让配准继续向地图收敛,
  // 否则每轮都从旧种子重算, 永远得不到更优的结果。
  auto hold_candidate_until_good = [this, &rmse, &inlier_ratio, out_of_map](
                                     const Eigen::Isometry3d & candidate, const char * source) {
    if (!out_of_map) {
      previous_result_t_ = candidate;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "%s rejected by baseline check (RMSE %.4f, inlier ratio %.2f), "
      "holding the seed and retrying until a qualified baseline is found",
      source, rmse, inlier_ratio);
  };

  if (global_state_ == State::INITIAL) {
    // RViz 手动给的初始位姿: 跳过惯性漂移检查(否则会被判成大幅跳变直接丢弃),
    // 但仍然要过基准残差检查 —— 只有 RMSE+内点率达标且未越界, 才升级为新的基准 tf。
    // 走到这里必然已经有基准: 没有基准的手动位姿在 initialPoseCallback 里就直接生效了。
    if (good) {
      global_state_ = State::FINE;
      initial_result_t_ = baseline_tf_ = candidate;
      result_t_ = previous_result_t_ = candidate;
      RCLCPP_INFO(this->get_logger(), "Initial pose accepted as the new baseline tf");
    } else {
      global_state_ = State::SUSPECT;
      RCLCPP_WARN(
        this->get_logger(),
        "Initial pose rejected by baseline check (RMSE %.4f, inlier ratio %.2f), "
        "keeping the previous baseline",
        rmse, inlier_ratio);
      result_t_ = previous_result_t_ = initial_result_t_;
    }
  } else if (global_state_ != State::FINE) {
    // 首次基准初始化同样要过质量门: init_pose 只是种子, 可能离真值很远,
    // 只有 RMSE+内点率达标且未越界的第一次重定位才冻结为基准 tf;
    // 不合格的候选不发布也不冻结, 先留着收敛, 等出现合格结果再更新基准。
    if (good) {
      global_state_ = State::FINE;
      baseline_tf_ = initial_result_t_ = candidate;
      result_t_ = previous_result_t_ = candidate;
      RCLCPP_INFO(this->get_logger(), "First registration accepted as the new baseline tf");
    } else {
      hold_candidate_until_good(candidate, "First registration");
    }
    applyInertialConstraint(candidate, good);
  }
  // 基准误差过大 -> 不再信任它: 作废, 回到"无基准"状态, 等下一次合格结果或手动位姿重建。
  // 期间对外继续发布最后这个基准(冻结), 不会把发散的候选漏出去。
  if (global_state_ == State::FINE && !good) {
    global_state_ = State::SUSPECT;
    accumulation_reset_ = false;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Baseline error too large (RMSE %.4f > %.2f), dropping the baseline", rmse, error_max_);
  }

  // 对外发布的一直是基准 tf, 候选(哪怕被采纳)只用于刷新基准, 绝不直接发布;
  // 否则每轮 GICP 候选都会把 map->odom 抖一次。没有基准时保持上一次发布值不变。
  if (global_state_ != State::INITIAL) {
    result_t_ = initial_result_t_;
  }

  accumulated_cloud_->clear();
}

bool SmallGicpRelocalizationNode::applyInertialConstraint(
  const Eigen::Isometry3d & candidate, bool good)
{
  const double drift = (candidate.translation() - baseline_tf_.translation()).norm();
  if (drift > baseline_translation_tolerance_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Inertial constraint: candidate map->odom drifts %.2f m from the baseline (limit %.2f m), "
      "discarding it and holding the baseline",
      drift, baseline_translation_tolerance_);
    result_t_ = previous_result_t_ = baseline_tf_;
    return false;
  }

  previous_result_t_ = candidate;
  // 只有"效果好"(RMSE 小且内点率高, 未越界)的一次重定位, 才被采纳为新的基准 tf;
  // 基准是越界/惯性回退的目标, 必须保证它本身是可信的。
  if (good) {
    if (global_state_ != State::INITIAL) {
      const Eigen::Isometry3d delta = baseline_tf_.inverse() * candidate;
      const double translation_delta = delta.translation().head<2>().norm();
      const double rotation_delta = std::abs(
        std::remainder(std::atan2(delta.linear()(1, 0), delta.linear()(0, 0)), 2.0 * M_PI));
      if (
        translation_delta <= baseline_update_translation_threshold_ &&
        rotation_delta <= baseline_update_yaw_threshold_) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Holding map->odom baseline: micro correction dt=%.4f m, dr=%.4f rad", translation_delta,
          rotation_delta);
        return true;
      }
    }
    baseline_tf_ = initial_result_t_ = candidate;
  }
  result_t_ = baseline_tf_;
  return true;
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
