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

#include <chrono>
#include <cmath>
#include <lifecycle_msgs/msg/state.hpp>
#include <limits>

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{
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

// NOTE: 发布路径改成"只发基准 tf"之后不再需要每帧重写位姿;
// 而且这个函数把 eulerAngles(0,1,2)(= R = Rx*Ry*Rz)的角拿去按 Rz*Ry*Rx 重组,
// 只要 roll/pitch 非零, 每次调用都会把旋转改坏一点。要重新启用必须先统一两侧约定。
void SmallGicpRelocalizationNode::regulateRegistration(Eigen::Isometry3d & T)
{
  // 保留当前旋转中的 yaw
  const double yaw = std::atan2(T.linear()(1, 0), T.linear()(0, 0));

  // z 位置置零
  T.translation().z() = 0.0;

  // 只保留 yaw，roll = pitch = 0
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

bool SmallGicpRelocalizationNode::isOutOfMap(
  const Eigen::Isometry3d & T_map_odom, Eigen::Vector3d & robot_in_map) const
{
  if (!has_map_bounds_) {
    return false;  // 地图范围还没算出来, 不判
  }
  Eigen::Isometry3d T_odom_base;
  try {
    T_odom_base = tf2::transformToEigen(
      tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero).transform);
  } catch (const tf2::TransformException &) {
    return false;  // 拿不到 odom->base 就不猜
  }
  // T_map_odom 是 map->odom, 它的 translation 是 odom 原点落在地图的哪里, 不是机器人的
  // 位置; 机器人在地图里的位置要再乘上 odom->base。只判水平方向, z 由 tz 限位管。
  robot_in_map = (T_map_odom * T_odom_base).translation();
  for (int axis = 0; axis < 2; ++axis) {
    if (
      robot_in_map[axis] < map_min_bound_[axis] - map_boundary_margin_ ||
      robot_in_map[axis] > map_max_bound_[axis] + map_boundary_margin_) {
      return true;
    }
  }
  return false;
}

void SmallGicpRelocalizationNode::publishTransform()
{
  // active + FINE 时发布可信基准；inactive（包括 LOST 后停用）以及其余状态
  // 发布 result_t_。停用期间仍需维持 map -> odom，供 Nav2 继续使用 TF 树。
  // 不回写 result_t_ / previous_result_t_ —— previous_result_t_ 是 2Hz 注册的迭代种子,
  // 让 20Hz 的发布循环去改它会和注册互相抢状态。
  const bool lifecycle_active =
    this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  Eigen::Isometry3d to_publish =
    lifecycle_active && global_state_ == State::FINE ? baseline_tf_ : result_t_;

  regulateRegistration(to_publish);
  if (to_publish.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // inactive 后不再接收 registered_scan，必须用当前时间刷新时间戳，
  // 否则 TF cache 中的最后一条 map -> odom 会过期。
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = to_publish.translation();
  const Eigen::Quaterniond rotation(to_publish.rotation());

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

    if (global_state_ == State::SUSPECT || global_state_ == State::LOST) {
      // 还没有基准, 说明自动重定位没能初始化出可信的 tf。实车手动定位时这段位姿
      // 就是唯一可靠来源, 所以直接采信操作员给的 tf 作为基准, 不经过质量门, 也不等注册。
      // 之后正常注册出合格结果时会自动把它升级掉。
      baseline_tf_ = initial_result_t_ = result_t_ = previous_result_t_ = map_to_odom;
      RCLCPP_WARN(
        this->get_logger(),
        "No baseline yet; accepting the manual pose as the baseline tf (manual localization, "
        "bypassing the quality gate)");
      return;
    }

    // 人工给定的初始位姿只作为下一次注册的种子, 由基准残差检查决定是否升级为基准 tf。
    // 这里绝不能写进 result_t_: publishTransform 以 20Hz 广播 result_t_,
    // 直接赋值会把这段没经过任何配准检查的位姿当成 map->odom 发出去,
    // 且当本轮 GICP 不收敛(performRegistration 提前 return)时它不会被改回去,
    // 于是"给了错误 2D pose"永远不会触发回退。
    previous_result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

void SmallGicpRelocalizationNode::checkStateTime()
{
  const rclcpp::Time now = this->now();
  if (global_state_ == State::INITIAL) {
    if (!initial_timer_started_) {
      initial_start_time_ = now;
      initial_timer_started_ = true;
    } else if ((now - initial_start_time_).seconds() > initial_state_timeout_sec_) {
      global_state_ = State::SUSPECT;
      initial_timer_started_ = false;
      suspect_timer_started_ = false;
    }
  } else {
    initial_timer_started_ = false;
  }

  if (global_state_ == State::SUSPECT) {
    if (!suspect_timer_started_) {
      suspect_start_time_ = now;
      suspect_timer_started_ = true;
    } else if ((now - suspect_start_time_).seconds() > suspect_state_timeout_sec_) {
      global_state_ = State::LOST;
      suspect_timer_started_ = false;
      RCLCPP_WARN(this->get_logger(), "GICP relocalization LOST");
    }
  } else {
    suspect_timer_started_ = false;
  }

  if (
    global_state_ == State::LOST &&
    this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    std_msgs::msg::Bool lost_msg;
    lost_msg.data = true;
    lost_pub_->publish(lost_msg);
    publishState();

    RCLCPP_WARN(this->get_logger(), "定位状态为 LOST,正在停用节点");

    const auto inactive_state = this->deactivate();

    if (inactive_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_ERROR(
        this->get_logger(), "节点未进入 Inactive 状态，当前状态：%s",
        inactive_state.label().c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "节点已进入 Inactive 状态，开始清理先验地图");
    const auto unconfigured_state = this->cleanup();
    if (unconfigured_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      RCLCPP_ERROR(
        this->get_logger(), "节点未进入 Unconfigured 状态，当前状态：%s",
        unconfigured_state.label().c_str());
    }
    return;
  }

  std_msgs::msg::Bool lost_msg;
  lost_msg.data = global_state_ == State::LOST;
  lost_pub_->publish(lost_msg);
  publishState();
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
