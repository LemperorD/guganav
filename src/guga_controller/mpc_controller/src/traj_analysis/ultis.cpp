#include "mpc_controller/core/path_handler.hpp"

#include <algorithm>
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace mpc_controller
{

// ==========================================================================
void PathHandler::configure(
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  double transform_tolerance)
{
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  transform_tolerance_ = transform_tolerance;

  if (costmap_ros_) {
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
  }
}

// ==========================================================================
void PathHandler::setLookaheadParams(
  double min_dist, double velocity_gain, double max_path_len)
{
  lookahead_min_ = min_dist;
  lookahead_velocity_gain_ = velocity_gain;
  path_max_length_ = max_path_len;
}

// ==========================================================================
double PathHandler::computeLookahead(double current_speed) const
{
  // d = d_min + k_v * |v|
  return lookahead_min_ + lookahead_velocity_gain_ * std::abs(current_speed);
}

// ==========================================================================
double PathHandler::costmapMaxExtent() const
{
  if (!costmap_) { return path_max_length_; }
  const double sx = costmap_->getSizeInMetersX();
  const double sy = costmap_->getSizeInMetersY();
  return std::max(sx, sy) / 2.0;
}

// ==========================================================================
std::optional<geometry_msgs::msg::PoseStamped> PathHandler::transformPose(
  const geometry_msgs::msg::PoseStamped & in_pose,
  const std::string & target_frame)
{
  if (!tf_) { return std::nullopt; }
  try {
    geometry_msgs::msg::PoseStamped out;
    const auto timeout = tf2::durationFromSec(transform_tolerance_);
    tf_->transform(in_pose, out, target_frame, timeout);
    return out;
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

// ==========================================================================
std::optional<nav_msgs::msg::Path> PathHandler::transformPath(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const nav_msgs::msg::Path & global_plan)
{
  if (global_plan.poses.empty()) { return std::nullopt; }
  if (!tf_ || !costmap_ros_) { return std::nullopt; }

  // 1. 变换 robot pose 到全局帧
  auto robot_in_global = transformPose(robot_pose, global_frame_);
  if (!robot_in_global.has_value()) { return std::nullopt; }

  // 2. 找到路径上最近点
  const auto dist_fn = [](const geometry_msgs::msg::PoseStamped & a,
                           const geometry_msgs::msg::PoseStamped & b)
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  auto it = nav2_util::geometry_utils::min_by(
    global_plan.poses.begin(), global_plan.poses.end(),
    [&](const auto & wp) { return dist_fn(wp, *robot_in_global); });

  if (it == global_plan.poses.end()) { return std::nullopt; }

  // 3. 裁剪到 costmap 范围
  const double max_extent = costmapMaxExtent();
  local_path_.poses.clear();
  local_path_.header.frame_id = robot_base_frame_;
  local_path_.header.stamp = robot_pose.header.stamp;

  double accumulated = 0.0;
  for (auto wp = it; wp != global_plan.poses.end(); ++wp) {
    // 变换到 base_link 帧
    geometry_msgs::msg::PoseStamped in_global;
    in_global.header = global_plan.header;
    in_global.pose = wp->pose;

    auto in_base = transformPose(in_global, robot_base_frame_);
    if (!in_base.has_value()) { break; }

    // 检查是否在 costmap 范围内
    const double d = std::hypot(in_base->pose.position.x,
                                in_base->pose.position.y);
    if (d > max_extent) { break; }

    local_path_.poses.push_back(*in_base);

    // 累计距离
    if (wp != it) {
      const auto & prev = *(wp - 1);
      const double dd = dist_fn(*wp, prev);
      accumulated += dd;
      if (accumulated > path_max_length_) { break; }
    }
  }

  if (local_path_.poses.empty()) { return std::nullopt; }
  return local_path_;
}

}  // namespace mpc_controller
