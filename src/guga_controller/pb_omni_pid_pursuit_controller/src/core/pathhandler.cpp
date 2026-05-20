#include "pb_omni_pid_pursuit_controller/core/pathhandler.hpp"
using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub)
    : tf_(std::move(tf)),
      costmap_ros_(std::move(costmap_ros)),
      local_path_pub_(std::move(pub)),
      transform_tolerance_(tf2::durationFromSec(1.0)),
      max_robot_pose_search_dist_(getCostmapMaxExtent()) {
}

double PathHandler::getCostmapMaxExtent() const {
  const double max_costmap_dim_meters = std::max(
      costmap_ros_->getCostmap()->getSizeInMetersX(),
      costmap_ros_->getCostmap()->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

std::optional<geometry_msgs::msg::PoseStamped> PathHandler::transformPose(
    const std::string& frame,
    const geometry_msgs::msg::PoseStamped& in_pose) const {
  if (in_pose.header.frame_id == frame) {
    return in_pose;
  }

  try {
    geometry_msgs::msg::PoseStamped out_pose;
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return out_pose;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return std::nullopt;
}

nav_msgs::msg::Path PathHandler::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped& pose,
    nav_msgs::msg::Path& global_plan) {
  if (global_plan.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  auto robot_pose =
      transformPose(global_plan.header.frame_id, pose);
  if (!robot_pose) {
    throw nav2_core::PlannerException(
        "Unable to transform robot pose into global plan's frame");
  }

  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound =
      nav2_util::geometry_utils::first_after_integrated_distance(
          global_plan.poses.begin(), global_plan.poses.end(),
          max_robot_pose_search_dist_);

  auto transformation_begin = nav2_util::geometry_utils::min_by(
      global_plan.poses.begin(), closest_pose_upper_bound,
      [&robot_pose](const geometry_msgs::msg::PoseStamped& ps) {
        return euclidean_distance(*robot_pose, ps);
      });

  auto transformation_end = std::find_if(
      transformation_begin, global_plan.poses.end(), [&](const auto& pose) {
        return euclidean_distance(pose, *robot_pose) > max_costmap_extent;
      });

  auto transform_global_pose_to_local = [&](const auto& global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = global_plan.header.frame_id;
    stamped_pose.header.stamp = robot_pose->header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    auto transformed_pose =
        transformPose(costmap_ros_->getBaseFrameID(), stamped_pose);
    if (transformed_pose) {
      transformed_pose->pose.position.z = 0.0;
      return *transformed_pose;
    }
    geometry_msgs::msg::PoseStamped empty;
    empty.pose.position.z = 0.0;
    return empty;
  };

  nav_msgs::msg::Path transformed_plan;
  std::transform(transformation_begin, transformation_end,
                 std::back_inserter(transformed_plan.poses),
                 transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose->header.stamp;

  global_plan.poses.erase(begin(global_plan.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}
