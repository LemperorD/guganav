#ifndef NAV2_WRAPPER_HPP_
#define NAV2_WRAPPER_HPP_

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <memory>
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <vector>
#include <iterator>
#include <optional>

namespace mpc_controller {

struct NavWrapperConfig
{
  bool use_interpolation;
  bool use_curvature_scaling;
  double curvature_min;
  double curvature_max;
  double reduction_ratio_at_high_curvature;
  double curvature_forward_dist;
  double curvature_backward_dist;
  double max_velocity_scaling_factor_rate;
} typedef NavConfig;

class NavWrapper
{
public:
  explicit NavWrapper(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub,
    double transform_tolerance = 1.0);

  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose, nav_msgs::msg::Path& global_plan);

  [[nodiscard]] double getCostmapMaxExtent() const;

  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> transformPose(
    const std::string& frame,
    const geometry_msgs::msg::PoseStamped& in_pose) const;

  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2,
    double r);

  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point& near_point,
    const geometry_msgs::msg::Point& current_point,
    const geometry_msgs::msg::Point& far_point);

  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path& path);

  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path& path, const std::vector<double>& cumulative_distances,
    double target_distance) const;

  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);

  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;
  
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point,
    const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;
  
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;

  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose, const geometry_msgs::msg::PoseStamped & forward_pose) const;

  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

public: // setters and getters for configuration
  void setUseInterpolation(bool use_interpolation) { use_interpolation_ = use_interpolation; }
  bool use_interpolation() const { return use_interpolation_; }

  void setUseCurvatureScaling(bool use_curvature_scaling) { use_curvature_scaling_ = use_curvature_scaling; }
  bool use_curvature_scaling() const { return use_curvature_scaling_; }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  tf2::Duration transform_tolerance_;
  rclcpp::Logger logger_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  double max_robot_pose_search_dist_;

  bool use_interpolation_;
  bool use_curvature_scaling_;
  bool use_velocity_scaled_lookahead_dist_;
  double curvature_min_;
  double curvature_max_;
  double reduction_ratio_at_high_curvature_;
  double curvature_forward_dist_;
  double curvature_backward_dist_;
  double min_approach_linear_velocity_;
  double max_velocity_scaling_factor_rate_;
  double control_duration_;
  double lookahead_dist_;
  double lookahead_time_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;

  double last_velocity_scaling_factor_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    curvature_points_pub_;
};

} // namespace mpc_controller

#endif // NAV2_WRAPPER_HPP_