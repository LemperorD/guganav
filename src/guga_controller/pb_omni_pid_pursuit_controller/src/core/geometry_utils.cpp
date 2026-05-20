#include "pb_omni_pid_pursuit_controller/core/geometry_utils.hpp"
geometry_msgs::msg::Point geometry_utils::circleSegmentIntersection(
    const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2,
    double r) {
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two
  // points. https://mathworld.wolfram.com/Circle-LineIntersection.html This
  // works because the poses are transformed into the robot frame. This can be
  // derived from solving the system of equations of a line and a circle which
  // results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well
  // as at https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = (dx * dx) + (dy * dy);
  double d = (x1 * y2) - (x2 * y1);

  // Augmentation to only return point within segment
  double d1 = (x1 * x1) + (y1 * y1);
  double d2 = (x2 * x2) + (y2 * y2);
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - d * d);
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

double geometry_utils::calculateCurvatureRadius(
    const geometry_msgs::msg::Point& near_point,
    const geometry_msgs::msg::Point& current_point,
    const geometry_msgs::msg::Point& far_point) {
  double x1 = near_point.x;
  double y1 = near_point.y;
  double x2 = current_point.x;
  double y2 = current_point.y;
  double x3 = far_point.x;
  double y3 = far_point.y;

  double center_x =
      ((((x1 * x1) + (y1 * y1)) * (y2 - y3))
       + (((x2 * x2) + (y2 * y2)) * (y3 - y1))
       + (((x3 * x3) + (y3 * y3)) * (y1 - y2)))
      / (2 * ((x1 * (y2 - y3)) + (x2 * (y3 - y1)) + (x3 * (y1 - y2))));
  double center_y =
      ((((x1 * x1) + (y1 * y1)) * (x3 - x2))
       + (((x2 * x2) + (y2 * y2)) * (x1 - x3))
       + (((x3 * x3) + (y3 * y3)) * (x2 - x1)))
      / (2 * ((x1 * (y2 - y3)) + (x2 * (y3 - y1)) + (x3 * (y1 - y2))));
  double radius = std::hypot(x2 - center_x, y2 - center_y);
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

std::vector<double> geometry_utils::calculateCumulativeDistances(
    const nav_msgs::msg::Path& path) {
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto& prev_pose = path.poses[i - 1].pose.position;
    const auto& curr_pose = path.poses[i].pose.position;
    double distance = hypot(curr_pose.x - prev_pose.x,
                            curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

geometry_msgs::msg::PoseStamped geometry_utils::findPoseAtDistance(
    const nav_msgs::msg::Path& path,
    const std::vector<double>& cumulative_distances, double target_distance) {
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }
  auto it = std::lower_bound(cumulative_distances.begin(),
                             cumulative_distances.end(), target_distance);
  size_t index = std::distance(cumulative_distances.begin(), it);

  if (index == 0) {
    return path.poses.front();
  }

  double ratio = (target_distance - cumulative_distances[index - 1])
               / (cumulative_distances[index]
                  - cumulative_distances[index - 1]);
  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
      pose1.pose.position.x
      + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
      pose1.pose.position.y
      + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
      pose1.pose.position.z
      + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}