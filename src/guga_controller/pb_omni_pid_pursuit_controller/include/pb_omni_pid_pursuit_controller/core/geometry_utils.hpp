#include <math.h>
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
namespace geometry_utils
{
geometry_msgs::msg::Point circleSegmentIntersection(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

double calculateCurvatureRadius(const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point);

std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path);

geometry_msgs::msg::PoseStamped findPoseAtDistance(
  const nav_msgs::msg::Path & path,
  const std::vector<double> & cumulative_distances,
  double target_distance);
}

