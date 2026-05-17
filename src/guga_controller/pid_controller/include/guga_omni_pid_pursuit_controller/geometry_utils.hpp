#include <math.h>
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
/**
   * @brief Calculates the intersection point of a circle and a line segment
   * @param p1 Start point of the line segment
   * @param p2 End point of the line segment
   * @param r Radius of the circle
   * @return Intersection point (geometry_msgs::msg::Point)
   */
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

/**
   * @brief Calculates the radius of curvature using three points
   * @param near_point Pose before the current point
   * @param current_point Current pose (lookahead pose)
   * @param far_point Pose after the current point
   * @return Radius of curvature
   */
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point);
