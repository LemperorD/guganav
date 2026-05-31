#include "pb_omni_pid_pursuit_controller/core/geometry_utils.hpp"
#include "gtest/gtest.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

// 直线上三点的累积距离
TEST(GeometryUtilsTest, CumulativeDistances_StraightLine) {
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0;
  p.pose.position.y = 0;
  path.poses.push_back(p);
  p.pose.position.x = 1;
  path.poses.push_back(p);
  p.pose.position.x = 3;
  path.poses.push_back(p);

  auto distances = geometry_utils::calculateCumulativeDistances(path);

  ASSERT_EQ(distances.size(), 3u);
  EXPECT_DOUBLE_EQ(distances[0], 0.0);
  EXPECT_DOUBLE_EQ(distances[1], 1.0);
  EXPECT_DOUBLE_EQ(distances[2], 3.0);  // 1 + 2
}

// 空路径返回 [0]
TEST(GeometryUtilsTest, CumulativeDistances_EmptyPath) {
  nav_msgs::msg::Path path;
  auto distances = geometry_utils::calculateCumulativeDistances(path);
  ASSERT_EQ(distances.size(), 1u);
  EXPECT_DOUBLE_EQ(distances[0], 0.0);
}

// 目标距离落在中段：线性插值
TEST(GeometryUtilsTest, FindPoseAtDistance_MidSegment) {
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0;
  p.pose.position.y = 0;
  path.poses.push_back(p);
  p.pose.position.x = 1;
  path.poses.push_back(p);
  p.pose.position.x = 3;
  path.poses.push_back(p);

  auto dists = geometry_utils::calculateCumulativeDistances(path);
  auto result = geometry_utils::findPoseAtDistance(path, dists, 2.0);

  // target 2.0 is between cumulative[1]=1.0 and cumulative[2]=3.0
  // ratio = (2-1)/(3-1) = 0.5 → x = 1 + 0.5*(3-1) = 2
  EXPECT_DOUBLE_EQ(result.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(result.pose.position.y, 0.0);
}

// 负距离 → 返回第一个 pose
TEST(GeometryUtilsTest, FindPoseAtDistance_BeforeStart) {
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.pose.position.x = 5;
  path.poses.push_back(p1);
  p2.pose.position.x = 10;
  path.poses.push_back(p2);

  auto dists = geometry_utils::calculateCumulativeDistances(path);
  auto result = geometry_utils::findPoseAtDistance(path, dists, -1.0);

  EXPECT_DOUBLE_EQ(result.pose.position.x, 5.0);
}

// 超出路径末端 → 返回最后一个 pose
TEST(GeometryUtilsTest, FindPoseAtDistance_BeyondEnd) {
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.pose.position.x = 0;
  path.poses.push_back(p1);
  p2.pose.position.x = 10;
  path.poses.push_back(p2);

  auto dists = geometry_utils::calculateCumulativeDistances(path);
  auto result = geometry_utils::findPoseAtDistance(path, dists, 999.0);

  EXPECT_DOUBLE_EQ(result.pose.position.x, 10.0);
}

// 圆-线段交点：X 轴上 p1(0,0)→p2(2,0)，半径 1 → 交点在 (1,0)
TEST(GeometryUtilsTest, CircleSegmentIntersection_OnXAxis) {
  geometry_msgs::msg::Point p1, p2;
  p1.x = 0;
  p1.y = 0;
  p2.x = 2;
  p2.y = 0;

  auto result = geometry_utils::circleSegmentIntersection(p1, p2, 1.0);

  EXPECT_NEAR(std::hypot(result.x, result.y), 1.0, 1e-6);
  EXPECT_GT(result.x, 0.0) << "Should return point inside segment";
}

// 三点共线 → 曲率半径极大
TEST(GeometryUtilsTest, CurvatureRadius_StraightLine) {
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = 0;
  p1.y = 0;
  p2.x = 1;
  p2.y = 0;
  p3.x = 2;
  p3.y = 0;

  double radius = geometry_utils::calculateCurvatureRadius(p1, p2, p3);

  EXPECT_GE(radius, 1e8) << "Collinear points → near-infinite curvature radius";
}

// 三点在单位圆上 (0°, 90°, 180°) → 半径 ≈ 1.0
TEST(GeometryUtilsTest, CurvatureRadius_UnitCircle) {
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = 1;
  p1.y = 0;  // 0°
  p2.x = 0;
  p2.y = 1;  // 90°
  p3.x = -1;
  p3.y = 0;  // 180°

  double radius = geometry_utils::calculateCurvatureRadius(p1, p2, p3);

  EXPECT_NEAR(radius, 1.0, 0.01);
}
