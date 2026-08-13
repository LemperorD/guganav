// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "angles/angles.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/goal_checker.hpp"

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

namespace mppi::utils
{
using xt::evaluation_strategy::immediate;

/**
 * @brief 将位置坐标转换为单位姿态四元数的 Pose
 * @param x: X 坐标
 * @param y: Y 坐标
 * @param z: Z 坐标
 * @return 返回值: 构造后的位姿，供轨迹可视化标记使用
 */
inline geometry_msgs::msg::Pose createPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  return pose;
}

/**
 * @brief 构造三维缩放向量
 * @param x: X 方向尺寸
 * @param y: Y 方向尺寸
 * @param z: Z 方向尺寸
 * @return 返回值: 缩放向量，供可视化标记设置尺寸
 */
inline geometry_msgs::msg::Vector3 createScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

/**
 * @brief 构造 RGBA 颜色消息
 * @param r: 红色分量
 * @param g: 绿色分量
 * @param b: 蓝色分量
 * @param a: 透明度分量
 * @return 返回值: 颜色消息，供可视化标记设置颜色
 */
inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

/**
 * @brief 将位姿、尺寸和颜色组合为球形可视化标记
 * @param id: 标记 ID
 * @param pose: 标记位姿
 * @param scale: 标记尺寸
 * @param color: 标记颜色
 * @param frame_id: 标记坐标系
 * @param ns: 标记命名空间
 * @return 返回值: 可视化标记，供 TrajectoryVisualizer 汇总发布
 */
inline visualization_msgs::msg::Marker createMarker(
  int id, const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const std::string & frame_id, const std::string & ns)
{
  using visualization_msgs::msg::Marker;
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0, 0);
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;

  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

/**
 * @brief 将差速控制量转换为带时间戳的速度消息
 * @param vx: 纵向线速度
 * @param wz: Z 轴角速度
 * @param stamp: 消息时间戳
 * @param frame: 速度参考坐标系
 * @return 返回值: 差速底盘速度消息，供控制器输出链执行
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float wz, const builtin_interfaces::msg::Time & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  twist.twist.angular.z = wz;

  return twist;
}

/**
 * @brief 将全向控制量转换为带时间戳的速度消息
 * @param vx: 纵向线速度
 * @param vy: 横向线速度
 * @param wz: Z 轴角速度
 * @param stamp: 消息时间戳
 * @param frame: 速度参考坐标系
 * @return 返回值: 全向底盘速度消息，供控制器输出链执行
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float vy, float wz, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  auto twist = toTwistStamped(vx, wz, stamp, frame);
  twist.twist.linear.y = vy;

  return twist;
}

/**
 * @brief 将 ROS 路径消息转换为优化器路径张量
 * @param path: 待转换的路径消息
 * @return 返回值: `x/y/yaw` 路径张量，供 critics 批量评分
 */
inline models::Path toTensor(const nav_msgs::msg::Path & path)
{
  auto result = models::Path{};
  result.reset(path.poses.size());

  for (size_t i = 0; i < path.poses.size(); ++i) {
    result.x(i) = path.poses[i].pose.position.x;
    result.y(i) = path.poses[i].pose.position.y;
    result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);
  }

  return result;
}

/**
 * @brief 按 GoalChecker 容差判断机器人是否接近路径终点
 * @param goal_checker: 目标检查器
 * @param robot: 机器人当前位姿
 * @param path: 提供终点位置的路径张量
 * @return 返回值: 位于目标位置容差内时为 true，critics 据此切换近目标评分逻辑
 */
inline bool withinPositionGoalTolerance(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

    const auto pose_tolerance_sq = pose_tolerance.position.x * pose_tolerance.position.x;

    auto dx = robot.position.x - goal_x;
    auto dy = robot.position.y - goal_y;

    auto dist_sq = dx * dx + dy * dy;

    if (dist_sq < pose_tolerance_sq) {
      return true;
    }
  }

  return false;
}

/**
 * @brief 按给定距离容差判断机器人是否接近路径终点
 * @param pose_tolerance: 位置容差
 * @param robot: 机器人当前位姿
 * @param path: 提供终点位置的路径张量
 * @return 返回值: 位于目标位置容差内时为 true，critics 据此停用远距离评分项
 */
inline bool withinPositionGoalTolerance(
  float pose_tolerance,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  const auto pose_tolerance_sq = pose_tolerance * pose_tolerance;

  auto dx = robot.position.x - goal_x;
  auto dy = robot.position.y - goal_y;

  auto dist_sq = dx * dx + dy * dy;

  if (dist_sq < pose_tolerance_sq) {
    return true;
  }

  return false;
}

/**
  * @brief 将弧度角归一化到 `[-pi, pi]`
  * @param angles: 待归一化的标量或张量角度
  * @return 返回值: 归一化角度，供航向误差和轨迹积分计算使用
  */
template<typename T>
auto normalize_angles(const T & angles)
{
  auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
  return xt::eval(xt::where(theta <= 0.0, theta + M_PI, theta - M_PI));
}

/**
  * @brief 计算两个弧度角之间的最短有符号角距离
  * @param from: 起始角度
  * @param to: 目标角度
  * @return 返回值: `[-pi, pi]` 范围内的角差，供航向 critic 计算误差
  */
template<typename F, typename T>
auto shortest_angular_distance(
  const F & from,
  const T & to)
{
  return normalize_angles(to - from);
}

/**
 * @brief 计算候选轨迹集合到达的最远参考路径点索引
 * @param data: 包含候选轨迹和参考路径的评分上下文
 * @return 返回值: 最远路径点索引，供路径跟随和对齐 critics 选择前视目标
 */
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());

  const auto dx = data.path.x - traj_x;
  const auto dy = data.path.y - traj_y;

  const auto dists = dx * dx + dy * dy;

  size_t max_id_by_trajectories = 0, min_id_by_path = 0;
  float min_distance_by_path = std::numeric_limits<float>::max();
  float cur_dist = 0.0f;

  for (size_t i = 0; i < dists.shape(0); i++) {
    min_id_by_path = 0;
    min_distance_by_path = std::numeric_limits<float>::max();
    for (size_t j = 0; j < dists.shape(1); j++) {
      cur_dist = dists(i, j);
      if (cur_dist < min_distance_by_path) {
        min_distance_by_path = cur_dist;
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }
  return max_id_by_trajectories;
}

/**
 * @brief 计算候选轨迹起点最近的参考路径点索引
 * @param data: 包含候选轨迹和参考路径的评分上下文
 * @return 返回值: 最近路径点索引，供路径对齐 critic 限定搜索起点
 */
inline size_t findPathTrajectoryInitialPoint(const CriticData & data)
{
  // 所有候选轨迹从同一初始状态出发，因此只需使用第一条轨迹的首点。
  const auto dx = data.path.x - data.trajectories.x(0, 0);
  const auto dy = data.path.y - data.trajectories.y(0, 0);
  const auto dists = dx * dx + dy * dy;

  float min_distance_by_path = std::numeric_limits<float>::max();
  size_t min_id = 0;
  for (size_t j = 0; j < dists.shape(0); j++) {
    if (dists(j) < min_distance_by_path) {
      min_distance_by_path = dists(j);
      min_id = j;
    }
  }

  return min_id;
}

/**
 * @brief 在尚未缓存时计算最远到达路径点
 * @param data: 接收缓存索引的评分上下文
 */
inline void setPathFurthestPointIfNotSet(CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
  }
}

/**
 * @brief 根据代价地图计算参考路径点有效性
 * @param data: 接收路径有效性缓存的评分上下文
 * @param costmap_ros: 提供路径点代价值和未知空间策略的对象
 */
inline void findPathCosts(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto * costmap = costmap_ros->getCostmap();
  unsigned int map_x, map_y;
  const size_t path_segments_count = data.path.x.shape(0) - 1;
  data.path_pts_valid = std::vector<bool>(path_segments_count, false);
  for (unsigned int idx = 0; idx < path_segments_count; idx++) {
    const auto path_x = data.path.x(idx);
    const auto path_y = data.path.y(idx);
    if (!costmap->worldToMap(path_x, path_y, map_x, map_y)) {
      (*data.path_pts_valid)[idx] = false;
      continue;
    }

    switch (costmap->getCost(map_x, map_y)) {
      using namespace nav2_costmap_2d; // NOLINT
      case (LETHAL_OBSTACLE):
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (INSCRIBED_INFLATED_OBSTACLE):
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (NO_INFORMATION):
        const bool is_tracking_unknown =
          costmap_ros->getLayeredCostmap()->isTrackingUnknown();
        (*data.path_pts_valid)[idx] = is_tracking_unknown ? true : false;
        continue;
    }

    (*data.path_pts_valid)[idx] = true;
  }
}

/**
 * @brief 在尚未缓存时计算参考路径点有效性
 * @param data: 接收路径有效性缓存的评分上下文
 * @param costmap_ros: 提供路径点代价值的对象
 */
inline void setPathCostsIfNotSet(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (!data.path_pts_valid) {
    findPathCosts(data, costmap_ros);
  }
}

/**
 * @brief 计算带航向位姿指向目标点的角误差
 * @param pose: 起始位姿
 * @param point_x: 目标点 X 坐标
 * @param point_y: 目标点 Y 坐标
 * @param forward_preference: 是否只考虑正向朝向
 * @return 返回值: 最小绝对角误差，供 PathAngleCritic 评价轨迹朝向
 */
inline float posePointAngle(
  const geometry_msgs::msg::Pose & pose, double point_x, double point_y, bool forward_preference)
{
  float pose_x = pose.position.x;
  float pose_y = pose.position.y;
  float pose_yaw = tf2::getYaw(pose.orientation);

  float yaw = atan2f(point_y - pose_y, point_x - pose_x);

  // 允许倒车时，在正向航向与反向航向中选择更小的角误差。
  if (!forward_preference) {
    return std::min(
      fabs(angles::shortest_angular_distance(yaw, pose_yaw)),
      fabs(angles::shortest_angular_distance(yaw, angles::normalize_angle(pose_yaw + M_PI))));
  }

  return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief 对最优控制序列应用 Savitzky-Golay 平滑滤波
 * @param control_sequence: 待就地平滑的控制序列
 * @param control_history: 用于处理序列前端边界的历史控制量
 * @param settings: 用于选择实际输出偏移的优化器设置
 */
inline void savitskyGolayFilter(
  models::ControlSequence & control_sequence,
  std::array<mppi::models::Control, 4> & control_history,
  const models::OptimizerSettings & settings)
{
  // 二次九点 Savitzky-Golay 滤波系数。
  xt::xtensor<float, 1> filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
  filter /= 231.0;

  const unsigned int num_sequences = control_sequence.vx.shape(0) - 1;

  // 序列过短时滤波没有稳定意义。
  if (num_sequences < 20) {
    return;
  }

  auto applyFilter = [&](const xt::xtensor<float, 1> & data) -> float {
      return xt::sum(data * filter, {0}, immediate)();
    };

  auto applyFilterOverAxis =
    [&](xt::xtensor<float, 1> & sequence,
      const float hist_0, const float hist_1, const float hist_2, const float hist_3) -> void
    {
      unsigned int idx = 0;
      sequence(idx) = applyFilter(
      {
        hist_0,
        hist_1,
        hist_2,
        hist_3,
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_1,
        hist_2,
        hist_3,
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_2,
        hist_3,
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_3,
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      for (idx = 4; idx != num_sequences - 4; idx++) {
        sequence(idx) = applyFilter(
        {
          sequence(idx - 4),
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 4)});
      }

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 3)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 2),
        sequence(idx + 2)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 1),
        sequence(idx + 1),
        sequence(idx + 1)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx),
        sequence(idx),
        sequence(idx),
        sequence(idx)});
    };

  // 分别平滑三个控制轴。
  applyFilterOverAxis(
    control_sequence.vx, control_history[0].vx,
    control_history[1].vx, control_history[2].vx, control_history[3].vx);
  applyFilterOverAxis(
    control_sequence.vy, control_history[0].vy,
    control_history[1].vy, control_history[2].vy, control_history[3].vy);
  applyFilterOverAxis(
    control_sequence.wz, control_history[0].wz,
    control_history[1].wz, control_history[2].wz, control_history[3].wz);

  // 更新历史控制量，为下一周期的序列前端滤波提供边界数据。
  unsigned int offset = settings.shift_control_sequence ? 1 : 0;
  control_history[0] = control_history[1];
  control_history[1] = control_history[2];
  control_history[2] = control_history[3];
  control_history[3] = {
    control_sequence.vx(offset),
    control_sequence.vy(offset),
    control_sequence.wz(offset)};
}

/**
 * @brief 查找路径首次方向反转后的第一个点
 * @param path: 待检查的路径
 * @return 返回值: 换向后首点索引；没有换向时返回路径长度，供路径分段使用
 */
inline unsigned int findFirstPathInversion(nav_msgs::msg::Path & path)
{
  // 至少需要三个点才能判断相邻线段是否反向。
  if (path.poses.size() < 3) {
    return path.poses.size();
  }

  // 逐点检查相邻路径线段的方向关系。
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // 构造换向点前后的 OA、AB 两个路径向量。
    float oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    float oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    float ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    float ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // 点积为负表示两段方向相反，即路径出现尖点换向。
    float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      return idx + 1;
    }
  }

  return path.poses.size();
}

/**
 * @brief 删除首次换向点之后的路径
 * @param path: 待就地截断的路径
 * @return 返回值: 换向位置索引；无换向时返回 0，供路径处理器记录后续切换位置
 */
inline unsigned int removePosesAfterFirstInversion(nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path cropped_path = path;
  const unsigned int first_after_inversion = findFirstPathInversion(cropped_path);
  if (first_after_inversion == path.poses.size()) {
    return 0u;
  }

  cropped_path.poses.erase(
    cropped_path.poses.begin() + first_after_inversion, cropped_path.poses.end());
  path = cropped_path;
  return first_after_inversion;
}

/**
 * @brief 在累计路径距离数组中查找最接近目标距离的路径点
 * @param vec: 单调递增的累计距离数组
 * @param dist: 目标累计距离
 * @param init: 搜索起始索引
 * @return 返回值: 最近路径点索引，供 critics 按前视距离选择评分点
 */
inline size_t findClosestPathPt(const std::vector<float> & vec, float dist, size_t init = 0)
{
  auto iter = std::lower_bound(vec.begin() + init, vec.end(), dist);
  if (iter == vec.begin() + init) {
    return 0;
  }
  if (dist - *(iter - 1) < *iter - dist) {
    return iter - 1 - vec.begin();
  }
  return iter - vec.begin();
}

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
