#pragma once

#include <memory>
#include <optional>
#include <string>
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mpc_controller
{

/**
 * @brief Nav2 Path 的坐标变换和预处理。
 *
 * - 将全局路径变换到车体系 / local costmap 帧
 * - 裁剪到 costmap 范围内
 * - 计算速度自适应前瞻距离
 */
class PathHandler
{
public:
  PathHandler() = default;

  /** @brief 设置 TF buffer 和 costmap 引用。 */
  void configure(
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    double transform_tolerance);

  /**
   * @brief 将全局路径变换为局部路径。
   * @param robot_pose 当前机器人位姿（全局帧）
   * @param global_plan 全局规划路径
   * @return 变换到机器人基帧的局部路径，若无则返回 std::nullopt
   */
  [[nodiscard]] std::optional<nav_msgs::msg::Path> transformPath(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & global_plan);

  /** @brief 速度自适应前瞻距离。 */
  [[nodiscard]] double computeLookahead(double current_speed) const;

  /** @brief 获取当前局部路径。 */
  [[nodiscard]] const nav_msgs::msg::Path & localPath() const { return local_path_; }

  /** @brief 设置前瞻参数。 */
  void setLookaheadParams(double min_dist, double velocity_gain, double max_path_len);

private:
  /** @brief 将单个位姿变换到目标帧。 */
  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> transformPose(
    const geometry_msgs::msg::PoseStamped & in_pose,
    const std::string & target_frame);

  /** @brief costmap 最大范围的一半。 */
  [[nodiscard]] double costmapMaxExtent() const;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  double transform_tolerance_{0.5};

  double lookahead_min_{0.2};
  double lookahead_velocity_gain_{1.0};
  double path_max_length_{5.0};

  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};

  nav_msgs::msg::Path local_path_;
};

}  // namespace mpc_controller
