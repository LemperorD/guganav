// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_

#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
using PathRange = std::pair<PathIterator, PathIterator>;

/**
 * @class mppi::PathHandler
 * @brief 负责参考路径坐标变换、裁剪和方向反转处理
 */

class PathHandler
{
public:
  /**
    * @brief 构造路径处理器
    */
  PathHandler() = default;

  /**
    * @brief 析构路径处理器
    */
  ~PathHandler() = default;

  /**
    * @brief 启动时初始化路径处理器
    * @param parent: 控制器服务器生命周期节点的弱引用
    * @param name: 控制器插件名称
    * @param costmap_ros: 提供局部地图尺寸和坐标系的对象
    * @param tf: 用于路径坐标变换的 TF 缓冲区
    * @param dynamic_parameter_handler: 动态参数处理器
    */
  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>,
    std::shared_ptr<tf2_ros::Buffer>, ParametersHandler *);

  /**
    * @brief 设置新的全局参考路径
    * @param plan: 全局规划器输出的路径
    */
  void setPath(const nav_msgs::msg::Path & plan);

  /**
    * @brief 获取当前保存的全局参考路径
    * @return 返回值: 路径引用，供控制器检查、转换和可视化使用
    */
  nav_msgs::msg::Path & getPath();

  /**
   * @brief 将全局路径转换并裁剪到局部代价地图范围
   * @param robot_pose: 机器人当前位姿
   * @return 返回值: 局部坐标系下的有效路径，供 MPPI 轨迹评分与跟踪使用
   */
  nav_msgs::msg::Path transformPath(const geometry_msgs::msg::PoseStamped & robot_pose);

protected:
  /**
    * @brief 将位姿转换到指定坐标系
    * @param frame: 目标坐标系
    * @param in_pose: 输入位姿
    * @param out_pose: 接收转换结果的输出位姿
    * @return 返回值: 成功时为 true，调用方据此决定是否继续路径处理
    */
  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
    * @brief 获取局部代价地图中心到边缘的最大径向距离
    * @return 返回值: 路径截取半径，用于限制进入优化器的路径范围
    */
  double getMaxCostmapDist();

  /**
    * @brief 将位姿转换到全局路径坐标系
    * @param pose: 当前位姿
    * @return 返回值: 全局路径坐标系中的位姿，供最近路径点搜索使用
    */
  geometry_msgs::msg::PoseStamped
  transformToGlobalPlanFrame(const geometry_msgs::msg::PoseStamped & pose);

  /**
    * @brief 获取落在局部代价地图窗口内的全局路径段
    * @param global_pose: 全局路径坐标系中的机器人位姿
    * @return 返回值: 转换后的局部路径及首个全局路径迭代器，后者用于裁剪已走过的路径
    */
  std::pair<nav_msgs::msg::Path, PathIterator> getGlobalPlanConsideringBoundsInCostmapFrame(
    const geometry_msgs::msg::PoseStamped & global_pose);

  /**
    * @brief 从全局路径删除已通过的部分
    * @param plan: 待同步更新的路径消息
    * @param end: 要删除区间的末端迭代器
    */
  void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);

  /**
    * @brief 判断机器人是否已到达路径换向点容差范围
    * @param robot_pose: 机器人当前位姿
    * @return 返回值: 达到容差时为 true，路径处理器据此切换到换向点之后的路径段
    */
  bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);

  std::string name_;  ///< 控制器插件实例名称及参数命名空间。
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;  ///< 局部代价地图接口。
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  ///< 路径坐标变换缓冲区。
  ParametersHandler * parameters_handler_;  ///< 动态参数处理器的非持有指针。

  nav_msgs::msg::Path global_plan_;  ///< 当前完整全局参考路径。
  nav_msgs::msg::Path global_plan_up_to_inversion_;  ///< 截止到下一换向点的路径段。
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};  ///< 路径处理日志记录器。

  double max_robot_pose_search_dist_{0};  ///< 最近路径点的最大搜索距离。
  double prune_distance_{0};  ///< 保留机器人前方路径的裁剪距离。
  double transform_tolerance_{0};  ///< 坐标变换允许的时间容差。
  float inversion_xy_tolerance_{0.2};  ///< 到达换向点的位置容差。
  float inversion_yaw_tolerance{0.4};  ///< 到达换向点的航向容差。
  bool enforce_path_inversion_{false};  ///< 是否按路径尖点强制分段换向。
  unsigned int inversion_locale_{0u};  ///< 当前换向点在全局路径中的索引。
};
}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
