// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_

#include <memory>
#include <string>
#include <xtensor/xtensor.hpp>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

namespace mppi
{

/**
 * @class mppi::TrajectoryVisualizer
 * @brief 将候选轨迹、最优轨迹和参考路径发布为调试可视化消息
 */
class TrajectoryVisualizer
{
public:
  /**
    * @brief 构造轨迹可视化器
    */
  TrajectoryVisualizer() = default;

  /**
    * @brief 配置可视化发布器和抽样步长
    * @param parent: 控制器服务器生命周期节点的弱引用
    * @param name: 控制器插件名称
    * @param frame_id: 可视化消息使用的坐标系
    * @param parameters_handler: 动态参数处理器
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    const std::string & frame_id, ParametersHandler * parameters_handler);

  /**
    * @brief 关闭时清理可视化资源
    */
  void on_cleanup();

  /**
    * @brief 激活生命周期发布器
    */
  void on_activate();

  /**
    * @brief 停用生命周期发布器
    */
  void on_deactivate();

  /**
    * @brief 将最优轨迹转换为待发布标记
    * @param trajectory: 最优位姿轨迹张量
    * @param marker_namespace: 区分标记集合的命名空间
    */
  void add(const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace);

  /**
    * @brief 将候选轨迹按配置步长转换为待发布标记
    * @param trajectories: 批量候选轨迹
    * @param marker_namespace: 区分标记集合的命名空间
    */
  void add(const models::Trajectories & trajectories, const std::string & marker_namespace);

  /**
    * @brief 发布累计轨迹标记和局部参考路径
    * @param plan: 要同步发布的局部参考路径
    */
  void visualize(const nav_msgs::msg::Path & plan);

  /**
    * @brief 清空尚未发布的轨迹标记
    */
  void reset();

protected:
  std::string frame_id_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  trajectories_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_ = 0;

  ParametersHandler * parameters_handler_;

  size_t trajectory_step_{0};
  size_t time_step_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
