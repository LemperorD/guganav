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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief 加载并调度轨迹评分 critic 插件的管理器
 */
class CriticManager
{
public:
  /**
    * @brief 构造 critic 管理器
    */
  CriticManager() = default;

  /**
    * @brief 析构 critic 管理器并释放插件实例
    */
  virtual ~CriticManager() = default;

  /**
    * @brief 配置管理器并加载参数指定的 critic 插件
    * @param parent: 控制器服务器生命周期节点的弱引用
    * @param name: 控制器插件名称
    * @param costmap_ros: 提供环境代价地图的对象
    * @param dynamic_parameter_handler: 动态参数处理器
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>, ParametersHandler *);

  /**
    * @brief 依次调用已加载 critics 对候选轨迹评分
    * @param data: 传递给各 critic 的评分上下文，成本将在其中累加
    */
  void evalTrajectoriesScores(CriticData & data) const;

protected:
  /**
    * @brief 读取待加载的 critic 名称列表
    */
  void getParams();

  /**
    * @brief 通过 pluginlib 实例化 critic 插件
    */
  virtual void loadCritics();

  /**
    * @brief 将配置名称展开为完整命名空间下的 critic ID
    * @param name: 配置中的 critic 简称或完整名称
    * @return 返回值: 完整 critic ID，供 pluginlib 创建对应插件实例
    */
  std::string getFullName(const std::string & name);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;  ///< 控制器服务器生命周期节点。
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  ///< 评分器共享的代价地图接口。
  std::string name_;  ///< 所属控制器的插件实例名称。

  ParametersHandler * parameters_handler_;  ///< 动态参数处理器的非持有指针。
  std::vector<std::string> critic_names_;  ///< 参数配置的评分器名称列表。
  std::unique_ptr<pluginlib::ClassLoader<critics::CriticFunction>> loader_;  ///< 评分插件加载器。
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;  ///< 已实例化的评分器列表。

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};  ///< 管理器日志记录器。
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
