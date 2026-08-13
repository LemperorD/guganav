// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey
// Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <string>
#include <memory>

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_mppi_controller {

  using namespace mppi;  // NOLINT

  /**
   * @class mppi::MPPIController
   * @brief Nav2 MPPI 控制器主插件，负责连接路径处理、优化器与速度指令输出
   */
  class MPPIController : public nav2_core::Controller {
  public:
    /**
     * @brief 构造 MPPI 控制器
     */
    MPPIController() = default;

    /**
     * @brief 配置控制器并初始化依赖组件
     * @param parent: 控制器服务器生命周期节点的弱引用
     * @param name: 插件实例名称，用作参数命名空间
     * @param tf: 用于坐标变换查询的 TF 缓冲区
     * @param costmap_ros: 提供环境代价地图和机器人坐标系信息的对象
     */
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                   std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                   const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
                       costmap_ros) override;

    /**
     * @brief 清理控制器持有的运行资源
     */
    void cleanup() override;

    /**
     * @brief 激活控制器及其生命周期发布器
     */
    void activate() override;

    /**
     * @brief 停用控制器及其生命周期发布器
     */
    void deactivate() override;

    /**
     * @brief 在导航任务之间重置控制器内部状态
     */
    void reset();

    /**
     * @brief 使用 MPPI 优化器计算本周期速度指令
     * @param robot_pose: 机器人当前位姿
     * @param robot_speed: 机器人当前线速度与角速度
     * @param goal_checker: 目标检查器，用于判断是否接近或到达目标
     * @return 返回值: 带时间戳和坐标系的速度指令，由 controller_server
     * 发送给后续速度平滑与底盘链路
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& robot_pose,
        const geometry_msgs::msg::Twist& robot_speed,
        nav2_core::GoalChecker* goal_checker) override;

    /**
     * @brief 设置新的待跟踪参考路径
     * @param path: 全局规划器输出的参考路径
     */
    void setPlan(const nav_msgs::msg::Path& path) override;

    /**
     * @brief 根据速度限制回调更新控制器约束
     * @param speed_limit: 绝对速度上限或相对百分比
     * @param percentage: 为 true 时按百分比解释速度限制
     */
    void setSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;

  protected:
    /**
     * @brief 发布候选轨迹、最优轨迹和局部参考路径用于调试
     * @param transformed_plan: 已转换到控制坐标系的局部参考路径
     */
    void visualize(nav_msgs::msg::Path transformed_plan);

    std::string name_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::unique_ptr<ParametersHandler> parameters_handler_;
    Optimizer optimizer_;
    PathHandler path_handler_;
    TrajectoryVisualizer trajectory_visualizer_;

    bool visualize_;

    double reset_period_;
    // 上一次调用 computeVelocityCommands() 的时间，用于判断是否需要重置优化器。
    rclcpp::Time last_time_called_;
  };

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
