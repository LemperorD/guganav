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

#ifndef NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
#define NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_

#include <string>
#include <memory>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/critic_manager.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/tools/noise_generator.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#ifdef __APPLE__
  #include "nav2_mppi_controller/tools/apple_utils.hpp"
#endif

namespace mppi
{

/**
 * @class mppi::Optimizer
 * @brief MPPI 控制器的核心采样、评分与控制序列优化器
 */
class Optimizer
{
public:
  /**
    * @brief 构造 MPPI 优化器
    */
  Optimizer() = default;

  /**
   * @brief 析构优化器并关闭噪声生成线程
   */
  ~Optimizer() {shutdown();}


  /**
   * @brief 启动时初始化优化器、运动模型和评分器
   * @param parent: 控制器服务器生命周期节点的弱引用
   * @param name: 插件实例名称，用作参数命名空间
   * @param costmap_ros: 提供环境代价地图的对象
   * @param dynamic_parameter_handler: 动态参数处理器
   */
  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler);

  /**
   * @brief 进程结束时关闭优化器持有的后台资源
   */
  void shutdown();

  /**
   * @brief 使用 MPPI 算法计算一个控制周期的最优控制
   * @param robot_pose: 当前机器人位姿
   * @param robot_speed: 当前机器人速度
   * @param plan: 待跟踪的局部参考路径
   * @param goal_checker: 用于判断目标完成状态的检查器
   * @return 返回值: 从最优控制序列取出的首个可执行速度指令，供 controller_server 下发
   */
  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    nav2_core::GoalChecker * goal_checker);

  /**
   * @brief 获取当前周期生成的全部候选轨迹
   * @return 返回值: 候选轨迹集合的引用，供轨迹可视化器发布采样结果
   */
  models::Trajectories & getGeneratedTrajectories();

  /**
   * @brief 获取当前周期的最优预测轨迹
   * @return 返回值: 由最优控制序列积分得到的位姿张量，供可视化和调试使用
   */
  xt::xtensor<float, 2> getOptimizedTrajectory();

  /**
   * @brief 根据外部速度限制更新优化器速度约束
   * @param speed_limit: 要应用的速度限制
   * @param percentage: 为 true 时将限制解释为基准速度的百分比
   */
  void setSpeedLimit(double speed_limit, bool percentage);

  /**
   * @brief 将优化问题恢复到初始状态
   */
  void reset();

protected:
  /**
   * @brief 生成候选轨迹、调用 critics 评分并更新最优控制序列
   */
  void optimize();

  /**
   * @brief 为新一轮轨迹展开准备机器人状态、路径和评分上下文
   * @param robot_pose: 当前机器人位姿
   * @param robot_speed: 当前机器人速度
   * @param plan: 待跟踪的局部参考路径
   * @param goal_checker: 用于判断目标完成状态的检查器
   */
  void prepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker);

  /**
   * @brief 读取优化器主要参数并建立动态参数回调
   */
  void getParams();

  /**
   * @brief 根据底盘类型设置运动模型
   * @param model: 要使用的运动模型名称
   */
  void setMotionModel(const std::string & model);

  /**
   * @brief 将最优控制序列前移一个时间步，作为下一控制周期的初值
   */
  void shiftControlSequence();

  /**
   * @brief 围绕上一周期最优控制添加噪声并更新候选轨迹
   */
  void generateNoisedTrajectories();

  /**
   * @brief 对控制序列应用底盘硬约束
   */
  void applyControlSequenceConstraints();

  /**
   * @brief 更新状态张量内每个时间步的预测速度
   * @param state: 待写入预测速度的状态张量
   */
  void updateStateVelocities(models::State & state) const;

  /**
   * @brief 用实测速度设置状态张量的初始速度
   * @param state: 待写入初始速度的状态张量
   */
  void updateInitialStateVelocities(models::State & state) const;

  /**
   * @brief 使用运动模型在整个预测时域传播状态速度
   * @param state: 待写入预测速度的状态张量
   */
  void propagateStateVelocitiesFromInitials(models::State & state) const;

  /**
   * @brief 将批量状态速度积分为候选位姿轨迹
   * @param trajectories: 接收积分结果的候选轨迹集合
   * @param state: 包含各采样控制速度的状态张量
   */
  void integrateStateVelocities(
    models::Trajectories & trajectories,
    const models::State & state) const;

  /**
   * @brief 将单条速度序列积分为位姿轨迹
   * @param trajectories: 接收积分结果的位姿张量
   * @param state: 包含线速度和角速度的序列张量
   */
  void integrateStateVelocities(
    xt::xtensor<float, 2> & trajectories,
    const xt::xtensor<float, 2> & state) const;

  /**
   * @brief 使用轨迹代价的 softmax 权重更新最优控制序列
   */
  void updateControlSequence();

  /**
   * @brief 将最优控制序列中的可执行项转换为速度消息
   * @param stamp: 写入输出消息的时间戳
   * @return 返回值: 带坐标系和时间戳的速度指令，供底盘控制链执行
   */
  geometry_msgs::msg::TwistStamped
  getControlFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp);

  /**
   * @brief 判断当前运动模型是否支持横向运动
   * @return 返回值: 为 true 时优化器会生成并积分 `vy`，否则只处理 `vx` 和 `wz`
   */
  bool isHolonomic() const;

  /**
   * @brief 根据控制频率和模型步长决定下一周期是否使用控制序列偏移
   */
  void setOffset(double controller_frequency);

  /**
   * @brief 当全部候选轨迹碰撞时执行重试恢复逻辑
   * @param fail: 当前优化迭代是否失败
   * @return 返回值: 恢复后仍失败时返回 true，调用方据此抛出控制器异常
   */
  bool fallback(bool fail);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string name_;

  std::shared_ptr<MotionModel> motion_model_;

  ParametersHandler * parameters_handler_;
  CriticManager critic_manager_;
  NoiseGenerator noise_generator_;

  models::OptimizerSettings settings_;

  models::State state_;
  models::ControlSequence control_sequence_;
  std::array<mppi::models::Control, 4> control_history_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  xt::xtensor<float, 1> costs_;

  CriticData critics_data_ =
  {state_, generated_trajectories_, path_, costs_, settings_.model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};  /// 注意：此结构保存成员引用，成员声明顺序不可随意调整。

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

template<typename E>
inline auto cumsum_1d(const E & expression)
{
  #ifdef __APPLE__
  return utils::manual_cumsum_1d(expression);
  #else
  return xt::cumsum(expression, 0);
  #endif
}

template<typename E>
inline auto cumsum_2d(const E & expression, int axis)
{
 #ifdef __APPLE__
  return utils::manual_cumsum_2d(expression, axis);
 #else
  return xt::cumsum(expression, axis);
 #endif
}

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
