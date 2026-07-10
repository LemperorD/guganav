#ifndef MPC_CONTROLLER_NODE_HPP_
#define MPC_CONTROLLER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mpc_controller
{

/**
 * @brief MPC 全向移动机器人路径跟踪控制器 — Nav2 插件。
 *
 * 实现 nav2_core::Controller 接口，在 controller_server 中以 20 Hz 运行。
 *
 * 控制流水线:
 *   1. PathHandler::transformPath() — 全局路径 → 局部路径（车体系）
 *   2. TrajectoryGenerator::generate() — Nav2 Path → N 个等距 ReferencePoint
 *   3. MpcSolver::solve() — QP 求解 → 最优控制 u*
 *   4. 返回 TwistStamped (vx, vy, ω)
 */
class MpcControllerNode : public nav2_core::Controller
{
public:
  MpcControllerNode() = default;
  ~MpcControllerNode() override = default;

  // 禁止拷贝
  MpcControllerNode(const MpcControllerNode &) = delete;
  MpcControllerNode & operator=(const MpcControllerNode &) = delete;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  [[nodiscard]] geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit,
                     const bool & percentage) override;

private:
  /** 
   * @brief 从参数服务器加载配置。
   */
  void loadParameters();

  /** 
   * @brief 根据 TrajectoryMode 切换轨迹生成器。
   */
  void selectTrajectoryGenerator();

  // ROS2
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;

  // 发布器
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    local_plan_pub_;

  // 管道组件
  MpcConfig config_;
  MpcState state_;
  PathHandler path_handler_;
  std::unique_ptr<TrajectoryGenerator> traj_gen_;
  MpcSolver mpc_solver_;

  // 已存储的全局规划
  nav_msgs::msg::Path global_plan_;

  // 速度限制
  double speed_limit_{3.0};
  bool speed_limit_percentage_{false};

  // 线程安全
  std::mutex mutex_;
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER_NODE_HPP_
