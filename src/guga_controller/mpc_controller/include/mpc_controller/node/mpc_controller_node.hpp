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

#include "mpc_controller/core/mpc_wrapper.hpp"

namespace mpc_controller
{

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

private:
  /** 
   * @brief 从参数服务器加载配置。
   */
  void loadParameters();

  // ROS2
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;

  // MPC控制器
  MpcWrapper mpc_wrapper_;

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
