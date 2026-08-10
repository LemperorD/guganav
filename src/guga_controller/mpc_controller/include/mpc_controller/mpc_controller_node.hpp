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
#include "nav2_util/node_utils.hpp"

#include "mpc_controller/mpc_wrapper.hpp"

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

  void setSpeedLimit(
    const double & speed_limit,
    const bool & percentage) override;

private:

  /** 
   * @brief 从参数服务器加载配置。
   */
  void loadParameters();

  /**
   * @brief 配置MpcWrapper。
   * @param config MpcWrapper配置结构体。
   */
  void ConfigMpcWrapper(MpcConfig & config);

  // ROS2
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  nav2_costmap_2d::Costmap2D* costmap_{};
  rclcpp::Logger logger_{rclcpp::get_logger("MpcControllerNode")};
  rclcpp::Clock::SharedPtr clock_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;

  // MPC控制器
  std::shared_ptr<MpcWrapper> mpc_wrapper_;

  // 已存储的全局规划路径
  nav_msgs::msg::Path global_plan_;

  // 线程安全
  std::mutex mutex_;

  // 配置参数
  MpcConfig mpc_config_;
};

}  // namespace mpc_controller

inline const StateBound Point2State(const geometry_msgs::msg::PoseStamped& pose) const
{
  const StateBound state(3, 0.0);
  state[0] = pose.pose.position.x;
  state[1] = pose.pose.position.y;

  const auto & q = pose.pose.orientation;
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  state[2] = std::atan2(siny, cosy);

  return state;
}

inline const Control Vel2Ctrl (const geometry_msgs::msg::Twist& vel) const
{
  const Control ctrl(3, 0.0);
  ctrl[0] = vel.linear.x;
  ctrl[1] = vel.linear.y;
  ctrl[2] = vel.angular.z;

  return ctrl;
}

#endif  // MPC_CONTROLLER_NODE_HPP_
