#ifndef MPC_CONTROLLER_NODE_HPP_
#define MPC_CONTROLLER_NODE_HPP_

// #define PREDICTED_PLAN_DEBUG
// #define SOLVE_TIME_DEBUG
#define REFERENCE_DEBUG
// #define LOCAL_PLAN_LENGTH_DEBUG
// #define PREDICT_INPUT

#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/node_utils.hpp"

#include "mpc_controller/mpc_wrapper.hpp"
#include "mpc_controller/backward.hpp"

namespace backward{
  backward::SignalHandling sh;
}

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

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

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

  /**
   * @brief 获取局部规划路径
   * @param pose 当前机器人位姿。
   * @return 局部规划路径。
   */
  nav_msgs::msg::Path getLocalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 生成参考轨迹
   * @param local_plan 局部规划路径。
   * @return 参考轨迹。
   */
  RefHorizon getReferenceHorizon(const nav_msgs::msg::Path & local_plan);

  // ROS2
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  nav2_costmap_2d::Costmap2D* costmap_{};
  rclcpp::Logger logger_{rclcpp::get_logger("MpcControllerNode")};
  rclcpp::Clock::SharedPtr clock_;

  // pub
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_plan_pub_;

  // MPC控制器
  std::shared_ptr<MpcWrapper> mpc_wrapper_;

  // 已存储的全局规划路径
  nav_msgs::msg::Path global_plan_;

  // 线程安全
  std::mutex mutex_;

  // 配置参数
  MpcConfig mpc_config_;

  // 局部规划路径长度
  double local_plan_length_{2.0};

  inline geometry_msgs::msg::PoseStamped transformPoseToGlobal(const geometry_msgs::msg::PoseStamped & pose) const;

#ifdef PREDICT_INPUT
private: // 尝试使用控制预测序列计算局部规划路径长度
  InputHorizon predicted_inputs_;
  std::thread predict_thread_;
  void predictInputThreadFunction();
#endif
};

}  // namespace mpc_controller

// Tools Functions
inline const StateBound Point2State(const geometry_msgs::msg::PoseStamped& pose)
{
  StateBound state;
  state << pose.pose.position.x, pose.pose.position.y;

  const auto & q = pose.pose.orientation;
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  state << std::atan2(siny, cosy);

  return state;
}

inline Input Vel2Ctrl(const geometry_msgs::msg::Twist& vel)
{
  Input ctrl;
  ctrl << vel.linear.x, vel.linear.y, vel.angular.z;
  return ctrl;
}

inline const geometry_msgs::msg::PoseStamped State2Point(const State& state)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = state[0];
  pose.pose.position.y = state[1];
  pose.pose.position.z = 0.0;

  const double yaw = state[2];
  const double half_yaw = yaw * 0.5;
  pose.pose.orientation.w = std::cos(half_yaw);
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(half_yaw);

  return pose;
}

inline const geometry_msgs::msg::Twist Ctrl2Vel (const Input& vel)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vel[0];
  twist.linear.y = vel[1];
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = vel[2];

  return twist;
}

inline const nav_msgs::msg::Path StateHorizon2Path(const nav_msgs::msg::Path& global_plan, const StateHorizon& state_horizon)
{
  nav_msgs::msg::Path path;
  path.poses.reserve(state_horizon.cols());
  for (int i = 0; i < state_horizon.cols(); ++i) {
    auto pose = State2Point(state_horizon.col(i));
    pose.header = global_plan.header;
    path.poses.push_back(pose);
#ifdef PREDICTED_PLAN_DEBUG
    std::cout << "\033[1;33mPredicted point " << i << ": "
              << "x: " << pose.pose.position.x << ", "
              << "y: " << pose.pose.position.y << ", "
              << "theta: " << pose.pose.orientation.z << "\033[0m" << std::endl;
#endif
  }
  path.header.frame_id = global_plan.header.frame_id;
  return path;
}

#endif  // MPC_CONTROLLER_NODE_HPP_
