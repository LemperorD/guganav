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
#include "mpc_controller/core/nav_wrapper.hpp"

inline const std::vector<double>& convertPoint2Vector(const geometry_msgs::msg::PoseStamped& pose)
{
  std::vector<double> point(3, 0.0);
  point[0] = pose.pose.position.x;
  point[1] = pose.pose.position.y;

  const auto & q = pose.pose.orientation;
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  point[2] = std::atan2(siny, cosy);

  return point;
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

  

private:
  /**
   * @brief 获取前瞻点。
   * @param lookahead_dist 前瞻距离。
   * @param transformed_plan 已转换的全局路径。
   * @return 前瞻点向量 (x, y, theta)。
   */
  const std::vector<double>& getLookAheadPoint(
    const double& lookahead_dist, const nav_msgs::msg::Path& transformed_plan) const;

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
   * @brief 配置NavWrapper。
   * @param config NavWrapper配置结构体。
   */
  void ConfigNavWrapper(NavConfig & config);

  // ROS2
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> local_plan_pub_;

  // MPC控制器
  std::shared_ptr<MpcWrapper> mpc_wrapper_;

  // Nav2路径处理器
  std::shared_ptr<NavWrapper> nav_wrapper_;

  // 已存储的全局规划路径
  nav_msgs::msg::Path global_plan_;

  // 线程安全
  std::mutex mutex_;

  // 配置参数
  MpcConfig mpc_config_;
  NavConfig nav_config_;
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER_NODE_HPP_
