#include "mpc_controller/node/mpc_controller_node.hpp"

namespace mpc_controller
{

void MpcControllerNode::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);
  tf_buffer_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  auto node = parent_.lock(); if (!node) { return; }

  // 加载参数
  loadParameters();

  RCLCPP_INFO(node->get_logger(),
    "[mpc_controller] Configured. N=%d, dt=%.3f, mode=%d",
    config_.horizon_n, config_.control_dt,
    static_cast<int>(config_.trajectory_mode));
}

void MpcControllerNode::activate()
{
  if (local_plan_pub_) {
    local_plan_pub_->on_activate();
  }
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Activated");
}

void MpcControllerNode::deactivate()
{
  if (local_plan_pub_) {
    local_plan_pub_->on_deactivate();
  }
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Deactivated");
}

void MpcControllerNode::cleanup()
{
  local_plan_pub_.reset(); costmap_ros_.reset(); tf_buffer_.reset();
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Cleaned up");
}

void MpcControllerNode::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped MpcControllerNode::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  if (global_plan_.poses.empty()) { return cmd_vel; }

  const double current_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  const double lookahead = path_handler_.computeLookahead(std::max(current_speed, 0.1));

  double robot_yaw = 0.0;
  {
    const auto & q = pose.pose.orientation;
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    robot_yaw = std::atan2(siny, cosy);
  }

  const Eigen::Vector3d x0(
    pose.pose.position.x,
    pose.pose.position.y,
    robot_yaw);

  const Eigen::Vector3d u_opt = mpc_wrapper_.solve(x0, ref_traj);

  cmd_vel.twist.linear.x = u_opt(0);
  cmd_vel.twist.linear.y = u_opt(1);
  cmd_vel.twist.angular.z = u_opt(2);

  return cmd_vel;
}

void MpcControllerNode::loadParameters()
{
  auto node = node_.lock();
  if (!node) { return; }

  // 时域
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".horizon_n", rclcpp::ParameterValue(15));
  node->get_parameter(plugin_name_ + ".horizon_n", config_.horizon_n);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_dt", rclcpp::ParameterValue(0.05));
  node->get_parameter(plugin_name_ + ".control_dt", config_.control_dt);

  // 状态权重
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".qx", rclcpp::ParameterValue(10.0));
  node->get_parameter(plugin_name_ + ".qx", config_.qx);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".qy", rclcpp::ParameterValue(10.0));
  node->get_parameter(plugin_name_ + ".qy", config_.qy);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".qtheta", rclcpp::ParameterValue(2.0));
  node->get_parameter(plugin_name_ + ".qtheta", config_.qtheta);

  // 控制权重
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rvx", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".rvx", config_.rvx);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rvy", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".rvy", config_.rvy);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".romega", rclcpp::ParameterValue(0.05));
  node->get_parameter(plugin_name_ + ".romega", config_.romega);

  // 控制约束
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".vx_min", rclcpp::ParameterValue(-3.0));
  node->get_parameter(plugin_name_ + ".vx_min", config_.vx_min);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".vx_max", rclcpp::ParameterValue(3.0));
  node->get_parameter(plugin_name_ + ".vx_max", config_.vx_max);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".vy_min", rclcpp::ParameterValue(-3.0));
  node->get_parameter(plugin_name_ + ".vy_min", config_.vy_min);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".vy_max", rclcpp::ParameterValue(3.0));
  node->get_parameter(plugin_name_ + ".vy_max", config_.vy_max);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".omega_min", rclcpp::ParameterValue(-6.0));
  node->get_parameter(plugin_name_ + ".omega_min", config_.omega_min);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".omega_max", rclcpp::ParameterValue(6.0));
  node->get_parameter(plugin_name_ + ".omega_max", config_.omega_max);

  // 轨迹生成
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".trajectory_mode", rclcpp::ParameterValue(0));
  int mode = 0;
  node->get_parameter(plugin_name_ + ".trajectory_mode", mode);
  config_.trajectory_mode = static_cast<TrajectoryMode>(
    std::clamp(mode, 0, 2));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_min", rclcpp::ParameterValue(0.2));
  node->get_parameter(plugin_name_ + ".lookahead_min",
                      config_.lookahead_min);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_velocity_gain",
    rclcpp::ParameterValue(1.0));
  node->get_parameter(plugin_name_ + ".lookahead_velocity_gain",
                      config_.lookahead_velocity_gain);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_max_length", rclcpp::ParameterValue(5.0));
  node->get_parameter(plugin_name_ + ".path_max_length",
                      config_.path_max_length);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".transform_tolerance",
                      config_.transform_tolerance);
}

}  // namespace mpc_controller

// Pluginlib 注册
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcControllerNode, nav2_core::Controller)
