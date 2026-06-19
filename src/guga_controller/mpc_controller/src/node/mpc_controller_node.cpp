#include "mpc_controller/node/mpc_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace mpc_controller
{

// ==========================================================================
// configure — Lifecycle: 初始化所有组件和参数
// ==========================================================================
void MpcControllerNode::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  auto node = node_.lock();
  if (!node) { return; }

  // 加载参数
  loadParameters();

  // 发布器
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "local_plan", rclcpp::SystemDefaultsQoS());

  // 初始化管道组件
  path_handler_.configure(
    tf_, costmap_ros_, config_.transform_tolerance);
  path_handler_.setLookaheadParams(
    config_.lookahead_min,
    config_.lookahead_velocity_gain,
    config_.path_max_length);

  selectTrajectoryGenerator();
  mpc_solver_.configure(config_);

  RCLCPP_INFO(node->get_logger(),
    "[mpc_controller] Configured. N=%d, dt=%.3f, mode=%d",
    config_.horizon_n, config_.control_dt,
    static_cast<int>(config_.trajectory_mode));
}

// ==========================================================================
// activate / deactivate / cleanup
// ==========================================================================
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
  local_plan_pub_.reset();
  traj_gen_.reset();
  costmap_ros_.reset();
  tf_.reset();
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Cleaned up");
}

// ==========================================================================
// setPlan — 从 Nav2 BT 接收全局路径
// ==========================================================================
void MpcControllerNode::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = path;
}

// ==========================================================================
// computeVelocityCommands — 每 20 Hz 调用一次
// ==========================================================================
geometry_msgs::msg::TwistStamped
MpcControllerNode::computeVelocityCommands(
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

  // ---- 1. 路径变换 ----
  auto local_plan_opt = path_handler_.transformPath(pose, global_plan_);
  if (!local_plan_opt.has_value()) { return cmd_vel; }

  const auto & local_plan = *local_plan_opt;

  // 发布局部路径
  if (local_plan_pub_) {
    local_plan_pub_->publish(local_plan);
  }

  // ---- 2. 速度自适应前瞻 ----
  const double current_speed = std::hypot(
    velocity.linear.x, velocity.linear.y);
  const double lookahead = path_handler_.computeLookahead(
    std::max(current_speed, 0.1));  // 最小速度 0.1 防止除零

  // ---- 3. 轨迹生成 ----
  if (!traj_gen_) {
    traj_gen_ = std::make_unique<DiscreteGenerator>();
  }
  const ReferenceTrajectory ref_traj = traj_gen_->generate(
    local_plan, config_.horizon_n, config_.control_dt, lookahead);

  if (ref_traj.empty()) { return cmd_vel; }

  // ---- 4. MPC 求解 ----
  // 从 pose 获取机器人状态
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

  const Eigen::Vector3d u_opt = mpc_solver_.solve(x0, ref_traj, state_);

  // ---- 5. 应用速度限制并输出 ----
  const double speed_limit = speed_limit_percentage_
    ? speed_limit_ * std::max(
        std::abs(config_.vx_max),
        std::abs(config_.vy_max))
    : speed_limit_;

  const double u_norm = std::hypot(u_opt(0), u_opt(1));
  const double scale = (u_norm > speed_limit && u_norm > 1e-9)
    ? speed_limit / u_norm : 1.0;

  cmd_vel.twist.linear.x = u_opt(0) * scale;
  cmd_vel.twist.linear.y = u_opt(1) * scale;
  cmd_vel.twist.angular.z = u_opt(2);

  // 角速度也应用速度限制（如果需要）
  if (std::abs(cmd_vel.twist.angular.z) > config_.omega_max) {
    cmd_vel.twist.angular.z = std::copysign(
      config_.omega_max, cmd_vel.twist.angular.z);
  }

  return cmd_vel;
}

// ==========================================================================
// setSpeedLimit
// ==========================================================================
void MpcControllerNode::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

// ==========================================================================
// loadParameters — 从参数服务器读取配置
// ==========================================================================
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

  // 平滑权重
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rdvx", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".rdvx", config_.rdvx);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rdvy", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".rdvy", config_.rdvy);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rdomega", rclcpp::ParameterValue(0.3));
  node->get_parameter(plugin_name_ + ".rdomega", config_.rdomega);

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

// ==========================================================================
// selectTrajectoryGenerator
// ==========================================================================
void MpcControllerNode::selectTrajectoryGenerator()
{
  switch (config_.trajectory_mode) {
    case TrajectoryMode::B_SPLINE:
      traj_gen_ = std::make_unique<BSplineGenerator>();
      break;
    case TrajectoryMode::MINCO:
      traj_gen_ = std::make_unique<MincoGenerator>();
      break;
    case TrajectoryMode::DISCRETE:
    default:
      traj_gen_ = std::make_unique<DiscreteGenerator>();
      break;
  }
}

}  // namespace mpc_controller

// ==========================================================================
// Pluginlib 注册
// ==========================================================================
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mpc_controller::MpcControllerNode,
  nav2_core::Controller)
