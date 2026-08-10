#include "mpc_controller/mpc_controller_node.hpp"

namespace mpc_controller
{

void MpcControllerNode::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) { throw nav2_core::PlannerException("Unable to lock node!"); }
  node_ = parent;
  
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

  // 初始化wrapper
  mpc_wrapper_ = std::make_shared<MpcWrapper>();

  // 加载参数并配置wrapper
  loadParameters(); ConfigMpcWrapper(mpc_config_);

  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Configured");
}

void MpcControllerNode::activate()
{
  if (local_plan_pub_) { local_plan_pub_->on_activate(); }
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Activated");
}

void MpcControllerNode::deactivate()
{
  if (local_plan_pub_) { local_plan_pub_->on_deactivate(); }
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Deactivated");
}

void MpcControllerNode::cleanup()
{
  local_plan_pub_.reset();

  costmap_ros_.reset(); tf_buffer_.reset();
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Cleaned up");
}

void MpcControllerNode::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped MpcControllerNode::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
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

  double robot_yaw = 0.0;
  {
    const auto & q = pose.pose.orientation;
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    robot_yaw = std::atan2(siny, cosy);
  }

  std::vector<double> x0(3, 0.0);
  x0[0] = pose.pose.position.x;
  x0[1] = pose.pose.position.y;
  x0[2] = robot_yaw;

  std::vector<double> u0(3, 0.0);
  u0[0] = velocity.linear.x;
  u0[1] = velocity.linear.y;
  u0[2] = velocity.angular.z;

  mpc_wrapper_->set_x0(x0);


  mpc_wrapper_->set_yref(ref_traj);

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  std::vector<double> u_opt = mpc_wrapper_->solve();
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_time = end_time - start_time;
  RCLCPP_INFO(logger_, "MPC solve time: %.6f seconds", elapsed_time.count());

  cmd_vel.twist.linear.x = u_opt[0];
  cmd_vel.twist.linear.y = u_opt[1];
  cmd_vel.twist.angular.z = u_opt[2];

  return cmd_vel;
}

std::vector<double> MpcControllerNode::getReferencePath() const
{

}

void MpcControllerNode::loadParameters()
{
  auto node = node_.lock(); if (!node) { return; }

  // --- MPC参数部分 ---
  // 预测时域与控制时域
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.horizon_n", rclcpp::ParameterValue(15));
  node->get_parameter(name_ + ".mpc.horizon_n", mpc_config_.horizon_n);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.control_dt", rclcpp::ParameterValue(0.05));
  node->get_parameter(name_ + ".mpc.control_dt", mpc_config_.control_dt);

  // 状态权重
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.qx", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + ".mpc.qx", mpc_config_.cost_weights.qx);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.qy", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + ".mpc.qy", mpc_config_.cost_weights.qy);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.qtheta", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".mpc.qtheta", mpc_config_.cost_weights.qtheta);

  // 控制权重
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.rvx", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + ".mpc.rvx", mpc_config_.cost_weights.rvx);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.rvy", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + ".mpc.rvy", mpc_config_.cost_weights.rvy);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.romega", rclcpp::ParameterValue(0.05));
  node->get_parameter(name_ + ".mpc.romega", mpc_config_.cost_weights.romega);

  // 终端状态权重
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.qx_e", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + ".mpc.qx_e", mpc_config_.cost_weights.qx_e);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.qy_e", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + ".mpc.qy_e", mpc_config_.cost_weights.qy_e);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.qtheta_e", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".mpc.qtheta_e", mpc_config_.cost_weights.qtheta_e);

  // 控制约束
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.vx_min", rclcpp::ParameterValue(-3.0));
  node->get_parameter(name_ + ".mpc.vx_min", mpc_config_.vx_min);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.vx_max", rclcpp::ParameterValue(3.0));
  node->get_parameter(name_ + ".mpc.vx_max", mpc_config_.vx_max);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.vy_min", rclcpp::ParameterValue(-3.0));
  node->get_parameter(name_ + ".mpc.vy_min", mpc_config_.vy_min);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.vy_max", rclcpp::ParameterValue(3.0));
  node->get_parameter(name_ + ".mpc.vy_max", mpc_config_.vy_max);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.omega_min", rclcpp::ParameterValue(-6.0));
  node->get_parameter(name_ + ".mpc.omega_min", mpc_config_.omega_min);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".mpc.omega_max", rclcpp::ParameterValue(6.0));
  node->get_parameter(name_ + ".mpc.omega_max", mpc_config_.omega_max);
}
 
void MpcControllerNode::ConfigMpcWrapper(MpcConfig & config)
{
  // 设置MPC求解器的代价函数权重矩阵
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d QE = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  Q <<
    config.cost_weights.qx,0,0,
    0,config.cost_weights.qy,0,
    0,0,config.cost_weights.qtheta;
  QE <<
    config.cost_weights.qx_e,0,0,
    0,config.cost_weights.qy_e,0,
    0,0,config.cost_weights.qtheta_e;
  R <<
    config.cost_weights.rvx,0,0,
    0,config.cost_weights.rvy,0,
    0,0,config.cost_weights.romega;
  mpc_wrapper_->setCosts(Q, R, QE);

  // 设置MPC求解器的控制输入约束
  mpc_wrapper_->setControlLimits(
    config.vx_min, config.vx_max, config.vy_min, config.vy_max,
    config.omega_min, config.omega_max);
}

void MpcControllerNode::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // USELESS API
}

} // namespace mpc_controller

// Pluginlib 注册
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcControllerNode, nav2_core::Controller)