#include "mpc_controller/node/mpc_controller_node.hpp"

namespace mpc_controller
{

void MpcControllerNode::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros):
    parent_(parent),
    name_(std::move(name)),
    tf_buffer_(std::move(tf)),
    costmap_ros_(std::move(costmap_ros))
{
  auto node = parent_.lock(); if (!node) { return; }
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

  // 初始化wrapper
  mpc_wrapper_ = std::make_shared<MpcWrapper>();
  nav_wrapper_ = std::make_shared<NavWrapper>(tf_buffer_, costmap_ros_, local_plan_pub_, 0.1);

  // 加载参数并配置wrapper
  loadParameters(); ConfigMpcWrapper(mpc_config_); ConfigNavWrapper(nav_config_);

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
  local_plan_pub_.reset(); costmap_ros_.reset(); tf_buffer_.reset();
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
  const double lookahead_distance = nav_wrapper_->computeLookahead(std::max(current_speed, 0.1));

  double robot_yaw = 0.0;
  {
    const auto & q = pose.pose.orientation;
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    robot_yaw = std::atan2(siny, cosy);
  }

  const Eigen::Vector3d x0(pose.pose.position.x, pose.pose.position.y, robot_yaw);

  mpc_wrapper_->set_x0({x0(0), x0(1), x0(2)});
  mpc_wrapper_->set_xinit({x0(0), x0(1), x0(2)});
  mpc_wrapper_->set_uinit({velocity.linear.x, velocity.linear.y, velocity.angular.z});

  const std::vector<double>& ref_point = getLookAheadPoint(lookahead_distance, global_plan_);
  const Eigen::Vector3d u_opt = mpc_wrapper_->solve(x0, ref_point);

  cmd_vel.twist.linear.x = u_opt(0);
  cmd_vel.twist.linear.y = u_opt(1);
  cmd_vel.twist.angular.z = u_opt(2);

  if (nav_wrapper_->use_curvature_limitation()) {
    nav_wrapper_->applyCurvatureLimitation(global_plan_, pose, cmd_vel.twist.linear.x);
  }

  return cmd_vel;
}

const std::vector<double>& MpcControllerNode::getLookAheadPoint(const double& lookahead_dist, const nav_msgs::msg::Path& transformed_plan) const
{
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(),
    [&](const auto& ps) {
      return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    }
  );

  std::vector<double> lookahead_point(3, 0.0);
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
    lookahead_point = convertPoint2Vector(goal_pose_it);
  }
  else if (nav_wrapper_->use_interpolation() && goal_pose_it != transformed_plan.poses.begin())
  {
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = geometry_utils::circleSegmentIntersection(
        prev_pose_it->pose.position, goal_pose_it->pose.position,
        lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    lookahead_point = convertPoint2Vector(pose);
  }

  return lookahead_point;
}

void MpcControllerNode::loadParameters()
{
  auto node = node_.lock(); if (!node) { return; }

  // --- MPC参数部分 ---
  // 预测时域与控制时域
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".horizon_n", rclcpp::ParameterValue(15));
  node->get_parameter(plugin_name_ + ".horizon_n", config_.horizon_n);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".control_dt", rclcpp::ParameterValue(0.05));
  node->get_parameter(plugin_name_ + ".control_dt", mpc_config_.control_dt);

  // 状态权重
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".qx", rclcpp::ParameterValue(10.0));
  node->get_parameter(plugin_name_ + ".qx", mpc_config_.cost_weights.qx);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".qy", rclcpp::ParameterValue(10.0));
  node->get_parameter(plugin_name_ + ".qy", mpc_config_.cost_weights.qy);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".qtheta", rclcpp::ParameterValue(2.0));
  node->get_parameter(plugin_name_ + ".qtheta", mpc_config_.cost_weights.qtheta);

  // 控制权重
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rvx", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".rvx", mpc_config_.cost_weights.rvx);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rvy", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".rvy", mpc_config_.cost_weights.rvy);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".romega", rclcpp::ParameterValue(0.05));
  node->get_parameter(plugin_name_ + ".romega", mpc_config_.cost_weights.romega);

  // 终端状态权重
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".qx_e", rclcpp::ParameterValue(10.0));
  node->get_parameter(plugin_name_ + ".qx_e", mpc_config_.cost_weights.qx_e);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".qy_e", rclcpp::ParameterValue(10.0));
  node->get_parameter(plugin_name_ + ".qy_e", mpc_config_.cost_weights.qy_e);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".qtheta_e", rclcpp::ParameterValue(2.0));
  node->get_parameter(plugin_name_ + ".qtheta_e", mpc_config_.cost_weights.qtheta_e);

  // 控制约束
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".vx_min", rclcpp::ParameterValue(-3.0));
  node->get_parameter(plugin_name_ + ".vx_min", mpc_config_.vx_min);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".vx_max", rclcpp::ParameterValue(3.0));
  node->get_parameter(plugin_name_ + ".vx_max", mpc_config_.vx_max);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".vy_min", rclcpp::ParameterValue(-3.0));
  node->get_parameter(plugin_name_ + ".vy_min", mpc_config_.vy_min);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".vy_max", rclcpp::ParameterValue(3.0));
  node->get_parameter(plugin_name_ + ".vy_max", mpc_config_.vy_max);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".omega_min", rclcpp::ParameterValue(-6.0));
  node->get_parameter(plugin_name_ + ".omega_min", mpc_config_.omega_min);

  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".omega_max", rclcpp::ParameterValue(6.0));
  node->get_parameter(plugin_name_ + ".omega_max", mpc_config_.omega_max);

  // --- NAV参数部分 ---
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
  node->get_parameter(plugin_name_ + ".use_interpolation", nav_config_.use_interpolation);
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

void MpcControllerNode::ConfigNavWrapper(NavConfig & config)
{
  nav_wrapper_->setUseInterpolation(config.use_interpolation);
}

} // namespace mpc_controller

// Pluginlib 注册
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcControllerNode, nav2_core::Controller)