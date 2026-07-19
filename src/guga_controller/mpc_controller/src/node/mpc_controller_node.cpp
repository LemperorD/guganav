#include "mpc_controller/node/mpc_controller_node.hpp"

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
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);

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
  if (carrot_pub_) { carrot_pub_->on_activate(); }
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Activated");
}

void MpcControllerNode::deactivate()
{
  if (local_plan_pub_) { local_plan_pub_->on_deactivate(); }
  if (carrot_pub_) { carrot_pub_->on_deactivate(); }
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Deactivated");
}

void MpcControllerNode::cleanup()
{
  local_plan_pub_.reset(); carrot_pub_.reset();
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
  const double lookahead_distance = nav_wrapper_->getLookaheadDistance();
  std::cout << "Current Speed: " << current_speed << ", Lookahead Distance: " << lookahead_distance << std::endl;

  double robot_yaw = 0.0;
  {
    const auto & q = pose.pose.orientation;
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    robot_yaw = std::atan2(siny, cosy);
  }

  std::vector<double> x0(3);
  x0[0] = pose.pose.position.x;
  x0[1] = pose.pose.position.y;
  x0[2] = robot_yaw;

  // mpc_wrapper_->set_x0({x0[0], x0[1], x0[2]});
  // mpc_wrapper_->set_xinit({x0[0], x0[1], x0[2]});

  mpc_wrapper_->set_x0({0, 0, 0});
  mpc_wrapper_->set_xinit({0, 0, 0});
  mpc_wrapper_->set_uinit({0, 0, 0});

  auto transformed_plan = nav_wrapper_->transformGlobalPlan(pose, global_plan_);

  std::vector<double> ref_point = getLookAheadPoint(lookahead_distance, transformed_plan);
  mpc_wrapper_->set_yref({ref_point[0], ref_point[1], ref_point[2]}, {0, 0, 0});

  std::cout << "Lookahead Point: [" << ref_point[0] << ", " << ref_point[1] << ", " << ref_point[2] << "]" << std::endl;

  std::vector<double> u_opt = mpc_wrapper_->solve();

  cmd_vel.twist.linear.x = u_opt[0];
  cmd_vel.twist.linear.y = u_opt[1];
  cmd_vel.twist.angular.z = u_opt[2];

  // if (nav_wrapper_->use_curvature_scaling()) {
  //   nav_wrapper_->applyCurvatureLimitation(global_plan_, pose, cmd_vel.twist.linear.x);
  // }

  return cmd_vel;
}

std::vector<double> MpcControllerNode::getLookAheadPoint(const double& lookahead_dist, const nav_msgs::msg::Path& transformed_plan) const
{
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(),
    [&](const auto& ps) {
      return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    }
  );

  std::vector<double> lookahead_point(3, 0.0);
  if (goal_pose_it == transformed_plan.poses.end()) {
    std::cout << "use end point" << std::endl;
    goal_pose_it = std::prev(transformed_plan.poses.end());
    carrot_pub_->publish(createCarrotMsg(*goal_pose_it));
    lookahead_point = convertPoint2Vector(*goal_pose_it);
  }
  else if (nav_wrapper_->use_interpolation() && goal_pose_it != transformed_plan.poses.begin())
  {
    std::cout << "use interpolated point" << std::endl;
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = nav_wrapper_->circleSegmentIntersection(
        prev_pose_it->pose.position, goal_pose_it->pose.position,
        lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    carrot_pub_->publish(createCarrotMsg(pose));
    lookahead_point = convertPoint2Vector(pose);
  }
  else
  {
    std::cout << "use next point" << std::endl;
    carrot_pub_->publish(createCarrotMsg(*goal_pose_it));
    lookahead_point = convertPoint2Vector(*goal_pose_it);
  }

  return lookahead_point;
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

  // --- NAV参数部分 ---
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".nav.use_interpolation", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".nav.use_interpolation", nav_config_.use_interpolation);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".nav.use_curvature_scaling", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".nav.use_curvature_scaling", nav_config_.use_curvature_scaling);

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".nav.lookahead_distance", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".nav.lookahead_distance", nav_config_.lookahead_distance);
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
  nav_wrapper_->setUseCurvatureScaling(config.use_curvature_scaling);
  nav_wrapper_->setLookaheadDistance(config.lookahead_distance);
}

void MpcControllerNode::setSpeedLimit(const double & speed_limit, const bool & percentage)
{

}

} // namespace mpc_controller

// Pluginlib 注册
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcControllerNode, nav2_core::Controller)