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
  predicted_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("predicted_plan", 1);

  // 初始化wrapper
  mpc_wrapper_ = std::make_shared<MpcWrapper>();

  // 加载参数并配置wrapper
  loadParameters(); ConfigMpcWrapper(mpc_config_);

#ifdef PREDICT_INPUT
  // 初始化预测输入线程, 1Hz
  predict_thread_ = std::thread(&MpcControllerNode::predictInputThreadFunction, this);
#endif

  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Configured");
}

void MpcControllerNode::activate()
{
  if (local_plan_pub_) { local_plan_pub_->on_activate(); }
  if (predicted_plan_pub_) { predicted_plan_pub_->on_activate(); }
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Activated");
}

void MpcControllerNode::deactivate()
{
  if (local_plan_pub_) { local_plan_pub_->on_deactivate(); }
  if (predicted_plan_pub_) { predicted_plan_pub_->on_deactivate(); }
  RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Deactivated");
}

void MpcControllerNode::cleanup()
{
  local_plan_pub_.reset();
  predicted_plan_pub_.reset();
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
  (void)velocity;

  // 将机器人位姿转换到全局路径的坐标系并设置为初始状态约束
  geometry_msgs::msg::PoseStamped global_pose = transformPoseToGlobal(pose);
  StateBound x0 = Point2State(global_pose);
  mpc_wrapper_->setInitialState(x0);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = global_pose.header;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  if (global_plan_.poses.empty()) { return cmd_vel; }

  // 计算局部路径并写入参考轨迹
  auto local_plan = getLocalPlan(global_pose);
  local_plan_pub_->publish(local_plan);
  auto ref_traj = getReferenceHorizon(local_plan);
  TerminalRef end_ref = ref_traj.col(kHorizonSteps - 1).head<3>();

#ifdef REFERENCE_DEBUG
  std::cout << "\033[1;33mReference trajectory: \033[0m" << std::endl;
  for (int i = 0; i < ref_traj.cols(); ++i) {
    std::cout << "\033[1;33mPoint " << i << ": x=" << ref_traj(0, i) << ", y=" << ref_traj(1, i) << ", theta=" << ref_traj(2, i) << "\033[0m" << std::endl;
  }
  std::cout << "\033[1;34mTerminal reference: x=" << end_ref(0) << ", y=" << end_ref(1) << ", theta=" << end_ref(2) << "\033[0m" << std::endl;
#endif

  mpc_wrapper_->setReferenceTrajectory(ref_traj, end_ref);

  // 求解MPC并获取预测状态轨迹
  auto predicted_states = mpc_wrapper_->predictedStates();
  if (predicted_states.size() != 0) {
    auto predicted_plan = StateHorizon2Path(global_plan_, predicted_states);
    predicted_plan_pub_->publish(predicted_plan);
#ifdef PREDICTED_PLAN_DEBUG
    std::cout << "\033[1;32mPredicted plan published,"
              << "size: " << predicted_plan.poses.size() << ", "
              << "frame: " << predicted_plan.header.frame_id
              << "\033[0m" << std::endl;
#endif
  } else {
    std::cout << "\033[1;31mPredicted states are empty\033[0m" << std::endl;
  }
  
  // 求解
  Input u_opt = mpc_wrapper_->solve();

#ifdef SOLVE_TIME_DEBUG
  double solve_time = mpc_wrapper_->solve_time();
  std::cout << "\033[1;34mMPC solve time: " << solve_time << " s\033[0m" << std::endl;
#endif

  // acados 模型将 u=[vx,vy,omega] 建模为 body 系速度
  // (x_dot = vx*cos(theta) - vy*sin(theta))，Nav2 cmd_vel 也是 body 系，
  // 因此直接输出，不做任何旋转。
  cmd_vel.twist.linear.x = u_opt[0];
  cmd_vel.twist.linear.y = u_opt[1];
  cmd_vel.twist.angular.z = u_opt[2];

  return cmd_vel;
}

nav_msgs::msg::Path MpcControllerNode::getLocalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path local_plan;
  local_plan.header = global_plan_.header;

  if (global_plan_.poses.empty()) return local_plan;

  // 1. 找到全局路径上距离机器人最近的点
  auto closest = std::min_element(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&pose](const auto &a, const auto &b) {
      return std::hypot(a.pose.position.x - pose.pose.position.x,
                        a.pose.position.y - pose.pose.position.y) <
             std::hypot(b.pose.position.x - pose.pose.position.x,
                        b.pose.position.y - pose.pose.position.y);
    });

  // 2. 从最近点开始沿路径前进, 累计距离不超过 local_plan_length_
  local_plan.poses.push_back(*closest);

#ifdef LOCAL_PLAN_LENGTH_DEBUG
  std::cout << "local_plan_length: " << local_plan_length_ << std::endl;
#endif

  double cum_dist = 0.0;
  for (auto it = closest; it + 1 != global_plan_.poses.end(); ++it) {
    double dx = (it + 1)->pose.position.x - it->pose.position.x;
    double dy = (it + 1)->pose.position.y - it->pose.position.y;
    cum_dist += std::hypot(dx, dy);
    local_plan.poses.push_back(*(it + 1));
    if (cum_dist >= local_plan_length_) break;
  }

  return local_plan;
}

RefHorizon MpcControllerNode::getReferenceHorizon(const nav_msgs::msg::Path & local_plan)
{
  RefHorizon ref = RefHorizon::Zero();

  if (local_plan.poses.size() < 2) return ref;

  // 1. 计算局部路径的累计弧长
  std::vector<double> arc_len(local_plan.poses.size(), 0.0);
  for (size_t i = 1; i < local_plan.poses.size(); ++i) {
    double dx = local_plan.poses[i].pose.position.x - local_plan.poses[i-1].pose.position.x;
    double dy = local_plan.poses[i].pose.position.y - local_plan.poses[i-1].pose.position.y;
    arc_len[i] = arc_len[i - 1] + std::hypot(dx, dy);
  }

  const double total_len = arc_len.back();

  // 2. 沿路径均匀采样 N 个参考点
  //    stage 0 对应最近的参考, stage N-1 对应最远的参考
  const double step = total_len / kHorizonSteps;

  for (int s = 0; s < kHorizonSteps; ++s) {
    double target = step * (s + 1);  // s 越大参考点越远

    // 二分查找目标弧长所在的路径段
    auto it = std::lower_bound(arc_len.begin(), arc_len.end(), target);

    if (it == arc_len.begin()) {
      // 目标距离小于第一个段, 直接用起点
      ref(0, s) = local_plan.poses[0].pose.position.x;
      ref(1, s) = local_plan.poses[0].pose.position.y;
      ref(2, s) = 0.0;
    } else {
      size_t idx = std::min(static_cast<size_t>(std::distance(arc_len.begin(), it)),
                            arc_len.size() - 1);

      double seg_len = arc_len[idx] - arc_len[idx - 1];
      double ratio = (seg_len > 1e-9)
        ? std::clamp((target - arc_len[idx - 1]) / seg_len, 0.0, 1.0)
        : 0.0;

      // 线性插值位置
      ref(0, s) = local_plan.poses[idx-1].pose.position.x + ratio *
                 (local_plan.poses[idx].pose.position.x - local_plan.poses[idx-1].pose.position.x);
      ref(1, s) = local_plan.poses[idx-1].pose.position.y + ratio *
                 (local_plan.poses[idx].pose.position.y - local_plan.poses[idx-1].pose.position.y);

      // θ_ref: 路径切线方向
      double dx = local_plan.poses[idx].pose.position.x - local_plan.poses[idx-1].pose.position.x;
      double dy = local_plan.poses[idx].pose.position.y - local_plan.poses[idx-1].pose.position.y;
      ref(2, s) = std::atan2(dy, dx);
    }

    // 控制参考量设为零 (只惩罚控制努力, 不预设速度方向)
    ref(3, s) = 0.0;  // vx_ref
    ref(4, s) = 0.0;  // vy_ref
    ref(5, s) = 0.0;  // ω_ref
  }

  return ref;
}

void MpcControllerNode::loadParameters()
{
  auto node = node_.lock(); if (!node) { return; }

  // --- MPC参数部分 ---
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

  nav2_util::declare_parameter_if_not_declared(node, name_ + ".local_plan_length", rclcpp::ParameterValue(6.0));
  node->get_parameter(name_ + ".local_plan_length", local_plan_length_);

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
  mpc_wrapper_->setCostWeights(Q, R, QE);

  // 设置MPC求解器的控制输入约束
  mpc_wrapper_->setControlLimits(
    config.vx_min, config.vx_max, config.vy_min, config.vy_max,
    config.omega_min, config.omega_max);
}

void MpcControllerNode::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  RCLCPP_DEBUG(logger_, "Setting speed limit: %f, percentage: %d", speed_limit, percentage);
  // USELESS API, 这样写减少编译时的warning
}

inline geometry_msgs::msg::PoseStamped MpcControllerNode::transformPoseToGlobal(const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  try {
    transformed_pose = tf_buffer_->transform(pose, global_plan_.header.frame_id, tf2::durationFromSec(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "Transform failed: %s", ex.what());
    transformed_pose = pose; // 如果转换失败，返回原始位姿
  }
  return transformed_pose;
}

// geometry_msgs::msg::Twist MpcControllerNode::transformVelocity( const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff) {
//   geometry_msgs::msg::Twist out;
//   out.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
//   out.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
//   out.angular.z = twist->angular.z;
//   return out;
// }

#ifdef PREDICT_INPUT
void MpcControllerNode::predictInputThreadFunction()
{
  predicted_inputs_ = mpc_wrapper_->predictedInputs();
}
#endif

} // namespace mpc_controller

// Pluginlib 注册
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcControllerNode, nav2_core::Controller)
