// #define ODEMETRY_DEBUG

#include "nonrotating_vel_transform/nonrotating_vel_transform.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nonrotating_vel_transform
{

// 控制器尚未激活（没收到 local_plan）的时间阈值，用于决定 odom 是否直接更新角度
constexpr double CONTROLLER_TIMEOUT = 0.5;

NonrotatingVelTransform::NonrotatingVelTransform(const rclcpp::NodeOptions & options)
: Node("nonrotating_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start Nonrotating Vel Transform!");

  onConfigure(); // 配置参数

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  vis_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(vis_cmd_vel_topic_, 10);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1,
    std::bind(&NonrotatingVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&NonrotatingVelTransform::cmdVelCallback, this, std::placeholders::_1));

  chassis_mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    chassis_mode_topic_, 1,
    std::bind(&NonrotatingVelTransform::chassisModeCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&NonrotatingVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&NonrotatingVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(std::bind(
    &NonrotatingVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  tf_sub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&NonrotatingVelTransform::updateGimbalYaw, this));

  // 50Hz Timer to send transform from `robot_base_frame` to `nonrotating_robot_base_frame`
  tf_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&NonrotatingVelTransform::publishTransform, this));

  // 定时重发速度指令（默认 50 Hz）：控制器只在 30 Hz 更新指令，而本节点此前只在
  // odom+local_plan 同步时（约 10 Hz）发布，等于把 3 个控制周期压成一个阶跃。
  // 这里按固定频率把最新指令 + 当前角度补偿重发出去，只做无损传输。
  const double period_s = 1.0 / std::clamp(publish_frequency_, 1.0, 200.0);
  cmd_vel_pub_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_s)),
    std::bind(&NonrotatingVelTransform::publishCommand, this));
}

void NonrotatingVelTransform::chassisModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  chassis_mode_ = msg->data;
}

void NonrotatingVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  spin_speed_ = msg->data;
}

void NonrotatingVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // NOTE: Haven't synced with local_plan
  if ((rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
#ifdef ODEMETRY_DEBUG
    std::cout << "odom parent frame: " << msg->header.frame_id << std::endl;
    std::cout << "odom child frame: " << msg->child_frame_id << std::endl;
#endif
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
    last_odom_stamp_ = msg->header.stamp;
  }
}

double NonrotatingVelTransform::estimateRobotBaseAngle() const
{
  // littleTES/goHome：底盘 yaw 变化率 ≈ spin_speed_（路径 wz 远小于自旋）。
  // odometry 话题受点云频率限制（约 10Hz），两次更新间用最后 yaw + spin*dt
  // 线性外推，旋转补偿与 tf 连续不跳变（否则 cmd_vel 会飘）。
  if (chassis_mode_ == chassisFollowed) {
    return current_robot_base_angle_;
  }
  if (last_odom_stamp_.nanoseconds() == 0) {
    return current_robot_base_angle_;
  }
  const double dt = (this->now() - last_odom_stamp_).seconds();
  if (dt <= 0.0 || dt > 0.5) {
    return current_robot_base_angle_;
  }
  return current_robot_base_angle_ + spin_speed_ * dt;
}

void NonrotatingVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 只缓存，不在这里发布：发布统一交给 publishCommand() 定时器，保证送到 MCU 的
  // 指令是"最新一条"且频率稳定（恢复行为、遥控等不产生 local_plan 的指令同样覆盖）。
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  latest_cmd_vel_ = msg;
  last_cmd_vel_time_ = this->now();
}

void NonrotatingVelTransform::publishCommand()
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  if (!latest_cmd_vel_) {
    return;  // 还没收到过指令：保持静默，不要凭空发 0
  }

  const double since_last = (this->now() - last_cmd_vel_time_).seconds();
  if (since_last > cmd_vel_timeout_) {
    // 上游停了（控制器/smoother 挂了）：必须主动下发 0，不能继续重发最后一条
    // 非零指令，否则机器人会带着旧速度一直跑。全 0 与坐标系无关，直接发。
    cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%.2f s 未收到速度指令（> cmd_vel_timeout %.2f s），已下发 0 速度", since_last,
      cmd_vel_timeout_);
    return;
  }

  const double yaw_diff =
    selectVelocityYawDiff(chassis_mode_, chassis_followed_yaw_, estimateRobotBaseAngle());
  auto aft_tf_vel = transformVelocity(latest_cmd_vel_, yaw_diff);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  // 可视化约 1/3 频率（50 Hz 下 ~17 Hz），避免 RViz 被 marker 刷屏
  if (++vis_pub_counter_ % 3 == 0) {
    visualizeVelocity(aft_tf_vel);
  }
}

void NonrotatingVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = rclcpp::Clock().now();
}

void NonrotatingVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  // 只用同步到的 odom 更新角度估计（重发时由 estimateRobotBaseAngle() 外推）；
  // 发布统一由 publishCommand() 定时器负责，避免两条不同频率的发布路径。
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  last_odom_stamp_ = odom_msg->header.stamp;
}

void NonrotatingVelTransform::updateGimbalYaw()
{
  try {
    auto tf = tf_buffer_->lookupTransform(chassis_frame_, robot_base_frame_, tf2::TimePointZero);

    tf2::Quaternion q(
      tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
      tf.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    chassis_followed_yaw_ = yaw;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to lookup TF %s->%s: %s", chassis_frame_.c_str(),
      robot_base_frame_.c_str(), ex.what());
  }
}

void NonrotatingVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = nonrotating_robot_base_frame_;

  double tf_yaw = 0.0;
  if (chassis_mode_ == chassisFollowed)
    tf_yaw = -chassis_followed_yaw_;
  else
    tf_yaw = -estimateRobotBaseAngle();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, tf_yaw);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

double NonrotatingVelTransform::selectVelocityYawDiff(
  uint8_t chassis_mode, double chassis_followed_yaw, double robot_base_angle)
{
  return chassis_mode == chassisFollowed ? chassis_followed_yaw : robot_base_angle;
}

geometry_msgs::msg::Twist NonrotatingVelTransform::rotateVelocity(
  const geometry_msgs::msg::Twist & twist, double yaw_diff)
{
  geometry_msgs::msg::Twist out = twist;
  out.linear.x = twist.linear.x * std::cos(yaw_diff) + twist.linear.y * std::sin(yaw_diff);
  out.linear.y = -twist.linear.x * std::sin(yaw_diff) + twist.linear.y * std::cos(yaw_diff);
  return out;
}

geometry_msgs::msg::Twist NonrotatingVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, double yaw_diff)
{
  const double nonrotating_to_chassis_yaw = chassis_followed_yaw_ - yaw_diff;
  auto out = output_in_chassis_frame_ ? rotateVelocity(*twist, nonrotating_to_chassis_yaw)
                                      : rotateVelocity(*twist, yaw_diff);
  if (chassis_mode_ == chassisFollowed) {
    out.angular.z = twist->angular.z;
  } else {
    out.angular.z = twist->angular.z + spin_speed_;
  }
  return out;
}

void NonrotatingVelTransform::visualizeVelocity(const geometry_msgs::msg::Twist & vel)
{
  auto now = this->get_clock()->now();
  double scale = vis_scale_;

  visualization_msgs::msg::Marker linear_marker;
  linear_marker.header.frame_id = output_in_chassis_frame_ ? chassis_frame_ : robot_base_frame_;
  linear_marker.header.stamp = now;
  linear_marker.ns = "cmd_vel";
  linear_marker.id = 0;
  linear_marker.type = visualization_msgs::msg::Marker::ARROW;
  linear_marker.action = visualization_msgs::msg::Marker::ADD;
  linear_marker.pose.orientation.w = 1.0;
  linear_marker.scale.x = 0.06;
  linear_marker.scale.y = 0.12;
  linear_marker.scale.z = 0.0;
  linear_marker.color.r = 0.0;
  linear_marker.color.g = 1.0;
  linear_marker.color.b = 0.0;
  linear_marker.color.a = 0.8;
  linear_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;

  geometry_msgs::msg::Point end;
  end.x = vel.linear.x * scale;
  end.y = vel.linear.y * scale;
  end.z = 0.0;

  linear_marker.points.push_back(start);
  linear_marker.points.push_back(end);

  vis_marker_pub_->publish(linear_marker);

  visualization_msgs::msg::Marker angular_marker;
  angular_marker.header.frame_id = output_in_chassis_frame_ ? chassis_frame_ : robot_base_frame_;
  angular_marker.header.stamp = now;
  angular_marker.ns = "cmd_vel";
  angular_marker.id = 1;
  angular_marker.type = visualization_msgs::msg::Marker::ARROW;
  angular_marker.action = visualization_msgs::msg::Marker::ADD;
  angular_marker.pose.position.z = 0.05;
  angular_marker.pose.orientation.w = 1.0;
  angular_marker.scale.x = 0.04;
  angular_marker.scale.y = 0.08;
  angular_marker.scale.z = 0.0;
  angular_marker.color.r = 1.0;
  angular_marker.color.g = 0.5;
  angular_marker.color.b = 0.0;
  angular_marker.color.a = 0.8;
  angular_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  geometry_msgs::msg::Point z_start;
  z_start.x = 0.0;
  z_start.y = 0.0;
  z_start.z = 0.0;

  geometry_msgs::msg::Point z_end;
  z_end.x = 0.0;
  z_end.y = 0.0;
  z_end.z = vel.angular.z * scale * 0.5;

  angular_marker.points.push_back(z_start);
  angular_marker.points.push_back(z_end);

  vis_marker_pub_->publish(angular_marker);
}

void NonrotatingVelTransform::onConfigure()
{
  this->declare_parameter<std::string>("robot_base_frame", "base_footprint");
  this->declare_parameter<std::string>(
    "nonrotating_robot_base_frame", "base_footprint_nonrotating");
  this->declare_parameter<std::string>("chassis_frame", "chassis");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("vis_cmd_vel_topic", "cmd_vel_marker");
  this->declare_parameter<std::string>("vis_frame_id", "base_footprint");
  this->declare_parameter<double>("vis_scale", 1.0);
  this->declare_parameter<std::string>("chassis_mode_topic", "chassis_mode");
  // 启动时的底盘模式：默认 1=littleTES（导航启动即小陀螺），
  // 之后由 chassis_mode 话题（如 simple_decision）覆盖
  this->declare_parameter<int>("initial_chassis_mode", 1);
  this->declare_parameter<float>("init_spin_speed", 3.14);
  this->declare_parameter<bool>("output_in_chassis_frame", false);
  // 速度指令重发频率（Hz）：控制器 30 Hz 更新，这里以更高频率把同一指令送给 MCU，
  // 只做无损传输（不做限速/滤波），保证 MPPI 的预测与实际执行一致
  this->declare_parameter<double>("publish_frequency", 50.0);
  // 超过该时长没有新指令就下发 0 速度（默认 0.5 s）
  this->declare_parameter<double>("cmd_vel_timeout", 0.5);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("nonrotating_robot_base_frame", nonrotating_robot_base_frame_);
  this->get_parameter("chassis_frame", chassis_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("vis_cmd_vel_topic", vis_cmd_vel_topic_);
  this->get_parameter("vis_scale", vis_scale_);
  this->get_parameter("chassis_mode_topic", chassis_mode_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);
  this->get_parameter("output_in_chassis_frame", output_in_chassis_frame_);
  this->get_parameter("publish_frequency", publish_frequency_);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
  if (publish_frequency_ < 1.0) {
    RCLCPP_WARN(get_logger(), "publish_frequency=%.2f 过小，按 1 Hz 处理", publish_frequency_);
    publish_frequency_ = 1.0;
  }
  if (cmd_vel_timeout_ < 0.05) {
    RCLCPP_WARN(get_logger(), "cmd_vel_timeout=%.3f 过小，按 0.05 s 处理", cmd_vel_timeout_);
    cmd_vel_timeout_ = 0.05;
  }
  RCLCPP_INFO(
    get_logger(), "速度指令重发 %.1f Hz（无损传输），超时 %.2f s 下发 0", publish_frequency_,
    cmd_vel_timeout_);
  // initial_chassis_mode：启动时默认小陀螺（launch 中按 navigation_profile 区分）
  int initial_chassis_mode{1};
  this->get_parameter("initial_chassis_mode", initial_chassis_mode);
  chassis_mode_ = static_cast<uint8_t>(initial_chassis_mode);
}

}  // namespace nonrotating_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nonrotating_vel_transform::NonrotatingVelTransform)
