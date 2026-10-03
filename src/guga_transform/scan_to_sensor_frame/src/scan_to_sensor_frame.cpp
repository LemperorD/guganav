#include "scan_to_sensor_frame/scan_to_sensor_frame.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>

namespace scan_to_sensor_frame
{

ScanToSensorFrameNode::ScanToSensorFrameNode(const rclcpp::NodeOptions & options)
: Node("scan_to_sensor_frame", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<double>("min_odometry_dt", 1e-3);
  this->declare_parameter<double>("max_linear_velocity", 10.0);
  this->declare_parameter<double>("max_angular_velocity", 20.0);

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("min_odometry_dt", min_odometry_dt_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);
  pub_robot_base_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("robot_base_odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &ScanToSensorFrameNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));

  // 启动四个线程执行本功能包的四个并行任务
  sensor_scan_thread_ = std::thread(std::bind(&ScanToSensorFrameNode::updateSensorScan, this));
  chassis_odom_thread_ = std::thread(std::bind(&ScanToSensorFrameNode::updateChassisOdometry, this));
  robot_base_odom_thread_ = std::thread(std::bind(&ScanToSensorFrameNode::updateRobotBaseOdometry, this));
  chassis_tf_thread_ = std::thread(std::bind(&ScanToSensorFrameNode::updateChassisTF, this));
}

// 析构函数,释放线程资源
ScanToSensorFrameNode::~ScanToSensorFrameNode() {
  stopping_.store(true);
  chassis_tf_cv_.notify_all();
  chassis_odom_cv_.notify_all();
  robot_base_odom_cv_.notify_all();
  sensor_scan_cv_.notify_all();
  if (chassis_tf_thread_.joinable()) {
    chassis_tf_thread_.join();
  }
  if (chassis_odom_thread_.joinable()) {
    chassis_odom_thread_.join();
  }
  if (robot_base_odom_thread_.joinable()) {
    robot_base_odom_thread_.join();
  }
  if (sensor_scan_thread_.joinable()) {
    sensor_scan_thread_.join();
  }
}

void ScanToSensorFrameNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
#ifdef TEST_TIME
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
#endif

  tf2::Transform odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, odom_to_lidar);

  tf2::Transform lidar_to_robot_base;
  tf2::Transform lidar_to_chassis;
  const bool has_robot_base_transform = getTransform(
    lidar_frame_, robot_base_frame_, pcd_msg->header.stamp, lidar_to_robot_base);
  const bool has_chassis_transform = getTransform(
    lidar_frame_, base_frame_, pcd_msg->header.stamp, lidar_to_chassis);
  if (!has_robot_base_transform || !has_chassis_transform) {
    // 单位阵回退会在车体旋转时把点云投到错误位置，因此丢弃不完整帧。
    return;
  }

  const auto odom_to_chassis = odom_to_lidar * lidar_to_chassis;
  const auto odom_to_robot_base = odom_to_lidar * lidar_to_robot_base;
  const auto & odom_frame = odometry_msg->header.frame_id;
  const auto & stamp = pcd_msg->header.stamp;

  {
    std::lock_guard<std::mutex> lock(chassis_tf_mutex_);
    chassis_tf_work_ = {odom_to_chassis, odom_frame, stamp};
    chassis_tf_ready_ = true;
  }
  chassis_tf_cv_.notify_one();

  {
    std::lock_guard<std::mutex> lock(chassis_odom_mutex_);
    chassis_odom_work_ = {odom_to_chassis, odom_frame, stamp};
    chassis_odom_ready_ = true;
  }
  chassis_odom_cv_.notify_one();

  {
    std::lock_guard<std::mutex> lock(robot_base_odom_mutex_);
    robot_base_odom_work_ = {odom_to_robot_base, odom_frame, stamp};
    robot_base_odom_ready_ = true;
  }
  robot_base_odom_cv_.notify_one();

  {
    std::lock_guard<std::mutex> lock(sensor_scan_mutex_);
    sensor_scan_work_ = {*pcd_msg, odom_to_lidar};
    sensor_scan_ready_ = true;
  }
  sensor_scan_cv_.notify_one();

#ifdef TEST_TIME
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> duration = end_time - start_time;
  std::cout << CYAN_LIGHT <<"laserCloudAndOdometryHandler duration: " << duration.count() << " ms" << RESET << std::endl;
#endif
}

bool ScanToSensorFrameNode::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const rclcpp::Time & time, tf2::Transform & transform)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(transform_stamped.transform, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF lookup failed: %s. Dropping synchronized scan.", ex.what());
    return false;
  }
}

void ScanToSensorFrameNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void ScanToSensorFrameNode::publishChassisOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ptr)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (has_previous_chassis_odometry_) {
    const double dt = (stamp - previous_chassis_odometry_stamp_).seconds();
    if (dt > min_odometry_dt_) {
      const auto linear_velocity =
        (transform.getOrigin() - previous_chassis_odometry_transform_.getOrigin()) / dt;

      tf2::Quaternion q_diff =
        transform.getRotation() * previous_chassis_odometry_transform_.getRotation().inverse();
      q_diff.normalize();
      const double angle = std::remainder(q_diff.getAngle(), 2.0 * M_PI);
      const auto angular_velocity = q_diff.getAxis() * angle / dt;

      const double linear_speed = linear_velocity.length();
      const double angular_speed = angular_velocity.length();
      if (
        std::isfinite(linear_speed) && std::isfinite(angular_speed) &&
        linear_speed <= max_linear_velocity_ && angular_speed <= max_angular_velocity_) {
        out.twist.twist.linear.x = linear_velocity.x();
        out.twist.twist.linear.y = linear_velocity.y();
        out.twist.twist.linear.z = linear_velocity.z();
        out.twist.twist.angular.x = angular_velocity.x();
        out.twist.twist.angular.y = angular_velocity.y();
        out.twist.twist.angular.z = angular_velocity.z();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Discard odometry twist spike: dt=%.6f, linear=%.3f, angular=%.3f",
          dt, linear_speed, angular_speed);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip odometry twist update because dt is too small or non-positive: %.6f", dt);
    }
  }

  previous_chassis_odometry_transform_ = transform;
  previous_chassis_odometry_stamp_ = stamp;
  has_previous_chassis_odometry_ = true;

  odom_pub_ptr->publish(out);
}

void ScanToSensorFrameNode::publishRobotBaseOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ptr)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (has_previous_robot_base_odometry_) {
    const double dt = (stamp - previous_robot_base_odometry_stamp_).seconds();
    if (dt > min_odometry_dt_) {
      const auto linear_velocity =
        (transform.getOrigin() - previous_robot_base_odometry_transform_.getOrigin()) / dt;

      tf2::Quaternion q_diff =
        transform.getRotation() * previous_robot_base_odometry_transform_.getRotation().inverse();
      q_diff.normalize();
      const double angle = std::remainder(q_diff.getAngle(), 2.0 * M_PI);
      const auto angular_velocity = q_diff.getAxis() * angle / dt;

      const double linear_speed = linear_velocity.length();
      const double angular_speed = angular_velocity.length();
      if (
        std::isfinite(linear_speed) && std::isfinite(angular_speed) &&
        linear_speed <= max_linear_velocity_ && angular_speed <= max_angular_velocity_) {
        out.twist.twist.linear.x = linear_velocity.x();
        out.twist.twist.linear.y = linear_velocity.y();
        out.twist.twist.linear.z = linear_velocity.z();
        out.twist.twist.angular.x = angular_velocity.x();
        out.twist.twist.angular.y = angular_velocity.y();
        out.twist.twist.angular.z = angular_velocity.z();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Discard odometry twist spike: dt=%.6f, linear=%.3f, angular=%.3f",
          dt, linear_speed, angular_speed);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip odometry twist update because dt is too small or non-positive: %.6f", dt);
    }
  }

  previous_robot_base_odometry_transform_ = transform;
  previous_robot_base_odometry_stamp_ = stamp;
  has_previous_robot_base_odometry_ = true;

  odom_pub_ptr->publish(out);
}

void ScanToSensorFrameNode::updateChassisTF() {
  while (rclcpp::ok() && !stopping_.load()) {
    TransformWork work;
    {
      std::unique_lock<std::mutex> lock(chassis_tf_mutex_);
      chassis_tf_cv_.wait(lock, [this]() {
        return chassis_tf_ready_ || stopping_.load() || !rclcpp::ok();
      });
      if (!rclcpp::ok() || stopping_.load()) { return; }
      work = chassis_tf_work_;
      chassis_tf_ready_ = false;
    }
    publishTransform(work.transform, work.parent_frame, base_frame_, work.stamp);
  }
}

void ScanToSensorFrameNode::updateChassisOdometry() {
  while (rclcpp::ok() && !stopping_.load()) {
    TransformWork work;
    {
      std::unique_lock<std::mutex> lock(chassis_odom_mutex_);
      chassis_odom_cv_.wait(lock, [this]() {
        return chassis_odom_ready_ || stopping_.load() || !rclcpp::ok();
      });
      if (!rclcpp::ok() || stopping_.load()) { return; }
      work = chassis_odom_work_;
      chassis_odom_ready_ = false;
    }
    publishChassisOdometry(
      work.transform, work.parent_frame, base_frame_, work.stamp, pub_chassis_odometry_);
  }
}

void ScanToSensorFrameNode::updateRobotBaseOdometry() {
  while (rclcpp::ok() && !stopping_.load()) {
    TransformWork work;
    {
      std::unique_lock<std::mutex> lock(robot_base_odom_mutex_);
      robot_base_odom_cv_.wait(lock, [this]() {
        return robot_base_odom_ready_ || stopping_.load() || !rclcpp::ok();
      });
      if (!rclcpp::ok() || stopping_.load()) { return; }
      work = robot_base_odom_work_;
      robot_base_odom_ready_ = false;
    }
    publishRobotBaseOdometry(
      work.transform, work.parent_frame, robot_base_frame_, work.stamp,
      pub_robot_base_odometry_);
  }
}

void ScanToSensorFrameNode::updateSensorScan() {
  while (rclcpp::ok() && !stopping_.load()) {
    SensorScanWork work;
    {
      std::unique_lock<std::mutex> lock(sensor_scan_mutex_);
      sensor_scan_cv_.wait(lock, [this]() {
        return sensor_scan_ready_ || stopping_.load() || !rclcpp::ok();
      });
      if (!rclcpp::ok() || stopping_.load()) { return; }
      work = sensor_scan_work_;
      sensor_scan_ready_ = false;
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl_ros::transformPointCloud(
      lidar_frame_, work.odom_to_lidar.inverse(), work.cloud, output);
    pub_laser_cloud_->publish(output);
  }
}

} // namespace scan_to_sensor_frame

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(scan_to_sensor_frame::ScanToSensorFrameNode)
