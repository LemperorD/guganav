#include "point_lio/ros2/sub/SensorSubscriptions.h"

void SensorSubscriptions::configure(rclcpp::Node* node,
                                    const PointLioParams& config,
                                    FrameProcessor* processor,
                                    Preprocess* preprocess,
                                    rclcpp::CallbackGroup::SharedPtr group) {
  node_ = node;
  config_ = &config;
  processor_ = processor;
  preprocessor_ = preprocess;

  rclcpp::SubscriptionOptions options;
  options.callback_group = group;

  if (config.lidar.lidar_type == AVIA) {
    livox_sub_ = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        config.sensor.lidar_topic, rclcpp::SensorDataQoS(),
        [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
          onLivox(msg);
        },
        options);
  } else {
    pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        config.sensor.lidar_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          onPointCloud(msg);
        },
        options);
  }

  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
      config.sensor.imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { onImu(msg); },
      options);
}

void SensorSubscriptions::onLivox(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
  ++scan_count_;
  auto points = std::make_shared<PointCloudXYZI>(10000, 1);
  preprocessor_->process(msg, points);
  processor_->onLidarFrame(std::move(points),
                           rclcpp::Time(msg->header.stamp).seconds());
}

void SensorSubscriptions::onPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  ++scan_count_;
  auto points = std::make_shared<PointCloudXYZI>(20000, 1);
  preprocessor_->process(msg, points);
  processor_->onLidarFrame(std::move(points),
                           rclcpp::Time(msg->header.stamp).seconds());
}

void SensorSubscriptions::onImu(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  processor_->onImu(preprocessor_->processImu(msg),
                    rclcpp::Time(msg->header.stamp).seconds());
}
