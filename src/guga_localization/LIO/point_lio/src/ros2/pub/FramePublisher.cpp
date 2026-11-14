#include "point_lio/ros2/pub/FramePublisher.h"

#include <pcl_conversions/pcl_conversions.h>

void FramePublisher::configure(rclcpp::Node* node,
                               const PublishParams& params) {
  params_ = &params;
  pub_world_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_registered", 20);
  pub_map_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map",
                                                                   20);
}

void FramePublisher::publishWorld(const PointCloudXYZI& cloud,
                                  const rclcpp::Time& stamp) const {
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = "camera_init";
  pub_world_->publish(msg);
}

void FramePublisher::publishMap(
    const sensor_msgs::msg::PointCloud2& msg) const {
  pub_map_->publish(msg);
}
