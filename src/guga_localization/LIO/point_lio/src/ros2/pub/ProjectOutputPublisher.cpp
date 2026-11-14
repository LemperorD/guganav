#include "point_lio/ros2/pub/ProjectOutputPublisher.h"

#include <pcl_conversions/pcl_conversions.h>

void ProjectOutputPublisher::configure(rclcpp::Node* node,
                                       const OutputFrameParams& out,
                                       OdomProcessor* projector) {
  out_ = &out;
  projector_ = projector;
  scan_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      out.registered_scan_topic, 5);
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
      out.lidar_odometry_topic, 5);
}

bool ProjectOutputPublisher::passesZFilter(const Eigen::Vector3d& point) const {
  if (!out_->z_filter.enabled) {
    return true;
  }
  const double z = projector_->apply(point).z();
  return z >= out_->z_filter.min_z && z <= out_->z_filter.max_z;
}

void ProjectOutputPublisher::publish(const PointCloudXYZI& world_cloud,
                                     const nav_msgs::msg::Odometry& odometry,
                                     bool odometry_valid,
                                     const rclcpp::Time& stamp) {
  if (scan_pub_) {
    // 与 loam_interface 一致: 整幅世界系点云左乘安装变换后标 odom
    PointCloudXYZI cloud_odom;
    cloud_odom.reserve(world_cloud.size());
    for (const auto& p : world_cloud.points) {
      const Eigen::Vector3d pw(p.x, p.y, p.z);
      if (!passesZFilter(pw)) {
        continue;
      }
      const Eigen::Vector3d po = projector_->apply(pw);
      PointType q = p;
      q.x = static_cast<float>(po.x());
      q.y = static_cast<float>(po.y());
      q.z = static_cast<float>(po.z());
      cloud_odom.points.emplace_back(q);
    }
    cloud_odom.width = cloud_odom.points.size();
    cloud_odom.height = 1;
    cloud_odom.is_dense = false;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud_odom, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = out_->odom_frame;
    scan_pub_->publish(msg);
  }

  if (odom_pub_ && odometry_valid) {
    nav_msgs::msg::Odometry msg = odometry;
    const auto& pose = odometry.pose.pose;
    const Eigen::Vector3d p_lio(pose.position.x, pose.position.y,
                                pose.position.z);
    const Eigen::Quaterniond q_lio(pose.orientation.w, pose.orientation.x,
                                   pose.orientation.y, pose.orientation.z);
    const Eigen::Vector3d p_odom = projector_->apply(p_lio);
    const Eigen::Quaterniond q_odom(projector_->rotation()
                                    * q_lio.toRotationMatrix());
    msg.pose.pose.position.x = p_odom.x();
    msg.pose.pose.position.y = p_odom.y();
    msg.pose.pose.position.z = p_odom.z();
    msg.pose.pose.orientation.x = q_odom.x();
    msg.pose.pose.orientation.y = q_odom.y();
    msg.pose.pose.orientation.z = q_odom.z();
    msg.pose.pose.orientation.w = q_odom.w();
    msg.header.frame_id = out_->odom_frame;
    msg.child_frame_id = out_->lidar_frame;
    odom_pub_->publish(msg);
  }
}
