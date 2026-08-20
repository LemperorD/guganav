/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm
 * implementation. Copyright (C) 2025  Yingjie Huang Licensed under the MIT
 * License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "io/pcd_io.h"
#include "lidar_adapter/custom_mid360_driver.h"
#include "lidar_adapter/generic_pointcloud2.h"
#include "lidar_adapter/livox_custom_msg.h"
#include "lidar_adapter/livox_pointcloud2.h"
#include "lidar_adapter/unitree_lidar.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace small_point_lio {

  SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions& options)
      : Node("small_point_lio", options) {
    std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
    std::string imu_topic = declare_parameter<std::string>("imu_topic");
    std::string lidar_type = declare_parameter<std::string>("lidar_type");
    std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
    std::string base_frame =
        declare_parameter<std::string>("base_frame", "base_footprint");
    // 是否广播 odom->base_frame TF。接入仓库链路时由 sensor_scan_generation
    // 统一发布 odom->base_footprint,此处关闭避免同帧双发布者冲突。
    bool publish_tf = declare_parameter<bool>("publish_tf", true);
    bool save_pcd = declare_parameter<bool>("save_pcd");
    small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
    odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry",
                                                                   1000);
    pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 1000);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    if (save_pcd) {
      pointcloud_mapping = std::make_unique<util::PointcloudMapping>(0.02);
    }
    map_save_trigger = create_service<std_srvs::srv::Trigger>(
        "map_save", [this, save_pcd, lidar_frame](
                        const std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res) {
          if (!save_pcd) {
            res->success = false;
            res->message = "pcd save is disabled";
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                         "pcd save is disabled");
            return;
          }
          res->success = true;
          RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                      "waiting for pcd saving ...");
          auto pointcloud_to_save =
              std::make_shared<std::vector<Eigen::Vector3f>>();
          *pointcloud_to_save = pointcloud_mapping->get_points();
          std::thread([pointcloud_to_save, lidar_frame]() {
            io::pcd::write_pcd(ROOT_DIR + "/pcd/scan.pcd", *pointcloud_to_save);
            RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                        "save pcd success");
          }).detach();
        });
    small_point_lio->set_odometry_callback([this, lidar_frame,
                                            publish_tf](const common::Odometry&
                                                             odometry) {
      last_odometry = odometry;

      builtin_interfaces::msg::Time time_msg;
      time_msg.sec = std::floor(odometry.timestamp);
      time_msg.nanosec = static_cast<uint32_t>(
          (odometry.timestamp - time_msg.sec) * 1e9);

      // LIO 输出即为 odom->lidar_frame 位姿(odom 系,雷达在原点)。
      // 不在此叠加 base 外参:sensor_scan_generation 会自行查询
      // lidar->base_footprint 并合成 odom->base_footprint,避免双重变换。
      nav_msgs::msg::Odometry odometry_msg;
      odometry_msg.header.stamp = time_msg;
      odometry_msg.header.frame_id = "odom";
      odometry_msg.child_frame_id = lidar_frame;
      odometry_msg.pose.pose.position.x = odometry.position.x();
      odometry_msg.pose.pose.position.y = odometry.position.y();
      odometry_msg.pose.pose.position.z = odometry.position.z();
      odometry_msg.pose.pose.orientation.x = odometry.orientation.x();
      odometry_msg.pose.pose.orientation.y = odometry.orientation.y();
      odometry_msg.pose.pose.orientation.z = odometry.orientation.z();
      odometry_msg.pose.pose.orientation.w = odometry.orientation.w();

      if (publish_tf) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = time_msg;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = lidar_frame;
        transform_stamped.transform.translation.x = odometry.position.x();
        transform_stamped.transform.translation.y = odometry.position.y();
        transform_stamped.transform.translation.z = odometry.position.z();
        transform_stamped.transform.rotation.x = odometry.orientation.x();
        transform_stamped.transform.rotation.y = odometry.orientation.y();
        transform_stamped.transform.rotation.z = odometry.orientation.z();
        transform_stamped.transform.rotation.w = odometry.orientation.w();
        tf_broadcaster->sendTransform(transform_stamped);
      }

      odometry_publisher->publish(odometry_msg);
    });
    small_point_lio->set_pointcloud_callback([this, save_pcd,
                                              lidar_frame](const std::vector<
                                                           Eigen::Vector3f>&
                                                               pointcloud) {
      if (pointcloud_publisher->get_subscription_count() > 0) {
        builtin_interfaces::msg::Time time_msg;
        time_msg.sec = std::floor(last_odometry.timestamp);
        time_msg.nanosec = static_cast<uint32_t>(
            (last_odometry.timestamp - time_msg.sec) * 1e9);

        // 点云已由 LIO 输出为 odom 系(雷达原点),不做 base 外参变换,
        // 与 sensor_scan_generation 的 odom->lidar 位姿语义保持一致。
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = time_msg;
        msg.header.frame_id = "odom";
        msg.width = pointcloud.size();
        msg.height = 1;
        msg.fields.reserve(4);
        sensor_msgs::msg::PointField field;
        field.name = "x";
        field.offset = 0;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "y";
        field.offset = 4;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "z";
        field.offset = 8;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        field.name = "intensity";
        field.offset = 12;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
        msg.is_bigendian = false;
        msg.point_step = 16;
        msg.row_step = msg.width * msg.point_step;
        msg.data.resize(msg.row_step * msg.height);
        auto pointer = reinterpret_cast<float*>(msg.data.data());
        for (const auto& point : pointcloud) {
          *pointer = point.x();
          ++pointer;
          *pointer = point.y();
          ++pointer;
          *pointer = point.z();
          ++pointer;
          *pointer = 0;
          ++pointer;
        }
        msg.is_dense = false;
        pointcloud_publisher->publish(msg);
      }
      if (save_pcd) {
        for (const auto& point : pointcloud) {
          pointcloud_mapping->add_point(point);
        }
      }
    });
    if (lidar_type == "livox_custom_msg") {
#ifdef HAVE_LIVOX_DRIVER
      lidar_adapter = std::make_unique<LivoxCustomMsgAdapter>();
#else
      RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                   "livox_custom_msg requested but not available!");
      rclcpp::shutdown();
      return;
#endif
    } else if (lidar_type == "livox_pointcloud2") {
      lidar_adapter = std::make_unique<LivoxPointCloud2Adapter>();
    } else if (lidar_type == "generic_pointcloud2") {
      // 仿真/通用 PointCloud2:只读 x/y/z,不要求 livox 的 tag/timestamp 字段
      lidar_adapter = std::make_unique<GenericPointCloud2Adapter>();
    } else if (lidar_type == "custom_mid360_driver") {
      lidar_adapter = std::make_unique<CustomMid360DriverAdapter>();
    } else if (lidar_type == "unilidar") {
      lidar_adapter = std::make_unique<UnilidarAdapter>();
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknwon lidar type");
      rclcpp::shutdown();
      return;
    }
    lidar_adapter->setup_subscription(
        this, lidar_topic,
        [this](const std::vector<common::Point>& pointcloud) {
          small_point_lio->on_point_cloud_callback(pointcloud);
          small_point_lio->handle_once();
        });
    imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu& msg) {
          common::ImuMsg imu_msg;
          imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x,
                                                     msg.angular_velocity.y,
                                                     msg.angular_velocity.z);
          imu_msg.linear_acceleration = Eigen::Vector3d(
              msg.linear_acceleration.x, msg.linear_acceleration.y,
              msg.linear_acceleration.z);
          imu_msg.timestamp = msg.header.stamp.sec
                              + msg.header.stamp.nanosec * 1e-9;
          small_point_lio->on_imu_callback(imu_msg);
          small_point_lio->handle_once();
        });
  }

}  // namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
