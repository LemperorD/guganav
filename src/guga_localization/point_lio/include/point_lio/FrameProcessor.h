#pragma once
#include <functional>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "point_lio/Lidar.h"
#include "point_lio/Synchronizer.h"
#include "point_lio/Filter.h"
#include "point_lio/common_lib.h"

enum class PointLioStage {
  WAITINGFORDATA,
  INITIALIZINGIMU,
  INITIALIZINGMAP,
  TRACKING
};

class FrameProcessor {
public:
  FrameProcessor(Imu& imu, Filter& filter, PointLioStage& stage_,
                 Synchronizer& synchronizer_, Lidar& lidar,
                 MeasureGroup& measurement, PointLioParams& config,
                 MainLoopState& state);
  static PointCloudXYZI::Ptr loadPointcloudFromPcd(
      const std::string& file_path);
  bool syncPackages();
  void initScan();
  void preparePointMeasurements() const;
  bool initMapState(
      std::function<void(const sensor_msgs::msg::PointCloud2&)> publish);
  void publishOdometry(
      const std::function<void(const nav_msgs::msg::Odometry&)>& publish,
      const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
          publish_tf);
  bool prepareFrame(
      std::function<void(const sensor_msgs::msg::PointCloud2&)> publish);
  bool initializeIteration(
      std::function<void(const sensor_msgs::msg::PointCloud2&)> publish);

  template <bool ImuAsInput, typename KF>
  void processFramePoints(
      KF& kf, double& last_time, auto& q,
      const std::function<void(const nav_msgs::msg::Odometry&)>& publish,
      const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
          publish_tf);

  double time_current_{0.0};
  double lidar_end_time_{0.0};
  bool is_first_frame_{true};
  double time_update_last_{0.0};

private:
  Imu& imu_;
  Lidar& lidar_;
  Filter& filter_;
  MeasureGroup& measures_;
  PointLioStage& stage_;
  MainLoopState& state_;
  PointLioParams& config_;
  Synchronizer& synchronizer_;
};
