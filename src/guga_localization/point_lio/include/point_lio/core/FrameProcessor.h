#pragma once
#include <functional>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "point_lio/core/Lidar.h"
#include "point_lio/core/Synchronizer.h"
#include "point_lio/core/Filter.h"
#include "point_lio/core/common_lib.h"
#include "point_lio/core/ProcessingState.h"
#include "point_lio/core/Measurement.h"

enum class PointLioStage {
  WAITINGFORDATA,
  INITIALIZINGIMU,
  INITIALIZINGMAP,
  TRACKING
};

class FrameProcessor {
public:
  FrameProcessor(Imu& imu, PointLioStage& stage_, Lidar& lidar,
                 const PointLioParams& config, MainLoopState& state);
  void initialize();
  void setPose(geometry_msgs::msg::Pose& pose) const;
  void pointBodyLidarToIMU(const PointType* pi, PointType* po) const;
  void configureSynchronizer(double lidar_time_interval);
  /// 处理一帧; 返回 true 表示本帧确实被处理 (据此决定是否发布输出)
  [[nodiscard]] bool processIteration(
      const std::function<void(const sensor_msgs::msg::PointCloud2&)>&
          publish_map,
      const std::function<void(const nav_msgs::msg::Odometry&)>& publish_odom,
      const std::function<void(const geometry_msgs::msg::TransformStamped&)>&
          publish_tf);
  [[nodiscard]] double lidarEndTime() const {
    return lidar_end_time_;
  }

private:
  double time_current_{0.0};
  double lidar_end_time_{0.0};
  bool is_first_frame_{true};
  double time_update_last_{0.0};
  double last_time_input_{0.0};
  double last_time_output_{0.0};
  input_ikfom input_in_;

  static PointCloudXYZI::Ptr loadPointcloudFromPcd(
      const std::string& file_path);
  void initializeFilter();
  bool syncPackages();
  void initScan();
  void preparePointMeasurements() const;
  void mapIncremental() const;
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

  Imu& imu_;
  Lidar& lidar_;
  Filter filter_;
  MeasureGroup measures_;
  PointLioStage& stage_;
  MainLoopState& state_;
  const PointLioParams& config_;
  Synchronizer synchronizer_;
};
