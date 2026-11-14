/**
 * @file FrameProcessor.h
 * @brief 帧调度类声明.
 *
 * 负责一帧的完整流程: 配时间、传播、量测更新、地图增量与结果写回.
 */

#pragma once
#include <functional>
#include <memory>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "point_lio/core/Lidar.h"
#include "point_lio/core/Synchronizer.h"
#include "point_lio/core/common_lib.h"
#include "point_lio/core/DataHandler.h"

enum class PointLioStage {
  WAITINGFORDATA,
  INITIALIZINGIMU,
  INITIALIZINGMAP,
  TRACKING
};

class ManifoldEskf;

class FrameProcessor {
public:
  FrameProcessor(PointLioStage& stage_, DataHandler& data_handler,
                 MainLoopState& state);
  void onLidarFrame(PointCloudXYZI::Ptr points, double timestamp);
  void onImu(const ImuMeasurement& measurement, double timestamp);
  void onLidarFrames(std::deque<PointCloudXYZI::Ptr>& frames,
                     std::deque<double>& timestamps);
  ~FrameProcessor();
  void initialize();
  void setPose(geometry_msgs::msg::Pose& pose) const;
  void pointBodyLidarToIMU(const PointType* pi, PointType* po) const;
  void configure(double lidar_time_interval);
  /// 处理一帧; 返回 true 表示本帧确实被处理 (据此决定是否发布输出)
  [[nodiscard]] bool processIteration();
  [[nodiscard]] double lidarEndTime() const {
    return lidar_end_time_;
  }
  [[nodiscard]] const PointCloudXYZI::Ptr& currentLidarOrigin() const;
  /// 当前帧降采样后的世界系点云, 供发布与 PCD 保存使用
  [[nodiscard]] const PointCloudXYZI::Ptr& currentDownsampledWorld() const;

private:
  double time_current_{0.0};
  double lidar_end_time_{0.0};
  bool is_first_frame_{true};
  double time_update_last_{0.0};
  double last_time_output_{0.0};
  int startup_frames_{0};
  DataHandler& data_handler_;

  static PointCloudXYZI::Ptr loadPointcloudFromPcd(
      const std::string& file_path);
  void initializeFilter();
  bool syncPackages();
  void initScan();
  void mapIncremental() const;
  bool initMapState();
  void publishOdometry();
  bool prepareFrame();
  bool initializeIteration();

  void processFramePoints(MeasureGroup& meas, double& last_time);

  Imu imu_;
  Lidar lidar_;
  std::unique_ptr<ManifoldEskf> manifold_;
  MeasureGroup measures_;
  FilterOutput& output_ = data_handler_.getFilterOutput();
  PointLioStage& stage_;
  MainLoopState& state_;
  const PointLioParams& config_ = data_handler_.config;
  Synchronizer synchronizer_;

  sensor_msgs::msg::Imu last;
  sensor_msgs::msg::Imu next;
};
