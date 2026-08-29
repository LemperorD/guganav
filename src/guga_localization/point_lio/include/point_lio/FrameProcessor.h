#pragma once
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
                 MeasureGroup& measurement, PointLioParams& config);
  static PointCloudXYZI::Ptr loadPointcloudFromPcd(
      const std::string& file_path);
  bool syncPackages();
  void initScan();
  double time_current_{0.0};

private:
  struct MainLoopState {
    int sleep_time = 0;  ///< 等待计数

    // ---- 工作缓存 (滤波器 / 消息 / 点云) ----
    pcl::VoxelGrid<PointType> downsize_filter_surf;  ///< 配准后降采样
    nav_msgs::msg::Path path;                        ///< 轨迹消息
    nav_msgs::msg::Odometry odom_aft_mapped;         ///< 里程计消息
    geometry_msgs::msg::PoseStamped msg_body_pose;   ///< 位姿消息
    PointCloudXYZI::Ptr feats_undistort =
        std::make_shared<PointCloudXYZI>();  ///< 去畸变特征点云
    PointCloudXYZI::Ptr init_feats_world =
        std::make_shared<PointCloudXYZI>();  ///< 初始化世界系点云
    PointCloudXYZI::Ptr pcl_wait_save =
        std::make_shared<PointCloudXYZI>();  ///< 待保存点云

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  double lidar_end_time_{0.0};

  Imu& imu_;
  Lidar& lidar_;
  Filter& filter_;
  MeasureGroup& measures_;
  PointLioStage& stage_;
  MainLoopState state_;  ///< 主循环状态
  PointLioParams& config_;
  Synchronizer& synchronizer_;
};
