#include "point_lio/FrameProcessor.h"
#include "point_lio/Synchronizer.h"

FrameProcessor::FrameProcessor(Imu& imu, Filter& filter, PointLioStage& stage,
                               Synchronizer& synchronizer, Lidar& lidar,
                               MeasureGroup& measurement, PointLioParams& config)
    : imu_(imu),
      filter_(filter),
      stage_(stage),
      synchronizer_(synchronizer),
      lidar_(lidar),
      measures_(measurement),
      config_(config) {
}

PointCloudXYZI::Ptr FrameProcessor::loadPointcloudFromPcd(
    const std::string& file_path) {
  auto pcd_ptr = std::make_shared<PointCloudXYZI>();

  if (pcl::io::loadPCDFile(file_path, *pcd_ptr) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("laserMapping"),
                 "Couldn't read pcd file %s", file_path.c_str());
    return nullptr;
  }

  RCLCPP_INFO(rclcpp::get_logger("laserMapping"), "Loaded %zu points from %s",
              pcd_ptr->size(), file_path.c_str());
  return pcd_ptr;
}

void FrameProcessor::initScan() {
  const auto& imu_next = imu_.next();
  std::cout << "first imu time: " << get_time_sec(imu_next.header.stamp)
            << '\n';
  time_current_ = 0.0;

  if (config_.imu.processor.enabled) {
    filter_.input().x_.gravity = config_.imu.processor.gravity;
    filter_.output().x_.gravity = config_.imu.processor.gravity;
    imu_.discardBefore(measures_.lidar_start_time);
  } else {
    filter_.input().x_.gravity = config_.imu.processor.gravity;
    filter_.output().x_.gravity = config_.imu.processor.gravity;
    filter_.output().x_.acc = config_.imu.processor.gravity;
    filter_.output().x_.acc *= -1;
  }
}

bool FrameProcessor::syncPackages() {
  return synchronizer_.syncPackages(lidar_, imu_, measures_);
}
