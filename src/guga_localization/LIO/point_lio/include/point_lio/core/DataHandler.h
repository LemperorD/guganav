/**
 * @file DataHandler.h
 * @brief 模块间交接的数据仓库.
 *
 * 存放配置结构、滤波器状态、里程计与地图点云, 供节点和处理器共用.
 */

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "point_lio/core/common_lib.h"
#include "point_lio/core/parameters.h"

/// 模块间交接的数据仓库: 配置与跨帧输出.
class DataHandler {
public:
  DataHandler() = default;

  PointLioParams config;
  State filter_state;
  nav_msgs::msg::Odometry odometry;
  bool odometry_valid{false};
  sensor_msgs::msg::PointCloud2 map_cloud;
  FilterOutput& getFilterOutput() {
    return filter_output_;
  }

private:
  FilterOutput filter_output_;
};
