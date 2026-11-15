/**
 * @file DataHandler.h
 * @brief 模块间交接的数据仓库.
 *
 * 存放配置结构、滤波器状态、里程计与地图点云, 供节点和处理器共用.
 */

#include "point_lio/utilities/common_lib.h"
#include "point_lio/core/parameters.h"

/// 模块间交接的数据仓库: 配置与跨帧输出.
struct DataHandler {
public:
  DataHandler() = default;

  PointLioParams config;
  State filter_state;
  State odom_state;
  double odom_stamp{0.0};
  bool odometry_valid{false};
  PointCloudXYZI::Ptr init_map_cloud;
  FilterOutput filter_output_;
};
