#pragma once

// 白盒测试用的派生封装。
//
// 生产类把内部编排与数据放在 protected 的"接缝"区，测试用派生类把它们提升为
// 公有——与 nav2_mppi_controller 的做法一致（那里叫 OptimizerTester /
// CriticManagerWrapper）。好处是生产头文件里不出现任何测试类名，也不必维护
// friend 名单；代价是封口比 friend 松一点：任何派生类都能碰这些接缝。
//
// 只提升测试真正要用的那部分：方法用 using 声明转公开，数据用引用访问器暴露。

#include "terrain_analysis/per_frame_height_map.hpp"
#include "terrain_analysis/persistent_voxel_map.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <memory>
#include <vector>

namespace terrain_analysis {

  /** @brief 体素地图的测试封装：逐阶段方法与网格数据转为公有。 */
  class TestVoxelMap : public PersistentVoxelMap {
  public:
    using PersistentVoxelMap::addFrame;
    using PersistentVoxelMap::cells;
    using PersistentVoxelMap::elapsedSeconds;
    using PersistentVoxelMap::frameCloud;
    using PersistentVoxelMap::insideReceiveBand;
    using PersistentVoxelMap::keepPoint;
    using PersistentVoxelMap::PersistentVoxelMap;  // 继承构造函数
    using PersistentVoxelMap::rebuild;
    using PersistentVoxelMap::rollover;
    using PersistentVoxelMap::shiftX;
    using PersistentVoxelMap::shiftY;
    using PersistentVoxelMap::timestamp;

    // 下面几个返回引用，供测试注入场景（改雷达位置、塞点云、改时刻）。
    guga_common::Point3d& lidar() noexcept {
      return lidar_;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& frameCloudPtr() noexcept {
      return frame_cloud_;
    }
    double& time() noexcept {
      return time_;
    }
    double& initTime() noexcept {
      return init_time_;
    }
  };

  /** @brief 高度图的测试封装：三段阶段与网格数据转为公有。 */
  class TestHeightMap : public PerFrameHeightMap {
  public:
    using PerFrameHeightMap::aboveGroundFloor;
    using PerFrameHeightMap::abovePenetrationFloor;
    using PerFrameHeightMap::computeHeightMap;
    using PerFrameHeightMap::computePlanarElevation;
    using PerFrameHeightMap::estimateTerrainGround;
    using PerFrameHeightMap::insideOutputBand;
    using PerFrameHeightMap::PerFrameHeightMap;  // 继承构造函数

    // 下面几个返回引用，供测试注入候选高度、检查逐格高程与清空输出。
    std::array<std::vector<double>, PerFrameHeightGrid::NUM>&
    pointElev() noexcept {
      return point_elev_;
    }
    std::array<double, PerFrameHeightGrid::NUM>& voxelElev() noexcept {
      return voxel_elev_;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacleCloudPtr() noexcept {
      return obstacle_cloud_;
    }
  };

}  // namespace terrain_analysis
