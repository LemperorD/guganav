#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/context.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

class ConnectivityTest : public testing::Test {
protected:
  ConnectivityTest() : cfg_(DefaultTerrainConfig()) {
    resetState();
    cfg_.check_terrain_connectivity = true;
    cfg_.terrain_connectivity_threshold = 0.5;
    cfg_.ceiling_filter_threshold = 2.0;
    cfg_.terrain_under_vehicle = -0.75;
  }

  void resetState() {
    state_ = {};
    for (auto& ptr : state_.terrain_voxel_cloud) {
      ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }
    state_.laser_cloud_crop =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.terrain_cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.terrain_cloud_elev =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.terrain_cloud_local =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.laser_cloud_downsampled =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.laser_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  TerrainConfig cfg_;
  TerrainState state_;
};

// ── checkTerrainConnectivity ──

// 车辆正下方 cell 无数据时，用 vehicleZ + terrain_under_vehicle 作为地面高度
TEST_F(ConnectivityTest, CheckTerrainConnectivity_SeedsVehicleCell) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.planar_voxel_elev.fill(0);
  state_.planar_point_elev.fill({});

  TerrainAlgorithm::checkTerrainConnectivity(cfg_, state_);

  // stub — no assertion yet
}

// 高度差在 connectivity_threshold 内的相邻 cell 标记为连通
TEST_F(ConnectivityTest,
       CheckTerrainConnectivity_SimilarHeight_Connected) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.planar_voxel_elev.fill(0);
  state_.planar_point_elev.fill({0.0F});

  int center = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_point_elev[center] = {0.0F};
  int neighbor = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH + 1,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_point_elev[neighbor] = {0.3F};  // diff 0.3 < 0.5 threshold

  TerrainAlgorithm::checkTerrainConnectivity(cfg_, state_);

  // stub
}

// 高度差超过 ceiling_filter_threshold 的 cell 标记为天花板/噪声
TEST_F(ConnectivityTest,
       CheckTerrainConnectivity_LargeHeightDiff_NotConnected) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.planar_voxel_elev.fill(0);
  state_.planar_point_elev.fill({});

  int center = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_point_elev[center] = {0.0F};
  int neighbor = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH + 1,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_point_elev[neighbor] = {5.0F};  // diff 5.0 > 2.0 ceiling

  TerrainAlgorithm::checkTerrainConnectivity(cfg_, state_);

  // stub
}

// 关闭 connectivity 检查时不执行任何操作
TEST_F(ConnectivityTest,
       CheckTerrainConnectivity_Disabled_NoEffect) {
  cfg_.check_terrain_connectivity = false;
  // No assertion needed — just ensures no crash
  TerrainAlgorithm::checkTerrainConnectivity(cfg_, state_);
}

// ── mergeLocalTerrain ──

// local terrain map 半径内的点被合并到输出点云
TEST_F(ConnectivityTest,
       MergeLocalTerrain_WithinRadius_PointsMerged) {
  cfg_.local_terrain_map_radius = 4.0;
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud_elev->clear();
  state_.terrain_cloud_local->clear();
  state_.terrain_cloud_local->push_back({1.0F, 0, 0, 0});  // distance 1.0 < 4.0

  TerrainAlgorithm::mergeLocalTerrain(cfg_, state_);

  // stub
}

// 超出半径的点不合并
TEST_F(ConnectivityTest,
       MergeLocalTerrain_BeyondRadius_Excluded) {
  cfg_.local_terrain_map_radius = 4.0;
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud_elev->clear();
  state_.terrain_cloud_local->clear();
  state_.terrain_cloud_local->push_back(
      {10.0F, 0, 0, 0});  // distance 10.0 > 4.0

  TerrainAlgorithm::mergeLocalTerrain(cfg_, state_);

  // stub — no points should be added
}

// radius 为 0 时不合并任何点（默认禁用）
TEST_F(ConnectivityTest,
       MergeLocalTerrain_ZeroRadius_NoMerge) {
  cfg_.local_terrain_map_radius = 0.0;
  state_.terrain_cloud_elev->clear();
  state_.terrain_cloud_local->clear();
  state_.terrain_cloud_local->push_back({1.0F, 0, 0, 0});

  TerrainAlgorithm::mergeLocalTerrain(cfg_, state_);

  // stub
}
