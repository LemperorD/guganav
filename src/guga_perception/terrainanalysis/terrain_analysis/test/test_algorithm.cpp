#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/context.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

class AlgorithmTest : public testing::Test {
protected:
  AlgorithmTest() : cfg_(DefaultTerrainConfig()) {
    resetState();
    cfg_.use_sorting = true;
    cfg_.quantile_z = 0.25;
    cfg_.limit_ground_lift = false;
  }

  void resetState() {
    state_ = {};
    for (auto& ptr : state_.terrain_voxel_cloud) {
      ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }
    state_.laser_cloud_crop =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.terrain_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.terrain_cloud_elev =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.laser_cloud_downsampled =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    state_.laser_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  TerrainConfig cfg_;
  TerrainState state_;
};

// 车辆未移动时，体素网格不发生滚动
TEST_F(AlgorithmTest, RolloverVoxels_Stationary_NoShift) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  int sx = state_.terrain_voxel_shift_x;
  int sy = state_.terrain_voxel_shift_y;

  TerrainAlgorithm::rolloverVoxels(cfg_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_x, sx);
  EXPECT_EQ(state_.terrain_voxel_shift_y, sy);
}

// 车辆向左超出 voxel 范围时，沿 X 负向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_LeftOfCenter_ShiftsXNegative) {
  state_.vehicle_x = -2.0;
  int sx = state_.terrain_voxel_shift_x;

  TerrainAlgorithm::rolloverVoxels(cfg_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_x, sx - 1);
}

// 车辆向右超出 voxel 范围时，沿 X 正向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_RightOfCenter_ShiftsXPositive) {
  state_.vehicle_x = 2.0;
  int sx = state_.terrain_voxel_shift_x;

  TerrainAlgorithm::rolloverVoxels(cfg_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_x, sx + 1);
}

// 车辆向下超出 voxel 范围时，沿 Y 负向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_BelowCenter_ShiftsYNegative) {
  state_.vehicle_y = -2.0;
  int sy = state_.terrain_voxel_shift_y;

  TerrainAlgorithm::rolloverVoxels(cfg_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_y, sy - 1);
}

// 车辆向上超出 voxel 范围时，沿 Y 正向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_AboveCenter_ShiftsYPositive) {
  state_.vehicle_y = 2.0;
  int sy = state_.terrain_voxel_shift_y;

  TerrainAlgorithm::rolloverVoxels(cfg_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_y, sy + 1);
}

// 滚动后目标 cell 被清空，原有数据随 shift 迁移
TEST_F(AlgorithmTest, RolloverVoxels_ShiftLeft_PreservesDataFromShiftedCell) {
  state_.vehicle_x = -2.0;
  state_.terrain_voxel_cloud[0]->clear();
  pcl::PointXYZI p{0, 0, 0, 0};
  state_.terrain_voxel_cloud[0]->push_back(p);

  TerrainAlgorithm::rolloverVoxels(cfg_, state_);

  // After shift-left, voxel(0,0) becomes the destination cell and gets cleared
  EXPECT_TRUE(state_.terrain_voxel_cloud[0]->points.empty());
}

// ── voxelize ──
// 原点处的单个点被分配到网格正中的 cell
TEST_F(AlgorithmTest, Voxelize_MapsPointToCenterCell) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.laser_cloud_crop->clear();
  state_.laser_cloud_crop->push_back({0, 0, 0, 0});

  TerrainAlgorithm::voxelize(cfg_, state_);

  size_t center = TerrainConfig::terrainVoxelIndex(
      TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH);
  EXPECT_EQ(state_.terrain_voxel_cloud[center]->points.size(), 1U);
  EXPECT_EQ(state_.terrain_voxel_update_num[center], 1);
}

// 空点云不产生任何体素分配
TEST_F(AlgorithmTest, Voxelize_EmptyCloud_NoChange) {
  state_.laser_cloud_crop->clear();

  TerrainAlgorithm::voxelize(cfg_, state_);

  for (int i = 0; i < TerrainConfig::TERRAIN_VOXEL_NUM; i++) {
    EXPECT_EQ(state_.terrain_voxel_update_num[i], 0);
  }
}

// ── computeElevation ──
// 排序模式下取指定分位数作为地面高度估计
TEST_F(AlgorithmTest, ComputeElevation_UseSorting_ReturnsQuantile) {
  cfg_.use_sorting = true;
  cfg_.quantile_z = 0.5;
  size_t cell = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_elev.fill(999);
  state_.planar_point_elev[cell] = {0.1, 0.5, 0.3, 0.2, 0.4};

  TerrainAlgorithm::computeElevation(cfg_, state_);

  // sorted: 0.1, 0.2, 0.3, 0.4, 0.5. quantile 0.5*(5) = 2 → index 2 → 0.3
  EXPECT_FLOAT_EQ(state_.planar_voxel_elev[cell], 0.3F);
}

// 最小值模式下取最低点作为地面高度估计
TEST_F(AlgorithmTest, ComputeElevation_UseMinimum_ReturnsMinimum) {
  cfg_.use_sorting = false;
  size_t cell = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_elev.fill(999);
  state_.planar_point_elev[cell] = {1.5, 0.5, 1.0};

  TerrainAlgorithm::computeElevation(cfg_, state_);

  EXPECT_FLOAT_EQ(state_.planar_voxel_elev[cell], 0.5F);
}

// 分位数与最小值差距过大时，限制地面高度不超过 min+max_ground_lift
TEST_F(AlgorithmTest,
       ComputeElevation_LiftLimited_CapsAtMinimumPlusMaxGroundLift) {
  cfg_.use_sorting = true;
  cfg_.quantile_z = 0.5;
  cfg_.limit_ground_lift = true;
  cfg_.max_ground_lift = 0.3;
  size_t cell = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_elev.fill(999);
  // sorted: 0.5, 1.0, 2.0. quantile 0.5*3 = 1 → 1.0. diff 1.0-0.5=0.5 > 0.3
  state_.planar_point_elev[cell] = {0.5, 2.0, 1.0};

  TerrainAlgorithm::computeElevation(cfg_, state_);

  // lift limited → 0.5 + 0.3 = 0.8
  EXPECT_FLOAT_EQ(state_.planar_voxel_elev[cell], 0.8F);
}

// ── detectDynamicObstacles ──
TEST_F(AlgorithmTest, DetectDynamicObstacles_NearPoint_AddsMinPointNumToCell) {
  cfg_.clear_dy_obs = true;
  cfg_.min_dy_obs_distance = 5.0;  // high → all points "close"
  cfg_.min_dy_obs_point_num = 7;
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.planar_voxel_dy_obs.fill(0);
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back({0.1F, 0, 0, 0});

  TerrainAlgorithm::detectDynamicObstacles(cfg_, state_);

  int total = 0;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    total += state_.planar_voxel_dy_obs[i];
  }
  EXPECT_GT(total, 0);
}

// clear_dy_obs 标志为 true 时仍然处理点云
TEST_F(AlgorithmTest,
       DetectDynamicObstacles_ClearInitEnabled_StillProcessesPoints) {
  cfg_.clear_dy_obs = true;
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.planar_voxel_dy_obs.fill(0);
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back({0.1F, 0, 0, 0});

  TerrainAlgorithm::detectDynamicObstacles(cfg_, state_);

  int total = 0;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    total += state_.planar_voxel_dy_obs[i];
  }
  // detects even when init flag is true — it always processes
  EXPECT_GT(total, 0);
}

// ── filterDynamicObstaclePoints ──
// 高角度点（头顶悬挂物）清零对应 cell 的动态障碍计数
TEST_F(AlgorithmTest,
       FilterDynamicObstaclePoints_HighAnglePoint_ResetsCellCounter) {
  cfg_.clear_dy_obs = true;
  cfg_.min_dy_obs_angle = 10.0 * M_PI / 180.0;
  cfg_.min_dy_obs_relative_z = -0.5;
  size_t cell = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_dy_obs[cell] = 10;
  state_.laser_cloud_crop->clear();
  // high relative_z → angle close to 90° > 10°
  state_.laser_cloud_crop->push_back({0.05F, 0, 2.0F, 0});

  TerrainAlgorithm::filterDynamicObstaclePoints(cfg_, state_);

  EXPECT_EQ(state_.planar_voxel_dy_obs[cell], 0);
}

// 低角度点（地面/低障碍）保持 cell 计数不变
TEST_F(AlgorithmTest,
       FilterDynamicObstaclePoints_LowAnglePoint_KeepsCellCounter) {
  cfg_.clear_dy_obs = true;
  cfg_.min_dy_obs_angle = 90.0 * M_PI / 180.0;  // nearly impossible to exceed
  cfg_.min_dy_obs_relative_z = -0.5;
  size_t cell = TerrainConfig::planarVoxelIndex(
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH,
      TerrainConfig::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_dy_obs[cell] = 10;
  state_.laser_cloud_crop->clear();
  state_.laser_cloud_crop->push_back({0.05F, 0, 0, 0});

  TerrainAlgorithm::filterDynamicObstaclePoints(cfg_, state_);

  EXPECT_EQ(state_.planar_voxel_dy_obs[cell], 10);
}

// ── addNoDataObstacles ──
// 数据稀疏区域生成虚拟障碍点，防止无数据区域的路径规划
TEST_F(AlgorithmTest,
       AddNoDataObstacles_EmptyPlanarVoxels_CreatesObstaclePoints) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  cfg_.no_data_block_skip_num = 1;
  cfg_.min_block_point_num = 5;
  cfg_.vehicle_height = 1.5;
  state_.planar_voxel_edge.fill(0);
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    state_.planar_point_elev[i].clear();
  }
  state_.terrain_cloud_elev->clear();

  TerrainAlgorithm::addNoDataObstacles(cfg_, state_);

  EXPECT_GT(state_.terrain_cloud_elev->points.size(), 0U);
}

// 所有 voxel 点数充足时不生成任何虚拟障碍
TEST_F(AlgorithmTest,
       AddNoDataObstacles_AllVoxelsHaveEnoughPoints_NoObstaclesCreated) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  cfg_.min_block_point_num = 5;
  state_.planar_voxel_edge.fill(0);
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    state_.planar_point_elev[i] = {0.1, 0.2, 0.3, 0.4, 0.5};
  }
  state_.terrain_cloud_elev->clear();

  TerrainAlgorithm::addNoDataObstacles(cfg_, state_);

  EXPECT_EQ(state_.terrain_cloud_elev->points.size(), 0U);
}
