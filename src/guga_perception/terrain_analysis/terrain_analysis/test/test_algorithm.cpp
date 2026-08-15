#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/config.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

class AlgorithmTest : public testing::Test {
protected:
  AlgorithmTest() {
    resetState();
    config_.use_sorting = true;
    config_.quantile_z = 0.25;
    config_.limit_ground_lift = false;
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

  TerrainConfig config_;
  TerrainState state_;
};

// 车辆未移动时，体素网格不发生滚动
TEST_F(AlgorithmTest, RolloverVoxels_Stationary_NoShift) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  int sx = state_.terrain_voxel_shift_x;
  int sy = state_.terrain_voxel_shift_y;

  terrain_analysis::algorithm::rolloverVoxels(config_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_x, sx);
  EXPECT_EQ(state_.terrain_voxel_shift_y, sy);
}

// 车辆向左超出 voxel 范围时，沿 X 负向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_LeftOfCenter_ShiftsXNegative) {
  state_.vehicle_x = -2.0;
  int sx = state_.terrain_voxel_shift_x;

  terrain_analysis::algorithm::rolloverVoxels(config_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_x, sx - 1);
}

// 车辆向右超出 voxel 范围时，沿 X 正向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_RightOfCenter_ShiftsXPositive) {
  state_.vehicle_x = 2.0;
  int sx = state_.terrain_voxel_shift_x;

  terrain_analysis::algorithm::rolloverVoxels(config_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_x, sx + 1);
}

// 车辆向下超出 voxel 范围时，沿 Y 负向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_BelowCenter_ShiftsYNegative) {
  state_.vehicle_y = -2.0;
  int sy = state_.terrain_voxel_shift_y;

  terrain_analysis::algorithm::rolloverVoxels(config_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_y, sy - 1);
}

// 车辆向上超出 voxel 范围时，沿 Y 正向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_AboveCenter_ShiftsYPositive) {
  state_.vehicle_y = 2.0;
  int sy = state_.terrain_voxel_shift_y;

  terrain_analysis::algorithm::rolloverVoxels(config_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_y, sy + 1);
}

// 滚动后目标 cell 被清空，原有数据随 shift 迁移
TEST_F(AlgorithmTest, RolloverVoxels_ShiftLeft_PreservesDataFromShiftedCell) {
  state_.vehicle_x = -2.0;
  state_.terrain_voxel_cloud[0]->clear();
  pcl::PointXYZI p{0, 0, 0, 0};
  state_.terrain_voxel_cloud[0]->push_back(p);

  terrain_analysis::algorithm::rolloverVoxels(config_, state_);

  // After shift-left, voxel(0,0) becomes the destination cell and gets cleared
  EXPECT_TRUE(state_.terrain_voxel_cloud[0]->points.empty());
}

// 车辆同时向左下方移动，X 和 Y 各滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_LeftAndDown_ShiftsBothAxes) {
  state_.vehicle_x = -2.0;
  state_.vehicle_y = -2.0;
  int sx = state_.terrain_voxel_shift_x;
  int sy = state_.terrain_voxel_shift_y;

  terrain_analysis::algorithm::rolloverVoxels(config_, state_);

  EXPECT_EQ(state_.terrain_voxel_shift_x, sx - 1);
  EXPECT_EQ(state_.terrain_voxel_shift_y, sy - 1);
}

// ── voxelize ──
// 原点处的单个点被分配到网格正中的 cell
TEST_F(AlgorithmTest, Voxelize_MapsPointToCenterCell) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.laser_cloud_crop->clear();
  state_.laser_cloud_crop->push_back({0, 0, 0, 0});

  terrain_analysis::algorithm::voxelize(config_, state_);

  size_t center = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  EXPECT_EQ(state_.terrain_voxel_cloud[center]->points.size(), 1U);
  EXPECT_EQ(state_.terrain_voxel_update_num[center], 1);
}

// 空点云不产生任何体素分配
TEST_F(AlgorithmTest, Voxelize_EmptyCloud_NoChange) {
  state_.laser_cloud_crop->clear();

  terrain_analysis::algorithm::voxelize(config_, state_);

  for (int i = 0; i < TerrainGrid::TERRAIN_VOXEL_NUM; i++) {
    EXPECT_EQ(state_.terrain_voxel_update_num[i], 0);
  }
}

// ── computeElevation ──
// 排序模式下取指定分位数作为地面高度估计
TEST_F(AlgorithmTest, ComputeElevation_UseSorting_ReturnsQuantile) {
  config_.use_sorting = true;
  config_.quantile_z = 0.5;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_elev.fill(999);
  state_.planar_point_elev[cell] = {0.1, 0.5, 0.3, 0.2, 0.4};

  terrain_analysis::algorithm::computeElevation(config_, state_);

  // sorted: 0.1, 0.2, 0.3, 0.4, 0.5. quantile 0.5*(5) = 2 → index 2 → 0.3
  EXPECT_FLOAT_EQ(state_.planar_voxel_elev[cell], 0.3F);
}

// 最小值模式下取最低点作为地面高度估计
TEST_F(AlgorithmTest, ComputeElevation_UseMinimum_ReturnsMinimum) {
  config_.use_sorting = false;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_elev.fill(999);
  state_.planar_point_elev[cell] = {1.5, 0.5, 1.0};

  terrain_analysis::algorithm::computeElevation(config_, state_);

  EXPECT_FLOAT_EQ(state_.planar_voxel_elev[cell], 0.5F);
}

// 分位数与最小值差距过大时，限制地面高度不超过 min+max_ground_lift
TEST_F(AlgorithmTest,
       ComputeElevation_LiftLimited_CapsAtMinimumPlusMaxGroundLift) {
  config_.use_sorting = true;
  config_.quantile_z = 0.5;
  config_.limit_ground_lift = true;
  config_.max_ground_lift = 0.3;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_elev.fill(999);
  // sorted: 0.5, 1.0, 2.0. quantile 0.5*3 = 1 → 1.0. diff 1.0-0.5=0.5 > 0.3
  state_.planar_point_elev[cell] = {0.5, 2.0, 1.0};

  terrain_analysis::algorithm::computeElevation(config_, state_);

  // lift limited → 0.5 + 0.3 = 0.8
  EXPECT_FLOAT_EQ(state_.planar_voxel_elev[cell], 0.8F);
}

// quantile_z=1.0 时 quantile_index 达到 point_count 边界，回退到最后一点
TEST_F(AlgorithmTest, ComputeElevation_QuantileIndexAtBoundary_ClampedToLast) {
  config_.use_sorting = true;
  config_.quantile_z = 1.0;
  config_.limit_ground_lift = false;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_elev.fill(999);
  // 3 points: sorted 0.1, 0.3, 0.9. quantile 1.0*3 = 3 >= 3 → clamp to 2 → 0.9
  state_.planar_point_elev[cell] = {0.1, 0.9, 0.3};

  terrain_analysis::algorithm::computeElevation(config_, state_);

  EXPECT_FLOAT_EQ(state_.planar_voxel_elev[cell], 0.9F);
}

// ── detectDynamicObstacles ──
TEST_F(AlgorithmTest, DetectDynamicObstacles_NearPoint_AddsMinPointNumToCell) {
  config_.clear_dy_obs = true;
  config_.min_dy_obs_distance = 5.0;  // high → all points "close"
  config_.min_dy_obs_point_num = 7;
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.planar_voxel_dy_obs.fill(0);
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back({0.1F, 0, 0, 0});

  terrain_analysis::algorithm::detectDynamicObstacles(config_, state_);

  int total = 0;
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    total += state_.planar_voxel_dy_obs[i];
  }
  EXPECT_GT(total, 0);
}

// clear_dy_obs 标志为 true 时仍然处理点云
TEST_F(AlgorithmTest,
       DetectDynamicObstacles_ClearInitEnabled_StillProcessesPoints) {
  config_.clear_dy_obs = true;
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.planar_voxel_dy_obs.fill(0);
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back({0.1F, 0, 0, 0});

  terrain_analysis::algorithm::detectDynamicObstacles(config_, state_);

  int total = 0;
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    total += state_.planar_voxel_dy_obs[i];
  }
  // detects even when init flag is true — it always processes
  EXPECT_GT(total, 0);
}

// 传感器视角内的点触发动态障碍计数递增
TEST_F(AlgorithmTest,
       DetectDynamicObstacles_PointInVfov_IncrementsCellCounter) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.cos_vehicle_roll = 1;
  state_.sin_vehicle_roll = 0;
  state_.cos_vehicle_pitch = 1;
  state_.sin_vehicle_pitch = 0;
  state_.cos_vehicle_yaw = 1;
  state_.sin_vehicle_yaw = 0;
  state_.planar_voxel_dy_obs.fill(0);
  state_.terrain_cloud->clear();
  // Point at moderate distance, slightly elevated → within typical VFOV
  state_.terrain_cloud->push_back({1.0F, 0.1F, 0.3F, 0});

  config_.min_dy_obs_distance = 0.0;
  config_.min_dy_obs_point_num = 3;
  config_.min_dy_obs_angle = -1.0;  // below any realistic scan angle
  config_.min_dy_obs_relative_z = -1.0;
  config_.min_dy_obs_vfov = -0.5;  // radians, wide open
  config_.max_dy_obs_vfov = 0.5;
  config_.abs_dy_obs_relative_z_threshold = 0.01;  // tiny → rely on VFOV

  terrain_analysis::algorithm::detectDynamicObstacles(config_, state_);

  int total = 0;
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    total += state_.planar_voxel_dy_obs[i];
  }
  EXPECT_GT(total, 0);
}

// 传感器视角外的点不触发动态障碍计数
TEST_F(AlgorithmTest, DetectDynamicObstacles_PointOutsideVfov_NoIncrement) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.cos_vehicle_roll = 1;
  state_.sin_vehicle_roll = 0;
  state_.cos_vehicle_pitch = 1;
  state_.sin_vehicle_pitch = 0;
  state_.cos_vehicle_yaw = 1;
  state_.sin_vehicle_yaw = 0;
  state_.planar_voxel_dy_obs.fill(0);
  state_.terrain_cloud->clear();
  // Point far away → scan angle will be very shallow, outside VFOV
  state_.terrain_cloud->push_back({10.0F, 0.0F, 0.0F, 0});

  config_.min_dy_obs_distance = 0.0;
  config_.min_dy_obs_point_num = 3;
  config_.min_dy_obs_angle = -1.0;
  config_.min_dy_obs_relative_z = -1.0;
  config_.min_dy_obs_vfov = 0.1;  // narrow VFOV
  config_.max_dy_obs_vfov = 0.2;
  config_.abs_dy_obs_relative_z_threshold = 0.0;  // off

  terrain_analysis::algorithm::detectDynamicObstacles(config_, state_);

  int total = 0;
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    total += state_.planar_voxel_dy_obs[i];
  }
  EXPECT_EQ(total, 0);
}

// ── filterDynamicObstaclePoints ──
// 高角度点（头顶悬挂物）清零对应 cell 的动态障碍计数
TEST_F(AlgorithmTest,
       FilterDynamicObstaclePoints_HighAnglePoint_ResetsCellCounter) {
  config_.clear_dy_obs = true;
  config_.min_dy_obs_angle = 10.0 * M_PI / 180.0;
  config_.min_dy_obs_relative_z = -0.5;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_dy_obs[cell] = 10;
  state_.laser_cloud_crop->clear();
  // high relative_z → angle close to 90° > 10°
  state_.laser_cloud_crop->push_back({0.05F, 0, 2.0F, 0});

  terrain_analysis::algorithm::filterDynamicObstaclePoints(config_, state_);

  EXPECT_EQ(state_.planar_voxel_dy_obs[cell], 0);
}

// 低角度点（地面/低障碍）保持 cell 计数不变
TEST_F(AlgorithmTest,
       FilterDynamicObstaclePoints_LowAnglePoint_KeepsCellCounter) {
  config_.clear_dy_obs = true;
  config_.min_dy_obs_angle = 90.0 * M_PI
                             / 180.0;  // nearly impossible to exceed
  config_.min_dy_obs_relative_z = -0.5;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_dy_obs[cell] = 10;
  state_.laser_cloud_crop->clear();
  state_.laser_cloud_crop->push_back({0.05F, 0, 0, 0});

  terrain_analysis::algorithm::filterDynamicObstaclePoints(config_, state_);

  EXPECT_EQ(state_.planar_voxel_dy_obs[cell], 10);
}

// ── addNoDataObstacles ──
// 数据稀疏区域生成虚拟障碍点，防止无数据区域的路径规划
TEST_F(AlgorithmTest,
       AddNoDataObstacles_EmptyPlanarVoxels_CreatesObstaclePoints) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  config_.no_data_block_skip_num = 1;
  config_.min_block_point_num = 5;
  config_.vehicle_height = 1.5;
  state_.planar_voxel_edge.fill(0);
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    state_.planar_point_elev[i].clear();
  }
  state_.terrain_cloud_elev->clear();

  terrain_analysis::algorithm::addNoDataObstacles(config_, state_);

  EXPECT_GT(state_.terrain_cloud_elev->points.size(), 0U);
}

// 所有 voxel 点数充足时不生成任何虚拟障碍
TEST_F(AlgorithmTest,
       AddNoDataObstacles_AllVoxelsHaveEnoughPoints_NoObstaclesCreated) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  config_.min_block_point_num = 5;
  state_.planar_voxel_edge.fill(0);
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    state_.planar_point_elev[i] = {0.1, 0.2, 0.3, 0.4, 0.5};
  }
  state_.terrain_cloud_elev->clear();

  terrain_analysis::algorithm::addNoDataObstacles(config_, state_);

  EXPECT_EQ(state_.terrain_cloud_elev->points.size(), 0U);
}

// ── estimateGround ──

// 栅格边缘点触发射线邻居越界检查，不崩溃
TEST_F(AlgorithmTest, EstimateGround_EdgePoint_HandlesOobNeighbors) {
  constexpr double SZ = 0.2;
  double edge = SZ * (25 - 1);  // ~4.8m, column=49, delta_col+1=50 in bounds
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud->clear();
  pcl::PointXYZI pt;
  pt.x = static_cast<float>(edge);
  pt.y = 0;
  pt.z = 0;
  pt.intensity = 0;
  state_.terrain_cloud->push_back(pt);

  terrain_analysis::algorithm::estimateGround(config_, state_);
  // No crash = pass; point Z=0 is within min/max relative_z range
  EXPECT_TRUE(true);
}

// 超出 planar grid 的点被跳过，避免在计算 base index 时越界
TEST_F(AlgorithmTest, EstimateGround_PointOutsidePlanarGrid_Ignored) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back({6.0F, 0.0F, 0.0F, 0.0F});

  terrain_analysis::algorithm::estimateGround(config_, state_);

  for (const auto& elevations : state_.planar_point_elev) {
    EXPECT_TRUE(elevations.empty());
  }
}

// ── computeHeightMap ──

// Z 超出范围的点被过滤
TEST_F(AlgorithmTest, ComputeHeightMap_PointOutOfZRange_Filtered) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud_elev->clear();
  state_.planar_voxel_elev.fill(0);
  state_.planar_voxel_dy_obs.fill(0);
  for (auto& e : state_.planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config_.min_block_point_num = 5;
  config_.vehicle_height = 1.0;
  config_.consider_drop = false;

  // Point at z=2.0 exceeds max_relative_z (0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 2.0F;
  pt.intensity = 0;
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back(pt);

  terrain_analysis::algorithm::computeHeightMap(config_, state_);
  EXPECT_TRUE(state_.terrain_cloud_elev->points.empty());
}

// consider_drop 开启时高度取绝对值，负高度也被接受
TEST_F(AlgorithmTest, ComputeHeightMap_ConsiderDrop_AcceptsNegativeHeight) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud_elev->clear();
  state_.planar_voxel_elev.fill(0.3);  // ground at +0.3, point at z=0 → -0.3m
  state_.planar_voxel_dy_obs.fill(0);
  for (auto& e : state_.planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config_.min_block_point_num = 5;
  config_.vehicle_height = 1.0;
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.consider_drop = true;
  config_.clear_dy_obs = false;

  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.0F;
  pt.intensity = 0;
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back(pt);

  terrain_analysis::algorithm::computeHeightMap(config_, state_);
  EXPECT_EQ(state_.terrain_cloud_elev->points.size(), 1U);
  // height = abs(0 - 0.3) = 0.3 < 1.0 → accepted
}

// 高度超过 vehicle_height 的点被过滤
TEST_F(AlgorithmTest, ComputeHeightMap_AboveVehicleHeight_Filtered) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud_elev->clear();
  state_.planar_voxel_elev.fill(0);
  state_.planar_voxel_dy_obs.fill(0);
  for (auto& e : state_.planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config_.min_block_point_num = 5;
  config_.vehicle_height = 0.1;
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.consider_drop = false;
  config_.clear_dy_obs = false;

  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.5F;
  pt.intensity = 0;
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back(pt);

  terrain_analysis::algorithm::computeHeightMap(config_, state_);
  // height = 0.5 - 0 = 0.5 >= 0.1 → filtered
  EXPECT_TRUE(state_.terrain_cloud_elev->points.empty());
}

// 车顶上方超过安全间隙的点（天花板）不参与地面估计，防止 elev 被抬高后
// 地面点高度差变负、天花板点高度差落入障碍区间（窄隧道场景）
TEST_F(AlgorithmTest, EstimateGround_CeilingPoint_ExcludedFromElevation) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  config_.ceiling_clearance = 0.2;
  state_.terrain_cloud->clear();
  // 天花板点：相对车高 0.26m（模拟 260mm 顶隙的隧道），位于车辆正上方
  state_.terrain_cloud->push_back({0.0F, 0.0F, 0.26F, 0.0F});

  terrain_analysis::algorithm::estimateGround(config_, state_);

  size_t center = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  EXPECT_TRUE(state_.planar_point_elev[center].empty());
}

// 车顶下方/间隙内的点仍正常参与地面估计
TEST_F(AlgorithmTest, EstimateGround_BelowCeilingClearance_Participates) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  config_.ceiling_clearance = 0.2;
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back({0.0F, 0.0F, -0.1F, 0.0F});

  terrain_analysis::algorithm::estimateGround(config_, state_);

  size_t center = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  EXPECT_EQ(state_.planar_point_elev[center].size(), 1U);
}

// 车顶上方达到安全间隙的点（天花板/横梁）不作为障碍输出：顶隙足够，
// 车辆可从下方通过
TEST_F(AlgorithmTest, ComputeHeightMap_CeilingPoint_NotObstacle) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud_elev->clear();
  state_.planar_voxel_elev.fill(0);  // 地面高度 0
  state_.planar_voxel_dy_obs.fill(0);
  for (auto& e : state_.planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};  // 满足 min_block_point_num
  }
  config_.min_block_point_num = 5;
  config_.vehicle_height = 1.0;  // 旧逻辑下 0.26 < 1.0 会被输出为障碍
  config_.ceiling_clearance = 0.2;
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.consider_drop = false;
  config_.clear_dy_obs = false;

  // 天花板点：相对车高 0.26m，高于 ceiling_clearance(0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.26F;
  pt.intensity = 0;
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back(pt);

  terrain_analysis::algorithm::computeHeightMap(config_, state_);
  EXPECT_TRUE(state_.terrain_cloud_elev->points.empty());
}

// 车顶上方安全间隙内的点仍然是障碍（低矮横梁/门楣不应漏检）
TEST_F(AlgorithmTest, ComputeHeightMap_BelowCeilingClearance_StillObstacle) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud_elev->clear();
  state_.planar_voxel_elev.fill(0);
  state_.planar_voxel_dy_obs.fill(0);
  for (auto& e : state_.planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config_.min_block_point_num = 5;
  config_.vehicle_height = 1.0;
  config_.ceiling_clearance = 0.2;
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.consider_drop = false;
  config_.clear_dy_obs = false;

  // 低矮障碍点：相对车高 0.1m，低于 ceiling_clearance(0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.1F;
  pt.intensity = 0;
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back(pt);

  terrain_analysis::algorithm::computeHeightMap(config_, state_);
  ASSERT_EQ(state_.terrain_cloud_elev->points.size(), 1U);
  // height_above_ground = 0.1 - 0 = 0.1，写入 intensity
  EXPECT_NEAR(state_.terrain_cloud_elev->points[0].intensity, 0.1F, 1e-6);
}

// ── keepVoxelPoint boundary tests (via updateVoxels) ──

namespace {
  // Set up a single voxel cell with one point at the vehicle origin,
  // then trigger update. Returns the point count retained in the cell.
  int updateSinglePoint(TerrainConfig& config, TerrainState& state,
                        double relative_z, double distance) {
    config.min_relative_z = -1.5;
    config.max_relative_z = 0.2;
    config.distance_ratio_z = 0.2;
    config.decay_time = 999.0;
    config.no_decay_distance = 999.0;
    config.voxel_point_update_thre = 1;

    state.vehicle_x = 0;
    state.vehicle_y = 0;
    state.vehicle_z = 0.0;
    state.clearing_cloud = false;
    state.clearing_distance = 0.0;
    state.laser_cloud_time = 1.0;
    state.system_init_time = 0.0;

    int center_cell = TerrainGrid::terrainVoxelIndex(
        TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
        TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
    auto& cell = *state.terrain_voxel_cloud[center_cell];
    cell.clear();
    pcl::PointXYZI point;
    point.x = static_cast<float>(distance);
    point.y = 0.0F;
    point.z = static_cast<float>(relative_z);
    point.intensity = 0.0F;
    cell.push_back(point);

    state.terrain_voxel_update_num[center_cell] =
        config.voxel_point_update_thre;

    terrain_analysis::algorithm::updateVoxels(config, state);
    return static_cast<int>(
        state.terrain_voxel_cloud[center_cell]->points.size());
  }
}  // namespace

// 略高于下限边界的点被保留
TEST_F(AlgorithmTest, KeepVoxelPoint_BelowLowerBoundary_Excluded) {
  double z_margin = config_.distance_ratio_z * 1.0;     // = 0.2
  double boundary = config_.min_relative_z - z_margin;  // = -1.7
  int kept = updateSinglePoint(config_, state_, boundary + 0.01, 1.0);
  EXPECT_EQ(kept, 1);
}

// 等于下限边界的点被排除
TEST_F(AlgorithmTest, KeepVoxelPoint_AtLowerBoundary_Excluded) {
  double z_margin = config_.distance_ratio_z * 1.0;
  double boundary = config_.min_relative_z - z_margin;
  int kept = updateSinglePoint(config_, state_, boundary, 1.0);
  EXPECT_EQ(kept, 0);
}

// 等于上限边界的点被排除
TEST_F(AlgorithmTest, KeepVoxelPoint_AtUpperBoundary_Excluded) {
  double z_margin = config_.distance_ratio_z * 1.0;
  double boundary = config_.max_relative_z + z_margin;  // = 0.4
  int kept = updateSinglePoint(config_, state_, boundary, 1.0);
  EXPECT_EQ(kept, 0);
}

// 略低于上限边界的点被保留
TEST_F(AlgorithmTest, KeepVoxelPoint_BelowUpperBoundary_Kept) {
  double z_margin = config_.distance_ratio_z * 1.0;
  double boundary = config_.max_relative_z + z_margin;
  int kept = updateSinglePoint(config_, state_, boundary - 0.01, 1.0);
  EXPECT_EQ(kept, 1);
}

// 点的时间戳过期且离车辆较远 → 被清除
TEST_F(AlgorithmTest, KeepVoxelPoint_ExpiredFarPoint_Excluded) {
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.decay_time = 1.0;
  config_.no_decay_distance = 0.0;
  config_.voxel_point_update_thre = 1;

  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0.0;
  state_.laser_cloud_time = 10.0;
  state_.system_init_time = 0.0;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  auto& cell = *state_.terrain_voxel_cloud[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 2.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 0.0F;
  cell.push_back(point);

  state_.terrain_voxel_update_num[center_cell] =
      config_.voxel_point_update_thre;

  terrain_analysis::algorithm::updateVoxels(config_, state_);
  EXPECT_TRUE(state_.terrain_voxel_cloud[center_cell]->points.empty());
}

// 近点即使过期也保留（near 优先于 decay）
TEST_F(AlgorithmTest, KeepVoxelPoint_NearPointEvenIfExpired_Kept) {
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.decay_time = 1.0;
  config_.no_decay_distance = 3.0;
  config_.voxel_point_update_thre = 1;

  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0.0;
  state_.laser_cloud_time = 10.0;
  state_.system_init_time = 0.0;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  auto& cell = *state_.terrain_voxel_cloud[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 1.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 0.0F;
  cell.push_back(point);

  state_.terrain_voxel_update_num[center_cell] =
      config_.voxel_point_update_thre;

  terrain_analysis::algorithm::updateVoxels(config_, state_);
  EXPECT_EQ(state_.terrain_voxel_cloud[center_cell]->points.size(), 1U);
}

// 清除模式下距离范围内的点被排除
TEST_F(AlgorithmTest, KeepVoxelPoint_WithinClearingDistance_Excluded) {
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.decay_time = 999.0;
  config_.no_decay_distance = 999.0;
  config_.voxel_point_update_thre = 1;

  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0.0;
  state_.laser_cloud_time = 1.0;
  state_.system_init_time = 0.0;
  state_.clearing_cloud = true;
  state_.clearing_distance = 2.0;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  auto& cell = *state_.terrain_voxel_cloud[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 1.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 1.0F;
  cell.push_back(point);

  state_.terrain_voxel_update_num[center_cell] =
      config_.voxel_point_update_thre;

  terrain_analysis::algorithm::updateVoxels(config_, state_);
  EXPECT_TRUE(state_.terrain_voxel_cloud[center_cell]->points.empty());
}

// ── shouldPruneVoxel (via updateVoxels) ──

// update_num 未达阈值且时间未到 → 不修剪
TEST_F(AlgorithmTest, ShouldPruneVoxel_NotEnoughPointsOrTime_NotPruned) {
  config_.voxel_point_update_thre = 100;
  config_.voxel_time_update_thre = 10.0;
  state_.laser_cloud_time = 1.0;
  state_.system_init_time = 0.0;
  state_.clearing_cloud = false;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  state_.terrain_voxel_update_num[center_cell] = 5;
  state_.terrain_voxel_update_time[center_cell] = 0.0;

  terrain_analysis::algorithm::updateVoxels(config_, state_);
  EXPECT_NE(state_.terrain_voxel_cloud[center_cell], nullptr);
}

// clearing_cloud 模式触发强制修剪
TEST_F(AlgorithmTest, ShouldPruneVoxel_ClearingCloud_Pruned) {
  config_.voxel_point_update_thre = 100;
  config_.voxel_time_update_thre = 10.0;
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.decay_time = 999.0;
  config_.no_decay_distance = 999.0;
  state_.laser_cloud_time = 1.0;
  state_.system_init_time = 0.0;
  state_.clearing_cloud = true;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  auto& cell = *state_.terrain_voxel_cloud[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 0.1F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 1.0F;
  cell.push_back(point);
  state_.terrain_voxel_update_num[center_cell] = 5;

  terrain_analysis::algorithm::updateVoxels(config_, state_);
  EXPECT_EQ(cell.points.size(), 0U);
}

// update_num 达到阈值 → 触发修剪
TEST_F(AlgorithmTest, ShouldPruneVoxel_PointCountReached_Pruned) {
  config_.voxel_point_update_thre = 10;
  config_.voxel_time_update_thre = 999.0;
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.decay_time = 999.0;
  config_.no_decay_distance = 999.0;
  state_.laser_cloud_time = 1.0;
  state_.system_init_time = 0.0;
  state_.clearing_cloud = false;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  auto& cell = *state_.terrain_voxel_cloud[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 0.1F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 1.0F;
  cell.push_back(point);
  state_.terrain_voxel_update_num[center_cell] =
      config_.voxel_point_update_thre;

  terrain_analysis::algorithm::updateVoxels(config_, state_);
  EXPECT_EQ(cell.points.size(), 1U);
}

// ── computeHeightMap 过滤分支 ──

// dy_obs 计数达标 + clear_dy_obs → 该 cell 被过滤
TEST_F(AlgorithmTest, ComputeHeightMap_DynamicObstacleCell_Filtered) {
  state_.vehicle_x = 0;
  state_.vehicle_y = 0;
  state_.vehicle_z = 0;
  state_.terrain_cloud_elev->clear();
  state_.planar_voxel_elev.fill(0);
  for (auto& e : state_.planar_point_elev) {
    e = {0., 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config_.min_block_point_num = 5;
  config_.vehicle_height = 1.0;
  config_.min_relative_z = -10.0;
  config_.max_relative_z = 10.0;
  config_.clear_dy_obs = true;
  config_.min_dy_obs_point_num = 3;

  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state_.planar_voxel_dy_obs[cell] = 5;

  pcl::PointXYZI pt;
  pt.x = 0.0F;
  pt.y = 0;
  pt.z = 0.1F;
  pt.intensity = 0;
  state_.terrain_cloud->clear();
  state_.terrain_cloud->push_back(pt);

  terrain_analysis::algorithm::computeHeightMap(config_, state_);
  EXPECT_TRUE(state_.terrain_cloud_elev->points.empty());
}
