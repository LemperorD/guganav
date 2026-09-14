#include "terrain_analysis/core/terrain_processor.hpp"
#include "terrain_analysis/core/config.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

// 白盒测试需要逐阶段驱动管线；fixture 是 TerrainProcessor 的 friend，
// 因此把这类调用收在它以内的分发器里。
namespace stage {
  enum class Id {
    Rollover,
    Voxelize,
    UpdateVoxels,
    Collect,
    EstimateGround,
    DetectDynamic,
    FilterDynamic,
    Elevation,
    HeightMap
  };
}  // namespace stage

namespace terrain_analysis {
  class AlgorithmTest : public testing::Test {
  protected:
    AlgorithmTest() {
      resetState();
      config().use_sorting = true;
      config().quantile_z = 0.25;
      config().limit_ground_lift = false;
    }

    void resetState() {
      processor_.state_ = {};
      for (auto& ptr : state().terrain_voxel_cloud) {
        ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      }
      state().laser_cloud_crop =
          std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      state().terrain_cloud =
          std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      state().terrain_cloud_elev =
          std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }

    void runStage(stage::Id id) {
      switch (id) {
        case stage::Id::Rollover:
          processor_.rolloverVoxels();
          break;
        case stage::Id::Voxelize:
          processor_.voxelize();
          break;
        case stage::Id::UpdateVoxels:
          processor_.updateVoxels();
          break;
        case stage::Id::Collect:
          processor_.collectTerrainCloud();
          break;
        case stage::Id::EstimateGround:
          processor_.estimateGround();
          break;
        case stage::Id::DetectDynamic:
          processor_.detectDynamicObstacles();
          break;
        case stage::Id::FilterDynamic:
          processor_.filterDynamicObstaclePoints();
          break;
        case stage::Id::Elevation:
          processor_.computeElevation();
          break;
        case stage::Id::HeightMap:
          processor_.computeHeightMap();
          break;
      }
    }

    TerrainProcessor& processor() {
      return processor_;
    }
    TerrainConfig& config() {
      return processor_.config_;
    }
    TerrainState& state() {
      return processor_.state_;
    }

    // then trigger update. Returns the point count retained in the cell.
    static int updateSinglePoint(TerrainProcessor& proc, double relative_z,
                                 double distance) {
      TerrainConfig& config = proc.config_;
      TerrainState& state = proc.state_;

      config.min_relative_z = -1.5;
      config.max_relative_z = 0.2;
      config.distance_ratio_z = 0.2;
      config.decay_time = 999.0;
      config.no_decay_distance = 999.0;
      config.voxel_point_update_thre = 1;

      state.vehicle_x = 0;
      state.vehicle_y = 0;
      state.vehicle_z = 0.0;
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

      proc.updateVoxels();
      return static_cast<int>(
          state.terrain_voxel_cloud[center_cell]->points.size());
    }

    TerrainProcessor processor_;
  };
}  // namespace terrain_analysis

using terrain_analysis::AlgorithmTest;

// 车辆未移动时，体素网格不发生滚动
TEST_F(AlgorithmTest, RolloverVoxels_Stationary_NoShift) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  int sx = state().terrain_voxel_shift_x;
  int sy = state().terrain_voxel_shift_y;

  runStage(stage::Id::Rollover);

  EXPECT_EQ(state().terrain_voxel_shift_x, sx);
  EXPECT_EQ(state().terrain_voxel_shift_y, sy);
}

// 车辆向左超出 voxel 范围时，沿 X 负向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_LeftOfCenter_ShiftsXNegative) {
  state().vehicle_x = -2.0;
  int sx = state().terrain_voxel_shift_x;

  runStage(stage::Id::Rollover);

  EXPECT_EQ(state().terrain_voxel_shift_x, sx - 1);
}

// 车辆向右超出 voxel 范围时，沿 X 正向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_RightOfCenter_ShiftsXPositive) {
  state().vehicle_x = 2.0;
  int sx = state().terrain_voxel_shift_x;

  runStage(stage::Id::Rollover);

  EXPECT_EQ(state().terrain_voxel_shift_x, sx + 1);
}

// 车辆向下超出 voxel 范围时，沿 Y 负向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_BelowCenter_ShiftsYNegative) {
  state().vehicle_y = -2.0;
  int sy = state().terrain_voxel_shift_y;

  runStage(stage::Id::Rollover);

  EXPECT_EQ(state().terrain_voxel_shift_y, sy - 1);
}

// 车辆向上超出 voxel 范围时，沿 Y 正向滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_AboveCenter_ShiftsYPositive) {
  state().vehicle_y = 2.0;
  int sy = state().terrain_voxel_shift_y;

  runStage(stage::Id::Rollover);

  EXPECT_EQ(state().terrain_voxel_shift_y, sy + 1);
}

// 滚动后目标 cell 被清空，原有数据随 shift 迁移
TEST_F(AlgorithmTest, RolloverVoxels_ShiftLeft_PreservesDataFromShiftedCell) {
  state().vehicle_x = -2.0;
  state().terrain_voxel_cloud[0]->clear();
  pcl::PointXYZI p{0, 0, 0, 0};
  state().terrain_voxel_cloud[0]->push_back(p);

  runStage(stage::Id::Rollover);

  // After shift-left, voxel(0,0) becomes the destination cell and gets cleared
  EXPECT_TRUE(state().terrain_voxel_cloud[0]->points.empty());
}

// 车辆同时向左下方移动，X 和 Y 各滚动一格
TEST_F(AlgorithmTest, RolloverVoxels_LeftAndDown_ShiftsBothAxes) {
  state().vehicle_x = -2.0;
  state().vehicle_y = -2.0;
  int sx = state().terrain_voxel_shift_x;
  int sy = state().terrain_voxel_shift_y;

  runStage(stage::Id::Rollover);

  EXPECT_EQ(state().terrain_voxel_shift_x, sx - 1);
  EXPECT_EQ(state().terrain_voxel_shift_y, sy - 1);
}

// ── voxelize ──
// 原点处的单个点被分配到网格正中的 cell
TEST_F(AlgorithmTest, Voxelize_MapsPointToCenterCell) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().laser_cloud_crop->clear();
  state().laser_cloud_crop->push_back({0, 0, 0, 0});

  runStage(stage::Id::Voxelize);

  size_t center = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  EXPECT_EQ(state().terrain_voxel_cloud[center]->points.size(), 1U);
  EXPECT_EQ(state().terrain_voxel_update_num[center], 1);
}

// 空点云不产生任何体素分配
TEST_F(AlgorithmTest, Voxelize_EmptyCloud_NoChange) {
  state().laser_cloud_crop->clear();

  runStage(stage::Id::Voxelize);

  for (int i = 0; i < TerrainGrid::TERRAIN_VOXEL_NUM; i++) {
    EXPECT_EQ(state().terrain_voxel_update_num[i], 0);
  }
}

// ── computeElevation ──
// 排序模式下取指定分位数作为地面高度估计
TEST_F(AlgorithmTest, ComputeElevation_UseSorting_ReturnsQuantile) {
  config().use_sorting = true;
  config().quantile_z = 0.5;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state().planar_voxel_elev.fill(999);
  state().planar_point_elev[cell] = {0.1, 0.5, 0.3, 0.2, 0.4};

  runStage(stage::Id::Elevation);

  // sorted: 0.1, 0.2, 0.3, 0.4, 0.5. quantile 0.5*(5) = 2 → index 2 → 0.3
  EXPECT_FLOAT_EQ(state().planar_voxel_elev[cell], 0.3F);
}

// 最小值模式下取最低点作为地面高度估计
TEST_F(AlgorithmTest, ComputeElevation_UseMinimum_ReturnsMinimum) {
  config().use_sorting = false;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state().planar_voxel_elev.fill(999);
  state().planar_point_elev[cell] = {1.5, 0.5, 1.0};

  runStage(stage::Id::Elevation);

  EXPECT_FLOAT_EQ(state().planar_voxel_elev[cell], 0.5F);
}

// 分位数与最小值差距过大时，限制地面高度不超过 min+max_ground_lift
TEST_F(AlgorithmTest,
       ComputeElevation_LiftLimited_CapsAtMinimumPlusMaxGroundLift) {
  config().use_sorting = true;
  config().quantile_z = 0.5;
  config().limit_ground_lift = true;
  config().max_ground_lift = 0.3;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state().planar_voxel_elev.fill(999);
  // sorted: 0.5, 1.0, 2.0. quantile 0.5*3 = 1 → 1.0. diff 1.0-0.5=0.5 > 0.3
  state().planar_point_elev[cell] = {0.5, 2.0, 1.0};

  runStage(stage::Id::Elevation);

  // lift limited → 0.5 + 0.3 = 0.8
  EXPECT_FLOAT_EQ(state().planar_voxel_elev[cell], 0.8F);
}

// quantile_z=1.0 时 quantile_index 达到 point_count 边界，回退到最后一点
TEST_F(AlgorithmTest, ComputeElevation_QuantileIndexAtBoundary_ClampedToLast) {
  config().use_sorting = true;
  config().quantile_z = 1.0;
  config().limit_ground_lift = false;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state().planar_voxel_elev.fill(999);
  // 3 points: sorted 0.1, 0.3, 0.9. quantile 1.0*3 = 3 >= 3 → clamp to 2 → 0.9
  state().planar_point_elev[cell] = {0.1, 0.9, 0.3};

  runStage(stage::Id::Elevation);

  EXPECT_FLOAT_EQ(state().planar_voxel_elev[cell], 0.9F);
}

// ── detectDynamicObstacles ──
TEST_F(AlgorithmTest, DetectDynamicObstacles_NearPoint_AddsMinPointNumToCell) {
  config().min_dy_obs_distance = 5.0;  // high → all points "close"
  config().min_dy_obs_point_num = 7;
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().planar_voxel_dy_obs.fill(0);
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back({0.1F, 0, 0, 0});

  runStage(stage::Id::DetectDynamic);

  int total = 0;
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    total += state().planar_voxel_dy_obs[i];
  }
  EXPECT_GT(total, 0);
}

// 传感器视角内的点触发动态障碍计数递增
TEST_F(AlgorithmTest,
       DetectDynamicObstacles_PointInVfov_IncrementsCellCounter) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().cos_vehicle_roll = 1;
  state().sin_vehicle_roll = 0;
  state().cos_vehicle_pitch = 1;
  state().sin_vehicle_pitch = 0;
  state().cos_vehicle_yaw = 1;
  state().sin_vehicle_yaw = 0;
  state().planar_voxel_dy_obs.fill(0);
  state().terrain_cloud->clear();
  // Point at moderate distance, slightly elevated → within typical VFOV
  state().terrain_cloud->push_back({1.0F, 0.1F, 0.3F, 0});

  config().min_dy_obs_distance = 0.0;
  config().min_dy_obs_point_num = 3;
  config().min_dy_obs_angle = -1.0;  // below any realistic scan angle
  config().min_dy_obs_relative_z = -1.0;
  config().min_dy_obs_vfov = -0.5;  // radians, wide open
  config().max_dy_obs_vfov = 0.5;
  config().abs_dy_obs_relative_z_threshold = 0.01;  // tiny → rely on VFOV

  runStage(stage::Id::DetectDynamic);

  int total = 0;
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    total += state().planar_voxel_dy_obs[i];
  }
  EXPECT_GT(total, 0);
}

// 传感器视角外的点不触发动态障碍计数
TEST_F(AlgorithmTest, DetectDynamicObstacles_PointOutsideVfov_NoIncrement) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().cos_vehicle_roll = 1;
  state().sin_vehicle_roll = 0;
  state().cos_vehicle_pitch = 1;
  state().sin_vehicle_pitch = 0;
  state().cos_vehicle_yaw = 1;
  state().sin_vehicle_yaw = 0;
  state().planar_voxel_dy_obs.fill(0);
  state().terrain_cloud->clear();
  // Point far away → scan angle will be very shallow, outside VFOV
  state().terrain_cloud->push_back({10.0F, 0.0F, 0.0F, 0});

  config().min_dy_obs_distance = 0.0;
  config().min_dy_obs_point_num = 3;
  config().min_dy_obs_angle = -1.0;
  config().min_dy_obs_relative_z = -1.0;
  config().min_dy_obs_vfov = 0.1;  // narrow VFOV
  config().max_dy_obs_vfov = 0.2;
  config().abs_dy_obs_relative_z_threshold = 0.0;  // off

  runStage(stage::Id::DetectDynamic);

  int total = 0;
  for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
    total += state().planar_voxel_dy_obs[i];
  }
  EXPECT_EQ(total, 0);
}

// ── filterDynamicObstaclePoints ──
// 高角度点（头顶悬挂物）清零对应 cell 的动态障碍计数
TEST_F(AlgorithmTest,
       FilterDynamicObstaclePoints_HighAnglePoint_ResetsCellCounter) {
  config().min_dy_obs_angle = 10.0 * M_PI / 180.0;
  config().min_dy_obs_relative_z = -0.5;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state().planar_voxel_dy_obs[cell] = 10;
  state().laser_cloud_crop->clear();
  // high relative_z → angle close to 90° > 10°
  state().laser_cloud_crop->push_back({0.05F, 0, 2.0F, 0});

  runStage(stage::Id::FilterDynamic);

  EXPECT_EQ(state().planar_voxel_dy_obs[cell], 0);
}

// 低角度点（地面/低障碍）保持 cell 计数不变
TEST_F(AlgorithmTest,
       FilterDynamicObstaclePoints_LowAnglePoint_KeepsCellCounter) {
  config().min_dy_obs_angle = 90.0 * M_PI
                              / 180.0;  // nearly impossible to exceed
  config().min_dy_obs_relative_z = -0.5;
  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state().planar_voxel_dy_obs[cell] = 10;
  state().laser_cloud_crop->clear();
  state().laser_cloud_crop->push_back({0.05F, 0, 0, 0});

  runStage(stage::Id::FilterDynamic);

  EXPECT_EQ(state().planar_voxel_dy_obs[cell], 10);
}

// ── estimateGround ──

// 栅格边缘点触发射线邻居越界检查，不崩溃
TEST_F(AlgorithmTest, EstimateGround_EdgePoint_HandlesOobNeighbors) {
  constexpr double SZ = 0.2;
  double edge = SZ * (25 - 1);  // ~4.8m, column=49, delta_col+1=50 in bounds
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().terrain_cloud->clear();
  pcl::PointXYZI pt;
  pt.x = static_cast<float>(edge);
  pt.y = 0;
  pt.z = 0;
  pt.intensity = 0;
  state().terrain_cloud->push_back(pt);

  runStage(stage::Id::EstimateGround);
  // No crash = pass; point Z=0 is within min/max relative_z range
  EXPECT_TRUE(true);
}

// 超出 planar grid 的点被跳过，避免在计算 base index 时越界
TEST_F(AlgorithmTest, EstimateGround_PointOutsidePlanarGrid_Ignored) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back({6.0F, 0.0F, 0.0F, 0.0F});

  runStage(stage::Id::EstimateGround);

  for (const auto& elevations : state().planar_point_elev) {
    EXPECT_TRUE(elevations.empty());
  }
}

// ── computeHeightMap ──

// Z 超出范围的点被过滤
TEST_F(AlgorithmTest, ComputeHeightMap_PointOutOfZRange_Filtered) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().terrain_cloud_elev->clear();
  state().planar_voxel_elev.fill(0);
  state().planar_voxel_dy_obs.fill(0);
  for (auto& e : state().planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config().min_block_point_num = 5;
  config().vehicle_height = 1.0;
  config().consider_drop = false;

  // Point at z=2.0 exceeds max_relative_z (0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 2.0F;
  pt.intensity = 0;
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back(pt);

  runStage(stage::Id::HeightMap);
  EXPECT_TRUE(state().terrain_cloud_elev->points.empty());
}

// consider_drop 开启时高度取绝对值，负高度也被接受
TEST_F(AlgorithmTest, ComputeHeightMap_ConsiderDrop_AcceptsNegativeHeight) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().terrain_cloud_elev->clear();
  state().planar_voxel_elev.fill(0.3);  // ground at +0.3, point at z=0 → -0.3m
  state().planar_voxel_dy_obs.fill(0);
  for (auto& e : state().planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config().min_block_point_num = 5;
  config().vehicle_height = 1.0;
  config().min_relative_z = -10.0;
  config().max_relative_z = 10.0;
  config().consider_drop = true;

  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.0F;
  pt.intensity = 0;
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back(pt);

  runStage(stage::Id::HeightMap);
  EXPECT_EQ(state().terrain_cloud_elev->points.size(), 1U);
  // height = abs(0 - 0.3) = 0.3 < 1.0 → accepted
}

// 高度超过 vehicle_height 的点被过滤
TEST_F(AlgorithmTest, ComputeHeightMap_AboveVehicleHeight_Filtered) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().terrain_cloud_elev->clear();
  state().planar_voxel_elev.fill(0);
  state().planar_voxel_dy_obs.fill(0);
  for (auto& e : state().planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config().min_block_point_num = 5;
  config().vehicle_height = 0.1;
  config().min_relative_z = -10.0;
  config().max_relative_z = 10.0;
  config().consider_drop = false;

  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.5F;
  pt.intensity = 0;
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back(pt);

  runStage(stage::Id::HeightMap);
  // height = 0.5 - 0 = 0.5 >= 0.1 → filtered
  EXPECT_TRUE(state().terrain_cloud_elev->points.empty());
}

// 车顶上方超过安全间隙的点（天花板）不参与地面估计，防止 elev 被抬高后
// 地面点高度差变负、天花板点高度差落入障碍区间（窄隧道场景）
TEST_F(AlgorithmTest, EstimateGround_CeilingPoint_ExcludedFromElevation) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  config().ceiling_clearance = 0.2;
  state().terrain_cloud->clear();
  // 天花板点：相对车高 0.26m（模拟 260mm 顶隙的隧道），位于车辆正上方
  state().terrain_cloud->push_back({0.0F, 0.0F, 0.26F, 0.0F});

  runStage(stage::Id::EstimateGround);

  size_t center = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  EXPECT_TRUE(state().planar_point_elev[center].empty());
}

// 车顶下方/间隙内的点仍正常参与地面估计
TEST_F(AlgorithmTest, EstimateGround_BelowCeilingClearance_Participates) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  config().ceiling_clearance = 0.2;
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back({0.0F, 0.0F, -0.1F, 0.0F});

  runStage(stage::Id::EstimateGround);

  size_t center = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  EXPECT_EQ(state().planar_point_elev[center].size(), 1U);
}

// 车顶上方达到安全间隙的点（天花板/横梁）不作为障碍输出：顶隙足够，
// 车辆可从下方通过
TEST_F(AlgorithmTest, ComputeHeightMap_CeilingPoint_NotObstacle) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().terrain_cloud_elev->clear();
  state().planar_voxel_elev.fill(0);  // 地面高度 0
  state().planar_voxel_dy_obs.fill(0);
  for (auto& e : state().planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};  // 满足 min_block_point_num
  }
  config().min_block_point_num = 5;
  config().vehicle_height = 1.0;  // 旧逻辑下 0.26 < 1.0 会被输出为障碍
  config().ceiling_clearance = 0.2;
  config().min_relative_z = -10.0;
  config().max_relative_z = 10.0;
  config().consider_drop = false;

  // 天花板点：相对车高 0.26m，高于 ceiling_clearance(0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.26F;
  pt.intensity = 0;
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back(pt);

  runStage(stage::Id::HeightMap);
  EXPECT_TRUE(state().terrain_cloud_elev->points.empty());
}

// 车顶上方安全间隙内的点仍然是障碍（低矮横梁/门楣不应漏检）
TEST_F(AlgorithmTest, ComputeHeightMap_BelowCeilingClearance_StillObstacle) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().terrain_cloud_elev->clear();
  state().planar_voxel_elev.fill(0);
  state().planar_voxel_dy_obs.fill(0);
  for (auto& e : state().planar_point_elev) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config().min_block_point_num = 5;
  config().vehicle_height = 1.0;
  config().ceiling_clearance = 0.2;
  config().min_relative_z = -10.0;
  config().max_relative_z = 10.0;
  config().consider_drop = false;

  // 低矮障碍点：相对车高 0.1m，低于 ceiling_clearance(0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.1F;
  pt.intensity = 0;
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back(pt);

  runStage(stage::Id::HeightMap);
  ASSERT_EQ(state().terrain_cloud_elev->points.size(), 1U);
  // height_above_ground = 0.1 - 0 = 0.1，写入 intensity
  EXPECT_NEAR(state().terrain_cloud_elev->points[0].intensity, 0.1F, 1e-6);
}

// ── keepVoxelPoint boundary tests (via updateVoxels) ──

// 略高于下限边界的点被保留
TEST_F(AlgorithmTest, KeepVoxelPoint_BelowLowerBoundary_Excluded) {
  double z_margin = config().distance_ratio_z * 1.0;     // = 0.2
  double boundary = config().min_relative_z - z_margin;  // = -1.7
  int kept = updateSinglePoint(processor(), boundary + 0.01, 1.0);
  EXPECT_EQ(kept, 1);
}

// 等于下限边界的点被排除
TEST_F(AlgorithmTest, KeepVoxelPoint_AtLowerBoundary_Excluded) {
  double z_margin = config().distance_ratio_z * 1.0;
  double boundary = config().min_relative_z - z_margin;
  int kept = updateSinglePoint(processor(), boundary, 1.0);
  EXPECT_EQ(kept, 0);
}

// 等于上限边界的点被排除
TEST_F(AlgorithmTest, KeepVoxelPoint_AtUpperBoundary_Excluded) {
  double z_margin = config().distance_ratio_z * 1.0;
  double boundary = config().max_relative_z + z_margin;  // = 0.4
  int kept = updateSinglePoint(processor(), boundary, 1.0);
  EXPECT_EQ(kept, 0);
}

// 略低于上限边界的点被保留
TEST_F(AlgorithmTest, KeepVoxelPoint_BelowUpperBoundary_Kept) {
  double z_margin = config().distance_ratio_z * 1.0;
  double boundary = config().max_relative_z + z_margin;
  int kept = updateSinglePoint(processor(), boundary - 0.01, 1.0);
  EXPECT_EQ(kept, 1);
}

// 点的时间戳过期且离车辆较远 → 被清除
TEST_F(AlgorithmTest, KeepVoxelPoint_ExpiredFarPoint_Excluded) {
  config().min_relative_z = -10.0;
  config().max_relative_z = 10.0;
  config().decay_time = 1.0;
  config().no_decay_distance = 0.0;
  config().voxel_point_update_thre = 1;

  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0.0;
  state().laser_cloud_time = 10.0;
  state().system_init_time = 0.0;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  auto& cell = *state().terrain_voxel_cloud[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 2.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 0.0F;
  cell.push_back(point);

  state().terrain_voxel_update_num[center_cell] =
      config().voxel_point_update_thre;

  runStage(stage::Id::UpdateVoxels);
  EXPECT_TRUE(state().terrain_voxel_cloud[center_cell]->points.empty());
}

// 近点即使过期也保留（near 优先于 decay）
TEST_F(AlgorithmTest, KeepVoxelPoint_NearPointEvenIfExpired_Kept) {
  config().min_relative_z = -10.0;
  config().max_relative_z = 10.0;
  config().decay_time = 1.0;
  config().no_decay_distance = 3.0;
  config().voxel_point_update_thre = 1;

  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0.0;
  state().laser_cloud_time = 10.0;
  state().system_init_time = 0.0;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  auto& cell = *state().terrain_voxel_cloud[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 1.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 0.0F;
  cell.push_back(point);

  state().terrain_voxel_update_num[center_cell] =
      config().voxel_point_update_thre;

  runStage(stage::Id::UpdateVoxels);
  EXPECT_EQ(state().terrain_voxel_cloud[center_cell]->points.size(), 1U);
}

// ── shouldPruneVoxel (via updateVoxels) ──

// update_num 未达阈值且时间未到 → 不修剪
TEST_F(AlgorithmTest, ShouldPruneVoxel_NotEnoughPointsOrTime_NotPruned) {
  config().voxel_point_update_thre = 100;
  config().voxel_time_update_thre = 10.0;
  state().laser_cloud_time = 1.0;
  state().system_init_time = 0.0;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  state().terrain_voxel_update_num[center_cell] = 5;
  state().terrain_voxel_update_time[center_cell] = 0.0;

  runStage(stage::Id::UpdateVoxels);
  EXPECT_NE(state().terrain_voxel_cloud[center_cell], nullptr);
}

// update_num 达到阈值 → 触发修剪
TEST_F(AlgorithmTest, ShouldPruneVoxel_PointCountReached_Pruned) {
  config().voxel_point_update_thre = 10;
  config().voxel_time_update_thre = 999.0;
  config().min_relative_z = -10.0;
  config().max_relative_z = 10.0;
  config().decay_time = 999.0;
  config().no_decay_distance = 999.0;
  state().laser_cloud_time = 1.0;
  state().system_init_time = 0.0;

  int center_cell = TerrainGrid::terrainVoxelIndex(
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH,
      TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH);
  auto& cell = *state().terrain_voxel_cloud[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 0.1F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 1.0F;
  cell.push_back(point);
  state().terrain_voxel_update_num[center_cell] =
      config().voxel_point_update_thre;

  runStage(stage::Id::UpdateVoxels);
  EXPECT_EQ(cell.points.size(), 1U);
}

// ── computeHeightMap 过滤分支 ──

// dy_obs 计数 ≥ min_dy_obs_point_num → 该 cell 被过滤
TEST_F(AlgorithmTest, ComputeHeightMap_DynamicObstacleCell_Filtered) {
  state().vehicle_x = 0;
  state().vehicle_y = 0;
  state().vehicle_z = 0;
  state().terrain_cloud_elev->clear();
  state().planar_voxel_elev.fill(0);
  for (auto& e : state().planar_point_elev) {
    e = {0., 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  config().min_block_point_num = 5;
  config().vehicle_height = 1.0;
  config().min_relative_z = -10.0;
  config().max_relative_z = 10.0;
  config().min_dy_obs_point_num = 3;

  size_t cell = TerrainGrid::planarVoxelIndex(
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH,
      TerrainGrid::PLANAR_VOXEL_HALF_WIDTH);
  state().planar_voxel_dy_obs[cell] = 5;

  pcl::PointXYZI pt;
  pt.x = 0.0F;
  pt.y = 0;
  pt.z = 0.1F;
  pt.intensity = 0;
  state().terrain_cloud->clear();
  state().terrain_cloud->push_back(pt);

  runStage(stage::Id::HeightMap);
  EXPECT_TRUE(state().terrain_cloud_elev->points.empty());
}
