#include "terrain_analysis/core/grid.hpp"
#include "terrain_analysis/core/per_frame_height_map.hpp"
#include "terrain_analysis/core/persistent_voxel_map.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

// 白盒测试需要逐阶段驱动两半管线；fixture 是 PersistentVoxelMap /
// PerFrameHeightMap 的 friend，因此把这类调用收在它以内的分发器里。
namespace stage {
  enum class Id {
    ROLLOVER,
    VOXELIZE,
    UPDATE_TERRAIN_VOXELS,
    COLLECT,
    ESTIMATE_TERRAIN_GROUND,
    PLANAR_ELEVATION,
    HEIGHT_MAP
  };
}  // namespace stage

namespace terrain_analysis {
  class AlgorithmTest : public testing::Test {
  protected:
    AlgorithmTest() {
      resetState();
      heightConfig().use_sorting = true;
      heightConfig().quantile_z = 0.25;
      heightConfig().limit_ground_lift = false;
    }

    void resetState() {
      height_map_ = std::make_unique<PerFrameHeightMap>();
      voxel_map_ = std::make_unique<PersistentVoxelMap>();
      for (auto& ptr : voxelCells()) {
        ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      }
      terrain_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }

    /**
     * @brief 放宽高度带：前半段的接收/保留带与后半段的输出地板一起放宽。
     *
     * 拆成两半之后这三个字段分属两个配置，但都来自同一个 minRelZ/maxRelZ 参数，
     * 所以测试里仍然一起设，语义与拆分前一致。
     */
    void widenBand(double lower, double upper) {
      voxelConfig().min_relative_z = lower;
      voxelConfig().max_relative_z = upper;
      heightConfig().min_relative_z = lower;
    }

    void runStage(stage::Id id) {
      switch (id) {
        case stage::Id::ROLLOVER:
          voxelMap().rollover(lidarPosition(), voxelConfig());
          break;
        case stage::Id::VOXELIZE:
          voxelMap().addFrame(*frameCloud(), lidarPosition(), voxelConfig());
          break;
        case stage::Id::UPDATE_TERRAIN_VOXELS:
          voxelMap().rebuild(lidarPosition(), elapsed(), voxelConfig());
          break;
        case stage::Id::COLLECT:
          voxelMap().collectCloud(*terrainCloud());
          break;
        case stage::Id::ESTIMATE_TERRAIN_GROUND:
          heightMap().estimateTerrainGround(*terrainCloud(), lidarPosition(),
                                            heightConfig());
          break;
        case stage::Id::PLANAR_ELEVATION:
          heightMap().computePlanarElevation(heightConfig());
          break;
        case stage::Id::HEIGHT_MAP:
          heightMap().computeHeightMap(*terrainCloud(), lidarPosition(),
                                       heightConfig());
          break;
      }
    }

    // 下面这些是"内部字段的引用访问器"：friend 只授予 fixture 的成员函数，
    // 测试体本身没有访问权，故统一经这里取引用。
    guga_common::Point3d& lidar() {
      return voxel_map_->lidar_;
    }
    double& frameTime() {
      return voxel_map_->time_;
    }
    double& initTime() {
      return voxel_map_->init_time_;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& frameCloud() {
      return voxel_map_->frame_cloud_;
    }
    std::array<double, PerFrameHeightGrid::NUM>& voxelElev() {
      return height_map_->voxel_elev_;
    }
    std::array<std::vector<double>, PerFrameHeightGrid::NUM>& pointElev() {
      return height_map_->point_elev_;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacleCloud() {
      return height_map_->obstacle_cloud_;
    }

    PerFrameHeightMap& heightMap() {
      return *height_map_;
    }
    int shiftX() const {
      return voxel_map_->shiftX();
    }
    int shiftY() const {
      return voxel_map_->shiftY();
    }
    std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr, PersistentVoxelGrid::NUM>&
    voxelCells() {
      return voxel_map_->cells();
    }
    PersistentVoxelMap& voxelMap() {
      return *voxel_map_;
    }
    PersistentVoxelConfig& voxelConfig() {
      return voxel_config_;
    }
    PerFrameHeightConfig& heightConfig() {
      return height_config_;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr& terrainCloud() {
      return terrain_cloud_;
    }
    guga_common::Point3d lidarPosition() const {
      return voxel_map_->lidar_;
    }
    double elapsed() const {
      return voxel_map_->time_ - voxel_map_->init_time_;
    }

    // 把一个点放进体素网格的中心格，再触发重建。返回该格保留的点数。
    int updateSinglePoint(double relative_z, double distance) {
      PersistentVoxelConfig& config = voxelConfig();
      config.min_relative_z = -1.5;
      config.max_relative_z = 0.2;
      config.distance_ratio_z = 0.2;
      config.decay_time = 999.0;
      config.no_decay_distance = 999.0;

      voxel_map_->lidar_.x = 0;
      voxel_map_->lidar_.y = 0;
      voxel_map_->lidar_.z = 0.0;
      voxel_map_->time_ = 1.0;
      voxel_map_->init_time_ = 0.0;

      int center_cell = PersistentVoxelGrid::linearIndex(
          PersistentVoxelGrid::HALF_WIDTH, PersistentVoxelGrid::HALF_WIDTH);
      auto& cell = *voxelCells()[center_cell];
      cell.clear();
      pcl::PointXYZI point;
      point.x = static_cast<float>(distance);
      point.y = 0.0F;
      point.z = static_cast<float>(relative_z);
      point.intensity = 0.0F;
      cell.push_back(point);

      voxelMap().rebuild(lidarPosition(), elapsed(), voxelConfig());
      return static_cast<int>(voxelCells()[center_cell]->points.size());
    }

    /** @brief 两份配置由 fixture 持有并注入；两半自己不持有配置。 */
    PersistentVoxelConfig voxel_config_;
    PerFrameHeightConfig height_config_;
    std::unique_ptr<PerFrameHeightMap> height_map_;
    std::unique_ptr<PersistentVoxelMap> voxel_map_;
    /** @brief 两半之间的交接数据（采集点云），由测试持有。 */
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
  };
}  // namespace terrain_analysis

using terrain_analysis::AlgorithmTest;

// 雷达未移动时，体素网格不发生滚动
TEST_F(AlgorithmTest, RolloverVoxelMap_Stationary_NoShift) {
  lidar().x = 0;
  lidar().y = 0;
  int sx = shiftX();
  int sy = shiftY();

  runStage(stage::Id::ROLLOVER);

  EXPECT_EQ(shiftX(), sx);
  EXPECT_EQ(shiftY(), sy);
}

// 雷达向左超出 voxel 范围时，沿 X 负向滚动一格
TEST_F(AlgorithmTest, RolloverVoxelMap_LeftOfCenter_ShiftsXNegative) {
  lidar().x = -2.0;
  int sx = shiftX();

  runStage(stage::Id::ROLLOVER);

  EXPECT_EQ(shiftX(), sx - 1);
}

// 雷达向右超出 voxel 范围时，沿 X 正向滚动一格
TEST_F(AlgorithmTest, RolloverVoxelMap_RightOfCenter_ShiftsXPositive) {
  lidar().x = 2.0;
  int sx = shiftX();

  runStage(stage::Id::ROLLOVER);

  EXPECT_EQ(shiftX(), sx + 1);
}

// 雷达向下超出 voxel 范围时，沿 Y 负向滚动一格
TEST_F(AlgorithmTest, RolloverVoxelMap_BelowCenter_ShiftsYNegative) {
  lidar().y = -2.0;
  int sy = shiftY();

  runStage(stage::Id::ROLLOVER);

  EXPECT_EQ(shiftY(), sy - 1);
}

// 雷达向上超出 voxel 范围时，沿 Y 正向滚动一格
TEST_F(AlgorithmTest, RolloverVoxelMap_AboveCenter_ShiftsYPositive) {
  lidar().y = 2.0;
  int sy = shiftY();

  runStage(stage::Id::ROLLOVER);

  EXPECT_EQ(shiftY(), sy + 1);
}

// 滚动后目标 cell 被清空，原有数据随 shift 迁移
TEST_F(AlgorithmTest, RolloverVoxelMap_ShiftLeft_PreservesDataFromShiftedCell) {
  lidar().x = -2.0;
  voxelCells()[0]->clear();
  pcl::PointXYZI p{0, 0, 0, 0};
  voxelCells()[0]->push_back(p);

  runStage(stage::Id::ROLLOVER);

  // After shift-left, voxel(0,0) becomes the destination cell and gets cleared
  EXPECT_TRUE(voxelCells()[0]->points.empty());
}

// 雷达同时向左下方移动，X 和 Y 各滚动一格
TEST_F(AlgorithmTest, RolloverVoxelMap_LeftAndDown_ShiftsBothAxes) {
  lidar().x = -2.0;
  lidar().y = -2.0;
  int sx = shiftX();
  int sy = shiftY();

  runStage(stage::Id::ROLLOVER);

  EXPECT_EQ(shiftX(), sx - 1);
  EXPECT_EQ(shiftY(), sy - 1);
}

// ── voxelizeTerrain ──
// 原点处的单个点被分配到网格正中的 cell
TEST_F(AlgorithmTest, Voxelize_MapsPointToCenterCell) {
  lidar().x = 0;
  lidar().y = 0;
  frameCloud()->clear();
  frameCloud()->push_back({0, 0, 0, 0});

  runStage(stage::Id::VOXELIZE);

  size_t center = PersistentVoxelGrid::linearIndex(
      PersistentVoxelGrid::HALF_WIDTH, PersistentVoxelGrid::HALF_WIDTH);
  EXPECT_EQ(voxelCells()[center]->points.size(), 1U);
}

// 超出体素网格范围的点不进入任何格
TEST_F(AlgorithmTest, Voxelize_PointOutsideGrid_Dropped) {
  lidar().x = 0;
  lidar().y = 0;
  frameCloud()->clear();
  frameCloud()->push_back(
      {50.0F, 50.0F, 0.0F, 0.0F});  // 远超 21×21 的 1 m 网格

  runStage(stage::Id::VOXELIZE);

  size_t total = 0;
  for (const auto& cell : voxelCells()) {
    total += cell->points.size();
  }
  EXPECT_EQ(total, 0U);
}

// 空点云不产生任何体素分配
TEST_F(AlgorithmTest, Voxelize_EmptyCloud_NoChange) {
  frameCloud()->clear();

  runStage(stage::Id::VOXELIZE);

  for (int i = 0; i < PersistentVoxelGrid::NUM; i++) {
    EXPECT_TRUE(voxelCells()[i]->points.empty());
  }
}

// ── computePlanarElevation ──
// 排序模式下取指定分位数作为地面高度估计
TEST_F(AlgorithmTest, ComputeElevation_UseSorting_ReturnsQuantile) {
  heightConfig().use_sorting = true;
  heightConfig().quantile_z = 0.5;
  size_t cell = PerFrameHeightGrid::linearIndex(PerFrameHeightGrid::HALF_WIDTH,
                                                PerFrameHeightGrid::HALF_WIDTH);
  voxelElev().fill(999);
  pointElev()[cell] = {0.1, 0.5, 0.3, 0.2, 0.4};

  runStage(stage::Id::PLANAR_ELEVATION);

  // sorted: 0.1, 0.2, 0.3, 0.4, 0.5. quantile 0.5*(5) = 2 → index 2 → 0.3
  EXPECT_FLOAT_EQ(voxelElev()[cell], 0.3F);
}

// 最小值模式下取最低点作为地面高度估计
TEST_F(AlgorithmTest, ComputeElevation_UseMinimum_ReturnsMinimum) {
  heightConfig().use_sorting = false;
  size_t cell = PerFrameHeightGrid::linearIndex(PerFrameHeightGrid::HALF_WIDTH,
                                                PerFrameHeightGrid::HALF_WIDTH);
  voxelElev().fill(999);
  pointElev()[cell] = {1.5, 0.5, 1.0};

  runStage(stage::Id::PLANAR_ELEVATION);

  EXPECT_FLOAT_EQ(voxelElev()[cell], 0.5F);
}

// 分位数与最小值差距过大时，限制地面高度不超过 min+max_ground_lift
TEST_F(AlgorithmTest,
       ComputeElevation_LiftLimited_CapsAtMinimumPlusMaxGroundLift) {
  heightConfig().use_sorting = true;
  heightConfig().quantile_z = 0.5;
  heightConfig().limit_ground_lift = true;
  heightConfig().max_ground_lift = 0.3;
  size_t cell = PerFrameHeightGrid::linearIndex(PerFrameHeightGrid::HALF_WIDTH,
                                                PerFrameHeightGrid::HALF_WIDTH);
  voxelElev().fill(999);
  // sorted: 0.5, 1.0, 2.0. quantile 0.5*3 = 1 → 1.0. diff 1.0-0.5=0.5 > 0.3
  pointElev()[cell] = {0.5, 2.0, 1.0};

  runStage(stage::Id::PLANAR_ELEVATION);

  // lift limited → 0.5 + 0.3 = 0.8
  EXPECT_FLOAT_EQ(voxelElev()[cell], 0.8F);
}

// quantile_z=1.0 时 quantile_index 达到 point_count 边界，回退到最后一点
TEST_F(AlgorithmTest, ComputeElevation_QuantileIndexAtBoundary_ClampedToLast) {
  heightConfig().use_sorting = true;
  heightConfig().quantile_z = 1.0;
  heightConfig().limit_ground_lift = false;
  size_t cell = PerFrameHeightGrid::linearIndex(PerFrameHeightGrid::HALF_WIDTH,
                                                PerFrameHeightGrid::HALF_WIDTH);
  voxelElev().fill(999);
  // 3 points: sorted 0.1, 0.3, 0.9. quantile 1.0*3 = 3 >= 3 → clamp to 2 → 0.9
  pointElev()[cell] = {0.1, 0.9, 0.3};

  runStage(stage::Id::PLANAR_ELEVATION);

  EXPECT_FLOAT_EQ(voxelElev()[cell], 0.9F);
}
// ── estimateTerrainGround ──

// 栅格边缘点触发射线邻居越界检查，不崩溃
TEST_F(AlgorithmTest, EstimateTerrainGround_EdgePoint_HandlesOobNeighbors) {
  constexpr double SZ = 0.2;
  double edge = SZ * (25 - 1);  // ~4.8m, column=49, delta_col+1=50 in bounds
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  terrainCloud()->clear();
  pcl::PointXYZI pt;
  pt.x = static_cast<float>(edge);
  pt.y = 0;
  pt.z = 0;
  pt.intensity = 0;
  terrainCloud()->push_back(pt);

  runStage(stage::Id::ESTIMATE_TERRAIN_GROUND);
  // No crash = pass; point Z=0 is within min/max relative_z range
  EXPECT_TRUE(true);
}

// 超出 planar grid 的点被跳过，避免在计算 base index 时越界
TEST_F(AlgorithmTest, EstimateTerrainGround_PointOutsidePlanarGrid_Ignored) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  terrainCloud()->clear();
  terrainCloud()->push_back({6.0F, 0.0F, 0.0F, 0.0F});

  runStage(stage::Id::ESTIMATE_TERRAIN_GROUND);

  for (const auto& elevations : pointElev()) {
    EXPECT_TRUE(elevations.empty());
  }
}

// ── computeHeightMap ──

// Z 超出范围的点被过滤
TEST_F(AlgorithmTest, ComputeHeightMap_PointOutOfZRange_Filtered) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  obstacleCloud()->clear();
  voxelElev().fill(0);
  for (auto& e : pointElev()) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  heightConfig().min_block_point_num = 5;
  heightConfig().consider_drop = false;

  // Point at z=2.0 exceeds max_relative_z (0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 2.0F;
  pt.intensity = 0;
  terrainCloud()->clear();
  terrainCloud()->push_back(pt);

  runStage(stage::Id::HEIGHT_MAP);
  EXPECT_TRUE(obstacleCloud()->points.empty());
}

// consider_drop 开启时高度取绝对值，负高度也被接受
TEST_F(AlgorithmTest, ComputeHeightMap_ConsiderDrop_AcceptsNegativeHeight) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  obstacleCloud()->clear();
  voxelElev().fill(0.3);  // ground at +0.3, point at z=0 → -0.3m
  for (auto& e : pointElev()) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  heightConfig().min_block_point_num = 5;
  widenBand(-10.0, 10.0);
  heightConfig().consider_drop = true;

  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.0F;
  pt.intensity = 0;
  terrainCloud()->clear();
  terrainCloud()->push_back(pt);

  runStage(stage::Id::HEIGHT_MAP);
  EXPECT_EQ(obstacleCloud()->points.size(), 1U);
  // height = abs(0 - 0.3) = 0.3 < 1.0 → accepted
}

// 地面带死区：距地面小于 min_obstacle_height 的点不作为障碍输出
// 远低于雷达的穿透点被地板判据挡掉，不进入输出
TEST_F(AlgorithmTest, ComputeHeightMap_BelowLidarFloor_Filtered) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  obstacleCloud()->clear();
  voxelElev().fill(0);
  for (auto& e : pointElev()) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  heightConfig().min_block_point_num = 5;
  heightConfig().consider_drop = false;

  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = -2.0F;  // 比 minRelZ(-1.5) 还低：穿透点
  pt.intensity = 0;
  terrainCloud()->clear();
  terrainCloud()->push_back(pt);

  runStage(stage::Id::HEIGHT_MAP);

  EXPECT_TRUE(obstacleCloud()->points.empty())
      << "低于雷达 minRelZ 的点不应输出";
}

TEST_F(AlgorithmTest, ComputeHeightMap_BelowMinObstacleHeight_Filtered) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  obstacleCloud()->clear();
  voxelElev().fill(0);
  for (auto& e : pointElev()) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  heightConfig().min_block_point_num = 5;
  widenBand(-10.0, 10.0);
  heightConfig().consider_drop = false;
  heightConfig().min_obstacle_height = 0.04;

  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.02F;  // 距地面 0.02 < 0.04：落在死区内
  pt.intensity = 0;
  terrainCloud()->clear();
  terrainCloud()->push_back(pt);

  runStage(stage::Id::HEIGHT_MAP);
  EXPECT_TRUE(obstacleCloud()->points.empty());
}

// 车高与净空之间的点仍然是障碍（车过不去，不能漏检）
// 旧实现有一条 `height < vehicle_height(0.52)` 的截断，会把 0.52~0.62 之间的点
// 一并丢掉；该截断已删除，ceiling_clearance 是唯一上界。
TEST_F(AlgorithmTest,
       ComputeHeightMap_BetweenVehicleHeightAndCeiling_Obstacle) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  obstacleCloud()->clear();
  voxelElev().fill(0);
  for (auto& e : pointElev()) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  heightConfig().min_block_point_num = 5;
  widenBand(-10.0, 10.0);
  heightConfig().consider_drop = false;
  heightConfig().ceiling_clearance = 0.62;  // 车高 0.52 + 0.10

  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.55F;  // 0.52 < 0.55 < 0.62：旧实现会丢弃，现在必须输出
  pt.intensity = 0;
  terrainCloud()->clear();
  terrainCloud()->push_back(pt);

  runStage(stage::Id::HEIGHT_MAP);
  ASSERT_EQ(obstacleCloud()->points.size(), 1U);
  EXPECT_NEAR(obstacleCloud()->points[0].intensity, 0.55, 1e-5);
}

// 高于净空的点同样参与地面估计：净空筛选已从本阶段移除（见下方断言注释）
TEST_F(AlgorithmTest,
       EstimateTerrainGround_AboveCeilingClearance_StillParticipates) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  terrainCloud()->clear();
  // 高于 ceiling_clearance 的点**现在也参与**地面估计——净空判据已从本
  // 阶段移除，只保留在 computeHeightMap（障碍输出）。
  // 理由：净空是"障碍能否通过"的判据，与"哪些点属于地面"无关；留在这里会按
  // 车高砍掉抬升的地面（坡面），并让候选数随车高漂移、经分位数放大成 elev
  // 偏差。 新暴露的风险：隧道天花板若未被 ingest 的高度过滤挡下，会抬高 elev
  // 使真实 地面点丢失——实车偏置下 ingest 上界(z≈0.27)已先挡掉，故暂不构成问题。
  terrainCloud()->push_back({0.0F, 0.0F, 0.26F, 0.0F});

  runStage(stage::Id::ESTIMATE_TERRAIN_GROUND);

  size_t center = PerFrameHeightGrid::linearIndex(
      PerFrameHeightGrid::HALF_WIDTH, PerFrameHeightGrid::HALF_WIDTH);
  EXPECT_EQ(pointElev()[center].size(), 1U);
}

// 车顶下方/间隙内的点仍正常参与地面估计
TEST_F(AlgorithmTest,
       EstimateTerrainGround_BelowCeilingClearance_Participates) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  terrainCloud()->clear();
  // 点距地面 -0.1m（planar_voxel_elev 为 0）：在地板之上、低于
  // CEILING_CLEARANCE
  terrainCloud()->push_back({0.0F, 0.0F, -0.1F, 0.0F});

  runStage(stage::Id::ESTIMATE_TERRAIN_GROUND);

  size_t center = PerFrameHeightGrid::linearIndex(
      PerFrameHeightGrid::HALF_WIDTH, PerFrameHeightGrid::HALF_WIDTH);
  EXPECT_EQ(pointElev()[center].size(), 1U);
}

// 车顶上方达到安全间隙的点（天花板/横梁）不作为障碍输出：顶隙足够，
// 车辆可从下方通过
TEST_F(AlgorithmTest, ComputeHeightMap_CeilingPoint_NotObstacle) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  obstacleCloud()->clear();
  voxelElev().fill(0);  // 地面高度 0
  for (auto& e : pointElev()) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};  // 满足 min_block_point_num
  }
  heightConfig().min_block_point_num = 5;
  widenBand(-10.0, 10.0);
  heightConfig().consider_drop = false;
  heightConfig().ceiling_clearance =
      0.2;  // 显式设定，不依赖默认值（随车高而异）

  // 天花板点：距地面 0.26m（planar_voxel_elev=0），高于 ceiling_clearance(0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.26F;
  pt.intensity = 0;
  terrainCloud()->clear();
  terrainCloud()->push_back(pt);

  runStage(stage::Id::HEIGHT_MAP);
  EXPECT_TRUE(obstacleCloud()->points.empty());
}

// 车顶上方安全间隙内的点仍然是障碍（低矮横梁/门楣不应漏检）
TEST_F(AlgorithmTest, ComputeHeightMap_BelowCeilingClearance_StillObstacle) {
  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0;
  obstacleCloud()->clear();
  voxelElev().fill(0);
  for (auto& e : pointElev()) {
    e = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  }
  heightConfig().min_block_point_num = 5;
  widenBand(-10.0, 10.0);
  heightConfig().consider_drop = false;
  heightConfig().ceiling_clearance =
      0.2;  // 显式设定，不依赖默认值（随车高而异）

  // 低矮障碍点：距地面 0.05m（planar_voxel_elev=0），低于
  // ceiling_clearance(0.2)
  pcl::PointXYZI pt;
  pt.x = 0.5F;
  pt.y = 0;
  pt.z = 0.05F;
  pt.intensity = 0;
  terrainCloud()->clear();
  terrainCloud()->push_back(pt);

  runStage(stage::Id::HEIGHT_MAP);
  ASSERT_EQ(obstacleCloud()->points.size(), 1U);
  // height_above_ground = 0.05 - 0 = 0.05，写入 intensity
  EXPECT_NEAR(obstacleCloud()->points[0].intensity, 0.05F, 1e-6);
}

// ── keepPoint 边界测试（经 rebuild）──

// 略高于下限边界的点被保留
TEST_F(AlgorithmTest, KeepVoxelPoint_BelowLowerBoundary_Excluded) {
  double z_margin = voxelConfig().distance_ratio_z * 1.0;     // = 0.2
  double boundary = voxelConfig().min_relative_z - z_margin;  // = -1.7
  int kept = updateSinglePoint(boundary + 0.01, 1.0);
  EXPECT_EQ(kept, 1);
}

// 等于下限边界的点被排除
TEST_F(AlgorithmTest, KeepVoxelPoint_AtLowerBoundary_Excluded) {
  double z_margin = voxelConfig().distance_ratio_z * 1.0;
  double boundary = voxelConfig().min_relative_z - z_margin;
  int kept = updateSinglePoint(boundary, 1.0);
  EXPECT_EQ(kept, 0);
}

// 等于上限边界的点被排除
TEST_F(AlgorithmTest, KeepVoxelPoint_AtUpperBoundary_Excluded) {
  double z_margin = voxelConfig().distance_ratio_z * 1.0;
  double boundary = voxelConfig().max_relative_z + z_margin;  // = 0.4
  int kept = updateSinglePoint(boundary, 1.0);
  EXPECT_EQ(kept, 0);
}

// 略低于上限边界的点被保留
TEST_F(AlgorithmTest, KeepVoxelPoint_BelowUpperBoundary_Kept) {
  double z_margin = voxelConfig().distance_ratio_z * 1.0;
  double boundary = voxelConfig().max_relative_z + z_margin;
  int kept = updateSinglePoint(boundary - 0.01, 1.0);
  EXPECT_EQ(kept, 1);
}

// 点的时间戳过期且离雷达较远 → 被清除
TEST_F(AlgorithmTest, KeepVoxelPoint_ExpiredFarPoint_Excluded) {
  widenBand(-10.0, 10.0);
  voxelConfig().decay_time = 1.0;
  voxelConfig().no_decay_distance = 0.0;

  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0.0;
  frameTime() = 10.0;
  initTime() = 0.0;

  int center_cell = PersistentVoxelGrid::linearIndex(
      PersistentVoxelGrid::HALF_WIDTH, PersistentVoxelGrid::HALF_WIDTH);
  auto& cell = *voxelCells()[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 2.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 0.0F;
  cell.push_back(point);

  runStage(stage::Id::UPDATE_TERRAIN_VOXELS);
  EXPECT_TRUE(voxelCells()[center_cell]->points.empty());
}

// 叶内混有新老观测时，代表点必须取"最新"的那个点：
// 旧实现交给 PCL VoxelGrid 取质心、对 intensity 取平均，这个仍被观测到的表面
// 会被平均时刻判成过期而删除。
TEST_F(AlgorithmTest, UpdateVoxels_MixedAgeLeaf_KeepsNewestObservation) {
  widenBand(-10.0, 10.0);
  voxelConfig().decay_time = 0.5;
  voxelConfig().no_decay_distance = 0.0;
  voxelConfig().scan_voxel_size = 0.05;
  voxelConfig().scan_voxel_size_z = 0.05;

  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0.0;
  frameTime() = 1.2;  // 本帧
  initTime() = 0.0;

  int center_cell = PersistentVoxelGrid::linearIndex(
      PersistentVoxelGrid::HALF_WIDTH, PersistentVoxelGrid::HALF_WIDTH);
  auto& cell = *voxelCells()[center_cell];
  cell.clear();
  // 同一 0.05 m 叶内的三点：两个早已过期，一个本帧刚观测到
  const double times[] = {0.10, 0.30, 1.20};
  for (double time : times) {
    pcl::PointXYZI point;
    point.x = 1.0F;
    point.y = 0.0F;
    point.z = 0.0F;
    point.intensity = static_cast<float>(time);
    cell.push_back(point);
  }

  runStage(stage::Id::UPDATE_TERRAIN_VOXELS);

  ASSERT_EQ(voxelCells()[center_cell]->points.size(), 1U);
  EXPECT_FLOAT_EQ(voxelCells()[center_cell]->points[0].intensity, 1.20F)
      << "代表点必须是本帧的观测，而不是叶内的某个平均时刻";
}

// 异性叶：地面点与矮物体点在水平方向只差几厘米，但垂直叶更细时必须分开成
// 两个叶。若两者同叶，每叶只留最新观测点，地面点会把矮物体点顶掉（水平/垂直
// 都取 0.1 m 时实测 6 cm 矮台阶输出归零）。
TEST_F(AlgorithmTest, UpdateVoxels_AnisotropicLeaf_KeepsLowObstacle) {
  widenBand(-10.0, 10.0);
  voxelConfig().decay_time = 999.0;
  voxelConfig().no_decay_distance = 999.0;
  voxelConfig().scan_voxel_size = 0.1;     // 水平
  voxelConfig().scan_voxel_size_z = 0.05;  // 垂直

  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0.0;
  frameTime() = 1.0;
  initTime() = 0.0;

  int center_cell = PersistentVoxelGrid::linearIndex(
      PersistentVoxelGrid::HALF_WIDTH, PersistentVoxelGrid::HALF_WIDTH);
  auto& cell = *voxelCells()[center_cell];
  cell.clear();
  pcl::PointXYZI ground;  // 地面点，本帧观测到
  ground.x = 2.0F;
  ground.y = 0.0F;
  ground.z = 0.0F;
  ground.intensity = 1.0F;
  cell.push_back(ground);
  pcl::PointXYZI low;  // 同一水平位置、高 6 cm 的矮台阶，水平差 2 cm
  low.x = 2.02F;
  low.y = 0.0F;
  low.z = 0.06F;
  low.intensity = 1.0F;
  cell.push_back(low);

  runStage(stage::Id::UPDATE_TERRAIN_VOXELS);

  EXPECT_EQ(voxelCells()[center_cell]->points.size(), 2U)
      << "地面点与矮物体点必须落在不同的垂直叶里，各自保留";
}

// 叶内只有过期观测（本帧没有新点）→ 整叶按年龄删除
TEST_F(AlgorithmTest, UpdateVoxels_OnlyStaleLeaf_Removed) {
  widenBand(-10.0, 10.0);
  voxelConfig().decay_time = 0.5;
  voxelConfig().no_decay_distance = 0.0;
  voxelConfig().scan_voxel_size = 0.05;
  voxelConfig().scan_voxel_size_z = 0.05;

  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0.0;
  frameTime() = 1.2;
  initTime() = 0.0;

  int center_cell = PersistentVoxelGrid::linearIndex(
      PersistentVoxelGrid::HALF_WIDTH, PersistentVoxelGrid::HALF_WIDTH);
  auto& cell = *voxelCells()[center_cell];
  cell.clear();
  for (double time : {0.10, 0.30}) {
    pcl::PointXYZI point;
    point.x = 1.0F;
    point.y = 0.0F;
    point.z = 0.0F;
    point.intensity = static_cast<float>(time);
    cell.push_back(point);
  }

  runStage(stage::Id::UPDATE_TERRAIN_VOXELS);
  EXPECT_TRUE(voxelCells()[center_cell]->points.empty());
}

// 同一格内、不同高度的两个叶互不影响：地面叶被刷新时，
// 上方那一叶的旧点仍按自己的时刻过期。
TEST_F(AlgorithmTest, UpdateVoxels_RefreshOneLeaf_DoesNotReviveAnother) {
  widenBand(-10.0, 10.0);
  voxelConfig().decay_time = 0.5;
  voxelConfig().no_decay_distance = 0.0;
  voxelConfig().scan_voxel_size = 0.05;
  voxelConfig().scan_voxel_size_z = 0.05;

  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0.0;
  frameTime() = 1.2;
  initTime() = 0.0;

  int center_cell = PersistentVoxelGrid::linearIndex(
      PersistentVoxelGrid::HALF_WIDTH, PersistentVoxelGrid::HALF_WIDTH);
  auto& cell = *voxelCells()[center_cell];
  cell.clear();
  pcl::PointXYZI stale;  // 上方叶：0.30 m 处，早已过期
  stale.x = 1.0F;
  stale.y = 0.0F;
  stale.z = 0.30F;
  stale.intensity = 0.10F;
  cell.push_back(stale);
  pcl::PointXYZI fresh;  // 地面叶：本帧刚观测到
  fresh.x = 1.0F;
  fresh.y = 0.0F;
  fresh.z = 0.0F;
  fresh.intensity = 1.20F;
  cell.push_back(fresh);

  runStage(stage::Id::UPDATE_TERRAIN_VOXELS);

  ASSERT_EQ(voxelCells()[center_cell]->points.size(), 1U);
  EXPECT_FLOAT_EQ(voxelCells()[center_cell]->points[0].intensity, 1.20F);
}

// 近点即使过期也保留（near 优先于 decay）
TEST_F(AlgorithmTest, KeepVoxelPoint_NearPointEvenIfExpired_Kept) {
  widenBand(-10.0, 10.0);
  voxelConfig().decay_time = 1.0;
  voxelConfig().no_decay_distance = 3.0;

  lidar().x = 0;
  lidar().y = 0;
  lidar().z = 0.0;
  frameTime() = 10.0;
  initTime() = 0.0;

  int center_cell = PersistentVoxelGrid::linearIndex(
      PersistentVoxelGrid::HALF_WIDTH, PersistentVoxelGrid::HALF_WIDTH);
  auto& cell = *voxelCells()[center_cell];
  cell.clear();
  pcl::PointXYZI point;
  point.x = 1.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 0.0F;
  cell.push_back(point);

  runStage(stage::Id::UPDATE_TERRAIN_VOXELS);
  EXPECT_EQ(voxelCells()[center_cell]->points.size(), 1U);
}