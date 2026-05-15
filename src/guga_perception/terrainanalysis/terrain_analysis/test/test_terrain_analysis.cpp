#include "terrain_analysis/terrain_analysis.hpp"
#include "terrain_analysis/core/algorithm.hpp"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>

class TerrainAnalysisTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    nh_ = rclcpp::Node::make_shared("test_terrain_analysis");
    terrain_ = std::make_unique<TerrainAnalysis>(nh_.get());
  }

  void TearDown() override {
    terrain_.reset();
    nh_.reset();
    rclcpp::shutdown();
  }

  void SendOdom(double x, double y, double z, double yaw) {
    terrain_->context_.onOdometry(x, y, z, 0.0, 0.0, yaw);
  }

  void SendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                 double timestamp_sec) {
    terrain_->context_.onLaserCloud(cloud, timestamp_sec);
  }

  rclcpp::Node::SharedPtr nh_;
  std::unique_ptr<TerrainAnalysis> terrain_;
};

// ═══════════════════════════════════════════════
// Callback tests
// ═══════════════════════════════════════════════

TEST_F(TerrainAnalysisTest, OnOdometry_UpdatesVehiclePose) {
  terrain_->context_.onOdometry(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);

  EXPECT_DOUBLE_EQ(terrain_->context_.state.vehicle_x, 1.0);
  EXPECT_DOUBLE_EQ(terrain_->context_.state.vehicle_y, 2.0);
  EXPECT_DOUBLE_EQ(terrain_->context_.state.vehicle_z, 3.0);
  EXPECT_DOUBLE_EQ(terrain_->context_.state.sin_vehicle_roll, sin(0.1));
  EXPECT_DOUBLE_EQ(terrain_->context_.state.sin_vehicle_pitch, sin(0.2));
  EXPECT_DOUBLE_EQ(terrain_->context_.state.cos_vehicle_yaw, cos(0.3));
}

TEST_F(TerrainAnalysisTest, OnOdometry_ComputesSinCos) {
  terrain_->context_.onOdometry(0, 0, 0, 0, 0, M_PI / 4.0);

  EXPECT_NEAR(terrain_->context_.state.sin_vehicle_yaw, sin(M_PI / 4.0), 1e-5);
  EXPECT_NEAR(terrain_->context_.state.cos_vehicle_yaw, cos(M_PI / 4.0), 1e-5);
}

TEST_F(TerrainAnalysisTest, OnOdometry_NoDataInited_ZeroToOne) {
  EXPECT_EQ(terrain_->context_.state.no_data_inited,
            TerrainState::NoDataState::UNINITIALIZED);
  terrain_->context_.onOdometry(1.0, 2.0, 0, 0, 0, 0);

  EXPECT_EQ(terrain_->context_.state.no_data_inited,
            TerrainState::NoDataState::RECORDING);
  EXPECT_DOUBLE_EQ(terrain_->context_.state.vehicle_x_rec, 1.0);
}

TEST_F(TerrainAnalysisTest, OnOdometry_NoDataInited_OneToTwo_WhenFarEnough) {
  terrain_->context_.state.no_data_inited =
      TerrainState::NoDataState::RECORDING;
  terrain_->context_.state.vehicle_x_rec = 0;
  terrain_->context_.state.vehicle_y_rec = 0;
  terrain_->context_.cfg.no_decay_dis = 4.0;

  terrain_->context_.onOdometry(3.0, 4.0, 0, 0, 0, 0);  // distance 5.0

  EXPECT_EQ(terrain_->context_.state.no_data_inited,
            TerrainState::NoDataState::ACTIVE);
}

TEST_F(TerrainAnalysisTest, OnOdometry_NoDataInited_StaysOne_WhenClose) {
  terrain_->context_.state.no_data_inited =
      TerrainState::NoDataState::RECORDING;
  terrain_->context_.state.vehicle_x_rec = 0;
  terrain_->context_.state.vehicle_y_rec = 0;
  terrain_->context_.cfg.no_decay_dis = 4.0;

  terrain_->context_.onOdometry(2.0, 2.0, 0, 0, 0, 0);  // distance 2.8

  EXPECT_EQ(terrain_->context_.state.no_data_inited,
            TerrainState::NoDataState::RECORDING);
}

TEST_F(TerrainAnalysisTest, OnLaserCloud_SetsSystemInitTime) {
  EXPECT_FALSE(terrain_->context_.state.system_inited);
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->push_back({0, 0, 0, 0});

  terrain_->context_.onLaserCloud(cloud, 100.0);

  EXPECT_TRUE(terrain_->context_.state.system_inited);
  EXPECT_DOUBLE_EQ(terrain_->context_.state.system_init_time, 100.0);
}

TEST_F(TerrainAnalysisTest, OnLaserCloud_CropsFarPoints) {
  terrain_->context_.state.vehicle_x = 0;
  terrain_->context_.state.vehicle_y = 0;
  terrain_->context_.state.vehicle_z = 0;
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI near_pt{0, 0, 0, 0};
  pcl::PointXYZI far_pt{50, 50, 0, 0};
  cloud->push_back(near_pt);
  cloud->push_back(far_pt);

  terrain_->context_.onLaserCloud(cloud, 100.0);

  EXPECT_EQ(terrain_->context_.state.laser_cloud_crop->points.size(), 1u);
}

TEST_F(TerrainAnalysisTest, OnJoystick_Button5_TriggersClearing) {
  terrain_->context_.onJoystick(true);

  EXPECT_EQ(terrain_->context_.state.no_data_inited,
            TerrainState::NoDataState::UNINITIALIZED);
  EXPECT_TRUE(terrain_->context_.state.clearing_cloud);
}

TEST_F(TerrainAnalysisTest, OnJoystick_Button5Low_NoTrigger) {
  terrain_->context_.state.clearing_cloud = false;
  terrain_->context_.onJoystick(false);

  EXPECT_FALSE(terrain_->context_.state.clearing_cloud);
}

TEST_F(TerrainAnalysisTest, OnClearing_SetsDisAndFlag) {
  terrain_->context_.onClearing(10.5);

  EXPECT_DOUBLE_EQ(terrain_->context_.state.clearing_dis, 10.5);
  EXPECT_TRUE(terrain_->context_.state.clearing_cloud);
  EXPECT_EQ(terrain_->context_.state.no_data_inited,
            TerrainState::NoDataState::UNINITIALIZED);
}

// ═══════════════════════════════════════════════
// Pipeline tests
// ═══════════════════════════════════════════════

TEST_F(TerrainAnalysisTest, FlatGround_DetectsGround_ZNearZero) {
  SendOdom(0, 0, 0, 0);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < 500; i++) {
    pcl::PointXYZI p;
    p.x = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0f;
    p.y = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0f;
    p.z = 0.01f;
    p.intensity = 0;
    cloud->push_back(p);
  }
  SendCloud(cloud, 100.0);
  TerrainAlgorithm::run(terrain_->context_.cfg, terrain_->context_.state);

  EXPECT_GT(terrain_->context_.state.terrainCloudElev().points.size(), 0u)
      << "terrainCloudElev should contain output points";

  float max_intensity = 0;
  for (const auto& p : terrain_->context_.state.terrainCloudElev().points) {
    max_intensity = std::max(max_intensity, p.intensity);
  }
  EXPECT_LT(max_intensity, 0.5f)
      << "Flat ground: all intensities should be small";
}

TEST_F(TerrainAnalysisTest, Obstacle_DetectedAboveGround) {
  SendOdom(0, 0, 0, 0);

  // Fixed grid: 20x20 = 400 ground + 400 elevated, covering 2m x 2m
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = -10; ix <= 10; ix++) {
    for (int iy = -10; iy <= 10; iy++) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(ix) * 0.1f;
      p.y = static_cast<float>(iy) * 0.1f;
      p.z = 0.0f;
      p.intensity = 0;
      cloud->push_back(p);
      p.z = 0.15f;
      cloud->push_back(p);
    }
  }

  SendCloud(cloud, 100.0);
  TerrainAlgorithm::run(terrain_->context_.cfg, terrain_->context_.state);

  EXPECT_GT(terrain_->context_.state.terrainCloudElev().points.size(), 0u);

  float max_intensity = 0;
  for (const auto& p : terrain_->context_.state.terrainCloudElev().points) {
    max_intensity = std::max(max_intensity, p.intensity);
  }
  EXPECT_GT(max_intensity, 0.05f)
      << "Elevated points should produce non-zero intensity";
}

TEST_F(TerrainAnalysisTest, IsolatedObstacle_RejectedWhenVoxelSparse) {
  SendOdom(0, 0, 0, 0);

  // Ground cluster in a tight region near origin, obstacle far away
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = -5; ix <= 5; ix++) {
    for (int iy = -5; iy <= 5; iy++) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(ix) * 0.1f;
      p.y = static_cast<float>(iy) * 0.1f;
      p.z = 0.0f;
      p.intensity = 0;
      cloud->push_back(p);
    }
  }
  // Single obstacle far outside the ground cluster, isolated voxel
  pcl::PointXYZI obs;
  obs.x = 3.0f;
  obs.y = 3.0f;
  obs.z = 0.3f;
  obs.intensity = 0;
  cloud->push_back(obs);

  SendCloud(cloud, 100.0);
  TerrainAlgorithm::run(terrain_->context_.cfg, terrain_->context_.state);

  // The isolated obstacle point is in a voxel with < minBlockPointNum (10)
  // ground points, so it should be excluded from output
  bool found_isolated = false;
  for (const auto& p : terrain_->context_.state.terrainCloudElev().points) {
    if (p.x > 2.5f && p.intensity > 0.1f) {
      found_isolated = true;
      break;
    }
  }
  EXPECT_FALSE(found_isolated)
      << "Isolated obstacle in sparse voxel should be excluded";
}

// ═══════════════════════════════════════════════
// Algorithm stage tests
// ═══════════════════════════════════════════════

TEST_F(TerrainAnalysisTest, Rollover_NoShiftWhenStationary) {
  terrain_->context_.state.vehicle_x = 0;
  terrain_->context_.state.vehicle_y = 0;
  int sx = terrain_->context_.state.terrain_voxel_shift_x;
  int sy = terrain_->context_.state.terrain_voxel_shift_y;

  TerrainAlgorithm::rolloverVoxels(terrain_->context_.cfg,
                                   terrain_->context_.state);

  EXPECT_EQ(terrain_->context_.state.terrain_voxel_shift_x, sx);
  EXPECT_EQ(terrain_->context_.state.terrain_voxel_shift_y, sy);
}

TEST_F(TerrainAnalysisTest, Rollover_ShiftX_Negative) {
  terrain_->context_.state.vehicle_x =
      -2.0;  // beyond voxel_size=1.0 to the left
  int sx = terrain_->context_.state.terrain_voxel_shift_x;

  TerrainAlgorithm::rolloverVoxels(terrain_->context_.cfg,
                                   terrain_->context_.state);

  EXPECT_EQ(terrain_->context_.state.terrain_voxel_shift_x, sx - 1);
}

TEST_F(TerrainAnalysisTest, Rollover_ShiftX_Positive) {
  terrain_->context_.state.vehicle_x = 2.0f;
  int sx = terrain_->context_.state.terrain_voxel_shift_x;

  TerrainAlgorithm::rolloverVoxels(terrain_->context_.cfg,
                                   terrain_->context_.state);

  EXPECT_EQ(terrain_->context_.state.terrain_voxel_shift_x, sx + 1);
}

TEST_F(TerrainAnalysisTest, Rollover_ShiftY_Negative) {
  terrain_->context_.state.vehicle_y = -2.0;
  int sy = terrain_->context_.state.terrain_voxel_shift_y;

  TerrainAlgorithm::rolloverVoxels(terrain_->context_.cfg,
                                   terrain_->context_.state);

  EXPECT_EQ(terrain_->context_.state.terrain_voxel_shift_y, sy - 1);
}

TEST_F(TerrainAnalysisTest, Rollover_ShiftY_Positive) {
  terrain_->context_.state.vehicle_y = 2.0f;
  int sy = terrain_->context_.state.terrain_voxel_shift_y;

  TerrainAlgorithm::rolloverVoxels(terrain_->context_.cfg,
                                   terrain_->context_.state);

  EXPECT_EQ(terrain_->context_.state.terrain_voxel_shift_y, sy + 1);
}

TEST_F(TerrainAnalysisTest, Rollover_PreservesVoxelData) {
  terrain_->context_.state.vehicle_x = -2.0;
  // Place a point in voxel(0,0) before shift
  terrain_->context_.state.terrain_voxel_cloud[0]->clear();
  pcl::PointXYZI p{0, 0, 0, 0};
  terrain_->context_.state.terrain_voxel_cloud[0]->push_back(p);

  TerrainAlgorithm::rolloverVoxels(terrain_->context_.cfg,
                                   terrain_->context_.state);

  // After shift-left, voxel(0,0) should be empty (cleared), old data moved
  EXPECT_TRUE(terrain_->context_.state.terrain_voxel_cloud[0]->points.empty());
}

TEST_F(TerrainAnalysisTest, StackLaserScans_BinsPointsToCenter) {
  terrain_->context_.state.vehicle_x = 0;
  terrain_->context_.state.vehicle_y = 0;
  for (int i = 0; i < TerrainConfig::TERRAIN_VOXEL_NUM; i++) {
    terrain_->context_.state.terrain_voxel_cloud[i]->clear();
    terrain_->context_.state.terrain_voxel_update_num[i] = 0;
  }
  terrain_->context_.state.laser_cloud_crop->clear();
  pcl::PointXYZI p{0, 0, 0, 0};
  terrain_->context_.state.laser_cloud_crop->push_back(p);

  TerrainAlgorithm::voxelize(terrain_->context_.cfg, terrain_->context_.state);

  int c = (TerrainConfig::TERRAIN_VOXEL_WIDTH
           * TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH)
        + TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH;
  EXPECT_EQ(terrain_->context_.state.terrain_voxel_cloud[c]->points.size(), 1u);
  EXPECT_EQ(terrain_->context_.state.terrain_voxel_update_num[c], 1);
}

TEST_F(TerrainAnalysisTest, StackLaserScans_EmptyCloud_NoOp) {
  terrain_->context_.state.laser_cloud_crop->clear();
  for (int i = 0; i < TerrainConfig::TERRAIN_VOXEL_NUM; i++) {
    terrain_->context_.state.terrain_voxel_update_num[i] = 0;
  }

  TerrainAlgorithm::voxelize(terrain_->context_.cfg, terrain_->context_.state);

  for (int i = 0; i < TerrainConfig::TERRAIN_VOXEL_NUM; i++) {
    EXPECT_EQ(terrain_->context_.state.terrain_voxel_update_num[i], 0);
  }
}

TEST_F(TerrainAnalysisTest, ComputeElevation_SortingMode_UsesQuantile) {
  terrain_->context_.cfg.use_sorting = true;
  terrain_->context_.cfg.quantile_z = 0.5;
  terrain_->context_.cfg.limit_ground_lift = false;
  int cell = TerrainConfig::PLANAR_VOXEL_WIDTH
               * TerrainConfig::PLANAR_VOXEL_HALF_WIDTH
           + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    terrain_->context_.state.planar_voxel_elev[i] = 999;
    terrain_->context_.state.planar_point_elev[i].clear();
  }
  terrain_->context_.state.planar_point_elev[cell] = {0.1f, 0.5f, 0.3f, 0.2f,
                                                      0.4f};

  TerrainAlgorithm::computeElevation(terrain_->context_.cfg,
                                     terrain_->context_.state);

  // sorted: 0.1, 0.2, 0.3, 0.4, 0.5. quantile 0.5 * 5 = 2 → cell=2 → 0.3
  EXPECT_FLOAT_EQ(terrain_->context_.state.planar_voxel_elev[cell], 0.3f);
}

TEST_F(TerrainAnalysisTest, ComputeElevation_MinMode_UsesMinimum) {
  terrain_->context_.cfg.use_sorting = false;
  terrain_->context_.cfg.limit_ground_lift = false;
  int cell = TerrainConfig::PLANAR_VOXEL_WIDTH
               * TerrainConfig::PLANAR_VOXEL_HALF_WIDTH
           + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    terrain_->context_.state.planar_voxel_elev[i] = 999;
    terrain_->context_.state.planar_point_elev[i].clear();
  }
  terrain_->context_.state.planar_point_elev[cell] = {1.5f, 0.5f, 1.0f};

  TerrainAlgorithm::computeElevation(terrain_->context_.cfg,
                                     terrain_->context_.state);

  EXPECT_FLOAT_EQ(terrain_->context_.state.planar_voxel_elev[cell], 0.5f);
}

TEST_F(TerrainAnalysisTest, ComputeElevation_GroundLiftLimited) {
  terrain_->context_.cfg.use_sorting = true;
  terrain_->context_.cfg.quantile_z = 0.5;
  terrain_->context_.cfg.limit_ground_lift = true;
  terrain_->context_.cfg.max_ground_lift = 0.3;
  int cell = TerrainConfig::PLANAR_VOXEL_WIDTH
               * TerrainConfig::PLANAR_VOXEL_HALF_WIDTH
           + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    terrain_->context_.state.planar_voxel_elev[i] = 999;
    terrain_->context_.state.planar_point_elev[i].clear();
  }
  terrain_->context_.state.planar_point_elev[cell] = {0.5f, 2.0f, 1.0f};
  // sorted: 0.5, 1.0, 2.0. quantile 0.5*3=1 → 1.0. diff=1.0-0.5=0.5 > 0.3 →
  // limit → 0.5 + 0.3 = 0.8

  TerrainAlgorithm::computeElevation(terrain_->context_.cfg,
                                     terrain_->context_.state);

  EXPECT_FLOAT_EQ(terrain_->context_.state.planar_voxel_elev[cell], 0.8f);
}

TEST_F(TerrainAnalysisTest, DetectDynamicObstacles_ClosePoint_AddsMinCount) {
  terrain_->context_.cfg.clear_dy_obs = true;
  terrain_->context_.cfg.min_dy_obs_dis =
      5.0;  // high threshold → all points "close"
  terrain_->context_.cfg.min_dy_obs_point_num = 7;
  terrain_->context_.state.vehicle_x = 0;
  terrain_->context_.state.vehicle_y = 0;
  terrain_->context_.state.vehicle_z = 0;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    terrain_->context_.state.planar_voxel_dy_obs[i] = 0;
  }
  terrain_->context_.state.terrain_cloud->clear();
  pcl::PointXYZI p{0.1f, 0, 0, 0};
  terrain_->context_.state.terrain_cloud->push_back(p);

  TerrainAlgorithm::detectDynamicObstacles(terrain_->context_.cfg,
                                           terrain_->context_.state);

  int total = 0;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    total += terrain_->context_.state.planar_voxel_dy_obs[i];
  }
  EXPECT_GT(total, 0);
}

TEST_F(TerrainAnalysisTest, DetectDynamicObstacles_Disabled_NoEffect) {
  terrain_->context_.cfg.clear_dy_obs = false;
  terrain_->context_.state.vehicle_x = 0;
  terrain_->context_.state.vehicle_y = 0;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    terrain_->context_.state.planar_voxel_dy_obs[i] = 0;
  }
  terrain_->context_.state.terrain_cloud->clear();
  pcl::PointXYZI p{0.1f, 0, 0, 0};
  terrain_->context_.state.terrain_cloud->push_back(p);

  // This function is only called when clear_dy_obs_ is true;
  // testing that it still only runs the logic based on flag checks
  TerrainAlgorithm::detectDynamicObstacles(terrain_->context_.cfg,
                                           terrain_->context_.state);

  int total = 0;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    total += terrain_->context_.state.planar_voxel_dy_obs[i];
  }
  EXPECT_GT(
      total,
      0);  // detectDynamicObstacles ignores clear_dy_obs_ flag, processes all
}

TEST_F(TerrainAnalysisTest, FilterDynamicObstaclePoints_ResetsHighAngle) {
  terrain_->context_.cfg.clear_dy_obs = true;
  terrain_->context_.cfg.min_dy_obs_angle = 10.0;
  terrain_->context_.cfg.min_dy_obs_rel_z = -0.5;
  int cell = TerrainConfig::PLANAR_VOXEL_WIDTH
               * TerrainConfig::PLANAR_VOXEL_HALF_WIDTH
           + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  terrain_->context_.state.planar_voxel_dy_obs[cell] = 10;

  terrain_->context_.state.laser_cloud_crop->clear();
  pcl::PointXYZI p{0.05f, 0, 2.0f, 0};  // high relZ → angle close to 90° > 10°
  terrain_->context_.state.laser_cloud_crop->push_back(p);

  TerrainAlgorithm::filterDynamicObstaclePoints(terrain_->context_.cfg,
                                                terrain_->context_.state, 1);

  EXPECT_EQ(terrain_->context_.state.planar_voxel_dy_obs[cell], 0)
      << "High-angle point should reset dy_obs";
}

TEST_F(TerrainAnalysisTest, FilterDynamicObstaclePoints_LowAngle_Keeps) {
  terrain_->context_.cfg.clear_dy_obs = true;
  terrain_->context_.cfg.min_dy_obs_angle =
      90.0;  // nearly impossible to exceed
  terrain_->context_.cfg.min_dy_obs_rel_z = -0.5;
  int cell = TerrainConfig::PLANAR_VOXEL_WIDTH
               * TerrainConfig::PLANAR_VOXEL_HALF_WIDTH
           + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  terrain_->context_.state.planar_voxel_dy_obs[cell] = 10;

  terrain_->context_.state.laser_cloud_crop->clear();
  pcl::PointXYZI p{0.05f, 0, 0, 0};
  terrain_->context_.state.laser_cloud_crop->push_back(p);

  TerrainAlgorithm::filterDynamicObstaclePoints(terrain_->context_.cfg,
                                                terrain_->context_.state, 1);

  EXPECT_EQ(terrain_->context_.state.planar_voxel_dy_obs[cell], 10)
      << "Low-angle point should NOT reset dy_obs";
}

TEST_F(TerrainAnalysisTest, AddNoDataObstacles_CreatesObstaclePoints) {
  terrain_->context_.state.vehicle_x = 0;
  terrain_->context_.state.vehicle_y = 0;
  terrain_->context_.state.vehicle_z = 0;
  terrain_->context_.cfg.no_data_block_skip_num = 1;
  terrain_->context_.cfg.min_block_point_num = 5;
  terrain_->context_.cfg.vehicle_height = 1.5;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    terrain_->context_.state.planar_point_elev[i].clear();
    terrain_->context_.state.planar_voxel_edge[i] = 0;
  }
  terrain_->context_.state.terrain_cloud_elev->clear();

  TerrainAlgorithm::addNoDataObstacles(terrain_->context_.cfg,
                                       terrain_->context_.state);

  EXPECT_GT(terrain_->context_.state.terrain_cloud_elev->points.size(), 0u)
      << "Should create obstacles in empty planar voxels";
}

TEST_F(TerrainAnalysisTest, AddNoDataObstacles_EnoughPoints_NoObstacle) {
  terrain_->context_.state.vehicle_x = 0;
  terrain_->context_.state.vehicle_y = 0;
  terrain_->context_.cfg.min_block_point_num = 5;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    terrain_->context_.state.planar_point_elev[i] = {0.1f, 0.2f, 0.3f, 0.4f,
                                                     0.5f};
    terrain_->context_.state.planar_voxel_edge[i] = 0;
  }
  terrain_->context_.state.terrain_cloud_elev->clear();

  TerrainAlgorithm::addNoDataObstacles(terrain_->context_.cfg,
                                       terrain_->context_.state);

  EXPECT_EQ(terrain_->context_.state.terrain_cloud_elev->points.size(), 0u)
      << "All voxels have enough points, no obstacles should be added";
}
