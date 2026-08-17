#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/config.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <cmath>

namespace terrain_analysis {
  class StateIngestTest : public testing::Test {
  protected:
    TerrainConfig config_;
    TerrainState state_;
  };  // namespace testing::Test

  // ── Construction ──
  // 默认构造后 no_data_inited 未初始化，system_inited 为 false
  TEST_F(StateIngestTest, DefaultState_NoDataNotInited) {
    EXPECT_EQ(state_.no_data_inited, TerrainState::NoDataState::UNINITIALIZED);
    EXPECT_FALSE(state_.system_inited);
  }

  // ── ingestOdometry ──
  // 接收里程计消息后更新车辆位置和朝向三角函数
  TEST_F(StateIngestTest, IngestOdometry_StoresVehiclePose) {
    terrain_analysis::algorithm::ingestOdometry(config_, state_, 1.0, 2.0, 3.0,
                                                0.1, 0.2, 0.3);

    EXPECT_DOUBLE_EQ(state_.vehicle_x, 1.0);
    EXPECT_DOUBLE_EQ(state_.vehicle_y, 2.0);
    EXPECT_DOUBLE_EQ(state_.vehicle_z, 3.0);
  }

  // 接收里程计后正确计算 roll/pitch/yaw 的正余弦
  TEST_F(StateIngestTest, IngestOdometry_ComputesSinCos) {
    terrain_analysis::algorithm::ingestOdometry(config_, state_, 0, 0, 0, 0, 0,
                                                M_PI / 4.0);

    EXPECT_NEAR(state_.sin_vehicle_yaw, sin(M_PI / 4.0), 1e-5);
    EXPECT_NEAR(state_.cos_vehicle_yaw, cos(M_PI / 4.0), 1e-5);
    EXPECT_NEAR(state_.sin_vehicle_roll, sin(0), 1e-9);
    EXPECT_NEAR(state_.cos_vehicle_pitch, cos(0), 1e-9);
  }

  // 首次调用 odom 后进入 RECORDING 状态，记录初始位置
  TEST_F(StateIngestTest, IngestOdometry_FirstCall_TransitionsToRecording) {
    terrain_analysis::algorithm::ingestOdometry(config_, state_, 1.0, 2.0, 0, 0,
                                                0, 0);

    EXPECT_EQ(state_.no_data_inited, TerrainState::NoDataState::RECORDING);
    EXPECT_DOUBLE_EQ(state_.vehicle_x_initial, 1.0);
    EXPECT_DOUBLE_EQ(state_.vehicle_y_initial, 2.0);
  }

  // 移动距离超过 no_decay_distance 后转为 ACTIVE 状态
  TEST_F(StateIngestTest, IngestOdometry_MovedFarEnough_TransitionsToActive) {
    state_.no_data_inited = TerrainState::NoDataState::RECORDING;
    state_.vehicle_x_initial = 0;
    state_.vehicle_y_initial = 0;
    config_.no_decay_distance = 4.0;

    terrain_analysis::algorithm::ingestOdometry(config_, state_, 3.0, 4.0, 0, 0,
                                                0,
                                                0);  // distance = 5.0 > 4.0

    EXPECT_EQ(state_.no_data_inited, TerrainState::NoDataState::ACTIVE);
  }

  // 移动距离不足时保持 RECORDING 状态不变
  TEST_F(StateIngestTest, IngestOdometry_NotFarEnough_StaysRecording) {
    state_.no_data_inited = TerrainState::NoDataState::RECORDING;
    state_.vehicle_x_initial = 0;
    state_.vehicle_y_initial = 0;
    config_.no_decay_distance = 4.0;

    terrain_analysis::algorithm::ingestOdometry(config_, state_, 2.0, 2.0, 0, 0,
                                                0,
                                                0);  // distance ≈ 2.8 < 4.0

    EXPECT_EQ(state_.no_data_inited, TerrainState::NoDataState::RECORDING);
  }

  // ── ingestLaserCloud ──
  // 首次接收点云时记录 system_init_time
  TEST_F(StateIngestTest, IngestLaserCloud_FirstCall_SetsInitTime) {
    terrain_analysis::algorithm::ingestLaserCloud(config_, state_,
                                                  MakeCloud(0, 0, 0), 100.0);

    EXPECT_TRUE(state_.system_inited);
    EXPECT_DOUBLE_EQ(state_.system_init_time, 100.0);
  }

  // 超出体素网格范围的点被裁剪掉
  TEST_F(StateIngestTest, IngestLaserCloud_FiltersPointsBeyondVoxelRange) {
    state_.vehicle_x = 0;
    state_.vehicle_y = 0;
    state_.vehicle_z = 0;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->push_back({0, 0, 0, 0});
    cloud->push_back({50, 50, 0, 0});  // far outside voxel range

    terrain_analysis::algorithm::ingestLaserCloud(config_, state_, cloud,
                                                  100.0);

    EXPECT_EQ(state_.laser_cloud_crop->points.size(), 1U);
  }

  // ── ingestClearing ──
  // 接收清除距离后更新 clearing_distance 并触发清除标志
  TEST_F(StateIngestTest, IngestClearing_SetsDistanceAndTriggersClearing) {
    terrain_analysis::algorithm::ingestClearing(state_, 10.5);

    EXPECT_DOUBLE_EQ(state_.clearing_distance, 10.5);
    EXPECT_TRUE(state_.clearing_cloud);
    EXPECT_EQ(state_.no_data_inited, TerrainState::NoDataState::UNINITIALIZED);
  }
}  // namespace terrain_analysis