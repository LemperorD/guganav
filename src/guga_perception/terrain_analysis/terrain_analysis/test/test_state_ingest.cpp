#include "terrain_analysis/core/terrain_processor.hpp"
#include "terrain_analysis/core/config.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <cmath>

namespace terrain_analysis {
  class StateIngestTest : public testing::Test {
  protected:
    TerrainProcessor processor_;
    TerrainConfig& config_ = processor_.config_;
    TerrainState& state_ = processor_.state_;
  };  // namespace testing::Test

  // ── Construction ──
  // 默认构造后 system_inited 为 false
  TEST_F(StateIngestTest, DefaultState_SystemNotInited) {
    EXPECT_FALSE(state_.system_inited);
  }

  // ── ingestOdometry ──
  // 接收里程计消息后更新车辆位置和朝向三角函数
  TEST_F(StateIngestTest, IngestOdometry_StoresVehiclePose) {
    processor_.ingestOdometry(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);

    EXPECT_DOUBLE_EQ(state_.vehicle_x, 1.0);
    EXPECT_DOUBLE_EQ(state_.vehicle_y, 2.0);
    EXPECT_DOUBLE_EQ(state_.vehicle_z, 3.0);
  }

  // 接收里程计后正确计算 roll/pitch/yaw 的正余弦
  TEST_F(StateIngestTest, IngestOdometry_ComputesSinCos) {
    processor_.ingestOdometry(0, 0, 0, 0, 0, M_PI / 4.0);

    EXPECT_NEAR(state_.sin_vehicle_yaw, sin(M_PI / 4.0), 1e-5);
    EXPECT_NEAR(state_.cos_vehicle_yaw, cos(M_PI / 4.0), 1e-5);
    EXPECT_NEAR(state_.sin_vehicle_roll, sin(0), 1e-9);
    EXPECT_NEAR(state_.cos_vehicle_pitch, cos(0), 1e-9);
  }

  // ── ingestLaserCloud ──
  // 首次接收点云时记录 system_init_time
  TEST_F(StateIngestTest, IngestLaserCloud_FirstCall_SetsInitTime) {
    processor_.ingestLaserCloud(MakeCloud(0, 0, 0), 100.0);

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

    processor_.ingestLaserCloud(cloud, 100.0);

    EXPECT_EQ(state_.laser_cloud_crop->points.size(), 1U);
  }
}  // namespace terrain_analysis
