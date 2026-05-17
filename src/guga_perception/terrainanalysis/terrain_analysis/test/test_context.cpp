#include "terrain_analysis/core/context.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <cmath>

class ContextTest : public testing::Test {
protected:
  ContextTest() {
  }
  TerrainAnalysisContext context_;
};

// ── Construction ──
// 默认构造后 no_data_inited 未初始化，system_inited 为 false
TEST_F(ContextTest, DefaultState_NoDataNotInited) {
  EXPECT_EQ(context_.state.no_data_inited,
            TerrainState::NoDataState::UNINITIALIZED);
  EXPECT_FALSE(context_.state.system_inited);
}

// ── onOdometry ──
// 接收里程计消息后更新车辆位置和朝向三角函数
TEST_F(ContextTest, OnOdometry_StoresVehiclePose) {
  context_.onOdometry(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);

  EXPECT_DOUBLE_EQ(context_.state.vehicle_x, 1.0);
  EXPECT_DOUBLE_EQ(context_.state.vehicle_y, 2.0);
  EXPECT_DOUBLE_EQ(context_.state.vehicle_z, 3.0);
}

// 接收里程计后正确计算 roll/pitch/yaw 的正余弦
TEST_F(ContextTest, OnOdometry_ComputesSinCos) {
  context_.onOdometry(0, 0, 0, 0, 0, M_PI / 4.0);

  EXPECT_NEAR(context_.state.sin_vehicle_yaw, sin(M_PI / 4.0), 1e-5);
  EXPECT_NEAR(context_.state.cos_vehicle_yaw, cos(M_PI / 4.0), 1e-5);
  EXPECT_NEAR(context_.state.sin_vehicle_roll, sin(0), 1e-9);
  EXPECT_NEAR(context_.state.cos_vehicle_pitch, cos(0), 1e-9);
}

// 首次调用 odom 后进入 RECORDING 状态，记录初始位置
TEST_F(ContextTest, OnOdometry_FirstCall_TransitionsToRecording) {
  context_.onOdometry(1.0, 2.0, 0, 0, 0, 0);

  EXPECT_EQ(context_.state.no_data_inited,
            TerrainState::NoDataState::RECORDING);
  EXPECT_DOUBLE_EQ(context_.state.vehicle_x_initial, 1.0);
  EXPECT_DOUBLE_EQ(context_.state.vehicle_y_initial, 2.0);
}

// 移动距离超过 no_decay_distance 后转为 ACTIVE 状态
TEST_F(ContextTest, OnOdometry_MovedFarEnough_TransitionsToActive) {
  context_.state.no_data_inited = TerrainState::NoDataState::RECORDING;
  context_.state.vehicle_x_initial = 0;
  context_.state.vehicle_y_initial = 0;
  context_.cfg.no_decay_distance = 4.0;

  context_.onOdometry(3.0, 4.0, 0, 0, 0, 0);  // distance = 5.0 > 4.0

  EXPECT_EQ(context_.state.no_data_inited, TerrainState::NoDataState::ACTIVE);
}

// 移动距离不足时保持 RECORDING 状态不变
TEST_F(ContextTest, OnOdometry_NotFarEnough_StaysRecording) {
  context_.state.no_data_inited = TerrainState::NoDataState::RECORDING;
  context_.state.vehicle_x_initial = 0;
  context_.state.vehicle_y_initial = 0;
  context_.cfg.no_decay_distance = 4.0;

  context_.onOdometry(2.0, 2.0, 0, 0, 0, 0);  // distance ≈ 2.8 < 4.0

  EXPECT_EQ(context_.state.no_data_inited,
            TerrainState::NoDataState::RECORDING);
}

// ── onLaserCloud ──
// 首次接收点云时记录 system_init_time
TEST_F(ContextTest, OnLaserCloud_FirstCall_SetsInitTime) {
  context_.onLaserCloud(MakeCloud(0, 0, 0), 100.0);

  EXPECT_TRUE(context_.state.system_inited);
  EXPECT_DOUBLE_EQ(context_.state.system_init_time, 100.0);
}

// 超出体素网格范围的点被裁剪掉
TEST_F(ContextTest, OnLaserCloud_FiltersPointsBeyondVoxelRange) {
  context_.state.vehicle_x = 0;
  context_.state.vehicle_y = 0;
  context_.state.vehicle_z = 0;
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->push_back({0, 0, 0, 0});
  cloud->push_back({50, 50, 0, 0});  // far outside voxel range

  context_.onLaserCloud(cloud, 100.0);

  EXPECT_EQ(context_.state.laser_cloud_crop->points.size(), 1U);
}

// ── onJoystick ──
// joystick 按钮按下时触发清除模式，重置 no_data 状态
TEST_F(ContextTest, OnJoystick_ButtonPressed_TriggersClearing) {
  context_.onJoystick(true);

  EXPECT_EQ(context_.state.no_data_inited,
            TerrainState::NoDataState::UNINITIALIZED);
  EXPECT_TRUE(context_.state.clearing_cloud);
}

// 按钮松开时 clearing_cloud 状态不变
TEST_F(ContextTest, OnJoystick_ButtonReleased_NoChange) {
  context_.state.clearing_cloud = false;
  context_.onJoystick(false);

  EXPECT_FALSE(context_.state.clearing_cloud);
}

// ── onClearing ──
// 接收清除距离后更新 clearing_distance 并触发清除标志
TEST_F(ContextTest, OnClearing_SetsDistanceAndTriggersClearing) {
  context_.onClearing(10.5);

  EXPECT_DOUBLE_EQ(context_.state.clearing_distance, 10.5);
  EXPECT_TRUE(context_.state.clearing_cloud);
  EXPECT_EQ(context_.state.no_data_inited,
            TerrainState::NoDataState::UNINITIALIZED);
}
