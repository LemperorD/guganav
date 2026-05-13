#include "terrain_analysis/terrain_analysis.hpp"

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
    terrain_->ctx_.onOdometry(x, y, z, 0.0, 0.0, yaw);
  }

  void SendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                 double timestamp_sec) {
    terrain_->ctx_.onLaserCloud(cloud, timestamp_sec);
  }

  rclcpp::Node::SharedPtr nh_;
  std::unique_ptr<TerrainAnalysis> terrain_;
};

// ═══════════════════════════════════════════════
// Callback tests
// ═══════════════════════════════════════════════

TEST_F(TerrainAnalysisTest, OnOdometry_UpdatesVehiclePose) {
  terrain_->ctx_.onOdometry(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);

  EXPECT_FLOAT_EQ(terrain_->ctx_.vehicle_x_, 1.0f);
  EXPECT_FLOAT_EQ(terrain_->ctx_.vehicle_y_, 2.0f);
  EXPECT_FLOAT_EQ(terrain_->ctx_.vehicle_z_, 3.0f);
  EXPECT_FLOAT_EQ(terrain_->ctx_.vehicle_roll_, 0.1f);
  EXPECT_FLOAT_EQ(terrain_->ctx_.vehicle_pitch_, 0.2f);
  EXPECT_FLOAT_EQ(terrain_->ctx_.vehicle_yaw_, 0.3f);
}

TEST_F(TerrainAnalysisTest, OnOdometry_ComputesSinCos) {
  terrain_->ctx_.onOdometry(0, 0, 0, 0, 0, M_PI / 4.0);

  EXPECT_NEAR(terrain_->ctx_.sin_vehicle_yaw_, sin(M_PI / 4.0), 1e-5);
  EXPECT_NEAR(terrain_->ctx_.cos_vehicle_yaw_, cos(M_PI / 4.0), 1e-5);
}

TEST_F(TerrainAnalysisTest, OnOdometry_NoDataInited_ZeroToOne) {
  EXPECT_EQ(terrain_->ctx_.no_data_inited_, 0);
  terrain_->ctx_.onOdometry(1.0, 2.0, 0, 0, 0, 0);

  EXPECT_EQ(terrain_->ctx_.no_data_inited_, 1);
  EXPECT_FLOAT_EQ(terrain_->ctx_.vehicle_x_rec_, 1.0f);
}

TEST_F(TerrainAnalysisTest, OnOdometry_NoDataInited_OneToTwo_WhenFarEnough) {
  terrain_->ctx_.no_data_inited_ = 1;
  terrain_->ctx_.vehicle_x_rec_ = 0;
  terrain_->ctx_.vehicle_y_rec_ = 0;
  terrain_->ctx_.no_decay_dis_ = 4.0;

  terrain_->ctx_.onOdometry(3.0, 4.0, 0, 0, 0, 0);  // distance 5.0

  EXPECT_EQ(terrain_->ctx_.no_data_inited_, 2);
}

TEST_F(TerrainAnalysisTest, OnOdometry_NoDataInited_StaysOne_WhenClose) {
  terrain_->ctx_.no_data_inited_ = 1;
  terrain_->ctx_.vehicle_x_rec_ = 0;
  terrain_->ctx_.vehicle_y_rec_ = 0;
  terrain_->ctx_.no_decay_dis_ = 4.0;

  terrain_->ctx_.onOdometry(2.0, 2.0, 0, 0, 0, 0);  // distance 2.8

  EXPECT_EQ(terrain_->ctx_.no_data_inited_, 1);
}

TEST_F(TerrainAnalysisTest, OnLaserCloud_SetsSystemInitTime) {
  EXPECT_FALSE(terrain_->ctx_.system_inited_);
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->push_back({0, 0, 0, 0});

  terrain_->ctx_.onLaserCloud(cloud, 100.0);

  EXPECT_TRUE(terrain_->ctx_.system_inited_);
  EXPECT_DOUBLE_EQ(terrain_->ctx_.system_init_time_, 100.0);
}

TEST_F(TerrainAnalysisTest, OnLaserCloud_CropsFarPoints) {
  terrain_->ctx_.vehicle_x_ = 0;
  terrain_->ctx_.vehicle_y_ = 0;
  terrain_->ctx_.vehicle_z_ = 0;
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI near_pt{0, 0, 0, 0};
  pcl::PointXYZI far_pt{50, 50, 0, 0};
  cloud->push_back(near_pt);
  cloud->push_back(far_pt);

  terrain_->ctx_.onLaserCloud(cloud, 100.0);

  EXPECT_EQ(terrain_->ctx_.laser_cloud_crop_->points.size(), 1u);
}

TEST_F(TerrainAnalysisTest, OnJoystick_Button5_TriggersClearing) {
  terrain_->ctx_.onJoystick(true);

  EXPECT_EQ(terrain_->ctx_.no_data_inited_, 0);
  EXPECT_TRUE(terrain_->ctx_.clearing_cloud_);
}

TEST_F(TerrainAnalysisTest, OnJoystick_Button5Low_NoTrigger) {
  terrain_->ctx_.clearing_cloud_ = false;
  terrain_->ctx_.onJoystick(false);

  EXPECT_FALSE(terrain_->ctx_.clearing_cloud_);
}

TEST_F(TerrainAnalysisTest, OnClearing_SetsDisAndFlag) {
  terrain_->ctx_.onClearing(10.5);

  EXPECT_DOUBLE_EQ(terrain_->ctx_.clearing_dis_, 10.5);
  EXPECT_TRUE(terrain_->ctx_.clearing_cloud_);
  EXPECT_EQ(terrain_->ctx_.no_data_inited_, 0);
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
  terrain_->ctx_.processTerrainData();

  EXPECT_GT(terrain_->terrainCloudElev().points.size(), 0u)
      << "terrainCloudElev should contain output points";

  float max_intensity = 0;
  for (const auto& p : terrain_->terrainCloudElev().points) {
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
  terrain_->ctx_.processTerrainData();

  EXPECT_GT(terrain_->terrainCloudElev().points.size(), 0u);

  float max_intensity = 0;
  for (const auto& p : terrain_->terrainCloudElev().points) {
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
  terrain_->ctx_.processTerrainData();

  // The isolated obstacle point is in a voxel with < minBlockPointNum (10)
  // ground points, so it should be excluded from output
  bool found_isolated = false;
  for (const auto& p : terrain_->terrainCloudElev().points) {
    if (p.x > 2.5f && p.intensity > 0.1f) {
      found_isolated = true;
      break;
    }
  }
  EXPECT_FALSE(found_isolated)
      << "Isolated obstacle in sparse voxel should be excluded";
}
