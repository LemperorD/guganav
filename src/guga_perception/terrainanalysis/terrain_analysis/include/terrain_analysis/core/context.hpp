#pragma once

#include <gtest/internal/gtest-internal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

class TerrainAnalysisContext {
  friend class TerrainAnalysisTest;

public:
  TerrainAnalysisContext();
  ~TerrainAnalysisContext();

  TerrainAnalysisContext(const TerrainAnalysisContext&) = delete;
  TerrainAnalysisContext& operator=(const TerrainAnalysisContext&) = delete;
  TerrainAnalysisContext(TerrainAnalysisContext&&) = delete;
  TerrainAnalysisContext& operator=(TerrainAnalysisContext&&) = delete;

  // Callbacks (domain types only, no ROS2)
  void onOdometry(double x, double y, double z, double roll, double pitch,
                  double yaw);
  void onLaserCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    double timestamp_sec);
  void onJoystick(bool button5);
  void onClearing(float dis);

  const pcl::PointCloud<pcl::PointXYZI>& terrainCloudElev() const {
    return *terrain_cloud_elev_;
  }

  // ── public data access for node pipeline ──
  // (temporary until processTerrainData moves into Context)

  static constexpr int kTerrainVoxelWidth = 21;
  static constexpr int kTerrainVoxelHalfWidth = (kTerrainVoxelWidth - 1) / 2;
  static constexpr int kTerrainVoxelNum = kTerrainVoxelWidth
                                        * kTerrainVoxelWidth;

  static constexpr int kPlanarVoxelWidth = 51;
  static constexpr int kPlanarVoxelHalfWidth = (kPlanarVoxelWidth - 1) / 2;
  static constexpr int kPlanarVoxelNum = kPlanarVoxelWidth * kPlanarVoxelWidth;

  // Parameters
  float scan_voxel_size_ = 0.05;
  float decay_time_ = 2.0;
  float no_decay_dis_ = 4.0;
  float clearing_dis_ = 8.0;
  bool clearing_cloud_ = false;
  bool use_sorting_ = true;
  double quantile_z_ = 0.25;
  bool consider_drop_ = false;
  bool limit_ground_lift_ = false;
  double max_ground_lift_ = 0.15;
  bool clear_dy_obs_ = false;
  double min_dy_obs_dis_ = 0.3;
  double min_dy_obs_angle_ = 0.0;
  double min_dy_obs_rel_z_ = -0.5;
  double abs_dy_obs_rel_z_thre_ = 0.2;
  double min_dy_obs_vfov_ = -16.0;
  double max_dy_obs_vfov_ = 16.0;
  int min_dy_obs_point_num_ = 1;
  bool no_data_obstacle_ = false;
  int no_data_block_skip_num_ = 0;
  int min_block_point_num_ = 10;
  double vehicle_height_ = 1.5;
  int voxel_point_update_thre_ = 100;
  double voxel_time_update_thre_ = 2.0;
  double min_rel_z_ = -1.5;
  double max_rel_z_ = 0.2;
  double dis_ratio_z_ = 0.2;

  // Terrain voxel
  float terrain_voxel_size_ = 1.0f;
  int terrain_voxel_shift_x_ = 0;
  int terrain_voxel_shift_y_ = 0;

  // Planar voxel
  float planar_voxel_size_ = 0.2f;

  // Point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_crop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_dwz_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_voxel_cloud_[kTerrainVoxelNum]{};

  int terrain_voxel_update_num_[kTerrainVoxelNum]{};
  float terrain_voxel_update_time_[kTerrainVoxelNum]{};
  float planar_voxel_elev_[kPlanarVoxelNum]{};
  int planar_voxel_edge_[kPlanarVoxelNum]{};
  int planar_voxel_dy_obs_[kPlanarVoxelNum]{};
  std::vector<float> planar_point_elev_[kPlanarVoxelNum];

  double laser_cloud_time_ = 0.0;
  bool new_laser_cloud_ = false;

  double system_init_time_ = 0.0;
  bool system_inited_ = false;
  int no_data_inited_ = 0;

  // Vehicle pose
  float vehicle_roll_ = 0.0f, vehicle_pitch_ = 0.0f, vehicle_yaw_ = 0.0f;
  float vehicle_x_ = 0.0f, vehicle_y_ = 0.0f, vehicle_z_ = 0.0f;
  float vehicle_x_rec_ = 0.0f, vehicle_y_rec_ = 0.0f;

  float sin_vehicle_roll_ = 0.0f, cos_vehicle_roll_ = 0.0f;
  float sin_vehicle_pitch_ = 0.0f, cos_vehicle_pitch_ = 0.0f;
  float sin_vehicle_yaw_ = 0.0f, cos_vehicle_yaw_ = 0.0f;

  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;
};
