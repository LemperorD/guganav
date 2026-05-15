#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <vector>

// ═══════════════════════════════════════════════
// Compile-time constants + runtime parameters
// ═══════════════════════════════════════════════

struct TerrainConfig {
  static constexpr int TERRAIN_VOXEL_WIDTH = 21;
  static constexpr int TERRAIN_VOXEL_HALF_WIDTH = (TERRAIN_VOXEL_WIDTH - 1) / 2;
  static constexpr int TERRAIN_VOXEL_NUM = TERRAIN_VOXEL_WIDTH
                                         * TERRAIN_VOXEL_WIDTH;

  static constexpr int PLANAR_VOXEL_WIDTH = 51;
  static constexpr int PLANAR_VOXEL_HALF_WIDTH = (PLANAR_VOXEL_WIDTH - 1) / 2;
  static constexpr int PLANAR_VOXEL_NUM = PLANAR_VOXEL_WIDTH
                                        * PLANAR_VOXEL_WIDTH;

  static constexpr size_t terrain_voxel_index(int row, int col) {
    return static_cast<size_t>(TERRAIN_VOXEL_WIDTH * row + col);
  }
  static constexpr size_t planar_voxel_index(int row, int col) {
    return static_cast<size_t>(PLANAR_VOXEL_WIDTH * row + col);
  }

  float scan_voxel_size = 0.05;
  float decay_time = 2.0;
  float no_decay_dis = 4.0;
  double clearing_dis = 8.0;
  bool use_sorting = true;
  double quantile_z = 0.25;
  bool consider_drop = false;
  bool limit_ground_lift = false;
  double max_ground_lift = 0.15;
  bool clear_dy_obs = false;
  double min_dy_obs_dis = 0.3;
  double min_dy_obs_angle = 0.0;
  double min_dy_obs_rel_z = -0.5;
  double abs_dy_obs_rel_z_thre = 0.2;
  double min_dy_obs_vfov = -16.0;
  double max_dy_obs_vfov = 16.0;
  int min_dy_obs_point_num = 1;
  bool no_data_obstacle = false;
  int no_data_block_skip_num = 0;
  int min_block_point_num = 10;
  double vehicle_height = 1.5;
  int voxel_point_update_thre = 100;
  double voxel_time_update_thre = 2.0;
  double min_rel_z = -1.5;
  double max_rel_z = 0.2;
  double dis_ratio_z = 0.2;

  double terrain_voxel_size = 1.0;
  double planar_voxel_size = 0.2;
};

// ═══════════════════════════════════════════════
// Mutable run-time state
// ═══════════════════════════════════════════════

struct TerrainState {
  enum class NoDataState : uint8_t {
    UNINITIALIZED = 0,
    RECORDING = 1,
    ACTIVE = 2
  };

  // Vehicle pose
  double vehicle_x = 0.0, vehicle_y = 0.0, vehicle_z = 0.0;
  double vehicle_x_rec = 0.0, vehicle_y_rec = 0.0;
  double sin_vehicle_roll = 0.0, cos_vehicle_roll = 0.0;
  double sin_vehicle_pitch = 0.0, cos_vehicle_pitch = 0.0;
  double sin_vehicle_yaw = 0.0, cos_vehicle_yaw = 0.0;

  // Point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_crop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_dwz;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev;
  std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr,
             TerrainConfig::TERRAIN_VOXEL_NUM>
      terrain_voxel_cloud;
  std::array<int, TerrainConfig::TERRAIN_VOXEL_NUM> terrain_voxel_update_num;
  std::array<double, TerrainConfig::TERRAIN_VOXEL_NUM>
      terrain_voxel_update_time;
  std::array<double, TerrainConfig::PLANAR_VOXEL_NUM> planar_voxel_elev;
  std::array<int, TerrainConfig::PLANAR_VOXEL_NUM> planar_voxel_edge;
  std::array<int, TerrainConfig::PLANAR_VOXEL_NUM> planar_voxel_dy_obs;
  std::array<std::vector<float>, TerrainConfig::PLANAR_VOXEL_NUM>
      planar_point_elev;

  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter;

  // Terrain voxel offsets
  int terrain_voxel_shift_x = 0;
  int terrain_voxel_shift_y = 0;

  // Laser cloud state
  double laser_cloud_time = 0.0;
  bool new_laser_cloud = false;

  // System state
  double system_init_time = 0.0;
  bool system_inited = false;
  NoDataState no_data_inited = NoDataState::UNINITIALIZED;

  // Mutated by callbacks
  bool clearing_cloud = false;
  double clearing_dis = 8.0;

  [[nodiscard]] double horizontalDistanceTo(double px, double py) const {
    return sqrt(((px - vehicle_x) * (px - vehicle_x))
                + ((py - vehicle_y) * (py - vehicle_y)));
  }

  [[nodiscard]] bool hasNewCloud() const {
    return new_laser_cloud;
  }
  [[nodiscard]] const auto& terrainCloudElev() const {
    return *terrain_cloud_elev;
  }
};

// ═══════════════════════════════════════════════
// Context — owns config + state, receives callbacks
// ═══════════════════════════════════════════════

class TerrainAnalysisContext {
public:
  TerrainAnalysisContext();
  ~TerrainAnalysisContext();

  TerrainAnalysisContext(const TerrainAnalysisContext&) = delete;
  TerrainAnalysisContext& operator=(const TerrainAnalysisContext&) = delete;
  TerrainAnalysisContext(TerrainAnalysisContext&&) = delete;
  TerrainAnalysisContext& operator=(TerrainAnalysisContext&&) = delete;

  void onOdometry(double x, double y, double z, double roll, double pitch,
                  double yaw);
  void onLaserCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    double timestamp_sec);
  void onJoystick(bool button5);
  void onClearing(float dis);

  TerrainConfig cfg;
  TerrainState state;
};
