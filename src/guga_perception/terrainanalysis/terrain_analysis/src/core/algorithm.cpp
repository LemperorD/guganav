// Copyright 2024 Hongbiao Zhu
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/context.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>

// TerrainAlgorithm static methods

// ── Voxel grid shift helpers ──

void TerrainAlgorithm::run(const TerrainConfig& config, TerrainState& state) {
  state.new_laser_cloud = false;

  rolloverVoxels(config, state);
  voxelize(config, state);
  updateVoxels(config, state);
  extractTerrainCloud(state);

  estimateGround(config, state);

  if (config.clear_dy_obs) {
    detectDynamicObstacles(config, state);
    filterDynamicObstaclePoints(config, state,
                                state.laser_cloud_crop->points.size());
  }

  computeElevation(config, state);
  computeHeightMap(config, state, state.terrain_cloud->points.size());

  if (config.no_data_obstacle
      && state.no_data_inited == TerrainState::NoDataState::ACTIVE) {
    addNoDataObstacles(config, state);
  }

  state.clearing_cloud = false;
}

namespace {
  enum class Axis : uint8_t { X, Y };

  void shiftGrid(TerrainState& state, Axis axis, bool positive) {
    static constexpr int WIDTH = TerrainConfig::TERRAIN_VOXEL_WIDTH;
    const int src = positive ? 0 : WIDTH - 1;
    const int dst = positive ? WIDTH - 1 : 0;
    const int step = positive ? 1 : -1;

    for (int fixed = 0; fixed < WIDTH; fixed++) {
      auto cell = [&](int m) {
        return axis == Axis::X ? TerrainConfig::terrainVoxelIndex(m, fixed)
                               : TerrainConfig::terrainVoxelIndex(fixed, m);
      };
      auto ptr = state.terrain_voxel_cloud[cell(src)];
      for (int m = src; m != dst; m += step) {
        state.terrain_voxel_cloud[cell(m)] =
            state.terrain_voxel_cloud[cell(m + step)];
      }
      auto& dst_cell = state.terrain_voxel_cloud[cell(dst)];
      dst_cell = ptr;
      dst_cell->clear();
    }
  }

  int toVoxelIndex(float point_cloud, double vehicle_cloud, double voxel_size,
                   int half_width) {
    const double half_voxel_size = voxel_size / 2;
    int cell = static_cast<int>(
                   std::floor((point_cloud - vehicle_cloud + half_voxel_size)
                              / voxel_size))
             + half_width;
    return cell;
  }

  bool shouldPruneVoxel(const TerrainConfig& config, const TerrainState& state,
                        int cell) {
    if (state.clearing_cloud) {
      return true;
    }
    if (state.terrain_voxel_update_num[cell]
        >= config.voxel_point_update_thre) {
      return true;
    }
    double elapsed = state.laser_cloud_time - state.system_init_time
                   - state.terrain_voxel_update_time[cell];
    return elapsed >= config.voxel_time_update_thre;
  }

  bool keepVoxelPoint(double rel_z, double dis, double point_time,
                      const TerrainConfig& config, const TerrainState& state) {
    const double z_margin = config.dis_ratio_z * dis;
    if (rel_z <= config.min_rel_z - z_margin) {
      return false;
    }
    if (rel_z >= config.max_rel_z + z_margin) {
      return false;
    }
    bool near = dis < config.no_decay_dis;
    bool decayed = (state.laser_cloud_time - state.system_init_time
                    - point_time)
                >= config.decay_time;
    if (decayed && !near) {
      return false;
    }
    if (dis < state.clearing_dis && state.clearing_cloud) {
      return false;
    }
    return true;
  }

  struct SensorPoint {
    double x;
    double y;
    double z;
  };

  SensorPoint transformToSensorFrame(double x, double y, double z,
                                     const TerrainState& state) {
    double rotated_x = (x * state.cos_vehicle_yaw)
                     + (y * state.sin_vehicle_yaw);
    double rotated_y = -(x * state.sin_vehicle_yaw)
                     + (y * state.cos_vehicle_yaw);

    double pitched_x = (rotated_x * state.cos_vehicle_pitch)
                     - (z * state.sin_vehicle_pitch);
    double pitched_z = (rotated_x * state.sin_vehicle_pitch)
                     + (z * state.cos_vehicle_pitch);

    double rolled_y = (rotated_y * state.cos_vehicle_roll)
                    + (pitched_z * state.sin_vehicle_roll);
    double rolled_z = -(rotated_y * state.sin_vehicle_roll)
                    + (pitched_z * state.cos_vehicle_roll);

    return {pitched_x, rolled_y, rolled_z};
  }

  void resetPlanarVoxels(TerrainState& state) {
    state.planar_voxel_elev.fill(0);
    state.planar_voxel_edge.fill(0);
    state.planar_voxel_dy_obs.fill(0);
    for (auto& point_elevations : state.planar_point_elev) {
      point_elevations.clear();
    }
  }
}  // namespace

void TerrainAlgorithm::rolloverVoxels(const TerrainConfig& config,
                                      TerrainState& state) {
  const double voxel_size = config.terrain_voxel_size;
  double cen_x = voxel_size * state.terrain_voxel_shift_x;
  double cen_y = voxel_size * state.terrain_voxel_shift_y;

  while (state.vehicle_x - cen_x < -voxel_size) {
    shiftGrid(state, Axis::X, false);
    cen_x = voxel_size * --state.terrain_voxel_shift_x;
  }
  while (state.vehicle_x - cen_x > voxel_size) {
    shiftGrid(state, Axis::X, true);
    cen_x = voxel_size * ++state.terrain_voxel_shift_x;
  }
  while (state.vehicle_y - cen_y < -voxel_size) {
    shiftGrid(state, Axis::Y, false);
    cen_y = voxel_size * --state.terrain_voxel_shift_y;
  }
  while (state.vehicle_y - cen_y > voxel_size) {
    shiftGrid(state, Axis::Y, true);
    cen_y = voxel_size * ++state.terrain_voxel_shift_y;
  }
}

void TerrainAlgorithm::voxelize(const TerrainConfig& config,
                                TerrainState& state) {
  const double voxel_size = config.terrain_voxel_size;
  constexpr int voxel_half_width = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH;
  constexpr int voxel_width = TerrainConfig::TERRAIN_VOXEL_WIDTH;
  const double vehicle_x = state.vehicle_x;
  const double vehicle_y = state.vehicle_y;

  for (const auto& point : state.laser_cloud_crop->points) {
    int row = toVoxelIndex(point.x, vehicle_x, voxel_size, voxel_half_width);
    int column = toVoxelIndex(point.y, vehicle_y, voxel_size, voxel_half_width);
    if (row < 0 || row >= voxel_width || column < 0 || column >= voxel_width) {
      continue;
    }
    size_t cell = TerrainConfig::terrainVoxelIndex(row, column);
    state.terrain_voxel_cloud[cell]->push_back(point);
    state.terrain_voxel_update_num[cell]++;
  }
}

void TerrainAlgorithm::updateVoxels(const TerrainConfig& config,
                                    TerrainState& state) {
  const double laser_time = state.laser_cloud_time;
  const double init_time = state.system_init_time;
  const double vehicle_z = state.vehicle_z;

  for (int cell = 0; cell < TerrainConfig::TERRAIN_VOXEL_NUM; cell++) {
    if (!shouldPruneVoxel(config, state, cell)) {
      continue;
    }
    auto& cell_cloud = *state.terrain_voxel_cloud[cell];

    state.laser_cloud_downsampled->clear();
    state.down_size_filter.setInputCloud(state.terrain_voxel_cloud[cell]);
    state.down_size_filter.filter(*state.laser_cloud_downsampled);

    cell_cloud.clear();
    for (const auto& point : state.laser_cloud_downsampled->points) {
      double dis = state.horizontalDistanceTo(point.x, point.y);
      if (keepVoxelPoint(point.z - vehicle_z, dis, point.intensity, config,
                         state)) {
        cell_cloud.push_back(point);
      }
    }

    state.terrain_voxel_update_num[cell] = 0;
    state.terrain_voxel_update_time[cell] = laser_time - init_time;
  }
}

void TerrainAlgorithm::extractTerrainCloud(TerrainState& state) {
  static constexpr int EXTRACT_HALF_WINDOW = 5;
  state.terrain_cloud->clear();
  for (int row = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH - EXTRACT_HALF_WINDOW;
       row <= TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + EXTRACT_HALF_WINDOW;
       row++) {
    for (int column =
             TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH - EXTRACT_HALF_WINDOW;
         column
         <= TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + EXTRACT_HALF_WINDOW;
         column++) {
      *state.terrain_cloud +=
          *state.terrain_voxel_cloud[TerrainConfig::terrainVoxelIndex(row,
                                                                      column)];
    }
  }
}

void TerrainAlgorithm::estimateGround(const TerrainConfig& config,
                                      TerrainState& state) {
  resetPlanarVoxels(state);

  const double voxel_size = config.planar_voxel_size;
  constexpr int half_width = TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  constexpr int width = TerrainConfig::PLANAR_VOXEL_WIDTH;
  const double vehicle_x = state.vehicle_x;
  const double vehicle_y = state.vehicle_y;
  const double vehicle_z = state.vehicle_z;

  for (const auto& point : state.terrain_cloud->points) {
    int col = toVoxelIndex(point.x, vehicle_x, voxel_size, half_width);
    int row = toVoxelIndex(point.y, vehicle_y, voxel_size, half_width);
    double relative_z = point.z - vehicle_z;
    if (relative_z <= config.min_rel_z || relative_z >= config.max_rel_z) {
      continue;
    }
    size_t base = TerrainConfig::planarVoxelIndex(row, col);
    static constexpr int PLANAR_VOXEL_WIDTH = TerrainConfig::PLANAR_VOXEL_WIDTH;
    for (int delta_row = -1; delta_row <= 1; delta_row++) {
      int neighbor_row = row + delta_row;
      if (neighbor_row < 0 || neighbor_row >= width) {
        continue;
      }
      for (int delta_col = -1; delta_col <= 1; delta_col++) {
        int neighbor_col = col + delta_col;
        if (neighbor_col >= 0 && neighbor_col < width) {
          int index = static_cast<int>(base) + (delta_row * PLANAR_VOXEL_WIDTH)
                    + delta_col;
          state.planar_point_elev[static_cast<size_t>(index)].push_back(
              point.z);
        }
      }
    }
  }
}

void TerrainAlgorithm::detectDynamicObstacles(const TerrainConfig& config,
                                              TerrainState& state) {
  const double voxel_size = config.planar_voxel_size;
  constexpr int half_width = TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  constexpr int width = TerrainConfig::PLANAR_VOXEL_WIDTH;
  const double vehicle_x = state.vehicle_x;
  const double vehicle_y = state.vehicle_y;
  const double vehicle_z = state.vehicle_z;

  for (const auto& point : state.terrain_cloud->points) {
    int col = toVoxelIndex(point.x, vehicle_x, voxel_size, half_width);
    int row = toVoxelIndex(point.y, vehicle_y, voxel_size, half_width);
    if (col < 0 || col >= width || row < 0 || row >= width) {
      continue;
    }
    size_t cell = TerrainConfig::planarVoxelIndex(row, col);

    double relative_x = point.x - vehicle_x;
    double relative_y = point.y - vehicle_y;
    double relative_z = point.z - vehicle_z;
    double distance = sqrt((relative_x * relative_x)
                           + (relative_y * relative_y));

    if (distance <= config.min_dy_obs_dis) {
      state.planar_voxel_dy_obs[cell] += config.min_dy_obs_point_num;
      continue;
    }

    double scan_angle = atan2(relative_z - config.min_dy_obs_rel_z, distance)
                      * 180.0 / M_PI;
    if (scan_angle <= config.min_dy_obs_angle) {
      continue;
    }

    auto sensor = transformToSensorFrame(relative_x, relative_y, relative_z,
                                         state);
    double sensor_distance = sqrt((sensor.x * sensor.x)
                                  + (sensor.y * sensor.y));
    double sensor_angle = atan2(sensor.z, sensor_distance) * 180.0 / M_PI;
    if ((sensor_angle > config.min_dy_obs_vfov
         && sensor_angle < config.max_dy_obs_vfov)
        || std::abs(sensor.z) < config.abs_dy_obs_rel_z_thre) {
      state.planar_voxel_dy_obs[cell]++;
    }
  }
}

void TerrainAlgorithm::filterDynamicObstaclePoints(
    const TerrainConfig& config, TerrainState& state,
    size_t laser_cloud_crop_size) {
  const double voxel_size = config.planar_voxel_size;
  constexpr int half_width = TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  constexpr int width = TerrainConfig::PLANAR_VOXEL_WIDTH;
  pcl::PointXYZI point;
  for (size_t i = 0; i < laser_cloud_crop_size; i++) {
    point = state.laser_cloud_crop->points[i];
    int col = toVoxelIndex(point.x, state.vehicle_x, voxel_size, half_width);
    int row = toVoxelIndex(point.y, state.vehicle_y, voxel_size, half_width);
    if (col < 0 || col >= width || row < 0 || row >= width) {
      continue;
    }

    float pointX1 = point.x - state.vehicle_x;
    float pointY1 = point.y - state.vehicle_y;
    float pointZ1 = point.z - state.vehicle_z;
    float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
    float angle1 = atan2(pointZ1 - config.min_dy_obs_rel_z, dis1) * 180.0
                 / M_PI;
    if (angle1 > config.min_dy_obs_angle) {
      state.planar_voxel_dy_obs[TerrainConfig::planarVoxelIndex(row, col)] = 0;
    }
  }
}

void TerrainAlgorithm::computeElevation(const TerrainConfig& config,
                                        TerrainState& state) {
  if (config.use_sorting) {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      int size = state.planar_point_elev[i].size();
      if (size == 0) {
        continue;
      }
      sort(state.planar_point_elev[i].begin(),
           state.planar_point_elev[i].end());

      int quantileID = static_cast<int>(config.quantile_z * size);
      if (quantileID < 0) {
        quantileID = 0;
      } else if (quantileID >= size) {
        quantileID = size - 1;
      }

      if (state.planar_point_elev[i][quantileID]
              > state.planar_point_elev[i][0] + config.max_ground_lift
          && config.limit_ground_lift) {
        state.planar_voxel_elev[i] = state.planar_point_elev[i][0]
                                   + config.max_ground_lift;
      } else {
        state.planar_voxel_elev[i] = state.planar_point_elev[i][quantileID];
      }
    }
  } else {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      int size = state.planar_point_elev[i].size();
      if (size == 0) {
        continue;
      }
      float minZ = 1000.0;
      int minID = -1;
      for (int j = 0; j < size; j++) {
        if (state.planar_point_elev[i][j] < minZ) {
          minZ = state.planar_point_elev[i][j];
          minID = j;
        }
      }
      if (minID != -1) {
        state.planar_voxel_elev[i] = state.planar_point_elev[i][minID];
      }
    }
  }
}

void TerrainAlgorithm::computeHeightMap(const TerrainConfig& config,
                                        TerrainState& state,
                                        size_t terrain_cloud_size) {
  const double voxel_size = config.planar_voxel_size;
  constexpr int half_width = TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  constexpr int width = TerrainConfig::PLANAR_VOXEL_WIDTH;
  state.terrain_cloud_elev->clear();
  size_t terrain_cloud_elev_size = 0;
  pcl::PointXYZI point;

  for (size_t i = 0; i < terrain_cloud_size; i++) {
    point = state.terrain_cloud->points[i];
    if (!(point.z - state.vehicle_z > config.min_rel_z
          && point.z - state.vehicle_z < config.max_rel_z)) {
      continue;
    }
    int col = toVoxelIndex(point.x, state.vehicle_x, voxel_size, half_width);
    int row = toVoxelIndex(point.y, state.vehicle_y, voxel_size, half_width);
    if (col < 0 || col >= width || row < 0 || row >= width) {
      continue;
    }
    int cell = TerrainConfig::planarVoxelIndex(row, col);
    if (state.planar_voxel_dy_obs[cell] >= config.min_dy_obs_point_num
        && config.clear_dy_obs) {
      continue;
    }

    float disZ = point.z - state.planar_voxel_elev[cell];
    if (config.consider_drop) {
      disZ = fabs(disZ);
    }

    int planarSize = state.planar_point_elev[cell].size();
    if (disZ >= 0 && disZ < config.vehicle_height
        && planarSize >= config.min_block_point_num) {
      state.terrain_cloud_elev->push_back(point);
      state.terrain_cloud_elev->points[terrain_cloud_elev_size].intensity =
          disZ;
      terrain_cloud_elev_size++;
    }
  }
}

void TerrainAlgorithm::addNoDataObstacles(const TerrainConfig& config,
                                          TerrainState& state) {
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    if (state.planar_point_elev[i].size()
        < static_cast<size_t>(config.min_block_point_num)) {
      state.planar_voxel_edge[i] = 1;
    }
  }

  for (int noDataBlockSkipCount = 0;
       noDataBlockSkipCount < config.no_data_block_skip_num;
       noDataBlockSkipCount++) {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      if (state.planar_voxel_edge[i] < 1) {
        continue;
      }
      int row = i / TerrainConfig::PLANAR_VOXEL_WIDTH;
      int column = i % TerrainConfig::PLANAR_VOXEL_WIDTH;
      bool edgeVoxel = false;
      for (int dX = -1; dX <= 1 && !edgeVoxel; dX++) {
        for (int dY = -1; dY <= 1 && !edgeVoxel; dY++) {
          if (row + dX >= 0 && row + dX < TerrainConfig::PLANAR_VOXEL_WIDTH
              && column + dY >= 0
              && column + dY < TerrainConfig::PLANAR_VOXEL_WIDTH) {
            int ni = TerrainConfig::planarVoxelIndex(row + dX, column) + dY;
            if (state.planar_voxel_edge[ni] < state.planar_voxel_edge[i]) {
              edgeVoxel = true;
            }
          }
        }
      }
      if (!edgeVoxel) {
        state.planar_voxel_edge[i]++;
      }
    }
  }

  pcl::PointXYZI point;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    if (state.planar_voxel_edge[i] <= config.no_data_block_skip_num) {
      continue;
    }
    int row = i / TerrainConfig::PLANAR_VOXEL_WIDTH;
    int column = i % TerrainConfig::PLANAR_VOXEL_WIDTH;

    point.x = config.planar_voxel_size
                * (row - TerrainConfig::PLANAR_VOXEL_HALF_WIDTH)
            + state.vehicle_x;
    point.y = config.planar_voxel_size
                * (column - TerrainConfig::PLANAR_VOXEL_HALF_WIDTH)
            + state.vehicle_y;
    point.z = state.vehicle_z;
    point.intensity = config.vehicle_height;

    point.x -= config.planar_voxel_size / 4.0;
    point.y -= config.planar_voxel_size / 4.0;
    state.terrain_cloud_elev->push_back(point);

    point.x += config.planar_voxel_size / 2.0;
    state.terrain_cloud_elev->push_back(point);

    point.y += config.planar_voxel_size / 2.0;
    state.terrain_cloud_elev->push_back(point);

    point.x -= config.planar_voxel_size / 2.0;
    state.terrain_cloud_elev->push_back(point);
  }
}
