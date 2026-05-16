// Copyright 2024 Hongbiao Zhu
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/context.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>

void TerrainAlgorithm::run(const TerrainConfig& config, TerrainState& state) {
  state.new_laser_cloud = false;

  rolloverVoxels(config, state);
  voxelize(config, state);
  updateVoxels(config, state);
  extractTerrainCloud(state);

  estimateGround(config, state);

  if (config.clear_dy_obs) {
    detectDynamicObstacles(config, state);
    filterDynamicObstaclePoints(config, state);
  }

  computeElevation(config, state);
  computeHeightMap(config, state);

  if (config.no_data_obstacle
      && state.no_data_inited == TerrainState::NoDataState::ACTIVE) {
    addNoDataObstacles(config, state);
  }

  if (config.check_terrain_connectivity) {
    checkTerrainConnectivity(config, state);
  }

  if (config.local_terrain_map_radius > 0.0) {
    mergeLocalTerrain(config, state);
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

  bool keepVoxelPoint(double relative_z, double distance, double point_time,
                      const TerrainConfig& config, const TerrainState& state) {
    const double z_margin = config.distance_ratio_z * distance;
    if (relative_z <= config.min_relative_z - z_margin) {
      return false;
    }
    if (relative_z >= config.max_relative_z + z_margin) {
      return false;
    }
    bool near = distance < config.no_decay_distance;
    bool decayed = (state.laser_cloud_time - state.system_init_time
                    - point_time)
                >= config.decay_time;
    if (decayed && !near) {
      return false;
    }
    if (distance < state.clearing_distance && state.clearing_cloud) {
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

  void elevateByQuantile(const TerrainConfig& config, TerrainState& state,
                         int cell) {
    auto& elevations = state.planar_point_elev[cell];
    int point_count = static_cast<int>(elevations.size());
    if (point_count == 0) {
      return;
    }
    sort(elevations.begin(), elevations.end());

    int quantile_index = static_cast<int>(config.quantile_z * point_count);
    if (quantile_index >= point_count) {
      quantile_index = point_count - 1;
    }
    double minimum_z = elevations[0];
    double quantile_z = elevations[quantile_index];
    state.planar_voxel_elev[cell] =
        config.limit_ground_lift
            ? std::min(quantile_z, minimum_z + config.max_ground_lift)
            : quantile_z;
  }

  void elevateByMinimum(const TerrainConfig& config, TerrainState& state,
                        int cell) {
    (void)config;
    auto& elevations = state.planar_point_elev[cell];
    if (elevations.empty()) {
      return;
    }
    state.planar_voxel_elev[cell] = *std::min_element(elevations.begin(),
                                                      elevations.end());
  }

  void markDataGapCells(const TerrainConfig& config, TerrainState& state) {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      if (state.planar_point_elev[i].size()
          < static_cast<size_t>(config.min_block_point_num)) {
        state.planar_voxel_edge[i] = 1;
      }
    }
  }

  bool hasLowerNeighbor(const TerrainState& state, int i, int row, int column) {
    static constexpr int WIDTH = TerrainConfig::PLANAR_VOXEL_WIDTH;
    for (int delta_row = -1; delta_row <= 1; delta_row++) {
      for (int delta_col = -1; delta_col <= 1; delta_col++) {
        int neighbor_row = row + delta_row;
        int neighbor_col = column + delta_col;
        if (neighbor_row >= 0 && neighbor_row < WIDTH && neighbor_col >= 0
            && neighbor_col < WIDTH) {
          size_t neighbor_index = TerrainConfig::planarVoxelIndex(
              neighbor_row, neighbor_col);
          if (state.planar_voxel_edge[neighbor_index]
              < state.planar_voxel_edge[i]) {
            return true;
          }
        }
      }
    }
    return false;
  }

  void expandEdgeLabels(const TerrainConfig& config, TerrainState& state) {
    static constexpr int WIDTH = TerrainConfig::PLANAR_VOXEL_WIDTH;
    for (int iteration = 0; iteration < config.no_data_block_skip_num;
         iteration++) {
      for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
        if (state.planar_voxel_edge[i] < 1) {
          continue;
        }
        int row = i / WIDTH;
        int column = i % WIDTH;
        if (!hasLowerNeighbor(state, i, row, column)) {
          state.planar_voxel_edge[i]++;
        }
      }
    }
  }

  void emitObstacleCloud(const TerrainConfig& config, TerrainState& state) {
    const double vehicle_x = state.vehicle_x;
    const double vehicle_y = state.vehicle_y;
    const double vehicle_z = state.vehicle_z;
    pcl::PointXYZI point;
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      if (state.planar_voxel_edge[i] <= config.no_data_block_skip_num) {
        continue;
      }
      int row = i / TerrainConfig::PLANAR_VOXEL_WIDTH;
      int column = i % TerrainConfig::PLANAR_VOXEL_WIDTH;

      point.x = static_cast<float>(
          (config.planar_voxel_size
           * (row - TerrainConfig::PLANAR_VOXEL_HALF_WIDTH))
          + vehicle_x);
      point.y = static_cast<float>(
          (config.planar_voxel_size
           * (column - TerrainConfig::PLANAR_VOXEL_HALF_WIDTH))
          + vehicle_y);
      point.z = static_cast<float>(vehicle_z);
      point.intensity = static_cast<float>(config.vehicle_height);

      point.x -= static_cast<float>(config.planar_voxel_size / 4.0);
      point.y -= static_cast<float>(config.planar_voxel_size / 4.0);
      state.terrain_cloud_elev->push_back(point);

      point.x += static_cast<float>(config.planar_voxel_size / 2.0);
      state.terrain_cloud_elev->push_back(point);

      point.y += static_cast<float>(config.planar_voxel_size / 2.0);
      state.terrain_cloud_elev->push_back(point);

      point.x -= static_cast<float>(config.planar_voxel_size / 2.0);
      state.terrain_cloud_elev->push_back(point);
    }
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
  double center_x = voxel_size * state.terrain_voxel_shift_x;
  double center_y = voxel_size * state.terrain_voxel_shift_y;

  while (state.vehicle_x - center_x < -voxel_size) {
    shiftGrid(state, Axis::X, false);
    center_x = voxel_size * --state.terrain_voxel_shift_x;
  }
  while (state.vehicle_x - center_x > voxel_size) {
    shiftGrid(state, Axis::X, true);
    center_x = voxel_size * ++state.terrain_voxel_shift_x;
  }
  while (state.vehicle_y - center_y < -voxel_size) {
    shiftGrid(state, Axis::Y, false);
    center_y = voxel_size * --state.terrain_voxel_shift_y;
  }
  while (state.vehicle_y - center_y > voxel_size) {
    shiftGrid(state, Axis::Y, true);
    center_y = voxel_size * ++state.terrain_voxel_shift_y;
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
      double distance = state.horizontalDistanceTo(point.x, point.y);
      if (keepVoxelPoint(point.z - vehicle_z, distance, point.intensity, config,
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
    if (relative_z <= config.min_relative_z || relative_z >= config.max_relative_z) {
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

    if (distance <= config.min_dy_obs_distance) {
      state.planar_voxel_dy_obs[cell] += config.min_dy_obs_point_num;
      continue;
    }

    double scan_angle = atan2(relative_z - config.min_dy_obs_relative_z, distance);
    if (scan_angle <= config.min_dy_obs_angle) {
      continue;
    }

    auto sensor = transformToSensorFrame(relative_x, relative_y, relative_z,
                                         state);
    double sensor_distance = sqrt((sensor.x * sensor.x)
                                  + (sensor.y * sensor.y));
    double sensor_angle = atan2(sensor.z, sensor_distance);
    if ((sensor_angle > config.min_dy_obs_vfov
         && sensor_angle < config.max_dy_obs_vfov)
        || std::abs(sensor.z) < config.abs_dy_obs_relative_z_threshold) {
      state.planar_voxel_dy_obs[cell]++;
    }
  }
}

void TerrainAlgorithm::filterDynamicObstaclePoints(const TerrainConfig& config,
                                                   TerrainState& state) {
  const double voxel_size = config.planar_voxel_size;
  constexpr int half_width = TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  constexpr int width = TerrainConfig::PLANAR_VOXEL_WIDTH;
  const double vehicle_x = state.vehicle_x;
  const double vehicle_y = state.vehicle_y;
  const double vehicle_z = state.vehicle_z;

  for (const auto& point : state.laser_cloud_crop->points) {
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
    double scan_angle = atan2(relative_z - config.min_dy_obs_relative_z, distance);
    if (scan_angle > config.min_dy_obs_angle) {
      state.planar_voxel_dy_obs[cell] = 0;
    }
  }
}

void TerrainAlgorithm::computeElevation(const TerrainConfig& config,
                                        TerrainState& state) {
  if (config.use_sorting) {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      elevateByQuantile(config, state, i);
    }
  } else {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      elevateByMinimum(config, state, i);
    }
  }
}

void TerrainAlgorithm::computeHeightMap(const TerrainConfig& config,
                                        TerrainState& state) {
  const double voxel_size = config.planar_voxel_size;
  constexpr int half_width = TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  constexpr int width = TerrainConfig::PLANAR_VOXEL_WIDTH;
  const double vehicle_x = state.vehicle_x;
  const double vehicle_y = state.vehicle_y;
  const double vehicle_z = state.vehicle_z;
  auto& elevations = state.terrain_cloud_elev;
  elevations->clear();

  for (const auto& point : state.terrain_cloud->points) {
    double relative_z = point.z - vehicle_z;
    if (relative_z <= config.min_relative_z || relative_z >= config.max_relative_z) {
      continue;
    }
    int col = toVoxelIndex(point.x, vehicle_x, voxel_size, half_width);
    int row = toVoxelIndex(point.y, vehicle_y, voxel_size, half_width);
    if (col < 0 || col >= width || row < 0 || row >= width) {
      continue;
    }
    size_t cell = TerrainConfig::planarVoxelIndex(row, col);
    if (state.planar_voxel_dy_obs[cell] >= config.min_dy_obs_point_num
        && config.clear_dy_obs) {
      continue;
    }

    double height_above_ground = point.z - state.planar_voxel_elev[cell];
    if (config.consider_drop) {
      height_above_ground = std::abs(height_above_ground);
    }

    auto point_count = state.planar_point_elev[cell].size();
    if (height_above_ground >= 0 && height_above_ground < config.vehicle_height
        && point_count >= static_cast<size_t>(config.min_block_point_num)) {
      elevations->push_back(point);
      elevations->back().intensity = static_cast<float>(height_above_ground);
    }
  }
}

void TerrainAlgorithm::addNoDataObstacles(const TerrainConfig& config,
                                          TerrainState& state) {
  markDataGapCells(config, state);
  expandEdgeLabels(config, state);
  emitObstacleCloud(config, state);
}

void TerrainAlgorithm::checkTerrainConnectivity(const TerrainConfig& config,
                                                TerrainState& state) {
  (void)config;
  (void)state;
}

void TerrainAlgorithm::mergeLocalTerrain(const TerrainConfig& config,
                                          TerrainState& state) {
  (void)config;
  (void)state;
}
