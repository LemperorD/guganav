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

  rolloverTerrainVoxels(config, state);
  stackLaserScans(config, state);
  updateVoxels(config, state);
  extractTerrainCloud(config, state);

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
  // Shift grid along X axis: iterates columns, shifts rows.
  // positive=True: shift toward +X (right), positive=False: toward -X (left).
  enum class Axis : uint8_t { X, Y };

  void shiftGrid(TerrainState& st, Axis axis, bool positive) {
    static constexpr int WIDTH = TerrainConfig::TERRAIN_VOXEL_WIDTH;
    const int src = positive ? 0 : WIDTH - 1;
    const int dst = positive ? WIDTH - 1 : 0;
    const int step = positive ? 1 : -1;

    for (int fixed = 0; fixed < WIDTH; fixed++) {
      auto cell = [&](int m) {
        return axis == Axis::X ? TerrainConfig::terrain_voxel_index(m, fixed)
                               : TerrainConfig::terrain_voxel_index(fixed, m);
      };
      auto ptr = st.terrain_voxel_cloud[cell(src)];
      for (int m = src; m != dst; m += step) {
        st.terrain_voxel_cloud[cell(m)] =
            st.terrain_voxel_cloud[cell(m + step)];
      }
      auto& dst_cell = st.terrain_voxel_cloud[cell(dst)];
      dst_cell = ptr;
      dst_cell->clear();
    }
  }
}  // namespace

void TerrainAlgorithm::rolloverTerrainVoxels(const TerrainConfig& config,
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

namespace {
  int toVoxelIndex(float point_cloud, double vehicle_cloud, double voxel_size,
                   int half_width) {
    const double half_voxel_size = voxel_size / 2;
    int cell = static_cast<int>(
                   std::floor((point_cloud - vehicle_cloud + half_voxel_size)
                              / voxel_size))
             + half_width;
    return cell;
  }
}  // namespace

void TerrainAlgorithm::stackLaserScans(const TerrainConfig& config,
                                       TerrainState& state) {
  pcl::PointXYZI point;
  const double voxel_size = config.terrain_voxel_size;
  constexpr int voxel_half_width = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH;
  constexpr int voxel_width = TerrainConfig::TERRAIN_VOXEL_WIDTH;

  size_t size = state.laser_cloud_crop->points.size();
  for (size_t i = 0; i < size; i++) {
    point = state.laser_cloud_crop->points[i];
    int row = toVoxelIndex(point.x, state.vehicle_x, voxel_size,
                           voxel_half_width);
    int column = toVoxelIndex(point.y, state.vehicle_y, voxel_size,
                              voxel_half_width);
    if (row < 0 || row >= voxel_width || column < 0 || column >= voxel_width) {
      continue;
    }
    auto cell = TerrainConfig::terrain_voxel_index(row, column);
    state.terrain_voxel_cloud[cell]->push_back(point);
    state.terrain_voxel_update_num[cell]++;
  }
}

void TerrainAlgorithm::updateVoxels(const TerrainConfig& config,
                                    TerrainState& state) {
  pcl::PointXYZI point;
  for (int ind = 0; ind < TerrainConfig::TERRAIN_VOXEL_NUM; ind++) {
    if (state.terrain_voxel_update_num[ind] >= config.voxel_point_update_thre
        || state.laser_cloud_time - state.system_init_time
                   - state.terrain_voxel_update_time[ind]
               >= config.voxel_time_update_thre
        || state.clearing_cloud) {
      auto voxel_cloud_ptr = state.terrain_voxel_cloud[ind];

      state.laser_cloud_dwz->clear();
      state.down_size_filter.setInputCloud(voxel_cloud_ptr);
      state.down_size_filter.filter(*state.laser_cloud_dwz);

      voxel_cloud_ptr->clear();
      size_t cloud_size = state.laser_cloud_dwz->points.size();
      for (size_t i = 0; i < cloud_size; i++) {
        point = state.laser_cloud_dwz->points[i];
        double dis = state.horizontalDistanceTo(point.x, point.y);
        if (point.z - state.vehicle_z
                > config.min_rel_z - (config.dis_ratio_z * dis)
            && point.z - state.vehicle_z
                   < config.max_rel_z + (config.dis_ratio_z * dis)
            && (state.laser_cloud_time - state.system_init_time
                        - point.intensity
                    < config.decay_time
                || dis < config.no_decay_dis)
            && (dis >= state.clearing_dis || !state.clearing_cloud)) {
          voxel_cloud_ptr->push_back(point);
        }
      }

      state.terrain_voxel_update_num[ind] = 0;
      state.terrain_voxel_update_time[ind] = state.laser_cloud_time
                                           - state.system_init_time;
    }
  }
}

void TerrainAlgorithm::extractTerrainCloud(const TerrainConfig& config,
                                           TerrainState& state) {
  (void)config;
  state.terrain_cloud->clear();
  for (int row = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH - 5;
       row <= TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + 5; row++) {
    for (int column = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH - 5;
         column <= TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + 5; column++) {
      *state.terrain_cloud +=
          *state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(
              row, column)];
    }
  }
}

void TerrainAlgorithm::estimateGround(const TerrainConfig& config,
                                      TerrainState& state) {
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    state.planar_voxel_elev[i] = 0;
    state.planar_voxel_edge[i] = 0;
    state.planar_voxel_dy_obs[i] = 0;
    state.planar_point_elev[i].clear();
  }
  const double voxel_size = config.planar_voxel_size;
  constexpr int half_width = TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
  constexpr int width = TerrainConfig::PLANAR_VOXEL_WIDTH;
  pcl::PointXYZI point;
  size_t size = state.terrain_cloud->points.size();
  for (size_t i = 0; i < size; i++) {
    point = state.terrain_cloud->points[i];
    int ix = toVoxelIndex(point.x, state.vehicle_x, voxel_size, half_width);
    int iy = toVoxelIndex(point.y, state.vehicle_y, voxel_size, half_width);
    if (point.z - state.vehicle_z <= config.min_rel_z
        || point.z - state.vehicle_z >= config.max_rel_z) {
      continue;
    }
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        int nx = ix + dx, ny = iy + dy;
        if (nx >= 0 && nx < width && ny >= 0 && ny < width) {
          state.planar_point_elev[TerrainConfig::planar_voxel_index(nx, ny)]
              .push_back(point.z);
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
  pcl::PointXYZI point;
  size_t sz = state.terrain_cloud->points.size();
  for (size_t i = 0; i < sz; i++) {
    point = state.terrain_cloud->points[i];
    int ix = toVoxelIndex(point.x, state.vehicle_x, voxel_size, half_width);
    int iy = toVoxelIndex(point.y, state.vehicle_y, voxel_size, half_width);
    if (ix < 0 || ix >= width || iy < 0 || iy >= width) {
      continue;
    }

    float pointX1 = point.x - state.vehicle_x;
    float pointY1 = point.y - state.vehicle_y;
    float pointZ1 = point.z - state.vehicle_z;
    float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);

    if (dis1 <= config.min_dy_obs_dis) {
      state.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(ix, iy)] +=
          config.min_dy_obs_point_num;
      continue;
    }

    float angle1 = atan2(pointZ1 - config.min_dy_obs_rel_z, dis1) * 180.0
                 / M_PI;
    if (angle1 <= config.min_dy_obs_angle) {
      continue;
    }

    float pointX2 = pointX1 * state.cos_vehicle_yaw
                  + pointY1 * state.sin_vehicle_yaw;
    float pointY2 = -pointX1 * state.sin_vehicle_yaw
                  + pointY1 * state.cos_vehicle_yaw;
    float pointZ2 = pointZ1;

    float pointX3 = pointX2 * state.cos_vehicle_pitch
                  - pointZ2 * state.sin_vehicle_pitch;
    float pointY3 = pointY2;
    float pointZ3 = pointX2 * state.sin_vehicle_pitch
                  + pointZ2 * state.cos_vehicle_pitch;

    float pointX4 = pointX3;
    float pointY4 = pointY3 * state.cos_vehicle_roll
                  + pointZ3 * state.sin_vehicle_roll;
    float pointZ4 = -pointY3 * state.sin_vehicle_roll
                  + pointZ3 * state.cos_vehicle_roll;

    float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
    float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
    if ((angle4 > config.min_dy_obs_vfov && angle4 < config.max_dy_obs_vfov)
        || fabs(pointZ4) < config.abs_dy_obs_rel_z_thre) {
      state.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(ix, iy)]++;
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
    int ix = toVoxelIndex(point.x, state.vehicle_x, voxel_size, half_width);
    int iy = toVoxelIndex(point.y, state.vehicle_y, voxel_size, half_width);
    if (ix < 0 || ix >= width || iy < 0 || iy >= width) {
      continue;
    }

    float pointX1 = point.x - state.vehicle_x;
    float pointY1 = point.y - state.vehicle_y;
    float pointZ1 = point.z - state.vehicle_z;
    float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
    float angle1 = atan2(pointZ1 - config.min_dy_obs_rel_z, dis1) * 180.0
                 / M_PI;
    if (angle1 > config.min_dy_obs_angle) {
      state.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(ix, iy)] = 0;
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
    int ix = toVoxelIndex(point.x, state.vehicle_x, voxel_size, half_width);
    int iy = toVoxelIndex(point.y, state.vehicle_y, voxel_size, half_width);
    if (ix < 0 || ix >= width || iy < 0 || iy >= width) {
      continue;
    }
    int cell = TerrainConfig::planar_voxel_index(ix, iy);
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
            int ni = TerrainConfig::planar_voxel_index(row + dX, column) + dY;
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
