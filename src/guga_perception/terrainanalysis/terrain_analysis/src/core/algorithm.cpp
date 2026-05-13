// Copyright 2024 Hongbiao Zhu
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/context.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

// TerrainAlgorithm static methods

// ── Voxel grid shift helpers ──

namespace {
  // Shift grid along X axis: iterates columns, shifts rows.
  // positive=True: shift toward +X (right), positive=False: toward -X (left).
  void shiftGridX(TerrainState& st, bool positive) {
    static constexpr int W = TerrainConfig::TERRAIN_VOXEL_WIDTH;
    if (positive) {
      for (int col = 0; col < W; col++) {
        auto ptr = st.terrain_voxel_cloud[col];  // row 0
        for (int row = 0; row < W - 1; row++) {
          st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, col)] =
              st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row + 1,
                                                                        col)];
        }
        st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(W - 1, col)] =
            ptr;
        st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(W - 1, col)]
            ->clear();
      }
    } else {
      for (int col = 0; col < W; col++) {
        auto ptr = st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(
            W - 1, col)];
        for (int row = W - 1; row >= 1; row--) {
          st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, col)] =
              st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row - 1,
                                                                        col)];
        }
        st.terrain_voxel_cloud[col] = ptr;
        st.terrain_voxel_cloud[col]->clear();
      }
    }
  }

  // Shift grid along Y axis: iterates rows, shifts columns.
  void shiftGridY(TerrainState& st, bool positive) {
    static constexpr int W = TerrainConfig::TERRAIN_VOXEL_WIDTH;
    if (positive) {
      for (int row = 0; row < W; row++) {
        auto ptr =
            st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, 0)];
        for (int col = 0; col < W - 1; col++) {
          st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, col)] =
              st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(
                  row, col + 1)];
        }
        st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, W - 1)] =
            ptr;
        st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, W - 1)]
            ->clear();
      }
    } else {
      for (int row = 0; row < W; row++) {
        auto ptr = st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(
            row, W - 1)];
        for (int col = W - 1; col >= 1; col--) {
          st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, col)] =
              st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(
                  row, col - 1)];
        }
        st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, 0)] =
            ptr;
        st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(row, 0)]
            ->clear();
      }
    }
  }
}  // namespace

void TerrainAlgorithm::rolloverTerrainVoxels(const TerrainConfig& config,
                                             TerrainState& state) {
  const float vs = config.terrain_voxel_size;
  float cen_x = vs * state.terrain_voxel_shift_x;
  float cen_y = vs * state.terrain_voxel_shift_y;

  while (state.vehicle_x - cen_x < -vs) {
    shiftGridX(state, false);
    cen_x = vs * --state.terrain_voxel_shift_x;
  }
  while (state.vehicle_x - cen_x > vs) {
    shiftGridX(state, true);
    cen_x = vs * ++state.terrain_voxel_shift_x;
  }
  while (state.vehicle_y - cen_y < -vs) {
    shiftGridY(state, false);
    cen_y = vs * --state.terrain_voxel_shift_y;
  }
  while (state.vehicle_y - cen_y > vs) {
    shiftGridY(state, true);
    cen_y = vs * ++state.terrain_voxel_shift_y;
  }
}

void TerrainAlgorithm::stackLaserScans(const TerrainConfig& config,
                                       TerrainState& state) {
  pcl::PointXYZI point;
  int laserCloudCropSize = state.laser_cloud_crop->points.size();
  for (int i = 0; i < laserCloudCropSize; i++) {
    point = state.laser_cloud_crop->points[i];

    int ind_x = static_cast<int>(
                    (point.x - state.vehicle_x + config.terrain_voxel_size / 2)
                    / config.terrain_voxel_size)
              + TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH;
    int ind_y = static_cast<int>(
                    (point.y - state.vehicle_y + config.terrain_voxel_size / 2)
                    / config.terrain_voxel_size)
              + TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH;

    if (point.x - state.vehicle_x + config.terrain_voxel_size / 2 < 0) {
      ind_x--;
    }
    if (point.y - state.vehicle_y + config.terrain_voxel_size / 2 < 0) {
      ind_y--;
    }

    if (ind_x >= 0 && ind_x < TerrainConfig::TERRAIN_VOXEL_WIDTH && ind_y >= 0
        && ind_y < TerrainConfig::TERRAIN_VOXEL_WIDTH) {
      state
          .terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(ind_x, ind_y)]
          ->push_back(point);
      state.terrain_voxel_update_num[TerrainConfig::terrain_voxel_index(
          ind_x, ind_y)]++;
    }
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
      int cloud_size = state.laser_cloud_dwz->points.size();
      for (int i = 0; i < cloud_size; i++) {
        point = state.laser_cloud_dwz->points[i];
        float dis = state.horizontalDistanceTo(point.x, point.y);
        if (point.z - state.vehicle_z
                > config.min_rel_z - config.dis_ratio_z * dis
            && point.z - state.vehicle_z
                   < config.max_rel_z + config.dis_ratio_z * dis
            && (state.laser_cloud_time - state.system_init_time
                        - point.intensity
                    < config.decay_time
                || dis < config.no_decay_dis)
            && !(dis < state.clearing_dis && state.clearing_cloud)) {
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
  for (int ind_x = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH - 5;
       ind_x <= TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + 5; ind_x++) {
    for (int ind_y = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH - 5;
         ind_y <= TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + 5; ind_y++) {
      *state.terrain_cloud +=
          *state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(ind_x,
                                                                        ind_y)];
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

  pcl::PointXYZI point;
  int terrainCloudSize = state.terrain_cloud->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = state.terrain_cloud->points[i];

    int ind_x = static_cast<int>(
                    (point.x - state.vehicle_x + config.planar_voxel_size / 2)
                    / config.planar_voxel_size)
              + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
    int ind_y = static_cast<int>(
                    (point.y - state.vehicle_y + config.planar_voxel_size / 2)
                    / config.planar_voxel_size)
              + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;

    if (point.x - state.vehicle_x + config.planar_voxel_size / 2 < 0) {
      ind_x--;
    }
    if (point.y - state.vehicle_y + config.planar_voxel_size / 2 < 0) {
      ind_y--;
    }

    if (point.z - state.vehicle_z > config.min_rel_z
        && point.z - state.vehicle_z < config.max_rel_z) {
      for (int dX = -1; dX <= 1; dX++) {
        for (int dY = -1; dY <= 1; dY++) {
          if (ind_x + dX >= 0 && ind_x + dX < TerrainConfig::PLANAR_VOXEL_WIDTH
              && ind_y + dY >= 0
              && ind_y + dY < TerrainConfig::PLANAR_VOXEL_WIDTH) {
            state
                .planar_point_elev[TerrainConfig::planar_voxel_index(
                    ind_x + dX, ind_y + dY)]
                .push_back(point.z);
          }
        }
      }
    }
  }
}

void TerrainAlgorithm::detectDynamicObstacles(const TerrainConfig& config,
                                              TerrainState& state) {
  pcl::PointXYZI point;
  int terrainCloudSize = state.terrain_cloud->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = state.terrain_cloud->points[i];

    int ind_x = static_cast<int>(
                    (point.x - state.vehicle_x + config.planar_voxel_size / 2)
                    / config.planar_voxel_size)
              + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
    int ind_y = static_cast<int>(
                    (point.y - state.vehicle_y + config.planar_voxel_size / 2)
                    / config.planar_voxel_size)
              + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;

    if (point.x - state.vehicle_x + config.planar_voxel_size / 2 < 0) {
      ind_x--;
    }
    if (point.y - state.vehicle_y + config.planar_voxel_size / 2 < 0) {
      ind_y--;
    }

    if (ind_x < 0 || ind_x >= TerrainConfig::PLANAR_VOXEL_WIDTH || ind_y < 0
        || ind_y >= TerrainConfig::PLANAR_VOXEL_WIDTH) {
      continue;
    }

    float pointX1 = point.x - state.vehicle_x;
    float pointY1 = point.y - state.vehicle_y;
    float pointZ1 = point.z - state.vehicle_z;
    float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);

    if (dis1 <= config.min_dy_obs_dis) {
      state.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(
          ind_x, ind_y)] += config.min_dy_obs_point_num;
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
      state.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(ind_x,
                                                                  ind_y)]++;
    }
  }
}

void TerrainAlgorithm::filterDynamicObstaclePoints(const TerrainConfig& config,
                                                   TerrainState& state,
                                                   int laser_cloud_crop_size) {
  pcl::PointXYZI point;
  for (int i = 0; i < laser_cloud_crop_size; i++) {
    point = state.laser_cloud_crop->points[i];

    int ind_x = static_cast<int>(
                    (point.x - state.vehicle_x + config.planar_voxel_size / 2)
                    / config.planar_voxel_size)
              + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
    int ind_y = static_cast<int>(
                    (point.y - state.vehicle_y + config.planar_voxel_size / 2)
                    / config.planar_voxel_size)
              + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;

    if (point.x - state.vehicle_x + config.planar_voxel_size / 2 < 0) {
      ind_x--;
    }
    if (point.y - state.vehicle_y + config.planar_voxel_size / 2 < 0) {
      ind_y--;
    }

    if (ind_x < 0 || ind_x >= TerrainConfig::PLANAR_VOXEL_WIDTH || ind_y < 0
        || ind_y >= TerrainConfig::PLANAR_VOXEL_WIDTH) {
      continue;
    }

    float pointX1 = point.x - state.vehicle_x;
    float pointY1 = point.y - state.vehicle_y;
    float pointZ1 = point.z - state.vehicle_z;
    float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
    float angle1 = atan2(pointZ1 - config.min_dy_obs_rel_z, dis1) * 180.0
                 / M_PI;
    if (angle1 > config.min_dy_obs_angle) {
      state.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(ind_x,
                                                                  ind_y)] = 0;
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
                                        int terrain_cloud_size) {
  state.terrain_cloud_elev->clear();
  int terrainCloudElevSize = 0;
  pcl::PointXYZI point;

  for (int i = 0; i < terrain_cloud_size; i++) {
    point = state.terrain_cloud->points[i];
    if (!(point.z - state.vehicle_z > config.min_rel_z
          && point.z - state.vehicle_z < config.max_rel_z)) {
      continue;
    }

    int ind_x = static_cast<int>(
                    (point.x - state.vehicle_x + config.planar_voxel_size / 2)
                    / config.planar_voxel_size)
              + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
    int ind_y = static_cast<int>(
                    (point.y - state.vehicle_y + config.planar_voxel_size / 2)
                    / config.planar_voxel_size)
              + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;

    if (point.x - state.vehicle_x + config.planar_voxel_size / 2 < 0) {
      ind_x--;
    }
    if (point.y - state.vehicle_y + config.planar_voxel_size / 2 < 0) {
      ind_y--;
    }

    if (ind_x < 0 || ind_x >= TerrainConfig::PLANAR_VOXEL_WIDTH || ind_y < 0
        || ind_y >= TerrainConfig::PLANAR_VOXEL_WIDTH) {
      continue;
    }

    int idx = TerrainConfig::planar_voxel_index(ind_x, ind_y);
    if (state.planar_voxel_dy_obs[idx] >= config.min_dy_obs_point_num
        && config.clear_dy_obs) {
      continue;
    }

    float disZ = point.z - state.planar_voxel_elev[idx];
    if (config.consider_drop) {
      disZ = fabs(disZ);
    }

    int planarSize = state.planar_point_elev[idx].size();
    if (disZ >= 0 && disZ < config.vehicle_height
        && planarSize >= config.min_block_point_num) {
      state.terrain_cloud_elev->push_back(point);
      state.terrain_cloud_elev->points[terrainCloudElevSize].intensity = disZ;
      terrainCloudElevSize++;
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
      int ind_x = i / TerrainConfig::PLANAR_VOXEL_WIDTH;
      int ind_y = i % TerrainConfig::PLANAR_VOXEL_WIDTH;
      bool edgeVoxel = false;
      for (int dX = -1; dX <= 1 && !edgeVoxel; dX++) {
        for (int dY = -1; dY <= 1 && !edgeVoxel; dY++) {
          if (ind_x + dX >= 0 && ind_x + dX < TerrainConfig::PLANAR_VOXEL_WIDTH
              && ind_y + dY >= 0
              && ind_y + dY < TerrainConfig::PLANAR_VOXEL_WIDTH) {
            int ni = TerrainConfig::planar_voxel_index(ind_x + dX, ind_y) + dY;
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
    int ind_x = i / TerrainConfig::PLANAR_VOXEL_WIDTH;
    int ind_y = i % TerrainConfig::PLANAR_VOXEL_WIDTH;

    point.x = config.planar_voxel_size
                * (ind_x - TerrainConfig::PLANAR_VOXEL_HALF_WIDTH)
            + state.vehicle_x;
    point.y = config.planar_voxel_size
                * (ind_y - TerrainConfig::PLANAR_VOXEL_HALF_WIDTH)
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
