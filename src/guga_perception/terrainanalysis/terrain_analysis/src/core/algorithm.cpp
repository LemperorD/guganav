// Copyright 2024 Hongbiao Zhu
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/context.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

// TerrainAlgorithm static methods

void TerrainAlgorithm::rolloverTerrainVoxels(const TerrainConfig& cfg,
                                             TerrainState& state) {
  float terrain_voxel_cen_x = cfg.terrain_voxel_size
                            * static_cast<float>(state.terrain_voxel_shift_x);
  float terrain_voxel_cen_y = cfg.terrain_voxel_size
                            * static_cast<float>(state.terrain_voxel_shift_y);

  while (state.vehicle_x - terrain_voxel_cen_x < -cfg.terrain_voxel_size) {
    for (int ind_y = 0; ind_y < TerrainConfig::TERRAIN_VOXEL_WIDTH; ind_y++) {
      auto ptr =
          state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(TerrainConfig::TERRAIN_VOXEL_WIDTH - 1, ind_y)];
      for (int ind_x = TerrainConfig::TERRAIN_VOXEL_WIDTH - 1; ind_x >= 1;
           ind_x--) {
        state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(ind_x, ind_y)] =
            state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(ind_x - 1, ind_y)];
      }
      state.terrain_voxel_cloud[ind_y] = ptr;
      state.terrain_voxel_cloud[ind_y]->clear();
    }
    state.terrain_voxel_shift_x--;
    terrain_voxel_cen_x = cfg.terrain_voxel_size
                        * static_cast<float>(state.terrain_voxel_shift_x);
  }

  while (state.vehicle_x - terrain_voxel_cen_x > cfg.terrain_voxel_size) {
    for (int ind_y = 0; ind_y < TerrainConfig::TERRAIN_VOXEL_WIDTH; ind_y++) {
      auto ptr = state.terrain_voxel_cloud[ind_y];
      for (int ind_x = 0; ind_x < TerrainConfig::TERRAIN_VOXEL_WIDTH - 1;
           ind_x++) {
        state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(ind_x, ind_y)] =
            state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(ind_x + 1, ind_y)];
      }
      state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(TerrainConfig::TERRAIN_VOXEL_WIDTH - 1, ind_y)] = ptr;
      state
          .terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(TerrainConfig::TERRAIN_VOXEL_WIDTH - 1, ind_y)]
          ->clear();
    }
    state.terrain_voxel_shift_x++;
    terrain_voxel_cen_x = cfg.terrain_voxel_size
                        * static_cast<float>(state.terrain_voxel_shift_x);
  }

  while (state.vehicle_y - terrain_voxel_cen_y < -cfg.terrain_voxel_size) {
    for (int indX = 0; indX < TerrainConfig::TERRAIN_VOXEL_WIDTH; indX++) {
      auto ptr =
          state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX, TerrainConfig::TERRAIN_VOXEL_WIDTH - 1)];
      for (int indY = TerrainConfig::TERRAIN_VOXEL_WIDTH - 1; indY >= 1;
           indY--) {
        state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX,
                                                                     indY)] =
            state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX, indY - 1)];
      }
      state.terrain_voxel_cloud[TerrainConfig::TERRAIN_VOXEL_WIDTH * indX] =
          ptr;
      state.terrain_voxel_cloud[TerrainConfig::TERRAIN_VOXEL_WIDTH * indX]
          ->clear();
    }
    state.terrain_voxel_shift_y--;
    terrain_voxel_cen_y = cfg.terrain_voxel_size * state.terrain_voxel_shift_y;
  }

  while (state.vehicle_y - terrain_voxel_cen_y > cfg.terrain_voxel_size) {
    for (int indX = 0; indX < TerrainConfig::TERRAIN_VOXEL_WIDTH; indX++) {
      auto ptr =
          state.terrain_voxel_cloud[TerrainConfig::TERRAIN_VOXEL_WIDTH * indX];
      for (int indY = 0; indY < TerrainConfig::TERRAIN_VOXEL_WIDTH - 1;
           indY++) {
        state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX,
                                                                     indY)] =
            state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX, indY + 1)];
      }
      state.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX, TerrainConfig::TERRAIN_VOXEL_WIDTH - 1)] =
          ptr;
      state
          .terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX, TerrainConfig::TERRAIN_VOXEL_WIDTH - 1)]
          ->clear();
    }
    state.terrain_voxel_shift_y++;
    terrain_voxel_cen_y = cfg.terrain_voxel_size * state.terrain_voxel_shift_y;
  }
}

void TerrainAlgorithm::stackLaserScans(const TerrainConfig& cfg,
                                       TerrainState& st) {
  pcl::PointXYZI point;
  int laserCloudCropSize = st.laser_cloud_crop->points.size();
  for (int i = 0; i < laserCloudCropSize; i++) {
    point = st.laser_cloud_crop->points[i];

    int indX = static_cast<int>(
                   (point.x - st.vehicle_x + cfg.terrain_voxel_size / 2)
                   / cfg.terrain_voxel_size)
             + TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH;
    int indY = static_cast<int>(
                   (point.y - st.vehicle_y + cfg.terrain_voxel_size / 2)
                   / cfg.terrain_voxel_size)
             + TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH;

    if (point.x - st.vehicle_x + cfg.terrain_voxel_size / 2 < 0) {
      indX--;
    }
    if (point.y - st.vehicle_y + cfg.terrain_voxel_size / 2 < 0) {
      indY--;
    }

    if (indX >= 0 && indX < TerrainConfig::TERRAIN_VOXEL_WIDTH && indY >= 0
        && indY < TerrainConfig::TERRAIN_VOXEL_WIDTH) {
      st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX, indY)]
          ->push_back(point);
      st.terrain_voxel_update_num[TerrainConfig::terrain_voxel_index(indX,
                                                                     indY)]++;
    }
  }
}

void TerrainAlgorithm::updateVoxels(const TerrainConfig& cfg,
                                    TerrainState& st) {
  pcl::PointXYZI point;
  for (int ind = 0; ind < TerrainConfig::TERRAIN_VOXEL_NUM; ind++) {
    if (st.terrain_voxel_update_num[ind] >= cfg.voxel_point_update_thre
        || st.laser_cloud_time - st.system_init_time
                   - st.terrain_voxel_update_time[ind]
               >= cfg.voxel_time_update_thre
        || st.clearing_cloud) {
      auto voxel_cloud_ptr = st.terrain_voxel_cloud[ind];

      st.laser_cloud_dwz->clear();
      st.down_size_filter.setInputCloud(voxel_cloud_ptr);
      st.down_size_filter.filter(*st.laser_cloud_dwz);

      voxel_cloud_ptr->clear();
      int cloud_size = st.laser_cloud_dwz->points.size();
      for (int i = 0; i < cloud_size; i++) {
        point = st.laser_cloud_dwz->points[i];
        float dis = st.horizontalDistanceTo(point.x, point.y);
        if (point.z - st.vehicle_z > cfg.min_rel_z - cfg.dis_ratio_z * dis
            && point.z - st.vehicle_z < cfg.max_rel_z + cfg.dis_ratio_z * dis
            && (st.laser_cloud_time - st.system_init_time - point.intensity
                    < cfg.decay_time
                || dis < cfg.no_decay_dis)
            && !(dis < st.clearing_dis && st.clearing_cloud)) {
          voxel_cloud_ptr->push_back(point);
        }
      }

      st.terrain_voxel_update_num[ind] = 0;
      st.terrain_voxel_update_time[ind] = st.laser_cloud_time
                                        - st.system_init_time;
    }
  }
}

void TerrainAlgorithm::extractTerrainCloud(const TerrainConfig& cfg,
                                           TerrainState& st) {
  st.terrain_cloud->clear();
  for (int indX = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH - 5;
       indX <= TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + 5; indX++) {
    for (int indY = TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH - 5;
         indY <= TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + 5; indY++) {
      *st.terrain_cloud +=
          *st.terrain_voxel_cloud[TerrainConfig::terrain_voxel_index(indX,
                                                                     indY)];
    }
  }
}

void TerrainAlgorithm::estimateGround(const TerrainConfig& cfg,
                                      TerrainState& st) {
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    st.planar_voxel_elev[i] = 0;
    st.planar_voxel_edge[i] = 0;
    st.planar_voxel_dy_obs[i] = 0;
    st.planar_point_elev[i].clear();
  }

  pcl::PointXYZI point;
  int terrainCloudSize = st.terrain_cloud->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = st.terrain_cloud->points[i];

    int indX = static_cast<int>(
                   (point.x - st.vehicle_x + cfg.planar_voxel_size / 2)
                   / cfg.planar_voxel_size)
             + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
    int indY = static_cast<int>(
                   (point.y - st.vehicle_y + cfg.planar_voxel_size / 2)
                   / cfg.planar_voxel_size)
             + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;

    if (point.x - st.vehicle_x + cfg.planar_voxel_size / 2 < 0) {
      indX--;
    }
    if (point.y - st.vehicle_y + cfg.planar_voxel_size / 2 < 0) {
      indY--;
    }

    if (point.z - st.vehicle_z > cfg.min_rel_z
        && point.z - st.vehicle_z < cfg.max_rel_z) {
      for (int dX = -1; dX <= 1; dX++) {
        for (int dY = -1; dY <= 1; dY++) {
          if (indX + dX >= 0 && indX + dX < TerrainConfig::PLANAR_VOXEL_WIDTH
              && indY + dY >= 0
              && indY + dY < TerrainConfig::PLANAR_VOXEL_WIDTH) {
            st.planar_point_elev[TerrainConfig::planar_voxel_index(indX + dX,
                                                                   indY + dY)]
                .push_back(point.z);
          }
        }
      }
    }
  }
}

void TerrainAlgorithm::detectDynamicObstacles(const TerrainConfig& cfg,
                                              TerrainState& st) {
  pcl::PointXYZI point;
  int terrainCloudSize = st.terrain_cloud->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = st.terrain_cloud->points[i];

    int indX = static_cast<int>(
                   (point.x - st.vehicle_x + cfg.planar_voxel_size / 2)
                   / cfg.planar_voxel_size)
             + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
    int indY = static_cast<int>(
                   (point.y - st.vehicle_y + cfg.planar_voxel_size / 2)
                   / cfg.planar_voxel_size)
             + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;

    if (point.x - st.vehicle_x + cfg.planar_voxel_size / 2 < 0) {
      indX--;
    }
    if (point.y - st.vehicle_y + cfg.planar_voxel_size / 2 < 0) {
      indY--;
    }

    if (indX < 0 || indX >= TerrainConfig::PLANAR_VOXEL_WIDTH || indY < 0
        || indY >= TerrainConfig::PLANAR_VOXEL_WIDTH) {
      continue;
    }

    float pointX1 = point.x - st.vehicle_x;
    float pointY1 = point.y - st.vehicle_y;
    float pointZ1 = point.z - st.vehicle_z;
    float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);

    if (dis1 <= cfg.min_dy_obs_dis) {
      st.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(indX, indY)] +=
          cfg.min_dy_obs_point_num;
      continue;
    }

    float angle1 = atan2(pointZ1 - cfg.min_dy_obs_rel_z, dis1) * 180.0 / M_PI;
    if (angle1 <= cfg.min_dy_obs_angle) {
      continue;
    }

    float pointX2 = pointX1 * st.cos_vehicle_yaw + pointY1 * st.sin_vehicle_yaw;
    float pointY2 = -pointX1 * st.sin_vehicle_yaw
                  + pointY1 * st.cos_vehicle_yaw;
    float pointZ2 = pointZ1;

    float pointX3 = pointX2 * st.cos_vehicle_pitch
                  - pointZ2 * st.sin_vehicle_pitch;
    float pointY3 = pointY2;
    float pointZ3 = pointX2 * st.sin_vehicle_pitch
                  + pointZ2 * st.cos_vehicle_pitch;

    float pointX4 = pointX3;
    float pointY4 = pointY3 * st.cos_vehicle_roll
                  + pointZ3 * st.sin_vehicle_roll;
    float pointZ4 = -pointY3 * st.sin_vehicle_roll
                  + pointZ3 * st.cos_vehicle_roll;

    float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
    float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
    if ((angle4 > cfg.min_dy_obs_vfov && angle4 < cfg.max_dy_obs_vfov)
        || fabs(pointZ4) < cfg.abs_dy_obs_rel_z_thre) {
      st.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(indX, indY)]++;
    }
  }
}

void TerrainAlgorithm::filterDynamicObstaclePoints(const TerrainConfig& cfg,
                                                   TerrainState& st,
                                                   int laser_cloud_crop_size) {
  pcl::PointXYZI point;
  for (int i = 0; i < laser_cloud_crop_size; i++) {
    point = st.laser_cloud_crop->points[i];

    int indX = static_cast<int>(
                   (point.x - st.vehicle_x + cfg.planar_voxel_size / 2)
                   / cfg.planar_voxel_size)
             + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
    int indY = static_cast<int>(
                   (point.y - st.vehicle_y + cfg.planar_voxel_size / 2)
                   / cfg.planar_voxel_size)
             + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;

    if (point.x - st.vehicle_x + cfg.planar_voxel_size / 2 < 0) {
      indX--;
    }
    if (point.y - st.vehicle_y + cfg.planar_voxel_size / 2 < 0) {
      indY--;
    }

    if (indX < 0 || indX >= TerrainConfig::PLANAR_VOXEL_WIDTH || indY < 0
        || indY >= TerrainConfig::PLANAR_VOXEL_WIDTH) {
      continue;
    }

    float pointX1 = point.x - st.vehicle_x;
    float pointY1 = point.y - st.vehicle_y;
    float pointZ1 = point.z - st.vehicle_z;
    float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
    float angle1 = atan2(pointZ1 - cfg.min_dy_obs_rel_z, dis1) * 180.0 / M_PI;
    if (angle1 > cfg.min_dy_obs_angle) {
      st.planar_voxel_dy_obs[TerrainConfig::planar_voxel_index(indX, indY)] = 0;
    }
  }
}

void TerrainAlgorithm::computeElevation(const TerrainConfig& cfg,
                                        TerrainState& st) {
  if (cfg.use_sorting) {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      int size = st.planar_point_elev[i].size();
      if (size == 0) {
        continue;
      }
      sort(st.planar_point_elev[i].begin(), st.planar_point_elev[i].end());

      int quantileID = static_cast<int>(cfg.quantile_z * size);
      if (quantileID < 0) {
        quantileID = 0;
      } else if (quantileID >= size) {
        quantileID = size - 1;
      }

      if (st.planar_point_elev[i][quantileID]
              > st.planar_point_elev[i][0] + cfg.max_ground_lift
          && cfg.limit_ground_lift) {
        st.planar_voxel_elev[i] = st.planar_point_elev[i][0]
                                + cfg.max_ground_lift;
      } else {
        st.planar_voxel_elev[i] = st.planar_point_elev[i][quantileID];
      }
    }
  } else {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      int size = st.planar_point_elev[i].size();
      if (size == 0) {
        continue;
      }
      float minZ = 1000.0;
      int minID = -1;
      for (int j = 0; j < size; j++) {
        if (st.planar_point_elev[i][j] < minZ) {
          minZ = st.planar_point_elev[i][j];
          minID = j;
        }
      }
      if (minID != -1) {
        st.planar_voxel_elev[i] = st.planar_point_elev[i][minID];
      }
    }
  }
}

void TerrainAlgorithm::computeHeightMap(const TerrainConfig& cfg,
                                        TerrainState& st,
                                        int terrain_cloud_size) {
  st.terrain_cloud_elev->clear();
  int terrainCloudElevSize = 0;
  pcl::PointXYZI point;

  for (int i = 0; i < terrain_cloud_size; i++) {
    point = st.terrain_cloud->points[i];
    if (!(point.z - st.vehicle_z > cfg.min_rel_z
          && point.z - st.vehicle_z < cfg.max_rel_z)) {
      continue;
    }

    int indX = static_cast<int>(
                   (point.x - st.vehicle_x + cfg.planar_voxel_size / 2)
                   / cfg.planar_voxel_size)
             + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;
    int indY = static_cast<int>(
                   (point.y - st.vehicle_y + cfg.planar_voxel_size / 2)
                   / cfg.planar_voxel_size)
             + TerrainConfig::PLANAR_VOXEL_HALF_WIDTH;

    if (point.x - st.vehicle_x + cfg.planar_voxel_size / 2 < 0) {
      indX--;
    }
    if (point.y - st.vehicle_y + cfg.planar_voxel_size / 2 < 0) {
      indY--;
    }

    if (indX < 0 || indX >= TerrainConfig::PLANAR_VOXEL_WIDTH || indY < 0
        || indY >= TerrainConfig::PLANAR_VOXEL_WIDTH) {
      continue;
    }

    int idx = TerrainConfig::planar_voxel_index(indX, indY);
    if (st.planar_voxel_dy_obs[idx] >= cfg.min_dy_obs_point_num
        && cfg.clear_dy_obs) {
      continue;
    }

    float disZ = point.z - st.planar_voxel_elev[idx];
    if (cfg.consider_drop) {
      disZ = fabs(disZ);
    }

    int planarSize = st.planar_point_elev[idx].size();
    if (disZ >= 0 && disZ < cfg.vehicle_height
        && planarSize >= cfg.min_block_point_num) {
      st.terrain_cloud_elev->push_back(point);
      st.terrain_cloud_elev->points[terrainCloudElevSize].intensity = disZ;
      terrainCloudElevSize++;
    }
  }
}

void TerrainAlgorithm::addNoDataObstacles(const TerrainConfig& cfg,
                                          TerrainState& st) {
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    if (st.planar_point_elev[i].size()
        < static_cast<size_t>(cfg.min_block_point_num)) {
      st.planar_voxel_edge[i] = 1;
    }
  }

  for (int noDataBlockSkipCount = 0;
       noDataBlockSkipCount < cfg.no_data_block_skip_num;
       noDataBlockSkipCount++) {
    for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
      if (st.planar_voxel_edge[i] < 1) {
        continue;
      }
      int indX = i / TerrainConfig::PLANAR_VOXEL_WIDTH;
      int indY = i % TerrainConfig::PLANAR_VOXEL_WIDTH;
      bool edgeVoxel = false;
      for (int dX = -1; dX <= 1 && !edgeVoxel; dX++) {
        for (int dY = -1; dY <= 1 && !edgeVoxel; dY++) {
          if (indX + dX >= 0 && indX + dX < TerrainConfig::PLANAR_VOXEL_WIDTH
              && indY + dY >= 0
              && indY + dY < TerrainConfig::PLANAR_VOXEL_WIDTH) {
            int ni = TerrainConfig::planar_voxel_index(indX + dX, indY) + dY;
            if (st.planar_voxel_edge[ni] < st.planar_voxel_edge[i]) {
              edgeVoxel = true;
            }
          }
        }
      }
      if (!edgeVoxel) {
        st.planar_voxel_edge[i]++;
      }
    }
  }

  pcl::PointXYZI point;
  for (int i = 0; i < TerrainConfig::PLANAR_VOXEL_NUM; i++) {
    if (st.planar_voxel_edge[i] <= cfg.no_data_block_skip_num) {
      continue;
    }
    int indX = i / TerrainConfig::PLANAR_VOXEL_WIDTH;
    int indY = i % TerrainConfig::PLANAR_VOXEL_WIDTH;

    point.x = cfg.planar_voxel_size
                * (indX - TerrainConfig::PLANAR_VOXEL_HALF_WIDTH)
            + st.vehicle_x;
    point.y = cfg.planar_voxel_size
                * (indY - TerrainConfig::PLANAR_VOXEL_HALF_WIDTH)
            + st.vehicle_y;
    point.z = st.vehicle_z;
    point.intensity = cfg.vehicle_height;

    point.x -= cfg.planar_voxel_size / 4.0;
    point.y -= cfg.planar_voxel_size / 4.0;
    st.terrain_cloud_elev->push_back(point);

    point.x += cfg.planar_voxel_size / 2.0;
    st.terrain_cloud_elev->push_back(point);

    point.y += cfg.planar_voxel_size / 2.0;
    st.terrain_cloud_elev->push_back(point);

    point.x -= cfg.planar_voxel_size / 2.0;
    st.terrain_cloud_elev->push_back(point);
  }
}

void TerrainAlgorithm::run(const TerrainConfig& cfg, TerrainState& st) {
  st.new_laser_cloud = false;

  rolloverTerrainVoxels(cfg, st);
  stackLaserScans(cfg, st);
  updateVoxels(cfg, st);
  extractTerrainCloud(cfg, st);

  estimateGround(cfg, st);

  if (cfg.clear_dy_obs) {
    detectDynamicObstacles(cfg, st);
    filterDynamicObstaclePoints(cfg, st, st.laser_cloud_crop->points.size());
  }

  computeElevation(cfg, st);
  computeHeightMap(cfg, st, st.terrain_cloud->points.size());

  if (cfg.no_data_obstacle
      && st.no_data_inited == TerrainState::NoDataState::ACTIVE) {
    addNoDataObstacles(cfg, st);
  }

  st.clearing_cloud = false;
}
