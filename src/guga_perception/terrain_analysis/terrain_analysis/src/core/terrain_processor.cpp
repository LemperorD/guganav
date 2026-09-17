// Copyright 2024 Hongbiao Zhu
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.
//
// 本文件实现 TerrainProcessor：自持 TerrainConfig/TerrainState，对外仅暴露
// ingest* / run / terrainCloudElev；管线各阶段为私有成员，可自由重构。

#include "terrain_analysis/core/terrain_processor.hpp"
#include "terrain_analysis/core/config.hpp"
#include "terrain_analysis/core/grid_lookup.hpp"
#include "terrain_analysis/core/state.hpp"
#include "terrain_analysis/core/terrain_voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace terrain_analysis {

  void TerrainProcessor::ingestOdometry(double x, double y, double z,
                                        double roll, double pitch, double yaw) {
    state_.lidar.x = x;
    state_.lidar.y = y;
    state_.lidar.z = z;

    state_.sin_lidar_roll = sin(roll);
    state_.cos_lidar_roll = cos(roll);
    state_.sin_lidar_pitch = sin(pitch);
    state_.cos_lidar_pitch = cos(pitch);
    state_.sin_lidar_yaw = sin(yaw);
    state_.cos_lidar_yaw = cos(yaw);
  }

  void TerrainProcessor::ingestLaserCloud(
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
      double timestamp_sec) {
    state_.laser_cloud_time = timestamp_sec;
    if (!state_.system_inited) {
      state_.system_init_time = state_.laser_cloud_time;
      state_.system_inited = true;
    }

    const double lidar_z = state_.lidar.z;
    const double max_range = config_.terrain_voxel_size
                             * (TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH + 1);
    state_.laser_cloud_crop->clear();
    for (const auto& point : cloud->points) {
      double relative_z = point.z - lidar_z;
      double distance = horizontalDistanceTo(point.x, point.y);
      const double z_margin = config_.distance_ratio_z * distance;
      if (relative_z > config_.min_relative_z - z_margin
          && relative_z < config_.max_relative_z + z_margin
          && distance < max_range) {
        pcl::PointXYZI cropped = point;
        cropped.intensity = static_cast<float>(state_.laser_cloud_time
                                               - state_.system_init_time);
        state_.laser_cloud_crop->push_back(cropped);
      }
    }

    state_.new_laser_cloud = true;
  }

  void TerrainProcessor::runStages() {
    state_.new_laser_cloud = false;

    // 分组与数据流向：
    //   C 平面高程   F3,F4 <- F2(已采集)
    //   E 输出       F6    <- F2,F3,F4
    // A 组（体素地图维护）与 B 组（采集）由调用方先做，不在这里。
    // 组间必需依赖：B->{C,E}、C->E。各数组的清空由属主负责（见各阶段开头）。
    estimateTerrainGround();
    computePlanarElevation();
    computeHeightMap();
  }

  void TerrainProcessor::estimateTerrainGround() {
    // 只清本阶段拥有的地面候选 F3；F4 由 computePlanarElevation 清，
    // F5 由 detectDynamicObstacles 清——每份数据只有一个属主。
    for (auto& point_elevations : state_.planar_point_elev) {
      point_elevations.clear();
    }

    for (const auto& point : state_.terrain_cloud->points) {
      // 唯一的候选筛选是下界，且用**绝对 z**（odom）：地面在 odom 中大体水平，
      // 地板过滤只需挡住远低于地面的穿透点，用绝对量比"相对雷达"更贴合语义，
      // 也不随雷达上下抖动而移动。
      if (point.z <= config_.ground_floor_z) {
        continue;
      }
      // 这里曾有一条 ceiling_clearance 上界（"净空"）。已移除：
      // 净空是**障碍输出**的判据（车辆能否从下方通过），与"哪些点属于地面"
      // 无关；放在本阶段只会按车高砍掉抬升的地面（坡面），并使候选数随车高
      // 漂移、经分位数放大成 elev 偏差。地面候选的上界改由地面自身决定——
      // 高于地面的部分本就是障碍，会由 computeHeightMap 按净空处理。
      const GridIndex grid_index = gridIndex(
          point.x, point.y, state_.lidar.x, state_.lidar.y,
          config_.planar_voxel_size, TerrainGrid::PLANAR_VOXEL_WIDTH);
      if (!grid_index.valid) {
        continue;
      }

      addToPlanarNeighborhood3x3(grid_index.row, grid_index.col, point.z);
    }
  }

  void TerrainProcessor::detectDynamicObstacles() {
    // 本阶段（与 filter 一起）拥有 F5，逐帧重建。
    state_.planar_voxel_dy_obs.fill(0);

    const double lidar_x = state_.lidar.x;
    const double lidar_y = state_.lidar.y;
    const double lidar_z = state_.lidar.z;

    for (const auto& point : state_.terrain_cloud->points) {
      const GridIndex grid_index = gridIndex(
          point.x, point.y, state_.lidar.x, state_.lidar.y,
          config_.planar_voxel_size, TerrainGrid::PLANAR_VOXEL_WIDTH);
      if (!grid_index.valid) {
        continue;
      }
      size_t cell = TerrainGrid::planarVoxelIndex(grid_index.row,
                                                  grid_index.col);

      double relative_x = point.x - lidar_x;
      double relative_y = point.y - lidar_y;
      double relative_z = point.z - lidar_z;
      double distance = sqrt((relative_x * relative_x)
                             + (relative_y * relative_y));

      if (distance <= config_.min_dy_obs_distance) {
        state_.planar_voxel_dy_obs[cell] += config_.min_dy_obs_point_num;
        continue;
      }

      double scan_angle = atan2(relative_z - config_.min_dy_obs_relative_z,
                                distance);
      if (scan_angle <= config_.min_dy_obs_angle) {
        continue;
      }

      auto sensor = transformToSensorFrame(relative_x, relative_y, relative_z);
      double sensor_distance = sqrt((sensor.x * sensor.x)
                                    + (sensor.y * sensor.y));
      double sensor_angle = atan2(sensor.z, sensor_distance);
      if ((sensor_angle > config_.min_dy_obs_vfov
           && sensor_angle < config_.max_dy_obs_vfov)
          || std::abs(sensor.z) < config_.abs_dy_obs_relative_z_threshold) {
        state_.planar_voxel_dy_obs[cell]++;
      }
    }
  }

  void TerrainProcessor::filterDynamicObstaclePoints() {
    const double lidar_x = state_.lidar.x;
    const double lidar_y = state_.lidar.y;
    const double lidar_z = state_.lidar.z;

    for (const auto& point : state_.laser_cloud_crop->points) {
      const GridIndex grid_index = gridIndex(
          point.x, point.y, state_.lidar.x, state_.lidar.y,
          config_.planar_voxel_size, TerrainGrid::PLANAR_VOXEL_WIDTH);
      if (!grid_index.valid) {
        continue;
      }
      size_t cell = TerrainGrid::planarVoxelIndex(grid_index.row,
                                                  grid_index.col);

      double relative_x = point.x - lidar_x;
      double relative_y = point.y - lidar_y;
      double relative_z = point.z - lidar_z;
      double distance = sqrt((relative_x * relative_x)
                             + (relative_y * relative_y));
      double scan_angle = atan2(relative_z - config_.min_dy_obs_relative_z,
                                distance);
      if (scan_angle > config_.min_dy_obs_angle) {
        state_.planar_voxel_dy_obs[cell] = 0;
      }
    }
  }

  void TerrainProcessor::computePlanarElevation() {
    // 本阶段拥有 F4：没有候选的格保持 0（见 computeHeightMap 的说明）。
    state_.planar_voxel_elev.fill(0);

    if (config_.use_sorting) {
      for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
        elevateByQuantile(i);
      }
    } else {
      for (int i = 0; i < TerrainGrid::PLANAR_VOXEL_NUM; i++) {
        elevateByMinimum(i);
      }
    }
  }

  void TerrainProcessor::computeHeightMap() {
    const double lidar_z = state_.lidar.z;
    auto& elevations = state_.terrain_cloud_elev;
    elevations->clear();

    for (const auto& point : state_.terrain_cloud->points) {
      const GridIndex grid_index = gridIndex(
          point.x, point.y, state_.lidar.x, state_.lidar.y,
          config_.planar_voxel_size, TerrainGrid::PLANAR_VOXEL_WIDTH);
      if (!grid_index.valid) {
        continue;
      }
      const size_t cell = TerrainGrid::planarVoxelIndex(grid_index.row,
                                                        grid_index.col);
      // 该点所在处的地面高度（本帧估计值），下面所有高度判据都以它为基准。
      const double ground_z = state_.planar_voxel_elev[cell];
      const double height_above_ground = point.z - ground_z;

      // 下界：地板过滤（挡掉地面以下/穿透点）。此处用**相对雷达**的高度，
      // 因为要挡的是"远低于雷达"的穿透点，与地形无关。
      if (point.z - lidar_z <= config_.min_relative_z) {
        continue;
      }
      // 上界：距地面达到安全间隙的点（天花板/横梁）不输出为障碍——
      // 净空足够时车辆可从下方通过（隧道场景）。以**局部地面**为基准：
      // 净空是"地面到障碍下沿"的距离，这也使判据在坡面上保持一致。
      // 这是障碍输出**唯一**的上界：不再叠加按车高的截断，否则车高与净空
      // 之间的那一带（车高 0.52 → 净空 0.62 之间）会被漏检，而车过不去。
      if (height_above_ground >= config_.ceiling_clearance) {
        continue;
      }
      if (state_.planar_voxel_dy_obs[cell] >= config_.min_dy_obs_point_num) {
        continue;
      }

      double height = height_above_ground;
      if (config_.consider_drop) {
        height = std::abs(height);
      }

      auto point_count = state_.planar_point_elev[cell].size();
      // 下界：地面带的死区，吸收地面高度估计的误差。估计值偏低时，真实地面点会
      // 算出几厘米的正高度；若从 0 起算，它们会被当作低矮障碍标记出去。
      if (height >= config_.min_obstacle_height
          && point_count >= static_cast<size_t>(config_.min_block_point_num)) {
        elevations->push_back(point);
        elevations->back().intensity = static_cast<float>(height);
      }
    }
  }

  double TerrainProcessor::horizontalDistanceTo(double px, double py) const {
    return horizontalDistance(px, py, state_.lidar.x, state_.lidar.y);
  }

  // point_time 为该点的观测时刻（相对首帧的秒数）。对同一叶的代表点而言，
  // 它是叶内最新的观测时刻，见 updateTerrainVoxels。
  TerrainProcessor::SensorPoint TerrainProcessor::transformToSensorFrame(
      double x, double y, double z) const {
    double rotated_x = (x * state_.cos_lidar_yaw) + (y * state_.sin_lidar_yaw);
    double rotated_y = -(x * state_.sin_lidar_yaw) + (y * state_.cos_lidar_yaw);

    double pitched_x = (rotated_x * state_.cos_lidar_pitch)
                       - (z * state_.sin_lidar_pitch);
    double pitched_z = (rotated_x * state_.sin_lidar_pitch)
                       + (z * state_.cos_lidar_pitch);

    double rolled_y = (rotated_y * state_.cos_lidar_roll)
                      + (pitched_z * state_.sin_lidar_roll);
    double rolled_z = -(rotated_y * state_.sin_lidar_roll)
                      + (pitched_z * state_.cos_lidar_roll);

    return {pitched_x, rolled_y, rolled_z};
  }

  void TerrainProcessor::addToPlanarNeighborhood3x3(int row, int col,
                                                    double z) {
    constexpr int width = TerrainGrid::PLANAR_VOXEL_WIDTH;

    for (int delta_row = -1; delta_row <= 1; delta_row++) {
      const int neighbor_row = row + delta_row;
      if (neighbor_row < 0 || neighbor_row >= width) {
        continue;
      }
      for (int delta_col = -1; delta_col <= 1; delta_col++) {
        const int neighbor_col = col + delta_col;
        if (neighbor_col < 0 || neighbor_col >= width) {
          continue;
        }
        // 行偏移按整行换算（乘网格宽度），列偏移直接相加
        const size_t index = TerrainGrid::planarVoxelIndex(neighbor_row,
                                                           neighbor_col);
        state_.planar_point_elev[index].push_back(z);
      }
    }
  }

  void TerrainProcessor::elevateByQuantile(int cell) {
    auto& elevations = state_.planar_point_elev[cell];
    int point_count = static_cast<int>(elevations.size());
    if (point_count == 0) {
      return;
    }
    sort(elevations.begin(), elevations.end());

    int quantile_index = static_cast<int>(config_.quantile_z * point_count);
    if (quantile_index >= point_count) {
      quantile_index = point_count - 1;
    }
    double minimum_z = elevations[0];
    double quantile_z = elevations[quantile_index];
    state_.planar_voxel_elev[cell] =
        config_.limit_ground_lift
            ? std::min(quantile_z, minimum_z + config_.max_ground_lift)
            : quantile_z;
  }

  void TerrainProcessor::elevateByMinimum(int cell) {
    auto& elevations = state_.planar_point_elev[cell];
    if (elevations.empty()) {
      return;
    }
    state_.planar_voxel_elev[cell] = *std::min_element(elevations.begin(),
                                                       elevations.end());
  }

}  // namespace terrain_analysis
