// Copyright 2024 Hongbiao Zhu
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.
//
// 本文件实现 TerrainProcessor：自持 TerrainConfig/TerrainState，对外仅暴露
// ingest* / run / terrainCloudElev；管线各阶段为私有成员，可自由重构。

#include "terrain_analysis/core/terrain_processor.hpp"
#include "terrain_analysis/core/config.hpp"
#include "terrain_analysis/core/state.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace terrain_analysis {

  void TerrainProcessor::ingestOdometry(double x, double y, double z,
                                        double roll, double pitch, double yaw) {
    state_.vehicle_x = x;
    state_.vehicle_y = y;
    state_.vehicle_z = z;

    state_.sin_vehicle_roll = sin(roll);
    state_.cos_vehicle_roll = cos(roll);
    state_.sin_vehicle_pitch = sin(pitch);
    state_.cos_vehicle_pitch = cos(pitch);
    state_.sin_vehicle_yaw = sin(yaw);
    state_.cos_vehicle_yaw = cos(yaw);
  }

  void TerrainProcessor::ingestLaserCloud(
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
      double timestamp_sec) {
    state_.laser_cloud_time = timestamp_sec;
    if (!state_.system_inited) {
      state_.system_init_time = state_.laser_cloud_time;
      state_.system_inited = true;
    }

    const double vehicle_z = state_.vehicle_z;
    const double max_range = config_.terrain_voxel_size
                             * (TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH + 1);
    state_.laser_cloud_crop->clear();
    for (const auto& point : cloud->points) {
      double relative_z = point.z - vehicle_z;
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

  void TerrainProcessor::run() {
    state_.new_laser_cloud = false;

    rolloverTerrainVoxels();
    voxelizeTerrain();
    updateTerrainVoxels();
    collectTerrainCloud();

    estimateTerrainGround();

    detectDynamicObstacles();
    filterDynamicObstaclePoints();

    computePlanarElevation();
    computeHeightMap();
  }

  void TerrainProcessor::rolloverTerrainVoxels() {
    const double terrain_voxel_size = config_.terrain_voxel_size;
    double center_x = terrain_voxel_size * state_.terrain_voxel_shift_x;
    double center_y = terrain_voxel_size * state_.terrain_voxel_shift_y;

    while (state_.vehicle_x - center_x < -terrain_voxel_size) {
      shiftGrid(Axis::AXIS_X, ShiftDirection::TOWARD_NEGATIVE);
      center_x = terrain_voxel_size * --state_.terrain_voxel_shift_x;
    }

    while (state_.vehicle_x - center_x > terrain_voxel_size) {
      shiftGrid(Axis::AXIS_X, ShiftDirection::TOWARD_POSITIVE);
      center_x = terrain_voxel_size * ++state_.terrain_voxel_shift_x;
    }

    while (state_.vehicle_y - center_y < -terrain_voxel_size) {
      shiftGrid(Axis::AXIS_Y, ShiftDirection::TOWARD_NEGATIVE);
      center_y = terrain_voxel_size * --state_.terrain_voxel_shift_y;
    }

    while (state_.vehicle_y - center_y > terrain_voxel_size) {
      shiftGrid(Axis::AXIS_Y, ShiftDirection::TOWARD_POSITIVE);
      center_y = terrain_voxel_size * ++state_.terrain_voxel_shift_y;
    }
  }

  void TerrainProcessor::voxelizeTerrain() {
    for (const auto& point : state_.laser_cloud_crop->points) {
      const GridIndex grid_index = voxelIndexOf(VoxelGrid::TERRAIN, point.x,
                                                point.y);
      if (!grid_index.valid) {
        continue;
      }
      size_t cell = TerrainGrid::terrainVoxelIndex(grid_index.row,
                                                   grid_index.col);
      state_.terrain_voxel_cloud[cell]->push_back(point);
      state_.terrain_voxel_update_num[cell]++;
    }
  }

  void TerrainProcessor::updateTerrainVoxels() {
    const double laser_time = state_.laser_cloud_time;
    const double init_time = state_.system_init_time;
    const double vehicle_z = state_.vehicle_z;

    // 降采样器仅服务本次重建：不放进共享状态，避免隐式共享与不可重入
    pcl::VoxelGrid<pcl::PointXYZI> down_size_filter;
    const auto leaf = static_cast<float>(config_.scan_voxel_size);
    down_size_filter.setLeafSize(leaf, leaf, leaf);
    pcl::PointCloud<pcl::PointXYZI> downsampled;

    for (int cell = 0; cell < TerrainGrid::TERRAIN_VOXEL_NUM; cell++) {
      if (!shouldPruneTerrainVoxel(cell)) {
        continue;
      }
      auto& cell_cloud = *state_.terrain_voxel_cloud[cell];

      downsampled.clear();
      down_size_filter.setInputCloud(state_.terrain_voxel_cloud[cell]);
      down_size_filter.filter(downsampled);
      cell_cloud.clear();

      for (const auto& point : downsampled.points) {
        double distance = horizontalDistanceTo(point.x, point.y);
        if (keepTerrainVoxelPoint(point.z - vehicle_z, distance,
                                  point.intensity)) {
          cell_cloud.push_back(point);
        }
      }

      state_.terrain_voxel_update_num[cell] = 0;
      state_.terrain_voxel_update_time[cell] = laser_time - init_time;
    }
  }

  void TerrainProcessor::collectTerrainCloud() {
    static constexpr int EXTRACT_HALF_WINDOW = 5;
    state_.terrain_cloud->clear();
    for (int row = TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH - EXTRACT_HALF_WINDOW;
         row <= TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH + EXTRACT_HALF_WINDOW;
         row++) {
      for (int column =
               TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH - EXTRACT_HALF_WINDOW;
           column
           <= TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH + EXTRACT_HALF_WINDOW;
           column++) {
        *state_.terrain_cloud +=
            *state_.terrain_voxel_cloud[TerrainGrid::terrainVoxelIndex(row,
                                                                       column)];
      }
    }
  }

  void TerrainProcessor::estimateTerrainGround() {
    resetPlanarVoxels();

    for (const auto& point : state_.terrain_cloud->points) {
      // 唯一的候选筛选是下界，且用**绝对 z**（odom）：地面在 odom 中大体水平，
      // 地板过滤只需挡住远低于地面的穿透点，用绝对量比"相对车辆"更贴合语义，
      // 也不随车体上下抖动而移动。
      if (point.z <= config_.ground_floor_z) {
        continue;
      }
      // 这里曾有一条 ceiling_clearance 上界（"净空"）。已移除：
      // 净空是**障碍输出**的判据（车辆能否从下方通过），与"哪些点属于地面"
      // 无关；放在本阶段只会按车高砍掉抬升的地面（坡面），并使候选数随车高
      // 漂移、经分位数放大成 elev 偏差。地面候选的上界改由地面自身决定——
      // 高于地面的部分本就是障碍，会由 computeHeightMap 按净空处理。
      const GridIndex grid_index = voxelIndexOf(VoxelGrid::PLANAR, point.x,
                                                point.y);
      if (!grid_index.valid) {
        continue;
      }

      addToPlanarNeighborhood3x3(grid_index.row, grid_index.col, point.z);
    }
  }

  void TerrainProcessor::detectDynamicObstacles() {
    const double vehicle_x = state_.vehicle_x;
    const double vehicle_y = state_.vehicle_y;
    const double vehicle_z = state_.vehicle_z;

    for (const auto& point : state_.terrain_cloud->points) {
      const GridIndex grid_index = voxelIndexOf(VoxelGrid::PLANAR, point.x,
                                                point.y);
      if (!grid_index.valid) {
        continue;
      }
      size_t cell = TerrainGrid::planarVoxelIndex(grid_index.row,
                                                  grid_index.col);

      double relative_x = point.x - vehicle_x;
      double relative_y = point.y - vehicle_y;
      double relative_z = point.z - vehicle_z;
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
    const double vehicle_x = state_.vehicle_x;
    const double vehicle_y = state_.vehicle_y;
    const double vehicle_z = state_.vehicle_z;

    for (const auto& point : state_.laser_cloud_crop->points) {
      const GridIndex grid_index = voxelIndexOf(VoxelGrid::PLANAR, point.x,
                                                point.y);
      if (!grid_index.valid) {
        continue;
      }
      size_t cell = TerrainGrid::planarVoxelIndex(grid_index.row,
                                                  grid_index.col);

      double relative_x = point.x - vehicle_x;
      double relative_y = point.y - vehicle_y;
      double relative_z = point.z - vehicle_z;
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
    const double vehicle_z = state_.vehicle_z;
    auto& elevations = state_.terrain_cloud_elev;
    elevations->clear();

    for (const auto& point : state_.terrain_cloud->points) {
      const GridIndex grid_index = voxelIndexOf(VoxelGrid::PLANAR, point.x,
                                                point.y);
      if (!grid_index.valid) {
        continue;
      }
      const size_t cell = TerrainGrid::planarVoxelIndex(grid_index.row,
                                                        grid_index.col);
      // 该点所在处的地面高度（本帧估计值），下面所有高度判据都以它为基准。
      const double ground_z = state_.planar_voxel_elev[cell];
      const double height_above_ground = point.z - ground_z;

      // 下界：地板过滤（挡掉地面以下/穿透点）。此处用**相对车辆**的高度，
      // 因为要挡的是"远低于车"的穿透点，与地形无关。
      if (point.z - vehicle_z <= config_.min_relative_z) {
        continue;
      }
      // 上界：距地面达到安全间隙的点（天花板/横梁）不输出为障碍——
      // 净空足够时车辆可从下方通过（隧道场景）。以**局部地面**为基准：
      // 净空是"地面到障碍下沿"的距离，这也使判据在坡面上保持一致。
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
      if (height >= 0 && height < config_.vehicle_height
          && point_count >= static_cast<size_t>(config_.min_block_point_num)) {
        elevations->push_back(point);
        elevations->back().intensity = static_cast<float>(height);
      }
    }
  }

  double TerrainProcessor::horizontalDistanceTo(double px, double py) const {
    return sqrt(((px - state_.vehicle_x) * (px - state_.vehicle_x))
                + ((py - state_.vehicle_y) * (py - state_.vehicle_y)));
  }

  bool TerrainProcessor::shouldPruneTerrainVoxel(int cell) const {
    if (state_.terrain_voxel_update_num[cell]
        >= config_.voxel_point_update_thre) {
      return true;
    }
    double elapsed = state_.laser_cloud_time - state_.system_init_time
                     - state_.terrain_voxel_update_time[cell];
    return elapsed >= config_.voxel_time_update_thre;
  }

  bool TerrainProcessor::keepTerrainVoxelPoint(double relative_z,
                                               double distance,
                                               double point_time) const {
    const double z_margin = config_.distance_ratio_z * distance;
    if (relative_z <= config_.min_relative_z - z_margin) {
      return false;
    }
    if (relative_z >= config_.max_relative_z + z_margin) {
      return false;
    }
    bool near = distance < config_.no_decay_distance;
    bool decayed = (state_.laser_cloud_time - state_.system_init_time
                    - point_time)
                   >= config_.decay_time;
    if (decayed && !near) {
      return false;
    }
    return true;
  }

  TerrainProcessor::SensorPoint TerrainProcessor::transformToSensorFrame(
      double x, double y, double z) const {
    double rotated_x = (x * state_.cos_vehicle_yaw)
                       + (y * state_.sin_vehicle_yaw);
    double rotated_y = -(x * state_.sin_vehicle_yaw)
                       + (y * state_.cos_vehicle_yaw);

    double pitched_x = (rotated_x * state_.cos_vehicle_pitch)
                       - (z * state_.sin_vehicle_pitch);
    double pitched_z = (rotated_x * state_.sin_vehicle_pitch)
                       + (z * state_.cos_vehicle_pitch);

    double rolled_y = (rotated_y * state_.cos_vehicle_roll)
                      + (pitched_z * state_.sin_vehicle_roll);
    double rolled_z = -(rotated_y * state_.sin_vehicle_roll)
                      + (pitched_z * state_.cos_vehicle_roll);

    return {pitched_x, rolled_y, rolled_z};
  }

  void TerrainProcessor::shiftGrid(Axis axis, ShiftDirection direction) {
    static constexpr int WIDTH = TerrainGrid::TERRAIN_VOXEL_WIDTH;
    const bool toward_positive = direction == ShiftDirection::TOWARD_POSITIVE;
    const int src = toward_positive ? 0 : WIDTH - 1;
    const int dst = toward_positive ? WIDTH - 1 : 0;
    const int step = toward_positive ? 1 : -1;

    for (int fixed = 0; fixed < WIDTH; fixed++) {
      auto cell = [&](int m) {
        return axis == Axis::AXIS_X ? TerrainGrid::terrainVoxelIndex(m, fixed)
                                    : TerrainGrid::terrainVoxelIndex(fixed, m);
      };
      auto ptr = state_.terrain_voxel_cloud[cell(src)];
      for (int m = src; m != dst; m += step) {
        state_.terrain_voxel_cloud[cell(m)] =
            state_.terrain_voxel_cloud[cell(m + step)];
      }
      auto& dst_cell = state_.terrain_voxel_cloud[cell(dst)];
      dst_cell = ptr;
      dst_cell->clear();
    }
  }

  void TerrainProcessor::resetPlanarVoxels() {
    state_.planar_voxel_elev.fill(0);
    state_.planar_voxel_dy_obs.fill(0);
    for (auto& point_elevations : state_.planar_point_elev) {
      point_elevations.clear();
    }
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

  TerrainProcessor::GridIndex TerrainProcessor::voxelIndexOf(VoxelGrid grid,
                                                             double x,
                                                             double y) const {
    // 一维坐标 → 网格下标：半格偏移使格心对齐整数下标
    const auto axis_index = [](double coordinate, double vehicle_coordinate,
                               double voxel_size, int half_width) {
      const double half_voxel_size = voxel_size / 2;
      return static_cast<int>(
                 std::floor((coordinate - vehicle_coordinate + half_voxel_size)
                            / voxel_size))
             + half_width;
    };

    const bool terrain = grid == VoxelGrid::TERRAIN;
    const double voxel_size = terrain ? config_.terrain_voxel_size
                                      : config_.planar_voxel_size;
    const int width = terrain ? TerrainGrid::TERRAIN_VOXEL_WIDTH
                              : TerrainGrid::PLANAR_VOXEL_WIDTH;
    const int half_width = (width - 1) / 2;

    GridIndex out;
    out.row = axis_index(y, state_.vehicle_y, voxel_size, half_width);
    out.col = axis_index(x, state_.vehicle_x, voxel_size, half_width);
    out.valid = out.row >= 0 && out.row < width && out.col >= 0
                && out.col < width;
    return out;
  }

}  // namespace terrain_analysis
