// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/core/context.hpp"

#include <cmath>

TerrainAnalysisContext::TerrainAnalysisContext() {
  state.laser_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  state.laser_cloud_crop = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  state.laser_cloud_downsampled =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  state.terrain_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  state.terrain_cloud_elev =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  for (auto& ptr : state.terrain_voxel_cloud) {
    ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }
}

TerrainAnalysisContext::~TerrainAnalysisContext() = default;

void TerrainAnalysisContext::onOdometry(double x, double y, double z,
                                        double roll, double pitch, double yaw) {
  state.vehicle_x = x;
  state.vehicle_y = y;
  state.vehicle_z = z;

  state.sin_vehicle_roll = sin(roll);
  state.cos_vehicle_roll = cos(roll);
  state.sin_vehicle_pitch = sin(pitch);
  state.cos_vehicle_pitch = cos(pitch);
  state.sin_vehicle_yaw = sin(yaw);
  state.cos_vehicle_yaw = cos(yaw);

  if (state.no_data_inited == TerrainState::NoDataState::UNINITIALIZED) {
    state.vehicle_x_initial = state.vehicle_x;
    state.vehicle_y_initial = state.vehicle_y;
    state.no_data_inited = TerrainState::NoDataState::RECORDING;
  }
  if (state.no_data_inited == TerrainState::NoDataState::RECORDING) {
    double distance = state.horizontalDistanceTo(state.vehicle_x_initial,
                                                  state.vehicle_y_initial);
    if (distance >= cfg.no_decay_distance) {
      state.no_data_inited = TerrainState::NoDataState::ACTIVE;
    }
  }
}

void TerrainAnalysisContext::onLaserCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double timestamp_sec) {
  state.laser_cloud_time = timestamp_sec;
  if (!state.system_inited) {
    state.system_init_time = state.laser_cloud_time;
    state.system_inited = true;
  }

  state.laser_cloud->clear();
  *state.laser_cloud = *cloud;

  const double vehicle_z = state.vehicle_z;
  const double max_range = cfg.terrain_voxel_size
                         * (TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + 1);
  state.laser_cloud_crop->clear();
  for (const auto& point : state.laser_cloud->points) {
    double relative_z = point.z - vehicle_z;
    double distance = state.horizontalDistanceTo(point.x, point.y);
    const double z_margin = cfg.distance_ratio_z * distance;
    if (relative_z > cfg.min_relative_z - z_margin
        && relative_z < cfg.max_relative_z + z_margin
        && distance < max_range) {
      pcl::PointXYZI cropped = point;
      cropped.intensity = static_cast<float>(state.laser_cloud_time
                                           - state.system_init_time);
      state.laser_cloud_crop->push_back(cropped);
    }
  }

  state.new_laser_cloud = true;
}

void TerrainAnalysisContext::onJoystick(bool button5) {
  if (button5) {
    state.no_data_inited = TerrainState::NoDataState::UNINITIALIZED;
    state.clearing_cloud = true;
  }
}

void TerrainAnalysisContext::onClearing(double distance_clearing) {
  state.no_data_inited = TerrainState::NoDataState::UNINITIALIZED;
  state.clearing_distance = distance_clearing;
  state.clearing_cloud = true;
}
