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

  using NoDataState = TerrainState::NoDataState;
  if (state.no_data_inited == NoDataState::UNINITIALIZED) {
    state.vehicle_x_rec = state.vehicle_x;
    state.vehicle_y_rec = state.vehicle_y;
    state.no_data_inited = NoDataState::RECORDING;
  }
  if (state.no_data_inited == NoDataState::RECORDING) {
    double dis = state.horizontalDistanceTo(state.vehicle_x_rec,
                                            state.vehicle_y_rec);
    if (dis >= cfg.no_decay_dis) {
      state.no_data_inited = NoDataState::ACTIVE;
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

  pcl::PointXYZI point;
  state.laser_cloud_crop->clear();
  int sz = static_cast<int>(state.laser_cloud->points.size());
  for (int i = 0; i < sz; i++) {
    point = state.laser_cloud->points[i];

    float px = point.x;
    float py = point.y;
    float pz = point.z;

    double dis = state.horizontalDistanceTo(px, py);
    if (pz - state.vehicle_z > cfg.min_rel_z - (cfg.dis_ratio_z * dis)
        && pz - state.vehicle_z < cfg.max_rel_z + (cfg.dis_ratio_z * dis)
        && dis < cfg.terrain_voxel_size
                     * (TerrainConfig::TERRAIN_VOXEL_HALF_WIDTH + 1)) {
      point.x = px;
      point.y = py;
      point.z = pz;
      point.intensity = static_cast<float>(state.laser_cloud_time
                                           - state.system_init_time);
      state.laser_cloud_crop->push_back(point);
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

void TerrainAnalysisContext::onClearing(float dis) {
  state.no_data_inited = TerrainState::NoDataState::UNINITIALIZED;
  state.clearing_dis = dis;
  state.clearing_cloud = true;
}
