// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/core/context.hpp"

#include <cmath>

TerrainAnalysisContext::TerrainAnalysisContext() {
  laser_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_crop_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_dwz_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_elev_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  for (auto& cloud_ptr : terrain_voxel_cloud_) {
    cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }
}

TerrainAnalysisContext::~TerrainAnalysisContext() = default;

void TerrainAnalysisContext::onOdometry(double x, double y, double z,
                                        double roll, double pitch, double yaw) {
  vehicle_x_ = x;
  vehicle_y_ = y;
  vehicle_z_ = z;
  vehicle_roll_ = roll;
  vehicle_pitch_ = pitch;
  vehicle_yaw_ = yaw;

  sin_vehicle_roll_ = sin(vehicle_roll_);
  cos_vehicle_roll_ = cos(vehicle_roll_);
  sin_vehicle_pitch_ = sin(vehicle_pitch_);
  cos_vehicle_pitch_ = cos(vehicle_pitch_);
  sin_vehicle_yaw_ = sin(vehicle_yaw_);
  cos_vehicle_yaw_ = cos(vehicle_yaw_);

  if (no_data_inited_ == 0) {
    vehicle_x_rec_ = vehicle_x_;
    vehicle_y_rec_ = vehicle_y_;
    no_data_inited_ = 1;
  }
  if (no_data_inited_ == 1) {
    double dis = horizontalDistanceTo(vehicle_x_rec_, vehicle_y_rec_);
    if (dis >= no_decay_dis_) {
      no_data_inited_ = 2;
    }
  }
}

void TerrainAnalysisContext::onLaserCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double timestamp_sec) {
  laser_cloud_time_ = timestamp_sec;
  if (!system_inited_) {
    system_init_time_ = laser_cloud_time_;
    system_inited_ = true;
  }

  laser_cloud_->clear();
  *laser_cloud_ = *cloud;

  pcl::PointXYZI point;
  laser_cloud_crop_->clear();
  int laser_cloud_size = static_cast<int>(laser_cloud_->points.size());
  for (int i = 0; i < laser_cloud_size; i++) {
    point = laser_cloud_->points[i];

    float point_x = point.x;
    float point_y = point.y;
    float point_z = point.z;

    double dis = horizontalDistanceTo(point_x, point_y);
    if (point_z - vehicle_z_ > min_rel_z_ - (dis_ratio_z_ * dis)
        && point_z - vehicle_z_ < max_rel_z_ + (dis_ratio_z_ * dis)
        && dis < terrain_voxel_size_ * (kTerrainVoxelHalfWidth + 1)) {
      point.x = point_x;
      point.y = point_y;
      point.z = point_z;
      point.intensity = static_cast<float>(laser_cloud_time_
                                           - system_init_time_);
      laser_cloud_crop_->push_back(point);
    }
  }

  new_laser_cloud_ = true;
}

void TerrainAnalysisContext::onJoystick(bool button5) {
  if (button5) {
    no_data_inited_ = 0;
    clearing_cloud_ = true;
  }
}

void TerrainAnalysisContext::onClearing(float dis) {
  no_data_inited_ = 0;
  clearing_dis_ = dis;
  clearing_cloud_ = true;
}
