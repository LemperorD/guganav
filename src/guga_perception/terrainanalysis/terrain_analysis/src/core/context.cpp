// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#include "terrain_analysis/core/context.hpp"

#include <cmath>

TerrainAnalysisContext::TerrainAnalysisContext() {
  laser_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_crop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_dwz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  terrain_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  terrain_cloud_elev_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  for (int i = 0; i < kTerrainVoxelNum; i++) {
    terrain_voxel_cloud_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
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
    float dis = sqrt(
        (vehicle_x_ - vehicle_x_rec_) * (vehicle_x_ - vehicle_x_rec_)
        + (vehicle_y_ - vehicle_y_rec_) * (vehicle_y_ - vehicle_y_rec_));
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
  int laserCloudSize = laser_cloud_->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = laser_cloud_->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicle_x_) * (pointX - vehicle_x_)
                     + (pointY - vehicle_y_) * (pointY - vehicle_y_));
    if (pointZ - vehicle_z_ > min_rel_z_ - dis_ratio_z_ * dis
        && pointZ - vehicle_z_ < max_rel_z_ + dis_ratio_z_ * dis
        && dis < terrain_voxel_size_ * (kTerrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laser_cloud_time_ - system_init_time_;
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
