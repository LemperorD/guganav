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

void TerrainAnalysisContext::onOdometry(float x, float y, float z,
                                       float roll, float pitch, float yaw) {
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
    float dis =
        sqrt((vehicle_x_ - vehicle_x_rec_) * (vehicle_x_ - vehicle_x_rec_) +
             (vehicle_y_ - vehicle_y_rec_) * (vehicle_y_ - vehicle_y_rec_));
    if (dis >= no_decay_dis_) no_data_inited_ = 2;
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

    float dis = sqrt((pointX - vehicle_x_) * (pointX - vehicle_x_) +
                     (pointY - vehicle_y_) * (pointY - vehicle_y_));
    if (pointZ - vehicle_z_ > min_rel_z_ - dis_ratio_z_ * dis &&
        pointZ - vehicle_z_ < max_rel_z_ + dis_ratio_z_ * dis &&
        dis < terrain_voxel_size_ * (kTerrainVoxelHalfWidth + 1)) {
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

// ── Pipeline ──

void TerrainAnalysisContext::processTerrainData() {
  new_laser_cloud_ = false;

  float terrainVoxelCenX = terrain_voxel_size_ * terrain_voxel_shift_x_;
  float terrainVoxelCenY = terrain_voxel_size_ * terrain_voxel_shift_y_;

  while (vehicle_x_ - terrainVoxelCenX < -terrain_voxel_size_) {
    for (int indY = 0; indY < kTerrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrain_voxel_cloud_[kTerrainVoxelWidth * (kTerrainVoxelWidth - 1) + indY];
      for (int indX = kTerrainVoxelWidth - 1; indX >= 1; indX--) {
        terrain_voxel_cloud_[kTerrainVoxelWidth * indX + indY] =
            terrain_voxel_cloud_[kTerrainVoxelWidth * (indX - 1) + indY];
      }
      terrain_voxel_cloud_[indY] = terrainVoxelCloudPtr;
      terrain_voxel_cloud_[indY]->clear();
    }
    terrain_voxel_shift_x_--;
    terrainVoxelCenX = terrain_voxel_size_ * terrain_voxel_shift_x_;
  }

  while (vehicle_x_ - terrainVoxelCenX > terrain_voxel_size_) {
    for (int indY = 0; indY < kTerrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrain_voxel_cloud_[indY];
      for (int indX = 0; indX < kTerrainVoxelWidth - 1; indX++) {
        terrain_voxel_cloud_[kTerrainVoxelWidth * indX + indY] =
            terrain_voxel_cloud_[kTerrainVoxelWidth * (indX + 1) + indY];
      }
      terrain_voxel_cloud_[kTerrainVoxelWidth * (kTerrainVoxelWidth - 1) + indY] =
          terrainVoxelCloudPtr;
      terrain_voxel_cloud_[kTerrainVoxelWidth * (kTerrainVoxelWidth - 1) + indY]
          ->clear();
    }
    terrain_voxel_shift_x_++;
    terrainVoxelCenX = terrain_voxel_size_ * terrain_voxel_shift_x_;
  }

  while (vehicle_y_ - terrainVoxelCenY < -terrain_voxel_size_) {
    for (int indX = 0; indX < kTerrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrain_voxel_cloud_[kTerrainVoxelWidth * indX + (kTerrainVoxelWidth - 1)];
      for (int indY = kTerrainVoxelWidth - 1; indY >= 1; indY--) {
        terrain_voxel_cloud_[kTerrainVoxelWidth * indX + indY] =
            terrain_voxel_cloud_[kTerrainVoxelWidth * indX + (indY - 1)];
      }
      terrain_voxel_cloud_[kTerrainVoxelWidth * indX] = terrainVoxelCloudPtr;
      terrain_voxel_cloud_[kTerrainVoxelWidth * indX]->clear();
    }
    terrain_voxel_shift_y_--;
    terrainVoxelCenY = terrain_voxel_size_ * terrain_voxel_shift_y_;
  }

  while (vehicle_y_ - terrainVoxelCenY > terrain_voxel_size_) {
    for (int indX = 0; indX < kTerrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrain_voxel_cloud_[kTerrainVoxelWidth * indX];
      for (int indY = 0; indY < kTerrainVoxelWidth - 1; indY++) {
        terrain_voxel_cloud_[kTerrainVoxelWidth * indX + indY] =
            terrain_voxel_cloud_[kTerrainVoxelWidth * indX + (indY + 1)];
      }
      terrain_voxel_cloud_[kTerrainVoxelWidth * indX + (kTerrainVoxelWidth - 1)] =
          terrainVoxelCloudPtr;
      terrain_voxel_cloud_[kTerrainVoxelWidth * indX + (kTerrainVoxelWidth - 1)]
          ->clear();
    }
    terrain_voxel_shift_y_++;
    terrainVoxelCenY = terrain_voxel_size_ * terrain_voxel_shift_y_;
  }

  // stack registered laser scans
  pcl::PointXYZI point;
  int laserCloudCropSize = laser_cloud_crop_->points.size();
  for (int i = 0; i < laserCloudCropSize; i++) {
    point = laser_cloud_crop_->points[i];

    int indX = static_cast<int>((point.x - vehicle_x_ + terrain_voxel_size_ / 2) /
                                 terrain_voxel_size_) +
               kTerrainVoxelHalfWidth;
    int indY = static_cast<int>((point.y - vehicle_y_ + terrain_voxel_size_ / 2) /
                                 terrain_voxel_size_) +
               kTerrainVoxelHalfWidth;

    if (point.x - vehicle_x_ + terrain_voxel_size_ / 2 < 0) indX--;
    if (point.y - vehicle_y_ + terrain_voxel_size_ / 2 < 0) indY--;

    if (indX >= 0 && indX < kTerrainVoxelWidth && indY >= 0 &&
        indY < kTerrainVoxelWidth) {
      terrain_voxel_cloud_[kTerrainVoxelWidth * indX + indY]->push_back(point);
      terrain_voxel_update_num_[kTerrainVoxelWidth * indX + indY]++;
    }
  }

  for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
    if (terrain_voxel_update_num_[ind] >= voxel_point_update_thre_ ||
        laser_cloud_time_ - system_init_time_ - terrain_voxel_update_time_[ind] >=
            voxel_time_update_thre_ ||
        clearing_cloud_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrain_voxel_cloud_[ind];

      laser_cloud_dwz_->clear();
      down_size_filter_.setInputCloud(terrainVoxelCloudPtr);
      down_size_filter_.filter(*laser_cloud_dwz_);

      terrainVoxelCloudPtr->clear();
      int laserCloudDwzSize = laser_cloud_dwz_->points.size();
      for (int i = 0; i < laserCloudDwzSize; i++) {
        point = laser_cloud_dwz_->points[i];
        float dis = sqrt((point.x - vehicle_x_) * (point.x - vehicle_x_) +
                         (point.y - vehicle_y_) * (point.y - vehicle_y_));
        if (point.z - vehicle_z_ > min_rel_z_ - dis_ratio_z_ * dis &&
            point.z - vehicle_z_ < max_rel_z_ + dis_ratio_z_ * dis &&
            (laser_cloud_time_ - system_init_time_ - point.intensity <
                 decay_time_ ||
             dis < no_decay_dis_) &&
            !(dis < clearing_dis_ && clearing_cloud_)) {
          terrainVoxelCloudPtr->push_back(point);
        }
      }

      terrain_voxel_update_num_[ind] = 0;
      terrain_voxel_update_time_[ind] = laser_cloud_time_ - system_init_time_;
    }
  }

  terrain_cloud_->clear();
  for (int indX = kTerrainVoxelHalfWidth - 5;
       indX <= kTerrainVoxelHalfWidth + 5; indX++) {
    for (int indY = kTerrainVoxelHalfWidth - 5;
         indY <= kTerrainVoxelHalfWidth + 5; indY++) {
      *terrain_cloud_ += *terrain_voxel_cloud_[kTerrainVoxelWidth * indX + indY];
    }
  }

  // estimate ground
  for (int i = 0; i < kPlanarVoxelNum; i++) {
    planar_voxel_elev_[i] = 0;
    planar_voxel_edge_[i] = 0;
    planar_voxel_dy_obs_[i] = 0;
    planar_point_elev_[i].clear();
  }

  int terrainCloudSize = terrain_cloud_->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrain_cloud_->points[i];

    int indX = static_cast<int>((point.x - vehicle_x_ + planar_voxel_size_ / 2) /
                                 planar_voxel_size_) +
               kPlanarVoxelHalfWidth;
    int indY = static_cast<int>((point.y - vehicle_y_ + planar_voxel_size_ / 2) /
                                 planar_voxel_size_) +
               kPlanarVoxelHalfWidth;

    if (point.x - vehicle_x_ + planar_voxel_size_ / 2 < 0) indX--;
    if (point.y - vehicle_y_ + planar_voxel_size_ / 2 < 0) indY--;

    if (point.z - vehicle_z_ > min_rel_z_ && point.z - vehicle_z_ < max_rel_z_) {
      for (int dX = -1; dX <= 1; dX++) {
        for (int dY = -1; dY <= 1; dY++) {
          if (indX + dX >= 0 && indX + dX < kPlanarVoxelWidth &&
              indY + dY >= 0 && indY + dY < kPlanarVoxelWidth) {
            planar_point_elev_[kPlanarVoxelWidth * (indX + dX) + indY + dY]
                .push_back(point.z);
          }
        }
      }
    }

    if (clear_dy_obs_) {
      if (indX >= 0 && indX < kPlanarVoxelWidth && indY >= 0 &&
          indY < kPlanarVoxelWidth) {
        float pointX1 = point.x - vehicle_x_;
        float pointY1 = point.y - vehicle_y_;
        float pointZ1 = point.z - vehicle_z_;

        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
        if (dis1 > min_dy_obs_dis_) {
          float angle1 = atan2(pointZ1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
          if (angle1 > min_dy_obs_angle_) {
            float pointX2 = pointX1 * cos_vehicle_yaw_ + pointY1 * sin_vehicle_yaw_;
            float pointY2 = -pointX1 * sin_vehicle_yaw_ + pointY1 * cos_vehicle_yaw_;
            float pointZ2 = pointZ1;

            float pointX3 =
                pointX2 * cos_vehicle_pitch_ - pointZ2 * sin_vehicle_pitch_;
            float pointY3 = pointY2;
            float pointZ3 =
                pointX2 * sin_vehicle_pitch_ + pointZ2 * cos_vehicle_pitch_;

            float pointX4 = pointX3;
            float pointY4 =
                pointY3 * cos_vehicle_roll_ + pointZ3 * sin_vehicle_roll_;
            float pointZ4 =
                -pointY3 * sin_vehicle_roll_ + pointZ3 * cos_vehicle_roll_;

            float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
            float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
            if ((angle4 > min_dy_obs_vfov_ && angle4 < max_dy_obs_vfov_) ||
                fabs(pointZ4) < abs_dy_obs_rel_z_thre_) {
              planar_voxel_dy_obs_[kPlanarVoxelWidth * indX + indY]++;
            }
          }
        } else {
          planar_voxel_dy_obs_[kPlanarVoxelWidth * indX + indY] +=
              min_dy_obs_point_num_;
        }
      }
    }
  }

  if (clear_dy_obs_) {
    for (int i = 0; i < laserCloudCropSize; i++) {
      point = laser_cloud_crop_->points[i];

      int indX = static_cast<int>((point.x - vehicle_x_ + planar_voxel_size_ / 2) /
                                   planar_voxel_size_) +
                 kPlanarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicle_y_ + planar_voxel_size_ / 2) /
                                   planar_voxel_size_) +
                 kPlanarVoxelHalfWidth;

      if (point.x - vehicle_x_ + planar_voxel_size_ / 2 < 0) indX--;
      if (point.y - vehicle_y_ + planar_voxel_size_ / 2 < 0) indY--;

      if (indX >= 0 && indX < kPlanarVoxelWidth && indY >= 0 &&
          indY < kPlanarVoxelWidth) {
        float pointX1 = point.x - vehicle_x_;
        float pointY1 = point.y - vehicle_y_;
        float pointZ1 = point.z - vehicle_z_;

        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
        float angle1 = atan2(pointZ1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
        if (angle1 > min_dy_obs_angle_) {
          planar_voxel_dy_obs_[kPlanarVoxelWidth * indX + indY] = 0;
        }
      }
    }
  }

  if (use_sorting_) {
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      int planarPointElevSize = planar_point_elev_[i].size();
      if (planarPointElevSize > 0) {
        sort(planar_point_elev_[i].begin(), planar_point_elev_[i].end());

        int quantileID = static_cast<int>(quantile_z_ * planarPointElevSize);
        if (quantileID < 0)
          quantileID = 0;
        else if (quantileID >= planarPointElevSize)
          quantileID = planarPointElevSize - 1;

        if (planar_point_elev_[i][quantileID] >
                planar_point_elev_[i][0] + max_ground_lift_ &&
            limit_ground_lift_) {
          planar_voxel_elev_[i] = planar_point_elev_[i][0] + max_ground_lift_;
        } else {
          planar_voxel_elev_[i] = planar_point_elev_[i][quantileID];
        }
      }
    }
  } else {
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      int planarPointElevSize = planar_point_elev_[i].size();
      if (planarPointElevSize > 0) {
        float minZ = 1000.0;
        int minID = -1;
        for (int j = 0; j < planarPointElevSize; j++) {
          if (planar_point_elev_[i][j] < minZ) {
            minZ = planar_point_elev_[i][j];
            minID = j;
          }
        }

        if (minID != -1) {
          planar_voxel_elev_[i] = planar_point_elev_[i][minID];
        }
      }
    }
  }

  terrain_cloud_elev_->clear();
  int terrainCloudElevSize = 0;
  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrain_cloud_->points[i];
    if (point.z - vehicle_z_ > min_rel_z_ && point.z - vehicle_z_ < max_rel_z_) {
      int indX = static_cast<int>((point.x - vehicle_x_ + planar_voxel_size_ / 2) /
                                   planar_voxel_size_) +
                 kPlanarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicle_y_ + planar_voxel_size_ / 2) /
                                   planar_voxel_size_) +
                 kPlanarVoxelHalfWidth;

      if (point.x - vehicle_x_ + planar_voxel_size_ / 2 < 0) indX--;
      if (point.y - vehicle_y_ + planar_voxel_size_ / 2 < 0) indY--;

      if (indX >= 0 && indX < kPlanarVoxelWidth && indY >= 0 &&
          indY < kPlanarVoxelWidth) {
        if (planar_voxel_dy_obs_[kPlanarVoxelWidth * indX + indY] <
                min_dy_obs_point_num_ ||
            !clear_dy_obs_) {
          float disZ =
              point.z - planar_voxel_elev_[kPlanarVoxelWidth * indX + indY];
          if (consider_drop_) disZ = fabs(disZ);
          int planarPointElevSize =
              planar_point_elev_[kPlanarVoxelWidth * indX + indY].size();
          if (disZ >= 0 && disZ < vehicle_height_ &&
              planarPointElevSize >= min_block_point_num_) {
            terrain_cloud_elev_->push_back(point);
            terrain_cloud_elev_->points[terrainCloudElevSize].intensity = disZ;
            terrainCloudElevSize++;
          }
        }
      }
    }
  }

  if (no_data_obstacle_ && no_data_inited_ == 2) {
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      int planarPointElevSize = planar_point_elev_[i].size();
      if (planarPointElevSize < min_block_point_num_) {
        planar_voxel_edge_[i] = 1;
      }
    }

    for (int noDataBlockSkipCount = 0; noDataBlockSkipCount < no_data_block_skip_num_;
         noDataBlockSkipCount++) {
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        if (planar_voxel_edge_[i] >= 1) {
          int indX = static_cast<int>(i / kPlanarVoxelWidth);
          int indY = i % kPlanarVoxelWidth;
          bool edgeVoxel = false;
          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < kPlanarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < kPlanarVoxelWidth) {
                if (planar_voxel_edge_[kPlanarVoxelWidth * (indX + dX) + indY + dY] <
                    planar_voxel_edge_[i]) {
                  edgeVoxel = true;
                }
              }
            }
          }

          if (!edgeVoxel) planar_voxel_edge_[i]++;
        }
      }
    }

    for (int i = 0; i < kPlanarVoxelNum; i++) {
      if (planar_voxel_edge_[i] > no_data_block_skip_num_) {
        int indX = static_cast<int>(i / kPlanarVoxelWidth);
        int indY = i % kPlanarVoxelWidth;

        point.x = planar_voxel_size_ * (indX - kPlanarVoxelHalfWidth) + vehicle_x_;
        point.y = planar_voxel_size_ * (indY - kPlanarVoxelHalfWidth) + vehicle_y_;
        point.z = vehicle_z_;
        point.intensity = vehicle_height_;

        point.x -= planar_voxel_size_ / 4.0;
        point.y -= planar_voxel_size_ / 4.0;
        terrain_cloud_elev_->push_back(point);

        point.x += planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);

        point.y += planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);

        point.x -= planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);
      }
    }
  }

  clearing_cloud_ = false;
}
