// Copyright 2024 Hongbiao Zhu
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/context.hpp"

#include <algorithm>
#include <cmath>

namespace TerrainAlgorithm {

void run(TerrainAnalysisContext& ctx) {
  ctx.new_laser_cloud_ = false;

  float terrainVoxelCenX = ctx.terrain_voxel_size_ * ctx.terrain_voxel_shift_x_;
  float terrainVoxelCenY = ctx.terrain_voxel_size_ * ctx.terrain_voxel_shift_y_;

  while (ctx.vehicle_x_ - terrainVoxelCenX < -ctx.terrain_voxel_size_) {
    for (int indY = 0; indY < TerrainAnalysisContext::kTerrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth *
                                       (TerrainAnalysisContext::kTerrainVoxelWidth - 1) +
                                   indY];
      for (int indX = TerrainAnalysisContext::kTerrainVoxelWidth - 1; indX >= 1; indX--) {
        ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX + indY] =
            ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * (indX - 1) + indY];
      }
      ctx.terrain_voxel_cloud_[indY] = terrainVoxelCloudPtr;
      ctx.terrain_voxel_cloud_[indY]->clear();
    }
    ctx.terrain_voxel_shift_x_--;
    terrainVoxelCenX = ctx.terrain_voxel_size_ * ctx.terrain_voxel_shift_x_;
  }

  while (ctx.vehicle_x_ - terrainVoxelCenX > ctx.terrain_voxel_size_) {
    for (int indY = 0; indY < TerrainAnalysisContext::kTerrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          ctx.terrain_voxel_cloud_[indY];
      for (int indX = 0; indX < TerrainAnalysisContext::kTerrainVoxelWidth - 1; indX++) {
        ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX + indY] =
            ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * (indX + 1) + indY];
      }
      ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth *
                                   (TerrainAnalysisContext::kTerrainVoxelWidth - 1) +
                               indY] = terrainVoxelCloudPtr;
      ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth *
                                   (TerrainAnalysisContext::kTerrainVoxelWidth - 1) +
                               indY]
          ->clear();
    }
    ctx.terrain_voxel_shift_x_++;
    terrainVoxelCenX = ctx.terrain_voxel_size_ * ctx.terrain_voxel_shift_x_;
  }

  while (ctx.vehicle_y_ - terrainVoxelCenY < -ctx.terrain_voxel_size_) {
    for (int indX = 0; indX < TerrainAnalysisContext::kTerrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX +
                                   (TerrainAnalysisContext::kTerrainVoxelWidth - 1)];
      for (int indY = TerrainAnalysisContext::kTerrainVoxelWidth - 1; indY >= 1; indY--) {
        ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX + indY] =
            ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX +
                                     (indY - 1)];
      }
      ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX] =
          terrainVoxelCloudPtr;
      ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX]->clear();
    }
    ctx.terrain_voxel_shift_y_--;
    terrainVoxelCenY = ctx.terrain_voxel_size_ * ctx.terrain_voxel_shift_y_;
  }

  while (ctx.vehicle_y_ - terrainVoxelCenY > ctx.terrain_voxel_size_) {
    for (int indX = 0; indX < TerrainAnalysisContext::kTerrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX];
      for (int indY = 0; indY < TerrainAnalysisContext::kTerrainVoxelWidth - 1; indY++) {
        ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX + indY] =
            ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX +
                                     (indY + 1)];
      }
      ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX +
                               (TerrainAnalysisContext::kTerrainVoxelWidth - 1)] =
          terrainVoxelCloudPtr;
      ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX +
                               (TerrainAnalysisContext::kTerrainVoxelWidth - 1)]
          ->clear();
    }
    ctx.terrain_voxel_shift_y_++;
    terrainVoxelCenY = ctx.terrain_voxel_size_ * ctx.terrain_voxel_shift_y_;
  }

  // stack registered laser scans
  pcl::PointXYZI point;
  int laserCloudCropSize = ctx.laser_cloud_crop_->points.size();
  for (int i = 0; i < laserCloudCropSize; i++) {
    point = ctx.laser_cloud_crop_->points[i];

    int indX = static_cast<int>((point.x - ctx.vehicle_x_ + ctx.terrain_voxel_size_ / 2) /
                                 ctx.terrain_voxel_size_) +
               TerrainAnalysisContext::kTerrainVoxelHalfWidth;
    int indY = static_cast<int>((point.y - ctx.vehicle_y_ + ctx.terrain_voxel_size_ / 2) /
                                 ctx.terrain_voxel_size_) +
               TerrainAnalysisContext::kTerrainVoxelHalfWidth;

    if (point.x - ctx.vehicle_x_ + ctx.terrain_voxel_size_ / 2 < 0) indX--;
    if (point.y - ctx.vehicle_y_ + ctx.terrain_voxel_size_ / 2 < 0) indY--;

    if (indX >= 0 && indX < TerrainAnalysisContext::kTerrainVoxelWidth &&
        indY >= 0 && indY < TerrainAnalysisContext::kTerrainVoxelWidth) {
      ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX + indY]
          ->push_back(point);
      ctx.terrain_voxel_update_num_[TerrainAnalysisContext::kTerrainVoxelWidth * indX +
                                    indY]++;
    }
  }

  for (int ind = 0; ind < TerrainAnalysisContext::kTerrainVoxelNum; ind++) {
    if (ctx.terrain_voxel_update_num_[ind] >= ctx.voxel_point_update_thre_ ||
        ctx.laser_cloud_time_ - ctx.system_init_time_ -
                ctx.terrain_voxel_update_time_[ind] >=
            ctx.voxel_time_update_thre_ ||
        ctx.clearing_cloud_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          ctx.terrain_voxel_cloud_[ind];

      ctx.laser_cloud_dwz_->clear();
      ctx.down_size_filter_.setInputCloud(terrainVoxelCloudPtr);
      ctx.down_size_filter_.filter(*ctx.laser_cloud_dwz_);

      terrainVoxelCloudPtr->clear();
      int laserCloudDwzSize = ctx.laser_cloud_dwz_->points.size();
      for (int i = 0; i < laserCloudDwzSize; i++) {
        point = ctx.laser_cloud_dwz_->points[i];
        float dis = sqrt((point.x - ctx.vehicle_x_) * (point.x - ctx.vehicle_x_) +
                         (point.y - ctx.vehicle_y_) * (point.y - ctx.vehicle_y_));
        if (point.z - ctx.vehicle_z_ > ctx.min_rel_z_ - ctx.dis_ratio_z_ * dis &&
            point.z - ctx.vehicle_z_ < ctx.max_rel_z_ + ctx.dis_ratio_z_ * dis &&
            (ctx.laser_cloud_time_ - ctx.system_init_time_ - point.intensity <
                 ctx.decay_time_ ||
             dis < ctx.no_decay_dis_) &&
            !(dis < ctx.clearing_dis_ && ctx.clearing_cloud_)) {
          terrainVoxelCloudPtr->push_back(point);
        }
      }

      ctx.terrain_voxel_update_num_[ind] = 0;
      ctx.terrain_voxel_update_time_[ind] = ctx.laser_cloud_time_ - ctx.system_init_time_;
    }
  }

  ctx.terrain_cloud_->clear();
  for (int indX = TerrainAnalysisContext::kTerrainVoxelHalfWidth - 5;
       indX <= TerrainAnalysisContext::kTerrainVoxelHalfWidth + 5; indX++) {
    for (int indY = TerrainAnalysisContext::kTerrainVoxelHalfWidth - 5;
         indY <= TerrainAnalysisContext::kTerrainVoxelHalfWidth + 5; indY++) {
      *ctx.terrain_cloud_ +=
          *ctx.terrain_voxel_cloud_[TerrainAnalysisContext::kTerrainVoxelWidth * indX + indY];
    }
  }

  // estimate ground
  for (int i = 0; i < TerrainAnalysisContext::kPlanarVoxelNum; i++) {
    ctx.planar_voxel_elev_[i] = 0;
    ctx.planar_voxel_edge_[i] = 0;
    ctx.planar_voxel_dy_obs_[i] = 0;
    ctx.planar_point_elev_[i].clear();
  }

  int terrainCloudSize = ctx.terrain_cloud_->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = ctx.terrain_cloud_->points[i];

    int indX = static_cast<int>((point.x - ctx.vehicle_x_ + ctx.planar_voxel_size_ / 2) /
                                 ctx.planar_voxel_size_) +
               TerrainAnalysisContext::kPlanarVoxelHalfWidth;
    int indY = static_cast<int>((point.y - ctx.vehicle_y_ + ctx.planar_voxel_size_ / 2) /
                                 ctx.planar_voxel_size_) +
               TerrainAnalysisContext::kPlanarVoxelHalfWidth;

    if (point.x - ctx.vehicle_x_ + ctx.planar_voxel_size_ / 2 < 0) indX--;
    if (point.y - ctx.vehicle_y_ + ctx.planar_voxel_size_ / 2 < 0) indY--;

    if (point.z - ctx.vehicle_z_ > ctx.min_rel_z_ && point.z - ctx.vehicle_z_ < ctx.max_rel_z_) {
      for (int dX = -1; dX <= 1; dX++) {
        for (int dY = -1; dY <= 1; dY++) {
          if (indX + dX >= 0 && indX + dX < TerrainAnalysisContext::kPlanarVoxelWidth &&
              indY + dY >= 0 && indY + dY < TerrainAnalysisContext::kPlanarVoxelWidth) {
            ctx.planar_point_elev_[TerrainAnalysisContext::kPlanarVoxelWidth * (indX + dX) +
                                   indY + dY]
                .push_back(point.z);
          }
        }
      }
    }

    if (ctx.clear_dy_obs_) {
      if (indX >= 0 && indX < TerrainAnalysisContext::kPlanarVoxelWidth && indY >= 0 &&
          indY < TerrainAnalysisContext::kPlanarVoxelWidth) {
        float pointX1 = point.x - ctx.vehicle_x_;
        float pointY1 = point.y - ctx.vehicle_y_;
        float pointZ1 = point.z - ctx.vehicle_z_;

        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
        if (dis1 > ctx.min_dy_obs_dis_) {
          float angle1 = atan2(pointZ1 - ctx.min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
          if (angle1 > ctx.min_dy_obs_angle_) {
            float pointX2 = pointX1 * ctx.cos_vehicle_yaw_ + pointY1 * ctx.sin_vehicle_yaw_;
            float pointY2 = -pointX1 * ctx.sin_vehicle_yaw_ + pointY1 * ctx.cos_vehicle_yaw_;
            float pointZ2 = pointZ1;

            float pointX3 = pointX2 * ctx.cos_vehicle_pitch_ - pointZ2 * ctx.sin_vehicle_pitch_;
            float pointY3 = pointY2;
            float pointZ3 = pointX2 * ctx.sin_vehicle_pitch_ + pointZ2 * ctx.cos_vehicle_pitch_;

            float pointX4 = pointX3;
            float pointY4 = pointY3 * ctx.cos_vehicle_roll_ + pointZ3 * ctx.sin_vehicle_roll_;
            float pointZ4 = -pointY3 * ctx.sin_vehicle_roll_ + pointZ3 * ctx.cos_vehicle_roll_;

            float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
            float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
            if ((angle4 > ctx.min_dy_obs_vfov_ && angle4 < ctx.max_dy_obs_vfov_) ||
                fabs(pointZ4) < ctx.abs_dy_obs_rel_z_thre_) {
              ctx.planar_voxel_dy_obs_[TerrainAnalysisContext::kPlanarVoxelWidth * indX +
                                       indY]++;
            }
          }
        } else {
          ctx.planar_voxel_dy_obs_[TerrainAnalysisContext::kPlanarVoxelWidth * indX + indY] +=
              ctx.min_dy_obs_point_num_;
        }
      }
    }
  }

  if (ctx.clear_dy_obs_) {
    for (int i = 0; i < laserCloudCropSize; i++) {
      point = ctx.laser_cloud_crop_->points[i];

      int indX = static_cast<int>((point.x - ctx.vehicle_x_ + ctx.planar_voxel_size_ / 2) /
                                   ctx.planar_voxel_size_) +
                 TerrainAnalysisContext::kPlanarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - ctx.vehicle_y_ + ctx.planar_voxel_size_ / 2) /
                                   ctx.planar_voxel_size_) +
                 TerrainAnalysisContext::kPlanarVoxelHalfWidth;

      if (point.x - ctx.vehicle_x_ + ctx.planar_voxel_size_ / 2 < 0) indX--;
      if (point.y - ctx.vehicle_y_ + ctx.planar_voxel_size_ / 2 < 0) indY--;

      if (indX >= 0 && indX < TerrainAnalysisContext::kPlanarVoxelWidth && indY >= 0 &&
          indY < TerrainAnalysisContext::kPlanarVoxelWidth) {
        float pointX1 = point.x - ctx.vehicle_x_;
        float pointY1 = point.y - ctx.vehicle_y_;
        float pointZ1 = point.z - ctx.vehicle_z_;

        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
        float angle1 = atan2(pointZ1 - ctx.min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
        if (angle1 > ctx.min_dy_obs_angle_) {
          ctx.planar_voxel_dy_obs_[TerrainAnalysisContext::kPlanarVoxelWidth * indX + indY] = 0;
        }
      }
    }
  }

  if (ctx.use_sorting_) {
    for (int i = 0; i < TerrainAnalysisContext::kPlanarVoxelNum; i++) {
      int planarPointElevSize = ctx.planar_point_elev_[i].size();
      if (planarPointElevSize > 0) {
        sort(ctx.planar_point_elev_[i].begin(), ctx.planar_point_elev_[i].end());

        int quantileID = static_cast<int>(ctx.quantile_z_ * planarPointElevSize);
        if (quantileID < 0)
          quantileID = 0;
        else if (quantileID >= planarPointElevSize)
          quantileID = planarPointElevSize - 1;

        if (ctx.planar_point_elev_[i][quantileID] >
                ctx.planar_point_elev_[i][0] + ctx.max_ground_lift_ &&
            ctx.limit_ground_lift_) {
          ctx.planar_voxel_elev_[i] = ctx.planar_point_elev_[i][0] + ctx.max_ground_lift_;
        } else {
          ctx.planar_voxel_elev_[i] = ctx.planar_point_elev_[i][quantileID];
        }
      }
    }
  } else {
    for (int i = 0; i < TerrainAnalysisContext::kPlanarVoxelNum; i++) {
      int planarPointElevSize = ctx.planar_point_elev_[i].size();
      if (planarPointElevSize > 0) {
        float minZ = 1000.0;
        int minID = -1;
        for (int j = 0; j < planarPointElevSize; j++) {
          if (ctx.planar_point_elev_[i][j] < minZ) {
            minZ = ctx.planar_point_elev_[i][j];
            minID = j;
          }
        }

        if (minID != -1) {
          ctx.planar_voxel_elev_[i] = ctx.planar_point_elev_[i][minID];
        }
      }
    }
  }

  ctx.terrain_cloud_elev_->clear();
  int terrainCloudElevSize = 0;
  for (int i = 0; i < terrainCloudSize; i++) {
    point = ctx.terrain_cloud_->points[i];
    if (point.z - ctx.vehicle_z_ > ctx.min_rel_z_ && point.z - ctx.vehicle_z_ < ctx.max_rel_z_) {
      int indX = static_cast<int>((point.x - ctx.vehicle_x_ + ctx.planar_voxel_size_ / 2) /
                                   ctx.planar_voxel_size_) +
                 TerrainAnalysisContext::kPlanarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - ctx.vehicle_y_ + ctx.planar_voxel_size_ / 2) /
                                   ctx.planar_voxel_size_) +
                 TerrainAnalysisContext::kPlanarVoxelHalfWidth;

      if (point.x - ctx.vehicle_x_ + ctx.planar_voxel_size_ / 2 < 0) indX--;
      if (point.y - ctx.vehicle_y_ + ctx.planar_voxel_size_ / 2 < 0) indY--;

      if (indX >= 0 && indX < TerrainAnalysisContext::kPlanarVoxelWidth && indY >= 0 &&
          indY < TerrainAnalysisContext::kPlanarVoxelWidth) {
        if (ctx.planar_voxel_dy_obs_[TerrainAnalysisContext::kPlanarVoxelWidth * indX + indY] <
                ctx.min_dy_obs_point_num_ ||
            !ctx.clear_dy_obs_) {
          float disZ =
              point.z - ctx.planar_voxel_elev_[TerrainAnalysisContext::kPlanarVoxelWidth * indX +
                                                indY];
          if (ctx.consider_drop_) disZ = fabs(disZ);
          int planarPointElevSize =
              ctx.planar_point_elev_[TerrainAnalysisContext::kPlanarVoxelWidth * indX + indY]
                  .size();
          if (disZ >= 0 && disZ < ctx.vehicle_height_ &&
              planarPointElevSize >= ctx.min_block_point_num_) {
            ctx.terrain_cloud_elev_->push_back(point);
            ctx.terrain_cloud_elev_->points[terrainCloudElevSize].intensity = disZ;
            terrainCloudElevSize++;
          }
        }
      }
    }
  }

  if (ctx.no_data_obstacle_ && ctx.no_data_inited_ == 2) {
    for (int i = 0; i < TerrainAnalysisContext::kPlanarVoxelNum; i++) {
      int planarPointElevSize = ctx.planar_point_elev_[i].size();
      if (planarPointElevSize < ctx.min_block_point_num_) {
        ctx.planar_voxel_edge_[i] = 1;
      }
    }

    for (int noDataBlockSkipCount = 0; noDataBlockSkipCount < ctx.no_data_block_skip_num_;
         noDataBlockSkipCount++) {
      for (int i = 0; i < TerrainAnalysisContext::kPlanarVoxelNum; i++) {
        if (ctx.planar_voxel_edge_[i] >= 1) {
          int indX = static_cast<int>(i / TerrainAnalysisContext::kPlanarVoxelWidth);
          int indY = i % TerrainAnalysisContext::kPlanarVoxelWidth;
          bool edgeVoxel = false;
          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < TerrainAnalysisContext::kPlanarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < TerrainAnalysisContext::kPlanarVoxelWidth) {
                if (ctx.planar_voxel_edge_[TerrainAnalysisContext::kPlanarVoxelWidth *
                                              (indX + dX) +
                                          indY + dY] < ctx.planar_voxel_edge_[i]) {
                  edgeVoxel = true;
                }
              }
            }
          }

          if (!edgeVoxel) ctx.planar_voxel_edge_[i]++;
        }
      }
    }

    for (int i = 0; i < TerrainAnalysisContext::kPlanarVoxelNum; i++) {
      if (ctx.planar_voxel_edge_[i] > ctx.no_data_block_skip_num_) {
        int indX = static_cast<int>(i / TerrainAnalysisContext::kPlanarVoxelWidth);
        int indY = i % TerrainAnalysisContext::kPlanarVoxelWidth;

        point.x = ctx.planar_voxel_size_ * (indX - TerrainAnalysisContext::kPlanarVoxelHalfWidth) +
                  ctx.vehicle_x_;
        point.y = ctx.planar_voxel_size_ * (indY - TerrainAnalysisContext::kPlanarVoxelHalfWidth) +
                  ctx.vehicle_y_;
        point.z = ctx.vehicle_z_;
        point.intensity = ctx.vehicle_height_;

        point.x -= ctx.planar_voxel_size_ / 4.0;
        point.y -= ctx.planar_voxel_size_ / 4.0;
        ctx.terrain_cloud_elev_->push_back(point);

        point.x += ctx.planar_voxel_size_ / 2.0;
        ctx.terrain_cloud_elev_->push_back(point);

        point.y += ctx.planar_voxel_size_ / 2.0;
        ctx.terrain_cloud_elev_->push_back(point);

        point.x -= ctx.planar_voxel_size_ / 2.0;
        ctx.terrain_cloud_elev_->push_back(point);
      }
    }
  }

  ctx.clearing_cloud_ = false;
}

}  // namespace TerrainAlgorithm
