// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include "terrain_analysis/terrain_analysis.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ── run-time global definitions ──

double scanVoxelSize = 0.05;
double decayTime = 2.0;
double noDecayDis = 4.0;
double clearingDis = 8.0;
bool clearingCloud = false;
bool useSorting = true;
double quantileZ = 0.25;
bool considerDrop = false;
bool limitGroundLift = false;
double maxGroundLift = 0.15;
bool clearDyObs = false;
double minDyObsDis = 0.3;
double minDyObsAngle = 0;
double minDyObsRelZ = -0.5;
double absDyObsRelZThre = 0.2;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;
int minBlockPointNum = 10;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;

// terrain voxel
float terrainVoxelSize = 1.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;

// planar voxel
float planarVoxelSize = 0.2;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[kTerrainVoxelNum];

int terrainVoxelUpdateNum[kTerrainVoxelNum] = {0};
float terrainVoxelUpdateTime[kTerrainVoxelNum] = {0};
float planarVoxelElev[kPlanarVoxelNum] = {0};
int planarVoxelEdge[kPlanarVoxelNum] = {0};
int planarVoxelDyObs[kPlanarVoxelNum] = {0};
std::vector<float> planarPointElev[kPlanarVoxelNum];

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;
int noDataInited = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

// state estimation callback function
void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);

  if (noDataInited == 0) {
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1) {
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                     (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
    if (dis >= noDecayDis)
      noDataInited = 2;
  }
}

// registered laser scan callback function
void laserCloudHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {
  laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();
  if (!systemInited) {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY));
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
        pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// joystick callback function
void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy) {
  if (joy->buttons[5] > 0.5) {
    noDataInited = 0;
    clearingCloud = true;
  }
}

// cloud clearing callback function
void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis) {
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}

void processTerrainData() {
  newlaserCloud = false;

  // terrain voxel roll over
  float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
  float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

  while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
    for (int indY = 0; indY < terrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
      for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
      }
      terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
      terrainVoxelCloud[indY]->clear();
    }
    terrainVoxelShiftX--;
    terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
  }

  while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
    for (int indY = 0; indY < terrainVoxelWidth; indY++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[indY];
      for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
      }
      terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] =
          terrainVoxelCloudPtr;
      terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
          ->clear();
    }
    terrainVoxelShiftX++;
    terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
  }

  while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
    for (int indX = 0; indX < terrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
      for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
      }
      terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
      terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
    }
    terrainVoxelShiftY--;
    terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
  }

  while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
    for (int indX = 0; indX < terrainVoxelWidth; indX++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[terrainVoxelWidth * indX];
      for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
            terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
      }
      terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] =
          terrainVoxelCloudPtr;
      terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
          ->clear();
    }
    terrainVoxelShiftY++;
    terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
  }

  // stack registered laser scans
  pcl::PointXYZI point;
  int laserCloudCropSize = laserCloudCrop->points.size();
  for (int i = 0; i < laserCloudCropSize; i++) {
    point = laserCloudCrop->points[i];

    int indX = static_cast<int>((point.x - vehicleX + terrainVoxelSize / 2) /
                                 terrainVoxelSize) +
               terrainVoxelHalfWidth;
    int indY = static_cast<int>((point.y - vehicleY + terrainVoxelSize / 2) /
                                 terrainVoxelSize) +
               terrainVoxelHalfWidth;

    if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
      indX--;
    if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
      indY--;

    if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
        indY < terrainVoxelWidth) {
      terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
      terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
    }
  }

  for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
    if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
        laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
            voxelTimeUpdateThre ||
        clearingCloud) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
          terrainVoxelCloud[ind];

      laserCloudDwz->clear();
      downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
      downSizeFilter.filter(*laserCloudDwz);

      terrainVoxelCloudPtr->clear();
      int laserCloudDwzSize = laserCloudDwz->points.size();
      for (int i = 0; i < laserCloudDwzSize; i++) {
        point = laserCloudDwz->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
            point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
            (laserCloudTime - systemInitTime - point.intensity < decayTime ||
             dis < noDecayDis) &&
            !(dis < clearingDis && clearingCloud)) {
          terrainVoxelCloudPtr->push_back(point);
        }
      }

      terrainVoxelUpdateNum[ind] = 0;
      terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
    }
  }

  terrainCloud->clear();
  for (int indX = terrainVoxelHalfWidth - 5;
       indX <= terrainVoxelHalfWidth + 5; indX++) {
    for (int indY = terrainVoxelHalfWidth - 5;
         indY <= terrainVoxelHalfWidth + 5; indY++) {
      *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
    }
  }

  // estimate ground and compute elevation for each point
  for (int i = 0; i < kPlanarVoxelNum; i++) {
    planarVoxelElev[i] = 0;
    planarVoxelEdge[i] = 0;
    planarVoxelDyObs[i] = 0;
    planarPointElev[i].clear();
  }

  int terrainCloudSize = terrainCloud->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrainCloud->points[i];

    int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                planarVoxelSize) +
               planarVoxelHalfWidth;
    int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                planarVoxelSize) +
               planarVoxelHalfWidth;

    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
      indX--;
    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
      indY--;

    if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
      for (int dX = -1; dX <= 1; dX++) {
        for (int dY = -1; dY <= 1; dY++) {
          if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
              indY + dY >= 0 && indY + dY < planarVoxelWidth) {
            planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                .push_back(point.z);
          }
        }
      }
    }

    if (clearDyObs) {
      if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
          indY < planarVoxelWidth) {
        float pointX1 = point.x - vehicleX;
        float pointY1 = point.y - vehicleY;
        float pointZ1 = point.z - vehicleZ;

        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
        if (dis1 > minDyObsDis) {
          float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / M_PI;
          if (angle1 > minDyObsAngle) {
            float pointX2 = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
            float pointY2 = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
            float pointZ2 = pointZ1;

            float pointX3 =
                pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
            float pointY3 = pointY2;
            float pointZ3 =
                pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

            float pointX4 = pointX3;
            float pointY4 =
                pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
            float pointZ4 =
                -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

            float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
            float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
            if ((angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV) ||
                fabs(pointZ4) < absDyObsRelZThre) {
              planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
            }
          }
        } else {
          planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
              minDyObsPointNum;
        }
      }
    }
  }

  if (clearDyObs) {
    for (int i = 0; i < laserCloudCropSize; i++) {
      point = laserCloudCrop->points[i];

      int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                   planarVoxelSize) +
                 planarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                   planarVoxelSize) +
                 planarVoxelHalfWidth;

      if (point.x - vehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - vehicleY + planarVoxelSize / 2 < 0)
        indY--;

      if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
          indY < planarVoxelWidth) {
        float pointX1 = point.x - vehicleX;
        float pointY1 = point.y - vehicleY;
        float pointZ1 = point.z - vehicleZ;

        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
        float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / M_PI;
        if (angle1 > minDyObsAngle) {
          planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
        }
      }
    }
  }

  if (useSorting) {
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      int planarPointElevSize = planarPointElev[i].size();
      if (planarPointElevSize > 0) {
        sort(planarPointElev[i].begin(), planarPointElev[i].end());

        int quantileID = static_cast<int>(quantileZ * planarPointElevSize);
        if (quantileID < 0)
          quantileID = 0;
        else if (quantileID >= planarPointElevSize)
          quantileID = planarPointElevSize - 1;

        if (planarPointElev[i][quantileID] >
                planarPointElev[i][0] + maxGroundLift &&
            limitGroundLift) {
          planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
        } else {
          planarVoxelElev[i] = planarPointElev[i][quantileID];
        }
      }
    }
  } else {
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      int planarPointElevSize = planarPointElev[i].size();
      if (planarPointElevSize > 0) {
        float minZ = 1000.0;
        int minID = -1;
        for (int j = 0; j < planarPointElevSize; j++) {
          if (planarPointElev[i][j] < minZ) {
            minZ = planarPointElev[i][j];
            minID = j;
          }
        }

        if (minID != -1) {
          planarVoxelElev[i] = planarPointElev[i][minID];
        }
      }
    }
  }

  terrainCloudElev->clear();
  int terrainCloudElevSize = 0;
  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrainCloud->points[i];
    if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
      int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                   planarVoxelSize) +
                 planarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                   planarVoxelSize) +
                 planarVoxelHalfWidth;

      if (point.x - vehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - vehicleY + planarVoxelSize / 2 < 0)
        indY--;

      if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
          indY < planarVoxelWidth) {
        if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                minDyObsPointNum ||
            !clearDyObs) {
          float disZ =
              point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
          if (considerDrop)
            disZ = fabs(disZ);
          int planarPointElevSize =
              planarPointElev[planarVoxelWidth * indX + indY].size();
          if (disZ >= 0 && disZ < vehicleHeight &&
              planarPointElevSize >= minBlockPointNum) {
            terrainCloudElev->push_back(point);
            terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;
            terrainCloudElevSize++;
          }
        }
      }
    }
  }

  if (noDataObstacle && noDataInited == 2) {
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      int planarPointElevSize = planarPointElev[i].size();
      if (planarPointElevSize < minBlockPointNum) {
        planarVoxelEdge[i] = 1;
      }
    }

    for (int noDataBlockSkipCount = 0;
         noDataBlockSkipCount < noDataBlockSkipNum;
         noDataBlockSkipCount++) {
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        if (planarVoxelEdge[i] >= 1) {
          int indX = static_cast<int>(i / planarVoxelWidth);
          int indY = i % planarVoxelWidth;
          bool edgeVoxel = false;
          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                    dY] < planarVoxelEdge[i]) {
                  edgeVoxel = true;
                }
              }
            }
          }

          if (!edgeVoxel)
            planarVoxelEdge[i]++;
        }
      }
    }

    for (int i = 0; i < kPlanarVoxelNum; i++) {
      if (planarVoxelEdge[i] > noDataBlockSkipNum) {
        int indX = static_cast<int>(i / planarVoxelWidth);
        int indY = i % planarVoxelWidth;

        point.x = planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
        point.y = planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
        point.z = vehicleZ;
        point.intensity = vehicleHeight;

        point.x -= planarVoxelSize / 4.0;
        point.y -= planarVoxelSize / 4.0;
        terrainCloudElev->push_back(point);

        point.x += planarVoxelSize / 2.0;
        terrainCloudElev->push_back(point);

        point.y += planarVoxelSize / 2.0;
        terrainCloudElev->push_back(point);

        point.x -= planarVoxelSize / 2.0;
        terrainCloudElev->push_back(point);
      }
    }
  }

  clearingCloud = false;
}
