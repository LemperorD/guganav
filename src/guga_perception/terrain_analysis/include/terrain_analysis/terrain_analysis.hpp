#pragma once

#include <math.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <vector>

// ── compile-time constants ──

const int terrainVoxelWidth = 21;
constexpr int kTerrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

const int planarVoxelWidth = 51;
constexpr int kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;

// ── run-time globals ──

extern double scanVoxelSize;
extern double decayTime;
extern double noDecayDis;
extern double clearingDis;
extern bool clearingCloud;
extern bool useSorting;
extern double quantileZ;
extern bool considerDrop;
extern bool limitGroundLift;
extern double maxGroundLift;
extern bool clearDyObs;
extern double minDyObsDis;
extern double minDyObsAngle;
extern double minDyObsRelZ;
extern double absDyObsRelZThre;
extern double minDyObsVFOV;
extern double maxDyObsVFOV;
extern int minDyObsPointNum;
extern bool noDataObstacle;
extern int noDataBlockSkipNum;
extern int minBlockPointNum;
extern double vehicleHeight;
extern int voxelPointUpdateThre;
extern double voxelTimeUpdateThre;
extern double minRelZ;
extern double maxRelZ;
extern double disRatioZ;

// terrain voxel
extern float terrainVoxelSize;
extern int terrainVoxelShiftX;
extern int terrainVoxelShiftY;
extern int terrainVoxelHalfWidth;

// planar voxel
extern float planarVoxelSize;
extern int planarVoxelHalfWidth;

// point clouds
extern pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev;
extern pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[kTerrainVoxelNum];

extern int terrainVoxelUpdateNum[kTerrainVoxelNum];
extern float terrainVoxelUpdateTime[kTerrainVoxelNum];
extern float planarVoxelElev[kPlanarVoxelNum];
extern int planarVoxelEdge[kPlanarVoxelNum];
extern int planarVoxelDyObs[kPlanarVoxelNum];
extern std::vector<float> planarPointElev[kPlanarVoxelNum];

extern double laserCloudTime;
extern bool newlaserCloud;

extern double systemInitTime;
extern bool systemInited;
extern int noDataInited;

// vehicle pose
extern float vehicleRoll, vehiclePitch, vehicleYaw;
extern float vehicleX, vehicleY, vehicleZ;
extern float vehicleXRec, vehicleYRec;
extern float sinVehicleRoll, cosVehicleRoll;
extern float sinVehiclePitch, cosVehiclePitch;
extern float sinVehicleYaw, cosVehicleYaw;

extern pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

// ── callback / function declarations ──

void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
void laserCloudHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2);
void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy);
void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis);

void processTerrainData();
