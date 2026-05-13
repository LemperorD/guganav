#pragma once

struct TerrainConfig;
struct TerrainState;

class TerrainAlgorithm {
public:
  static void run(const TerrainConfig& cfg, TerrainState& st);

  static void rolloverTerrainVoxels(const TerrainConfig& cfg, TerrainState& st);
  static void stackLaserScans(const TerrainConfig& cfg, TerrainState& st);
  static void updateVoxels(const TerrainConfig& cfg, TerrainState& st);
  static void extractTerrainCloud(const TerrainConfig& cfg, TerrainState& st);
  static void estimateGround(const TerrainConfig& cfg, TerrainState& st);
  static void detectDynamicObstacles(const TerrainConfig& cfg, TerrainState& st);
  static void filterDynamicObstaclePoints(const TerrainConfig& cfg, TerrainState& st,
                                          int laser_cloud_crop_size);
  static void computeElevation(const TerrainConfig& cfg, TerrainState& st);
  static void computeHeightMap(const TerrainConfig& cfg, TerrainState& st,
                               int terrain_cloud_size);
  static void addNoDataObstacles(const TerrainConfig& cfg, TerrainState& st);
};
