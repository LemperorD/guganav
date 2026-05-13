#pragma once

struct TerrainConfig;
struct TerrainState;

class TerrainAlgorithm {
public:
  static void run(const TerrainConfig& config, TerrainState& state);

  static void rolloverTerrainVoxels(const TerrainConfig& config,
                                    TerrainState& state);
  static void stackLaserScans(const TerrainConfig& config, TerrainState& state);
  static void updateVoxels(const TerrainConfig& config, TerrainState& state);
  static void extractTerrainCloud(const TerrainConfig& config,
                                  TerrainState& state);
  static void estimateGround(const TerrainConfig& config, TerrainState& state);
  static void detectDynamicObstacles(const TerrainConfig& config,
                                     TerrainState& state);
  static void filterDynamicObstaclePoints(const TerrainConfig& config,
                                          TerrainState& state,
                                          int laser_cloud_crop_size);
  static void computeElevation(const TerrainConfig& config,
                               TerrainState& state);
  static void computeHeightMap(const TerrainConfig& config, TerrainState& state,
                               int terrain_cloud_size);
  static void addNoDataObstacles(const TerrainConfig& config,
                                 TerrainState& state);
};
