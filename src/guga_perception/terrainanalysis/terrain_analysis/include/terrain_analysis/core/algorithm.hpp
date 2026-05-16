#pragma once
#include <cstddef>

struct TerrainConfig;
struct TerrainState;

class TerrainAlgorithm {
public:
  static void run(const TerrainConfig& config, TerrainState& state);

  static void rolloverVoxels(const TerrainConfig& config, TerrainState& state);
  static void voxelize(const TerrainConfig& config, TerrainState& state);
  static void updateVoxels(const TerrainConfig& config, TerrainState& state);
  static void extractTerrainCloud(TerrainState& state);
  static void estimateGround(const TerrainConfig& config, TerrainState& state);
  static void detectDynamicObstacles(const TerrainConfig& config,
                                     TerrainState& state);
  static void filterDynamicObstaclePoints(const TerrainConfig& config,
                                          TerrainState& state);
  static void computeElevation(const TerrainConfig& config,
                               TerrainState& state);
  static void computeHeightMap(const TerrainConfig& config, TerrainState& state,
                               size_t terrain_cloud_size);
  static void addNoDataObstacles(const TerrainConfig& config,
                                 TerrainState& state);
};
