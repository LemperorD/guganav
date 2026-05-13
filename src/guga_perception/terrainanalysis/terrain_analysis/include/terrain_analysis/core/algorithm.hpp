#pragma once

class TerrainAnalysisContext;

namespace TerrainAlgorithm {

// ── Pipeline ──
void run(TerrainAnalysisContext& ctx);

// ── Stages (exposed for testing) ──
void rolloverTerrainVoxels(TerrainAnalysisContext& ctx);
void stackLaserScans(TerrainAnalysisContext& ctx);
void updateVoxels(TerrainAnalysisContext& ctx);
void extractTerrainCloud(TerrainAnalysisContext& ctx);
void estimateGround(TerrainAnalysisContext& ctx);
void detectDynamicObstacles(TerrainAnalysisContext& ctx);
void filterDynamicObstaclePoints(TerrainAnalysisContext& ctx, int laser_cloud_crop_size);
void computeElevation(TerrainAnalysisContext& ctx);
void computeHeightMap(TerrainAnalysisContext& ctx, int terrain_cloud_size);
void addNoDataObstacles(TerrainAnalysisContext& ctx);

}  // namespace TerrainAlgorithm
