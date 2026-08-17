// Copyright 2026 guganav contributors
//
// terrain_analysis 性能微基准（Google Benchmark）。
//
// 用例：
//   BM_TerrainAnalysis_FullFrame  — 完整帧流（里程计 + 点云 ingest + run），
//                                   模拟车辆直线前进触发体素滚动
//   BM_TerrainAnalysis_RunOnly    — 仅算法管线 run（状态已预填）
//   BM_*_SingleStage              — run 各阶段耗时拆分

#include <benchmark/benchmark.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

#include "terrain_analysis/core/algorithm.hpp"
#include "terrain_analysis/core/config.hpp"
#include "terrain_analysis/core/state.hpp"

namespace {

  // 合成一帧点云（odom 系，vehicle_z=1.5）：
  //   地面网格（z=0，relative_z=-1.5，落在 [-1.5, 0.5] 范围内）
  //   + 随机位置障碍柱（z=0.6）
  //   + 天花板点（z=1.8，relative_z=0.3，用于天花板过滤路径）
  pcl::PointCloud<pcl::PointXYZI>::Ptr makeFrameCloud() {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    constexpr float GRID = 0.4F;  // 网格间距，对应 ~±8m 范围（< max_range 11m）
    for (int ix = -20; ix <= 20; ix++) {
      for (int iy = -20; iy <= 20; iy++) {
        pcl::PointXYZI p;
        p.x = static_cast<float>(ix) * GRID;
        p.y = static_cast<float>(iy) * GRID;
        p.z = 0.0F;
        p.intensity = 0.0F;
        cloud->push_back(p);
      }
    }
    // 障碍柱：车辆前方 2m、左前方 3m、右侧 2.5m
    for (const auto [ox, oy] : {
             std::pair{2.0F, 0.0F},
             {3.0F, 1.5F},
             {0.0F, 2.5F}
    }) {
      for (int dz = 1; dz <= 6; dz++) {
        pcl::PointXYZI p;
        p.x = ox;
        p.y = oy;
        p.z = static_cast<float>(dz) * 0.1F;
        p.intensity = 0.0F;
        cloud->push_back(p);
      }
    }
    // 天花板：车辆正上方一片
    for (int dx = -3; dx <= 3; dx++) {
      for (int dy = -3; dy <= 3; dy++) {
        pcl::PointXYZI p;
        p.x = static_cast<float>(dx) * 0.5F;
        p.y = static_cast<float>(dy) * 0.5F;
        p.z = 1.8F;
        p.intensity = 0.0F;
        cloud->push_back(p);
      }
    }
    return cloud;
  }

  struct BenchContext {
    TerrainConfig config;
    TerrainState state;
    pcl::PointCloud<pcl::PointXYZI>::Ptr frame;
    double time = 1.0;

    BenchContext() {
      config.scan_voxel_size = 0.05;
      config.decay_time = 2.0;
      config.no_decay_distance = 4.0;
      config.clearing_distance = 8.0;
      config.use_sorting = true;
      config.quantile_z = 0.2;
      config.clear_dy_obs = true;
      config.no_data_obstacle = true;
      config.no_data_block_skip_num = 1;
      config.min_block_point_num = 10;
      config.vehicle_height = 0.5;
      config.ceiling_clearance = 0.2;
      config.voxel_point_update_thre = 100;
      config.voxel_time_update_thre = 1.0;
      config.min_relative_z = -1.5;
      config.max_relative_z = 0.5;
      config.distance_ratio_z = 0.2;
      frame = makeFrameCloud();
      // 首帧 odom 与点云，使 system_inited 生效
      terrain_analysis::algorithm::ingestOdometry(config, state, 0.0, 0.0, 1.5,
                                                  0.0, 0.0, 0.0);
      terrain_analysis::algorithm::ingestLaserCloud(config, state, frame, time);
      terrain_analysis::algorithm::run(config, state);
      time += 0.1;
    }
  };

  // 一帧完整处理：odom 前进（触发体素滚动）+ 新点云 + run
  void BM_TerrainAnalysis_FullFrame(benchmark::State& state) {
    BenchContext ctx;
    for (auto _ : state) {
      ctx.time += 0.1;
      terrain_analysis::algorithm::ingestOdometry(ctx.config, ctx.state,
                                                  ctx.state.vehicle_x + 0.2,
                                                  0.0, 1.5, 0.0, 0.0, 0.0);
      terrain_analysis::algorithm::ingestLaserCloud(ctx.config, ctx.state,
                                                    ctx.frame, ctx.time);
      terrain_analysis::algorithm::run(ctx.config, ctx.state);
    }
  }

  // 仅算法管线（状态预填后重复 run）
  void BM_TerrainAnalysis_RunOnly(benchmark::State& state) {
    BenchContext ctx;
    for (auto _ : state) {
      terrain_analysis::algorithm::run(ctx.config, ctx.state);
    }
  }

  void BM_TerrainAnalysis_Voxelize(benchmark::State& state) {
    BenchContext ctx;
    for (auto _ : state) {
      terrain_analysis::algorithm::voxelize(ctx.config, ctx.state);
    }
  }

  void BM_TerrainAnalysis_EstimateGround(benchmark::State& state) {
    BenchContext ctx;
    for (auto _ : state) {
      terrain_analysis::algorithm::estimateGround(ctx.config, ctx.state);
    }
  }

  void BM_TerrainAnalysis_ComputeHeightMap(benchmark::State& state) {
    BenchContext ctx;
    for (auto _ : state) {
      terrain_analysis::algorithm::computeHeightMap(ctx.config, ctx.state);
    }
  }

  void BM_TerrainAnalysis_ExtractTerrainCloud(benchmark::State& state) {
    BenchContext ctx;
    for (auto _ : state) {
      terrain_analysis::algorithm::extractTerrainCloud(ctx.state);
    }
  }

}  // namespace

BENCHMARK(BM_TerrainAnalysis_FullFrame)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_TerrainAnalysis_RunOnly)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_TerrainAnalysis_Voxelize)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_TerrainAnalysis_EstimateGround)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_TerrainAnalysis_ComputeHeightMap)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_TerrainAnalysis_ExtractTerrainCloud)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
