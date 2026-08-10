#pragma once

#include <cstddef>

namespace rog_map_layer
{

struct EsdfConfig
{
  // ESDF 距离参数
  double max_distance{2.0};              // ESDF 最大计算距离 (m)
  unsigned char obstacle_threshold{254}; // 障碍物判定阈值 (LETHAL_OBSTACLE)

  // 更新策略
  bool incremental_update{true};         // 是否启用增量更新
  double full_update_ratio{0.3};        // 变化比例超此值则全量重算

  // TBB 并行参数
  bool enable_parallel{true};            // 是否启用多线程
  int num_threads{3};                   // 线程数（默认 核心数-1）
  int tile_size{32};                     // 分块并行 tile 边长 (cells)
  int seam_iterations{2};               // 接缝修正迭代次数

  // 输出控制
  bool publish_esdf_grid{false};         // 是否发布 ESDF 可视化 OccupancyGrid
  bool write_to_master{true};           // 是否将 ESDF 代价写入 master grid

  // 从 costmap 继承（运行时填充）
  double resolution{};                   // 分辨率 (m/cell)
  size_t size_x{};                       // 网格宽度 (cells)
  size_t size_y{};                       // 网格高度 (cells)
};

}  // namespace rog_map_layer
