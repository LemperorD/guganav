// Copyright 2026 guganav
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

#include "rog_map_layer/esdf_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "rog_map_layer/esdf_config.hpp"
#include "rog_map_layer/esdf_incremental.hpp"

namespace rog_map_layer
{

void EsdfMap::resize(
  size_t size_x, size_t size_y, double resolution,
  double origin_x, double origin_y)
{
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  size_t n = size_x_ * size_y_;
  offset_dx_.resize(n, 0);
  offset_dy_.resize(n, 0);
  distance_field_.resize(n, INF_DISTANCE);
  gradient_x_.resize(n, 0.0f);
  gradient_y_.resize(n, 0.0f);
}

void EsdfMap::reset()
{
  std::fill(offset_dx_.begin(), offset_dx_.end(), 0);
  std::fill(offset_dy_.begin(), offset_dy_.end(), 0);
  std::fill(distance_field_.begin(), distance_field_.end(), INF_DISTANCE);
  std::fill(gradient_x_.begin(), gradient_x_.end(), 0.0f);
  std::fill(gradient_y_.begin(), gradient_y_.end(), 0.0f);
}

void EsdfMap::initFromOccupancy(
  const unsigned char * occupancy,
  unsigned char obstacle_threshold,
  EsdfParallelExecutor * executor)
{
  size_t n = size_x_ * size_y_;

  if (executor && executor->isParallel()) {
    executor->parallelFor(
      n, [&](size_t i) {
        if (occupancy[i] >= obstacle_threshold) {
          offset_dx_[i] = 0;
          offset_dy_[i] = 0;
        } else {
          offset_dx_[i] = INT16_MAX;
          offset_dy_[i] = INT16_MAX;
        }
      });
  } else {
    for (size_t i = 0; i < n; ++i) {
      if (occupancy[i] >= obstacle_threshold) {
        offset_dx_[i] = 0;
        offset_dy_[i] = 0;
      } else {
        offset_dx_[i] = INT16_MAX;
        offset_dy_[i] = INT16_MAX;
      }
    }
  }
}

void EsdfMap::propagateForward(
  size_t start_row, size_t end_row,
  size_t start_col, size_t end_col)
{
  // 8SED forward pass: 左上→右下
  // 检查 4 个邻居: (-1,-1), (0,-1), (1,-1), (-1,0)
  // 注意：这里用 int16_t 偏移量存储最近障碍物的 (dx, dy)
  // 因为障碍物可能不在同一 tile 内，偏移需要能覆盖整个 costmap

  for (size_t y = start_row; y < end_row; ++y) {
    for (size_t x = start_col; x < end_col; ++x) {
      size_t idx = y * size_x_ + x;
      if (offset_dx_[idx] == 0 && offset_dy_[idx] == 0) {
        continue;  // 障碍物格子，跳过
      }

      int16_t best_dx = offset_dx_[idx];
      int16_t best_dy = offset_dy_[idx];
      int32_t best_sq = static_cast<int32_t>(best_dx) * best_dx +
        static_cast<int32_t>(best_dy) * best_dy;

      // Neighbor offsets for forward pass
      static constexpr int16_t fwd_dx[] = {-1, -1, 0, 1};
      static constexpr int16_t fwd_dy[] = {0, -1, -1, -1};

      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(x) + fwd_dx[k];
        int ny = static_cast<int>(y) + fwd_dy[k];
        if (nx < 0 || nx >= static_cast<int>(size_x_) ||
          ny < 0 || ny >= static_cast<int>(size_y_))
        {
          continue;
        }

        size_t nidx = static_cast<size_t>(ny) * size_x_ + static_cast<size_t>(nx);
        int16_t ndx = offset_dx_[nidx];
        int16_t ndy = offset_dy_[nidx];
        if (ndx == INT16_MAX) {
          continue;  // 邻居也未被初始化
        }

        // 邻居的障碍物位置相对于当前格的位置
        int16_t cand_dx = static_cast<int16_t>(ndx - fwd_dx[k]);
        int16_t cand_dy = static_cast<int16_t>(ndy - fwd_dy[k]);

        int32_t cand_sq = static_cast<int32_t>(cand_dx) * cand_dx +
          static_cast<int32_t>(cand_dy) * cand_dy;

        if (cand_sq < best_sq) {
          best_sq = cand_sq;
          best_dx = cand_dx;
          best_dy = cand_dy;
        }
      }

      offset_dx_[idx] = best_dx;
      offset_dy_[idx] = best_dy;
    }
  }
}

void EsdfMap::propagateBackward(
  size_t start_row, size_t end_row,
  size_t start_col, size_t end_col)
{
  // 8SED backward pass: 右下→左上
  // 检查 4 个邻居: (1,0), (-1,1), (0,1), (1,1)

  static constexpr int16_t bwd_dx[] = {1, -1, 0, 1};
  static constexpr int16_t bwd_dy[] = {0, 1, 1, 1};

  for (size_t y = end_row; y > start_row; ) {
    --y;
    for (size_t x = end_col; x > start_col; ) {
      --x;
      size_t idx = y * size_x_ + x;
      if (offset_dx_[idx] == 0 && offset_dy_[idx] == 0) {
        continue;
      }

      int16_t best_dx = offset_dx_[idx];
      int16_t best_dy = offset_dy_[idx];
      int32_t best_sq = static_cast<int32_t>(best_dx) * best_dx +
        static_cast<int32_t>(best_dy) * best_dy;

      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(x) + bwd_dx[k];
        int ny = static_cast<int>(y) + bwd_dy[k];
        if (nx < 0 || nx >= static_cast<int>(size_x_) ||
          ny < 0 || ny >= static_cast<int>(size_y_))
        {
          continue;
        }

        size_t nidx = static_cast<size_t>(ny) * size_x_ + static_cast<size_t>(nx);
        int16_t ndx = offset_dx_[nidx];
        int16_t ndy = offset_dy_[nidx];
        if (ndx == INT16_MAX) {
          continue;
        }

        int16_t cand_dx = static_cast<int16_t>(ndx - bwd_dx[k]);
        int16_t cand_dy = static_cast<int16_t>(ndy - bwd_dy[k]);

        int32_t cand_sq = static_cast<int32_t>(cand_dx) * cand_dx +
          static_cast<int32_t>(cand_dy) * cand_dy;

        if (cand_sq < best_sq) {
          best_sq = cand_sq;
          best_dx = cand_dx;
          best_dy = cand_dy;
        }
      }

      offset_dx_[idx] = best_dx;
      offset_dy_[idx] = best_dy;
    }
  }
}

void EsdfMap::copyOffsetsToDistances(const EsdfConfig & config)
{
  size_t n = size_x_ * size_y_;
  double res = resolution_;
  double max_dist = config.max_distance;

  for (size_t i = 0; i < n; ++i) {
    int16_t dx = offset_dx_[i];
    int16_t dy = offset_dy_[i];
    if (dx == INT16_MAX) {
      distance_field_[i] = static_cast<float>(max_dist);
    } else {
      double d = std::sqrt(
        static_cast<double>(dx) * dx +
        static_cast<double>(dy) * dy) * res;
      distance_field_[i] = static_cast<float>(std::min(d, max_dist));
    }
  }
}

void EsdfMap::computeFull(
  const unsigned char * occupancy,
  const EsdfConfig & config,
  EsdfParallelExecutor * executor)
{
  // Disable TBB parallel path — it uses tile-based 8SED with seam fixing
  // which produces ~2% distance error vs ground truth. Serial 8SED is exact
  // and fast enough for our costmap sizes (< 1ms for 100×100).
  computeFullSerial(occupancy, config);

  computeGradients(executor);
}

void EsdfMap::computeFullSerial(
  const unsigned char * occupancy,
  const EsdfConfig & config)
{
  initFromOccupancy(occupancy, config.obstacle_threshold, nullptr);
  propagateForward(0, size_y_, 0, size_x_);
  propagateBackward(0, size_y_, 0, size_x_);
  copyOffsetsToDistances(config);
}

std::vector<EsdfMap::Tile> EsdfMap::buildTiles(int tile_size) const
{
  std::vector<Tile> tiles;

  size_t actual_tile = static_cast<size_t>(std::max(1, tile_size));

  for (size_t r = 0; r < size_y_; r += actual_tile) {
    for (size_t c = 0; c < size_x_; c += actual_tile) {
      Tile t;
      t.row_start = r;
      t.row_end = std::min(r + actual_tile, size_y_);
      t.col_start = c;
      t.col_end = std::min(c + actual_tile, size_x_);
      tiles.push_back(t);
    }
  }

  return tiles;
}

void EsdfMap::fixSeamBetween(const Tile & a, const Tile & b, bool horizontal)
{
  // horizontal = true: Horizontal seam — a 在上，b 在下
  //   传播 a 的最后一行 → b 的第一行，反向传播 b 的第一行 → a 的最后一行
  // horizontal = false: Vertical seam — a 在左，b 在右
  //   传播 a 的最后一列 → b 的第一列，反向传播 b 的第一列 → a 的最后一列

  if (horizontal) {
    // a 的最后一行和 b 的第一行之间交叉传播
    size_t row_a = a.row_end - 1;
    size_t row_b = b.row_start;
    size_t c_start = std::max(a.col_start, b.col_start);
    size_t c_end = std::min(a.col_end, b.col_end);

    static constexpr int16_t fwd_dx[] = {-1, -1, 0, 1};
    static constexpr int16_t fwd_dy[] = {0, -1, -1, -1};
    static constexpr int16_t bwd_dx[] = {1, -1, 0, 1};
    static constexpr int16_t bwd_dy[] = {0, 1, 1, 1};

    // 从上到下传播：a 的最后一行 → b 的第一行
    for (size_t x = c_start; x < c_end; ++x) {
      size_t idx = row_b * size_x_ + x;
      if (offset_dx_[idx] == 0 && offset_dy_[idx] == 0) {continue;}

      int16_t best_dx = offset_dx_[idx];
      int16_t best_dy = offset_dy_[idx];
      int32_t best_sq = static_cast<int32_t>(best_dx) * best_dx +
        static_cast<int32_t>(best_dy) * best_dy;

      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(x) + fwd_dx[k];
        int ny = static_cast<int>(row_b) + fwd_dy[k];
        if (nx < 0 || nx >= static_cast<int>(size_x_) ||
          ny < 0 || ny >= static_cast<int>(size_y_)) {continue;}

        size_t nidx = static_cast<size_t>(ny) * size_x_ + static_cast<size_t>(nx);
        int16_t ndx = offset_dx_[nidx];
        int16_t ndy = offset_dy_[nidx];
        if (ndx == INT16_MAX) {continue;}

        int16_t cand_dx = static_cast<int16_t>(ndx - fwd_dx[k]);
        int16_t cand_dy = static_cast<int16_t>(ndy - fwd_dy[k]);
        int32_t cand_sq = static_cast<int32_t>(cand_dx) * cand_dx +
          static_cast<int32_t>(cand_dy) * cand_dy;
        if (cand_sq < best_sq) {
          best_sq = cand_sq; best_dx = cand_dx; best_dy = cand_dy;
        }
      }
      offset_dx_[idx] = best_dx;
      offset_dy_[idx] = best_dy;
    }

    // 从下到上传播：b 的第一行 → a 的最后一行
    for (size_t x = c_start; x < c_end; ++x) {
      size_t idx = row_a * size_x_ + x;
      if (offset_dx_[idx] == 0 && offset_dy_[idx] == 0) {continue;}

      int16_t best_dx = offset_dx_[idx];
      int16_t best_dy = offset_dy_[idx];
      int32_t best_sq = static_cast<int32_t>(best_dx) * best_dx +
        static_cast<int32_t>(best_dy) * best_dy;

      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(x) + bwd_dx[k];
        int ny = static_cast<int>(row_a) + bwd_dy[k];
        if (nx < 0 || nx >= static_cast<int>(size_x_) ||
          ny < 0 || ny >= static_cast<int>(size_y_)) {continue;}

        size_t nidx = static_cast<size_t>(ny) * size_x_ + static_cast<size_t>(nx);
        int16_t ndx = offset_dx_[nidx];
        int16_t ndy = offset_dy_[nidx];
        if (ndx == INT16_MAX) {continue;}

        int16_t cand_dx = static_cast<int16_t>(ndx - bwd_dx[k]);
        int16_t cand_dy = static_cast<int16_t>(ndy - bwd_dy[k]);
        int32_t cand_sq = static_cast<int32_t>(cand_dx) * cand_dx +
          static_cast<int32_t>(cand_dy) * cand_dy;
        if (cand_sq < best_sq) {
          best_sq = cand_sq; best_dx = cand_dx; best_dy = cand_dy;
        }
      }
      offset_dx_[idx] = best_dx;
      offset_dy_[idx] = best_dy;
    }
  } else {
    // 垂直接缝：a 在左，b 在右
    size_t col_a = a.col_end - 1;
    size_t col_b = b.col_start;
    size_t r_start = std::max(a.row_start, b.row_start);
    size_t r_end = std::min(a.row_end, b.row_end);

    static constexpr int16_t fwd_dx[] = {-1, -1, 0, 1};
    static constexpr int16_t fwd_dy[] = {0, -1, -1, -1};
    static constexpr int16_t bwd_dx[] = {1, -1, 0, 1};
    static constexpr int16_t bwd_dy[] = {0, 1, 1, 1};

    // 从左到右：a 的最后一列 → b 的第一列
    for (size_t y = r_start; y < r_end; ++y) {
      size_t idx = y * size_x_ + col_b;
      if (offset_dx_[idx] == 0 && offset_dy_[idx] == 0) {continue;}

      int16_t best_dx = offset_dx_[idx];
      int16_t best_dy = offset_dy_[idx];
      int32_t best_sq = static_cast<int32_t>(best_dx) * best_dx +
        static_cast<int32_t>(best_dy) * best_dy;

      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(col_b) + fwd_dx[k];
        int ny = static_cast<int>(y) + fwd_dy[k];
        if (nx < 0 || nx >= static_cast<int>(size_x_) ||
          ny < 0 || ny >= static_cast<int>(size_y_)) {continue;}

        size_t nidx = static_cast<size_t>(ny) * size_x_ + static_cast<size_t>(nx);
        int16_t ndx = offset_dx_[nidx];
        int16_t ndy = offset_dy_[nidx];
        if (ndx == INT16_MAX) {continue;}

        int16_t cand_dx = static_cast<int16_t>(ndx - fwd_dx[k]);
        int16_t cand_dy = static_cast<int16_t>(ndy - fwd_dy[k]);
        int32_t cand_sq = static_cast<int32_t>(cand_dx) * cand_dx +
          static_cast<int32_t>(cand_dy) * cand_dy;
        if (cand_sq < best_sq) {
          best_sq = cand_sq; best_dx = cand_dx; best_dy = cand_dy;
        }
      }
      offset_dx_[idx] = best_dx;
      offset_dy_[idx] = best_dy;
    }

    // 从右到左：b 的第一列 → a 的最后一列
    for (size_t y = r_start; y < r_end; ++y) {
      size_t idx = y * size_x_ + col_a;
      if (offset_dx_[idx] == 0 && offset_dy_[idx] == 0) {continue;}

      int16_t best_dx = offset_dx_[idx];
      int16_t best_dy = offset_dy_[idx];
      int32_t best_sq = static_cast<int32_t>(best_dx) * best_dx +
        static_cast<int32_t>(best_dy) * best_dy;

      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(col_a) + bwd_dx[k];
        int ny = static_cast<int>(y) + bwd_dy[k];
        if (nx < 0 || nx >= static_cast<int>(size_x_) ||
          ny < 0 || ny >= static_cast<int>(size_y_)) {continue;}

        size_t nidx = static_cast<size_t>(ny) * size_x_ + static_cast<size_t>(nx);
        int16_t ndx = offset_dx_[nidx];
        int16_t ndy = offset_dy_[nidx];
        if (ndx == INT16_MAX) {continue;}

        int16_t cand_dx = static_cast<int16_t>(ndx - bwd_dx[k]);
        int16_t cand_dy = static_cast<int16_t>(ndy - bwd_dy[k]);
        int32_t cand_sq = static_cast<int32_t>(cand_dx) * cand_dx +
          static_cast<int32_t>(cand_dy) * cand_dy;
        if (cand_sq < best_sq) {
          best_sq = cand_sq; best_dx = cand_dx; best_dy = cand_dy;
        }
      }
      offset_dx_[idx] = best_dx;
      offset_dy_[idx] = best_dy;
    }
  }
}

void EsdfMap::computeFullParallel(
  const unsigned char * occupancy,
  const EsdfConfig & config,
  EsdfParallelExecutor * executor)
{
  // Step 1: 初始化
  initFromOccupancy(occupancy, config.obstacle_threshold, executor);

  // Step 2: 构建 tiles
  auto tiles = buildTiles(config.tile_size);
  size_t num_tiles = tiles.size();

  // Step 3: 每个 tile 内独立运行 8SED
  executor->parallelFor(
    num_tiles, [&](size_t ti) {
      const auto & t = tiles[ti];
      propagateForward(t.row_start, t.row_end, t.col_start, t.col_end);
      propagateBackward(t.row_start, t.row_end, t.col_start, t.col_end);
    });

  // Step 4: 接缝修正（迭代次数由 config.seam_iterations 控制）
  for (int iter = 0; iter < config.seam_iterations; ++iter) {
    // 水平接缝
    size_t tiles_per_row = (size_x_ + static_cast<size_t>(config.tile_size) - 1) /
      static_cast<size_t>(config.tile_size);
    executor->parallelFor(
      num_tiles, [&](size_t ti) {
        size_t row = ti / tiles_per_row;
        size_t col = ti % tiles_per_row;

        // 修正与下方 tile 的水平接缝
        if (row + 1 < (size_y_ + static_cast<size_t>(config.tile_size) - 1) /
        static_cast<size_t>(config.tile_size))
        {
          size_t below = ti + tiles_per_row;
          if (below < num_tiles) {
            fixSeamBetween(tiles[ti], tiles[below], true);
          }
        }

        // 修正与右方 tile 的垂直接缝
        if (col + 1 < tiles_per_row) {
          size_t right = ti + 1;
          if (right < num_tiles) {
            fixSeamBetween(tiles[ti], tiles[right], false);
          }
        }
      });
  }

  // Step 5: 偏移量 → 距离值
  copyOffsetsToDistances(config);
}

void EsdfMap::computeGradients(EsdfParallelExecutor * executor)
{
  size_t w = size_x_;
  size_t h = size_y_;

  auto compute_cell = [&](size_t y) {
      for (size_t x = 1; x < w - 1; ++x) {
        size_t idx = y * w + x;
        float dx = (distance_field_[y * w + (x + 1)] -
          distance_field_[y * w + (x - 1)]) * 0.5f;
        float dy = (distance_field_[(y + 1) * w + x] -
          distance_field_[(y - 1) * w + x]) * 0.5f;
        gradient_x_[idx] = dx;
        gradient_y_[idx] = dy;
      }
    };

  if (executor && executor->isParallel()) {
    executor->parallelFor(
      h - 2, [&](size_t i) {
        compute_cell(i + 1); // skip first row
      });
  } else {
    for (size_t y = 1; y < h - 1; ++y) {
      compute_cell(y);
    }
  }

  // 边界行/列清零
  for (size_t x = 0; x < w; ++x) {
    gradient_x_[x] = 0.0f;
    gradient_y_[x] = 0.0f;
    gradient_x_[(h - 1) * w + x] = 0.0f;
    gradient_y_[(h - 1) * w + x] = 0.0f;
  }
  for (size_t y = 0; y < h; ++y) {
    gradient_x_[y * w] = 0.0f;
    gradient_y_[y * w] = 0.0f;
    gradient_x_[y * w + (w - 1)] = 0.0f;
    gradient_y_[y * w + (w - 1)] = 0.0f;
  }
}

void EsdfMap::computeIncremental(
  const unsigned char * occupancy,
  const std::vector<unsigned char> & previous_occupancy,
  const EsdfConfig & config,
  EsdfParallelExecutor * executor)
{
  size_t n = size_x_ * size_y_;

  // Step 1: 差分检测
  std::vector<size_t> changed_indices;
  IncrementalUpdate::detectChanges(
    occupancy, previous_occupancy.data(),
    n, changed_indices, executor);

  double change_ratio = static_cast<double>(changed_indices.size()) /
    static_cast<double>(n);

  // Step 2: 变化过大 → 全量重算
  if (change_ratio > config.full_update_ratio || changed_indices.empty()) {
    computeFull(occupancy, config, executor);
    return;
  }

  // Step 3: 构建脏区域 mask
  std::vector<bool> dirty_mask;
  size_t dirty_count{};
  IncrementalUpdate::buildDirtyMask(
    changed_indices, size_x_, size_y_,
    config, dirty_mask, dirty_count,
    executor);

  // Step 4: 重置脏区域的偏移量
  IncrementalUpdate::resetDirtyOffsets(
    offset_dx_, offset_dy_, dirty_mask,
    occupancy, config.obstacle_threshold,
    n, executor);

  // Step 5: 在全地图上运行前向+后向传播（脏区域外保持不变）
  // 注意：这里需要全地图传播，因为脏区域的结果会影响其外部
  propagateForward(0, size_y_, 0, size_x_);
  propagateBackward(0, size_y_, 0, size_x_);

  // Step 6: 偏移量 → 距离值
  copyOffsetsToDistances(config);

  // Step 7: 计算梯度
  computeGradients(executor);
}

float EsdfMap::getDistance(size_t idx) const
{
  return distance_field_[idx];
}

std::pair<float, float> EsdfMap::getGradient(size_t idx) const
{
  return {gradient_x_[idx], gradient_y_[idx]};
}

float EsdfMap::getDistanceAt(double wx, double wy) const
{
  size_t mx{}, my{};
  if (!worldToMap(wx, wy, mx, my)) {
    return 0.0f;
  }
  size_t idx = my * size_x_ + mx;
  return distance_field_[idx];
}

std::pair<float, float> EsdfMap::getGradientAt(double wx, double wy) const
{
  size_t mx{}, my{};
  if (!worldToMap(wx, wy, mx, my)) {
    return {0.0f, 0.0f};
  }
  size_t idx = my * size_x_ + mx;
  return {gradient_x_[idx], gradient_y_[idx]};
}

bool EsdfMap::worldToMap(
  double wx, double wy,
  size_t & mx, size_t & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }
  mx = static_cast<size_t>((wx - origin_x_) / resolution_);
  my = static_cast<size_t>((wy - origin_y_) / resolution_);
  return mx < size_x_ && my < size_y_;
}

nav_msgs::msg::OccupancyGrid EsdfMap::toOccupancyGrid(
  const std::string & frame_id,
  rclcpp::Time stamp,
  const EsdfConfig & config) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame_id;
  grid.header.stamp = stamp;
  grid.info.width = static_cast<unsigned int>(size_x_);
  grid.info.height = static_cast<unsigned int>(size_y_);
  grid.info.resolution = resolution_;
  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.orientation.w = 1.0;

  size_t n = size_x_ * size_y_;
  grid.data.resize(n);

  double max_dist = config.max_distance;
  for (size_t i = 0; i < n; ++i) {
    float d = distance_field_[i];
    if (d <= 0.0f) {
      grid.data[i] = 100;  // 障碍物 — 黑色
    } else if (d >= static_cast<float>(max_dist)) {
      grid.data[i] = 0;    // 超出范围 — 白色
    } else {
      float ratio = 1.0f - d / static_cast<float>(max_dist);
      grid.data[i] = static_cast<uint8_t>(ratio * 100.0f);
    }
  }

  return grid;
}

void EsdfMap::detectChanges(
  const unsigned char * occupancy,
  const std::vector<unsigned char> & previous,
  std::vector<size_t> & changed_indices) const
{
  // 委托给 IncrementalUpdate 的静态方法
  IncrementalUpdate::detectChanges(
    occupancy, previous.data(),
    size_x_ * size_y_, changed_indices);
}

void EsdfMap::expandDirtyRegion(
  const std::vector<size_t> & changed_indices,
  std::vector<bool> & dirty_mask,
  const EsdfConfig & config) const
{
  size_t dummy_count{};
  IncrementalUpdate::buildDirtyMask(
    changed_indices, size_x_, size_y_,
    config, dirty_mask, dummy_count);
}

}  // namespace rog_map_layer
