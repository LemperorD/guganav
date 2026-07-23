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

#include "rog_map_layer/esdf_incremental.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "rog_map_layer/esdf_config.hpp"
#include "rog_map_layer/esdf_parallel.hpp"

namespace rog_map_layer
{

void IncrementalUpdate::detectChanges(
  const unsigned char * current,
  const unsigned char * previous,
  size_t n,
  std::vector<size_t> & changed_indices,
  EsdfParallelExecutor * executor)
{
  changed_indices.clear();

  if (executor && executor->isParallel()) {
    // 并行版本：每个线程收集自己的变化列表，最后合并
    // 使用 thread-local vector 避免锁竞争
    std::mutex merge_mutex;

    executor->parallelFor(
      n, [&](size_t i) {
        if (current[i] != previous[i]) {
          std::lock_guard<std::mutex> lock(merge_mutex);
          changed_indices.push_back(i);
        }
      });
  } else {
    for (size_t i = 0; i < n; ++i) {
      if (current[i] != previous[i]) {
        changed_indices.push_back(i);
      }
    }
  }
}

void IncrementalUpdate::buildDirtyMask(
  const std::vector<size_t> & changed_indices,
  size_t size_x,
  size_t size_y,
  const EsdfConfig & config,
  std::vector<bool> & dirty_mask,
  size_t & dirty_count,
  EsdfParallelExecutor * executor)
{
  size_t n = size_x * size_y;
  dirty_mask.assign(n, false);
  dirty_count = 0;

  // 计算膨胀半径（以 cells 为单位）
  int radius = static_cast<int>(std::ceil(
      config.max_distance / config.resolution));

  if (executor && executor->isParallel()) {
    std::mutex count_mutex;

    executor->parallelFor(
      changed_indices.size(), [&](size_t i) {
        size_t idx = changed_indices[i];
        int cy = static_cast<int>(idx / size_x);
        int cx = static_cast<int>(idx % size_x);

        int y_min = std::max(0, cy - radius);
        int y_max = std::min(static_cast<int>(size_y) - 1, cy + radius);
        int x_min = std::max(0, cx - radius);
        int x_max = std::min(static_cast<int>(size_x) - 1, cx + radius);

        for (int ny = y_min; ny <= y_max; ++ny) {
          for (int nx = x_min; nx <= x_max; ++nx) {
            size_t nidx = static_cast<size_t>(ny) * size_x +
            static_cast<size_t>(nx);
            if (!dirty_mask[nidx]) {
              dirty_mask[nidx] = true;
              std::lock_guard<std::mutex> lock(count_mutex);
              ++dirty_count;
            }
          }
        }
      });
  } else {
    for (size_t idx : changed_indices) {
      int cy = static_cast<int>(idx / size_x);
      int cx = static_cast<int>(idx % size_x);

      int y_min = std::max(0, cy - radius);
      int y_max = std::min(static_cast<int>(size_y) - 1, cy + radius);
      int x_min = std::max(0, cx - radius);
      int x_max = std::min(static_cast<int>(size_x) - 1, cx + radius);

      for (int ny = y_min; ny <= y_max; ++ny) {
        for (int nx = x_min; nx <= x_max; ++nx) {
          size_t nidx = static_cast<size_t>(ny) * size_x +
            static_cast<size_t>(nx);
          if (!dirty_mask[nidx]) {
            dirty_mask[nidx] = true;
            ++dirty_count;
          }
        }
      }
    }
  }
}

void IncrementalUpdate::resetDirtyOffsets(
  std::vector<int16_t> & offset_dx,
  std::vector<int16_t> & offset_dy,
  const std::vector<bool> & dirty_mask,
  const unsigned char * occupancy,
  unsigned char obstacle_threshold,
  size_t n,
  EsdfParallelExecutor * executor)
{
  if (executor && executor->isParallel()) {
    executor->parallelFor(
      n, [&](size_t i) {
        if (dirty_mask[i]) {
          if (occupancy[i] >= obstacle_threshold) {
            offset_dx[i] = 0;
            offset_dy[i] = 0;
          } else {
            offset_dx[i] = INT16_MAX;
            offset_dy[i] = INT16_MAX;
          }
        }
      });
  } else {
    for (size_t i = 0; i < n; ++i) {
      if (dirty_mask[i]) {
        if (occupancy[i] >= obstacle_threshold) {
          offset_dx[i] = 0;
          offset_dy[i] = 0;
        } else {
          offset_dx[i] = INT16_MAX;
          offset_dy[i] = INT16_MAX;
        }
      }
    }
  }
}

void IncrementalUpdate::resetDirtyDistances(
  std::vector<float> & distance_field,
  const std::vector<bool> & dirty_mask,
  const unsigned char * occupancy,
  unsigned char obstacle_threshold,
  size_t n,
  EsdfParallelExecutor * executor)
{
  if (executor && executor->isParallel()) {
    executor->parallelFor(
      n, [&](size_t i) {
        if (dirty_mask[i]) {
          distance_field[i] = (occupancy[i] >= obstacle_threshold) ? 0.0f : 1e10f;
        }
      });
  } else {
    for (size_t i = 0; i < n; ++i) {
      if (dirty_mask[i]) {
        distance_field[i] = (occupancy[i] >= obstacle_threshold) ? 0.0f : 1e10f;
      }
    }
  }
}

}  // namespace rog_map_layer
