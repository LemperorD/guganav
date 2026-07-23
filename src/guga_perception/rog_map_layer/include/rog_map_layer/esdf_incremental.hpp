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

#pragma once

#include <cstddef>
#include <vector>

#include "rog_map_layer/esdf_parallel.hpp"

namespace rog_map_layer
{

struct EsdfConfig;

// 增量更新辅助工具，全部为静态方法，采用模式 A（函数式数据流）
struct IncrementalUpdate
{
  /// @brief 并行差分检测：current vs previous occupancy，找出所有变化的 cell
  static void detectChanges(
    const unsigned char * current,
    const unsigned char * previous,
    size_t n,
    std::vector<size_t> & changed_indices,
    EsdfParallelExecutor * executor = nullptr);

  /// @brief 将变化格膨胀为脏区域 mask（并行标记邻居）
  static void buildDirtyMask(
    const std::vector<size_t> & changed_indices,
    size_t size_x,
    size_t size_y,
    const EsdfConfig & config,
    std::vector<bool> & dirty_mask,
    size_t & dirty_count,
    EsdfParallelExecutor * executor = nullptr);

  /// @brief 重置脏区域内格子的偏移量为初始状态
  static void resetDirtyOffsets(
    std::vector<int16_t> & offset_dx,
    std::vector<int16_t> & offset_dy,
    const std::vector<bool> & dirty_mask,
    const unsigned char * occupancy,
    unsigned char obstacle_threshold,
    size_t n,
    EsdfParallelExecutor * executor = nullptr);

  /// @brief 重置脏区域内格子的距离值为初始状态
  static void resetDirtyDistances(
    std::vector<float> & distance_field,
    const std::vector<bool> & dirty_mask,
    const unsigned char * occupancy,
    unsigned char obstacle_threshold,
    size_t n,
    EsdfParallelExecutor * executor = nullptr);
};

}  // namespace rog_map_layer
