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

#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <cstddef>

namespace rog_map_layer
{

class EsdfParallelExecutor
{
public:
  explicit EsdfParallelExecutor(int num_threads);

  ~EsdfParallelExecutor() = default;

  EsdfParallelExecutor(const EsdfParallelExecutor&) = delete;
  EsdfParallelExecutor& operator=(const EsdfParallelExecutor&) = delete;
  EsdfParallelExecutor(EsdfParallelExecutor&&) = default;
  EsdfParallelExecutor& operator=(EsdfParallelExecutor&&) = default;

  template <typename Func>
  void parallelFor(size_t n, Func&& f)
  {
    arena_.execute([&] {
      tbb::parallel_for(
        tbb::blocked_range<size_t>(0, n),
        [&](const tbb::blocked_range<size_t>& r) {
          for (size_t i = r.begin(); i != r.end(); ++i) {
            f(i);
          }
        });
    });
  }

  template <typename Func>
  void parallelFor2D(size_t rows, size_t cols, Func&& f)
  {
    arena_.execute([&] {
      tbb::parallel_for(
        tbb::blocked_range2d<size_t>(0, rows, 0, cols),
        [&](const tbb::blocked_range2d<size_t>& r) {
          for (size_t y = r.rows().begin(); y != r.rows().end(); ++y) {
            for (size_t x = r.cols().begin(); x != r.cols().end(); ++x) {
              f(x, y);
            }
          }
        });
    });
  }

  [[nodiscard]] bool isParallel() const { return num_threads_ > 1; }

private:
  int num_threads_{};
  tbb::task_arena arena_{};
};

}  // namespace rog_map_layer
