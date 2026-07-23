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

#include "rog_map_layer/esdf_parallel.hpp"

namespace rog_map_layer
{

EsdfParallelExecutor::EsdfParallelExecutor(int num_threads)
: num_threads_{std::max(1, num_threads)}
  , arena_{num_threads_}
{
}

}  // namespace rog_map_layer
