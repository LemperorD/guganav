// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Path
 * @brief 以位置和航向张量表示的参考路径
 */
struct Path
{
  // 各参考路径点的横轴坐标。
  xt::xtensor<float, 1> x;
  // 各参考路径点的纵轴坐标。
  xt::xtensor<float, 1> y;
  // 各参考路径点的航向角。
  xt::xtensor<float, 1> yaws;

  /**
    * @brief 按指定点数重新分配并清零路径张量
    * @param size: 路径点数量
    */
  void reset(unsigned int size)
  {
    x = xt::zeros<float>({size});
    y = xt::zeros<float>({size});
    yaws = xt::zeros<float>({size});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_
