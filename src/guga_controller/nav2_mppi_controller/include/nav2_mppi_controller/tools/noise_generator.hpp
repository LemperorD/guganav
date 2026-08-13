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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"

namespace mppi
{

/**
 * @class mppi::NoiseGenerator
 * @brief 围绕当前最优控制序列生成随机采样控制
 */
class NoiseGenerator
{
public:
  /**
    * @brief 构造噪声生成器
    */
  NoiseGenerator() = default;

  /**
   * @brief 使用优化器设置和运动模型初始化噪声生成器
   * @param settings: 优化器采样规模和标准差设置
   * @param is_holonomic: 底盘是否支持横向速度
   * @param name: 参数命名空间
   * @param param_handler: 参数读取工具
   */
  void initialize(
    mppi::models::OptimizerSettings & settings,
    bool is_holonomic, const std::string & name, ParametersHandler * param_handler);

  /**
   * @brief 关闭噪声生成线程
   */
  void shutdown();

  /**
   * @brief 通知后台线程为下一次优化迭代生成新噪声
   */
  void generateNextNoises();

  /**
   * @brief 将噪声叠加到控制序列并写入状态控制张量
   * @param state: 接收批量 `cvx/cvy/cwz` 采样控制的状态张量
   * @param control_sequence: 作为采样均值的当前最优控制序列
   */
  void setNoisedControls(models::State & state, const models::ControlSequence & control_sequence);

  /**
   * @brief 使用新设置重置噪声张量
   * @param settings: 新的优化器采样设置
   * @param is_holonomic: 底盘是否支持横向速度
   */
  void reset(mppi::models::OptimizerSettings & settings, bool is_holonomic);

protected:
  /**
   * @brief 后台执行噪声生成任务的线程入口
   */
  void noiseThread();

  /**
   * @brief 生成满足配置标准差的高斯控制噪声并写入内部张量
   */
  void generateNoisedControls();

  xt::xtensor<float, 2> noises_vx_;
  xt::xtensor<float, 2> noises_vy_;
  xt::xtensor<float, 2> noises_wz_;

  mppi::models::OptimizerSettings settings_;
  bool is_holonomic_;

  std::thread noise_thread_;
  std::condition_variable noise_cond_;
  std::mutex noise_lock_;
  bool active_{false}, ready_{false}, regenerate_noises_{false};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
