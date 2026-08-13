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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_

#include <xtensor/xtensor.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief 保存批量速度、采样控制量及机器人当前状态
 */
struct State
{
  // 各候选轨迹在预测时域内的纵向速度。
  xt::xtensor<float, 2> vx;
  // 各候选轨迹在预测时域内的横向速度。
  xt::xtensor<float, 2> vy;
  // 各候选轨迹在预测时域内的偏航角速度。
  xt::xtensor<float, 2> wz;

  // 添加采样噪声后的纵向控制速度。
  xt::xtensor<float, 2> cvx;
  // 添加采样噪声后的横向控制速度。
  xt::xtensor<float, 2> cvy;
  // 添加采样噪声后的偏航控制速度。
  xt::xtensor<float, 2> cwz;

  // 当前机器人位姿，作为轨迹展开的起点。
  geometry_msgs::msg::PoseStamped pose;
  // 当前机器人实测速度，作为状态传播的初值。
  geometry_msgs::msg::Twist speed;

  /**
    * @brief 按采样规模和预测步数重新分配并清零状态张量
    * @param batch_size: 候选轨迹数量
    * @param time_steps: 每条轨迹的预测时间步数
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    vx = xt::zeros<float>({batch_size, time_steps});
    vy = xt::zeros<float>({batch_size, time_steps});
    wz = xt::zeros<float>({batch_size, time_steps});

    cvx = xt::zeros<float>({batch_size, time_steps});
    cvy = xt::zeros<float>({batch_size, time_steps});
    cwz = xt::zeros<float>({batch_size, time_steps});
  }
};
}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
