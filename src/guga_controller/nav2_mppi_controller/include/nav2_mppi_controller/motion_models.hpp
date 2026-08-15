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

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>
#include <string>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief 描述底盘运动学约束的抽象运动模型
 */
class MotionModel
{
public:
  /**
    * @brief 构造运动模型
    */
  MotionModel() = default;

  /**
    * @brief 析构运动模型
    */
  virtual ~MotionModel() = default;

  /**
   * @brief 根据采样控制量预测底盘各时间步的速度
   * @param state: 包含采样控制量并接收预测速度的状态张量
   */
  virtual void predict(models::State & state)
  {
    using namespace xt::placeholders;  // NOLINT
    xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
      xt::view(state.cvx, xt::all(), xt::range(0, -1));

    xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
      xt::view(state.cwz, xt::all(), xt::range(0, -1));

    if (isHolonomic()) {
      xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
        xt::view(state.cvy, xt::all(), xt::range(0, -1));
    }
  }

  /**
   * @brief 判断运动模型是否支持横向速度
   * @return 返回值: 为 true 时优化器会启用 `vy` 采样、约束和轨迹积分
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief 对控制序列施加运动模型专属硬约束
   * @param control_sequence: 待就地约束的控制序列
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}
};

/**
 * @class mppi::AckermannMotionModel
 * @brief 阿克曼转向运动模型
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief 构造阿克曼运动模型并读取最小转弯半径
    * @param param_handler: 用于读取运动模型参数的参数处理器
    * @param name: 控制器参数命名空间名称
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief 判断阿克曼模型是否支持横向速度
   * @return 返回值: 固定返回 false，使优化器禁用 `vy`
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief 按最小转弯半径限制角速度
   * @param control_sequence: 待就地修正的 `vx/wz` 控制序列
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & wz = control_sequence.wz;
    auto abs_vx = xt::fabs(control_sequence.vx);
    auto abs_wz = xt::fabs(wz);

    for (size_t i = 0; i < wz.size(); ++i) {
      if ((abs_vx[i] / abs_wz[i]) < min_turning_r_) {
        wz[i] = std::copysign(abs_vx[i] / min_turning_r_, wz[i]);
      }
    }
  }

  /**
   * @brief 获取阿克曼底盘最小转弯半径
   * @return 返回值: 最小转弯半径，供约束测试和外部诊断使用
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};  ///< 阿克曼底盘允许的最小转弯半径。
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief 差速底盘运动模型
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief 构造差速运动模型
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief 判断差速模型是否支持横向速度
   * @return 返回值: 固定返回 false，使优化器禁用 `vy`
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief 全向底盘运动模型
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief 构造全向运动模型
    */
  OmniMotionModel() = default;

  /**
   * @brief 判断全向模型是否支持横向速度
   * @return 返回值: 固定返回 true，使优化器启用 `vy`
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
