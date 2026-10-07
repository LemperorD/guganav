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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "nav2_mppi_controller/models/constraints.hpp"
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
   * @brief 注入当前生效的约束与积分步长
   * @param control_constraints: 含加速度字段的约束集
   * @param model_dt: 相邻预测点之间的积分步长
   */
  void setConstraints(
    const models::ControlConstraints & control_constraints, float model_dt)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief 根据采样控制量预测底盘各时间步的速度
   * @param state: 包含采样控制量并接收预测速度的状态张量
   *
   * 未启用加速度约束时走原来的整体视图赋值，与移植前逐位一致。
   * 启用后改为逐样本串行夹紧**状态速度** `vx/vy/wz`，而采样控制量
   * `cvx/cvy/cwz` 保持原始值（上游 PR #5266 的语义）：加速度上限描述的是
   * "底盘实际能做到什么"，作用在状态量上；控制量保持未夹紧，softmax 才能
   * 分辨"略微超限"与"严重超限"，否则会出现 PR #6072 里那种 chattering。
   */
  virtual void predict(models::State & state)
  {
    using namespace xt::placeholders;  // NOLINT
    if (!accelConstraintsEnabled()) {
      xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
        xt::view(state.cvx, xt::all(), xt::range(0, -1));

      xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
        xt::view(state.cwz, xt::all(), xt::range(0, -1));

      if (isHolonomic()) {
        xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
          xt::view(state.cvy, xt::all(), xt::range(0, -1));
      }
      return;
    }

    const bool is_holo = isHolonomic();
    // 逐轴独立：某一轴的上限留 0 表示该轴不做加速度限制（保持控制量直通），
    // 只有显式给了上限的轴才夹紧。否则"只开 ax、az 留 0"会把 wz 冻结成常量。
    const bool limit_vx =
      control_constraints_.ax_max > 0.0f || control_constraints_.ax_min < 0.0f;
    const bool limit_vy =
      control_constraints_.ay_max > 0.0f || control_constraints_.ay_min < 0.0f;
    const bool limit_wz = control_constraints_.az_max > 0.0f;
    const float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    const float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    const float max_delta_vy = model_dt_ * control_constraints_.ay_max;
    const float min_delta_vy = model_dt_ * control_constraints_.ay_min;
    const float max_delta_wz = model_dt_ * control_constraints_.az_max;

    const size_t rows = state.vx.shape(0);
    const size_t cols = state.vx.shape(1);
    for (size_t i = 0; i < rows; ++i) {
      // 第 0 列是当前实测速度，作为加速度积分的起点。
      float vx_last = state.vx(i, 0);
      float vy_last = is_holo ? state.vy(i, 0) : 0.0f;
      float wz_last = state.wz(i, 0);
      for (size_t j = 1; j < cols; ++j) {
        if (limit_vx) {
          vx_last = std::clamp(
            state.cvx(i, j - 1), vx_last + min_delta_vx, vx_last + max_delta_vx);
        } else {
          vx_last = state.cvx(i, j - 1);
        }
        state.vx(i, j) = vx_last;

        if (limit_wz) {
          wz_last = std::clamp(
            state.cwz(i, j - 1), wz_last - max_delta_wz, wz_last + max_delta_wz);
        } else {
          wz_last = state.cwz(i, j - 1);
        }
        state.wz(i, j) = wz_last;

        if (is_holo) {
          if (limit_vy) {
            vy_last = std::clamp(
              state.cvy(i, j - 1), vy_last + min_delta_vy, vy_last + max_delta_vy);
          } else {
            vy_last = state.cvy(i, j - 1);
          }
          state.vy(i, j) = vy_last;
        }
      }
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

protected:
  /**
   * @brief 是否启用了任一轴的加速度约束
   * @return 返回值: 任一加速度上限非零时为 true
   */
  bool accelConstraintsEnabled() const
  {
    return control_constraints_.ax_max > 0.0f || control_constraints_.ax_min < 0.0f ||
           control_constraints_.ay_max > 0.0f || control_constraints_.ay_min < 0.0f ||
           control_constraints_.az_max > 0.0f;
  }

  float model_dt_{0.0f};  ///< 相邻预测点的积分步长（s）。
  models::ControlConstraints control_constraints_{0, 0, 0, 0, 0, 0, 0, 0, 0};  ///< 当前生效约束。
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
