/**
 * @file IMU_Processing.cpp
 * @brief IMU 预处理实现: 初始化、重力对齐、点云去畸变
 *
 * 实现:
 * - time_list 排序谓词
 * - IMU_init: 累积加速度/角速度的滑动平均 (在线零偏估计)
 * - Set_init: 估计重力与先验重力的旋转对齐
 * - Process: 主入口 (初始化分发)
 */

#include "point_lio/core/IMU_Processing.h"

#include <cmath>

bool time_list(PointType& x, PointType& y) {
  return point_time_offset_ms(x) < point_time_offset_ms(y);
};

ImuProcessor::ImuProcessor() : logger(rclcpp::get_logger("ImuProcess")) {
}

void ImuProcessor::configure(const Params& params) {
  gravity_ = params.gravity;
  gravity_init_ = params.gravity_init;
  gravity_magnitude_ = params.gravity_magnitude;
}

bool ImuProcessor::needInit() const {
  return stage_ == Stage::Initializing;
}

void ImuProcessor::reset() {
  RCLCPP_WARN(logger, "reset ImuProcess");
  mean_acc = V3D(0, 0, 0.0);
  stage_ = Stage::Initializing;
  init_iter_num = 1;
}

void ImuProcessor::Set_init_z(Eigen::Vector3d& tmp_gravity,
                              Eigen::Matrix3d& rot) {
  // 比力 f = a - g, 初始状态读出来是负的加速度
  const M3D hat_grav = -Hat(gravity_);

  const V3D cross = hat_grav * tmp_gravity;
  const double cross_norm = cross.norm();
  const double gravity_norm = gravity_.norm();
  const double tmp_norm = tmp_gravity.norm();

  const double align_sin = cross_norm / gravity_norm / tmp_norm;
  const double align_cos = gravity_.dot(tmp_gravity) / gravity_norm / tmp_norm;

  if (align_sin < TOLERANCE) {
    if (align_cos > TOLERANCE) {
      rot = IMatrix3;
    } else {
      // 两向量严格反向时绕任意垂直轴转 180 度, 避免使用反射矩阵 -I.
      const V3D unit_gravity = gravity_ / gravity_norm;
      V3D axis;

      if (std::abs(unit_gravity.x()) <= std::abs(unit_gravity.y())
          && std::abs(unit_gravity.x()) <= std::abs(unit_gravity.z())) {
        axis = unit_gravity.cross(V3D::UnitX());
      } else if (std::abs(unit_gravity.y()) <= std::abs(unit_gravity.z())) {
        axis = unit_gravity.cross(V3D::UnitY());
      } else {
        axis = unit_gravity.cross(V3D::UnitZ());
      }  // 取最小分量作为旋转轴得到最大的旋转角度基轴, 计算-180度旋转

      axis.normalize();
      rot = 2.0 * axis * axis.transpose() - IMatrix3;
    }
  } else {
    const V3D align_angle = cross / cross_norm * std::acos(align_cos);
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

void ImuProcessor::initState(State& output_state) {
  V3D tmp_gravity;

  tmp_gravity = -mean_acc / mean_acc.norm() * gravity_magnitude_;

  M3D rot_init;
  Set_init_z(tmp_gravity, rot_init);
  output_state.rot = rot_init;
  output_state.acc = -rot_init.transpose() * output_state.gravity;
}

void ImuProcessor::IMU_init(const MeasureGroup& meas, int& N) {
  RCLCPP_INFO(logger, "IMU Initializing: %.1f %%",
              double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc;

  for (const auto& imu : meas.imu_measure) {
    const auto& imu_acc = imu->linear_acceleration;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;

    N++;
  }
}

void ImuProcessor::process(const MeasureGroup& meas,
                           PointCloudXYZI::Ptr cur_pcl_un_,
                           State& output_state) {
  if (meas.imu_measure.empty()) {
    return;
  }

  if (stage_ == Stage::Initializing) {
    IMU_init(meas, init_iter_num);
    if (init_iter_num <= MAX_INI_COUNT) {
      return;
    }

    RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", 100.0);
    initState(output_state);
    stage_ = Stage::Ready;
  }

  *cur_pcl_un_ = *(meas.lidar_measure);

  // @todo: 实现 IMU 反向传播去畸变
}
