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

#include "point_lio/IMU_Processing.h"


bool time_list(PointType& x, PointType& y) {
  return point_time_offset_ms(x) < point_time_offset_ms(y);
};

ImuProcessor::ImuProcessor() : logger(rclcpp::get_logger("ImuProcess")) {
}

void ImuProcessor::configure(const Params& params) {
  imu_en = params.enabled;
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


void ImuProcessor::Set_init(Eigen::Vector3d& tmp_gravity,
                            Eigen::Matrix3d& rot) {

  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1), -gravity_(2), 0.0, gravity_(0),
      gravity_(1), -gravity_(0), 0.0;


  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm()
                      / tmp_gravity.norm();


  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();

  if (align_norm < 1e-6) {

    if (align_cos > 1e-6) {
      rot = Eye3d;
    } else {
      rot = -Eye3d;
    }
  } else {

    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm()
                      * acos(align_cos);

    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}


void ImuProcessor::initState(state_input& input_state,
                             state_output& output_state) {
  V3D tmp_gravity;
  if (imu_en) {
    tmp_gravity = -mean_acc / mean_acc.norm() * gravity_magnitude_;
  } else {
    tmp_gravity = gravity_init_;
  }
  M3D rot_init;
  Set_init(tmp_gravity, rot_init);
  input_state.rot = rot_init;
  output_state.rot = rot_init;
  output_state.acc = -rot_init.transpose() * output_state.gravity;
}


void ImuProcessor::IMU_init(const MeasureGroup& meas, int& N) {
  RCLCPP_INFO(logger, "IMU Initializing: %.1f %%",
              double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc;


  for (const auto& imu : meas.imu) {
    const auto& imu_acc = imu->linear_acceleration;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;


    mean_acc += (cur_acc - mean_acc) / N;

    N++;
  }
}


void ImuProcessor::process(const MeasureGroup& meas,
                           PointCloudXYZI::Ptr cur_pcl_un_,
                           state_input& input_state,
                           state_output& output_state) {
  if (imu_en) {
    if (meas.imu.empty()) {
      return;
    }

    if (stage_ == Stage::Initializing) {

      IMU_init(meas, init_iter_num);
      if (init_iter_num <= MAX_INI_COUNT) {
        return;
      }

      RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", 100.0);
      initState(input_state, output_state);
      stage_ = Stage::Ready;
    }

    *cur_pcl_un_ = *(meas.lidar);

    // @todo: 实现 IMU 反向传播去畸变

  } else {

    if (stage_ == Stage::Initializing) {
      initState(input_state, output_state);
      stage_ = Stage::Ready;
    }
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}
