#include "point_lio/core/ManifoldEskf.h"
#include <Eigen/src/Core/Matrix.h>

#include "point_lio/utilities/Profiler.h"

#include <algorithm>
#include <cmath>

using namespace Eigen;

namespace {

  using Vector24 = Eigen::Matrix<double, 24, 1>;
  using Matrix24 = Eigen::Matrix<double, 24, 24>;
  using Matrix3d = Eigen::Matrix3d;

  // 按论文 J 的导数定义实现的右雅可比, 首阶 I - 0.5[d]x.
  Matrix3d rightJacobian(const Eigen::Vector3d& d) {
    const double theta = d.norm();
    const Matrix3d K = Hat(d);
    if (theta < 1e-8) {
      return Matrix3d::Identity() - 0.5 * K;
    }
    const double a = (1.0 - std::cos(theta)) / (theta * theta);
    const double b = (theta - std::sin(theta)) / (theta * theta * theta);
    return Matrix3d::Identity() - a * K + b * K * K;
  }

}  // namespace

void ManifoldEskf::initialize(const PointLioParams& config,
                              const IVoxType::Ptr& map,
                              const Matrix3d& lidar_rotation,
                              const V3D& lidar_translation) {
  configureNoise(config.lidar.point_covariance, config.imu.measurement_acc_cov,
                 config.imu.measurement_gyro_cov);
  configureProcessNoise(config.filter.gyr_cov_output,
                        config.filter.acc_cov_output, config.filter.b_gyr_cov,
                        config.filter.b_acc_cov);

  Matrix<double, 24, 24> P = Matrix<double, 24, 24>::Identity() * 0.01;
  P.block<3, 3>(18, 18) = config.filter.gyr_cov_output * Matrix3d::Identity();
  P.block<3, 3>(21, 21) = config.filter.acc_cov_output * Matrix3d::Identity();
  P.block<3, 3>(9, 9) = config.filter.b_gyr_cov * Matrix3d::Identity();
  P.block<3, 3>(12, 12) = config.filter.b_acc_cov * Matrix3d::Identity();
  // 重力由静止初始化确定, 固定不估计, 避免与加速度零偏混在一起发散.
  P.middleRows<3>(15).setZero();
  P.middleCols<3>(15).setZero();
  setInitialCovariance(P);

  configureFixedExtrinsic(lidar_rotation, lidar_translation);
  setMap(map);
}

ManifoldEskf::ManifoldEskf() {
  F_w_.setZero();
  // 噪声顺序与论文式 11 一致: [n_bg, n_ba, n_g, n_a]
  F_w_.block<3, 3>(9, 0) = Matrix3d::Identity();   // bg 对 n_bg
  F_w_.block<3, 3>(12, 3) = Matrix3d::Identity();  // ba 对 n_ba
  F_w_.block<3, 3>(18, 6) = Matrix3d::Identity();  // omega 对 n_g
  F_w_.block<3, 3>(21, 9) = Matrix3d::Identity();  // acc 对 n_a

  H_I.setZero();
  H_I.block<3, 3>(0, 9) = Matrix3d::Identity();   // 角速度残差对 bg
  H_I.block<3, 3>(0, 18) = Matrix3d::Identity();  // 角速度残差对 omega
  H_I.block<3, 3>(3, 12) = Matrix3d::Identity();  // 加速度残差对 ba
  H_I.block<3, 3>(3, 21) = Matrix3d::Identity();  // 加速度残差对 acc
  D_I = Matrix<double, 6, 6>::Identity();

  Q_.setZero();
  Q_.block<3, 3>(0, 0) = 1e-6 * Matrix3d::Identity();  // n_bg
  Q_.block<3, 3>(3, 3) = 1e-4 * Matrix3d::Identity();  // n_ba
  Q_.block<3, 3>(6, 6) = 1e-4 * Matrix3d::Identity();  // n_g
  Q_.block<3, 3>(9, 9) = 1e-2 * Matrix3d::Identity();  // n_a
  R_I = 1e-2 * Matrix<double, 6, 6>::Identity();
}

void ManifoldEskf::configureProcessNoise(double gyr_cov, double acc_cov,
                                         double b_gyr_cov, double b_acc_cov) {
  Q_.setZero();
  Q_.block<3, 3>(0, 0) = b_gyr_cov * Matrix3d::Identity();  // n_bg
  Q_.block<3, 3>(3, 3) = b_acc_cov * Matrix3d::Identity();  // n_ba
  Q_.block<3, 3>(6, 6) = gyr_cov * Matrix3d::Identity();    // n_g
  Q_.block<3, 3>(9, 9) = acc_cov * Matrix3d::Identity();    // n_a
}

void ManifoldEskf::configureNoise(double lidar_cov, double imu_acc_cov,
                                  double imu_omg_cov) {
  lidar_cov_ = lidar_cov;
  R_I = Matrix<double, 6, 6>::Zero();
  R_I.block<3, 3>(0, 0) = imu_omg_cov * Matrix3d::Identity();
  R_I.block<3, 3>(3, 3) = imu_acc_cov * Matrix3d::Identity();
}

void ManifoldEskf::configureFixedExtrinsic(const Eigen::Matrix3d& R,
                                           const Eigen::Vector3d& t) {
  R_LI_fixed_ = R;
  t_LI_fixed_ = t;
}

void ManifoldEskf::setInitialCovariance(
    const Eigen::Matrix<double, 24, 24>& P) {
  P_ = P;
}

void ManifoldEskf::setStartTime(double t) {
  last_time_ = t;
}

void ManifoldEskf::setImuResidual(const Eigen::Matrix<double, 6, 1>& r) {
  r_I = r;
}

Vector24 ManifoldEskf::f_(const State& x_, const V3D& omega,
                          const V3D& acc) const {
  Vector24 d = Vector24::Zero();
  d.segment<3>(0) = x_.omega;
  d.segment<3>(3) = x_.vel;
  d.segment<3>(6) = x_.rot * x_.acc + x_.gravity;
  return d;
}

Matrix24 ManifoldEskf::predictCovariance(const double dt) const {
  // 计算 G = F_x P F_x^T + F_w Q F_w^T, 其中 F_x 是状态转移矩阵, F_w
  // 是噪声转移矩阵.

  // F_x 只有五个非零的非对角块, 其余是单位块,
  // 所以分两步按块乘:
  //   第一步 G = F_x P, 第 3 到第 7 行块是整行复制;
  //   第二步 G F_x^T, 第 3 到第 7 列块是整列复制.
  const Matrix3d F00 = Exp(-dt * x_.omega);
  const Matrix3d F20 = -dt * x_.rot * Hat(x_.acc);
  const Matrix3d F27 = dt * x_.rot;

  Matrix24 G;
  for (int c = 0; c < 8; ++c) {
    const int cc = 3 * c;
    G.block<3, 3>(0, cc) = F00 * P_.block<3, 3>(0, cc)
                           + dt * P_.block<3, 3>(18, cc);
    G.block<3, 3>(3, cc) = P_.block<3, 3>(3, cc) + dt * P_.block<3, 3>(6, cc);
    G.block<3, 3>(6, cc) = P_.block<3, 3>(6, cc) + F20 * P_.block<3, 3>(0, cc)
                           + F27 * P_.block<3, 3>(21, cc);
    for (int r = 3; r < 8; ++r) {
      G.block<3, 3>(3 * r, cc) = P_.block<3, 3>(3 * r, cc);
    }
  }

  Matrix24 out;
  for (int i = 0; i < 8; ++i) {
    const int rr = 3 * i;
    const Matrix3d g0 = G.block<3, 3>(rr, 0);
    const Matrix3d g1 = G.block<3, 3>(rr, 3);
    const Matrix3d g2 = G.block<3, 3>(rr, 6);
    const Matrix3d g6 = G.block<3, 3>(rr, 18);
    const Matrix3d g7 = G.block<3, 3>(rr, 21);
    out.block<3, 3>(rr, 0) = g0 * F00.transpose() + dt * g6;
    out.block<3, 3>(rr, 3) = g1 + dt * g2;
    out.block<3, 3>(rr, 6) = g2 + g0 * F20.transpose() + g7 * F27.transpose();
    out.block<3, 3>(rr, 9) = G.block<3, 3>(rr, 9);
    out.block<3, 3>(rr, 12) = G.block<3, 3>(rr, 12);
    out.block<3, 3>(rr, 15) = G.block<3, 3>(rr, 15);
    out.block<3, 3>(rr, 18) = g6;
    out.block<3, 3>(rr, 21) = g7;
  }

  // F_w Q F_w^T 把 Q 的四个对角块搬到对应状态块, 要求 Q 分块对角.
  out.block<3, 3>(9, 9) += dt * Q_.block<3, 3>(0, 0);
  out.block<3, 3>(12, 12) += dt * Q_.block<3, 3>(3, 3);
  out.block<3, 3>(18, 18) += dt * Q_.block<3, 3>(6, 6);
  out.block<3, 3>(21, 21) += dt * Q_.block<3, 3>(9, 9);
  return out;
}

void ManifoldEskf::pointBodyToWorld(PointType const* pi, PointType* po,
                                    const LidarParams& params,
                                    const State& state) const {
  const Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
  Eigen::Vector3d p_global;
  p_global = state.rot * (R_LI_fixed_ * p_body + t_LI_fixed_) + state.pos;
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void ManifoldEskf::updateWorldPoints(MeasureGroup& meas, FilterOutput& out,
                                     const LidarParams& params, int group_idx,
                                     int point_offset) {
  const int group_size = meas.time_seq[group_idx];
  for (int j = 0; j < group_size; ++j) {
    const int index = point_offset + j;
    PointType& point_body = meas.feats_down_body->points[index];
    PointType& point_world = out.feats_down_world->points[index];
    pointBodyToWorld(&point_body, &point_world, params, x_);
  }
}

bool ManifoldEskf::stateTransform(double lidar_time, const V3D& omega,
                                  const V3D& acc) {
  const double dt = lidar_time - last_time_;
  if (dt < 0.0) {
    return false;
  }
  x_predict = x_;
  const Vector24 delta = dt * f_(x_, omega, acc);
  x_predict.boxplus(delta);
  P_predict = predictCovariance(dt);
  last_time_ = lidar_time;
  return true;
}

void ManifoldEskf::applySequentialLidarUpdate(const MatrixXd& B,
                                              const VectorXd& r,
                                              const double sigma2, Matrix24& P,
                                              Vector24& dx) {
  // 逐点标量更新与批量更新代数等价:
  //   P+ = P - k g^T - g k^T + s k k^T,  g = P h,  s = h^T P h + sigma2,  k = g
  //   / s.
  for (int i = 0; i < B.rows(); ++i) {
    const Eigen::Matrix<double, 6, 1> h = B.row(i).transpose();
    const double s = h.dot(P.topLeftCorner<6, 6>() * h) + sigma2;
    const Vector24 g = P.leftCols<6>() * h;
    const Vector24 k = g / s;
    dx += k * (r(i) - h.dot(dx.head<6>()));
    P.noalias() -= k * g.transpose();
    P.noalias() -= g * k.transpose();
    P.noalias() += s * (k * k.transpose());
  }
}

void ManifoldEskf::applyLidarUpdate() {
  Matrix24 Pc = P_predict;
  Vector24 dx = Vector24::Zero();
  applySequentialLidarUpdate(B_L, r_L, lidar_cov_, Pc, dx);

  dx_o = dx;
  x_ = x_predict;
  x_.boxplus(dx_o);
  applyRotationErrorReset(Pc);
}

void ManifoldEskf::applyRotationErrorReset(const Matrix24& P_in) {
  const Matrix3d R_delta = x_predict.rot.transpose() * x_.rot;
  const Matrix3d Jr = rightJacobian(Log(R_delta));
  // J = I 除左上角 Jr, 所以 J P J^T 只改前 3 行与前 3 列.
  Matrix24 M;
  M.topRows<3>() = Jr * P_in.topRows<3>();
  M.bottomRows<21>() = P_in.bottomRows<21>();
  P_.leftCols<3>() = M.leftCols<3>() * Jr.transpose();
  P_.rightCols<21>() = M.rightCols<21>();
}

void ManifoldEskf::commitPrediction() {
  x_ = x_predict;
  P_ = P_predict;
}

void ManifoldEskf::applyImuUpdate(const std::array<bool, 6>& saturated) {
  Matrix<double, 6, 24> H = H_I;
  for (int i = 0; i < 6; ++i) {
    if (saturated[i]) {
      H.row(i).setZero();
    }
  }
  S_I = H * P_predict * H.transpose() + D_I * R_I * D_I.transpose();
  K_I = P_predict * H.transpose() * S_I.inverse();
  dx_o = K_I * r_I;
  const Matrix24 P_bias = (I - K_I * H) * P_predict * (I - K_I * H).transpose()
                          + K_I * R_I * K_I.transpose();

  x_ = x_predict;
  x_.boxplus(dx_o);
  applyRotationErrorReset(P_bias);
}

void ManifoldEskf::set_gravity(Eigen::Vector3d gravity) {
  x_.gravity = gravity;
}

const State& ManifoldEskf::get_x() const {
  return x_;
}

const State& ManifoldEskf::get_x_predict() const {
  return x_predict;
}

state_output ManifoldEskf::toStateOutput() const {
  return x_;
}

void ManifoldEskf::initState(const ImuInitEstimate& estimate,
                             const ImuParams& params) {
  if (estimate.static_count == 0) {
    // 没有静止段, 不做重力对齐, 零偏保持零, 交给滤波器在线收敛.
    x_.rot = IMatrix3;
    x_.acc = -x_.gravity;
    x_.bg = V3D::Zero();
    // TODO: 换成六面法标定的加速度零偏, 单位 m/s².
    x_.ba = V3D::Zero();
    return;
  }

  const V3D tmp_gravity = -estimate.mean_acc / estimate.mean_acc.norm()
                          * params.processor.gravity_magnitude;
  M3D rot_init;
  setInitialRotation(tmp_gravity, rot_init);
  x_.rot = rot_init;
  x_.acc = -rot_init.transpose() * x_.gravity;
  x_.bg = estimate.mean_gyr;
  // TODO: 换成六面法标定的加速度零偏, 单位 m/s².
  x_.ba = V3D::Zero();
}

void ManifoldEskf::setInitialRotation(const V3D& tmp_gravity, M3D& rot) {
  const M3D hat_grav = -Hat(x_.gravity);

  const V3D cross = hat_grav * tmp_gravity;
  const double cross_norm = cross.norm();
  const double gravity_norm = x_.gravity.norm();
  const double tmp_norm = tmp_gravity.norm();

  const double align_sin = cross_norm / gravity_norm / tmp_norm;
  const double align_cos = x_.gravity.dot(tmp_gravity) / gravity_norm
                           / tmp_norm;

  if (align_sin < kAlignTolerance) {
    if (align_cos > kAlignTolerance) {
      rot = IMatrix3;
    } else {
      // 两向量严格反向时绕任意垂直轴转 180 度, 避免使用反射矩阵 -I.
      const V3D unit_gravity = x_.gravity / gravity_norm;
      V3D axis;

      if (std::abs(unit_gravity.x()) <= std::abs(unit_gravity.y())
          && std::abs(unit_gravity.x()) <= std::abs(unit_gravity.z())) {
        axis = unit_gravity.cross(V3D::UnitX());
      } else if (std::abs(unit_gravity.y()) <= std::abs(unit_gravity.z())) {
        axis = unit_gravity.cross(V3D::UnitY());
      } else {
        axis = unit_gravity.cross(V3D::UnitZ());
      }

      axis.normalize();
      rot = 2.0 * axis * axis.transpose() - IMatrix3;
    }
  } else {
    const V3D align_angle = cross / cross_norm * std::acos(align_cos);
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

bool ManifoldEskf::imuUpdate(const ImuSample& sample, const ImuParams& params) {
  const double acc_scale = params.processor.gravity_magnitude / params.acc_norm;
  const V3D acc_meas = sample.measurement.linear_acceleration * acc_scale;

  // 比力模长应始终 >= g: 任何线性加速度都只会把它变大。模长低于
  // g - 阈值的样本在物理上不可能, 是仿真器加速度通道的坏点; 模长超过
  // 上限的同样丢弃。两类坏点都会被积分成垂直速度。
  const double acc_norm = acc_meas.norm();
  const bool acc_below_floor = params.acc_spike_thresh > 0.0
                               && acc_norm < params.processor.gravity_magnitude
                                                 - params.acc_spike_thresh;
  const bool acc_above_ceiling = params.acc_spike_max > 0.0
                                 && acc_norm > params.acc_spike_max;
  if (acc_below_floor || acc_above_ceiling) {
    return false;
  }

  const V3D omega = sample.measurement.angular_velocity;
  if (!stateTransform(sample.timestamp, omega, acc_meas)) {
    return false;
  }

  const auto& state = get_x_predict();
  Matrix<double, 6, 1> r_I;
  r_I.segment<3>(0) = sample.measurement.angular_velocity
                      - (state.omega + state.bg);
  r_I.segment<3>(3) = sample.measurement.linear_acceleration * acc_scale
                      - (state.acc + state.ba);

  std::array<bool, 6> saturated{};
  if (params.check_saturation) {
    const double gyro[3] = {sample.measurement.angular_velocity(0),
                            sample.measurement.angular_velocity(1),
                            sample.measurement.angular_velocity(2)};
    const double acc[3] = {sample.measurement.linear_acceleration(0),
                           sample.measurement.linear_acceleration(1),
                           sample.measurement.linear_acceleration(2)};
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(gyro[axis]) >= 0.99 * params.saturation_gyro) {
        saturated[axis] = true;
        r_I(axis) = 0.0;
      }
      if (std::abs(acc[axis]) >= 0.99 * params.saturation_acc) {
        saturated[axis + 3] = true;
        r_I(axis + 3) = 0.0;
      }
    }
  }

  setImuResidual(r_I);
  applyImuUpdate(saturated);
  return true;
}

int ManifoldEskf::lidarUpdate(MeasureGroup& meas, FilterOutput& out,
                              const LidarParams& params, int group_idx,
                              int point_offset, bool allow_update) {
  const double lidar_time = meas.lidar_start_time
                            + point_time_offset_ms(
                                  meas.feats_down_body->points[point_offset])
                                  / 1000.0;
  if (!stateTransform(lidar_time, get_x().omega, get_x().acc)) {
    return -1;
  }
  if (!allow_update) {
    commitPrediction();
    return 0;
  }
  const int effect_num = updateLidar(meas, out, params, group_idx,
                                     point_offset);
  if (effect_num == 0) {
    commitPrediction();
    out.processed_size = point_offset + meas.time_seq[group_idx];
    return 0;
  }
  applyLidarUpdate();
  updateWorldPoints(meas, out, params, group_idx, point_offset);
  out.processed_size = point_offset + meas.time_seq[group_idx];
  return effect_num;
}

int ManifoldEskf::updateLidar(MeasureGroup& meas, FilterOutput& out,
                              const LidarParams& params, int group_idx,
                              int point_offset) {
  const int group_size = meas.time_seq[group_idx];
  VF(4) pabcd;
  pabcd.setZero();
  {
    normvec_->resize(group_size);
    const size_t total = meas.feats_down_body->points.size();
    nearest_points_.resize(total);
    pbody_list_.resize(total);
    point_selected_surf_.resize(total);
  }
  int effect_num = 0;
  int near_num = 0;
  int plane_num = 0;

  for (int j = 0; j < group_size; ++j) {
    const int index = point_offset + j;
    PointType& point_body = meas.feats_down_body->points[index];
    PointType& point_world = out.feats_down_world->points[index];
    {
      pointBodyToWorld(&point_body, &point_world, params, x_predict);
      const Eigen::Vector3d p_lidar(point_body.x, point_body.y, point_body.z);
      pbody_list_[index] = R_LI_fixed_ * p_lidar + t_LI_fixed_;
    }

    const Eigen::Vector3d p_body = pbody_list_[index];
    const double p_norm = p_body.norm();
    auto& points_near = nearest_points_[index];
    {
      map_->GetClosestPoint(point_world, points_near, NUM_MATCH_POINTS);
    }

    if (points_near.size() < NUM_MATCH_POINTS) {
      point_selected_surf_[index] = false;
    } else {
      ++near_num;
      point_selected_surf_[index] = false;
      bool plane_ok = false;
      {
        plane_ok = esti_plane(pabcd, points_near, params.plane_threshold);
      }
      if (plane_ok) {
        ++plane_num;
        const double pd2 = std::fabs(pabcd(0) * point_world.x
                                     + pabcd(1) * point_world.y
                                     + pabcd(2) * point_world.z + pabcd(3));
        if (p_norm > params.match_threshold * pd2 * pd2) {
          point_selected_surf_[index] = true;
          normvec_->points[j].x = pabcd(0);
          normvec_->points[j].y = pabcd(1);
          normvec_->points[j].z = pabcd(2);
          normvec_->points[j].intensity = pabcd(3);
          ++effect_num;
        }
      }
    }
  }

  if (effect_num == 0) {
    return 0;
  }

  {
    B_L = MatrixXd::Zero(effect_num, 6);
    r_L = VectorXd::Zero(effect_num);
    int m = 0;
    for (int j = 0; j < group_size; ++j) {
      const int index = point_offset + j;
      if (!point_selected_surf_[index]) {
        continue;
      }
      const Eigen::Vector3d norm_vec(
          normvec_->points[j].x, normvec_->points[j].y, normvec_->points[j].z);

      const Eigen::Vector3d point_imu = pbody_list_[index];
      const Eigen::Vector3d C = x_predict.rot.transpose() * norm_vec;
      const Eigen::Vector3d A = Hat(point_imu) * C;
      B_L.block<1, 3>(m, 0) = A.transpose();
      B_L.block<1, 3>(m, 3) = norm_vec.transpose();

      r_L(m) = -norm_vec(0) * out.feats_down_world->points[index].x
               - norm_vec(1) * out.feats_down_world->points[index].y
               - norm_vec(2) * out.feats_down_world->points[index].z
               - normvec_->points[j].intensity;
      // 限制单点残差, 避免错误的平面拟合把位置一次拉飞.
      r_L(m) = std::clamp(r_L(m), -0.5, 0.5);
      ++m;
    }
  }

  return effect_num;
}
