#include "point_lio/core/ManifoldEskf.h"

#include <algorithm>
#include <cmath>

using namespace Eigen;

namespace {

  using Vector30 = Eigen::Matrix<double, 30, 1>;
  using Matrix30 = Eigen::Matrix<double, 30, 30>;
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

ManifoldEskf::ManifoldEskf() {
  F_w_.setZero();
  F_w_.block<3, 3>(9, 6) = Matrix3d::Identity();   // bg 对 n_bg
  F_w_.block<3, 3>(12, 9) = Matrix3d::Identity();  // ba 对 n_ba
  F_w_.block<3, 3>(18, 0) = Matrix3d::Identity();  // omega 对 w_omega
  F_w_.block<3, 3>(21, 3) = Matrix3d::Identity();  // acc 对 w_a
  D_L = MatrixXd::Identity(6, 6);

  H_I.setZero();
  H_I.block<3, 3>(0, 18) = Matrix3d::Identity();  // 角速度对 omega
  H_I.block<3, 3>(3, 21) = Matrix3d::Identity();  // 加速度对 acc
  D_I = Matrix<double, 6, 6>::Identity();

  Q_.setZero();
  Q_.block<3, 3>(0, 0) = 1e-4 * Matrix3d::Identity();  // w_omega
  Q_.block<3, 3>(3, 3) = 1e-2 * Matrix3d::Identity();  // w_a
  Q_.block<3, 3>(6, 6) = 1e-6 * Matrix3d::Identity();  // n_bg
  Q_.block<3, 3>(9, 9) = 1e-4 * Matrix3d::Identity();  // n_ba
  R_I = 1e-2 * Matrix<double, 6, 6>::Identity();
}

void ManifoldEskf::configureProcessNoise(double gyr_cov, double acc_cov,
                                         double b_gyr_cov, double b_acc_cov) {
  Q_.setZero();
  Q_.block<3, 3>(0, 0) = gyr_cov * Matrix3d::Identity();
  Q_.block<3, 3>(3, 3) = acc_cov * Matrix3d::Identity();
  Q_.block<3, 3>(6, 6) = b_gyr_cov * Matrix3d::Identity();
  Q_.block<3, 3>(9, 9) = b_acc_cov * Matrix3d::Identity();
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
    const Eigen::Matrix<double, 30, 30>& P) {
  P_ = P;
}

void ManifoldEskf::setStartTime(double t) {
  last_time_ = t;
}

void ManifoldEskf::setImuResidual(const Eigen::Matrix<double, 6, 1>& r) {
  r_I = r;
}

Vector30 ManifoldEskf::f_(const State& x_, const V3D& omega,
                          const V3D& acc) const {
  Vector30 d = Vector30::Zero();
  d.segment<3>(0) = omega;
  d.segment<3>(3) = x_.vel;
  d.segment<3>(6) = x_.rot * (acc - x_.ba) + x_.gravity;
  return d;
}

Matrix30 ManifoldEskf::F_x(const State& x_, const double dt) const {
  Matrix30 d = Matrix30::Identity();
  d.block<3, 3>(0, 0) = Exp(-dt * x_.omg);
  d.block<3, 3>(0, 18) = dt * Matrix3d::Identity();
  d.block<3, 3>(3, 6) = dt * Matrix3d::Identity();
  const V3D acc_bias = x_.acc - x_.ba;
  d.block<3, 3>(6, 0) = -dt * x_.rot * Hat(acc_bias);
  d.block<3, 3>(6, 21) = dt * x_.rot;
  return d;
}

void ManifoldEskf::pointBodyToWorld(PointType const* pi, PointType* po,
                                    const LidarParams& params,
                                    const State& state) const {
  const Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
  Eigen::Vector3d p_global;
  if (params.extrinsic_estimation) {
    p_global = state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I)
               + state.pos;
  } else {
    p_global = state.rot * (R_LI_fixed_ * p_body + t_LI_fixed_) + state.pos;
  }
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

int ManifoldEskf::updateLidar(MeasureGroup& meas, const LidarParams& params,
                              int group_idx, int point_offset) {
  const int group_size = meas.time_seq[group_idx];
  VF(4) pabcd;
  pabcd.setZero();
  normvec_->resize(group_size);
  const size_t total = meas.feats_down_body->points.size();
  nearest_points_.resize(total);
  pbody_list_.resize(total);
  int effect_num = 0;

  for (int j = 0; j < group_size; ++j) {
    const int index = point_offset + j;
    PointType& point_body = meas.feats_down_body->points[index];
    PointType& point_world = meas.feats_down_world->points[index];
    pointBodyToWorld(&point_body, &point_world, params, x_predict);

    const Eigen::Vector3d p_lidar(point_body.x, point_body.y, point_body.z);
    pbody_list_[index] =
        params.extrinsic_estimation
            ? x_predict.offset_R_L_I * p_lidar + x_predict.offset_T_L_I
            : R_LI_fixed_ * p_lidar + t_LI_fixed_;
    const Eigen::Vector3d p_body = pbody_list_[index];
    const double p_norm = p_body.norm();

    auto& points_near = nearest_points_[index];
    map_->GetClosestPoint(point_world, points_near, NUM_MATCH_POINTS);

    if (points_near.size() < NUM_MATCH_POINTS) {
      point_selected_surf_[index] = false;
    } else {
      point_selected_surf_[index] = false;
      if (esti_plane(pabcd, points_near, params.plane_threshold)) {
        const double pd2 =
            std::fabs(pabcd(0) * point_world.x + pabcd(1) * point_world.y
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

  H_L = MatrixXd::Zero(effect_num, 30);
  r_L = VectorXd::Zero(effect_num);
  D_L = MatrixXd::Identity(effect_num, effect_num);
  R_L = lidar_cov_ * MatrixXd::Identity(effect_num, effect_num);
  int m = 0;

  for (int j = 0; j < group_size; ++j) {
    const int index = point_offset + j;
    if (!point_selected_surf_[index]) {
      continue;
    }
    const Eigen::Vector3d norm_vec(normvec_->points[j].x,
                                   normvec_->points[j].y,
                                   normvec_->points[j].z);

    if (params.extrinsic_estimation) {
      const Eigen::Vector3d point_imu = pbody_list_[index];
      const Eigen::Vector3d p_lidar(
          meas.feats_down_body->points[index].x,
          meas.feats_down_body->points[index].y,
          meas.feats_down_body->points[index].z);
      const Eigen::Vector3d C = x_predict.rot.transpose() * norm_vec;
      const Eigen::Vector3d A = Hat(point_imu) * C;
      const Eigen::Vector3d B =
          Hat(p_lidar) * x_predict.offset_R_L_I.transpose() * C;
      H_L.block<1, 3>(m, 3) = norm_vec.transpose();
      H_L.block<1, 3>(m, 0) = A.transpose();
      H_L.block<1, 3>(m, 24) = B.transpose();
      H_L.block<1, 3>(m, 27) = C.transpose();
    } else {
      const Eigen::Vector3d point_imu = pbody_list_[index];
      const Eigen::Vector3d C = x_predict.rot.transpose() * norm_vec;
      const Eigen::Vector3d A = Hat(point_imu) * C;
      H_L.block<1, 3>(m, 3) = norm_vec.transpose();
      H_L.block<1, 3>(m, 0) = A.transpose();
    }

    r_L(m) = -norm_vec(0) * meas.feats_down_world->points[index].x
             - norm_vec(1) * meas.feats_down_world->points[index].y
             - norm_vec(2) * meas.feats_down_world->points[index].z
             - normvec_->points[j].intensity;
    // 限制单点残差, 避免错误的平面拟合把位置一次拉飞.
    r_L(m) = std::clamp(r_L(m), -0.5, 0.5);
    ++m;
  }
  return effect_num;
}

void ManifoldEskf::updateWorldPoints(MeasureGroup& meas,
                                     const LidarParams& params, int group_idx,
                                     int point_offset) {
  const int group_size = meas.time_seq[group_idx];
  for (int j = 0; j < group_size; ++j) {
    const int index = point_offset + j;
    PointType& point_body = meas.feats_down_body->points[index];
    PointType& point_world = meas.feats_down_world->points[index];
    pointBodyToWorld(&point_body, &point_world, params, x_);
  }
}

bool ManifoldEskf::stateTransform(double lidar_time, const V3D& omega,
                                  const V3D& acc) {
  const State zero_input{};
  const double dt = lidar_time - last_time_;
  if (dt < 0.0) {
    return false;
  }
  x_predict = x_;
  const Vector30 delta = dt * f_(x_, omega, acc);
  x_predict.boxplus(delta);
  P_predict = F_x(x_, dt) * P_ * F_x(x_, dt).transpose()
              + F_w_ * Q_ * F_w_.transpose() * dt;
  last_time_ = lidar_time;
  return true;
}

void ManifoldEskf::applyLidarUpdate() {
  S_L = H_L * P_predict * H_L.transpose() + D_L * R_L * D_L.transpose();
  K_L = P_predict * H_L.transpose() * S_L.inverse();
  dx_o = K_L * r_L;
  P_bias = (I - K_L * H_L) * P_predict * (I - K_L * H_L).transpose()
           + K_L * R_L * K_L.transpose();

  x_ = x_predict;
  x_.boxplus(dx_o);

  const Matrix3d R_delta = x_predict.rot.transpose() * x_.rot;
  const Eigen::Vector3d theta = Log(R_delta);
  const Eigen::Vector3d theta_ext = Log(x_predict.offset_R_L_I.transpose()
                                        * x_.offset_R_L_I);

  J = I;
  J.block<3, 3>(0, 0) = rightJacobian(theta);
  J.block<3, 3>(24, 24) = rightJacobian(theta_ext);

  P_ = J * P_bias * J.transpose();
}

void ManifoldEskf::commitPrediction() {
  x_ = x_predict;
  P_ = P_predict;
}

void ManifoldEskf::applyImuUpdate(const std::array<bool, 6>& saturated) {
  Matrix<double, 6, 30> H = H_I;
  for (int i = 0; i < 6; ++i) {
    if (saturated[i]) {
      H.row(i).setZero();
    }
  }
  S_I = H * P_predict * H.transpose() + D_I * R_I * D_I.transpose();
  K_I = P_predict * H.transpose() * S_I.inverse();
  dx_o = K_I * r_I;
  P_bias = (I - K_I * H) * P_predict * (I - K_I * H).transpose()
           + K_I * R_I * K_I.transpose();

  x_ = x_predict;
  x_.boxplus(dx_o);

  const Matrix3d R_delta = x_predict.rot.transpose() * x_.rot;
  const Eigen::Vector3d theta = Log(R_delta);
  const Eigen::Vector3d theta_ext = Log(x_predict.offset_R_L_I.transpose()
                                        * x_.offset_R_L_I);

  J = I;
  J.block<3, 3>(0, 0) = rightJacobian(theta);
  J.block<3, 3>(24, 24) = rightJacobian(theta_ext);

  P_ = J * P_bias * J.transpose();
}

State& ManifoldEskf::mutable_x() {
  return x_;
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
