/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <functional>
#include <vector>

#include "../mtk/build_manifold.hpp"
#include "../mtk/startIdx.hpp"
#include "../mtk/types/S2.hpp"
#include "../mtk/types/SEn.hpp"
#include "../mtk/types/SOn.hpp"
#include "../mtk/types/vect.hpp"

namespace esekfom {

  using namespace Eigen;

  template <typename T>
  struct dyn_share_modified {
    bool valid;
    bool converge;
    T M_Noise;
    Eigen::Matrix<T, Eigen::Dynamic, 1> z;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
    Eigen::Matrix<T, 6, 1> z_IMU;
    Eigen::Matrix<T, 6, 1> R_IMU;
    bool satu_check[6];
  };

  template <typename state, int process_noise_dof, typename input = state,
            typename measurement = state, int measurement_noise_dof = 0>
  class esekf {
    static constexpr int StateDof = state::DOF;
    static constexpr int StateDim = state::DIM;
    static constexpr int MeasurementDof = measurement::DOF;

  public:
    using Scalar = typename state::scalar;
    using Covariance = Matrix<Scalar, StateDof, StateDof>;
    using CovarianceJacobian = Matrix<Scalar, StateDim, StateDof>;
    using SparseMatrixType = SparseMatrix<Scalar>;
    using VectorizedState = Matrix<Scalar, StateDof, 1>;
    using FlattenedState = Matrix<Scalar, StateDim, 1>;
    using ProcessModel = std::function<FlattenedState(state&, const input&)>;
    using ProcessMatrix1 =
        std::function<Matrix<Scalar, StateDim, StateDof>(state&, const input&)>;
    using ProcessMatrix2 =
        std::function<Matrix<Scalar, StateDim, process_noise_dof>(
            state&, const input&)>;
    using ProcessNoiseCovariance =
        Matrix<Scalar, process_noise_dof, process_noise_dof>;
    using MeasurementModelDynShareModifiedCov = std::function<void(
        state&, Eigen::Matrix3d, Eigen::Matrix3d, dyn_share_modified<Scalar>&)>;
    using MeasurementModelDynShareModified =
        std::function<void(state&, dyn_share_modified<Scalar>&)>;
    using MeasurementMatrix1 =
        std::function<Matrix<Scalar, MeasurementDof, StateDof>(state&)>;
    using MeasurementMatrix1Dyn =
        std::function<Matrix<Scalar, Eigen::Dynamic, StateDof>(state&)>;
    using MeasurementMatrix2 = std::function<
        Matrix<Scalar, MeasurementDof, measurement_noise_dof>(state&)>;
    using MeasurementMatrix2Dyn =
        std::function<Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>(state&)>;
    using MeasurementNoiseCovariance =
        Matrix<Scalar, measurement_noise_dof, measurement_noise_dof>;
    using MeasurementNoiseCovarianceDyn =
        Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    esekf(const state& x = state(),
          const Covariance& P = Covariance::Identity())
        : x_(x), P_(P) {};

    void init_dyn_share_modified_2h(
        ProcessModel f_in, ProcessMatrix1 f_x_in,
        MeasurementModelDynShareModifiedCov h_dyn_share_in1) {
      f = f_in;
      f_x = f_x_in;
      // f_w = f_w_in;
      h_dyn_share_modified_1 = h_dyn_share_in1;
      // h_dyn_share_modified_3 = h_dyn_share_in3;
      maximum_iter = 1;
      x_.build_S2_state();
      x_.build_SO3_state();
      x_.build_vect_state();
      x_.build_SEN_state();
    }

    void init_dyn_share_modified_3h(
        ProcessModel f_in, ProcessMatrix1 f_x_in,
        MeasurementModelDynShareModifiedCov h_dyn_share_in1,
        MeasurementModelDynShareModified h_dyn_share_in2) {
      f = f_in;
      f_x = f_x_in;
      // f_w = f_w_in;
      h_dyn_share_modified_1 = h_dyn_share_in1;
      h_dyn_share_modified_2 = h_dyn_share_in2;
      // h_dyn_share_modified_3 = h_dyn_share_in3;
      maximum_iter = 1;
      x_.build_S2_state();
      x_.build_SO3_state();
      x_.build_vect_state();
      x_.build_SEN_state();
    }

    // iterated error state EKF propogation
    void predict(double& dt, ProcessNoiseCovariance& Q, const input& i_in,
                 bool predict_state, bool prop_cov) {
      if (predict_state) {
        FlattenedState f_ = f(x_, i_in);
        x_.oplus(f_, dt);
      }

      if (prop_cov) {
        FlattenedState f_ = f(x_, i_in);
        // state x_before = x_;

        CovarianceJacobian f_x_ = f_x(x_, i_in);
        Covariance f_x_final;
        F_x1 = Covariance::Identity();
        for (auto it = x_.vect_state.begin(); it != x_.vect_state.end(); it++) {
          int idx = (*it).first.first;
          int dim = (*it).first.second;
          int dof = (*it).second;
          for (int i = 0; i < StateDof; i++) {
            for (int j = 0; j < dof; j++) {
              f_x_final(idx + j, i) = f_x_(dim + j, i);
            }
          }
        }

        Matrix<Scalar, 3, 3> res_temp_SO3;
        MTK::vect<3, Scalar> seg_SO3;
        for (auto it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++) {
          int idx = (*it).first;
          int dim = (*it).second;
          for (int i = 0; i < 3; i++) {
            seg_SO3(i) = -1 * f_(dim + i) * dt;
          }
          F_x1.template block<3, 3>(idx, idx) = MTK::SO3<Scalar>::exp(
              seg_SO3);  // res.normalized().toRotationMatrix();
          res_temp_SO3 = MTK::A_matrix(seg_SO3);
          for (int i = 0; i < StateDof; i++) {
            f_x_final.template block<3, 1>(
                idx, i) = res_temp_SO3 * (f_x_.template block<3, 1>(dim, i));
          }
        }

        F_x1 += f_x_final * dt;
        P_ = F_x1 * P_ * (F_x1).transpose() + Q * (dt * dt);
      }
    }

    bool update_iterated_dyn_share_modified() {
      dyn_share_modified<Scalar> dyn_share;
      state x_propagated = x_;
      int dof_Measurement;
      double m_noise;
      for (int i = 0; i < maximum_iter; i++) {
        dyn_share.valid = true;
        h_dyn_share_modified_1(x_, P_.template block<3, 3>(0, 0),
                               P_.template block<3, 3>(3, 3), dyn_share);
        if (!dyn_share.valid) {
          return false;
          // continue;
        }
        Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> z = dyn_share.z;
        Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> h_x = dyn_share.h_x;
        dof_Measurement = h_x.rows();
        m_noise = dyn_share.M_Noise;
        // dof_Measurement_noise = dyn_share.R.rows();
        // x_.boxminus(dx, x_propagated);
        // dx_new = dx;
        // P_ = P_propagated;

        Matrix<Scalar, StateDof, Eigen::Dynamic> PHT;
        Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> HPHT;
        Matrix<Scalar, StateDof, Eigen::Dynamic> K_;
        if (StateDof > dof_Measurement) {
          PHT = P_.template block<StateDof, 12>(0, 0) * h_x.transpose();
          HPHT = h_x * PHT.topRows(12);
          for (int m = 0; m < dof_Measurement; m++) {
            HPHT(m, m) += m_noise;
          }
          K_ = PHT * HPHT.inverse();
        } else {
          Matrix<Scalar, 12, 12> HTH = m_noise * h_x.transpose() * h_x;
          Matrix<Scalar, StateDof, StateDof> P_inv = P_.inverse();
          P_inv.template block<12, 12>(0, 0) += HTH;
          P_inv = P_inv.inverse();
          K_ = P_inv.template block<StateDof, 12>(0, 0) * h_x.transpose()
               * m_noise;
        }
        Matrix<Scalar, StateDof, 1> dx_ = K_ * z;
        // dx_new;
        // state x_before = x_;

        x_.boxplus(dx_);
        {
          P_ = P_ - K_ * h_x * P_.template block<12, StateDof>(0, 0);
        }
      }
      return true;
    }

    void update_iterated_dyn_share_IMU() {
      dyn_share_modified<Scalar> dyn_share;
      for (int i = 0; i < maximum_iter; i++) {
        dyn_share.valid = true;
        h_dyn_share_modified_2(x_, dyn_share);

        Matrix<Scalar, 6, 1> z = dyn_share.z_IMU;

        Matrix<double, 30, 6> PHT;
        Matrix<double, 6, 30> HP;
        Matrix<double, 6, 6> HPHT;
        PHT.setZero();
        HP.setZero();
        HPHT.setZero();
        for (int l_ = 0; l_ < 6; l_++) {
          if (!dyn_share.satu_check[l_]) {
            PHT.col(l_) = P_.col(15 + l_) + P_.col(24 + l_);
            HP.row(l_) = P_.row(15 + l_) + P_.row(24 + l_);
          }
        }
        for (int l_ = 0; l_ < 6; l_++) {
          if (!dyn_share.satu_check[l_]) {
            HPHT.col(l_) = HP.col(15 + l_) + HP.col(24 + l_);
          }
          HPHT(l_, l_) += dyn_share.R_IMU(l_);  //, l);
        }
        Eigen::Matrix<double, 30, 6> K = PHT * HPHT.inverse();

        Matrix<Scalar, StateDof, 1> dx_ = K * z;

        P_ -= K * HP;
        x_.boxplus(dx_);
      }
      return;
    }

    void change_x(state& input_state) {
      x_ = input_state;

      if ((!x_.vect_state.size()) && (!x_.SO3_state.size())
          && (!x_.S2_state.size()) && (!x_.SEN_state.size())) {
        x_.build_S2_state();
        x_.build_SO3_state();
        x_.build_vect_state();
        x_.build_SEN_state();
      }
    }

    void change_P(Covariance& input_cov) {
      P_ = input_cov;
    }

    const state& get_x() const {
      return x_;
    }
    const Covariance& get_P() const {
      return P_;
    }
    state x_;
    Covariance P_;

  private:
    measurement m_;
    SparseMatrixType l_;
    SparseMatrixType f_x_1;
    SparseMatrixType f_x_2;
    Covariance F_x1 = Covariance::Identity();
    Covariance F_x2 = Covariance::Identity();
    Covariance L_ = Covariance::Identity();

    ProcessModel f;
    ProcessMatrix1 f_x;
    ProcessMatrix2 f_w;

    MeasurementMatrix1 h_x;
    MeasurementMatrix2 h_v;

    MeasurementMatrix1Dyn h_x_dyn;
    MeasurementMatrix2Dyn h_v_dyn;

    MeasurementModelDynShareModifiedCov h_dyn_share_modified_1;

    MeasurementModelDynShareModified h_dyn_share_modified_2;

    MeasurementModelDynShareModified h_dyn_share_modified_3;

    int maximum_iter = 0;
    Scalar limit[StateDof];

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}  // namespace esekfom
