#include <cmath>

#include <gtest/gtest.h>

#include "point_lio/core/ManifoldEskf.h"
#include "point_lio/utilities/common_lib.h"

namespace {

ImuParams makeImuParams() {
  ImuParams params;
  params.processor.gravity = V3D(0.0, 0.0, -9.81);
  params.processor.gravity_magnitude = 9.81;
  return params;
}

}  // namespace

TEST(EskfInit, NoStaticSamplesKeepsIdentity) {
  ManifoldEskf eskf;
  ImuInitEstimate estimate;  // static_count = 0
  eskf.initState(estimate, makeImuParams());

  const State& state = eskf.get_x();
  EXPECT_LT((state.rot - IMatrix3).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((state.acc - V3D(0.0, 0.0, 9.81)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT(state.bg.cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT(state.ba.cwiseAbs().maxCoeff(), 1e-12);
}

TEST(EskfInit, LevelStaticAlignsGravity) {
  ManifoldEskf eskf;
  ImuInitEstimate estimate;
  estimate.static_count = 100;
  estimate.mean_acc = V3D(0.0, 0.0, 9.81);
  estimate.mean_gyr = V3D(0.001, -0.002, 0.003);
  eskf.initState(estimate, makeImuParams());

  const State& state = eskf.get_x();
  EXPECT_LT((state.rot - IMatrix3).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((state.acc - estimate.mean_acc).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((state.bg - estimate.mean_gyr).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(EskfInit, TiltedStaticAlignsGravityDirection) {
  ManifoldEskf eskf;
  ImuInitEstimate estimate;
  estimate.static_count = 100;
  const double angle = M_PI / 6.0;
  estimate.mean_acc =
      V3D(0.0, 9.81 * std::sin(angle), 9.81 * std::cos(angle));
  eskf.initState(estimate, makeImuParams());

  const State& state = eskf.get_x();
  // rot 把机体测得的重力方向转到世界系重力方向 (0, 0, -g)。
  const V3D measured_gravity = -estimate.mean_acc.normalized() * 9.81;
  const V3D aligned = state.rot * measured_gravity;
  EXPECT_LT((aligned - V3D(0.0, 0.0, -9.81)).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_LT((state.acc - estimate.mean_acc).cwiseAbs().maxCoeff(), 1e-6);
}
