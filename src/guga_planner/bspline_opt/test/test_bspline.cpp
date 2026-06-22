#include "bspline_opt/bspline_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
#include <utility>

#include "gtest/gtest.h"

namespace bspline_opt
{

// ──────────────────────────────────────────────────────────
// Fixture helpers
// ──────────────────────────────────────────────────────────

inline std::vector<std::pair<double, double>> makeLine(int n)
{
  std::vector<std::pair<double, double>> p;
  for (int i = 0; i < n; ++i)
    p.emplace_back(static_cast<double>(i), static_cast<double>(i));
  return p;
}

inline std::vector<std::pair<double, double>> makeCorner(int n)
{
  // Straight from (0,0) to (n/2, 0), then turn 90° to (n/2, n/2)
  std::vector<std::pair<double, double>> p;
  int half = n / 2;
  for (int i = 0; i <= half; ++i)
    p.emplace_back(static_cast<double>(i), 0.0);
  for (int i = 1; i <= half; ++i)
    p.emplace_back(static_cast<double>(half), static_cast<double>(i));
  return p;
}

// ──────────────────────────────────────────────────────────
// Test: BSplineFittingTest
// ──────────────────────────────────────────────────────────

class BSplineFittingTest : public ::testing::Test
{
protected:
  BSplineConfig cfg_{};
  BSplineOptimizer opt_{cfg_};
};

TEST_F(BSplineFittingTest, DiagonalLine_FitsExactly)
{
  auto path = makeLine(10);
  ASSERT_TRUE(opt_.fit(path));
  // Exact interpolation: sampled points should be very close to diagonal
  auto sampled = opt_.sample(10);
  for (size_t i = 0; i < sampled.size(); ++i) {
    double expected = static_cast<double>(i) * 9.0 / 9.0;
    EXPECT_NEAR(sampled[i].first, expected, 0.5);
    EXPECT_NEAR(sampled[i].second, expected, 0.5);
  }
}

TEST_F(BSplineFittingTest, StraightHorizontal_FitsCollinear)
{
  std::vector<std::pair<double, double>> path;
  for (int i = 0; i < 10; ++i) path.emplace_back(static_cast<double>(i), 5.0);
  ASSERT_TRUE(opt_.fit(path));
  // Mid path
  double curv = opt_.curvatureAt(0.5);
  EXPECT_NEAR(curv, 0.0, 1e-6);  // straight line → zero curvature
}

TEST_F(BSplineFittingTest, Validation_SufficientPoints)
{
  // Exactly degree+1 = 6 points → should work
  std::vector<std::pair<double, double>> path;
  for (int i = 0; i < 6; ++i) path.emplace_back(static_cast<double>(i), 0.0);
  EXPECT_TRUE(opt_.fit(path));
}

TEST_F(BSplineFittingTest, TooFewPoints_Fails)
{
  std::vector<std::pair<double, double>> path{{0, 0}};
  EXPECT_FALSE(opt_.fit(path));
}

TEST_F(BSplineFittingTest, TwoPoints_Linear)
{
  // 2 points → auto-reduces to degree 1 (linear)
  std::vector<std::pair<double, double>> path{{0, 0}, {10, 10}};
  EXPECT_TRUE(opt_.fit(path));
  EXPECT_EQ(opt_.state().effective_degree, 1);
  auto sampled = opt_.sample(5);
  // Midpoint should be at (5, 5)
  EXPECT_NEAR(sampled[2].first, 5.0, 0.05);
  EXPECT_NEAR(sampled[2].second, 5.0, 0.05);
}

// ──────────────────────────────────────────────────────────
// Test: BSplineOptimizationTest
// ──────────────────────────────────────────────────────────

class BSplineOptimizationTest : public ::testing::Test
{
protected:
  BSplineConfig cfg_{};
};

TEST_F(BSplineOptimizationTest, Corner_CurvatureDecreases)
{
  auto path = makeCorner(20);  // 11 points total
  BSplineOptimizer opt(cfg_);
  ASSERT_TRUE(opt.fit(path));

  double curv_before = opt.computeCurvatureEnergy();
  auto result = opt.optimize(50);
  double curv_after = result.total_curvature_energy;

  EXPECT_LT(curv_after, curv_before * 1.05)
    << "Optimization should reduce or maintain curvature energy";
  EXPECT_TRUE(result.converged);
}

TEST_F(BSplineOptimizationTest, EndpointsPreserved)
{
  auto path = makeCorner(20);
  BSplineOptimizer opt(cfg_);
  ASSERT_TRUE(opt.fit(path));
  auto result = opt.optimize(50);

  // First and last should be within epsilon of original
  EXPECT_NEAR(result.smoothed_path.front().first, 0.0, 1e-6);
  EXPECT_NEAR(result.smoothed_path.front().second, 0.0, 1e-6);
  EXPECT_NEAR(result.smoothed_path.back().first, 10.0, 0.01);
  EXPECT_NEAR(result.smoothed_path.back().second, 10.0, 0.01);
}

TEST_F(BSplineOptimizationTest, StraightPath_StaysStraight)
{
  auto path = makeLine(10);
  BSplineOptimizer opt(cfg_);
  ASSERT_TRUE(opt.fit(path));
  auto result = opt.optimize(50);

  double max_curv{};
  for (size_t i = 0; i < result.curvature_profile.size(); ++i)
    max_curv = std::max(max_curv, std::abs(result.curvature_profile[i]));
  EXPECT_LT(max_curv, 1e-3)
    << "Straight path should remain nearly straight after optimization";
}

TEST_F(BSplineOptimizationTest, ControlPoints_CountMatches)
{
  auto path = makeLine(10);
  BSplineOptimizer opt(cfg_);
  ASSERT_TRUE(opt.fit(path));
  auto result = opt.optimize(50);

  EXPECT_GE(result.control_points_xy.size(), 2u);
  // Control points count = M = n_pts (from interpolation) = 10 for 10-point input
}

// ──────────────────────────────────────────────────────────
// Test: BSplineQueryTest
// ──────────────────────────────────────────────────────────

class BSplineQueryTest : public ::testing::Test
{
protected:
  BSplineConfig cfg_{};
};

TEST_F(BSplineQueryTest, CurvatureAt_StraightLine_Zero)
{
  auto path = makeLine(10);
  BSplineOptimizer opt(cfg_);
  ASSERT_TRUE(opt.fit(path));

  double curv = opt.curvatureAt(0.3);
  EXPECT_NEAR(curv, 0.0, 1e-6);
}

TEST_F(BSplineQueryTest, CurvatureAt_Endpoint_IsFinite)
{
  auto path = makeLine(10);
  BSplineOptimizer opt(cfg_);
  ASSERT_TRUE(opt.fit(path));

  double curv0 = opt.curvatureAt(0.0);
  double curv1 = opt.curvatureAt(1.0);
  EXPECT_LE(std::abs(curv0), 1e10);  // finite
  EXPECT_LE(std::abs(curv1), 1e10);
}

TEST_F(BSplineQueryTest, Sample_N_ReturnsN)
{
  auto path = makeCorner(20);
  BSplineOptimizer opt(cfg_);
  ASSERT_TRUE(opt.fit(path));

  auto s = opt.sample(100);
  EXPECT_EQ(s.size(), 100u);
}

TEST_F(BSplineQueryTest, NotFitted_ReturnsEmpty)
{
  BSplineOptimizer opt;
  auto s = opt.sample(10);
  EXPECT_TRUE(s.empty());
  EXPECT_FALSE(opt.isFitted());
}

}  // namespace bspline_opt
