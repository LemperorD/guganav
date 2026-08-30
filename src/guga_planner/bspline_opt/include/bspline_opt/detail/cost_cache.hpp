#pragma once

#include <Eigen/Core>
#include <vector>

namespace bspline_opt
{
namespace detail
{

struct BandRow
{
  int start{};
  int count{};
  double val[8]{};
};

struct CostCache
{
  int M{};
  int Ks{50};
  int Ke{200};
  std::vector<BandRow> d2_smooth{};
  std::vector<BandRow> b_dist{};
  std::vector<Eigen::Vector2d> dist_q{};
  std::vector<BandRow> b_esdf{};
};

void rowFromBasis(const std::vector<double> & b, int M, BandRow & row);

CostCache buildCostCache(
  const Eigen::RowVectorXd & knots, int M,
  const std::vector<Eigen::Vector2d> & orig_points,
  const Eigen::VectorXd & orig_params);

double bandedDot(const BandRow & row, const Eigen::MatrixXd & ctrl, int dim);

void fillCtrl(
  const std::vector<double> & params, double first_x, double first_y, double last_x,
  double last_y, int M, Eigen::MatrixXd & ctrl);

}  // namespace detail
}  // namespace bspline_opt
