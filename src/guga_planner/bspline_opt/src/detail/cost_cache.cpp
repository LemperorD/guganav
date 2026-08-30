#include "bspline_opt/detail/cost_cache.hpp"
#include "bspline_opt/detail/basis.hpp"
#include "bspline_opt/detail/common.hpp"
#include <cstddef>

namespace bspline_opt
{
namespace detail
{

void rowFromBasis(const std::vector<double> & b, int M, BandRow & row)
{
  row.start = 0;
  row.count = 0;
  for (int i = 0; i < M; ++i) {
    double v = b[static_cast<size_t>(i)];
    if (std::abs(v) > 1e-14) {
      if (row.count == 0) {row.start = i;}
      if (row.count < 8) {row.val[row.count] = v;}
      ++row.count;
    }
  }
}


CostCache buildCostCache(
  const Eigen::RowVectorXd & knots, int M,
  const std::vector<Eigen::Vector2d> & orig_points,
  const Eigen::VectorXd & orig_params)
{
  CostCache cc;
  cc.M = M;

  std::vector<double> N, N1, N2;
  auto pushRow = [&](double u, bool second_deriv, std::vector<BandRow> & rows) {
      basisDerivsAt(u, knots, kSplineDegree, M, N, N1, N2);
      BandRow r;
      rowFromBasis(second_deriv ? N2 : N, M, r);
      rows.push_back(r);
    };

  cc.d2_smooth.reserve(static_cast<size_t>(cc.Ks + 1));
  for (int k = 0; k <= cc.Ks; ++k) {
    pushRow(static_cast<double>(k) / cc.Ks, true, cc.d2_smooth);
  }

  // 距离项采样点: 均匀降采样到 ≤256 个原始航点, 控制代价有界。
  const size_t n = orig_points.size();
  const size_t stride = (n > 256) ? ((n + 255) / 256) : 1;
  for (size_t i = 0; i < n; i += stride) {
    pushRow(orig_params(static_cast<Eigen::Index>(i)), false, cc.b_dist);
    cc.dist_q.emplace_back(orig_points[i]);
  }
  if (n > 0 && (n - 1) % stride != 0) {
    pushRow(orig_params(static_cast<Eigen::Index>(n - 1)), false, cc.b_dist);
    cc.dist_q.emplace_back(orig_points[n - 1]);
  }

  cc.b_esdf.reserve(static_cast<size_t>(cc.Ke + 1));
  for (int k = 0; k <= cc.Ke; ++k) {
    pushRow(static_cast<double>(k) / cc.Ke, false, cc.b_esdf);
  }

  return cc;
}


double bandedDot(const BandRow & row, const Eigen::MatrixXd & ctrl, int dim)
{
  double s{};
  for (int k = 0; k < row.count; ++k) {
    s += row.val[k] * ctrl(dim, row.start + k);
  }
  return s;
}


void fillCtrl(
  const std::vector<double> & params, double first_x, double first_y,
  double last_x, double last_y, int M, Eigen::MatrixXd & ctrl)
{
  ctrl.resize(2, M);
  ctrl(0, 0) = first_x;  ctrl(1, 0) = first_y;
  ctrl(0, M - 1) = last_x;  ctrl(1, M - 1) = last_y;
  for (int i = 0; i < M - 2; ++i) {
    ctrl(0, i + 1) = params[2 * i];
    ctrl(1, i + 1) = params[2 * i + 1];
  }
}

}  // namespace detail
}  // namespace bspline_opt
