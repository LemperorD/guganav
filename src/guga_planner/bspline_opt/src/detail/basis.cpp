#include "bspline_opt/detail/basis.hpp"
#include "bspline_opt/detail/common.hpp"
#include <cstddef>

namespace bspline_opt
{
namespace detail
{

int findSpan(double u, const Eigen::RowVectorXd & knots, int n, int p)
{
  if (u >= knots(n + 1)) {return n;}
  if (u <= knots(p)) {return p;}
  int lo = p;
  int hi = n + 1;
  while (lo < hi - 1) {
    int mid = (lo + hi) / 2;
    if (u < knots(mid)) {hi = mid;} else {lo = mid;}
  }
  return lo;
}


void basisValuesAt(
  double u, const Eigen::RowVectorXd & knots, int p, int M,
  std::vector<double> & out)
{
  out.assign(static_cast<size_t>(M), 0.0);
  const int n = M - 1;
  const int j = findSpan(u, knots, n, p);

  std::vector<double> N(static_cast<size_t>(p + 1), 0.0);
  N[0] = 1.0;
  for (int k = 1; k <= p; ++k) {
    double saved = 0.0;
    for (int r = 0; r < k; ++r) {
      double tmp = N[static_cast<size_t>(r)];
      double den = knots(j + 1 + r) - knots(j - k + 1 + r);
      double right = (den != 0.0) ? (knots(j + 1 + r) - u) / den : 0.0;
      double left = (den != 0.0) ? (u - knots(j - k + 1 + r)) / den : 0.0;
      N[static_cast<size_t>(r)] = saved + right * tmp;
      saved = left * tmp;
    }
    N[static_cast<size_t>(k)] = saved;
  }

  for (int r = 0; r <= p; ++r) {
    int i = j - p + r;
    if (i >= 0 && i < M) {
      out[static_cast<size_t>(i)] = N[static_cast<size_t>(r)];
    }
  }
}


void basisDerivsAt(
  double u, const Eigen::RowVectorXd & knots, int p, int M,
  std::vector<double> & N, std::vector<double> & N1, std::vector<double> & N2)
{
  // NURBS Book Algorithm A2.3 (与 Eigen::Spline::basisFunctionDerivatives
  // 相同的 ndu 表算法), 正确处理钳制端 (u=0/u=1) 的单边导数。
  const int n = M - 1;
  const int span = findSpan(u, knots, n, p);
  constexpr int kOrder = 2;

  std::vector<double> ndu(static_cast<size_t>((p + 1) * (p + 1)), 0.0);
  std::vector<double> left(static_cast<size_t>(p + 1), 0.0);
  std::vector<double> right(static_cast<size_t>(p + 1), 0.0);

  ndu[0] = 1.0;
  for (int j = 1; j <= p; ++j) {
    left[static_cast<size_t>(j)] = u - knots(span + 1 - j);
    right[static_cast<size_t>(j)] = knots(span + j) - u;
    double saved = 0.0;
    for (int r = 0; r < j; ++r) {
      ndu[static_cast<size_t>(j) * (p + 1) + r] =
        right[static_cast<size_t>(r + 1)] + left[static_cast<size_t>(j - r)];
      const double temp =
        ndu[static_cast<size_t>(r) * (p + 1) + (j - 1)] /
        ndu[static_cast<size_t>(j) * (p + 1) + r];
      ndu[static_cast<size_t>(r) * (p + 1) + j] =
        saved + right[static_cast<size_t>(r + 1)] * temp;
      saved = left[static_cast<size_t>(j - r)] * temp;
    }
    ndu[static_cast<size_t>(j) * (p + 1) + j] = saved;
  }

  N.assign(static_cast<size_t>(M), 0.0);
  N1.assign(static_cast<size_t>(M), 0.0);
  N2.assign(static_cast<size_t>(M), 0.0);
  for (int j = 0; j <= p; ++j) {
    const int i = span - p + j;
    if (i >= 0 && i < M) {
      N[static_cast<size_t>(i)] =
        ndu[static_cast<size_t>(j) * (p + 1) + p];
    }
  }

  std::vector<double> a(2 * static_cast<size_t>(p + 1), 0.0);
  auto aAt = [&](int row, int col) -> double & {
      return a[static_cast<size_t>(row) * (p + 1) + col];
    };

  for (int r = 0; r <= p; ++r) {
    int s1 = 0;
    int s2 = 1;
    aAt(0, 0) = 1.0;

    for (int k = 1; k <= kOrder; ++k) {
      double d = 0.0;
      const int rk = r - k;
      const int pk = p - k;

      if (r >= k) {
        aAt(s2, 0) = aAt(s1, 0) /
          ndu[static_cast<size_t>(pk + 1) * (p + 1) + rk];
        d = aAt(s2, 0) * ndu[static_cast<size_t>(rk) * (p + 1) + pk];
      }

      const int j1 = (rk >= -1) ? 1 : -rk;
      const int j2 = (r - 1 <= pk) ? k - 1 : p - r;
      for (int j = j1; j <= j2; ++j) {
        aAt(s2, j) = (aAt(s1, j) - aAt(s1, j - 1)) /
          ndu[static_cast<size_t>(pk + 1) * (p + 1) + (rk + j)];
        d += aAt(s2, j) * ndu[static_cast<size_t>(rk + j) * (p + 1) + pk];
      }

      if (r <= pk) {
        aAt(s2, k) = -aAt(s1, k - 1) /
          ndu[static_cast<size_t>(pk + 1) * (p + 1) + r];
        d += aAt(s2, k) * ndu[static_cast<size_t>(r) * (p + 1) + pk];
      }

      const int i = span - p + r;
      if (i >= 0 && i < M) {
        if (k == 1) {
          N1[static_cast<size_t>(i)] = static_cast<double>(p) * d;
        } else if (k == 2) {
          N2[static_cast<size_t>(i)] =
            static_cast<double>(p * (p - 1)) * d;
        }
      }
      std::swap(s1, s2);
    }
  }
}

}  // namespace detail
}  // namespace bspline_opt
