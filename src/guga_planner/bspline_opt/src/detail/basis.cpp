#include "bspline_opt/basis_cache.hpp"
#include <cstddef>


namespace bspline_opt::detail
{

int BasisCache::findSpan(double u, const Eigen::RowVectorXd & knots, int n, int p)
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

void BasisCache::basisDerivsAt(
  double u, const Eigen::RowVectorXd & knots, int p, int M,
  std::vector<double> & N, std::vector<double> & N1, std::vector<double> & N2)
{
  // NURBS Book Algorithm A2.3 (与 Eigen::Spline::basisFunctionDerivatives
  // 相同的 ndu 表算法), 正确处理钳制端 (u=0/u=1) 的单边导数。
  const int n = M - 1;
  const int span = findSpan(u, knots, n, p);
  constexpr int korder = 2;

  std::vector<double> ndu(static_cast<size_t>((p + 1) * (p + 1)), 0.0);
  std::vector<double> left(static_cast<size_t>(p + 1), 0.0);
  std::vector<double> right(static_cast<size_t>(p + 1), 0.0);

  ndu[0] = 1.0;
  for (int j = 1; j <= p; ++j) {
    left[static_cast<size_t>(j)] = u - knots(span + 1 - j);
    right[static_cast<size_t>(j)] = knots(span + j) - u;
    double saved_left = 0.0;
    for (int r = 0; r < j; ++r) {
      ndu[static_cast<size_t>(j) * (p + 1) + r] =
        right[static_cast<size_t>(r + 1)] + left[static_cast<size_t>(j - r)];
      const double temp =
        ndu[static_cast<size_t>(r) * (p + 1) + (j - 1)] /
        ndu[static_cast<size_t>(j) * (p + 1) + r];
      ndu[static_cast<size_t>(r) * (p + 1) + j] =
        saved_left + right[static_cast<size_t>(r + 1)] * temp;
      saved_left = left[static_cast<size_t>(j - r)] * temp;
    }
    ndu[static_cast<size_t>(j) * (p + 1) + j] = saved_left;
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
  auto find_index = [&](int row, int col) -> double & {
      return a[static_cast<size_t>(row) * (p + 1) + col];
    };

  for (int r = 0; r <= p; ++r) {
    int s1 = 0;
    int s2 = 1;
   find_index(0, 0) = 1.0;

    for (int k = 1; k <= korder; ++k) {
      double d = 0.0;
      const int rk = r - k;
      const int pk = p - k;

      if (r >= k) {
       find_index(s2, 0) = find_index(s1, 0) /
          ndu[static_cast<size_t>(pk + 1) * (p + 1) + rk];
        d = find_index(s2, 0) * ndu[static_cast<size_t>(rk) * (p + 1) + pk];
      }

      const int j1 = (rk >= -1) ? 1 : -rk;
      const int j2 = (r - 1 <= pk) ? k - 1 : p - r;
      for (int j = j1; j <= j2; ++j) {
        find_index(s2, j) = (find_index(s1, j) - find_index(s1, j - 1)) /
          ndu[static_cast<size_t>(pk + 1) * (p + 1) + (rk + j)];
        d += find_index(s2, j) * ndu[static_cast<size_t>(rk + j) * (p + 1) + pk];
      }

      if (r <= pk) {
       find_index(s2, k) =  -find_index(s1, k - 1) /
          ndu[static_cast<size_t>(pk + 1) * (p + 1) + r];
        d += find_index(s2, k) * ndu[static_cast<size_t>(r) * (p + 1) + pk];
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

}  // namespace bspline_opt::detail

