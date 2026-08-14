#include "bspline_opt/bspline_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include <unsupported/Eigen/Splines>

namespace bspline_opt
{

namespace
{

// ══════════════════════════════════════════════════════════
// 障碍物避让辅助函数
// ══════════════════════════════════════════════════════════

// 读取代价地图中整数格元坐标处的值 (带边界检查)
inline unsigned char cellCost(
  const unsigned char * cmap, int w, int h, int cx, int cy)
{
  if (!cmap || cx < 0 || cx >= w || cy < 0 || cy >= h) {return 255;}
  return cmap[static_cast<size_t>(cy * w + cx)];
}

// 检查世界坐标点是否落入障碍物格元
inline bool inObstacle(
  const unsigned char * cmap, int w, int h, double px, double py)
{
  int cx = static_cast<int>(px);
  int cy = static_cast<int>(py);
  return cellCost(cmap, w, h, cx, cy) >= 253;
}

// 将单个点投射到最近空闲格元 (螺旋搜索, 最大半径 8)
// 返回 true 表示投射成功找到了空闲格元
inline bool projectPointToFree(
  const unsigned char * cmap, int w, int h, double & px, double & py)
{
  int ix = static_cast<int>(px);
  int iy = static_cast<int>(py);
  if (!inObstacle(cmap, w, h, px, py)) {return true;}

  for (int r = 1; r <= 8; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) < r && std::abs(dy) < r) {continue;}
        int tx = ix + dx;
        int ty = iy + dy;
        if (cellCost(cmap, w, h, tx, ty) < 253) {
          px = static_cast<double>(tx) + 0.5;
          py = static_cast<double>(ty) + 0.5;
          return true;
        }
      }
    }
  }
  return false;
}

// ══════════════════════════════════════════════════════════
// ESDF 查询辅助函数
// ══════════════════════════════════════════════════════════

// 双线性插值从 ESDF 距离场查询世界坐标处的距离
inline float esdfDistanceAt(
  const float * esdf_dist, int esdf_w, int esdf_h,
  double resolution, double origin_x, double origin_y,
  double wx, double wy, double max_dist)
{
  if (!esdf_dist) {return static_cast<float>(max_dist);}

  double fx = (wx - origin_x) / resolution;
  double fy = (wy - origin_y) / resolution;

  int ix = static_cast<int>(fx);
  int iy = static_cast<int>(fy);

  if (ix < 0 || ix + 1 >= esdf_w || iy < 0 || iy + 1 >= esdf_h) {
    return static_cast<float>(max_dist);
  }

  double dx = fx - static_cast<double>(ix);
  double dy = fy - static_cast<double>(iy);

  size_t idx00 = static_cast<size_t>(iy) * static_cast<size_t>(esdf_w) +
    static_cast<size_t>(ix);
  size_t idx10 = idx00 + 1;
  size_t idx01 = idx00 + static_cast<size_t>(esdf_w);
  size_t idx11 = idx01 + 1;

  float d00 = esdf_dist[idx00];
  float d10 = esdf_dist[idx10];
  float d01 = esdf_dist[idx01];
  float d11 = esdf_dist[idx11];

  float d0 = static_cast<float>(static_cast<double>(d00) * (1.0 - dx) +
    static_cast<double>(d10) * dx);
  float d1 = static_cast<float>(static_cast<double>(d01) * (1.0 - dx) +
    static_cast<double>(d11) * dx);

  return d0 * static_cast<float>(1.0 - dy) + d1 * static_cast<float>(dy);
}

// 双线性插值 ESDF 梯度场 (世界坐标 → (gx, gy), 单位: 距离(米)/格元)
inline void esdfGradientAt(
  const float * esdf_gx, const float * esdf_gy,
  int esdf_w, int esdf_h, double resolution, double origin_x, double origin_y,
  double wx, double wy, double & gx, double & gy)
{
  gx = 0.0; gy = 0.0;
  if (!esdf_gx || !esdf_gy) {return;}

  double fx = (wx - origin_x) / resolution;
  double fy = (wy - origin_y) / resolution;

  int ix = static_cast<int>(fx);
  int iy = static_cast<int>(fy);

  if (ix < 0 || ix + 1 >= esdf_w || iy < 0 || iy + 1 >= esdf_h) {return;}

  double dx = fx - static_cast<double>(ix);
  double dy = fy - static_cast<double>(iy);

  size_t idx00 = static_cast<size_t>(iy) * static_cast<size_t>(esdf_w) +
    static_cast<size_t>(ix);
  size_t idx10 = idx00 + 1;
  size_t idx01 = idx00 + static_cast<size_t>(esdf_w);
  size_t idx11 = idx01 + 1;

  float g00x = esdf_gx[idx00]; float g10x = esdf_gx[idx10];
  float g01x = esdf_gx[idx01]; float g11x = esdf_gx[idx11];
  float g0x = static_cast<float>(static_cast<double>(g00x) * (1.0 - dx) +
    static_cast<double>(g10x) * dx);
  float g1x = static_cast<float>(static_cast<double>(g01x) * (1.0 - dx) +
    static_cast<double>(g11x) * dx);
  gx = static_cast<double>(g0x * static_cast<float>(1.0 - dy) +
    g1x * static_cast<float>(dy));

  float g00y = esdf_gy[idx00]; float g10y = esdf_gy[idx10];
  float g01y = esdf_gy[idx01]; float g11y = esdf_gy[idx11];
  float g0y = static_cast<float>(static_cast<double>(g00y) * (1.0 - dx) +
    static_cast<double>(g10y) * dx);
  float g1y = static_cast<float>(static_cast<double>(g01y) * (1.0 - dx) +
    static_cast<double>(g11y) * dx);
  gy = static_cast<double>(g0y * static_cast<float>(1.0 - dy) +
    g1y * static_cast<float>(dy));
}

// ══════════════════════════════════════════════════════════
// B-spline 基函数预计算
//
// 梯度下降的代价函数在固定的参数网格上反复求值, 而样条对控制点是
// 线性的: C(u) = Σ N_i(u)·c_i。把基函数值 (及其二阶导) 一次性算出,
// 后续每次代价/梯度计算都只是几条带宽 ≤ degree+1 的带状点积,
// 不再为每个参数扰动重新构造/求值 Eigen Spline。
// ══════════════════════════════════════════════════════════

constexpr int kSplineDegree = 7;

// 查找参数 u 所在的节点区间 [knots(j), knots(j+1))
inline int findSpan(double u, const Eigen::RowVectorXd & knots, int n, int p)
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

// 在参数 u 处计算阶数 p 的基函数值, 输出按控制点索引 (0..M-1)。
inline void basisValuesAt(
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

// 在参数 u 处计算基函数值 N、一阶导 N1、二阶导 N2 (均按控制点索引)。
inline void basisDerivsAt(
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

// 一条带宽 ≤ degree+1 的基函数行 (只存非零段)
struct BandRow
{
  int start{};
  int count{};
  double val[8]{};
};

inline void rowFromBasis(const std::vector<double> & b, int M, BandRow & row)
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

// 代价函数求值所需的全部预计算数据
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

// 固定参数网格上的基函数行 (位置/二阶导), 每次 optimize() 构建一次。
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

inline double bandedDot(const BandRow & row, const Eigen::MatrixXd & ctrl, int dim)
{
  double s{};
  for (int k = 0; k < row.count; ++k) {
    s += row.val[k] * ctrl(dim, row.start + k);
  }
  return s;
}

inline void fillCtrl(
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

static double evalCost(
  const std::vector<double> & params, const CostCache & cc,
  double first_x, double first_y, double last_x, double last_y,
  double w_smooth, double w_dist,
  const float * esdf_dist, int esdf_w, int esdf_h,
  double esdf_res, double esdf_ox, double esdf_oy, double esdf_max,
  double esdf_safe_dist, double w_esdf)
{
  const int M = cc.M;
  Eigen::MatrixXd ctrl;
  fillCtrl(params, first_x, first_y, last_x, last_y, M, ctrl);

  double cost{};

  if (w_smooth > 0.0) {
    double s{};
    for (const auto & row : cc.d2_smooth) {
      double ddx = bandedDot(row, ctrl, 0);
      double ddy = bandedDot(row, ctrl, 1);
      s += ddx * ddx + ddy * ddy;
    }
    cost += w_smooth * s / static_cast<double>(cc.d2_smooth.size());
  }

  if (w_dist > 0.0) {
    for (size_t i = 0; i < cc.b_dist.size(); ++i) {
      double px = bandedDot(cc.b_dist[i], ctrl, 0);
      double py = bandedDot(cc.b_dist[i], ctrl, 1);
      double dx = px - cc.dist_q[i].x();
      double dy = py - cc.dist_q[i].y();
      cost += w_dist * (dx * dx + dy * dy);
    }
  }

  // ESDF distance-based obstacle cost (smooth gradient-aware)
  if (w_esdf > 0.0 && esdf_dist) {
    for (const auto & row : cc.b_esdf) {
      double px = bandedDot(row, ctrl, 0);
      double py = bandedDot(row, ctrl, 1);
      double wx = px * esdf_res + esdf_ox;
      double wy = py * esdf_res + esdf_oy;
      float dist = esdfDistanceAt(
        esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy, wx, wy, esdf_max);
      if (dist < static_cast<float>(esdf_safe_dist)) {
        double violation = esdf_safe_dist - static_cast<double>(dist);
        cost += w_esdf * violation * violation;
      }
    }
  }

  return cost;
}

// 解析梯度: 一次前向 (算每个采样点位置/误差) + 一次反向 (沿非零基函数
// 累加), 取代原先对每个参数做两次有限差分。
static void computeGradient(
  const std::vector<double> & params, const CostCache & cc,
  double first_x, double first_y, double last_x, double last_y,
  double w_smooth, double w_dist,
  const float * esdf_dist, const float * esdf_gx, const float * esdf_gy,
  int esdf_w, int esdf_h, double esdf_res, double esdf_ox, double esdf_oy,
  double esdf_max, double esdf_safe_dist, double w_esdf,
  std::vector<double> & grad)
{
  const int M = cc.M;
  grad.assign(static_cast<size_t>(2 * (M - 2)), 0.0);
  Eigen::MatrixXd ctrl;
  fillCtrl(params, first_x, first_y, last_x, last_y, M, ctrl);

  auto accum = [&](int j, double gx, double gy) {
      if (j >= 1 && j <= M - 2) {
        grad[static_cast<size_t>(2 * (j - 1))] += gx;
        grad[static_cast<size_t>(2 * (j - 1) + 1)] += gy;
      }
    };

  if (w_smooth > 0.0) {
    const double fac = 2.0 * w_smooth /
      static_cast<double>(cc.d2_smooth.size());
    for (const auto & row : cc.d2_smooth) {
      double ddx = bandedDot(row, ctrl, 0);
      double ddy = bandedDot(row, ctrl, 1);
      for (int k = 0; k < row.count; ++k) {
        accum(row.start + k, fac * ddx * row.val[k], fac * ddy * row.val[k]);
      }
    }
  }

  if (w_dist > 0.0) {
    for (size_t i = 0; i < cc.b_dist.size(); ++i) {
      const auto & row = cc.b_dist[i];
      double px = bandedDot(row, ctrl, 0);
      double py = bandedDot(row, ctrl, 1);
      double ex = px - cc.dist_q[i].x();
      double ey = py - cc.dist_q[i].y();
      for (int k = 0; k < row.count; ++k) {
        accum(
          row.start + k,
          2.0 * w_dist * ex * row.val[k],
          2.0 * w_dist * ey * row.val[k]);
      }
    }
  }

  if (w_esdf > 0.0 && esdf_dist) {
    for (const auto & row : cc.b_esdf) {
      double px = bandedDot(row, ctrl, 0);
      double py = bandedDot(row, ctrl, 1);
      double wx = px * esdf_res + esdf_ox;
      double wy = py * esdf_res + esdf_oy;
      float dist = esdfDistanceAt(
        esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy, wx, wy, esdf_max);
      if (dist < static_cast<float>(esdf_safe_dist)) {
        double gd_x{};
        double gd_y{};
        if (esdf_gx && esdf_gy) {
          esdfGradientAt(
            esdf_gx, esdf_gy, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy,
            wx, wy, gd_x, gd_y);
        } else {
          // 无梯度场时的回退: 中心差分距离场 (格元单位)
          gd_x = 0.5 * static_cast<double>(
            esdfDistanceAt(
              esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
              esdf_oy, wx + esdf_res, wy, esdf_max) -
            esdfDistanceAt(
              esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
              esdf_oy, wx - esdf_res, wy, esdf_max));
          gd_y = 0.5 * static_cast<double>(
            esdfDistanceAt(
              esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
              esdf_oy, wx, wy + esdf_res, esdf_max) -
            esdfDistanceAt(
              esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
              esdf_oy, wx, wy - esdf_res, esdf_max));
        }
        double fac = -2.0 * w_esdf *
          (esdf_safe_dist - static_cast<double>(dist));
        for (int k = 0; k < row.count; ++k) {
          accum(
            row.start + k, fac * gd_x * row.val[k], fac * gd_y * row.val[k]);
        }
      }
    }
  }
}

static std::vector<double> gradientDescent(
  const std::vector<double> & init_params, const CostCache & cc,
  double first_x, double first_y, double last_x, double last_y,
  double w_smooth, double w_dist,
  const float * esdf_dist, const float * esdf_gx, const float * esdf_gy,
  int esdf_w, int esdf_h, double esdf_res, double esdf_ox, double esdf_oy,
  double esdf_max, double esdf_safe_dist, double w_esdf,
  int max_iters, double corridor_hw, bool & converged, int & iters_out)
{
  std::vector<double> x = init_params;
  const int N = static_cast<int>(x.size());
  if (N == 0) {converged = true; iters_out = 0; return x;}

  double alpha{1e-3};
  constexpr double gtol = 1e-8;
  constexpr int patience = 20;

  double f_best = evalCost(
    x, cc, first_x, first_y, last_x, last_y,
    w_smooth, w_dist, esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy,
    esdf_max, esdf_safe_dist, w_esdf);
  int no_improve{};

  std::vector<double> grad(N);
  std::vector<double> x_try(N);

  int iter{};
  for (; iter < max_iters; ++iter) {
    computeGradient(
      x, cc, first_x, first_y, last_x, last_y,
      w_smooth, w_dist,
      esdf_dist, esdf_gx, esdf_gy, esdf_w, esdf_h,
      esdf_res, esdf_ox, esdf_oy, esdf_max, esdf_safe_dist, w_esdf, grad);

    double g_norm{};
    for (int i = 0; i < N; ++i) {
      g_norm += grad[static_cast<size_t>(i)] * grad[static_cast<size_t>(i)];
    }
    g_norm = std::sqrt(g_norm);
    if (g_norm < gtol) {break;}

    alpha = std::min(alpha * 2.0, 0.1);
    bool found{};
    for (int ls = 0; ls < 15; ++ls) {
      for (int i = 0; i < N; ++i) {
        x_try[static_cast<size_t>(i)] =
          x[static_cast<size_t>(i)] - alpha * grad[static_cast<size_t>(i)];
      }
      for (int i = 0; i < N; ++i) {
        // 紧邻固定起点/终点的控制点收紧走廊, 防止终点附近过度弯曲。
        const int ctrl_j = i / 2 + 1;  // 控制点索引 (1..M-2)
        double corr = corridor_hw;
        if (ctrl_j == 1 || ctrl_j == cc.M - 2) {
          corr = std::min(corr, 0.5);
        }
        // 走廊约束: 限制在初始位置 ± corridor_hw 范围内
        x_try[static_cast<size_t>(i)] = std::clamp(
          x_try[static_cast<size_t>(i)],
          init_params[static_cast<size_t>(i)] - corr,
          init_params[static_cast<size_t>(i)] + corr);
      }
      double f_try = evalCost(
        x_try, cc, first_x, first_y, last_x, last_y,
        w_smooth, w_dist, esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
        esdf_oy, esdf_max, esdf_safe_dist, w_esdf);
      if (f_try < f_best) {
        x = x_try;
        f_best = f_try;
        found = true;
        no_improve = 0;
        break;
      }
      alpha *= 0.5;
    }
    if (!found) {
      no_improve++;
      if (no_improve >= patience) {break;}
    }
  }
  iters_out = iter;
  converged = true;
  return x;
}

}  // namespace

// ══════════════════════════════════════════════════════════
// 公开 API
// ══════════════════════════════════════════════════════════

BSplineOptimizer::BSplineOptimizer(const BSplineConfig & config)
: config_(config)
{
}

bool BSplineOptimizer::fit(
  const std::vector<std::pair<double, double>> & path)
{
  fitted_ = false;
  state_ = {};
  spline_.reset();

  const int n_pts = static_cast<int>(path.size());
  if (n_pts < 2) {return false;}

  // ── 短路径回退 (线性) ──
  if (n_pts < 8) {
    state_.effective_degree = 1;
    state_.original_points.reserve(n_pts);
    for (const auto & [x, y] : path) {
      state_.original_points.emplace_back(x, y);
    }
    state_.control_points.resize(2, 2);
    state_.control_points(0, 0) = path.front().first;
    state_.control_points(1, 0) = path.front().second;
    state_.control_points(0, 1) = path.back().first;
    state_.control_points(1, 1) = path.back().second;
    fitted_ = true;
    return true;
  }

  const int eff_deg = 7;
  state_.effective_degree = eff_deg;

  state_.original_points.reserve(n_pts);
  for (const auto & [x, y] : path) {
    state_.original_points.emplace_back(x, y);
  }

  // Chord-length 参数化
  std::vector<double> arc_lengths(n_pts);
  arc_lengths[0] = 0.0;
  for (int i = 1; i < n_pts; ++i) {
    double dx = path[i].first - path[i - 1].first;
    double dy = path[i].second - path[i - 1].second;
    arc_lengths[i] = arc_lengths[i - 1] + std::hypot(dx, dy);
  }
  state_.total_arc_length = arc_lengths.back();
  if (state_.total_arc_length < 1e-9) {return false;}

  state_.parameters.resize(n_pts);
  for (int i = 0; i < n_pts; ++i) {
    state_.parameters[i] = arc_lengths[i] / state_.total_arc_length;
  }
  state_.parameters[n_pts - 1] = 1.0;

  // 点矩阵 (2 × N)
  Eigen::MatrixXd pts(2, n_pts);
  for (int i = 0; i < n_pts; ++i) {
    pts(0, i) = path[i].first;
    pts(1, i) = path[i].second;
  }

  // 节点平均
  Eigen::RowVectorXd chord_vec(n_pts);
  for (int i = 0; i < n_pts; ++i) {
    chord_vec(i) = state_.parameters(i);
  }
  Eigen::RowVectorXd knot_vec;
  Eigen::KnotAveraging(chord_vec, eff_deg, knot_vec);
  state_.knots = knot_vec;

  // ── 第1步: 通过 SplineFitting 插值全部 N 个航点 ──
  using SplineFitter =
    Eigen::SplineFitting<Eigen::Spline<double, 2, 7>>;
  auto dense_spline = SplineFitter::Interpolate(pts, eff_deg);

  // ── 第2步: 确定控制点数量 ──
  int M = std::min(config_.max_control_points, n_pts);
  if (M < 8) {M = 8;}
  if (M > n_pts) {M = n_pts;}

  // ── 第3步: 在 chord-length 参数处重新采样 M 个控制点 ──
  // 不使用均匀间距, 而是沿 ARC LENGTH 均匀选取原始航点子集对应的参数。
  // 这样可保留路径的几何形状。
  state_.control_points.resize(2, M);
  int best_idx = 0;
  for (int i = 0; i < M; ++i) {
    // 将索引 i (0..M-1) 映射到弧长比例, 再找到最近的 chord 参数
    double target_arc_frac = static_cast<double>(i) / static_cast<double>(M - 1);
    // parameters 单调递增, 用双指针线性扫描代替 O(M·N) 全量搜索。
    while (best_idx + 1 < n_pts &&
      std::abs(state_.parameters(best_idx + 1) - target_arc_frac) <
      std::abs(state_.parameters(best_idx) - target_arc_frac))
    {
      ++best_idx;
    }
    // 在此航点的 chord-length 参数处对密集样条求值
    double u = state_.parameters(best_idx);
    Eigen::Vector2d p = dense_spline(u);
    state_.control_points(0, i) = p.x();
    state_.control_points(1, i) = p.y();
  }

  // ── 第4步: 为 M 个控制点创建节点向量 ──
  // 使用从 M 个航点子集中均匀间隔的 chord-length
  Eigen::RowVectorXd sub_chord(M);
  for (int i = 0; i < M; ++i) {
    sub_chord(i) = static_cast<double>(i) / static_cast<double>(M - 1);
  }
  Eigen::RowVectorXd new_knots;
  Eigen::KnotAveraging(sub_chord, eff_deg, new_knots);
  state_.knots = new_knots;

  // 保存初始控制点位置 (flat doubles), 用于走廊约束
  state_.initial_params.clear();
  state_.initial_params.reserve(2 * (M - 2));
  for (int i = 1; i < M - 1; ++i) {
    state_.initial_params.push_back(state_.control_points(0, i));
    state_.initial_params.push_back(state_.control_points(1, i));
  }

  rebuildSpline();
  fitted_ = true;
  return true;
}

void BSplineOptimizer::rebuildSpline()
{
  if (state_.control_points.cols() < state_.effective_degree + 1) {return;}
  spline_ = std::make_unique<Spline2D>(state_.knots, state_.control_points);
}

std::vector<std::pair<double, double>> BSplineOptimizer::sample(int N) const
{
  std::vector<std::pair<double, double>> out{};
  if (!fitted_) {return out;}
  if (!spline_) {
    // Linear fallback: 端点间线性插值
    if (state_.original_points.empty()) {return out;}
    out.reserve(N);
    double x0 = state_.original_points.front().x();
    double y0 = state_.original_points.front().y();
    double x1 = state_.original_points.back().x();
    double y1 = state_.original_points.back().y();
    for (int i = 0; i < N; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(N - 1);
      out.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
    }
    return out;
  }
  out.reserve(N);
  for (int i = 0; i < N; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(N - 1);
    Eigen::Vector2d p = (*spline_)(u);
    out.emplace_back(p.x(), p.y());
  }
  return out;
}

double BSplineOptimizer::curvatureAt(double u) const
{
  if (!fitted_ || !spline_) {return 0.0;}
  Eigen::Matrix<double, 2, 3> derivs = spline_->derivatives<2>(u);
  double dx = derivs(0, 1);
  double dy = derivs(1, 1);
  double ddx = derivs(0, 2);
  double ddy = derivs(1, 2);
  double denom = dx * dx + dy * dy;
  if (denom < 1e-12) {return 0.0;}
  return std::abs(dx * ddy - dy * ddx) / (denom * std::sqrt(denom));
}

double BSplineOptimizer::computeCurvatureEnergy() const
{
  if (!fitted_ || !spline_) {return 0.0;}
  double energy{};
  constexpr int K = 200;
  for (int i = 0; i <= K; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(K);
    auto derivs = spline_->derivatives<2>(u);
    double ddx = derivs(0, 2);
    double ddy = derivs(1, 2);
    energy += (ddx * ddx + ddy * ddy);
  }
  return energy / static_cast<double>(K + 1);
}

BSplineResult BSplineOptimizer::optimize(int num_samples)
{
  BSplineResult result{};
  // spline_ 可能为空 (短路径线性兜底分支): 由 sample() 的线性回退处理
  if (!fitted_) {return result;}

  const int M = static_cast<int>(state_.control_points.cols());
  if (M < 3) {
    result.smoothed_path = sample(num_samples);
    result.curvature_profile.resize(num_samples);
    for (int i = 0; i < num_samples; ++i) {
      double u = static_cast<double>(i) / static_cast<double>(num_samples - 1);
      result.curvature_profile[i] = curvatureAt(u);
    }
    result.total_curvature_energy = computeCurvatureEnergy();
    result.converged = true;
    return result;
  }

  result.cost_initial = computeCurvatureEnergy();

  // ── 第1步: 可选梯度下降 ──
  if (config_.enable_gradient_descent) {
    const int n_interior = M - 2;
    const int param_size = 2 * n_interior;
    std::vector<double> params(param_size);
    for (int i = 0; i < n_interior; ++i) {
      params[2 * i] = state_.control_points(0, i + 1);
      params[2 * i + 1] = state_.control_points(1, i + 1);
    }

    double fx = state_.control_points(0, 0);
    double fy = state_.control_points(1, 0);
    double lx = state_.control_points(0, M - 1);
    double ly = state_.control_points(1, M - 1);

    bool converged{};
    int iters_out{};
    const auto cc = buildCostCache(
      state_.knots, M, state_.original_points, state_.parameters);
    auto opt = gradientDescent(
      params, cc,
      fx, fy, lx, ly,
      config_.smoothness_weight, 1.0,  // J_dist 权重固定为 1.0
      state_.esdf_distance, state_.esdf_gradient_x, state_.esdf_gradient_y,
      state_.esdf_w, state_.esdf_h,
      state_.esdf_resolution, state_.esdf_origin_x, state_.esdf_origin_y,
      state_.esdf_max_distance,
      config_.esdf_safe_distance, config_.esdf_weight,
      config_.max_iterations, config_.corridor_halfwidth, converged, iters_out);
    result.iterations = iters_out;

    for (int i = 0; i < n_interior; ++i) {
      state_.control_points(0, i + 1) = opt[2 * i];
      state_.control_points(1, i + 1) = opt[2 * i + 1];
    }
  }

  // ── 第2步: 控制点的障碍物避让投射 (螺旋搜索) ──
  if (state_.costmap_data != nullptr) {
    for (int i = 1; i < M - 1; ++i) {
      double & cx = state_.control_points(0, i);
      double & cy = state_.control_points(1, i);
      projectPointToFree(
        state_.costmap_data, state_.costmap_w,
        state_.costmap_h, cx, cy);
    }
  }

  rebuildSpline();

  // ── 第3步: 采样输出路径 ──
  auto path = sample(num_samples);

  result.smoothed_path = std::move(path);
  result.curvature_profile.resize(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(num_samples - 1);
    result.curvature_profile[i] = curvatureAt(u);
  }
  result.total_curvature_energy = computeCurvatureEnergy();
  result.cost_final = result.total_curvature_energy;
  result.converged = true;

  result.control_points_xy.reserve(M);
  for (int i = 0; i < M; ++i) {
    result.control_points_xy.emplace_back(
      state_.control_points(0, i), state_.control_points(1, i));
  }
  return result;
}

}  // namespace bspline_opt
