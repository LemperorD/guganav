#include "bspline_opt/detail/gradient_descent.hpp"
#include "bspline_opt/detail/cost_function.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace bspline_opt
{
namespace detail
{

std::vector<double> gradientDescent(
  const std::vector<double> & init_params, const CostCache & cc,
  double first_x, double first_y, double last_x, double last_y,
  double w_smooth, double w_dist,
  const float * esdf_dist, const float * esdf_gx, const float * esdf_gy,
  int esdf_w, int esdf_h, double esdf_res, double esdf_ox, double esdf_oy,
  double esdf_max, double esdf_safe_dist, double w_esdf,
  double js0, double jd0, double je0,
  int max_iters, double corridor_hw, bool & converged, int & iters_out)
{
  std::vector<double> x = init_params;
  const int N = static_cast<int>(x.size());
  if (N == 0) {converged = true; iters_out = 0; return x;}

  constexpr double gtol = 1e-7;
  constexpr int patience = 30;
  const double kMaxStepCells = 2.0;   // 单次迭代最大移动距离 (格元)

  std::vector<double> grad(N);
  std::vector<double> g_prev(N);
  std::vector<double> dir(N);
  std::vector<double> x_try(N);
  computeGradient(
    x, cc, first_x, first_y, last_x, last_y,
    w_smooth, w_dist,
    esdf_dist, esdf_gx, esdf_gy, esdf_w, esdf_h,
    esdf_res, esdf_ox, esdf_oy, esdf_max, esdf_safe_dist, w_esdf,
    js0, jd0, je0, grad);

  double f_best = evalCost(
    x, cc, first_x, first_y, last_x, last_y,
    w_smooth, w_dist, esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox, esdf_oy,
    esdf_max, esdf_safe_dist, w_esdf, js0, jd0, je0);

  auto evalClamped = [&](const std::vector<double> & xt, double & fout) {
      std::vector<double> xc = xt;
      for (int i = 0; i < N; ++i) {
        const int ctrl_j = i / 2 + 1;
        double corr = corridor_hw;
        if (ctrl_j == 1 || ctrl_j == cc.M - 2) {
          corr = std::min(corr, 0.5);
        }
        xc[static_cast<size_t>(i)] = std::clamp(
          xc[static_cast<size_t>(i)],
          init_params[static_cast<size_t>(i)] - corr,
          init_params[static_cast<size_t>(i)] + corr);
      }
      fout = evalCost(
        xc, cc, first_x, first_y, last_x, last_y,
        w_smooth, w_dist, esdf_dist, esdf_w, esdf_h, esdf_res, esdf_ox,
        esdf_oy, esdf_max, esdf_safe_dist, w_esdf, js0, jd0, je0);
      return xc;
    };

  // 初始搜索方向: 负梯度 (归一化)
  double g_norm = std::sqrt(
    std::inner_product(grad.begin(), grad.end(), grad.begin(), 0.0));
  for (int i = 0; i < N; ++i) {
    dir[static_cast<size_t>(i)] =
      (g_norm > 1e-12) ? -grad[static_cast<size_t>(i)] / g_norm : 0.0;
  }
  g_prev = grad;

  int no_improve{};
  int iter{};
  for (; iter < max_iters; ++iter) {
    g_norm = std::sqrt(
      std::inner_product(grad.begin(), grad.end(), grad.begin(), 0.0));
    if (g_norm < gtol) {break;}

    double alpha = kMaxStepCells;
    bool found{};
    for (int ls = 0; ls < 30; ++ls) {
      for (int i = 0; i < N; ++i) {
        x_try[static_cast<size_t>(i)] =
          x[static_cast<size_t>(i)] + alpha * dir[static_cast<size_t>(i)];
      }
      double f_try{};
      auto xc = evalClamped(x_try, f_try);
      if (f_try < f_best) {
        x = xc;
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
      // 卡住时重置为最速下降方向, 避免死循环
      for (int i = 0; i < N; ++i) {
        dir[static_cast<size_t>(i)] = -grad[static_cast<size_t>(i)];
      }
      double dn = std::sqrt(std::inner_product(dir.begin(), dir.end(), dir.begin(), 0.0));
      if (dn > 1e-12) {
        for (int i = 0; i < N; ++i) {
          dir[static_cast<size_t>(i)] /= dn;
        }
      }
      continue;
    }

    // 新梯度
    std::vector<double> g_new(N);
    computeGradient(
      x, cc, first_x, first_y, last_x, last_y,
      w_smooth, w_dist,
      esdf_dist, esdf_gx, esdf_gy, esdf_w, esdf_h,
      esdf_res, esdf_ox, esdf_oy, esdf_max, esdf_safe_dist, w_esdf,
      js0, jd0, je0, g_new);

    // Polak-Ribiere beta (带非负截断/重启), 然后归一化方向
    double denom = std::inner_product(grad.begin(), grad.end(), grad.begin(), 0.0);
    double beta = 0.0;
    if (denom > 1e-12) {
      double numer = 0.0;
      for (int i = 0; i < N; ++i) {
        numer += g_new[static_cast<size_t>(i)] *
          (g_new[static_cast<size_t>(i)] - grad[static_cast<size_t>(i)]);
      }
      beta = std::max(0.0, numer / denom);
    }
    grad = g_new;
    for (int i = 0; i < N; ++i) {
      dir[static_cast<size_t>(i)] = -grad[static_cast<size_t>(i)] + beta *
        dir[static_cast<size_t>(i)];
    }
    double dn = std::sqrt(std::inner_product(dir.begin(), dir.end(), dir.begin(), 0.0));
    if (dn > 1e-12) {
      for (int i = 0; i < N; ++i) {
        dir[static_cast<size_t>(i)] /= dn;
      }
    } else {
      for (int i = 0; i < N; ++i) {
        dir[static_cast<size_t>(i)] = -grad[static_cast<size_t>(i)];
      }
      dn = std::sqrt(std::inner_product(dir.begin(), dir.end(), dir.begin(), 0.0));
      if (dn > 1e-12) {
        for (int i = 0; i < N; ++i) {
          dir[static_cast<size_t>(i)] /= dn;
        }
      }
    }
    (void)g_prev;
  }
  iters_out = iter;
  converged = true;
  return x;
}

}  // namespace detail
}  // namespace bspline_opt
