#include "bspline_opt/detail/gradient_descent.hpp"
#include "bspline_opt/detail/cost_function.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>


namespace bspline_opt::detail
{

std::vector<double> gradientDescent(
  const std::vector<double> & init_params, 
  const CostCache & cc,
  EndPoints & endpoints,
  Weight & weight, 
  const float * esdf_gx, const float * esdf_gy,
  EsdfData & esdfdata,
  CostNormal & costnormal, 
  int max_iters,
  double corridor_hw, 
  bool & converged, 
  int & iters_out)
{
  std::vector<double> x = init_params;
  const int N = static_cast<int>(x.size());
  if (N == 0) {converged = true; iters_out = 0; return x;}

  constexpr double gtol = 1e-7;
  constexpr int patience = 30;
  const double iter_step = 2.0;   // 单次迭代最大移动距离 (格元)

  std::vector<double> grad(N);
  std::vector<double> g_prev(N);
  std::vector<double> direction(N);
  std::vector<double> x_try(N);
  auto normalise = [&] (std::vector<double>  gradient){
    double norm=sqrt(std::inner_product(gradient.begin(),gradient.end(),gradient.begin(),0.0));
    return norm;
  } ;
  computeGradient(
    x, 
    cc, 
    endpoints,
    weight,
    esdfdata, 
    costnormal, 
    grad,
    esdf_gx,esdf_gy);

  double f_best = evalCost(
    x, cc, 
    endpoints,weight,esdfdata,costnormal
  );

  auto evalclamped = [&](const std::vector<double> & x_try, double & cost_clamped) {
      std::vector<double> x_clamped = x_try;
      for (int i = 0; i < N; ++i) {
        const int ctrl_j = i / 2 + 1;
        double corr = corridor_hw;
        if (ctrl_j == 1 || ctrl_j == cc.M - 2) {
          corr = std::min(corr, 0.5);
        }
        x_clamped[static_cast<size_t>(i)] = std::clamp(
          x_clamped[static_cast<size_t>(i)],
          init_params[static_cast<size_t>(i)] - corr,
          init_params[static_cast<size_t>(i)] + corr);
      }
      cost_clamped = evalCost(
       x_clamped,cc,endpoints,weight,esdfdata,costnormal);
      return x_clamped;
    };

  // 初始搜索方向: 负梯度 (归一化)
  double g_norm = normalise(grad);
  for (int i = 0; i < N; ++i) {
    direction[static_cast<size_t>(i)] =
      (g_norm > 1e-12) ? -grad[static_cast<size_t>(i)] / g_norm : 0.0;
  }
  g_prev = grad;

  int no_improve{};
  int iter{};
  for (; iter < max_iters; ++iter) {
    g_norm = normalise(grad);
    if (g_norm < gtol) {break;}

    double alpha = iter_step;//alpha为梯度下降的学习率
    bool found{};
    for (int ls = 0; ls < 30; ++ls) {
      for (int i = 0; i < N; ++i) {
        x_try[static_cast<size_t>(i)] =
          x[static_cast<size_t>(i)] + alpha * direction[static_cast<size_t>(i)];
      }
      double f_try{};
      auto x_clamped = evalclamped(x_try, f_try);
      if (f_try < f_best) {
        x = x_clamped;
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
        direction[static_cast<size_t>(i)] = -grad[static_cast<size_t>(i)];
      }
      double descent_norm = normalise(direction);
      if (descent_norm > 1e-12) {
        for (int i = 0; i < N; ++i) {
          direction[static_cast<size_t>(i)] /= descent_norm;
        }
      }
      continue;
    }

    // 新梯度
    std::vector<double> g_new(N);
    computeGradient(
      x, cc,endpoints,weight,
    esdfdata,costnormal,g_new,esdf_gx,esdf_gy);

    // Polak-Ribiere beta (带非负截断/重启), 然后归一化方向
    double descent_norm = normalise(grad);
    double beta = 0.0;
    if (descent_norm > 1e-12) {
      double numer = 0.0;
      for (int i = 0; i < N; ++i) {
        numer += g_new[static_cast<size_t>(i)] *
          (g_new[static_cast<size_t>(i)] - grad[static_cast<size_t>(i)]);
      }
      beta = std::max(0.0, numer / descent_norm);
    }
    grad = g_new;
    for (int i = 0; i < N; ++i) {
      direction[static_cast<size_t>(i)] = -grad[static_cast<size_t>(i)] + beta *
        direction[static_cast<size_t>(i)];
    }
    double dn = normalise(direction);
    if (dn > 1e-12) {
      for (int i = 0; i < N; ++i) {
        direction[static_cast<size_t>(i)] /= dn;
      }
    } 
    else {
      for (int i = 0; i < N; ++i) {
        direction[static_cast<size_t>(i)] = -grad[static_cast<size_t>(i)];
      }
      dn = normalise(direction);
      if (dn > 1e-12) {
        for (int i = 0; i < N; ++i) {
          direction[static_cast<size_t>(i)] /= dn;
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
