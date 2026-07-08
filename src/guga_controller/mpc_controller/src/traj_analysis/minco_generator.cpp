#include "mpc_controller/trajectory/minco_generator.hpp"
#include "mpc_controller/trajectory/discrete_generator.hpp"

namespace mpc_controller
{

// ==========================================================================
ReferenceTrajectory MincoGenerator::generate(
  const nav_msgs::msg::Path & path,
  int horizon_n,
  double dt,
  double lookahead)
{
  // MINCO 优化尚未实现 — 当前回退到离散插值
  // TODO(ld): 集成 minco_smoother 的 minco::Minco + L-BFGS + ALM
  return fallback_.generate(path, horizon_n, dt, lookahead);
}

// ==========================================================================
double MincoGenerator::curvature(double s) const
{
  return fallback_.curvature(s);
}

}  // namespace mpc_controller
