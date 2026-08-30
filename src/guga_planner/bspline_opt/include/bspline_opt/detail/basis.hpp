#pragma once

#include <Eigen/Core>
#include <vector>

namespace bspline_opt
{
namespace detail
{

int findSpan(double u, const Eigen::RowVectorXd & knots, int n, int p);

void basisValuesAt(
  double u, const Eigen::RowVectorXd & knots, int p, int M,
  std::vector<double> & out);

void basisDerivsAt(
  double u, const Eigen::RowVectorXd & knots, int p, int M,
  std::vector<double> & N, std::vector<double> & N1, std::vector<double> & N2);

}  // namespace detail
}  // namespace bspline_opt
