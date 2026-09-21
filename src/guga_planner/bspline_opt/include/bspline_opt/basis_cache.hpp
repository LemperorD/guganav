#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

#include"bspline_opt/detail/cost_cache.hpp"
namespace bspline_opt::detail
{

struct BasiscacheConfig
{
    int kSplineDegree {7};
};

class BasisCache
{
public:
    explicit BasisCache(const BasiscacheConfig & config = BasiscacheConfig{}); 
    void rowFromBasis(const std::vector<double> & b, int M, BandRow & row);

    CostCache buildCostCache(
    const Eigen::RowVectorXd & knots, int M,
    const std::vector<Eigen::Vector2d> & orig_points,
    const Eigen::VectorXd & orig_params);

    double bandedDot(const BandRow & row, const Eigen::MatrixXd & ctrl, int dim);

    void fillCtrl(
    const std::vector<double> & params, double first_x, double first_y, double last_x,
    double last_y, int M, Eigen::MatrixXd & ctrl);

private:
    BasiscacheConfig config_{};
    int findSpan(double u, const Eigen::RowVectorXd & knots, int n, int p);
    void basisDerivsAt(
    double u, const Eigen::RowVectorXd & knots, int p, int M,
    std::vector<double> & N, std::vector<double> & N1, std::vector<double> & N2);
};
inline BasisCache::BasisCache(const BasiscacheConfig & config):
config_(config)
{
    
};
}


