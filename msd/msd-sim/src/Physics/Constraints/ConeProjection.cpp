// Ticket: 0052b_cone_projection_and_linear_algebra
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#include "msd-sim/src/Physics/Constraints/ConeProjection.hpp"

#include <array>
#include <cmath>

namespace msd_sim
{

ProjectionResult ConeProjection::project(
  double lambda_n, double lambda_t1, double lambda_t2, double mu)
{
  const double p_t_norm =
    std::sqrt(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);

  // Case 1: Interior of cone
  if (p_t_norm <= mu * lambda_n)
  {
    return {lambda_n, lambda_t1, lambda_t2, 1};
  }

  // Case 2: In dual cone (projection is origin)
  if (mu * p_t_norm <= -lambda_n)
  {
    return {0.0, 0.0, 0.0, 2};
  }

  // Case 3: Project to cone surface
  // Special case: mu = 0 means the cone is just the non-negative half-line
  if (mu < kNearZeroTolerance)
  {
    return {std::max(lambda_n, 0.0), 0.0, 0.0, 3};
  }

  const double alpha = 1.0 / (1.0 + mu * mu);
  const double proj_n = alpha * (lambda_n + mu * p_t_norm);
  double proj_t1 = 0.0;
  double proj_t2 = 0.0;
  if (p_t_norm > kNearZeroTolerance)
  {
    const double scale = mu * proj_n / p_t_norm;
    proj_t1 = scale * lambda_t1;
    proj_t2 = scale * lambda_t2;
  }
  return {proj_n, proj_t1, proj_t2, 3};
}

Eigen::Matrix3d ConeProjection::gradient(
  double lambda_n, double lambda_t1, double lambda_t2, double mu)
{
  const double p_t_norm =
    std::sqrt(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);

  // Case 1: Interior -> gradient is identity
  if (p_t_norm <= mu * lambda_n)
  {
    return Eigen::Matrix3d::Identity();
  }

  // Case 2: Dual cone -> gradient is zero
  if (mu * p_t_norm <= -lambda_n)
  {
    return Eigen::Matrix3d::Zero();
  }

  // Case 3: Cone surface projection gradient
  if (mu < kNearZeroTolerance)
  {
    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    if (lambda_n > 0.0)
    {
      J(0, 0) = 1.0;
    }
    return J;
  }

  const double r = p_t_norm;
  if (r < kNearZeroTolerance)
  {
    return Eigen::Matrix3d::Identity();
  }

  const double alpha = 1.0 / (1.0 + mu * mu);
  const double proj_n = alpha * (lambda_n + mu * r);

  Eigen::Matrix3d J;

  // Row 0: d(proj_n)/dp
  J(0, 0) = alpha;
  J(0, 1) = alpha * mu * lambda_t1 / r;
  J(0, 2) = alpha * mu * lambda_t2 / r;

  // Rows 1-2: d(proj_ti)/dp using product rule on proj_ti = mu * proj_n *
  // p_ti / r
  const std::array<double, 2> p_t = {lambda_t1, lambda_t2};
  for (int i = 0; i < 2; ++i)
  {
    const int row = i + 1;

    // d/dp_n
    J(row, 0) = mu * alpha * p_t[static_cast<size_t>(i)] / r;

    for (int j = 0; j < 2; ++j)
    {
      const int col = j + 1;
      const double delta_ij = (i == j) ? 1.0 : 0.0;

      const double dproj_n_dpj =
        alpha * mu * p_t[static_cast<size_t>(j)] / r;
      const double d_ratio =
        delta_ij / r
        - p_t[static_cast<size_t>(i)] * p_t[static_cast<size_t>(j)]
            / (r * r * r);

      J(row, col) =
        mu
        * (dproj_n_dpj * p_t[static_cast<size_t>(i)] / r
           + proj_n * d_ratio);
    }
  }

  return J;
}

Eigen::VectorXd ConeProjection::projectVector(
  const Eigen::VectorXd& lambda,
  const std::vector<double>& mu,
  int numContacts)
{
  Eigen::VectorXd result{lambda.size()};
  for (int i = 0; i < numContacts; ++i)
  {
    auto p = project(
      lambda[3 * i],
      lambda[3 * i + 1],
      lambda[3 * i + 2],
      mu[static_cast<size_t>(i)]);
    result[3 * i] = p.lambda_n;
    result[3 * i + 1] = p.lambda_t1;
    result[3 * i + 2] = p.lambda_t2;
  }
  return result;
}

}  // namespace msd_sim
