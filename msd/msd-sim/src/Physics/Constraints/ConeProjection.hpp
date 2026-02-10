// Ticket: 0052b_cone_projection_and_linear_algebra
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace msd_sim
{

/// Result of projecting a single contact's impulse onto its friction cone
struct ProjectionResult
{
    double lambda_n{0.0};
    double lambda_t1{0.0};
    double lambda_t2{0.0};
    int case_{0};  // 1 = interior, 2 = origin, 3 = cone surface
};

/// Stateless utility providing the Euclidean projection of a 3-vector onto the
/// friction cone K_mu = { (lambda_n, lambda_t1, lambda_t2) :
///     ||(lambda_t1, lambda_t2)|| <= mu * lambda_n, lambda_n >= 0 }
/// and its Jacobian matrix.
///
/// @see docs/designs/0052_custom_friction_cone_solver/design.md
/// @ticket 0052b_cone_projection_and_linear_algebra
class ConeProjection
{
public:
    /// Project (lambda_n, lambda_t1, lambda_t2) onto cone K_mu.
    /// Three geometric cases from math formulation M3:
    ///   Case 1: ||p_t|| <= mu * p_n  -> identity (interior)
    ///   Case 2: mu * ||p_t|| <= -p_n -> zero (dual cone)
    ///   Case 3: otherwise            -> cone surface (Eq. 24-25)
    [[nodiscard]] static ProjectionResult project(
        double lambda_n, double lambda_t1, double lambda_t2, double mu)
    {
        const double p_t_norm = std::sqrt(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);

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
        if (mu < 1e-15)
        {
            return {std::max(lambda_n, 0.0), 0.0, 0.0, 3};
        }

        const double alpha = 1.0 / (1.0 + mu * mu);
        const double proj_n = alpha * (lambda_n + mu * p_t_norm);
        double proj_t1 = 0.0;
        double proj_t2 = 0.0;
        if (p_t_norm > 1e-15)
        {
            const double scale = mu * proj_n / p_t_norm;
            proj_t1 = scale * lambda_t1;
            proj_t2 = scale * lambda_t2;
        }
        return {proj_n, proj_t1, proj_t2, 3};
    }

    /// Compute 3x3 Jacobian of projection.
    /// Case 1: I_3, Case 2: 0, Case 3: rank-2 matrix (Eq. 32-33 from M3)
    [[nodiscard]] static Eigen::Matrix3d gradient(
        double lambda_n, double lambda_t1, double lambda_t2, double mu)
    {
        const double p_t_norm = std::sqrt(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);

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
        if (mu < 1e-15)
        {
            Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
            if (lambda_n > 0.0)
            {
                J(0, 0) = 1.0;
            }
            return J;
        }

        const double r = p_t_norm;
        if (r < 1e-15)
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

        // Rows 1-2: d(proj_ti)/dp using product rule on proj_ti = mu * proj_n * p_ti / r
        const double p_t[2] = {lambda_t1, lambda_t2};
        for (int i = 0; i < 2; ++i)
        {
            const int row = i + 1;

            // d/dp_n
            J(row, 0) = mu * alpha * p_t[i] / r;

            for (int j = 0; j < 2; ++j)
            {
                const int col = j + 1;
                const double delta_ij = (i == j) ? 1.0 : 0.0;

                const double dproj_n_dpj = alpha * mu * p_t[j] / r;
                const double d_ratio = delta_ij / r - p_t[i] * p_t[j] / (r * r * r);

                J(row, col) = mu * (dproj_n_dpj * p_t[i] / r + proj_n * d_ratio);
            }
        }

        return J;
    }

    /// Project entire lambda vector (3C x 1) onto product cone K.
    /// Processes C independent 3-tuples at indices [3i, 3i+1, 3i+2].
    [[nodiscard]] static Eigen::VectorXd projectVector(
        const Eigen::VectorXd& lambda,
        const std::vector<double>& mu,
        int numContacts)
    {
        Eigen::VectorXd result{lambda.size()};
        for (int i = 0; i < numContacts; ++i)
        {
            auto p = project(lambda[3 * i], lambda[3 * i + 1], lambda[3 * i + 2], mu[static_cast<size_t>(i)]);
            result[3 * i] = p.lambda_n;
            result[3 * i + 1] = p.lambda_t1;
            result[3 * i + 2] = p.lambda_t2;
        }
        return result;
    }

    ConeProjection() = delete;  // Stateless utility, no instances
};

}  // namespace msd_sim
