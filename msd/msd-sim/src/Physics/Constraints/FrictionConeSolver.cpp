// Ticket: 0052c_newton_solver_core
// Ticket: 0053f_wire_solver_workspace
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#include "FrictionConeSolver.hpp"
#include "ConeProjection.hpp"
#include <algorithm>
#include <cmath>

namespace msd_sim
{

FrictionConeSolver::SolveResult FrictionConeSolver::solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<double>& mu,
    const Eigen::VectorXd& lambda0)
{
    const int n = static_cast<int>(b.size());
    const int numContacts = n / 3;

    // Resize workspace (Eigen reuses capacity when size <= previous max)
    ws_.resize(n);

    // Step 1: Regularize
    double eps = kRegularizationEpsilon;

    // Step 2: Cholesky factorization with fallback regularization
    ws_.A_reg.noalias() = A;
    ws_.A_reg.diagonal().array() += eps;
    bool factored = false;
    for (int attempt = 0; attempt < 6; ++attempt)
    {
        ws_.A_reg.noalias() = A;
        ws_.A_reg.diagonal().array() += eps;
        ws_.llt.compute(ws_.A_reg);
        if (ws_.llt.info() == Eigen::Success)
        {
            factored = true;
            break;
        }
        eps *= 10.0;
    }

    if (!factored)
    {
        return {Eigen::VectorXd::Zero(n), false, 0,
                std::numeric_limits<double>::infinity()};
    }

    // Step 3: Unconstrained optimum
    ws_.lambda_unc.noalias() = ws_.llt.solve(b);

    // Step 4: Initialize lambda
    if (lambda0.size() == n)
    {
        ws_.lambda = ConeProjection::projectVector(lambda0, mu, numContacts);
    }
    else
    {
        ws_.lambda = ConeProjection::projectVector(ws_.lambda_unc, mu, numContacts);
    }

    // Objective function: f(x) = 0.5 * x^T A_reg x - b^T x
    auto objective = [&](const Eigen::VectorXd& x) -> double {
        return 0.5 * x.dot(ws_.A_reg * x) - b.dot(x);
    };

    // Step 5: Newton iteration
    double residual = std::numeric_limits<double>::infinity();
    int iter = 0;
    for (iter = 0; iter < max_iterations_; ++iter)
    {
        // Step 5a: Gradient
        ws_.g.noalias() = ws_.A_reg * ws_.lambda;
        ws_.g -= b;

        // Step 5b: Projected gradient residual
        ws_.proj = ConeProjection::projectVector(ws_.lambda - ws_.g, mu, numContacts);
        residual = (ws_.lambda - ws_.proj).norm();

        // Step 5c: Convergence check
        if (residual < tolerance_)
        {
            return {ws_.lambda, true, iter, residual};
        }

        // Step 5d: Compute search direction using reduced-space Newton.
        // Build J_proj: block-diagonal Jacobian of the cone projection at current lambda.
        ws_.J_proj.setZero();
        for (int c = 0; c < numContacts; ++c)
        {
            double ln = ws_.lambda[3 * c];
            double lt1 = ws_.lambda[3 * c + 1];
            double lt2 = ws_.lambda[3 * c + 2];
            double lt_norm = std::sqrt(lt1 * lt1 + lt2 * lt2);

            Eigen::Matrix3d Jc;
            if (lt_norm <= mu[static_cast<size_t>(c)] * ln && ln >= 0)
            {
                Jc = Eigen::Matrix3d::Identity();
            }
            else if (mu[static_cast<size_t>(c)] * lt_norm <= -ln || (ln <= 0 && lt_norm < 1e-15))
            {
                Jc = Eigen::Matrix3d::Zero();
            }
            else
            {
                Jc = ConeProjection::gradient(ln, lt1, lt2, mu[static_cast<size_t>(c)]);
            }
            ws_.J_proj.block<3, 3>(3 * c, 3 * c) = Jc;
        }

        // Reduced Hessian: H_r = J^T A J
        ws_.H_r.noalias() = ws_.J_proj.transpose() * ws_.A_reg * ws_.J_proj;
        // Reduced gradient: g_r = J^T g
        ws_.g_r.noalias() = ws_.J_proj.transpose() * ws_.g;
        // Regularize for numerical stability
        ws_.H_r.diagonal().array() += 1e-12;

        ws_.llt_r.compute(ws_.H_r);
        if (ws_.llt_r.info() == Eigen::Success)
        {
            ws_.delta.noalias() = ws_.J_proj * ws_.llt_r.solve(-ws_.g_r);
        }
        else
        {
            // Fallback to standard Newton
            ws_.delta.noalias() = ws_.llt.solve(-ws_.g);
        }

        // Step 5e: Armijo line search with projection
        double alpha = 1.0;
        double f_current = objective(ws_.lambda);
        ws_.trial = ws_.lambda;
        bool made_progress = false;

        for (int ls = 0; ls < max_line_search_; ++ls)
        {
            ws_.trial = ConeProjection::projectVector(ws_.lambda + alpha * ws_.delta, mu, numContacts);
            double step_norm = (ws_.trial - ws_.lambda).norm();

            if (step_norm < 1e-15)
            {
                alpha *= armijo_beta_;
                continue;
            }

            double f_trial = objective(ws_.trial);
            double descent = ws_.g.dot(ws_.trial - ws_.lambda);
            if (f_trial <= f_current + armijo_c1_ * std::min(descent, 0.0))
            {
                ws_.lambda = ws_.trial;
                made_progress = true;
                break;
            }
            alpha *= armijo_beta_;
        }

        // If projected Newton didn't work, try SPG (Spectral Projected Gradient).
        if (!made_progress)
        {
            double gAg = ws_.g.dot(ws_.A_reg * ws_.g);
            alpha = (gAg > 1e-15) ? ws_.g.dot(ws_.g) / gAg : 1.0;

            for (int ls = 0; ls < max_line_search_; ++ls)
            {
                ws_.trial = ConeProjection::projectVector(ws_.lambda - alpha * ws_.g, mu, numContacts);
                double step_norm = (ws_.trial - ws_.lambda).norm();

                if (step_norm < 1e-15)
                {
                    alpha *= armijo_beta_;
                    continue;
                }

                double f_trial = objective(ws_.trial);
                double descent = ws_.g.dot(ws_.trial - ws_.lambda);
                if (f_trial <= f_current + armijo_c1_ * std::min(descent, 0.0))
                {
                    ws_.lambda = ws_.trial;
                    made_progress = true;
                    break;
                }
                alpha *= armijo_beta_;
            }
        }

        if (!made_progress)
        {
            break;
        }
    }

    return {ws_.lambda, false, iter, residual};
}

}  // namespace msd_sim
