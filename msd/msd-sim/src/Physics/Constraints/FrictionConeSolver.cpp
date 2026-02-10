// Ticket: 0052c_newton_solver_core
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
    const Eigen::VectorXd& lambda0) const
{
    const int n = static_cast<int>(b.size());
    const int numContacts = n / 3;

    // Step 1: Regularize
    double eps = kRegularizationEpsilon;

    // Step 2: Cholesky factorization with fallback regularization
    Eigen::MatrixXd A_reg = A + eps * Eigen::MatrixXd::Identity(n, n);
    Eigen::LLT<Eigen::MatrixXd> llt;
    bool factored = false;
    for (int attempt = 0; attempt < 6; ++attempt)
    {
        A_reg = A + eps * Eigen::MatrixXd::Identity(n, n);
        llt.compute(A_reg);
        if (llt.info() == Eigen::Success)
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
    Eigen::VectorXd lambda_unc = llt.solve(b);

    // Step 4: Initialize lambda
    Eigen::VectorXd lambda{n};
    if (lambda0.size() == n)
    {
        lambda = ConeProjection::projectVector(lambda0, mu, numContacts);
    }
    else
    {
        lambda = ConeProjection::projectVector(lambda_unc, mu, numContacts);
    }

    // Objective function: f(x) = 0.5 * x^T A_reg x - b^T x
    auto objective = [&](const Eigen::VectorXd& x) -> double {
        return 0.5 * x.dot(A_reg * x) - b.dot(x);
    };

    // Step 5: Newton iteration
    double residual = std::numeric_limits<double>::infinity();
    int iter = 0;
    for (iter = 0; iter < max_iterations_; ++iter)
    {
        // Step 5a: Gradient
        Eigen::VectorXd g = A_reg * lambda - b;

        // Step 5b: Projected gradient residual
        Eigen::VectorXd proj_lmg = ConeProjection::projectVector(lambda - g, mu, numContacts);
        residual = (lambda - proj_lmg).norm();

        // Step 5c: Convergence check
        if (residual < tolerance_)
        {
            return {lambda, true, iter, residual};
        }

        // Step 5d: Compute search direction using reduced-space Newton.
        // Build J_proj: block-diagonal Jacobian of the cone projection at current lambda.
        Eigen::MatrixXd J_proj = Eigen::MatrixXd::Zero(n, n);
        for (int c = 0; c < numContacts; ++c)
        {
            double ln = lambda[3 * c];
            double lt1 = lambda[3 * c + 1];
            double lt2 = lambda[3 * c + 2];
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
            J_proj.block<3, 3>(3 * c, 3 * c) = Jc;
        }

        // Reduced Hessian: H_r = J^T A J
        Eigen::MatrixXd H_r = J_proj.transpose() * A_reg * J_proj;
        // Reduced gradient: g_r = J^T g
        Eigen::VectorXd g_r = J_proj.transpose() * g;
        // Regularize for numerical stability
        H_r += 1e-12 * Eigen::MatrixXd::Identity(n, n);

        Eigen::VectorXd delta;
        Eigen::LLT<Eigen::MatrixXd> llt_r{H_r};
        if (llt_r.info() == Eigen::Success)
        {
            delta = J_proj * llt_r.solve(-g_r);
        }
        else
        {
            // Fallback to standard Newton
            delta = llt.solve(-g);
        }

        // Step 5e: Armijo line search with projection
        double alpha = 1.0;
        double f_current = objective(lambda);
        Eigen::VectorXd trial = lambda;
        bool made_progress = false;

        for (int ls = 0; ls < max_line_search_; ++ls)
        {
            trial = ConeProjection::projectVector(lambda + alpha * delta, mu, numContacts);
            double step_norm = (trial - lambda).norm();

            if (step_norm < 1e-15)
            {
                alpha *= armijo_beta_;
                continue;
            }

            double f_trial = objective(trial);
            double descent = g.dot(trial - lambda);
            if (f_trial <= f_current + armijo_c1_ * std::min(descent, 0.0))
            {
                lambda = trial;
                made_progress = true;
                break;
            }
            alpha *= armijo_beta_;
        }

        // If projected Newton didn't work, try SPG (Spectral Projected Gradient).
        if (!made_progress)
        {
            double gAg = g.dot(A_reg * g);
            alpha = (gAg > 1e-15) ? g.dot(g) / gAg : 1.0;

            for (int ls = 0; ls < max_line_search_; ++ls)
            {
                trial = ConeProjection::projectVector(lambda - alpha * g, mu, numContacts);
                double step_norm = (trial - lambda).norm();

                if (step_norm < 1e-15)
                {
                    alpha *= armijo_beta_;
                    continue;
                }

                double f_trial = objective(trial);
                double descent = g.dot(trial - lambda);
                if (f_trial <= f_current + armijo_c1_ * std::min(descent, 0.0))
                {
                    lambda = trial;
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

    return {lambda, false, iter, residual};
}

}  // namespace msd_sim
