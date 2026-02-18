// Ticket: 0068b_nlopt_friction_solver_class
// Ticket: 0069_friction_velocity_reversal
// Design: docs/designs/0068_nlopt_friction_cone_solver/design.md

#include "NLoptFrictionSolver.hpp"

#include <nlopt.hpp>
#include <spdlog/spdlog.h>
#include <cmath>
#include <stdexcept>

namespace msd_sim
{

NLoptFrictionSolver::NLoptFrictionSolver()
  : tolerance_{1e-6}, max_iterations_{100}, algorithm_{Algorithm::SLSQP}
{
}

NLoptFrictionSolver::NLoptFrictionSolver(Algorithm algo)
  : tolerance_{1e-6}, max_iterations_{100}, algorithm_{algo}
{
}

NLoptFrictionSolver::SolveResult NLoptFrictionSolver::solve(
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  const std::vector<double>& mu,
  const Eigen::VectorXd& lambda0,
  const std::vector<double>& normalUpperBounds,
  const std::vector<double>& tangent1LowerBounds)
{
  const auto num_contacts = static_cast<size_t>(mu.size());
  const auto num_vars = 3 * num_contacts;

  // Validate input dimensions
  if (static_cast<size_t>(A.rows()) != num_vars || static_cast<size_t>(A.cols()) != num_vars)
  {
    throw std::runtime_error("NLoptFrictionSolver: A matrix dimension mismatch");
  }
  if (static_cast<size_t>(b.size()) != num_vars)
  {
    throw std::runtime_error("NLoptFrictionSolver: b vector dimension mismatch");
  }

  // Clamp invalid mu values to 0.0 with warning
  std::vector<double> mu_clamped = mu;
  for (size_t i = 0; i < mu_clamped.size(); ++i)
  {
    if (mu_clamped[i] < 0.0)
    {
      spdlog::warn("NLoptFrictionSolver: negative friction coefficient mu[{}] = {} clamped to 0.0",
                   i, mu_clamped[i]);
      mu_clamped[i] = 0.0;
    }
  }

  // Select NLopt algorithm
  nlopt::algorithm nlopt_algo;
  switch (algorithm_)
  {
    case Algorithm::SLSQP:
      nlopt_algo = nlopt::LD_SLSQP;
      break;
    case Algorithm::COBYLA:
      nlopt_algo = nlopt::LN_COBYLA;
      break;
    case Algorithm::MMA:
      nlopt_algo = nlopt::LD_MMA;
      break;
    case Algorithm::AUGLAG_SLSQP:
      nlopt_algo = nlopt::AUGLAG;
      break;
    default:
      nlopt_algo = nlopt::LD_SLSQP;
      break;
  }

  // Create NLopt optimizer
  nlopt::opt opt{nlopt_algo, static_cast<unsigned>(num_vars)};

  // For AUGLAG, set sub-solver to SLSQP
  if (algorithm_ == Algorithm::AUGLAG_SLSQP)
  {
    nlopt::opt local_opt{nlopt::LD_SLSQP, static_cast<unsigned>(num_vars)};
    local_opt.set_ftol_rel(tolerance_);
    opt.set_local_optimizer(local_opt);
  }

  // Set objective function
  ObjectiveData obj_data{&A, &b};
  opt.set_min_objective(objective, &obj_data);

  // Set lower bounds: lambda_n >= 0, lambda_t1 per-contact (if provided), lambda_t2 unbounded
  std::vector<double> lower_bounds(num_vars, -HUGE_VAL);
  for (size_t i = 0; i < num_contacts; ++i)
  {
    lower_bounds[3 * i] = 0.0;  // lambda_n_i >= 0 (always)

    // lambda_t1_i >= tangent1LowerBounds[i] (if provided, for sliding mode)
    if (!tangent1LowerBounds.empty() &&
        tangent1LowerBounds.size() == num_contacts)
    {
      lower_bounds[3 * i + 1] = tangent1LowerBounds[i];
    }
    // lambda_t2_i unbounded (no change, remains -HUGE_VAL)
  }
  opt.set_lower_bounds(lower_bounds);

  // Add cone constraints: mu^2 * n^2 - t1^2 - t2^2 >= 0
  // NLopt expects c(x) <= 0, so we negate: -(mu^2 * n^2 - t1^2 - t2^2) <= 0
  std::vector<ConstraintData> constraint_data;
  constraint_data.reserve(num_contacts);
  for (size_t i = 0; i < num_contacts; ++i)
  {
    constraint_data.push_back(ConstraintData{static_cast<int>(i), mu_clamped[i]});
    opt.add_inequality_constraint(coneConstraint, &constraint_data[i], tolerance_);
  }

  // Set convergence criteria
  opt.set_ftol_rel(tolerance_);
  opt.set_maxeval(max_iterations_);

  // Set upper bounds on normal impulses if provided
  std::vector<double> upper_bounds(num_vars, HUGE_VAL);
  if (!normalUpperBounds.empty() &&
      normalUpperBounds.size() == num_contacts)
  {
    for (size_t i = 0; i < num_contacts; ++i)
    {
      upper_bounds[3 * i] = std::max(0.0, normalUpperBounds[i]);
    }
    opt.set_upper_bounds(upper_bounds);
  }

  // Initialize solution vector (warm-start or cold start)
  std::vector<double> x(num_vars, 0.0);
  if (static_cast<size_t>(lambda0.size()) == num_vars)
  {
    for (size_t i = 0; i < num_vars; ++i)
    {
      x[i] = lambda0[static_cast<Eigen::Index>(i)];
    }
    // Clamp warm-start to bounds
    for (size_t i = 0; i < num_vars; ++i)
    {
      x[i] = std::max(lower_bounds[i] == -HUGE_VAL ? x[i] : lower_bounds[i], x[i]);
      x[i] = std::min(upper_bounds[i] == HUGE_VAL ? x[i] : upper_bounds[i], x[i]);
    }
  }

  // Optimize
  double final_obj = std::numeric_limits<double>::quiet_NaN();
  nlopt::result result;
  try
  {
    result = opt.optimize(x, final_obj);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(std::string("NLoptFrictionSolver: optimization failed: ") + e.what());
  }

  // Map result code to converged flag
  bool converged = (result == nlopt::SUCCESS ||
                    result == nlopt::FTOL_REACHED ||
                    result == nlopt::XTOL_REACHED);

  // Convert solution back to Eigen
  Eigen::VectorXd lambda(static_cast<Eigen::Index>(num_vars));
  for (size_t i = 0; i < num_vars; ++i)
  {
    lambda[static_cast<Eigen::Index>(i)] = x[i];
  }

  // Compute per-contact constraint violations for diagnostics
  std::vector<double> constraint_violations;
  constraint_violations.reserve(num_contacts);
  for (size_t i = 0; i < num_contacts; ++i)
  {
    const auto idx = static_cast<Eigen::Index>(3 * i);
    const double n = lambda[idx];
    const double t1 = lambda[idx + 1];
    const double t2 = lambda[idx + 2];
    const double mu_val = mu_clamped[i];
    const double cone_val = mu_val * mu_val * n * n - t1 * t1 - t2 * t2;
    constraint_violations.push_back(cone_val);
  }

  // Get iteration count (not directly available from NLopt, use max as placeholder)
  const int iterations_performed = static_cast<int>(opt.get_numevals());

  return SolveResult{
    lambda,
    converged,
    iterations_performed,
    std::numeric_limits<double>::quiet_NaN(),  // residual not computed
    final_obj,
    constraint_violations
  };
}

double NLoptFrictionSolver::objective(
  const std::vector<double>& lambda,
  std::vector<double>& grad,
  void* data)
{
  const auto* obj_data = static_cast<const ObjectiveData*>(data);
  const Eigen::MatrixXd& A = *obj_data->A;
  const Eigen::VectorXd& b = *obj_data->b;

  const auto n = static_cast<Eigen::Index>(lambda.size());

  // Convert std::vector to Eigen::VectorXd
  Eigen::VectorXd lambda_eigen(n);
  for (Eigen::Index i = 0; i < n; ++i)
  {
    lambda_eigen[i] = lambda[static_cast<size_t>(i)];
  }

  // Compute objective: f = (1/2) lambda^T A lambda - b^T lambda
  const double f = 0.5 * lambda_eigen.dot(A * lambda_eigen) - b.dot(lambda_eigen);

  // Compute gradient: grad = A * lambda - b
  if (!grad.empty())
  {
    Eigen::VectorXd grad_eigen = A * lambda_eigen - b;
    for (Eigen::Index i = 0; i < n; ++i)
    {
      grad[static_cast<size_t>(i)] = grad_eigen[i];
    }
  }

  return f;
}

double NLoptFrictionSolver::coneConstraint(
  const std::vector<double>& lambda,
  std::vector<double>& grad,
  void* data)
{
  const auto* constraint_data = static_cast<const ConstraintData*>(data);
  const auto contact_idx = static_cast<size_t>(constraint_data->contactIndex);
  const double mu = constraint_data->mu;

  const double n = lambda[3 * contact_idx];
  const double t1 = lambda[3 * contact_idx + 1];
  const double t2 = lambda[3 * contact_idx + 2];

  // Constraint: mu^2 * n^2 - t1^2 - t2^2 >= 0
  // NLopt expects c(x) <= 0, so negate: -(mu^2 * n^2 - t1^2 - t2^2) <= 0
  const double c = mu * mu * n * n - t1 * t1 - t2 * t2;
  const double c_nlopt = -c;

  // Gradient (negated for NLopt convention)
  if (!grad.empty())
  {
    grad.assign(lambda.size(), 0.0);
    grad[3 * contact_idx]     = -2.0 * mu * mu * n;   // d(-c)/dn
    grad[3 * contact_idx + 1] = 2.0 * t1;             // d(-c)/dt1
    grad[3 * contact_idx + 2] = 2.0 * t2;             // d(-c)/dt2
  }

  return c_nlopt;
}

}  // namespace msd_sim
