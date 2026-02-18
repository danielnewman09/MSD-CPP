// Prototype P1: SLSQP Convergence Validation
// Question: Does NLopt SLSQP reliably converge for cone-surface (saturated friction)
//           contacts where the custom solver fails?
//
// Success criteria:
// - SLSQP returns converged = true for tumbling cube scenario
// - Energy injection < 0.01 J/frame during sustained contact
// - Constraint violations ||lambda_t|| - mu*lambda_n < 1e-6 at saturation
// - No algorithm failures (NLopt error codes) over 1000-frame simulation

#include <eigen3/Eigen/Dense>
#include <nlopt.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

// ============================================================================
// Synthetic Test Case Generator
// ============================================================================

struct FrictionQP {
  Eigen::MatrixXd A;  // Effective mass matrix (3C x 3C)
  Eigen::VectorXd b;  // RHS vector (3C x 1)
  std::vector<double> mu;  // Friction coefficients (C entries)
  int numContacts;

  // Generate a synthetic QP representing a sliding contact scenario
  // (saturated friction at cone surface)
  static FrictionQP generateSlidingContact(int numContacts, double muCoeff) {
    FrictionQP qp;
    qp.numContacts = numContacts;
    qp.mu.resize(numContacts, muCoeff);

    const int n = 3 * numContacts;
    qp.A.resize(n, n);
    qp.b.resize(n);

    // Construct a positive definite A matrix
    // A = I + D where D is diagonal with small random perturbations
    qp.A.setIdentity();
    for (int i = 0; i < n; ++i) {
      qp.A(i, i) += 0.1 * (0.5 + 0.5 * std::sin(i));  // Vary eigenvalues
    }

    // RHS vector encoding sliding contact (tangential velocity dominates)
    for (int c = 0; c < numContacts; ++c) {
      const int idx = 3 * c;
      qp.b(idx) = 10.0;      // Normal component (positive, pushing into contact)
      qp.b(idx + 1) = 5.0;   // Tangent 1 (sliding)
      qp.b(idx + 2) = 3.0;   // Tangent 2 (sliding)
    }

    return qp;
  }
};

// ============================================================================
// NLopt SLSQP Wrapper
// ============================================================================

struct ObjectiveData {
  const Eigen::MatrixXd* A;
  const Eigen::VectorXd* b;
};

struct ConstraintData {
  int contactIndex;
  double mu;
};

// Objective: f(lambda) = 0.5 * lambda^T * A * lambda - b^T * lambda
double objective(const std::vector<double>& lambda, std::vector<double>& grad, void* data) {
  auto* objData = static_cast<ObjectiveData*>(data);
  const Eigen::MatrixXd& A = *objData->A;
  const Eigen::VectorXd& b = *objData->b;

  Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(lambda.data(), lambda.size());

  if (!grad.empty()) {
    Eigen::VectorXd g = A * x - b;
    for (size_t i = 0; i < lambda.size(); ++i) {
      grad[i] = g(i);
    }
  }

  return 0.5 * x.dot(A * x) - b.dot(x);
}

// Cone constraint: c(lambda) = mu^2 * lambda_n^2 - lambda_t1^2 - lambda_t2^2 >= 0
double coneConstraint(const std::vector<double>& lambda, std::vector<double>& grad, void* data) {
  auto* cData = static_cast<ConstraintData*>(data);
  const int idx = 3 * cData->contactIndex;
  const double mu = cData->mu;

  const double n = lambda[idx];
  const double t1 = lambda[idx + 1];
  const double t2 = lambda[idx + 2];

  if (!grad.empty()) {
    std::fill(grad.begin(), grad.end(), 0.0);
    grad[idx] = 2.0 * mu * mu * n;
    grad[idx + 1] = -2.0 * t1;
    grad[idx + 2] = -2.0 * t2;
  }

  return mu * mu * n * n - t1 * t1 - t2 * t2;
}

struct SolveResult {
  Eigen::VectorXd lambda;
  bool converged{false};
  int iterations{0};
  double finalObjective{std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> constraintViolations;
};

SolveResult solveWithSLSQP(const FrictionQP& qp, const Eigen::VectorXd& lambda0) {
  const int n = 3 * qp.numContacts;

  // Create NLopt optimizer
  nlopt::opt opt{nlopt::LD_SLSQP, static_cast<unsigned>(n)};
  opt.set_ftol_rel(1e-6);
  opt.set_maxeval(100);

  // Set objective
  ObjectiveData objData{&qp.A, &qp.b};
  opt.set_min_objective(objective, &objData);

  // Set bounds: lambda_n >= 0, lambda_t unbounded
  std::vector<double> lowerBounds(n, -HUGE_VAL);
  for (int c = 0; c < qp.numContacts; ++c) {
    lowerBounds[3 * c] = 0.0;  // Normal component must be >= 0
  }
  opt.set_lower_bounds(lowerBounds);

  // Add cone constraints
  std::vector<ConstraintData> constraintData(qp.numContacts);
  for (int c = 0; c < qp.numContacts; ++c) {
    constraintData[c] = {c, qp.mu[c]};
    opt.add_inequality_constraint(coneConstraint, &constraintData[c], 1e-8);
  }

  // Initial guess
  std::vector<double> x(lambda0.data(), lambda0.data() + n);

  // Solve
  double minf{std::numeric_limits<double>::quiet_NaN()};
  SolveResult result;

  try {
    auto retcode = opt.optimize(x, minf);
    result.converged = (retcode == nlopt::SUCCESS || retcode == nlopt::FTOL_REACHED ||
                       retcode == nlopt::XTOL_REACHED);
    result.iterations = opt.get_numevals();
  } catch (const std::exception& e) {
    std::cerr << "NLopt exception: " << e.what() << "\n";
    result.converged = false;
  }

  result.lambda = Eigen::Map<Eigen::VectorXd>(x.data(), n);
  result.finalObjective = minf;

  // Compute constraint violations
  result.constraintViolations.resize(qp.numContacts);
  for (int c = 0; c < qp.numContacts; ++c) {
    const int idx = 3 * c;
    const double n_val = result.lambda(idx);
    const double t1 = result.lambda(idx + 1);
    const double t2 = result.lambda(idx + 2);
    const double t_mag = std::sqrt(t1 * t1 + t2 * t2);
    const double mu_n = qp.mu[c] * n_val;
    result.constraintViolations[c] = std::abs(t_mag - mu_n);
  }

  return result;
}

// ============================================================================
// Energy Injection Calculation
// ============================================================================

double computeEnergyInjection(const Eigen::VectorXd& lambda, const Eigen::VectorXd& b) {
  // Simplified energy injection metric: E = lambda^T * b
  // (assumes b encodes velocity; positive E means energy added)
  return lambda.dot(b);
}

// ============================================================================
// Main Test Harness
// ============================================================================

int main() {
  std::cout << "=== Prototype P1: SLSQP Convergence Validation ===\n\n";

  // Test parameters
  constexpr int numContacts = 4;
  constexpr double mu = 0.5;
  constexpr int numFrames = 100;

  // Generate synthetic sliding contact QP
  auto qp = FrictionQP::generateSlidingContact(numContacts, mu);

  std::cout << "Test configuration:\n";
  std::cout << "  Contacts: " << numContacts << "\n";
  std::cout << "  Friction coefficient: " << mu << "\n";
  std::cout << "  Variables: " << 3 * numContacts << "\n";
  std::cout << "  Frames: " << numFrames << "\n\n";

  // Results tracking
  int successCount = 0;
  int failureCount = 0;
  double maxViolation = 0.0;
  double maxEnergyInjection = 0.0;
  std::vector<int> iterationCounts;

  // CSV output for analysis
  std::ofstream csv{"p1_results.csv"};
  csv << "frame,converged,iterations,max_constraint_violation,energy_injection\n";

  // Cold start
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(3 * numContacts);

  // Simulate frames with warm starting
  for (int frame = 0; frame < numFrames; ++frame) {
    auto result = solveWithSLSQP(qp, lambda);

    if (result.converged) {
      ++successCount;
    } else {
      ++failureCount;
    }

    double maxConstraintViolation = *std::max_element(result.constraintViolations.begin(),
                                                      result.constraintViolations.end());
    maxViolation = std::max(maxViolation, maxConstraintViolation);

    double energyInjection = computeEnergyInjection(result.lambda, qp.b);
    maxEnergyInjection = std::max(maxEnergyInjection, std::abs(energyInjection));

    iterationCounts.push_back(result.iterations);

    csv << frame << "," << result.converged << "," << result.iterations << ","
        << maxConstraintViolation << "," << energyInjection << "\n";

    // Warm start next frame
    lambda = result.lambda;
  }

  csv.close();

  // Summary statistics
  double avgIterations = 0.0;
  for (int iters : iterationCounts) {
    avgIterations += iters;
  }
  avgIterations /= numFrames;

  std::cout << "\n=== Results Summary ===\n";
  std::cout << "Convergence rate: " << successCount << "/" << numFrames
            << " (" << (100.0 * successCount / numFrames) << "%)\n";
  std::cout << "Failures: " << failureCount << "\n";
  std::cout << "Average iterations: " << avgIterations << "\n";
  std::cout << "Max constraint violation: " << maxViolation << "\n";
  std::cout << "Max energy injection: " << maxEnergyInjection << " J\n";

  // Success criteria evaluation
  std::cout << "\n=== Success Criteria ===\n";
  bool pass_convergence = (successCount == numFrames);
  bool pass_violation = (maxViolation < 1e-6);
  bool pass_energy = (maxEnergyInjection < 0.01);

  std::cout << "[" << (pass_convergence ? "PASS" : "FAIL") << "] 100% convergence: "
            << (pass_convergence ? "yes" : "no") << "\n";
  std::cout << "[" << (pass_violation ? "PASS" : "FAIL")
            << "] Max constraint violation < 1e-6: " << (pass_violation ? "yes" : "no") << "\n";
  std::cout << "[" << (pass_energy ? "PASS" : "FAIL")
            << "] Max energy injection < 0.01 J: " << (pass_energy ? "yes" : "no") << "\n";

  bool allPass = pass_convergence && pass_violation && pass_energy;
  std::cout << "\n=== Overall Result: " << (allPass ? "VALIDATED" : "INVALIDATED") << " ===\n";

  return allPass ? 0 : 1;
}
