// Prototype P3: Warm-Start Effectiveness
// Question: Does warm-starting from previous frame's lambda reduce iteration count by > 30%?
// Success criteria:
//   - Cold start (lambda0 = 0): average 15-25 iterations per solve
//   - Warm start (lambda0 = previous frame): average < 10 iterations per solve
//   - Iteration reduction > 30% for steady-state contact

#include <Eigen/Dense>
#include <nlopt.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

// ============================================================================
// Minimal NLopt SLSQP Wrapper with Iteration Tracking
// ============================================================================

struct ObjectiveData
{
    const Eigen::MatrixXd* A;
    const Eigen::VectorXd* b;
};

struct ConstraintData
{
    int contactIndex;
    double mu;
};

double objective(const std::vector<double>& lambda, std::vector<double>& grad, void* data)
{
    auto* objData = static_cast<ObjectiveData*>(data);
    const Eigen::MatrixXd& A = *objData->A;
    const Eigen::VectorXd& b = *objData->b;
    Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(lambda.data(), lambda.size());
    double f = 0.5 * x.dot(A * x) - b.dot(x);
    if (!grad.empty())
    {
        Eigen::VectorXd g = A * x - b;
        for (size_t i = 0; i < grad.size(); ++i)
        {
            grad[i] = g(i);
        }
    }
    return f;
}

double coneConstraint(const std::vector<double>& lambda, std::vector<double>& grad, void* data)
{
    auto* conData = static_cast<ConstraintData*>(data);
    int i = conData->contactIndex;
    double mu = conData->mu;
    double lambda_n = lambda[3 * i + 0];
    double lambda_t1 = lambda[3 * i + 1];
    double lambda_t2 = lambda[3 * i + 2];
    double c = mu * mu * lambda_n * lambda_n - lambda_t1 * lambda_t1 - lambda_t2 * lambda_t2;
    if (!grad.empty())
    {
        grad[3 * i + 0] = 2.0 * mu * mu * lambda_n;
        grad[3 * i + 1] = -2.0 * lambda_t1;
        grad[3 * i + 2] = -2.0 * lambda_t2;
    }
    return c;
}

struct SolveResult
{
    Eigen::VectorXd lambda;
    bool converged{false};
    int iterations{0};
};

SolveResult solveWithNLopt(const Eigen::MatrixXd& A,
                           const Eigen::VectorXd& b,
                           const std::vector<double>& mu,
                           const Eigen::VectorXd& lambda0 = Eigen::VectorXd{})
{
    int numContacts = static_cast<int>(mu.size());
    int dim = 3 * numContacts;
    nlopt::opt opt(nlopt::LD_SLSQP, dim);
    ObjectiveData objData{&A, &b};
    opt.set_min_objective(objective, &objData);
    std::vector<ConstraintData> constraintData;
    for (int i = 0; i < numContacts; ++i)
    {
        constraintData.push_back({i, mu[i]});
        opt.add_inequality_constraint(coneConstraint, &constraintData.back(), 1e-8);
    }
    std::vector<double> lb(dim, -HUGE_VAL);
    for (int i = 0; i < numContacts; ++i)
    {
        lb[3 * i + 0] = 0.0;
    }
    opt.set_lower_bounds(lb);
    opt.set_ftol_rel(1e-6);
    opt.set_xtol_rel(1e-6);
    opt.set_maxeval(100);
    std::vector<double> x(dim, 0.0);
    if (lambda0.size() == dim)
    {
        for (int i = 0; i < dim; ++i)
        {
            x[i] = lambda0(i);
        }
    }
    double minf;
    SolveResult result;
    try
    {
        opt.optimize(x, minf);
        result.lambda = Eigen::Map<Eigen::VectorXd>(x.data(), dim);
        result.converged = true;
        result.iterations = opt.get_numevals();
    }
    catch (...)
    {
        result.converged = false;
        result.lambda = Eigen::VectorXd::Zero(dim);
        result.iterations = opt.get_numevals();
    }
    return result;
}

// ============================================================================
// Synthetic Test Case Generator
// ============================================================================

struct TestCase
{
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::vector<double> mu;
};

TestCase generateCase(int numContacts, double frictionCoeff)
{
    TestCase tc;
    int dim = 3 * numContacts;
    Eigen::MatrixXd M = Eigen::MatrixXd::Random(dim, dim);
    tc.A = M * M.transpose() + Eigen::MatrixXd::Identity(dim, dim) * 0.1;
    tc.b = Eigen::VectorXd::Random(dim) * 10.0;
    for (int i = 0; i < numContacts; ++i)
    {
        tc.mu.push_back(frictionCoeff);
    }
    return tc;
}

// ============================================================================
// Main: Run Warm-Start Effectiveness Test
// ============================================================================

int main()
{
    std::cout << "=============================================================================\n";
    std::cout << "Prototype P3: Warm-Start Effectiveness\n";
    std::cout << "=============================================================================\n\n";

    constexpr int numFrames = 100;
    constexpr int numContacts = 2;
    constexpr double frictionCoeff = 0.5;

    // Generate steady-state test case (minimal frame-to-frame variation)
    TestCase baseCase = generateCase(numContacts, frictionCoeff);

    std::vector<int> coldIterations;
    std::vector<int> warmIterations;

    std::ofstream csv("p3_results.csv");
    csv << "frame,cold_iters,warm_iters,reduction_pct\n";

    Eigen::VectorXd lambda_prev;  // For warm start

    std::cout << "Simulating " << numFrames << " frames of steady-state contact (" << numContacts << " contacts)...\n\n";

    for (int frame = 0; frame < numFrames; ++frame)
    {
        // Slight perturbation to simulate frame-to-frame variation in steady-state
        Eigen::MatrixXd A_perturbed = baseCase.A + Eigen::MatrixXd::Random(baseCase.A.rows(), baseCase.A.cols()) * 0.001;
        Eigen::VectorXd b_perturbed = baseCase.b + Eigen::VectorXd::Random(baseCase.b.size()) * 0.01;

        // Cold start (lambda0 = zeros)
        auto coldResult = solveWithNLopt(A_perturbed, b_perturbed, baseCase.mu);
        coldIterations.push_back(coldResult.iterations);

        // Warm start (lambda0 = previous frame's solution)
        Eigen::VectorXd warmStart = (lambda_prev.size() > 0) ? lambda_prev : Eigen::VectorXd::Zero(3 * numContacts);
        auto warmResult = solveWithNLopt(A_perturbed, b_perturbed, baseCase.mu, warmStart);
        warmIterations.push_back(warmResult.iterations);

        // Update warm start for next frame
        lambda_prev = warmResult.lambda;

        // Compute reduction percentage
        double reduction = (coldResult.iterations > 0)
                               ? 100.0 * (coldResult.iterations - warmResult.iterations) / coldResult.iterations
                               : 0.0;

        csv << frame << "," << coldResult.iterations << "," << warmResult.iterations << "," << reduction << "\n";
    }

    csv.close();

    // Compute statistics
    double coldMean = std::accumulate(coldIterations.begin(), coldIterations.end(), 0.0) / coldIterations.size();
    double warmMean = std::accumulate(warmIterations.begin(), warmIterations.end(), 0.0) / warmIterations.size();
    double reductionPct = 100.0 * (coldMean - warmMean) / coldMean;

    std::sort(coldIterations.begin(), coldIterations.end());
    std::sort(warmIterations.begin(), warmIterations.end());
    int coldMedian = coldIterations[coldIterations.size() / 2];
    int warmMedian = warmIterations[warmIterations.size() / 2];

    // Results
    std::cout << "=============================================================================\n";
    std::cout << "Results\n";
    std::cout << "=============================================================================\n";
    std::cout << "Cold Start:\n";
    std::cout << "  Mean:   " << std::fixed << std::setprecision(1) << coldMean << " iterations\n";
    std::cout << "  Median: " << coldMedian << " iterations\n";
    std::cout << "  Min:    " << coldIterations.front() << " iterations\n";
    std::cout << "  Max:    " << coldIterations.back() << " iterations\n";
    std::cout << "\n";
    std::cout << "Warm Start:\n";
    std::cout << "  Mean:   " << warmMean << " iterations\n";
    std::cout << "  Median: " << warmMedian << " iterations\n";
    std::cout << "  Min:    " << warmIterations.front() << " iterations\n";
    std::cout << "  Max:    " << warmIterations.back() << " iterations\n";
    std::cout << "\n";
    std::cout << "Reduction: " << std::setprecision(1) << reductionPct << "%\n";
    std::cout << "=============================================================================\n\n";

    // Success criteria check
    bool success = true;
    std::cout << "Success Criteria:\n";

    if (coldMean >= 15.0 && coldMean <= 25.0)
    {
        std::cout << "  ✓ Cold start average within expected range (15-25 iterations)\n";
    }
    else if (coldMean < 15.0)
    {
        std::cout << "  ⚠ Cold start average < 15 iterations (problem may be too easy)\n";
    }
    else
    {
        std::cout << "  ✗ Cold start average > 25 iterations (convergence slower than expected)\n";
    }

    if (warmMean < 10.0)
    {
        std::cout << "  ✓ Warm start average < 10 iterations\n";
    }
    else
    {
        std::cout << "  ✗ FAIL: Warm start average >= 10 iterations\n";
        success = false;
    }

    if (reductionPct > 30.0)
    {
        std::cout << "  ✓ Iteration reduction > 30%\n";
    }
    else
    {
        std::cout << "  ✗ FAIL: Iteration reduction < 30% (warm start not effective enough)\n";
        success = false;
    }

    std::cout << "\n";
    if (success)
    {
        std::cout << "P3: VALIDATED - Warm-start reduces iterations by > 30%\n";
        return 0;
    }
    else
    {
        std::cout << "P3: PARTIAL - Warm-start provides some benefit but < 30% reduction\n";
        std::cout << "  Consider: Keep warm-start if reduction > 10%, otherwise disable\n";
        return 1;
    }
}
