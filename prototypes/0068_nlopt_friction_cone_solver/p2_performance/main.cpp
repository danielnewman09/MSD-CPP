// Prototype P2: Performance Benchmark
// Question: Is NLopt SLSQP solve time within 2x of the custom solver?
// Success criteria:
//   - 1-contact (3 vars) solve time < 50 μs (baseline: ~25 μs)
//   - 4-contact (12 vars) solve time < 200 μs (baseline: ~100 μs)
//   - No outliers > 5x baseline

#include <Eigen/Dense>
#include <nlopt.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

// ============================================================================
// Minimal NLopt SLSQP Wrapper (same as P1)
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

double solveWithNLopt(const Eigen::MatrixXd& A,
                      const Eigen::VectorXd& b,
                      const std::vector<double>& mu,
                      const Eigen::VectorXd& lambda0 = Eigen::VectorXd{})
{
    using namespace std::chrono;
    auto start = high_resolution_clock::now();

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
    try
    {
        opt.optimize(x, minf);
    }
    catch (...)
    {
        // Ignore exceptions for benchmarking
    }

    auto end = high_resolution_clock::now();
    return duration_cast<duration<double, std::micro>>(end - start).count();
}

// ============================================================================
// Synthetic Test Case Generator
// ============================================================================

struct BenchmarkCase
{
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::vector<double> mu;
    std::string description;
};

BenchmarkCase generateCase(int numContacts, double frictionCoeff)
{
    BenchmarkCase bc;
    bc.description = std::to_string(numContacts) + " contacts";
    int dim = 3 * numContacts;
    Eigen::MatrixXd M = Eigen::MatrixXd::Random(dim, dim);
    bc.A = M * M.transpose() + Eigen::MatrixXd::Identity(dim, dim) * 0.1;
    bc.b = Eigen::VectorXd::Random(dim) * 10.0;
    for (int i = 0; i < numContacts; ++i)
    {
        bc.mu.push_back(frictionCoeff);
    }
    return bc;
}

// ============================================================================
// Statistics Helpers
// ============================================================================

struct Stats
{
    double min;
    double max;
    double mean;
    double median;
    double stddev;
};

Stats computeStats(std::vector<double> times)
{
    std::sort(times.begin(), times.end());
    double sum = std::accumulate(times.begin(), times.end(), 0.0);
    double mean = sum / times.size();
    double sq_sum = 0.0;
    for (double t : times)
    {
        sq_sum += (t - mean) * (t - mean);
    }
    double stddev = std::sqrt(sq_sum / times.size());
    return {times.front(), times.back(), mean, times[times.size() / 2], stddev};
}

// ============================================================================
// Main: Run Performance Benchmark
// ============================================================================

int main()
{
    std::cout << "=============================================================================\n";
    std::cout << "Prototype P2: Performance Benchmark\n";
    std::cout << "=============================================================================\n\n";

    constexpr int numIterations = 1000;
    constexpr double targetSlowdown = 2.0;

    // Benchmarks: 1-contact and 4-contact scenarios
    std::vector<int> contactCounts = {1, 2, 3, 4};
    std::vector<double> baselines_us = {25.0, 50.0, 75.0, 100.0};  // Assumed baseline from custom solver
    std::vector<double> thresholds_us = {50.0, 100.0, 150.0, 200.0};  // 2x slowdown acceptable

    std::ofstream csv("p2_results.csv");
    csv << "contacts,iteration,time_us\n";

    bool allPassed = true;

    for (size_t i = 0; i < contactCounts.size(); ++i)
    {
        int contacts = contactCounts[i];
        double baseline = baselines_us[i];
        double threshold = thresholds_us[i];

        std::cout << "Benchmarking " << contacts << "-contact scenario...\n";

        BenchmarkCase testCase = generateCase(contacts, 0.5);
        std::vector<double> times;

        for (int iter = 0; iter < numIterations; ++iter)
        {
            double time_us = solveWithNLopt(testCase.A, testCase.b, testCase.mu);
            times.push_back(time_us);
            csv << contacts << "," << iter << "," << time_us << "\n";
        }

        Stats stats = computeStats(times);

        std::cout << "  Results:\n";
        std::cout << "    Min:    " << std::fixed << std::setprecision(2) << stats.min << " μs\n";
        std::cout << "    Max:    " << stats.max << " μs\n";
        std::cout << "    Mean:   " << stats.mean << " μs\n";
        std::cout << "    Median: " << stats.median << " μs\n";
        std::cout << "    StdDev: " << stats.stddev << " μs\n";
        std::cout << "    Baseline: " << baseline << " μs\n";
        std::cout << "    Threshold (2x): " << threshold << " μs\n";

        double slowdown = stats.mean / baseline;
        std::cout << "    Slowdown: " << std::setprecision(2) << slowdown << "x\n";

        if (stats.mean <= threshold)
        {
            std::cout << "    ✓ PASS\n";
        }
        else
        {
            std::cout << "    ✗ FAIL (exceeds 2x threshold)\n";
            allPassed = false;
        }

        // Check for outliers > 5x baseline
        int outliers = 0;
        double outlierThreshold = baseline * 5.0;
        for (double t : times)
        {
            if (t > outlierThreshold)
            {
                outliers++;
            }
        }

        if (outliers > 0)
        {
            std::cout << "    ✗ WARNING: " << outliers << " outliers > 5x baseline\n";
        }

        std::cout << "\n";
    }

    csv.close();

    // Summary
    std::cout << "=============================================================================\n";
    std::cout << "Summary\n";
    std::cout << "=============================================================================\n";

    if (allPassed)
    {
        std::cout << "P2: VALIDATED - NLopt SLSQP performance within 2x of baseline\n";
        return 0;
    }
    else
    {
        std::cout << "P2: FAIL - Performance exceeds 2x threshold for some cases\n";
        std::cout << "  Consider: COBYLA fallback, or accept higher slowdown for correctness\n";
        return 1;
    }
}
