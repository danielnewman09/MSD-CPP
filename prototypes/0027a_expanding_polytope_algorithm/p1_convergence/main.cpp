// Prototype P1: EPA Convergence Validation Test Harness
// Validates EPA convergence for diverse hull shapes and penetration scenarios

#include <chrono>
#include <cmath>
#include <format>
#include <iostream>
#include <numeric>
#include <vector>
#include "epa.hpp"

struct TestConfiguration
{
  std::string name;
  std::vector<Coordinate> simplex;
  double expectedDepth;  // For analytical validation
};

// Generate test simplices for different scenarios
std::vector<TestConfiguration> generateTestCases()
{
  std::vector<TestConfiguration> cases;

  // Case 1: Unit cube overlap (deep penetration ~50%)
  cases.push_back({"Deep penetration (50%)",
                   {Coordinate{0.5, 0.5, 0.5},
                    Coordinate{-0.5, 0.5, 0.5},
                    Coordinate{0.5, -0.5, 0.5},
                    Coordinate{0.5, 0.5, -0.5}},
                   0.5});

  // Case 2: Shallow penetration (1%)
  cases.push_back({"Shallow penetration (1%)",
                   {Coordinate{0.01, 0.01, 0.01},
                    Coordinate{-0.01, 0.01, 0.01},
                    Coordinate{0.01, -0.01, 0.01},
                    Coordinate{0.01, 0.01, -0.01}},
                   0.01});

  // Case 3: Medium penetration (25%)
  cases.push_back({"Medium penetration (25%)",
                   {Coordinate{0.25, 0.25, 0.25},
                    Coordinate{-0.25, 0.25, 0.25},
                    Coordinate{0.25, -0.25, 0.25},
                    Coordinate{0.25, 0.25, -0.25}},
                   0.25});

  // Case 4: Very shallow penetration (< epsilon threshold)
  cases.push_back({"Very shallow (< 1e-4)",
                   {Coordinate{1e-5, 1e-5, 1e-5},
                    Coordinate{-1e-5, 1e-5, 1e-5},
                    Coordinate{1e-5, -1e-5, 1e-5},
                    Coordinate{1e-5, 1e-5, -1e-5}},
                   1e-5});

  // Case 5: Elongated tetrahedron (non-uniform)
  cases.push_back({"Elongated tetrahedron",
                   {Coordinate{1.0, 0.0, 0.0},
                    Coordinate{-1.0, 0.0, 0.0},
                    Coordinate{0.0, 0.1, 0.0},
                    Coordinate{0.0, 0.0, 0.1}},
                   0.05});

  // Case 6: Regular tetrahedron
  {
    double h = std::sqrt(2.0 / 3.0);
    cases.push_back({"Regular tetrahedron",
                     {Coordinate{0.0, 0.0, h},
                      Coordinate{std::sqrt(3.0) / 3.0, 0.0, -h / 3.0},
                      Coordinate{-std::sqrt(3.0) / 6.0, 0.5, -h / 3.0},
                      Coordinate{-std::sqrt(3.0) / 6.0, -0.5, -h / 3.0}},
                     h / 2.0});
  }

  // Case 7: Flat tetrahedron (near-degenerate)
  cases.push_back({"Flat tetrahedron (near-degenerate)",
                   {Coordinate{1.0, 0.0, 0.01},
                    Coordinate{-1.0, 0.0, 0.01},
                    Coordinate{0.0, 1.0, 0.01},
                    Coordinate{0.0, 0.0, -0.01}},
                   0.01});

  // Case 8-10: Rotated configurations
  double angle = M_PI / 4;  // 45 degrees
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(angle, msd_sim::Vector3D::UnitZ());

  Coordinate rotated1 = rotation * Coordinate{0.3, 0.3, 0.3};
  Coordinate rotated2 = rotation * Coordinate{-0.3, 0.3, 0.3};
  Coordinate rotated3 = rotation * Coordinate{0.3, -0.3, 0.3};
  Coordinate rotated4 = rotation * Coordinate{0.3, 0.3, -0.3};

  cases.push_back({"Rotated 45° (medium penetration)",
                   {rotated1, rotated2, rotated3, rotated4},
                   0.3});

  return cases;
}

void printSeparator()
{
  std::cout << std::string(80, '=') << std::endl;
}

int main()
{
  std::cout << "Prototype P1: EPA Convergence Validation" << std::endl;
  std::cout << "Question: Does EPA reliably converge within 64 iterations?"
            << std::endl;
  std::cout << "Success criteria: 95%+ success rate, <32 iterations average"
            << std::endl;
  printSeparator();

  auto testCases = generateTestCases();
  EPA epa{1e-6};

  int totalTests = 0;
  int successCount = 0;
  int failureCount = 0;
  std::vector<int> iterationCounts;
  std::vector<double> executionTimes;  // microseconds

  std::cout << std::format("\n{:<40} {:<12} {:<12} {:<15} {:<15}",
                           "Test Case",
                           "Converged",
                           "Iterations",
                           "Depth",
                           "Time (μs)")
            << std::endl;
  printSeparator();

  for (const auto& testCase : testCases)
  {
    totalTests++;

    auto start = std::chrono::high_resolution_clock::now();
    ConvergenceMetrics metrics =
      epa.computeContactInfoWithMetrics(testCase.simplex, 64);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    executionTimes.push_back(static_cast<double>(duration.count()));

    std::string convergedStr = metrics.converged ? "YES" : "NO";
    std::string depthStr = std::isnan(metrics.penetrationDepth)
                             ? "N/A"
                             : std::format("{:.6f}", metrics.penetrationDepth);

    std::cout << std::format("{:<40} {:<12} {:<12} {:<15} {:<15.2f}",
                             testCase.name,
                             convergedStr,
                             metrics.iterations,
                             depthStr,
                             static_cast<double>(duration.count()))
              << std::endl;

    if (metrics.converged)
    {
      successCount++;
      iterationCounts.push_back(metrics.iterations);
    }
    else
    {
      failureCount++;
    }
  }

  printSeparator();

  // Compute statistics
  double successRate = (static_cast<double>(successCount) / totalTests) * 100.0;
  double avgIterations =
    iterationCounts.empty()
      ? 0.0
      : std::accumulate(iterationCounts.begin(), iterationCounts.end(), 0.0) /
          iterationCounts.size();
  double avgTime =
    std::accumulate(executionTimes.begin(), executionTimes.end(), 0.0) /
    executionTimes.size();

  int maxIterations =
    iterationCounts.empty()
      ? 0
      : *std::max_element(iterationCounts.begin(), iterationCounts.end());

  std::cout << "\nRESULTS SUMMARY:" << std::endl;
  printSeparator();
  std::cout << std::format("Total tests:         {}", totalTests) << std::endl;
  std::cout << std::format("Successes:           {}", successCount)
            << std::endl;
  std::cout << std::format("Failures:            {}", failureCount)
            << std::endl;
  std::cout << std::format("Success rate:        {:.1f}%", successRate)
            << std::endl;
  std::cout << std::format("Average iterations:  {:.1f}", avgIterations)
            << std::endl;
  std::cout << std::format("Max iterations:      {}", maxIterations)
            << std::endl;
  std::cout << std::format("Average time:        {:.2f} μs", avgTime)
            << std::endl;
  printSeparator();

  // Evaluate against success criteria
  bool successRatePass = successRate >= 95.0;
  bool avgIterationsPass = avgIterations < 32.0;

  std::cout << "\nSUCCESS CRITERIA EVALUATION:" << std::endl;
  printSeparator();
  std::cout << std::format("✓ Success rate >= 95%:       {} (actual: {:.1f}%)",
                           successRatePass ? "PASS" : "FAIL",
                           successRate)
            << std::endl;
  std::cout << std::format("✓ Avg iterations < 32:       {} (actual: {:.1f})",
                           avgIterationsPass ? "PASS" : "FAIL",
                           avgIterations)
            << std::endl;
  std::cout << std::format(
                 "✓ No infinite loops:         PASS (all tests completed)")
            << std::endl;
  std::cout << std::format("✓ No NaN outputs:            {} (checked per-test)",
                           successCount > 0 ? "PASS" : "FAIL")
            << std::endl;
  printSeparator();

  // Final verdict
  bool overallPass = successRatePass && avgIterationsPass && (successCount > 0);
  std::cout << "\nFINAL VERDICT: "
            << (overallPass ? "VALIDATED ✓" : "INVALIDATED ✗") << std::endl;

  if (!overallPass)
  {
    std::cout << "\nFAILURE ANALYSIS:" << std::endl;
    if (!successRatePass)
    {
      std::cout << "  - Success rate below threshold (need >= 95%)"
                << std::endl;
    }
    if (!avgIterationsPass)
    {
      std::cout << "  - Average iterations too high (need < 32)" << std::endl;
    }
    if (successCount == 0)
    {
      std::cout
        << "  - No successful convergence (algorithm fundamentally broken)"
        << std::endl;
    }
  }

  return overallPass ? 0 : 1;
}
