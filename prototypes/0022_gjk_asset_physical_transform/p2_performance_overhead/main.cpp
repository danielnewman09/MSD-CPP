// Prototype P2: Performance Overhead Measurement
// Question: What is the performance overhead of on-the-fly transformation compared to
//           identity-transform baseline, and does it meet the <20% threshold for typical hulls?
// Success criteria:
//   - Transformation overhead < 10% for simple hulls (< 20 vertices)
//   - Transformation overhead < 20% for typical hulls (20-100 vertices)
//   - Transformation overhead < 30% for complex hulls (100-1000 vertices)

#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Include minimal types from codebase
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/Angle.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"

using namespace msd_sim;

// Minimal ReferenceFrame implementation (same as P1)
class SimpleReferenceFrame {
public:
    SimpleReferenceFrame()
        : origin_{0, 0, 0}
        , rotation_{Eigen::Matrix3d::Identity()}
    {}

    explicit SimpleReferenceFrame(const Coordinate& origin)
        : origin_{origin}
        , rotation_{Eigen::Matrix3d::Identity()}
    {}

    SimpleReferenceFrame(const Coordinate& origin, const EulerAngles& euler)
        : origin_{origin}
    {
        updateRotationMatrix(euler);
    }

    Coordinate globalToLocalRelative(const Coordinate& globalVector) const {
        Eigen::Vector3d result = rotation_.transpose() * globalVector;
        return Coordinate{result};
    }

    Coordinate localToGlobal(const Coordinate& localPoint) const {
        Eigen::Vector3d result = rotation_ * localPoint + origin_;
        return Coordinate{result};
    }

private:
    void updateRotationMatrix(const EulerAngles& euler) {
        double roll = euler.roll.getRad();
        double pitch = euler.pitch.getRad();
        double yaw = euler.yaw.getRad();

        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        rotation_ = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
    }

    Coordinate origin_;
    Eigen::Matrix3d rotation_;
};

// Simple support function
Coordinate supportFunction(const std::vector<Coordinate>& vertices, const Coordinate& direction) {
    double maxDot = -std::numeric_limits<double>::infinity();
    Coordinate maxVertex = vertices[0];

    for (const auto& vertex : vertices) {
        double dot = vertex.dot(direction);
        if (dot > maxDot) {
            maxDot = dot;
            maxVertex = vertex;
        }
    }

    return maxVertex;
}

// Transformed support function
Coordinate transformedSupport(const std::vector<Coordinate>& localVertices,
                               const SimpleReferenceFrame& frame,
                               const Coordinate& worldDirection) {
    Coordinate localDirection = frame.globalToLocalRelative(worldDirection);
    Coordinate localSupport = supportFunction(localVertices, localDirection);
    Coordinate worldSupport = frame.localToGlobal(localSupport);
    return worldSupport;
}

// Generate random vertices for a convex hull (approximate sphere)
std::vector<Coordinate> generateRandomHull(size_t vertexCount) {
    std::mt19937 rng{42};  // Fixed seed for reproducibility
    std::uniform_real_distribution<double> dist{-1.0, 1.0};

    std::vector<Coordinate> vertices;
    vertices.reserve(vertexCount);

    for (size_t i = 0; i < vertexCount; ++i) {
        Coordinate v{dist(rng), dist(rng), dist(rng)};
        v.normalize();  // Project to unit sphere
        vertices.push_back(v);
    }

    return vertices;
}

// Generate random search directions
std::vector<Coordinate> generateRandomDirections(size_t count) {
    std::mt19937 rng{123};  // Fixed seed
    std::uniform_real_distribution<double> dist{-1.0, 1.0};

    std::vector<Coordinate> directions;
    directions.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        Coordinate dir{dist(rng), dist(rng), dist(rng)};
        dir.normalize();
        directions.push_back(dir);
    }

    return directions;
}

struct BenchmarkResult {
    size_t vertexCount;
    double identityTime_us;
    double transformedTime_us;
    double overhead_percent;
};

BenchmarkResult benchmarkHullSize(size_t vertexCount, size_t numDirections, size_t iterations) {
    auto vertices = generateRandomHull(vertexCount);
    auto directions = generateRandomDirections(numDirections);

    // Identity frame
    SimpleReferenceFrame identityFrame{};

    // Non-trivial transform
    EulerAngles rotation{
        Angle::fromDegrees(15),
        Angle::fromDegrees(30),
        Angle::fromDegrees(45)
    };
    SimpleReferenceFrame transformedFrame{Coordinate{10, 20, 30}, rotation};

    // Benchmark identity transform
    double sumX = 0.0;  // Accumulate to prevent optimization
    auto identityStart = std::chrono::high_resolution_clock::now();
    for (size_t iter = 0; iter < iterations; ++iter) {
        for (const auto& dir : directions) {
            Coordinate result = transformedSupport(vertices, identityFrame, dir);
            sumX += result.x();  // Use result to prevent optimization
        }
    }
    auto identityEnd = std::chrono::high_resolution_clock::now();
    auto identityDuration = std::chrono::duration_cast<std::chrono::microseconds>(identityEnd - identityStart);

    // Benchmark transformed
    auto transformedStart = std::chrono::high_resolution_clock::now();
    for (size_t iter = 0; iter < iterations; ++iter) {
        for (const auto& dir : directions) {
            Coordinate result = transformedSupport(vertices, transformedFrame, dir);
            sumX += result.x();  // Use result to prevent optimization
        }
    }
    auto transformedEnd = std::chrono::high_resolution_clock::now();
    auto transformedDuration = std::chrono::duration_cast<std::chrono::microseconds>(transformedEnd - transformedStart);

    // Prevent sumX from being optimized away
    if (sumX == std::numeric_limits<double>::infinity()) {
        std::cout << "Impossible" << std::endl;
    }

    double identityTime = static_cast<double>(identityDuration.count()) / iterations;
    double transformedTime = static_cast<double>(transformedDuration.count()) / iterations;
    double overhead = ((transformedTime - identityTime) / identityTime) * 100.0;

    return BenchmarkResult{vertexCount, identityTime, transformedTime, overhead};
}

int main() {
    std::cout << "=== Prototype P2: Performance Overhead Measurement ===" << std::endl;
    std::cout << std::endl;

    const size_t numDirections = 100;  // Test with 100 directions
    const size_t iterations = 1000;     // Run 1000 iterations for stable timing

    std::vector<size_t> hullSizes = {10, 20, 50, 100, 200, 500, 1000};

    std::cout << "Benchmarking with " << numDirections << " directions, " << iterations << " iterations each" << std::endl;
    std::cout << std::endl;

    std::cout << std::setw(12) << "Vertices"
              << std::setw(18) << "Identity (μs)"
              << std::setw(20) << "Transformed (μs)"
              << std::setw(18) << "Overhead (%)"
              << std::setw(10) << "Status"
              << std::endl;
    std::cout << std::string(78, '-') << std::endl;

    int passCount = 0;
    int failCount = 0;

    for (size_t hullSize : hullSizes) {
        auto result = benchmarkHullSize(hullSize, numDirections, iterations);

        std::string status;
        std::string category;

        // Categorize and evaluate
        if (hullSize < 20) {
            category = "Simple";
            status = (result.overhead_percent < 10.0) ? "PASS" : "FAIL";
        } else if (hullSize <= 100) {
            category = "Typical";
            status = (result.overhead_percent < 20.0) ? "PASS" : "FAIL";
        } else {
            category = "Complex";
            status = (result.overhead_percent < 30.0) ? "PASS" : "FAIL";
        }

        if (status == "PASS") {
            passCount++;
        } else {
            failCount++;
        }

        std::cout << std::setw(12) << result.vertexCount
                  << std::setw(18) << std::fixed << std::setprecision(2) << result.identityTime_us
                  << std::setw(20) << result.transformedTime_us
                  << std::setw(18) << std::setprecision(1) << result.overhead_percent
                  << std::setw(10) << status
                  << std::endl;
    }

    std::cout << std::endl;
    std::cout << "=== Summary ===" << std::endl;
    std::cout << "Passed: " << passCount << "/" << hullSizes.size() << std::endl;
    std::cout << "Failed: " << failCount << "/" << hullSizes.size() << std::endl;
    std::cout << std::endl;

    std::cout << "Criteria:" << std::endl;
    std::cout << "  - Simple hulls (< 20 vertices): < 10% overhead" << std::endl;
    std::cout << "  - Typical hulls (20-100 vertices): < 20% overhead" << std::endl;
    std::cout << "  - Complex hulls (100-1000 vertices): < 30% overhead" << std::endl;
    std::cout << std::endl;

    if (failCount == 0) {
        std::cout << "SUCCESS: Performance overhead is ACCEPTABLE" << std::endl;
        return 0;
    } else {
        std::cout << "FAILURE: Performance overhead EXCEEDS thresholds" << std::endl;
        return 1;
    }
}
