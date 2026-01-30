// Diagnostic Test: Root-cause analysis for Baumgarte instability
// Tests H1: Unit mismatch between acceleration-level alpha and velocity-level ERP
//
// The design document recommends alpha=100 [1/s²] (acceleration-level Baumgarte).
// The prototype uses alpha as a dimensionless ERP fraction in: b = -(alpha/dt)*C
// These are different parameterizations of the same correction.
//
// Conversion: ERP = alpha_accel * dt²
//   alpha_accel=100 → ERP = 100 * 0.016² = 0.0256
//   ERP=0.2 → alpha_accel = 0.2 / 0.016² = 781.25

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>

constexpr double dt = 0.016;
constexpr double g = -9.81;
constexpr double mass = 1.0;
constexpr double cubeHeight = 1.0;
constexpr double e = 0.0;
constexpr int maxPGSIterations = 10;
constexpr int numFrames = 200;  // Shorter for diagnostic

struct Body {
    double z;
    double vz;
    double invM;
    Body(double z0, double m) : z{z0}, vz{0.0}, invM{1.0 / m} {}
};

struct Contact {
    int bodyA;
    int bodyB;
    Contact(int a, int b) : bodyA{a}, bodyB{b} {}

    double getGap(const std::vector<Body>& bodies) const {
        if (bodyA == -1) {
            return bodies[bodyB].z - cubeHeight / 2.0;
        }
        return (bodies[bodyB].z - cubeHeight / 2.0) -
               (bodies[bodyA].z + cubeHeight / 2.0);
    }

    double getGapVelocity(const std::vector<Body>& bodies) const {
        if (bodyA == -1) {
            return bodies[bodyB].vz;
        }
        return bodies[bodyB].vz - bodies[bodyA].vz;
    }
};

struct SimResult {
    double maxDrift;
    double maxVelocity;
    double maxEnergy;
    double minEnergy;
    bool stable;
};

// Run simulation with a given alpha value (ERP formulation: b -= (alpha/dt)*C)
SimResult runWithERP(double alpha_erp, const std::string& label) {
    std::vector<Body> bodies;
    bodies.emplace_back(0.5, mass);
    bodies.emplace_back(1.5, mass);
    bodies.emplace_back(2.5, mass);

    std::vector<double> initialZ;
    for (const auto& b : bodies) initialZ.push_back(b.z);

    std::vector<Contact> contacts;
    contacts.emplace_back(-1, 0);
    contacts.emplace_back(0, 1);
    contacts.emplace_back(1, 2);
    const int n = 3;

    // Build A matrix (constant for this scenario)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double aij = 0.0;
            if (contacts[i].bodyB == contacts[j].bodyB)
                aij += bodies[contacts[i].bodyB].invM;
            if (contacts[i].bodyA >= 0 && contacts[i].bodyA == contacts[j].bodyA)
                aij += bodies[contacts[i].bodyA].invM;
            if (contacts[i].bodyB == contacts[j].bodyA && contacts[j].bodyA >= 0)
                aij -= bodies[contacts[i].bodyB].invM;
            if (contacts[i].bodyA >= 0 && contacts[i].bodyA == contacts[j].bodyB)
                aij -= bodies[contacts[i].bodyA].invM;
            A(i, j) = aij;
        }
    }

    SimResult result{};
    result.maxDrift = 0.0;
    result.maxVelocity = 0.0;
    result.maxEnergy = -1e30;
    result.minEnergy = 1e30;
    result.stable = true;

    for (int frame = 0; frame < numFrames; ++frame) {
        // Apply gravity
        for (auto& b : bodies) b.vz += g * dt;

        // Build RHS
        Eigen::VectorXd b_vec(n);
        for (int i = 0; i < n; ++i) {
            double gap = contacts[i].getGap(bodies);
            double gapVel = contacts[i].getGapVelocity(bodies);
            b_vec(i) = -(1.0 + e) * gapVel;
            if (gap < 0.0) {
                b_vec(i) -= (alpha_erp / dt) * gap;
            }
        }

        // PGS solve
        Eigen::VectorXd lambda = Eigen::VectorXd::Zero(n);
        for (int iter = 0; iter < maxPGSIterations; ++iter) {
            for (int i = 0; i < n; ++i) {
                double residual = b_vec(i);
                for (int j = 0; j < n; ++j) {
                    if (j != i) residual -= A(i, j) * lambda(j);
                }
                lambda(i) = std::max(0.0, residual / A(i, i));
            }
        }

        // Apply impulses
        for (int i = 0; i < n; ++i) {
            bodies[contacts[i].bodyB].vz += lambda(i) * bodies[contacts[i].bodyB].invM;
            if (contacts[i].bodyA >= 0) {
                bodies[contacts[i].bodyA].vz -= lambda(i) * bodies[contacts[i].bodyA].invM;
            }
        }

        // Integrate positions
        for (auto& b : bodies) b.z += b.vz * dt;

        // Check stability
        for (size_t i = 0; i < bodies.size(); ++i) {
            double drift = std::abs(bodies[i].z - initialZ[i]);
            result.maxDrift = std::max(result.maxDrift, drift);
            result.maxVelocity = std::max(result.maxVelocity, std::abs(bodies[i].vz));

            if (std::abs(bodies[i].z) > 100.0 || std::isnan(bodies[i].z)) {
                result.stable = false;
                return result;
            }
        }

        // Energy
        double energy = 0.0;
        for (const auto& b : bodies) {
            energy += 0.5 * mass * b.vz * b.vz;
            energy += mass * (-g) * b.z;
        }
        result.maxEnergy = std::max(result.maxEnergy, energy);
        result.minEnergy = std::min(result.minEnergy, energy);
    }

    return result;
}

int main() {
    std::cout << "=== Diagnostic Test: Baumgarte Parameter Analysis ===\n\n";

    // ---- Test 1: Confirm instability with alpha=100 in ERP formulation ----
    std::cout << "--- Test 1: Confirm alpha=100 causes instability ---\n";
    {
        auto result = runWithERP(100.0, "alpha=100 (design value, wrong units)");
        std::cout << "  ERP alpha=100, effective gain = " << (100.0 / dt) << " /s\n";
        std::cout << "  Equivalent accel-level alpha = " << (100.0 / (dt * dt)) << " /s^2\n";
        std::cout << "  Stable: " << (result.stable ? "YES" : "NO (EXPLOSION)") << "\n";
        if (result.stable) {
            std::cout << "  Max drift: " << result.maxDrift << " m\n";
        }
        std::cout << "\n";
    }

    // ---- Test 2: Verify correct ERP from acceleration-level alpha=100 ----
    std::cout << "--- Test 2: Convert alpha=100 [1/s^2] to ERP ---\n";
    {
        double alpha_accel = 100.0;
        double erp = alpha_accel * dt * dt;
        std::cout << "  alpha_accel = " << alpha_accel << " [1/s^2]\n";
        std::cout << "  ERP = alpha_accel * dt^2 = " << erp << "\n";
        std::cout << "  Effective gain = " << (erp / dt) << " /s\n";
        auto result = runWithERP(erp, "ERP from alpha=100");
        std::cout << "  Stable: " << (result.stable ? "YES" : "NO") << "\n";
        if (result.stable) {
            std::cout << "  Max drift: " << std::fixed << std::setprecision(6)
                      << result.maxDrift << " m\n";
            std::cout << "  Energy range: " << std::setprecision(4)
                      << (result.maxEnergy - result.minEnergy) << " J\n";
        }
        std::cout << "\n";
    }

    // ---- Test 3: Parameter sweep to find stability boundary ----
    std::cout << "--- Test 3: ERP Parameter Sweep ---\n";
    std::cout << std::setw(12) << "ERP"
              << std::setw(16) << "Gain [1/s]"
              << std::setw(20) << "Equiv α [1/s²]"
              << std::setw(10) << "Stable"
              << std::setw(14) << "Max Drift"
              << std::setw(14) << "Max Vel"
              << std::setw(16) << "Energy Range"
              << "\n";
    std::cout << std::string(102, '-') << "\n";

    std::vector<double> erpValues = {
        0.01, 0.02, 0.0256, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5,
        0.6, 0.8, 1.0, 1.5, 2.0, 5.0, 10.0, 50.0, 100.0
    };

    double stabilityBoundary = 0.0;
    for (double erp : erpValues) {
        auto result = runWithERP(erp, "");
        double gain = erp / dt;
        double alpha_equiv = erp / (dt * dt);

        std::cout << std::fixed
                  << std::setw(12) << std::setprecision(4) << erp
                  << std::setw(16) << std::setprecision(1) << gain
                  << std::setw(20) << std::setprecision(1) << alpha_equiv
                  << std::setw(10) << (result.stable ? "YES" : "NO");

        if (result.stable) {
            std::cout << std::setw(14) << std::setprecision(6) << result.maxDrift
                      << std::setw(14) << std::setprecision(6) << result.maxVelocity
                      << std::setw(16) << std::setprecision(4) << (result.maxEnergy - result.minEnergy);
            stabilityBoundary = erp;
        }
        std::cout << "\n";
    }

    std::cout << "\n  Stability boundary: ERP ~ " << std::setprecision(2)
              << stabilityBoundary << "\n\n";

    // ---- Test 4: Verify the working alpha=0.2 maps to reasonable acceleration-level alpha ----
    std::cout << "--- Test 4: Map prototype's alpha=0.2 to acceleration-level ---\n";
    {
        double erp = 0.2;
        double alpha_accel = erp / (dt * dt);
        std::cout << "  Prototype ERP = " << erp << "\n";
        std::cout << "  Equivalent alpha_accel = " << std::setprecision(1) << alpha_accel << " [1/s^2]\n";
        std::cout << "  Theoretical stiff alpha = " << (4.0 / (dt * dt)) << " [1/s^2]\n";
        std::cout << "  Ratio to stiff: " << std::setprecision(4) << (alpha_accel / (4.0 / (dt * dt))) << "\n";
        std::cout << "  Ratio to design alpha=100: " << std::setprecision(2) << (alpha_accel / 100.0) << "\n";
        std::cout << "\n";
    }

    // ---- Summary ----
    std::cout << "=== Summary ===\n\n";
    std::cout << "Root cause: UNIT MISMATCH between formulations.\n\n";
    std::cout << "The design document specifies alpha=100 in acceleration-level units [1/s^2].\n";
    std::cout << "The prototype uses alpha as a dimensionless ERP fraction in: b -= (alpha/dt)*C\n\n";
    std::cout << "Conversion: ERP = alpha_accel * dt^2\n";
    std::cout << "  Design alpha=100 [1/s^2] → ERP = " << std::setprecision(4)
              << (100.0 * dt * dt) << "\n";
    std::cout << "  Prototype alpha=0.2 [ERP] → alpha_accel = " << std::setprecision(1)
              << (0.2 / (dt * dt)) << " [1/s^2]\n\n";
    std::cout << "Plugging alpha=100 directly into ERP formula gives:\n";
    std::cout << "  Effective alpha_accel = " << std::setprecision(1) << (100.0 / (dt * dt))
              << " [1/s^2] — 25x stiffer than theoretical max!\n";

    return 0;
}
