// Diagnostic: Long-run stability test for alpha=100 and other high values
// Check whether "explosive instability" develops over longer timeframes

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

constexpr double dt = 0.016;
constexpr double g = -9.81;
constexpr double mass = 1.0;
constexpr double cubeHeight = 1.0;
constexpr double e = 0.0;
constexpr int maxPGSIterations = 10;

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
        if (bodyA == -1) return bodies[bodyB].z - cubeHeight / 2.0;
        return (bodies[bodyB].z - cubeHeight / 2.0) - (bodies[bodyA].z + cubeHeight / 2.0);
    }

    double getGapVelocity(const std::vector<Body>& bodies) const {
        if (bodyA == -1) return bodies[bodyB].vz;
        return bodies[bodyB].vz - bodies[bodyA].vz;
    }
};

void runSimulation(double alpha_erp, int numFrames, const std::string& label) {
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

    std::cout << label << " (ERP=" << alpha_erp
              << ", frames=" << numFrames << "):\n";

    double maxDrift = 0.0;
    double maxVel = 0.0;
    double maxEnergy = -1e30;
    double minEnergy = 1e30;
    bool exploded = false;

    for (int frame = 0; frame < numFrames; ++frame) {
        for (auto& b : bodies) b.vz += g * dt;

        Eigen::VectorXd b_vec(n);
        for (int i = 0; i < n; ++i) {
            double gap = contacts[i].getGap(bodies);
            double gapVel = contacts[i].getGapVelocity(bodies);
            b_vec(i) = -(1.0 + e) * gapVel;
            if (gap < 0.0) {
                b_vec(i) -= (alpha_erp / dt) * gap;
            }
        }

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

        for (int i = 0; i < n; ++i) {
            bodies[contacts[i].bodyB].vz += lambda(i) * bodies[contacts[i].bodyB].invM;
            if (contacts[i].bodyA >= 0) {
                bodies[contacts[i].bodyA].vz -= lambda(i) * bodies[contacts[i].bodyA].invM;
            }
        }

        for (auto& b : bodies) b.z += b.vz * dt;

        // Track metrics
        double energy = 0.0;
        for (size_t i = 0; i < bodies.size(); ++i) {
            double drift = std::abs(bodies[i].z - initialZ[i]);
            maxDrift = std::max(maxDrift, drift);
            maxVel = std::max(maxVel, std::abs(bodies[i].vz));
            energy += 0.5 * mass * bodies[i].vz * bodies[i].vz;
            energy += mass * (-g) * bodies[i].z;

            if (std::abs(bodies[i].z) > 1000.0 || std::isnan(bodies[i].z)) {
                std::cout << "  EXPLOSION at frame " << frame
                          << " (body " << i << " z=" << bodies[i].z << ")\n";
                exploded = true;
                break;
            }
        }
        if (exploded) break;

        maxEnergy = std::max(maxEnergy, energy);
        minEnergy = std::min(minEnergy, energy);

        // Print snapshots at key frames
        if (frame == 0 || frame == 9 || frame == 99 || frame == 999 ||
            frame == 4999 || frame == 9999 || frame == numFrames - 1) {
            std::cout << "  Frame " << std::setw(5) << frame
                      << ": z=[" << std::fixed << std::setprecision(4)
                      << bodies[0].z << ", " << bodies[1].z << ", " << bodies[2].z
                      << "] v=[" << std::setprecision(4)
                      << bodies[0].vz << ", " << bodies[1].vz << ", " << bodies[2].vz
                      << "] drift=" << std::scientific << std::setprecision(2) << maxDrift
                      << "\n";
        }
    }

    if (!exploded) {
        std::cout << "  Final: maxDrift=" << std::scientific << std::setprecision(4) << maxDrift
                  << " m, maxVel=" << maxVel
                  << " m/s, energyRange=" << (maxEnergy - minEnergy) << " J\n";

        // Check pass/fail against P1 criteria
        bool passDrift = maxDrift < 0.01;
        bool passEnergy = (maxEnergy - minEnergy) < 5.0;
        std::cout << "  P1 position criteria (<0.01m): " << (passDrift ? "PASS" : "FAIL") << "\n";
        std::cout << "  P1 energy criteria (<5.0J):     " << (passEnergy ? "PASS" : "FAIL") << "\n";
    }
    std::cout << "\n";
}

int main() {
    std::cout << "=== Long-Run Stability Analysis ===\n\n";

    // Test alpha=100 (ERP) at 1000 frames (same as original P1)
    runSimulation(100.0, 1000, "Test A: alpha=100 (direct ERP), 1000 frames");

    // Test alpha=100 (ERP) at 10000 frames
    runSimulation(100.0, 10000, "Test B: alpha=100 (direct ERP), 10000 frames");

    // Test alpha=0.2 at 10000 frames (reference)
    runSimulation(0.2, 10000, "Test C: alpha=0.2 (working ERP), 10000 frames");

    // Test ERP=0.0256 (correct conversion from alpha_accel=100) at 10000 frames
    runSimulation(0.0256, 10000, "Test D: ERP=0.0256 (alpha_accel=100), 10000 frames");

    // Test borderline values
    runSimulation(1.0, 10000, "Test E: ERP=1.0, 10000 frames");
    runSimulation(2.0, 10000, "Test F: ERP=2.0, 10000 frames");

    return 0;
}
