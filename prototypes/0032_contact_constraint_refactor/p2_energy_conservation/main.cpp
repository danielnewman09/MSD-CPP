// Prototype P2: Energy Conservation Comparison
// Question: Does the constraint-based approach conserve energy better than
// impulse-based? Success criteria: See README.md

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

struct Frame
{
  double time;
  double z;
  double vz;
  double ke;
  double pe;
  double total_energy;
  double penetration;
};

struct SimulationResult
{
  std::string name;
  std::vector<Frame> frames;
  double max_penetration{0.0};
  double final_z{0.0};
  double final_vz{0.0};
  bool energy_monotonic{true};
};

class BouncingBallSimulation
{
public:
  BouncingBallSimulation()
    : mass_{1.0},
      radius_{1.0},
      gravity_{9.81},
      dt_{0.016},
      restitution_{0.8},
      velocity_threshold_{0.5}
  {
  }

  SimulationResult runImpulseBased()
  {
    SimulationResult result;
    result.name = "Impulse-Based";

    // Initial conditions
    msd_sim::Vector3D position{0.0, 0.0, 5.0};
    msd_sim::Vector3D velocity{0.0, 0.0, 0.0};

    const msd_sim::Vector3D normal{0.0, 0.0, 1.0};
    const double slop = 0.01;
    const double correction_factor = 0.8;

    for (int frame = 0; frame < 1000; ++frame)
    {
      double time = frame * dt_;

      // Record pre-integration state for penetration check
      double gap_before = position.z() - radius_;

      // Apply gravity
      velocity.z() -= gravity_ * dt_;

      // Integrate position
      position += velocity * dt_;

      // Check for contact AFTER integration
      double gap = position.z() - radius_;
      bool penetrating = gap < 0.0;

      if (penetrating)
      {
        double v_rel_n = velocity.dot(normal);

        // Apply impulse if separating velocity is significant
        if (v_rel_n < -velocity_threshold_)
        {
          // Compute impulse
          double effective_mass =
            1.0 / mass_;  // Ground infinite mass, center contact
          double lambda = -(1.0 + restitution_) * v_rel_n / effective_mass;

          // Apply impulse
          velocity += (lambda * normal) / mass_;
        }
        else if (v_rel_n < 0.0)
        {
          // Apply non-restitutive impulse for low-velocity contact
          double effective_mass = 1.0 / mass_;
          double lambda = -v_rel_n / effective_mass;
          velocity += (lambda * normal) / mass_;
        }

        // Position correction (Baumgarte-like stabilization)
        double correction = std::max(0.0, -gap - slop) * correction_factor;
        position += correction * normal;
      }

      // Record frame
      Frame f;
      f.time = time;
      f.z = position.z();
      f.vz = velocity.z();
      f.ke = 0.5 * mass_ * velocity.squaredNorm();
      f.pe = mass_ * gravity_ * position.z();
      f.total_energy = f.ke + f.pe;
      f.penetration = std::max(0.0, -gap);

      result.frames.push_back(f);
      result.max_penetration = std::max(result.max_penetration, f.penetration);
    }

    result.final_z = result.frames.back().z;
    result.final_vz = result.frames.back().vz;

    // Check energy monotonicity (should decrease due to dissipation)
    for (size_t i = 1; i < result.frames.size(); ++i)
    {
      // Allow small numerical increases (< 0.1 J)
      if (result.frames[i].total_energy >
          result.frames[i - 1].total_energy + 0.1)
      {
        result.energy_monotonic = false;
        break;
      }
    }

    return result;
  }

  SimulationResult runConstraintBased()
  {
    SimulationResult result;
    result.name = "Constraint-Based";

    // Initial conditions
    msd_sim::Vector3D position{0.0, 0.0, 5.0};
    msd_sim::Vector3D velocity{0.0, 0.0, 0.0};

    const msd_sim::Vector3D normal{0.0, 0.0, 1.0};
    const double slop = 0.01;

    for (int frame = 0; frame < 1000; ++frame)
    {
      double time = frame * dt_;

      // Record pre-integration gap for penetration tracking
      double gap_before = position.z() - radius_;

      // Apply gravity
      velocity.z() -= gravity_ * dt_;

      // Integrate position
      position += velocity * dt_;

      // Check for contact constraint AFTER integration
      double gap = position.z() - radius_;  // C(q)
      double gap_dot = velocity.z();        // Cdot(q, v)

      bool penetrating = gap < 0.0;

      if (penetrating)
      {
        // Effective mass for contact (ground infinite mass, center contact)
        double effective_mass = 1.0 / mass_;

        // Apply velocity-level contact constraint
        // If separating significantly: apply restitution
        // If barely moving: just stop penetration
        double desired_gap_dot = 0.0;
        if (gap_dot < -velocity_threshold_)
        {
          // Significant impact: apply restitution
          desired_gap_dot = -(1.0 + restitution_) * gap_dot;
        }
        else if (gap_dot < 0.0)
        {
          // Small penetration velocity: just stop it
          desired_gap_dot = 0.0;
        }

        // Compute required velocity change
        double delta_gap_dot = desired_gap_dot - gap_dot;

        // Compute constraint impulse
        double lambda = delta_gap_dot * mass_ * effective_mass;
        velocity += (lambda / mass_) * normal;

        // Position correction with slop (like impulse-based approach)
        double correction = std::max(0.0, -gap - slop) * 0.8;
        position += correction * normal;
      }

      // Update gap after corrections
      gap = position.z() - radius_;

      // Record frame
      Frame f;
      f.time = time;
      f.z = position.z();
      f.vz = velocity.z();
      f.ke = 0.5 * mass_ * velocity.squaredNorm();
      f.pe = mass_ * gravity_ * position.z();
      f.total_energy = f.ke + f.pe;
      f.penetration = std::max(0.0, -gap);

      result.frames.push_back(f);
      result.max_penetration = std::max(result.max_penetration, f.penetration);
    }

    result.final_z = result.frames.back().z;
    result.final_vz = result.frames.back().vz;

    // Check energy monotonicity (should decrease due to dissipation)
    for (size_t i = 1; i < result.frames.size(); ++i)
    {
      // Allow small numerical increases (< 0.1 J)
      if (result.frames[i].total_energy >
          result.frames[i - 1].total_energy + 0.1)
      {
        result.energy_monotonic = false;
        break;
      }
    }

    return result;
  }

private:
  double mass_;
  double radius_;
  double gravity_;
  double dt_;
  double restitution_;
  double velocity_threshold_;
};

void printResults(const SimulationResult& result)
{
  std::cout << "\n=== " << result.name << " Results ===" << std::endl;
  std::cout << std::fixed << std::setprecision(6);

  // Print first 10 bounces
  std::cout << "\nFirst 100 frames:" << std::endl;
  std::cout << std::setw(8) << "Frame" << std::setw(12) << "Time(s)"
            << std::setw(12) << "Z(m)" << std::setw(12) << "Vz(m/s)"
            << std::setw(12) << "KE(J)" << std::setw(12) << "PE(J)"
            << std::setw(12) << "Total(J)" << std::setw(12) << "Pen(m)"
            << std::endl;

  for (size_t i = 0; i < std::min(size_t{100}, result.frames.size()); i += 10)
  {
    const auto& f = result.frames[i];
    std::cout << std::setw(8) << i << std::setw(12) << f.time << std::setw(12)
              << f.z << std::setw(12) << f.vz << std::setw(12) << f.ke
              << std::setw(12) << f.pe << std::setw(12) << f.total_energy
              << std::setw(12) << f.penetration << std::endl;
  }

  // Print last 10 frames
  std::cout << "\nLast 10 frames:" << std::endl;
  std::cout << std::setw(8) << "Frame" << std::setw(12) << "Time(s)"
            << std::setw(12) << "Z(m)" << std::setw(12) << "Vz(m/s)"
            << std::setw(12) << "KE(J)" << std::setw(12) << "PE(J)"
            << std::setw(12) << "Total(J)" << std::setw(12) << "Pen(m)"
            << std::endl;

  for (size_t i = result.frames.size() - 10; i < result.frames.size(); ++i)
  {
    const auto& f = result.frames[i];
    std::cout << std::setw(8) << i << std::setw(12) << f.time << std::setw(12)
              << f.z << std::setw(12) << f.vz << std::setw(12) << f.ke
              << std::setw(12) << f.pe << std::setw(12) << f.total_energy
              << std::setw(12) << f.penetration << std::endl;
  }

  std::cout << "\nSummary:" << std::endl;
  std::cout << "  Max penetration: " << result.max_penetration << " m"
            << std::endl;
  std::cout << "  Final Z: " << result.final_z << " m" << std::endl;
  std::cout << "  Final Vz: " << result.final_vz << " m/s" << std::endl;
  std::cout << "  Initial energy: " << result.frames.front().total_energy
            << " J" << std::endl;
  std::cout << "  Final energy: " << result.frames.back().total_energy << " J"
            << std::endl;
  std::cout << "  Energy dissipated: "
            << (result.frames.front().total_energy -
                result.frames.back().total_energy)
            << " J" << std::endl;
  std::cout << "  Energy monotonic: "
            << (result.energy_monotonic ? "YES" : "NO") << std::endl;
}

bool checkCriteria(const SimulationResult& impulse,
                   const SimulationResult& constraint)
{
  std::cout << "\n=== SUCCESS CRITERIA ===" << std::endl;

  bool all_pass = true;

  // Criterion 1: Both approaches produce decreasing total energy
  std::cout << "\n1. Both approaches produce decreasing total energy over time:"
            << std::endl;
  double impulse_energy_change =
    impulse.frames.back().total_energy - impulse.frames.front().total_energy;
  double constraint_energy_change = constraint.frames.back().total_energy -
                                    constraint.frames.front().total_energy;
  std::cout << "   Impulse-based energy change: " << impulse_energy_change
            << " J" << std::endl;
  std::cout << "   Constraint-based energy change: " << constraint_energy_change
            << " J" << std::endl;
  bool criterion1 =
    (impulse_energy_change < 0.0) && (constraint_energy_change < 0.0);
  std::cout << "   Result: " << (criterion1 ? "PASS" : "FAIL") << std::endl;
  all_pass &= criterion1;

  // Criterion 2: Constraint approach does not inject energy
  std::cout << "\n2. Constraint approach does NOT inject energy (monotonic):"
            << std::endl;
  std::cout << "   Energy monotonic: "
            << (constraint.energy_monotonic ? "YES" : "NO") << std::endl;
  bool criterion2 = constraint.energy_monotonic;
  std::cout << "   Result: " << (criterion2 ? "PASS" : "FAIL") << std::endl;
  all_pass &= criterion2;

  // Criterion 3: Maximum penetration for constraint approach < 0.02m
  std::cout << "\n3. Maximum penetration depth for constraint approach < 0.02m:"
            << std::endl;
  std::cout << "   Max penetration: " << constraint.max_penetration << " m"
            << std::endl;
  bool criterion3 = constraint.max_penetration < 0.02;
  std::cout << "   Result: " << (criterion3 ? "PASS" : "FAIL") << std::endl;
  all_pass &= criterion3;

  // Criterion 4: Both approaches eventually reach rest
  std::cout << "\n4. Both approaches eventually reach rest (|v| < 0.01 m/s) "
               "within 1000 frames:"
            << std::endl;
  std::cout << "   Impulse-based final |v|: " << std::abs(impulse.final_vz)
            << " m/s" << std::endl;
  std::cout << "   Constraint-based final |v|: "
            << std::abs(constraint.final_vz) << " m/s" << std::endl;
  bool criterion4 = (std::abs(impulse.final_vz) < 0.01) &&
                    (std::abs(constraint.final_vz) < 0.01);
  std::cout << "   Result: " << (criterion4 ? "PASS" : "FAIL") << std::endl;
  all_pass &= criterion4;

  // Criterion 5: Final rest positions match within 0.05m
  std::cout << "\n5. Final rest positions match within 0.05m:" << std::endl;
  double position_difference = std::abs(impulse.final_z - constraint.final_z);
  std::cout << "   Impulse-based final Z: " << impulse.final_z << " m"
            << std::endl;
  std::cout << "   Constraint-based final Z: " << constraint.final_z << " m"
            << std::endl;
  std::cout << "   Difference: " << position_difference << " m" << std::endl;
  bool criterion5 = position_difference < 0.05;
  std::cout << "   Result: " << (criterion5 ? "PASS" : "FAIL") << std::endl;
  all_pass &= criterion5;

  std::cout << "\n=== OVERALL: " << (all_pass ? "ALL PASS" : "SOME FAILURES")
            << " ===" << std::endl;

  return all_pass;
}

int main()
{
  std::cout << "=== Energy Conservation Comparison Prototype ===" << std::endl;
  std::cout << "Comparing impulse-based vs constraint-based approaches"
            << std::endl;
  std::cout << "\nScenario:" << std::endl;
  std::cout << "  Sphere: m=1kg, r=1m" << std::endl;
  std::cout << "  Initial position: (0,0,5) m" << std::endl;
  std::cout << "  Initial velocity: (0,0,0) m/s" << std::endl;
  std::cout << "  Gravity: g=-9.81 m/s^2" << std::endl;
  std::cout << "  Restitution: e=0.8" << std::endl;
  std::cout << "  Time step: dt=0.016s (60 FPS)" << std::endl;
  std::cout << "  Duration: 1000 frames (~16 seconds)" << std::endl;

  BouncingBallSimulation sim;

  std::cout << "\nRunning impulse-based simulation..." << std::endl;
  auto impulse_result = sim.runImpulseBased();
  printResults(impulse_result);

  std::cout << "\nRunning constraint-based simulation..." << std::endl;
  auto constraint_result = sim.runConstraintBased();
  printResults(constraint_result);

  bool success = checkCriteria(impulse_result, constraint_result);

  return success ? 0 : 1;
}
