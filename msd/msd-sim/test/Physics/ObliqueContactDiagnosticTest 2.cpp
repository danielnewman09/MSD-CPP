// DEBUG: Diagnostic tests for oblique contact instability
// Ticket: DEBUG_0035d_oblique_contact_instability

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

namespace
{

std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate{-half, -half, -half},
          Coordinate{half, -half, -half},
          Coordinate{half, half, -half},
          Coordinate{-half, half, -half},
          Coordinate{-half, -half, half},
          Coordinate{half, -half, half},
          Coordinate{half, half, half},
          Coordinate{-half, half, half}};
}

struct StepData
{
  int step;
  double pos_x, pos_y, pos_z;
  double vel_x, vel_y, vel_z;
  double omega_x, omega_y, omega_z;
  double ke_translational;
  double ke_rotational;
  double ke_total;
};

struct SimConfig
{
  double tilt_angle = 0.0;     // rotation about x-axis
  double friction = 0.6;
  double restitution = 0.5;
  double init_vx = 0.5;
  double init_vy = 0.0;
  double init_vz = -10.0;
  int steps = 100;
};

std::vector<StepData> runSimulation(const SimConfig& cfg)
{
  WorldModel world;

  auto floorPoints = createCubePoints(10.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -5.0}};
  world.spawnEnvironmentObject(0, floorHull, floorFrame);

  auto boxPoints = createCubePoints(1.0);
  ConvexHull boxHull{boxPoints};
  ReferenceFrame boxFrame{Coordinate{0.0, 0.0, 0.49}};
  if (cfg.tilt_angle != 0.0)
  {
    boxFrame.setRotation(AngularCoordinate{cfg.tilt_angle, 0., 0.});
  }
  world.spawnObject(1, boxHull, boxFrame);

  uint32_t boxId = 1;
  AssetInertial& box = world.getObject(boxId);

  box.setFrictionCoefficient(cfg.friction);
  box.setCoefficientOfRestitution(cfg.restitution);
  box.getInertialState().velocity = Coordinate{cfg.init_vx, cfg.init_vy, cfg.init_vz};

  const double mass = box.getMass();
  const Eigen::Matrix3d& inertia = box.getInertiaTensor();

  auto computeKE = [&]() -> std::pair<double, double>
  {
    const auto& vel = box.getInertialState().velocity;
    const AngularRate omega = box.getInertialState().getAngularVelocity();
    const Eigen::Vector3d omegaVec = omega;
    double translational = 0.5 * mass * vel.squaredNorm();
    double rotational = 0.5 * omegaVec.transpose() * inertia * omegaVec;
    return {translational, rotational};
  };

  std::vector<StepData> data;
  data.reserve(static_cast<size_t>(cfg.steps + 1));

  // Record initial state
  {
    auto [ke_t, ke_r] = computeKE();
    const auto& vel = box.getInertialState().velocity;
    const auto& pos = box.getInertialState().position;
    const AngularRate omega = box.getInertialState().getAngularVelocity();
    data.push_back({-1, pos.x(), pos.y(), pos.z(),
                    vel.x(), vel.y(), vel.z(),
                    omega.x(), omega.y(), omega.z(),
                    ke_t, ke_r, ke_t + ke_r});
  }

  for (int i = 0; i < cfg.steps; ++i)
  {
    world.update(std::chrono::milliseconds{16 * (i + 1)});

    auto [ke_t, ke_r] = computeKE();
    const auto& vel = box.getInertialState().velocity;
    const auto& pos = box.getInertialState().position;
    const AngularRate omega = box.getInertialState().getAngularVelocity();
    data.push_back({i, pos.x(), pos.y(), pos.z(),
                    vel.x(), vel.y(), vel.z(),
                    omega.x(), omega.y(), omega.z(),
                    ke_t, ke_r, ke_t + ke_r});
  }

  return data;
}

void printStepData(const std::vector<StepData>& data, int fromStep, int toStep)
{
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Step  |   pos_x   pos_y   pos_z  |   vel_x   vel_y   vel_z  | omega_x omega_y omega_z |   KE_T      KE_R      KE_tot   | dKE\n";
  std::cout << std::string(140, '-') << "\n";

  for (size_t i = 0; i < data.size(); ++i)
  {
    const auto& d = data[i];
    if (d.step < fromStep || d.step > toStep)
      continue;

    double dke = 0.0;
    if (i > 0)
      dke = d.ke_total - data[i - 1].ke_total;

    std::cout << std::setw(4) << d.step << "  | "
              << std::setw(7) << d.pos_x << " "
              << std::setw(7) << d.pos_y << " "
              << std::setw(7) << d.pos_z << " | "
              << std::setw(7) << d.vel_x << " "
              << std::setw(7) << d.vel_y << " "
              << std::setw(7) << d.vel_z << " | "
              << std::setw(7) << d.omega_x << " "
              << std::setw(7) << d.omega_y << " "
              << std::setw(7) << d.omega_z << " | "
              << std::setw(9) << d.ke_translational << " "
              << std::setw(9) << d.ke_rotational << " "
              << std::setw(9) << d.ke_total << " | "
              << std::setw(9) << dke << "\n";
  }
}

}  // anonymous namespace

// ============================================================================
// Diagnostic Test 1: Flat box (no tilt) with same initial conditions
// Hypothesis: If flat box also shows energy injection, the issue is NOT
// specific to tilted geometry.
// ============================================================================
TEST(ObliqueContactDiag, D1_FlatBox_WithFriction)
{
  std::cout << "\n=== D1: Flat box (no tilt) with friction ===\n";
  SimConfig cfg;
  cfg.tilt_angle = 0.0;
  cfg.friction = 0.6;
  cfg.restitution = 0.5;
  cfg.steps = 100;

  auto data = runSimulation(cfg);
  printStepData(data, -1, 99);

  // Count KE increases
  int keIncreases = 0;
  int dirReversals = 0;
  for (size_t i = 1; i < data.size(); ++i)
  {
    if (data[i].ke_total > data[i - 1].ke_total * 1.02)
      ++keIncreases;
    if (data[i].vel_x < -0.01 && data[0].vel_x > 0)
      ++dirReversals;
  }

  std::cout << "\nKE increases (>2%): " << keIncreases
            << "\nDirection reversals: " << dirReversals << "\n";
}

// ============================================================================
// Diagnostic Test 2: Tilted box WITHOUT friction (mu=0)
// Hypothesis H1/H3: If tilted no-friction still reverses direction,
// the normal contact computation is wrong for tilted geometry.
// ============================================================================
TEST(ObliqueContactDiag, D2_TiltedBox_NoFriction)
{
  std::cout << "\n=== D2: Tilted box (0.1 rad) WITHOUT friction ===\n";
  SimConfig cfg;
  cfg.tilt_angle = 0.1;
  cfg.friction = 0.0;
  cfg.restitution = 0.5;
  cfg.steps = 100;

  auto data = runSimulation(cfg);
  printStepData(data, -1, 99);

  int keIncreases = 0;
  int dirReversals = 0;
  for (size_t i = 1; i < data.size(); ++i)
  {
    if (data[i].ke_total > data[i - 1].ke_total * 1.02)
      ++keIncreases;
    if (data[i].vel_x < -0.01 && data[0].vel_x > 0)
      ++dirReversals;
  }

  std::cout << "\nKE increases (>2%): " << keIncreases
            << "\nDirection reversals: " << dirReversals << "\n";

  // If direction reversal happens even without friction, the normal contact
  // is broken for tilted geometry
  if (dirReversals > 0)
  {
    std::cout << ">>> FINDING: Normal contact reverses direction for tilted "
                 "box even without friction!\n";
  }
}

// ============================================================================
// Diagnostic Test 3: Tilted box, friction ON, restitution OFF (e=0)
// Hypothesis H4: If near-rest instability disappears with e=0,
// restitution is amplifying small velocities.
// ============================================================================
TEST(ObliqueContactDiag, D3_TiltedBox_WithFriction_NoRestitution)
{
  std::cout << "\n=== D3: Tilted box with friction, NO restitution (e=0) ===\n";
  SimConfig cfg;
  cfg.tilt_angle = 0.1;
  cfg.friction = 0.6;
  cfg.restitution = 0.0;
  cfg.steps = 100;

  auto data = runSimulation(cfg);
  printStepData(data, -1, 99);

  int keIncreases = 0;
  for (size_t i = 1; i < data.size(); ++i)
  {
    if (data[i].ke_total > data[i - 1].ke_total * 1.02)
      ++keIncreases;
  }

  std::cout << "\nKE increases (>2%): " << keIncreases << "\n";

  if (keIncreases == 0)
  {
    std::cout << ">>> FINDING: Restitution is the sole cause of instability. "
                 "With e=0, energy is stable.\n";
  }
}

// ============================================================================
// Diagnostic Test 4: Tilted box, friction ON, restitution ON — ORIGINAL
// (baseline for comparison)
// ============================================================================
TEST(ObliqueContactDiag, D4_TiltedBox_Original)
{
  std::cout << "\n=== D4: Tilted box ORIGINAL CONFIG (for comparison) ===\n";
  SimConfig cfg;
  cfg.tilt_angle = 0.1;
  cfg.friction = 0.6;
  cfg.restitution = 0.5;
  cfg.steps = 100;

  auto data = runSimulation(cfg);
  printStepData(data, -1, 99);

  int keIncreases = 0;
  int dirReversals = 0;
  for (size_t i = 1; i < data.size(); ++i)
  {
    if (data[i].ke_total > data[i - 1].ke_total * 1.02)
      ++keIncreases;
    if (data[i].vel_x < -0.01 && data[0].vel_x > 0)
      ++dirReversals;
  }

  std::cout << "\nKE increases (>2%): " << keIncreases
            << "\nDirection reversals: " << dirReversals << "\n";
}

// ============================================================================
// Diagnostic Test 5: Tilted box, NO friction, NO restitution
// Isolates the normal contact + Baumgarte behavior for tilted geometry.
// ============================================================================
TEST(ObliqueContactDiag, D5_TiltedBox_NoFriction_NoRestitution)
{
  std::cout << "\n=== D5: Tilted box, NO friction, NO restitution ===\n";
  SimConfig cfg;
  cfg.tilt_angle = 0.1;
  cfg.friction = 0.0;
  cfg.restitution = 0.0;
  cfg.steps = 100;

  auto data = runSimulation(cfg);
  printStepData(data, -1, 99);

  int keIncreases = 0;
  for (size_t i = 1; i < data.size(); ++i)
  {
    if (data[i].ke_total > data[i - 1].ke_total * 1.02)
      ++keIncreases;
  }

  std::cout << "\nKE increases (>2%): " << keIncreases << "\n";
}

// ============================================================================
// Diagnostic Test 6: Tilted box with different tilt angles
// Tests if the issue scales with tilt angle.
// ============================================================================
TEST(ObliqueContactDiag, D6_TiltSweep)
{
  std::cout << "\n=== D6: Tilt angle sweep ===\n";

  double angles[] = {0.0, 0.01, 0.05, 0.1, 0.2, 0.5};
  for (double angle : angles)
  {
    SimConfig cfg;
    cfg.tilt_angle = angle;
    cfg.friction = 0.6;
    cfg.restitution = 0.5;
    cfg.steps = 100;

    auto data = runSimulation(cfg);

    int keIncreases = 0;
    int dirReversals = 0;
    double maxKE = 0;
    for (size_t i = 1; i < data.size(); ++i)
    {
      if (data[i].ke_total > data[i - 1].ke_total * 1.02)
        ++keIncreases;
      if (data[i].vel_x < -0.01 && data[0].vel_x > 0)
        ++dirReversals;
      maxKE = std::max(maxKE, data[i].ke_total);
    }

    double finalKE = data.back().ke_total;
    std::cout << "tilt=" << std::setw(5) << angle << " rad | "
              << "KE_increases=" << std::setw(3) << keIncreases << " | "
              << "dir_reversals=" << std::setw(3) << dirReversals << " | "
              << "max_KE=" << std::setw(10) << maxKE << " | "
              << "final_KE=" << std::setw(10) << finalKE << "\n";
  }
}
