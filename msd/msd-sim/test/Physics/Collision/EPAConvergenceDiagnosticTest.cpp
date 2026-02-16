// Ticket: 0048_epa_convergence_robustness
// Ticket: 0062e_replay_diagnostic_parameter_tests
// Diagnostic test to identify EPA convergence failure mode for shallow
// penetrations. Tests both the H6 scenario directly and sweeps penetration
// depths to find the threshold where EPA degrades.

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

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

}  // namespace

// ============================================================================
// Phase 1: Reproduce the H6 EPA convergence failure directly
// ============================================================================

TEST(EPAConvergenceDiagnostic, H6_TwoCubes_ShallowOverlap)
{
  // Exact H6 scenario: two 1m cubes with 0.01m overlap along X-axis
  auto cubePointsA = createCubePoints(1.0);
  auto cubePointsB = createCubePoints(1.0);
  ConvexHull hullA{cubePointsA};
  ConvexHull hullB{cubePointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.99, 0.0, 0.0}};

  AssetInertial assetA{1, 100, hullA, 10.0, frameA, 0.0};
  AssetInertial assetB{2, 101, hullB, 10.0, frameB, 0.0};

  CollisionHandler handler{1e-6};

  std::cout << "\n=== H6 REPRODUCTION: Two cubes, 0.01m overlap ===\n";

  try
  {
    auto result = handler.checkCollision(assetA, assetB);
    if (result.has_value())
    {
      std::cout << "Collision detected (no exception)\n";
      std::cout << "  Contact count: " << result->contactCount << "\n";
      std::cout << "  Normal: (" << result->normal.x() << ", "
                << result->normal.y() << ", " << result->normal.z() << ")\n";
      std::cout << "  Penetration: " << result->penetrationDepth << "\n";
      for (size_t i = 0; i < result->contactCount; ++i)
      {
        const auto& cp = result->contacts[i];
        std::cout << "  Contact " << i << ": depth=" << cp.depth << "\n";
      }
    }
    else
    {
      std::cout << "No collision detected (unexpected for overlapping cubes)\n";
    }
    SUCCEED() << "EPA converged without exception";
  }
  catch (const std::runtime_error& e)
  {
    std::cout << "EXCEPTION: " << e.what() << "\n";
    FAIL() << "EPA threw exception: " << e.what();
  }
}

// ============================================================================
// Phase 2: Sweep penetration depths to find degradation threshold
// ============================================================================

TEST(EPAConvergenceDiagnostic, PenetrationDepthSweep_CubeOnFloor)
{
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};

  CollisionHandler handler{1e-6};

  std::cout << "\n=== PENETRATION DEPTH SWEEP: Cube on floor ===\n";
  std::cout << "Depth(m)       | Contacts | Normal           | "
            << "EPA depth      | Exception\n";
  std::cout << "---------------|----------|------------------|"
            << "----------------|----------\n";

  std::vector<double> depths = {
    0.1, 0.05, 0.01, 0.005, 0.001, 0.0005, 0.0001, 5e-5, 1e-5, 5e-6, 1e-6,
    5e-7, 1e-7};

  for (double depth : depths)
  {
    double const cubeZ = 0.5 - depth;
    ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, cubeZ}};
    AssetInertial cube{1, 100, cubeHull, 1.0, cubeFrame, 0.5};
    AssetEnvironment floor{1, 200, floorHull, floorFrame, 0.5};

    try
    {
      auto result = handler.checkCollision(cube, floor);
      if (result.has_value())
      {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "%-14.1e | %-8zu | (%5.2f,%5.2f,%5.2f) | %-14.6e | No\n",
                      depth, result->contactCount, result->normal.x(),
                      result->normal.y(), result->normal.z(),
                      result->penetrationDepth);
        std::cout << buf;
      }
      else
      {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "%-14.1e | NO COLLISION DETECTED                     | No\n",
                      depth);
        std::cout << buf;
      }
    }
    catch (const std::runtime_error& e)
    {
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "%-14.1e | EXCEPTION: %-40s\n", depth, e.what());
      std::cout << buf;
    }
  }
  std::cout << "\n";
}

TEST(EPAConvergenceDiagnostic, PenetrationDepthSweep_TwoCubes)
{
  auto cubePointsA = createCubePoints(1.0);
  auto cubePointsB = createCubePoints(1.0);
  ConvexHull hullA{cubePointsA};
  ConvexHull hullB{cubePointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};

  CollisionHandler handler{1e-6};

  std::cout << "\n=== PENETRATION DEPTH SWEEP: Two equal cubes along X ===\n";
  std::cout << "Depth(m)       | Contacts | Normal           | "
            << "EPA depth      | Exception\n";
  std::cout << "---------------|----------|------------------|"
            << "----------------|----------\n";

  std::vector<double> depths = {
    0.1, 0.05, 0.01, 0.005, 0.001, 0.0005, 0.0001, 5e-5, 1e-5, 5e-6, 1e-6,
    5e-7, 1e-7};

  for (double depth : depths)
  {
    double const bX = 1.0 - depth;
    ReferenceFrame frameB{Coordinate{bX, 0.0, 0.0}};

    AssetInertial assetA{1, 100, hullA, 10.0, frameA, 0.0};
    AssetInertial assetB{2, 101, hullB, 10.0, frameB, 0.0};

    try
    {
      auto result = handler.checkCollision(assetA, assetB);
      if (result.has_value())
      {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "%-14.1e | %-8zu | (%5.2f,%5.2f,%5.2f) | %-14.6e | No\n",
                      depth, result->contactCount, result->normal.x(),
                      result->normal.y(), result->normal.z(),
                      result->penetrationDepth);
        std::cout << buf;
      }
      else
      {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "%-14.1e | NO COLLISION DETECTED                     | No\n",
                      depth);
        std::cout << buf;
      }
    }
    catch (const std::runtime_error& e)
    {
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "%-14.1e | EXCEPTION: %-40s\n", depth, e.what());
      std::cout << buf;
    }
  }
  std::cout << "\n";
}

// ============================================================================
// Phase 3: Simulate H6 frame-by-frame to find the exact failure frame
// ============================================================================

TEST_F(ReplayEnabledTest,
       EPAConvergenceDiagnostic_H6_SimulationFrameByFrame)
{
  disableGravity();

  auto cubePointsA = createCubePoints(1.0);
  auto cubePointsB = createCubePoints(1.0);
  ConvexHull hullA{cubePointsA};
  ConvexHull hullB{cubePointsB};

  // Use Engine's registry to spawn cubes (for replay compatibility)
  // Spawn cubes at H6 positions
  const auto& cubeA =
    spawnInertialWithVelocity("unit_cube", Coordinate{0.0, 0.0, 0.0},
                              Coordinate{0.0, 0.0, 0.0},  // stationary
                              10.0,                        // mass
                              0.0,                         // restitution
                              0.0);                        // friction
  uint32_t idA = cubeA.getInstanceId();

  const auto& cubeB =
    spawnInertialWithVelocity("unit_cube", Coordinate{0.99, 0.0, 0.0},
                              Coordinate{0.0, 0.0, 0.0},  // stationary
                              10.0,                        // mass
                              0.0,                         // restitution
                              0.0);                        // friction
  uint32_t idB = cubeB.getInstanceId();

  std::cout << "\n=== H6 SIMULATION FRAME-BY-FRAME ===\n";
  std::cout << "Frame | posA.x     | posB.x     | gap(m)       | velA.x     "
            << "| velB.x     | Status\n";
  std::cout << "------|------------|------------|--------------|------------"
            << "|------------|-------\n";

  CollisionHandler handler{1e-6};

  int failFrame = -1;
  std::string failReason;

  for (int i = 1; i <= 20; ++i)
  {
    auto const& stateA = world().getObject(idA).getInertialState();
    auto const& stateB = world().getObject(idB).getInertialState();
    double posAx = stateA.position.x();
    double posBx = stateB.position.x();
    double gap = (posBx - 0.5) - (posAx + 0.5);
    double velAx = stateA.velocity.x();
    double velBx = stateB.velocity.x();

    // Check ReferenceFrame vs InertialState consistency
    auto const& frameRefA = world().getObject(idA).getReferenceFrame();
    auto const& frameRefB = world().getObject(idB).getReferenceFrame();
    double originAx = frameRefA.getOrigin().x();
    double originBx = frameRefB.getOrigin().x();
    if (std::abs(originAx - posAx) > 1e-12 ||
        std::abs(originBx - posBx) > 1e-12)
    {
      std::cout << "  !!! MISMATCH: state.pos.x=" << posAx
                << " frame.origin.x=" << originAx << " (A)\n";
      std::cout << "  !!! MISMATCH: state.pos.x=" << posBx
                << " frame.origin.x=" << originBx << " (B)\n";
    }

    std::string status = "OK";
    try
    {
      auto const& objA = world().getObject(idA);
      auto const& objB = world().getObject(idB);
      auto probeResult = handler.checkCollision(objA, objB);
      if (probeResult.has_value())
      {
        char depthBuf[64];
        std::snprintf(depthBuf, sizeof(depthBuf), "coll(d=%.2e,n=%d)",
                      probeResult->penetrationDepth,
                      static_cast<int>(probeResult->contactCount));
        status = depthBuf;
      }
      else
      {
        status = "no-coll";
      }
    }
    catch (const std::runtime_error& e)
    {
      status = std::string("PROBE-EXC: ") + e.what();

      // Fresh-object comparison test
      ReferenceFrame freshFrameA{Coordinate{posAx, stateA.position.y(),
                                             stateA.position.z()}};
      freshFrameA.setQuaternion(stateA.orientation);
      ReferenceFrame freshFrameB{Coordinate{posBx, stateB.position.y(),
                                             stateB.position.z()}};
      freshFrameB.setQuaternion(stateB.orientation);

      AssetInertial freshA{1, 100, hullA, 10.0, freshFrameA, 0.0};
      AssetInertial freshB{2, 101, hullB, 10.0, freshFrameB, 0.0};

      try
      {
        auto freshResult = handler.checkCollision(freshA, freshB);
        if (freshResult.has_value())
        {
          std::cout << "  >>> FRESH objects at same pos: OK! d="
                    << freshResult->penetrationDepth << " n="
                    << freshResult->contactCount << "\n";
        }
        else
        {
          std::cout << "  >>> FRESH objects at same pos: no collision\n";
        }
      }
      catch (const std::runtime_error& e2)
      {
        std::cout << "  >>> FRESH objects ALSO fail: " << e2.what() << "\n";
      }

      // Print quaternions
      std::cout << "  qA=(" << stateA.orientation.w() << ","
                << stateA.orientation.x() << "," << stateA.orientation.y()
                << "," << stateA.orientation.z() << ")\n";
      std::cout << "  qB=(" << stateB.orientation.w() << ","
                << stateB.orientation.x() << "," << stateB.orientation.y()
                << "," << stateB.orientation.z() << ")\n";

      // Print the ReferenceFrame rotation matrix
      auto const& rotA = frameRefA.getRotation();
      auto const& rotB = frameRefB.getRotation();
      bool identA = rotA.isIdentity(1e-15);
      bool identB = rotB.isIdentity(1e-15);
      std::cout << "  rotA identity(1e-15): " << identA
                << " rotB identity(1e-15): " << identB << "\n";
      if (!identA)
      {
        std::cout << "  rotA:\n" << rotA << "\n";
      }
      if (!identB)
      {
        std::cout << "  rotB:\n" << rotB << "\n";
      }
    }

    // Print quaternion orientation too
    auto const& qA = stateA.orientation;
    auto const& qB = stateB.orientation;

    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "%-5d | %10.6f | %10.6f | %12.4e | %10.6f | %10.6f | %s\n",
                  i, posAx, posBx, gap, velAx, velBx, status.c_str());
    std::cout << buf;
    if (qA.w() < 0.9999999 || qB.w() < 0.9999999)
    {
      std::cout << "      qA=(" << qA.w() << "," << qA.x() << "," << qA.y()
                << "," << qA.z() << ")"
                << " qB=(" << qB.w() << "," << qB.x() << "," << qB.y() << ","
                << qB.z() << ")\n";
    }

    try
    {
      step(1);
    }
    catch (const std::runtime_error& e)
    {
      failFrame = i;
      failReason = e.what();
      std::cout << ">>> EXCEPTION at frame " << i << ": " << e.what() << "\n";
      break;
    }
  }

  if (failFrame > 0)
  {
    std::cout << "\nEPA failed at frame " << failFrame << ": " << failReason
              << "\n";
    auto const& stateA = world().getObject(idA).getInertialState();
    auto const& stateB = world().getObject(idB).getInertialState();
    std::cout << "Final posA: (" << stateA.position.x() << ", "
              << stateA.position.y() << ", " << stateA.position.z() << ")\n";
    std::cout << "Final posB: (" << stateB.position.x() << ", "
              << stateB.position.y() << ", " << stateB.position.z() << ")\n";
    std::cout << "Final velA: (" << stateA.velocity.x() << ", "
              << stateA.velocity.y() << ", " << stateA.velocity.z() << ")\n";
    std::cout << "Final velB: (" << stateB.velocity.x() << ", "
              << stateB.velocity.y() << ", " << stateB.velocity.z() << ")\n";
    FAIL() << "EPA convergence failure at frame " << failFrame;
  }
  else
  {
    std::cout << "\nAll 20 frames completed without EPA exception.\n";
    SUCCEED();
  }
}

// ============================================================================
// Phase 4a: Rotation sensitivity sweep — find EPA failure threshold
// ============================================================================

TEST(EPAConvergenceDiagnostic, RotationSensitivitySweep)
{
  auto cubePointsA = createCubePoints(1.0);
  auto cubePointsB = createCubePoints(1.0);
  ConvexHull hullA{cubePointsA};
  ConvexHull hullB{cubePointsB};

  CollisionHandler handler{1e-6};

  std::cout << "\n=== ROTATION SENSITIVITY: Tiny X-axis rotation ===\n";
  std::cout << "Rotation(rad) | Gap(m)  | Status\n";
  std::cout << "--------------|---------|-------\n";

  // Sweep tiny rotations around X-axis (what PositionCorrector produces)
  std::vector<double> rotations = {
    0.0,  1e-10, 1e-9, 1e-8, 5e-8, 1e-7, 2e-7, 3e-7, 5e-7, 1e-6, 1e-5, 1e-4};

  for (double rot : rotations)
  {
    Eigen::Quaterniond qA{
      Eigen::AngleAxisd{rot, Eigen::Vector3d::UnitX()}};
    Eigen::Quaterniond qB{
      Eigen::AngleAxisd{-rot, Eigen::Vector3d::UnitX()}};

    // 0.01m penetration
    ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
    frameA.setQuaternion(qA);
    ReferenceFrame frameB{Coordinate{0.99, 0.0, 0.0}};
    frameB.setQuaternion(qB);

    AssetInertial assetA{1, 100, hullA, 10.0, frameA, 0.0};
    AssetInertial assetB{2, 101, hullB, 10.0, frameB, 0.0};

    try
    {
      auto result = handler.checkCollision(assetA, assetB);
      if (result.has_value())
      {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%-13.1e | -0.01   | OK (d=%.2e, n=%d)\n",
                      rot, result->penetrationDepth,
                      static_cast<int>(result->contactCount));
        std::cout << buf;
      }
      else
      {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%-13.1e | -0.01   | NO COLLISION\n", rot);
        std::cout << buf;
      }
    }
    catch (const std::runtime_error& e)
    {
      char buf[128];
      std::snprintf(buf, sizeof(buf), "%-13.1e | -0.01   | EXCEPTION: %s\n", rot,
                    e.what());
      std::cout << buf;
    }
  }
  std::cout << "\n";
}

// ============================================================================
// Phase 4b: Exact simulation quaternion reproduction
// ============================================================================

TEST(EPAConvergenceDiagnostic, ExactSimulationQuaternionReproduction)
{
  auto cubePointsA = createCubePoints(1.0);
  auto cubePointsB = createCubePoints(1.0);
  ConvexHull hullA{cubePointsA};
  ConvexHull hullB{cubePointsB};

  CollisionHandler handler{1e-6};

  std::cout << "\n=== EXACT QUATERNION REPRODUCTION ===\n";

  // Exact values from simulation frame 1 output
  Eigen::Quaterniond qA{1.0, -1.40625e-07, 5.27344e-11, -1.0842e-19};
  Eigen::Quaterniond qB{1.0, -1.40625e-07, -5.27344e-11, 1.0842e-19};
  qA.normalize();
  qB.normalize();

  struct TestCase
  {
    double posAx, posBx;
    Eigen::Quaterniond qA, qB;
    std::string label;
  };

  Eigen::Quaterniond identity{1.0, 0.0, 0.0, 0.0};

  std::vector<TestCase> cases = {
    {0.0, 0.99, identity, identity, "identity quats (baseline)"},
    {-0.0005, 0.9905, identity, identity, "shifted pos, identity quats"},
    {0.0, 0.99, qA, qB, "original pos, sim quats"},
    {-0.0005, 0.9905, qA, qB, "shifted pos + sim quats (exact sim state)"},
    {0.0, 0.99, qA, identity, "original pos, only A rotated"},
    {0.0, 0.99, identity, qB, "original pos, only B rotated"},
  };

  for (auto const& tc : cases)
  {
    ReferenceFrame frameA{Coordinate{tc.posAx, 0.0, 0.0}};
    frameA.setQuaternion(tc.qA);
    ReferenceFrame frameB{Coordinate{tc.posBx, 0.0, 0.0}};
    frameB.setQuaternion(tc.qB);

    AssetInertial assetA{1, 100, hullA, 10.0, frameA, 0.0};
    AssetInertial assetB{2, 101, hullB, 10.0, frameB, 0.0};

    try
    {
      auto result = handler.checkCollision(assetA, assetB);
      if (result.has_value())
      {
        std::cout << "  " << tc.label << ": OK d=" << result->penetrationDepth
                  << " n=" << result->contactCount
                  << " normal=(" << result->normal.x() << ","
                  << result->normal.y() << "," << result->normal.z() << ")\n";
      }
      else
      {
        std::cout << "  " << tc.label << ": NO COLLISION\n";
      }
    }
    catch (const std::runtime_error& e)
    {
      std::cout << "  " << tc.label << ": EXCEPTION: " << e.what() << "\n";
    }
  }
}

// ============================================================================
// Phase 4: Directly probe the exact post-frame-1 configuration
// ============================================================================

TEST(EPAConvergenceDiagnostic, H6_PostPositionCorrection_DirectProbe)
{
  // After frame 1, PositionCorrector shifts objects:
  // A: 0.0 -> -0.0005, B: 0.99 -> 0.9905
  // This is 0.009m penetration — should work but doesn't.
  // Test: sweep tiny position offsets around this configuration.
  auto cubePointsA = createCubePoints(1.0);
  auto cubePointsB = createCubePoints(1.0);
  ConvexHull hullA{cubePointsA};
  ConvexHull hullB{cubePointsB};

  CollisionHandler handler{1e-6};

  std::cout
    << "\n=== PHASE 4: Post-correction probe (A shifted, B shifted) ===\n";
  std::cout << "posA.x        | posB.x        | gap(m)       | Status\n";
  std::cout << "--------------|---------------|--------------|-------\n";

  // Test the exact post-correction positions and nearby
  struct TestCase
  {
    double posAx;
    double posBx;
    std::string label;
  };

  std::vector<TestCase> cases = {
    {0.0, 0.99, "initial (works)"},
    {-0.0005, 0.9905, "post-frame1 exact"},
    {-0.001, 0.991, "post-frame1 symmetric"},
    {0.0, 0.991, "A=origin, B=0.991 (d=0.009)"},
    {0.0, 0.995, "A=origin, B=0.995 (d=0.005)"},
    {0.0, 0.999, "A=origin, B=0.999 (d=0.001)"},
    {-0.0005, 0.99, "A shifted only"},
    {0.0, 0.9905, "B shifted only (d=0.0095)"},
  };

  for (auto const& tc : cases)
  {
    ReferenceFrame frameA{Coordinate{tc.posAx, 0.0, 0.0}};
    ReferenceFrame frameB{Coordinate{tc.posBx, 0.0, 0.0}};

    AssetInertial assetA{1, 100, hullA, 10.0, frameA, 0.0};
    AssetInertial assetB{2, 101, hullB, 10.0, frameB, 0.0};

    try
    {
      auto result = handler.checkCollision(assetA, assetB);
      if (result.has_value())
      {
        char buf[256];
        std::snprintf(
          buf, sizeof(buf),
          "%13.7f | %13.7f | %12.4e | OK (d=%.2e,n=%d) [%s]\n", tc.posAx,
          tc.posBx, (tc.posBx - 0.5) - (tc.posAx + 0.5),
          result->penetrationDepth,
          static_cast<int>(result->contactCount), tc.label.c_str());
        std::cout << buf;
      }
      else
      {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "%13.7f | %13.7f | %12.4e | NO COLLISION [%s]\n",
                      tc.posAx, tc.posBx,
                      (tc.posBx - 0.5) - (tc.posAx + 0.5), tc.label.c_str());
        std::cout << buf;
      }
    }
    catch (const std::runtime_error& e)
    {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "%13.7f | %13.7f | %12.4e | EXCEPTION: %s [%s]\n",
                    tc.posAx, tc.posBx,
                    (tc.posBx - 0.5) - (tc.posAx + 0.5), e.what(),
                    tc.label.c_str());
      std::cout << buf;
    }
  }
}
