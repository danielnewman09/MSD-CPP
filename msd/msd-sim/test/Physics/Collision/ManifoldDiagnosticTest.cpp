// Ticket: 0047_face_contact_manifold_generation
// Diagnostic test to identify why EPA produces single contact point
// for face-on-face contacts. This test is temporary and will be
// removed after the root cause is identified and fixed.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
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

}  // namespace

TEST(ManifoldDiagnostic, FaceFaceContactPointCount)
{
  // Reproduce exact D1 test setup:
  // 1m cube centered at z=0.5 (bottom face at z=0)
  // 100m floor cube centered at z=-50 (top face at z=0)
  // Both axis-aligned, touching at z=0

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.5}};

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};

  // Log hull facet information
  std::cout << "\n=== MANIFOLD DIAGNOSTIC ===\n";
  std::cout << "Cube hull: " << cubeHull.getVertexCount() << " vertices, "
            << cubeHull.getFacetCount() << " facets\n";
  std::cout << "Floor hull: " << floorHull.getVertexCount() << " vertices, "
            << floorHull.getFacetCount() << " facets\n";

  // Check aligned facets for cube bottom face (normal = (0,0,-1))
  Vector3D const cubeBottomNormal{0.0, 0.0, -1.0};
  auto cubeFacets = cubeHull.getFacetsAlignedWith(cubeBottomNormal);
  std::cout << "\nCube facets aligned with (0,0,-1): " << cubeFacets.size()
            << "\n";
  for (size_t i = 0; i < cubeFacets.size(); ++i)
  {
    const auto& f = cubeFacets[i].get();
    std::cout << "  Facet " << i << ": normal=(" << f.normal.x() << ", "
              << f.normal.y() << ", " << f.normal.z() << "), vertices=[";
    for (size_t j = 0; j < f.vertexIndices.size(); ++j)
    {
      if (j > 0)
      {
        std::cout << ", ";
      }
      std::cout << f.vertexIndices[j];
      const auto& v = cubeHull.getVertices()[f.vertexIndices[j]];
      std::cout << "=(" << v.x() << "," << v.y() << "," << v.z() << ")";
    }
    std::cout << "]\n";
  }

  // Check aligned facets for floor top face (normal = (0,0,1))
  Vector3D const floorTopNormal{0.0, 0.0, 1.0};
  auto floorFacets = floorHull.getFacetsAlignedWith(floorTopNormal);
  std::cout << "\nFloor facets aligned with (0,0,1): " << floorFacets.size()
            << "\n";
  for (size_t i = 0; i < floorFacets.size(); ++i)
  {
    const auto& f = floorFacets[i].get();
    std::cout << "  Facet " << i << ": normal=(" << f.normal.x() << ", "
              << f.normal.y() << ", " << f.normal.z() << "), vertices=[";
    for (size_t j = 0; j < f.vertexIndices.size(); ++j)
    {
      if (j > 0)
      {
        std::cout << ", ";
      }
      std::cout << f.vertexIndices[j];
    }
    std::cout << "]\n";
  }

  // Now create the collision scenario with slight penetration
  // (cube bottom at z=0 overlaps with floor top at z=0)
  // Move cube down slightly so there's actual penetration for EPA
  ReferenceFrame cubeFramePenetrating{Coordinate{0.0, 0.0, 0.49}};

  AssetInertial cube{1, 100, cubeHull, 1.0, cubeFramePenetrating, 0.5};
  AssetEnvironment floor{1, 200, floorHull, floorFrame, 0.5};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value()) << "Expected collision between cube and floor";

  std::cout << "\n=== COLLISION RESULT ===\n";
  std::cout << "Contact count: " << result->contactCount << "\n";
  std::cout << "Normal: (" << result->normal.x() << ", " << result->normal.y()
            << ", " << result->normal.z() << ")\n";
  std::cout << "Penetration depth: " << result->penetrationDepth << "\n";

  for (size_t i = 0; i < result->contactCount; ++i)
  {
    const auto& cp = result->contacts[i];
    std::cout << "Contact " << i << ":\n"
              << "  pointA=(" << cp.pointA.x() << ", " << cp.pointA.y()
              << ", " << cp.pointA.z() << ")\n"
              << "  pointB=(" << cp.pointB.x() << ", " << cp.pointB.y()
              << ", " << cp.pointB.z() << ")\n"
              << "  depth=" << cp.depth << "\n";
  }

  std::cout << "=== END DIAGNOSTIC ===\n\n";

  // The key assertion: we expect 4 contact points for face-face contact
  EXPECT_GE(result->contactCount, 4u)
    << "DIAGNOSTIC: Face-face contact should produce 4 contact points, got "
    << result->contactCount;
}

// Ticket: 0047_face_contact_manifold_generation
// Sweep penetration depth from 0.01m down to 1e-7m to find the threshold
// where manifold quality degrades.
TEST(ManifoldDiagnostic, PenetrationDepthSweep)
{
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};

  CollisionHandler handler{1e-6};

  // Sweep penetration depths: 0.01, 0.005, 0.001, 0.0005, 0.0001, 5e-5,
  // 1e-5, 5e-6, 1e-6, 5e-7, 1e-7
  std::vector<double> depths = {0.01,  0.005, 0.001, 5e-4, 1e-4,
                                5e-5,  1e-5,  5e-6,  1e-6, 5e-7, 1e-7};

  std::cout << "\n=== PENETRATION DEPTH SWEEP ===\n";
  std::cout << "Depth(m)      Contacts  Normal            PenDepth\n";
  std::cout << "------------- --------  ----------------  ----------\n";

  for (double depth : depths)
  {
    // Cube center at z = 0.5 - depth (so bottom face is at -depth)
    double cubeZ = 0.5 - depth;
    ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, cubeZ}};

    AssetInertial cube{1, 100, cubeHull, 1.0, cubeFrame, 0.5};
    AssetEnvironment floor{1, 200, floorHull, floorFrame, 0.5};

    auto result = handler.checkCollision(cube, floor);

    if (result.has_value())
    {
      std::cout << std::scientific << depth << "  " << result->contactCount
                << "         (" << std::fixed << result->normal.x() << ", "
                << result->normal.y() << ", " << result->normal.z() << ")  "
                << std::scientific << result->penetrationDepth << "\n";

      // Log individual contact details for single-point results
      if (result->contactCount == 1)
      {
        const auto& cp = result->contacts[0];
        std::cout << "  -> SINGLE POINT: A=(" << cp.pointA.x() << ", "
                  << cp.pointA.y() << ", " << cp.pointA.z() << ") B=("
                  << cp.pointB.x() << ", " << cp.pointB.y() << ", "
                  << cp.pointB.z() << ")\n";
      }
    }
    else
    {
      std::cout << std::scientific << depth << "  NO COLLISION\n";
    }
  }
  std::cout << "=== END SWEEP ===\n\n";
}

// Ticket: 0047_face_contact_manifold_generation
// Run a D1-like simulation for 100 frames and log per-frame manifold data.
// This captures the exact frame-by-frame contact behavior during resting
// contact.
TEST(ManifoldDiagnostic, PerFrameManifoldDuringRestingContact)
{
  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Cube: 1m, resting on floor
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.5}};
  world.spawnObject(1, cubeHull, 1.0, cubeFrame);

  uint32_t cubeId = 1;
  world.getObject(cubeId).setCoefficientOfRestitution(0.5);

  // Also create standalone collision handler to probe manifold each frame
  CollisionHandler handler{1e-6};

  std::cout << "\n=== PER-FRAME MANIFOLD DIAGNOSTIC (50 frames) ===\n";
  std::cout
    << "Frame  Pos(x,y,z)                          Vel-Z      Omega      "
       "Contacts  PenDepth     Normal(x,y,z)\n";
  std::cout
    << "-----  ------------------------------------  ---------  ---------  "
       "--------  -----------  -------------------\n";

  for (int i = 1; i <= 50; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    const auto& cubeAsset = world.getObject(cubeId);
    const auto& state = cubeAsset.getInertialState();
    const auto& frame = cubeAsset.getReferenceFrame();
    double omega = state.getAngularVelocity().norm();

    // Verify ReferenceFrame matches InertialState
    Coordinate framePos = frame.getOrigin();
    double frameDrift =
      (framePos - state.position).norm();

    // Probe collision with current cube position
    const auto& floorAsset = world.getEnvironmentalObjects()[0];
    auto result = handler.checkCollision(cubeAsset, floorAsset);

    if (result.has_value())
    {
      std::cout << std::setw(5) << i << "  (" << std::fixed
                << std::setprecision(4) << state.position.x() << ","
                << std::setprecision(4) << state.position.y() << ","
                << std::setprecision(4) << state.position.z() << ")"
                << "  " << std::setprecision(4) << state.velocity.z() << "     "
                << std::setprecision(4) << omega << "     "
                << result->contactCount << "         " << std::scientific
                << std::setprecision(3) << result->penetrationDepth << "  ("
                << std::fixed << std::setprecision(4) << result->normal.x()
                << "," << result->normal.y() << "," << result->normal.z()
                << ")\n";

      // Log frame sync issues
      if (frameDrift > 1e-10)
      {
        std::cout << "       ** FRAME DESYNC: state=(" << state.position.x()
                  << "," << state.position.y() << "," << state.position.z()
                  << ") frame=(" << framePos.x() << "," << framePos.y() << ","
                  << framePos.z() << ") drift=" << std::scientific << frameDrift
                  << "\n";
      }

      // For first 20 frames, log ALL contact details
      if (i <= 20)
      {
        for (size_t j = 0; j < result->contactCount; ++j)
        {
          const auto& cp = result->contacts[j];
          std::cout << "       C" << j << ": A=(" << std::fixed
                    << std::setprecision(4) << cp.pointA.x() << ","
                    << cp.pointA.y() << "," << cp.pointA.z() << ") B=("
                    << cp.pointB.x() << "," << cp.pointB.y() << ","
                    << cp.pointB.z() << ") depth=" << std::scientific
                    << cp.depth << "\n";
        }
      }
    }
    else
    {
      std::cout << std::setw(5) << i << "  (" << std::fixed
                << std::setprecision(4) << state.position.x() << ","
                << std::setprecision(4) << state.position.y() << ","
                << std::setprecision(4) << state.position.z() << ")"
                << "  " << std::setprecision(4) << state.velocity.z() << "     "
                << std::setprecision(4) << omega << "     NO COLLISION\n";
    }
  }

  std::cout << "=== END PER-FRAME DIAGNOSTIC ===\n\n";
}

// Ticket: 0047_face_contact_manifold_generation
// Test EPA at EXACTLY zero penetration (cube bottom = floor top = z=0).
// This is the exact initial condition of D1 test.
TEST(ManifoldDiagnostic, ZeroPenetrationEPA)
{
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};

  CollisionHandler handler{1e-6};

  // Test at z=0.5 (exactly touching), z=0.4999 (0.1mm pen), z=0.4995 (0.5mm pen)
  std::vector<double> cubeZPositions = {0.5, 0.4999, 0.4995, 0.499, 0.495, 0.49};

  std::cout << "\n=== ZERO/NEAR-ZERO PENETRATION EPA DIAGNOSTIC ===\n";

  for (double cubeZ : cubeZPositions)
  {
    double geometricPen = 0.5 - cubeZ;  // How far cube bottom is below z=0
    ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, cubeZ}};

    AssetInertial cube{1, 100, cubeHull, 1.0, cubeFrame, 0.5};
    AssetEnvironment floor{1, 200, floorHull, floorFrame, 0.5};

    auto result = handler.checkCollision(cube, floor);

    std::cout << "\nCube z=" << std::fixed << std::setprecision(4) << cubeZ
              << " (geometric pen=" << std::scientific << std::setprecision(3)
              << geometricPen << ")\n";

    if (!result.has_value())
    {
      std::cout << "  NO COLLISION DETECTED\n";
      continue;
    }

    std::cout << "  Normal: (" << std::fixed << std::setprecision(6)
              << result->normal.x() << ", " << result->normal.y() << ", "
              << result->normal.z() << ")\n";
    std::cout << "  EPA pen depth: " << std::scientific << std::setprecision(6)
              << result->penetrationDepth << "\n";
    std::cout << "  Contact count: " << result->contactCount << "\n";

    // Check if normal is purely vertical
    double lateralNormal =
      std::sqrt(result->normal.x() * result->normal.x() +
                result->normal.y() * result->normal.y());
    std::cout << "  Normal lateral magnitude: " << std::scientific
              << lateralNormal << "\n";

    if (lateralNormal > 1e-6)
    {
      std::cout << "  *** WARNING: NON-VERTICAL NORMAL DETECTED! ***\n";
    }

    for (size_t i = 0; i < result->contactCount; ++i)
    {
      const auto& cp = result->contacts[i];
      std::cout << "  C" << i << ": A=(" << std::fixed << std::setprecision(4)
                << cp.pointA.x() << "," << cp.pointA.y() << ","
                << cp.pointA.z() << ") B=(" << cp.pointB.x() << ","
                << cp.pointB.y() << "," << cp.pointB.z()
                << ") depth=" << std::scientific << cp.depth << "\n";
    }
  }

  std::cout << "=== END ZERO PENETRATION DIAGNOSTIC ===\n\n";
}

// Ticket: 0047_face_contact_manifold_generation
// Trace the FIRST frame of collision pipeline step by step.
// Uses two consecutive updates: frame 0 (no time, just init) and frame 1.
// Logs cube state BEFORE and AFTER each update to isolate displacement source.
TEST(ManifoldDiagnostic, FirstFrameDisplacementSource)
{
  WorldModel world;

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.5}};
  world.spawnObject(1, cubeHull, 1.0, cubeFrame);

  uint32_t cubeId = 1;
  world.getObject(cubeId).setCoefficientOfRestitution(0.5);

  std::cout << "\n=== FIRST FRAME DISPLACEMENT SOURCE ===\n";

  // Log initial state
  {
    const auto& state = world.getObject(cubeId).getInertialState();
    std::cout << "INITIAL: pos=(" << std::fixed << std::setprecision(6)
              << state.position.x() << "," << state.position.y() << ","
              << state.position.z() << ") vel=(" << state.velocity.x() << ","
              << state.velocity.y() << "," << state.velocity.z() << ")\n";
  }

  // Run first update (16ms)
  world.update(std::chrono::milliseconds{16});

  // Log state after first frame
  {
    const auto& state = world.getObject(cubeId).getInertialState();
    double omega = state.getAngularVelocity().norm();
    std::cout << "AFTER FRAME 1: pos=(" << std::fixed << std::setprecision(6)
              << state.position.x() << "," << state.position.y() << ","
              << state.position.z() << ") vel=(" << state.velocity.x() << ","
              << state.velocity.y() << "," << state.velocity.z()
              << ") omega=" << omega << "\n";

    double lateralDrift =
      std::sqrt(state.position.x() * state.position.x() +
                state.position.y() * state.position.y());
    std::cout << "  Lateral drift: " << std::scientific << lateralDrift << "\n";

    if (lateralDrift > 0.01)
    {
      std::cout << "  *** LARGE LATERAL DRIFT ON FRAME 1 ***\n";
    }
  }

  // Run 4 more frames to see evolution
  for (int i = 2; i <= 5; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});
    const auto& state = world.getObject(cubeId).getInertialState();
    double omega = state.getAngularVelocity().norm();
    std::cout << "AFTER FRAME " << i << ": pos=(" << std::fixed
              << std::setprecision(6) << state.position.x() << ","
              << state.position.y() << "," << state.position.z() << ") vel=("
              << state.velocity.x() << "," << state.velocity.y() << ","
              << state.velocity.z() << ") omega=" << omega << "\n";
  }

  std::cout << "=== END FIRST FRAME DISPLACEMENT ===\n\n";
}
