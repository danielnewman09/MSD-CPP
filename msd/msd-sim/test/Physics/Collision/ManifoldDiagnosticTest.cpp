// Ticket: 0047_face_contact_manifold_generation
// Diagnostic test to identify why EPA produces single contact point
// for face-on-face contacts. This test is temporary and will be
// removed after the root cause is identified and fixed.

#include <gtest/gtest.h>

#include <iostream>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
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
