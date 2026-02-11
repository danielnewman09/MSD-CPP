// Ticket: 0055c_friction_direction_fix
// Design: docs/designs/0055c_friction_direction_fix/design.md

#include "msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.hpp"

#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"

namespace msd_sim
{

class VertexFaceManifoldGeneratorTest : public ::testing::Test
{
protected:
  VertexFaceManifoldGenerator generator_{};
};

// Test: Generate_CubeOnFloor_FourContacts
// Cube corner on flat floor should produce 4 contacts
TEST_F(VertexFaceManifoldGeneratorTest, Generate_CubeOnFloor_FourContacts)
{
  // Floor is XY plane at z=0, normal points up (0, 0, 1)
  // Cube corner (1, 1, 1) vertices define a square face
  std::vector<Coordinate> floorVerts{
    Coordinate{0.0, 0.0, 0.0},
    Coordinate{2.0, 0.0, 0.0},
    Coordinate{2.0, 2.0, 0.0},
    Coordinate{0.0, 2.0, 0.0}
  };

  Coordinate incidentVertex{1.0, 1.0, 0.5};  // Cube corner 0.5m above floor
  Vector3D contactNormal{0.0, 0.0, 1.0};  // Normal from cube toward floor
  double epaDepth = 0.5;

  std::array<ContactPoint, 4> contacts{};
  size_t count = generator_.generate(floorVerts, incidentVertex, contactNormal,
                                      epaDepth, contacts);

  // Should generate 4 contact points (one for each floor vertex projected)
  EXPECT_EQ(4, count);

  // All contacts should have EPA depth
  for (size_t i = 0; i < count; ++i)
  {
    EXPECT_DOUBLE_EQ(epaDepth, contacts[i].depth);
  }

  // All contacts should have incident vertex as pointB
  for (size_t i = 0; i < count; ++i)
  {
    EXPECT_DOUBLE_EQ(incidentVertex.x(), contacts[i].pointB.x());
    EXPECT_DOUBLE_EQ(incidentVertex.y(), contacts[i].pointB.y());
    EXPECT_DOUBLE_EQ(incidentVertex.z(), contacts[i].pointB.z());
  }

  // Contact pointA points should be the floor vertices (projected onto contact plane)
  // Since floor is already at z=0 and contact plane is at z=0.5, projection should
  // keep x,y same and set z=0.5
  for (size_t i = 0; i < count; ++i)
  {
    EXPECT_NEAR(floorVerts[i].x(), contacts[i].pointA.x(), 1e-9);
    EXPECT_NEAR(floorVerts[i].y(), contacts[i].pointA.y(), 1e-9);
    // All projected points should lie on contact plane (z = incidentVertex.z = 0.5)
    EXPECT_NEAR(0.5, contacts[i].pointA.z(), 1e-9);
  }
}

// Test: Generate_TriangleVertex_ThreeContacts
// Vertex on triangular face should produce 3 contacts
TEST_F(VertexFaceManifoldGeneratorTest, Generate_TriangleVertex_ThreeContacts)
{
  // Triangular face in XY plane at z=0
  std::vector<Coordinate> triangleVerts{
    Coordinate{0.0, 0.0, 0.0},
    Coordinate{2.0, 0.0, 0.0},
    Coordinate{1.0, std::sqrt(3.0), 0.0}  // Equilateral triangle
  };

  Coordinate incidentVertex{1.0, 0.5, 0.3};  // Vertex 0.3m above triangle
  Vector3D contactNormal{0.0, 0.0, 1.0};
  double epaDepth = 0.3;

  std::array<ContactPoint, 4> contacts{};
  size_t count = generator_.generate(triangleVerts, incidentVertex, contactNormal,
                                      epaDepth, contacts);

  // Should generate 3 contact points (triangle vertices)
  EXPECT_EQ(3, count);

  // All contacts should have EPA depth
  for (size_t i = 0; i < count; ++i)
  {
    EXPECT_DOUBLE_EQ(epaDepth, contacts[i].depth);
  }

  // All contacts should have projected triangle vertices as pointA
  // Projected z should equal incident vertex z (0.3)
  for (size_t i = 0; i < count; ++i)
  {
    EXPECT_NEAR(0.3, contacts[i].pointA.z(), 1e-9);
  }
}

// Test: Generate_DepthConsistency
// All contact depths should equal EPA depth (Option A implementation)
TEST_F(VertexFaceManifoldGeneratorTest, Generate_DepthConsistency)
{
  // Simple square face
  std::vector<Coordinate> faceVerts{
    Coordinate{-1.0, -1.0, 0.0},
    Coordinate{ 1.0, -1.0, 0.0},
    Coordinate{ 1.0,  1.0, 0.0},
    Coordinate{-1.0,  1.0, 0.0}
  };

  Coordinate incidentVertex{0.0, 0.0, 1.0};
  Vector3D contactNormal{0.0, 0.0, 1.0};
  double epaDepth = 1.5;

  std::array<ContactPoint, 4> contacts{};
  size_t count = generator_.generate(faceVerts, incidentVertex, contactNormal,
                                      epaDepth, contacts);

  ASSERT_EQ(4, count);

  // ALL contacts must have identical depth (Option A: uniform EPA depth)
  for (size_t i = 0; i < count; ++i)
  {
    EXPECT_DOUBLE_EQ(epaDepth, contacts[i].depth)
      << "Contact " << i << " depth mismatch";
  }
}

// Test: Generate_DegenerateFace_ReturnsZero
// < 3 face vertices should return 0 contacts
TEST_F(VertexFaceManifoldGeneratorTest, Generate_DegenerateFace_ReturnsZero)
{
  Coordinate incidentVertex{0.0, 0.0, 1.0};
  Vector3D contactNormal{0.0, 0.0, 1.0};
  double epaDepth = 1.0;
  std::array<ContactPoint, 4> contacts{};

  // Empty face (0 vertices)
  {
    std::vector<Coordinate> emptyFace{};
    size_t count = generator_.generate(emptyFace, incidentVertex, contactNormal,
                                        epaDepth, contacts);
    EXPECT_EQ(0, count);
  }

  // Single vertex face (1 vertex)
  {
    std::vector<Coordinate> singleVertFace{Coordinate{0.0, 0.0, 0.0}};
    size_t count = generator_.generate(singleVertFace, incidentVertex,
                                        contactNormal, epaDepth, contacts);
    EXPECT_EQ(0, count);
  }

  // Edge face (2 vertices)
  {
    std::vector<Coordinate> edgeFace{
      Coordinate{0.0, 0.0, 0.0},
      Coordinate{1.0, 0.0, 0.0}
    };
    size_t count = generator_.generate(edgeFace, incidentVertex, contactNormal,
                                        epaDepth, contacts);
    EXPECT_EQ(0, count);
  }
}

// Test: Generate_MaxContactLimit
// Generator should respect maxContacts limit (default 4)
TEST_F(VertexFaceManifoldGeneratorTest, Generate_MaxContactLimit)
{
  // Hexagonal face (6 vertices) - should be clamped to 4
  std::vector<Coordinate> hexVerts{
    Coordinate{ 1.0,  0.0, 0.0},
    Coordinate{ 0.5,  std::sqrt(3.0)/2.0, 0.0},
    Coordinate{-0.5,  std::sqrt(3.0)/2.0, 0.0},
    Coordinate{-1.0,  0.0, 0.0},
    Coordinate{-0.5, -std::sqrt(3.0)/2.0, 0.0},
    Coordinate{ 0.5, -std::sqrt(3.0)/2.0, 0.0}
  };

  Coordinate incidentVertex{0.0, 0.0, 1.0};
  Vector3D contactNormal{0.0, 0.0, 1.0};
  double epaDepth = 1.0;

  std::array<ContactPoint, 4> contacts{};
  size_t count = generator_.generate(hexVerts, incidentVertex, contactNormal,
                                      epaDepth, contacts);

  // Should clamp to maxContacts (4)
  EXPECT_EQ(4, count);
}

// Test: Generate_ProjectionPlaneCorrectness
// Verify projected points lie on contact plane
TEST_F(VertexFaceManifoldGeneratorTest, Generate_ProjectionPlaneCorrectness)
{
  // Face vertices at various z heights (tilted face)
  std::vector<Coordinate> tiltedFaceVerts{
    Coordinate{-1.0, -1.0,  0.0},
    Coordinate{ 1.0, -1.0,  0.1},
    Coordinate{ 1.0,  1.0,  0.2},
    Coordinate{-1.0,  1.0,  0.1}
  };

  Coordinate incidentVertex{0.0, 0.0, 1.0};
  Vector3D contactNormal{0.0, 0.0, 1.0};  // Vertical normal
  double epaDepth = 1.0;

  std::array<ContactPoint, 4> contacts{};
  size_t count = generator_.generate(tiltedFaceVerts, incidentVertex,
                                      contactNormal, epaDepth, contacts);

  ASSERT_EQ(4, count);

  // All projected points should lie on contact plane: z = incidentVertex.z = 1.0
  for (size_t i = 0; i < count; ++i)
  {
    EXPECT_NEAR(incidentVertex.z(), contacts[i].pointA.z(), 1e-9)
      << "Contact " << i << " not on contact plane";
  }
}

}  // namespace msd_sim
