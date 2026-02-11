// Ticket: 0055c_friction_direction_fix
// Design: docs/designs/0055c_friction_direction_fix/design.md

#include "msd-sim/src/Physics/Collision/VertexFaceDetector.hpp"

#include <gtest/gtest.h>

namespace msd_sim
{

class VertexFaceDetectorTest : public ::testing::Test
{
protected:
  VertexFaceDetector detector_{};
};

// Test: DetectContactType_FaceFace
// Both sides have >= 3 vertices → FaceFace
TEST_F(VertexFaceDetectorTest, DetectContactType_FaceFace)
{
  // Both sides are faces
  EXPECT_EQ(ContactType::FaceFace, detector_.detectContactType(3, 3));
  EXPECT_EQ(ContactType::FaceFace, detector_.detectContactType(4, 3));
  EXPECT_EQ(ContactType::FaceFace, detector_.detectContactType(3, 4));
  EXPECT_EQ(ContactType::FaceFace, detector_.detectContactType(4, 4));
  EXPECT_EQ(ContactType::FaceFace, detector_.detectContactType(5, 3));
  EXPECT_EQ(ContactType::FaceFace, detector_.detectContactType(3, 5));
}

// Test: DetectContactType_EdgeEdge
// Both sides have exactly 2 vertices → EdgeEdge
TEST_F(VertexFaceDetectorTest, DetectContactType_EdgeEdge)
{
  EXPECT_EQ(ContactType::EdgeEdge, detector_.detectContactType(2, 2));
}

// Test: DetectContactType_VertexFace
// One side has 1 vertex, other has >= 3 → VertexFace
TEST_F(VertexFaceDetectorTest, DetectContactType_VertexFace)
{
  // Reference is vertex, incident is face
  EXPECT_EQ(ContactType::VertexFace, detector_.detectContactType(1, 3));
  EXPECT_EQ(ContactType::VertexFace, detector_.detectContactType(1, 4));
  EXPECT_EQ(ContactType::VertexFace, detector_.detectContactType(1, 5));

  // Reference is face, incident is vertex
  EXPECT_EQ(ContactType::VertexFace, detector_.detectContactType(3, 1));
  EXPECT_EQ(ContactType::VertexFace, detector_.detectContactType(4, 1));
  EXPECT_EQ(ContactType::VertexFace, detector_.detectContactType(5, 1));

  // Verify isVertexFaceContact helper
  EXPECT_TRUE(detector_.isVertexFaceContact(1, 3));
  EXPECT_TRUE(detector_.isVertexFaceContact(3, 1));
  EXPECT_FALSE(detector_.isVertexFaceContact(3, 3));  // FaceFace
  EXPECT_FALSE(detector_.isVertexFaceContact(2, 2));  // EdgeEdge
  EXPECT_FALSE(detector_.isVertexFaceContact(1, 1));  // VertexVertex
}

// Test: DetectContactType_VertexVertex
// Both sides have exactly 1 vertex → VertexVertex
TEST_F(VertexFaceDetectorTest, DetectContactType_VertexVertex)
{
  EXPECT_EQ(ContactType::VertexVertex, detector_.detectContactType(1, 1));
}

// Test: DetectContactType_Unknown
// Edge cases and unexpected geometry → Unknown
TEST_F(VertexFaceDetectorTest, DetectContactType_Unknown)
{
  // Edge vs vertex (1 vs 2, or 2 vs 1)
  EXPECT_EQ(ContactType::Unknown, detector_.detectContactType(1, 2));
  EXPECT_EQ(ContactType::Unknown, detector_.detectContactType(2, 1));

  // Edge vs face (2 vs 3+, or 3+ vs 2) - these are edge-face, not vertex-face
  EXPECT_EQ(ContactType::Unknown, detector_.detectContactType(2, 3));
  EXPECT_EQ(ContactType::Unknown, detector_.detectContactType(3, 2));

  // Zero vertices (degenerate)
  EXPECT_EQ(ContactType::Unknown, detector_.detectContactType(0, 0));
  EXPECT_EQ(ContactType::Unknown, detector_.detectContactType(0, 3));
  EXPECT_EQ(ContactType::Unknown, detector_.detectContactType(3, 0));
}

}  // namespace msd_sim
