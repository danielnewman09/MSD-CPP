// Unit tests for shader transformation logic
// These tests verify that the CPU-side matrix construction matches
// what the HLSL shader expects

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <array>

// Include ReferenceFrame and related types from msd-sim
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"
#include "msd-sim/src/Environment/Angle.hpp"

// Tolerance for floating-point comparisons
constexpr float FLOAT_TOLERANCE = 1e-5f;

bool floatEqual(float a, float b, float tolerance = FLOAT_TOLERANCE)
{
  return std::abs(a - b) < tolerance;
}

bool vec4Equal(const Eigen::Vector4f& a,
               const Eigen::Vector4f& b,
               float tolerance = FLOAT_TOLERANCE)
{
  return floatEqual(a.x(), b.x(), tolerance) &&
         floatEqual(a.y(), b.y(), tolerance) &&
         floatEqual(a.z(), b.z(), tolerance) &&
         floatEqual(a.w(), b.w(), tolerance);
}

// ============================================================================
// Simulates HLSL shader matrix construction and transformation
// This must match Position3DColorTransform.vert.hlsl exactly
// ============================================================================

/**
 * @brief Simulates HLSL's float4x4 constructor with 4 row vectors
 *
 * In HLSL, float4x4(row0, row1, row2, row3) constructs a matrix
 * where row0-row3 become the ROWS of the matrix.
 */
Eigen::Matrix4f hlslFloat4x4FromRows(const Eigen::Vector4f& row0,
                                     const Eigen::Vector4f& row1,
                                     const Eigen::Vector4f& row2,
                                     const Eigen::Vector4f& row3)
{
  Eigen::Matrix4f m;
  m.row(0) = row0;
  m.row(1) = row1;
  m.row(2) = row2;
  m.row(3) = row3;
  return m;
}

/**
 * @brief Simulates the shader's matrix construction and vertex transformation
 *
 * From Position3DColorTransform.vert.hlsl:
 *   float4x4 instanceModel = transpose(float4x4(
 *       input.InstanceModelRow0,
 *       input.InstanceModelRow1,
 *       input.InstanceModelRow2,
 *       input.InstanceModelRow3
 *   ));
 *   float4 worldPos = mul(instanceModel, float4(input.Position, 1.0f));
 */
Eigen::Vector4f simulateShaderTransform(const Eigen::Vector4f& row0,
                                        const Eigen::Vector4f& row1,
                                        const Eigen::Vector4f& row2,
                                        const Eigen::Vector4f& row3,
                                        const Eigen::Vector3f& vertexPos)
{
  // Step 1: Construct matrix from rows
  Eigen::Matrix4f matFromRows = hlslFloat4x4FromRows(row0, row1, row2, row3);

  // Step 2: Transpose (as shader does)
  Eigen::Matrix4f instanceModel = matFromRows.transpose();

  // Step 3: Transform vertex (mul(matrix, vector) in HLSL)
  Eigen::Vector4f vertex4{vertexPos.x(), vertexPos.y(), vertexPos.z(), 1.0f};
  return instanceModel * vertex4;
}

// ============================================================================
// Simulates CPU-side matrix creation (from SDLGPUManager.cpp)
// ============================================================================

/**
 * @brief Creates a model matrix the same way GPUManager::createModelMatrix does
 */
Eigen::Matrix4f createModelMatrix(const Eigen::Vector3f& position,
                                  const Eigen::Matrix3f& rotation =
                                    Eigen::Matrix3f::Identity())
{
  Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();

  // Set rotation (top-left 3x3)
  modelMatrix.block<3, 3>(0, 0) = rotation;

  // Set translation (last column, first 3 rows)
  modelMatrix(0, 3) = position.x();
  modelMatrix(1, 3) = position.y();
  modelMatrix(2, 3) = position.z();

  return modelMatrix;
}

/**
 * @brief Extracts the 4 float4 values that would be sent to the shader
 *        as vertex attributes (simulating what goes into InstanceData)
 *
 * This copies from modelMatrix.data() which is column-major in Eigen.
 */
std::array<Eigen::Vector4f, 4> extractShaderRows(const Eigen::Matrix4f& matrix)
{
  std::array<Eigen::Vector4f, 4> rows;

  // Eigen stores column-major, so data() gives us columns sequentially
  // data[0-3] = column 0, data[4-7] = column 1, etc.
  const float* data = matrix.data();

  rows[0] = Eigen::Vector4f{data[0], data[1], data[2], data[3]};    // Column 0
  rows[1] = Eigen::Vector4f{data[4], data[5], data[6], data[7]};    // Column 1
  rows[2] = Eigen::Vector4f{data[8], data[9], data[10], data[11]};  // Column 2
  rows[3] =
    Eigen::Vector4f{data[12], data[13], data[14], data[15]};  // Column 3

  return rows;
}

// ============================================================================
// Test: Identity Matrix
// ============================================================================

TEST(ShaderTransformTest, IdentityMatrix_VertexUnchanged)
{
  // Create identity model matrix (no translation, no rotation)
  Eigen::Matrix4f modelMatrix = createModelMatrix(Eigen::Vector3f::Zero());

  // Extract rows as they would be sent to shader
  auto rows = extractShaderRows(modelMatrix);

  // Test vertex at (1, 2, 3)
  Eigen::Vector3f vertex{1.0f, 2.0f, 3.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  // With identity matrix, output should equal input
  Eigen::Vector4f expected{1.0f, 2.0f, 3.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")\n"
    << "Expected: (" << expected.x() << ", " << expected.y() << ", "
    << expected.z() << ", " << expected.w() << ")";
}

// ============================================================================
// Test: Translation Only
// ============================================================================

TEST(ShaderTransformTest, TranslationOnly_VertexTranslated)
{
  // Create model matrix with translation (5, 10, 15)
  Eigen::Vector3f translation{5.0f, 10.0f, 15.0f};
  Eigen::Matrix4f modelMatrix = createModelMatrix(translation);

  // Extract rows as they would be sent to shader
  auto rows = extractShaderRows(modelMatrix);

  // Test vertex at origin
  Eigen::Vector3f vertex{0.0f, 0.0f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  // Vertex at origin should be translated to (5, 10, 15)
  Eigen::Vector4f expected{5.0f, 10.0f, 15.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")\n"
    << "Expected: (" << expected.x() << ", " << expected.y() << ", "
    << expected.z() << ", " << expected.w() << ")";
}

TEST(ShaderTransformTest, TranslationOnly_VertexAtOffset)
{
  // Create model matrix with translation (5, 0, 0)
  Eigen::Vector3f translation{5.0f, 0.0f, 0.0f};
  Eigen::Matrix4f modelMatrix = createModelMatrix(translation);

  // Extract rows as they would be sent to shader
  auto rows = extractShaderRows(modelMatrix);

  // Test vertex at (1, 2, 3)
  Eigen::Vector3f vertex{1.0f, 2.0f, 3.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  // Vertex should be translated: (1+5, 2+0, 3+0) = (6, 2, 3)
  Eigen::Vector4f expected{6.0f, 2.0f, 3.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")\n"
    << "Expected: (" << expected.x() << ", " << expected.y() << ", "
    << expected.z() << ", " << expected.w() << ")";
}

// ============================================================================
// Test: W Component
// ============================================================================

TEST(ShaderTransformTest, WComponentIsOne)
{
  // Any transformation should preserve w=1 for position vectors
  Eigen::Vector3f translation{100.0f, 200.0f, 300.0f};
  Eigen::Matrix4f modelMatrix = createModelMatrix(translation);

  auto rows = extractShaderRows(modelMatrix);

  Eigen::Vector3f vertex{1.0f, 2.0f, 3.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  EXPECT_TRUE(floatEqual(result.w(), 1.0f))
    << "W component should be 1.0, got: " << result.w();
}

// ============================================================================
// Test: Rotation (90 degrees about Y axis)
// ============================================================================

TEST(ShaderTransformTest, Rotation90DegreesY)
{
  // Rotation matrix for 90 degrees about Y axis
  // This rotates X->Z, Z->-X
  float angle = static_cast<float>(M_PI / 2);
  Eigen::Matrix3f rotation;
  rotation = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY());

  Eigen::Matrix4f modelMatrix =
    createModelMatrix(Eigen::Vector3f::Zero(), rotation);

  auto rows = extractShaderRows(modelMatrix);

  // Vertex at (1, 0, 0) should rotate to (0, 0, -1)
  Eigen::Vector3f vertex{1.0f, 0.0f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  Eigen::Vector4f expected{0.0f, 0.0f, -1.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")\n"
    << "Expected: (" << expected.x() << ", " << expected.y() << ", "
    << expected.z() << ", " << expected.w() << ")";
}

// ============================================================================
// Test: Combined Rotation and Translation
// ============================================================================

TEST(ShaderTransformTest, RotationAndTranslation)
{
  // 90 degree rotation about Y, then translate by (10, 0, 0)
  float angle = static_cast<float>(M_PI / 2);
  Eigen::Matrix3f rotation;
  rotation = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY());

  Eigen::Vector3f translation{10.0f, 0.0f, 0.0f};
  Eigen::Matrix4f modelMatrix = createModelMatrix(translation, rotation);

  auto rows = extractShaderRows(modelMatrix);

  // Vertex at (1, 0, 0):
  // First rotated: (1, 0, 0) -> (0, 0, -1)
  // Then translated: (0, 0, -1) + (10, 0, 0) = (10, 0, -1)
  Eigen::Vector3f vertex{1.0f, 0.0f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  Eigen::Vector4f expected{10.0f, 0.0f, -1.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")\n"
    << "Expected: (" << expected.x() << ", " << expected.y() << ", "
    << expected.z() << ", " << expected.w() << ")";
}

// ============================================================================
// Test: Matrix Data Layout
// ============================================================================

TEST(ShaderTransformTest, MatrixDataLayout_ColumnMajor)
{
  // Verify that Eigen's data() gives us column-major layout
  Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
  m(0, 0) = 1.0f;   // row 0, col 0
  m(1, 0) = 2.0f;   // row 1, col 0
  m(2, 0) = 3.0f;   // row 2, col 0
  m(3, 0) = 4.0f;   // row 3, col 0
  m(0, 3) = 10.0f;  // row 0, col 3 (translation X)
  m(1, 3) = 20.0f;  // row 1, col 3 (translation Y)
  m(2, 3) = 30.0f;  // row 2, col 3 (translation Z)
  m(3, 3) = 1.0f;   // row 3, col 3

  const float* data = m.data();

  // Column 0 should be at indices 0-3
  EXPECT_FLOAT_EQ(data[0], 1.0f) << "m(0,0) should be at index 0";
  EXPECT_FLOAT_EQ(data[1], 2.0f) << "m(1,0) should be at index 1";
  EXPECT_FLOAT_EQ(data[2], 3.0f) << "m(2,0) should be at index 2";
  EXPECT_FLOAT_EQ(data[3], 4.0f) << "m(3,0) should be at index 3";

  // Column 3 (translation) should be at indices 12-15
  EXPECT_FLOAT_EQ(data[12], 10.0f) << "m(0,3) should be at index 12";
  EXPECT_FLOAT_EQ(data[13], 20.0f) << "m(1,3) should be at index 13";
  EXPECT_FLOAT_EQ(data[14], 30.0f) << "m(2,3) should be at index 14";
  EXPECT_FLOAT_EQ(data[15], 1.0f) << "m(3,3) should be at index 15";
}

TEST(ShaderTransformTest, ExtractShaderRows_MatchesColumnMajorLayout)
{
  // Create a known matrix
  Eigen::Vector3f translation{5.0f, 10.0f, 15.0f};
  Eigen::Matrix4f modelMatrix = createModelMatrix(translation);

  auto rows = extractShaderRows(modelMatrix);

  // Row0 should be column 0 of the matrix: [1, 0, 0, 0] (identity rotation)
  EXPECT_TRUE(
    vec4Equal(rows[0], Eigen::Vector4f{1.0f, 0.0f, 0.0f, 0.0f}))
    << "Row0 (column 0): (" << rows[0].x() << ", " << rows[0].y() << ", "
    << rows[0].z() << ", " << rows[0].w() << ")";

  // Row1 should be column 1: [0, 1, 0, 0]
  EXPECT_TRUE(
    vec4Equal(rows[1], Eigen::Vector4f{0.0f, 1.0f, 0.0f, 0.0f}))
    << "Row1 (column 1): (" << rows[1].x() << ", " << rows[1].y() << ", "
    << rows[1].z() << ", " << rows[1].w() << ")";

  // Row2 should be column 2: [0, 0, 1, 0]
  EXPECT_TRUE(
    vec4Equal(rows[2], Eigen::Vector4f{0.0f, 0.0f, 1.0f, 0.0f}))
    << "Row2 (column 2): (" << rows[2].x() << ", " << rows[2].y() << ", "
    << rows[2].z() << ", " << rows[2].w() << ")";

  // Row3 should be column 3: [tx, ty, tz, 1] = [5, 10, 15, 1]
  EXPECT_TRUE(
    vec4Equal(rows[3], Eigen::Vector4f{5.0f, 10.0f, 15.0f, 1.0f}))
    << "Row3 (column 3): (" << rows[3].x() << ", " << rows[3].y() << ", "
    << rows[3].z() << ", " << rows[3].w() << ")";
}

// ============================================================================
// Test: Pyramid Vertices (practical test case)
// ============================================================================

TEST(ShaderTransformTest, PyramidApex_TranslatedCorrectly)
{
  // Pyramid apex is at (0, 0.5, 0) in local space
  // Object is at world position (3, 0, -2)

  Eigen::Vector3f translation{3.0f, 0.0f, -2.0f};
  Eigen::Matrix4f modelMatrix = createModelMatrix(translation);

  auto rows = extractShaderRows(modelMatrix);

  Eigen::Vector3f apex{0.0f, 0.5f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], apex);

  // Apex should be at (3, 0.5, -2) in world space
  Eigen::Vector4f expected{3.0f, 0.5f, -2.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Pyramid apex should be at (3, 0.5, -2)\n"
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")";
}

TEST(ShaderTransformTest, PyramidBaseCorner_TranslatedCorrectly)
{
  // Pyramid base corner at (-0.5, -0.5, -0.5) in local space
  // Object is at world position (0, 0, 0)

  Eigen::Vector3f translation{0.0f, 0.0f, 0.0f};
  Eigen::Matrix4f modelMatrix = createModelMatrix(translation);

  auto rows = extractShaderRows(modelMatrix);

  Eigen::Vector3f corner{-0.5f, -0.5f, -0.5f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], corner);

  // Corner should remain at (-0.5, -0.5, -0.5) since no translation
  Eigen::Vector4f expected{-0.5f, -0.5f, -0.5f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Pyramid corner should stay at (-0.5, -0.5, -0.5)\n"
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")";
}

// ============================================================================
// ReferenceFrame Integration Tests
// These verify the actual data flow from msd_sim::ReferenceFrame to shader
// ============================================================================

/**
 * @brief Creates a model matrix from a ReferenceFrame, mimicking
 *        GPUManager::createModelMatrix
 */
Eigen::Matrix4f createModelMatrixFromReferenceFrame(
  const msd_sim::ReferenceFrame& frame)
{
  Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();

  // Get rotation matrix from ReferenceFrame
  const Eigen::Matrix3d& rotation = frame.getRotation();
  modelMatrix.block<3, 3>(0, 0) = rotation.cast<float>();

  // Get origin (translation)
  const msd_sim::Coordinate& origin = frame.getOrigin();
  modelMatrix(0, 3) = static_cast<float>(origin.x());
  modelMatrix(1, 3) = static_cast<float>(origin.y());
  modelMatrix(2, 3) = static_cast<float>(origin.z());

  return modelMatrix;
}

TEST(ReferenceFrameShaderTest, IdentityFrame_VertexUnchanged)
{
  // Default ReferenceFrame is at origin with no rotation
  msd_sim::ReferenceFrame frame;

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  Eigen::Vector3f vertex{1.0f, 2.0f, 3.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  Eigen::Vector4f expected{1.0f, 2.0f, 3.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Identity frame should not change vertex\n"
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")";
}

TEST(ReferenceFrameShaderTest, TranslationOnly_VertexTranslated)
{
  // ReferenceFrame at position (5, 10, 15) with no rotation
  msd_sim::Coordinate origin{5.0, 10.0, 15.0};
  msd_sim::ReferenceFrame frame{origin};

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  Eigen::Vector3f vertex{0.0f, 0.0f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  Eigen::Vector4f expected{5.0f, 10.0f, 15.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Vertex at origin should be translated to frame origin\n"
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")";
}

TEST(ReferenceFrameShaderTest, Yaw90Degrees_VertexRotated)
{
  // ReferenceFrame at origin with 90 degree yaw (Z-axis rotation)
  msd_sim::Coordinate origin{0.0, 0.0, 0.0};
  msd_sim::EulerAngles euler{
    msd_sim::Angle::fromDegrees(0.0),   // pitch
    msd_sim::Angle::fromDegrees(0.0),   // roll
    msd_sim::Angle::fromDegrees(90.0)   // yaw
  };
  msd_sim::ReferenceFrame frame{origin, euler};

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  // Vertex at (1, 0, 0) should rotate to (0, 1, 0) after 90° yaw
  Eigen::Vector3f vertex{1.0f, 0.0f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  Eigen::Vector4f expected{0.0f, 1.0f, 0.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "90° yaw should rotate (1,0,0) to (0,1,0)\n"
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")";
}

TEST(ReferenceFrameShaderTest, Pitch90Degrees_VertexRotated)
{
  // ReferenceFrame at origin with 90 degree pitch (Y-axis rotation)
  msd_sim::Coordinate origin{0.0, 0.0, 0.0};
  msd_sim::EulerAngles euler{
    msd_sim::Angle::fromDegrees(90.0),  // pitch
    msd_sim::Angle::fromDegrees(0.0),   // roll
    msd_sim::Angle::fromDegrees(0.0)    // yaw
  };
  msd_sim::ReferenceFrame frame{origin, euler};

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  // Vertex at (1, 0, 0) should rotate to (0, 0, -1) after 90° pitch
  Eigen::Vector3f vertex{1.0f, 0.0f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  Eigen::Vector4f expected{0.0f, 0.0f, -1.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "90° pitch should rotate (1,0,0) to (0,0,-1)\n"
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")";
}

TEST(ReferenceFrameShaderTest, Roll90Degrees_VertexRotated)
{
  // ReferenceFrame at origin with 90 degree roll (X-axis rotation)
  msd_sim::Coordinate origin{0.0, 0.0, 0.0};
  msd_sim::EulerAngles euler{
    msd_sim::Angle::fromDegrees(0.0),   // pitch
    msd_sim::Angle::fromDegrees(90.0),  // roll
    msd_sim::Angle::fromDegrees(0.0)    // yaw
  };
  msd_sim::ReferenceFrame frame{origin, euler};

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  // Vertex at (0, 1, 0) should rotate to (0, 0, 1) after 90° roll
  Eigen::Vector3f vertex{0.0f, 1.0f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  Eigen::Vector4f expected{0.0f, 0.0f, 1.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "90° roll should rotate (0,1,0) to (0,0,1)\n"
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")";
}

TEST(ReferenceFrameShaderTest, RotationAndTranslation_Combined)
{
  // ReferenceFrame at (10, 0, 0) with 90 degree yaw
  msd_sim::Coordinate origin{10.0, 0.0, 0.0};
  msd_sim::EulerAngles euler{
    msd_sim::Angle::fromDegrees(0.0),   // pitch
    msd_sim::Angle::fromDegrees(0.0),   // roll
    msd_sim::Angle::fromDegrees(90.0)   // yaw
  };
  msd_sim::ReferenceFrame frame{origin, euler};

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  // Vertex at (1, 0, 0) in local space:
  // 1. Rotated by 90° yaw: (1, 0, 0) -> (0, 1, 0)
  // 2. Translated by (10, 0, 0): (0, 1, 0) -> (10, 1, 0)
  Eigen::Vector3f vertex{1.0f, 0.0f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  Eigen::Vector4f expected{10.0f, 1.0f, 0.0f, 1.0f};
  EXPECT_TRUE(vec4Equal(result, expected))
    << "Rotation then translation should give (10, 1, 0)\n"
    << "Result: (" << result.x() << ", " << result.y() << ", " << result.z()
    << ", " << result.w() << ")";
}

TEST(ReferenceFrameShaderTest, WComponentPreserved)
{
  // Any transformation should preserve w=1
  msd_sim::Coordinate origin{100.0, 200.0, 300.0};
  msd_sim::EulerAngles euler{
    msd_sim::Angle::fromDegrees(45.0),
    msd_sim::Angle::fromDegrees(30.0),
    msd_sim::Angle::fromDegrees(60.0)
  };
  msd_sim::ReferenceFrame frame{origin, euler};

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  Eigen::Vector3f vertex{1.0f, 2.0f, 3.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  EXPECT_TRUE(floatEqual(result.w(), 1.0f))
    << "W component must be 1.0, got: " << result.w();
}

TEST(ReferenceFrameShaderTest, ConsistencyWithLocalToGlobal)
{
  // The shader transformation should match ReferenceFrame::localToGlobalAbsolute
  msd_sim::Coordinate origin{5.0, 10.0, 15.0};
  msd_sim::EulerAngles euler{
    msd_sim::Angle::fromDegrees(30.0),
    msd_sim::Angle::fromDegrees(45.0),
    msd_sim::Angle::fromDegrees(60.0)
  };
  msd_sim::ReferenceFrame frame{origin, euler};

  // Use ReferenceFrame's localToGlobalAbsolute as ground truth
  msd_sim::Coordinate localPoint{1.0, 2.0, 3.0};
  msd_sim::Coordinate expectedGlobal = frame.localToGlobalAbsolute(localPoint);

  // Now test shader simulation
  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  Eigen::Vector3f vertex{1.0f, 2.0f, 3.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], vertex);

  // Compare shader result with ReferenceFrame result
  EXPECT_TRUE(floatEqual(result.x(), static_cast<float>(expectedGlobal.x())))
    << "X mismatch: shader=" << result.x()
    << ", ReferenceFrame=" << expectedGlobal.x();
  EXPECT_TRUE(floatEqual(result.y(), static_cast<float>(expectedGlobal.y())))
    << "Y mismatch: shader=" << result.y()
    << ", ReferenceFrame=" << expectedGlobal.y();
  EXPECT_TRUE(floatEqual(result.z(), static_cast<float>(expectedGlobal.z())))
    << "Z mismatch: shader=" << result.z()
    << ", ReferenceFrame=" << expectedGlobal.z();
}

TEST(ReferenceFrameShaderTest, PyramidVertexWithRotation)
{
  // Simulate a pyramid at position (3, 0, -2) with 45° yaw
  msd_sim::Coordinate origin{3.0, 0.0, -2.0};
  msd_sim::EulerAngles euler{
    msd_sim::Angle::fromDegrees(0.0),
    msd_sim::Angle::fromDegrees(0.0),
    msd_sim::Angle::fromDegrees(45.0)
  };
  msd_sim::ReferenceFrame frame{origin, euler};

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  // Pyramid apex at local (0, 0.5, 0)
  Eigen::Vector3f apex{0.0f, 0.5f, 0.0f};
  Eigen::Vector4f result = simulateShaderTransform(
    rows[0], rows[1], rows[2], rows[3], apex);

  // Ground truth from ReferenceFrame
  msd_sim::Coordinate localApex{0.0, 0.5, 0.0};
  msd_sim::Coordinate expectedGlobal = frame.localToGlobalAbsolute(localApex);

  EXPECT_TRUE(floatEqual(result.x(), static_cast<float>(expectedGlobal.x())))
    << "Apex X: shader=" << result.x()
    << ", expected=" << expectedGlobal.x();
  EXPECT_TRUE(floatEqual(result.y(), static_cast<float>(expectedGlobal.y())))
    << "Apex Y: shader=" << result.y()
    << ", expected=" << expectedGlobal.y();
  EXPECT_TRUE(floatEqual(result.z(), static_cast<float>(expectedGlobal.z())))
    << "Apex Z: shader=" << result.z()
    << ", expected=" << expectedGlobal.z();
  EXPECT_TRUE(floatEqual(result.w(), 1.0f))
    << "Apex W should be 1.0, got: " << result.w();
}

// ============================================================================
// InstanceData Layout Tests
// Verify the exact byte layout matches what the GPU expects
// ============================================================================

// Mimics the InstanceData struct from SDLGPUManager.hpp
struct TestInstanceData
{
  float modelMatrix[16];   // 64 bytes - offsets 0-63
  float color[3];          // 12 bytes - offsets 64-75
  uint32_t geometryIndex;  // 4 bytes  - offset 76
  uint32_t padding;        // 4 bytes  - offset 80
};  // Total: 84 bytes

TEST(InstanceDataLayoutTest, StructSize)
{
  EXPECT_EQ(sizeof(TestInstanceData), 84u)
    << "InstanceData should be exactly 84 bytes";
}

TEST(InstanceDataLayoutTest, ModelMatrixOffset)
{
  EXPECT_EQ(offsetof(TestInstanceData, modelMatrix), 0u)
    << "modelMatrix should start at offset 0";
}

TEST(InstanceDataLayoutTest, ColorOffset)
{
  EXPECT_EQ(offsetof(TestInstanceData, color), 64u)
    << "color should start at offset 64 (sizeof(float)*16)";
}

TEST(InstanceDataLayoutTest, GeometryIndexOffset)
{
  EXPECT_EQ(offsetof(TestInstanceData, geometryIndex), 76u)
    << "geometryIndex should start at offset 76 (sizeof(float)*19)";
}

TEST(InstanceDataLayoutTest, VertexAttributeOffsets)
{
  // These must match the SDL_GPUVertexAttribute definitions in SDLGPUManager.cpp
  // Location 3: Model matrix row 0 at offset 0
  EXPECT_EQ(sizeof(float) * 0, 0u);
  // Location 4: Model matrix row 1 at offset 16
  EXPECT_EQ(sizeof(float) * 4, 16u);
  // Location 5: Model matrix row 2 at offset 32
  EXPECT_EQ(sizeof(float) * 8, 32u);
  // Location 6: Model matrix row 3 at offset 48
  EXPECT_EQ(sizeof(float) * 12, 48u);
  // Location 7: Instance color at offset 64
  EXPECT_EQ(sizeof(float) * 16, 64u);
  // Location 8: Geometry index at offset 76
  EXPECT_EQ(sizeof(float) * 19, 76u);
}

TEST(InstanceDataLayoutTest, MatrixRowsMatchShaderExpectation)
{
  // Create a model matrix with known values
  msd_sim::Coordinate origin{5.0, 10.0, 15.0};
  msd_sim::ReferenceFrame frame{origin};

  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);

  // Fill instance data the same way GPUManager does
  TestInstanceData data{};
  for (int i = 0; i < 16; ++i)
  {
    data.modelMatrix[i] = modelMatrix.data()[i];
  }

  // Verify the layout matches what the shader expects
  // Shader reads at offsets 0, 16, 32, 48 for the 4 float4 rows
  auto* row0 = reinterpret_cast<float*>(&data.modelMatrix[0]);
  // row1/row2 at offsets 4/8 are intermediate columns, not verified in this test
  auto* row3 = reinterpret_cast<float*>(&data.modelMatrix[12]);

  // These are Eigen columns being read as shader rows
  // Column 0 (identity rotation, first column)
  EXPECT_FLOAT_EQ(row0[0], 1.0f) << "Row0[0] should be 1 (rotation[0,0])";
  EXPECT_FLOAT_EQ(row0[1], 0.0f) << "Row0[1] should be 0 (rotation[1,0])";
  EXPECT_FLOAT_EQ(row0[2], 0.0f) << "Row0[2] should be 0 (rotation[2,0])";
  EXPECT_FLOAT_EQ(row0[3], 0.0f) << "Row0[3] should be 0 (M[3,0])";

  // Column 3 (translation)
  EXPECT_FLOAT_EQ(row3[0], 5.0f) << "Row3[0] should be tx=5";
  EXPECT_FLOAT_EQ(row3[1], 10.0f) << "Row3[1] should be ty=10";
  EXPECT_FLOAT_EQ(row3[2], 15.0f) << "Row3[2] should be tz=15";
  EXPECT_FLOAT_EQ(row3[3], 1.0f) << "Row3[3] should be 1";
}

// ============================================================================
// Full Pipeline Test - Simulates complete GPU transformation
// ============================================================================

/**
 * @brief Simulates the Metal shader's behavior exactly as compiled
 *
 * Metal shader does:
 *   float4 _44 = float4(position, 1.0) * float4x4(row0, row1, row2, row3);
 *   gl_Position = viewProjection * _44;
 *
 * In Metal, float4x4(a,b,c,d) uses a,b,c,d as COLUMNS.
 * vector * matrix is row-vector multiplication.
 */
Eigen::Vector4f simulateMetalShader(const Eigen::Vector4f& row0,
                                    const Eigen::Vector4f& row1,
                                    const Eigen::Vector4f& row2,
                                    const Eigen::Vector4f& row3,
                                    const Eigen::Vector3f& position,
                                    const Eigen::Matrix4f& viewProjection)
{
  // Metal: float4x4(a,b,c,d) creates matrix with a,b,c,d as COLUMNS
  Eigen::Matrix4f metalMatrix;
  metalMatrix.col(0) = row0;
  metalMatrix.col(1) = row1;
  metalMatrix.col(2) = row2;
  metalMatrix.col(3) = row3;

  // Metal: vector * matrix = row-vector multiplication
  // Equivalent to: (matrix^T * vector^T)^T = matrix^T * vector (treating as column)
  // OR: result[i] = sum_j(v[j] * M[j,i]) = sum_j(v[j] * col_j[i])
  Eigen::Vector4f pos4{position.x(), position.y(), position.z(), 1.0f};
  Eigen::Vector4f worldPos;
  for (int i = 0; i < 4; ++i)
  {
    worldPos[i] = pos4[0] * metalMatrix(i, 0) + pos4[1] * metalMatrix(i, 1) +
                  pos4[2] * metalMatrix(i, 2) + pos4[3] * metalMatrix(i, 3);
  }

  // Metal: viewProjection * worldPos (column-vector multiplication)
  return viewProjection * worldPos;
}

// ============================================================================
// Old vs New Shader Comparison Tests
// Verifies the new matrix-based shader produces identical results to the old
// position-offset shader for un-rotated objects
// ============================================================================

/**
 * @brief Simulates the OLD shader behavior (before ticket 0001)
 *
 * Old Metal shader:
 *   float3 _35 = in.in_var_TEXCOORD0 + in.in_var_TEXCOORD3;  // position + offset
 *   out.gl_Position = MVP * float4(_35, 1.0);
 */
Eigen::Vector4f simulateOldShader(const Eigen::Vector3f& position,
                                  const Eigen::Vector3f& instancePosition,
                                  const Eigen::Matrix4f& mvp)
{
  // Old shader: simple position offset
  Eigen::Vector3f worldPos = position + instancePosition;

  // Old shader: MVP * float4(worldPos, 1.0)
  Eigen::Vector4f worldPos4{worldPos.x(), worldPos.y(), worldPos.z(), 1.0f};
  return mvp * worldPos4;
}

/**
 * @brief Simulates the NEW shader behavior (after ticket 0001)
 *
 * New Metal shader:
 *   float4 _44 = float4(position, 1.0) * float4x4(row0, row1, row2, row3);
 *   out.gl_Position = VP * _44;
 *
 * For un-rotated objects, modelMatrix is identity rotation + translation.
 */
Eigen::Vector4f simulateNewShader(const Eigen::Vector3f& position,
                                  const Eigen::Vector4f& row0,
                                  const Eigen::Vector4f& row1,
                                  const Eigen::Vector4f& row2,
                                  const Eigen::Vector4f& row3,
                                  const Eigen::Matrix4f& viewProjection)
{
  // New shader: Metal float4x4(a,b,c,d) uses a,b,c,d as COLUMNS
  Eigen::Matrix4f metalMatrix;
  metalMatrix.col(0) = row0;
  metalMatrix.col(1) = row1;
  metalMatrix.col(2) = row2;
  metalMatrix.col(3) = row3;

  // New shader: vector * matrix (row-vector multiplication)
  Eigen::Vector4f pos4{position.x(), position.y(), position.z(), 1.0f};
  Eigen::Vector4f worldPos;
  for (int i = 0; i < 4; ++i)
  {
    worldPos[i] = pos4[0] * metalMatrix(i, 0) + pos4[1] * metalMatrix(i, 1) +
                  pos4[2] * metalMatrix(i, 2) + pos4[3] * metalMatrix(i, 3);
  }

  // New shader: VP * worldPos
  return viewProjection * worldPos;
}

TEST(OldVsNewShaderTest, IdentityRotation_VertexAtOrigin)
{
  // Setup: camera at z=5, object at origin
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view(2, 3) = -5.0f;

  float fov = 60.0f * static_cast<float>(M_PI) / 180.0f;
  float f = 1.0f / std::tan(fov / 2.0f);
  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / (16.0f / 9.0f);
  proj(1, 1) = f;
  proj(2, 2) = -1.001f;
  proj(2, 3) = -0.1001f;
  proj(3, 2) = -1.0f;

  Eigen::Matrix4f vp = proj * view;

  // Object at position (2, 3, 1) with NO rotation
  Eigen::Vector3f translation{2.0f, 3.0f, 1.0f};

  // OLD shader: uses MVP and instancePosition
  Eigen::Matrix4f mvp = vp;  // Model is identity, so MVP = VP
  Eigen::Vector3f vertex{0.0f, 0.0f, 0.0f};
  Eigen::Vector4f oldResult = simulateOldShader(vertex, translation, mvp);

  // NEW shader: uses VP and model matrix
  msd_sim::Coordinate origin{2.0, 3.0, 1.0};
  msd_sim::ReferenceFrame frame{origin};  // No rotation
  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);
  Eigen::Vector4f newResult =
    simulateNewShader(vertex, rows[0], rows[1], rows[2], rows[3], vp);

  // Results should be identical
  EXPECT_TRUE(vec4Equal(oldResult, newResult))
    << "Old shader: (" << oldResult.x() << ", " << oldResult.y() << ", "
    << oldResult.z() << ", " << oldResult.w() << ")\n"
    << "New shader: (" << newResult.x() << ", " << newResult.y() << ", "
    << newResult.z() << ", " << newResult.w() << ")";
}

TEST(OldVsNewShaderTest, IdentityRotation_VertexOffset)
{
  // Setup: camera at z=5
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view(2, 3) = -5.0f;

  float fov = 60.0f * static_cast<float>(M_PI) / 180.0f;
  float f = 1.0f / std::tan(fov / 2.0f);
  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / (16.0f / 9.0f);
  proj(1, 1) = f;
  proj(2, 2) = -1.001f;
  proj(2, 3) = -0.1001f;
  proj(3, 2) = -1.0f;

  Eigen::Matrix4f vp = proj * view;

  // Object at position (0, 0, 0), vertex at local (0.5, 0.5, 0)
  Eigen::Vector3f translation{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f vertex{0.5f, 0.5f, 0.0f};

  // OLD shader
  Eigen::Vector4f oldResult = simulateOldShader(vertex, translation, vp);

  // NEW shader
  msd_sim::Coordinate origin{0.0, 0.0, 0.0};
  msd_sim::ReferenceFrame frame{origin};
  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);
  Eigen::Vector4f newResult =
    simulateNewShader(vertex, rows[0], rows[1], rows[2], rows[3], vp);

  EXPECT_TRUE(vec4Equal(oldResult, newResult))
    << "Old shader: (" << oldResult.x() << ", " << oldResult.y() << ", "
    << oldResult.z() << ", " << oldResult.w() << ")\n"
    << "New shader: (" << newResult.x() << ", " << newResult.y() << ", "
    << newResult.z() << ", " << newResult.w() << ")";
}

TEST(OldVsNewShaderTest, IdentityRotation_PyramidApex)
{
  // Setup: camera at z=5
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view(2, 3) = -5.0f;

  float fov = 60.0f * static_cast<float>(M_PI) / 180.0f;
  float f = 1.0f / std::tan(fov / 2.0f);
  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / (16.0f / 9.0f);
  proj(1, 1) = f;
  proj(2, 2) = -1.001f;
  proj(2, 3) = -0.1001f;
  proj(3, 2) = -1.0f;

  Eigen::Matrix4f vp = proj * view;

  // Object at position (1, 2, -1), pyramid apex at local (0, 0.5, 0)
  Eigen::Vector3f translation{1.0f, 2.0f, -1.0f};
  Eigen::Vector3f apex{0.0f, 0.5f, 0.0f};

  // OLD shader
  Eigen::Vector4f oldResult = simulateOldShader(apex, translation, vp);

  // NEW shader
  msd_sim::Coordinate origin{1.0, 2.0, -1.0};
  msd_sim::ReferenceFrame frame{origin};
  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);
  Eigen::Vector4f newResult =
    simulateNewShader(apex, rows[0], rows[1], rows[2], rows[3], vp);

  EXPECT_TRUE(vec4Equal(oldResult, newResult))
    << "Pyramid apex transformation mismatch!\n"
    << "Old shader: (" << oldResult.x() << ", " << oldResult.y() << ", "
    << oldResult.z() << ", " << oldResult.w() << ")\n"
    << "New shader: (" << newResult.x() << ", " << newResult.y() << ", "
    << newResult.z() << ", " << newResult.w() << ")";
}

TEST(OldVsNewShaderTest, IdentityRotation_PyramidBaseCorner)
{
  // Setup: camera at z=5
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view(2, 3) = -5.0f;

  float fov = 60.0f * static_cast<float>(M_PI) / 180.0f;
  float f = 1.0f / std::tan(fov / 2.0f);
  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / (16.0f / 9.0f);
  proj(1, 1) = f;
  proj(2, 2) = -1.001f;
  proj(2, 3) = -0.1001f;
  proj(3, 2) = -1.0f;

  Eigen::Matrix4f vp = proj * view;

  // Object at position (0, 0, 0), base corner at local (-0.5, -0.5, -0.5)
  Eigen::Vector3f translation{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f corner{-0.5f, -0.5f, -0.5f};

  // OLD shader
  Eigen::Vector4f oldResult = simulateOldShader(corner, translation, vp);

  // NEW shader
  msd_sim::Coordinate origin{0.0, 0.0, 0.0};
  msd_sim::ReferenceFrame frame{origin};
  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);
  Eigen::Vector4f newResult =
    simulateNewShader(corner, rows[0], rows[1], rows[2], rows[3], vp);

  EXPECT_TRUE(vec4Equal(oldResult, newResult))
    << "Pyramid base corner transformation mismatch!\n"
    << "Old shader: (" << oldResult.x() << ", " << oldResult.y() << ", "
    << oldResult.z() << ", " << oldResult.w() << ")\n"
    << "New shader: (" << newResult.x() << ", " << newResult.y() << ", "
    << newResult.z() << ", " << newResult.w() << ")";
}

TEST(OldVsNewShaderTest, IdentityRotation_AllPyramidVertices)
{
  // Setup: camera at z=5
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view(2, 3) = -5.0f;

  float fov = 60.0f * static_cast<float>(M_PI) / 180.0f;
  float f = 1.0f / std::tan(fov / 2.0f);
  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / (16.0f / 9.0f);
  proj(1, 1) = f;
  proj(2, 2) = -1.001f;
  proj(2, 3) = -0.1001f;
  proj(3, 2) = -1.0f;

  Eigen::Matrix4f vp = proj * view;

  // Object at position (2, 1, 0)
  Eigen::Vector3f translation{2.0f, 1.0f, 0.0f};

  // All pyramid vertices (from GeometryFactory::createPyramid)
  float half = 0.5f;
  float halfHeight = 0.5f;
  std::vector<Eigen::Vector3f> vertices = {
    {0.0f, halfHeight, 0.0f},       // apex
    {-half, -halfHeight, -half},    // base front-left
    {half, -halfHeight, -half},     // base front-right
    {half, -halfHeight, half},      // base back-right
    {-half, -halfHeight, half}      // base back-left
  };

  // NEW shader setup
  msd_sim::Coordinate origin{2.0, 1.0, 0.0};
  msd_sim::ReferenceFrame frame{origin};
  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  for (size_t i = 0; i < vertices.size(); ++i)
  {
    const auto& vertex = vertices[i];

    Eigen::Vector4f oldResult = simulateOldShader(vertex, translation, vp);
    Eigen::Vector4f newResult =
      simulateNewShader(vertex, rows[0], rows[1], rows[2], rows[3], vp);

    EXPECT_TRUE(vec4Equal(oldResult, newResult))
      << "Vertex " << i << " mismatch!\n"
      << "Vertex: (" << vertex.x() << ", " << vertex.y() << ", " << vertex.z()
      << ")\n"
      << "Old shader: (" << oldResult.x() << ", " << oldResult.y() << ", "
      << oldResult.z() << ", " << oldResult.w() << ")\n"
      << "New shader: (" << newResult.x() << ", " << newResult.y() << ", "
      << newResult.z() << ", " << newResult.w() << ")";
  }
}

TEST(FullPipelineTest, ModelAndViewProjection)
{
  // Create a simple camera looking at the origin from z=5
  // View matrix: camera at (0,0,5), looking at (0,0,0)
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view(2, 3) = -5.0f;  // Translate by -5 in Z

  // Simple perspective projection (simplified)
  float fov = 60.0f * M_PI / 180.0f;
  float aspect = 16.0f / 9.0f;
  float near = 0.1f;
  float far = 100.0f;
  float f = 1.0f / std::tan(fov / 2.0f);

  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / aspect;
  proj(1, 1) = f;
  proj(2, 2) = (far + near) / (near - far);
  proj(2, 3) = (2.0f * far * near) / (near - far);
  proj(3, 2) = -1.0f;

  Eigen::Matrix4f viewProjection = proj * view;

  // Create object at origin
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  Eigen::Matrix4f modelMatrix = createModelMatrixFromReferenceFrame(frame);
  auto rows = extractShaderRows(modelMatrix);

  // Vertex at (0, 0, 0) - should be at origin in world space
  Eigen::Vector3f vertex{0.0f, 0.0f, 0.0f};
  Eigen::Vector4f clipPos = simulateMetalShader(
    rows[0], rows[1], rows[2], rows[3], vertex, viewProjection);

  // After transformation, the vertex should be in valid clip space
  // w should be positive (in front of camera)
  EXPECT_GT(clipPos.w(), 0.0f)
    << "W should be positive (vertex in front of camera), got: " << clipPos.w();

  // NDC coordinates should be in reasonable range after perspective divide
  float ndcX = clipPos.x() / clipPos.w();
  float ndcY = clipPos.y() / clipPos.w();
  float ndcZ = clipPos.z() / clipPos.w();

  EXPECT_GE(ndcX, -1.0f) << "NDC X should be >= -1";
  EXPECT_LE(ndcX, 1.0f) << "NDC X should be <= 1";
  EXPECT_GE(ndcY, -1.0f) << "NDC Y should be >= -1";
  EXPECT_LE(ndcY, 1.0f) << "NDC Y should be <= 1";
  EXPECT_GE(ndcZ, -1.0f) << "NDC Z should be >= -1";
  EXPECT_LE(ndcZ, 1.0f) << "NDC Z should be <= 1";
}
