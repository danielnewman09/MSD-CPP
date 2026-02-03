// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#include <gtest/gtest.h>

#include <cmath>
#include <format>

#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"

using namespace msd_sim;

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(AngularCoordinateTest, DefaultConstructorInitializesToZero)
{
  AngularCoordinate angular{};
  EXPECT_DOUBLE_EQ(angular.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(angular.roll(), 0.0);
  EXPECT_DOUBLE_EQ(angular.yaw(), 0.0);
}

TEST(AngularCoordinateTest, ValueConstructorStoresPitchRollYaw)
{
  AngularCoordinate angular{M_PI / 4, M_PI / 6, M_PI / 3};
  EXPECT_DOUBLE_EQ(angular.pitch(), M_PI / 4);
  EXPECT_DOUBLE_EQ(angular.roll(), M_PI / 6);
  EXPECT_DOUBLE_EQ(angular.yaw(), M_PI / 3);
}

// ============================================================================
// Deferred Normalization Tests
// ============================================================================

TEST(AngularCoordinateTest, SmallValuesBelowThresholdNotNormalized)
{
  // 3π is below 100π threshold
  AngularCoordinate angular{3 * M_PI, 0, 0};
  // Value is stored as-is, not normalized
  EXPECT_DOUBLE_EQ(angular.pitch(), 3 * M_PI);
}

TEST(AngularCoordinateTest, ValuesExceedingThresholdAreNormalized)
{
  // 101π exceeds 100π threshold
  AngularCoordinate angular{101 * M_PI, 0, 0};
  // Should be normalized to π
  EXPECT_NEAR(angular.pitch(), M_PI, 1e-10);
}

TEST(AngularCoordinateTest, NegativeValuesExceedingThresholdAreNormalized)
{
  // -101π exceeds -100π threshold
  AngularCoordinate angular{-101 * M_PI, 0, 0};
  // Normalizes to (-π, π] range, so -101π → +π (since -π is excluded from
  // range)
  EXPECT_NEAR(angular.pitch(), M_PI, 1e-10);
}

// ============================================================================
// Explicit Normalization Tests
// ============================================================================

TEST(AngularCoordinateTest, NormalizeThreePIToPI)
{
  AngularCoordinate angular{3 * M_PI, 0, 0};
  angular.normalize();
  EXPECT_NEAR(angular.pitch(), M_PI, 1e-6);
}

TEST(AngularCoordinateTest, NormalizeNegativeThreePIToPI)
{
  AngularCoordinate angular{-3 * M_PI, 0, 0};
  angular.normalize();
  // Normalizes to (-π, π] range, so -3π → +π (since -π is excluded from range)
  EXPECT_NEAR(angular.pitch(), M_PI, 1e-6);
}

TEST(AngularCoordinateTest, NormalizeFivePIOverTwoToPIOverTwo)
{
  AngularCoordinate angular{5 * M_PI / 2, 0, 0};
  angular.normalize();
  EXPECT_NEAR(angular.pitch(), M_PI / 2, 1e-6);
}

TEST(AngularCoordinateTest, NormalizedReturnsNewNormalizedInstance)
{
  AngularCoordinate original{3 * M_PI, 0, 0};
  AngularCoordinate normalized = original.normalized();

  // Original is unchanged
  EXPECT_DOUBLE_EQ(original.pitch(), 3 * M_PI);
  // Normalized copy has normalized value
  EXPECT_NEAR(normalized.pitch(), M_PI, 1e-6);
}

// ============================================================================
// Degree Accessor Tests
// ============================================================================

TEST(AngularCoordinateTest, DegreeAccessors)
{
  AngularCoordinate angular{M_PI / 2, M_PI / 4, M_PI / 6};
  EXPECT_NEAR(angular.pitchDeg(), 90.0, 1e-6);
  EXPECT_NEAR(angular.rollDeg(), 45.0, 1e-6);
  EXPECT_NEAR(angular.yawDeg(), 30.0, 1e-6);
}

// ============================================================================
// Setter Tests
// ============================================================================

TEST(AngularCoordinateTest, SetterSmallValueBelowThreshold)
{
  AngularCoordinate angular{};
  angular.setPitch(M_PI / 4);
  EXPECT_DOUBLE_EQ(angular.pitch(), M_PI / 4);
}

TEST(AngularCoordinateTest, SetterLargeValueExceedingThreshold)
{
  AngularCoordinate angular{};
  angular.setPitch(101 * M_PI);
  EXPECT_NEAR(angular.pitch(), M_PI, 1e-6);
}

// ============================================================================
// Eigen Integration Tests
// ============================================================================

TEST(AngularCoordinateTest, EigenExpressionTemplateConstructor)
{
  Eigen::Vector3d vec{M_PI / 4, M_PI / 6, M_PI / 3};
  AngularCoordinate angular{vec};
  EXPECT_DOUBLE_EQ(angular.pitch(), M_PI / 4);
  EXPECT_DOUBLE_EQ(angular.roll(), M_PI / 6);
  EXPECT_DOUBLE_EQ(angular.yaw(), M_PI / 3);
}

TEST(AngularCoordinateTest, EigenExpressionAssignment)
{
  AngularCoordinate angular{};
  Eigen::Vector3d vec{M_PI / 4, M_PI / 6, M_PI / 3};
  angular = vec;
  EXPECT_DOUBLE_EQ(angular.pitch(), M_PI / 4);
  EXPECT_DOUBLE_EQ(angular.roll(), M_PI / 6);
  EXPECT_DOUBLE_EQ(angular.yaw(), M_PI / 3);
}

// ============================================================================
// Eigen Operations Tests
// ============================================================================

TEST(AngularCoordinateTest, Addition)
{
  AngularCoordinate a{M_PI / 4, 0, 0};
  AngularCoordinate b{M_PI / 4, 0, 0};
  AngularCoordinate c = a + b;
  EXPECT_DOUBLE_EQ(c.pitch(), M_PI / 2);
}

TEST(AngularCoordinateTest, Subtraction)
{
  AngularCoordinate a{M_PI / 2, 0, 0};
  AngularCoordinate b{M_PI / 4, 0, 0};
  AngularCoordinate c = a - b;
  EXPECT_DOUBLE_EQ(c.pitch(), M_PI / 4);
}

TEST(AngularCoordinateTest, ScalarMultiplication)
{
  AngularCoordinate a{M_PI / 4, 0, 0};
  AngularCoordinate c = a * 2.0;
  EXPECT_DOUBLE_EQ(c.pitch(), M_PI / 2);
}

TEST(AngularCoordinateTest, CrossProduct)
{
  AngularCoordinate a{1.0, 0.0, 0.0};
  AngularCoordinate b{0.0, 1.0, 0.0};
  AngularCoordinate c = a.cross(b);
  EXPECT_NEAR(c[2], 1.0, 1e-10);
}

TEST(AngularCoordinateTest, DotProduct)
{
  AngularCoordinate a{1.0, 2.0, 3.0};
  AngularCoordinate b{4.0, 5.0, 6.0};
  double dotProd = a.dot(b);
  EXPECT_DOUBLE_EQ(dotProd, 32.0);
}

// ============================================================================
// Compound Operator Tests
// ============================================================================

TEST(AngularCoordinateTest, CompoundAddBelowThreshold)
{
  AngularCoordinate a{M_PI / 4, 0, 0};
  AngularCoordinate b{M_PI / 4, 0, 0};
  a += b;
  EXPECT_DOUBLE_EQ(a.pitch(), M_PI / 2);
}

TEST(AngularCoordinateTest, CompoundAddExceedingThreshold)
{
  AngularCoordinate a{50 * M_PI, 0, 0};
  AngularCoordinate b{52 * M_PI, 0, 0};
  a += b;
  // 102π should normalize to 0
  EXPECT_NEAR(a.pitch(), 0.0, 1e-6);
}

TEST(AngularCoordinateTest, CompoundSubtract)
{
  AngularCoordinate a{M_PI / 2, 0, 0};
  AngularCoordinate b{M_PI / 4, 0, 0};
  a -= b;
  EXPECT_DOUBLE_EQ(a.pitch(), M_PI / 4);
}

TEST(AngularCoordinateTest, CompoundMultiply)
{
  AngularCoordinate a{M_PI / 4, 0, 0};
  a *= 2.0;
  EXPECT_DOUBLE_EQ(a.pitch(), M_PI / 2);
}

TEST(AngularCoordinateTest, CompoundDivide)
{
  AngularCoordinate a{M_PI / 2, 0, 0};
  a /= 2.0;
  EXPECT_DOUBLE_EQ(a.pitch(), M_PI / 4);
}

// ============================================================================
// Rule of Zero Tests
// ============================================================================

TEST(AngularCoordinateTest, CopyConstructor)
{
  AngularCoordinate a{M_PI / 4, M_PI / 6, M_PI / 3};
  AngularCoordinate b{a};
  EXPECT_DOUBLE_EQ(b.pitch(), M_PI / 4);
  EXPECT_DOUBLE_EQ(b.roll(), M_PI / 6);
  EXPECT_DOUBLE_EQ(b.yaw(), M_PI / 3);
}

TEST(AngularCoordinateTest, MoveConstructor)
{
  AngularCoordinate a{M_PI / 4, M_PI / 6, M_PI / 3};
  AngularCoordinate b{std::move(a)};
  EXPECT_DOUBLE_EQ(b.pitch(), M_PI / 4);
  EXPECT_DOUBLE_EQ(b.roll(), M_PI / 6);
  EXPECT_DOUBLE_EQ(b.yaw(), M_PI / 3);
}

TEST(AngularCoordinateTest, CopyAssignment)
{
  AngularCoordinate a{M_PI / 4, M_PI / 6, M_PI / 3};
  AngularCoordinate b{};
  b = a;
  EXPECT_DOUBLE_EQ(b.pitch(), M_PI / 4);
  EXPECT_DOUBLE_EQ(b.roll(), M_PI / 6);
  EXPECT_DOUBLE_EQ(b.yaw(), M_PI / 3);
}

TEST(AngularCoordinateTest, MoveAssignment)
{
  AngularCoordinate a{M_PI / 4, M_PI / 6, M_PI / 3};
  AngularCoordinate b{};
  b = std::move(a);
  EXPECT_DOUBLE_EQ(b.pitch(), M_PI / 4);
  EXPECT_DOUBLE_EQ(b.roll(), M_PI / 6);
  EXPECT_DOUBLE_EQ(b.yaw(), M_PI / 3);
}

// ============================================================================
// std::format Support Tests
// ============================================================================

TEST(AngularCoordinateTest, FormatDefault)
{
  AngularCoordinate angular{0.523599, 0.785398, 1.047198};
  std::string formatted = std::format("{}", angular);
  EXPECT_TRUE(formatted.find("0.523599") != std::string::npos);
  EXPECT_TRUE(formatted.find("0.785398") != std::string::npos);
  EXPECT_TRUE(formatted.find("1.047198") != std::string::npos);
}

TEST(AngularCoordinateTest, FormatCustomPrecision)
{
  AngularCoordinate angular{0.523599, 0.785398, 1.047198};
  std::string formatted = std::format("{:.2f}", angular);
  EXPECT_TRUE(formatted.find("0.52") != std::string::npos);
  EXPECT_TRUE(formatted.find("0.79") != std::string::npos);
  EXPECT_TRUE(formatted.find("1.05") != std::string::npos);
}

// ============================================================================
// Memory Size Test
// ============================================================================

TEST(AngularCoordinateTest, MemoryFootprint)
{
  // Should be same size as Eigen::Vector3d (24 bytes)
  EXPECT_EQ(sizeof(AngularCoordinate), sizeof(Eigen::Vector3d));
  EXPECT_EQ(sizeof(AngularCoordinate), 24);
}
