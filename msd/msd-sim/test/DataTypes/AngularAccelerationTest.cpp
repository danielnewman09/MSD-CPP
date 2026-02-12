// Ticket: 0024_angular_coordinate

#include <gtest/gtest.h>
#include <cmath>
#include <format>

#include "msd-sim/src/DataTypes/AngularAcceleration.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"

using namespace msd_sim;

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(AngularAccelerationTest, DefaultConstructorInitializesToZero)
{
  AngularAcceleration accel{};
  EXPECT_DOUBLE_EQ(accel.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(accel.roll(), 0.0);
  EXPECT_DOUBLE_EQ(accel.yaw(), 0.0);
}

TEST(AngularAccelerationTest, ValueConstructorStoresRates)
{
  AngularAcceleration accel{1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(accel.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(accel.roll(), 2.0);
  EXPECT_DOUBLE_EQ(accel.yaw(), 3.0);
}

// ============================================================================
// No Normalization Tests
// ============================================================================

TEST(AngularAccelerationTest, LargePositiveValuesStoredAsIs)
{
  AngularAcceleration accel{100 * M_PI, 0, 0};
  EXPECT_DOUBLE_EQ(accel.pitch(), 100 * M_PI);
}

TEST(AngularAccelerationTest, LargeNegativeValuesStoredAsIs)
{
  AngularAcceleration accel{-100 * M_PI, 0, 0};
  EXPECT_DOUBLE_EQ(accel.pitch(), -100 * M_PI);
}

TEST(AngularAccelerationTest, ValuesExceedingTwoPi)
{
  double fourPi = 4.0 * M_PI;
  AngularAcceleration accel{0, 0, fourPi};
  EXPECT_DOUBLE_EQ(accel.yaw(), fourPi);
}

// ============================================================================
// Mutable Accessor Tests
// ============================================================================

TEST(AngularAccelerationTest, ModifyPitchViaReference)
{
  AngularAcceleration accel{};
  accel.pitch() = 5.0;
  EXPECT_DOUBLE_EQ(accel.pitch(), 5.0);
}

TEST(AngularAccelerationTest, ModifyRollViaReference)
{
  AngularAcceleration accel{};
  accel.roll() = 10.0;
  EXPECT_DOUBLE_EQ(accel.roll(), 10.0);
}

TEST(AngularAccelerationTest, ModifyYawViaReference)
{
  AngularAcceleration accel{};
  accel.yaw() = 15.0;
  EXPECT_DOUBLE_EQ(accel.yaw(), 15.0);
}

// ============================================================================
// Const Accessor Tests
// ============================================================================

TEST(AngularAccelerationTest, ConstAccessors)
{
  const AngularAcceleration accel{1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(accel.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(accel.roll(), 2.0);
  EXPECT_DOUBLE_EQ(accel.yaw(), 3.0);
}

// ============================================================================
// Eigen Integration Tests
// ============================================================================

TEST(AngularAccelerationTest, ConstructorFromEigenVector)
{
  msd_sim::Vector3D vec{1.0, 2.0, 3.0};
  AngularAcceleration accel{vec};
  EXPECT_DOUBLE_EQ(accel.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(accel.roll(), 2.0);
  EXPECT_DOUBLE_EQ(accel.yaw(), 3.0);
}

TEST(AngularAccelerationTest, EigenExpressionTemplateConstructor)
{
  msd_sim::Vector3D a{1.0, 2.0, 3.0};
  msd_sim::Vector3D b{4.0, 5.0, 6.0};
  AngularAcceleration accel{a + b};
  EXPECT_DOUBLE_EQ(accel.pitch(), 5.0);
  EXPECT_DOUBLE_EQ(accel.roll(), 7.0);
  EXPECT_DOUBLE_EQ(accel.yaw(), 9.0);
}

TEST(AngularAccelerationTest, EigenExpressionAssignment)
{
  AngularAcceleration accel{};
  msd_sim::Vector3D vec{1.0, 2.0, 3.0};
  accel = vec;
  EXPECT_DOUBLE_EQ(accel.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(accel.roll(), 2.0);
  EXPECT_DOUBLE_EQ(accel.yaw(), 3.0);
}

// ============================================================================
// Eigen Operations Tests
// ============================================================================

TEST(AngularAccelerationTest, Addition)
{
  AngularAcceleration a{1.0, 2.0, 3.0};
  AngularAcceleration b{4.0, 5.0, 6.0};
  AngularAcceleration c = a + b;
  EXPECT_DOUBLE_EQ(c.pitch(), 5.0);
  EXPECT_DOUBLE_EQ(c.roll(), 7.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 9.0);
}

TEST(AngularAccelerationTest, Subtraction)
{
  AngularAcceleration a{4.0, 5.0, 6.0};
  AngularAcceleration b{1.0, 2.0, 3.0};
  AngularAcceleration c = a - b;
  EXPECT_DOUBLE_EQ(c.pitch(), 3.0);
  EXPECT_DOUBLE_EQ(c.roll(), 3.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 3.0);
}

TEST(AngularAccelerationTest, ScalarMultiplication)
{
  AngularAcceleration a{1.0, 2.0, 3.0};
  AngularAcceleration c = a * 2.0;
  EXPECT_DOUBLE_EQ(c.pitch(), 2.0);
  EXPECT_DOUBLE_EQ(c.roll(), 4.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 6.0);
}

TEST(AngularAccelerationTest, CrossProduct)
{
  AngularAcceleration a{1.0, 0.0, 0.0};
  AngularAcceleration b{0.0, 1.0, 0.0};
  AngularAcceleration c = a.cross(b);
  EXPECT_NEAR(c[0], 0.0, 1e-10);
  EXPECT_NEAR(c[1], 0.0, 1e-10);
  EXPECT_NEAR(c[2], 1.0, 1e-10);
}

TEST(AngularAccelerationTest, DotProduct)
{
  AngularAcceleration a{1.0, 2.0, 3.0};
  AngularAcceleration b{4.0, 5.0, 6.0};
  double dotProd = a.dot(b);
  EXPECT_DOUBLE_EQ(dotProd, 32.0);
}

TEST(AngularAccelerationTest, Norm)
{
  AngularAcceleration a{3.0, 4.0, 0.0};
  double magnitude = a.norm();
  EXPECT_DOUBLE_EQ(magnitude, 5.0);
}

// ============================================================================
// Rule of Five Tests
// ============================================================================

TEST(AngularAccelerationTest, CopyConstructor)
{
  AngularAcceleration a{1.0, 2.0, 3.0};
  AngularAcceleration b{a};
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularAccelerationTest, MoveConstructor)
{
  AngularAcceleration a{1.0, 2.0, 3.0};
  AngularAcceleration b{std::move(a)};
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularAccelerationTest, CopyAssignment)
{
  AngularAcceleration a{1.0, 2.0, 3.0};
  AngularAcceleration b{};
  b = a;
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularAccelerationTest, MoveAssignment)
{
  AngularAcceleration a{1.0, 2.0, 3.0};
  AngularAcceleration b{};
  b = std::move(a);
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

// ============================================================================
// std::format Support Tests
// ============================================================================

TEST(AngularAccelerationTest, FormatDefault)
{
  AngularAcceleration accel{1.5, 2.5, 3.5};
  std::string formatted = std::format("{}", accel);
  EXPECT_TRUE(formatted.find("1.5") != std::string::npos);
  EXPECT_TRUE(formatted.find("2.5") != std::string::npos);
  EXPECT_TRUE(formatted.find("3.5") != std::string::npos);
}

TEST(AngularAccelerationTest, FormatCustomPrecision)
{
  AngularAcceleration accel{1.5, 2.5, 3.5};
  std::string formatted = std::format("{:.2f}", accel);
  EXPECT_TRUE(formatted.find("1.50") != std::string::npos);
  EXPECT_TRUE(formatted.find("2.50") != std::string::npos);
  EXPECT_TRUE(formatted.find("3.50") != std::string::npos);
}

// ============================================================================
// Memory Size Test
// ============================================================================

TEST(AngularAccelerationTest, MemoryFootprint)
{
  // Should be same size as Eigen::Vector3d (24 bytes)
  EXPECT_EQ(sizeof(AngularAcceleration), sizeof(Eigen::Vector3d));
  EXPECT_EQ(sizeof(AngularAcceleration), 24);
}
