// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#include <gtest/gtest.h>
#include <cmath>
#include <format>

#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"

using namespace msd_sim;

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(AngularRateTest, DefaultConstructorInitializesToZero)
{
  AngularRate rate{};
  EXPECT_DOUBLE_EQ(rate.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(rate.roll(), 0.0);
  EXPECT_DOUBLE_EQ(rate.yaw(), 0.0);
}

TEST(AngularRateTest, ValueConstructorStoresRates)
{
  AngularRate rate{1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(rate.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(rate.roll(), 2.0);
  EXPECT_DOUBLE_EQ(rate.yaw(), 3.0);
}

// ============================================================================
// No Normalization Tests
// ============================================================================

TEST(AngularRateTest, LargePositiveRatesStoredAsIs)
{
  AngularRate rate{100 * M_PI, 0, 0};
  // Value should NOT be normalized
  EXPECT_DOUBLE_EQ(rate.pitch(), 100 * M_PI);
}

TEST(AngularRateTest, LargeNegativeRatesStoredAsIs)
{
  AngularRate rate{-100 * M_PI, 0, 0};
  // Value should NOT be normalized
  EXPECT_DOUBLE_EQ(rate.pitch(), -100 * M_PI);
}

TEST(AngularRateTest, RatesExceedingTwoPi)
{
  double fourPi = 4.0 * M_PI;  // 720°/s
  AngularRate rate{0, 0, fourPi};
  // Value should remain 4π, not normalized to 0
  EXPECT_DOUBLE_EQ(rate.yaw(), fourPi);
}

// ============================================================================
// Mutable Accessor Tests
// ============================================================================

TEST(AngularRateTest, ModifyPitchViaReference)
{
  AngularRate rate{};
  rate.pitch() = 5.0;
  EXPECT_DOUBLE_EQ(rate.pitch(), 5.0);
}

TEST(AngularRateTest, ModifyRollViaReference)
{
  AngularRate rate{};
  rate.roll() = 10.0;
  EXPECT_DOUBLE_EQ(rate.roll(), 10.0);
}

TEST(AngularRateTest, ModifyYawViaReference)
{
  AngularRate rate{};
  rate.yaw() = 15.0;
  EXPECT_DOUBLE_EQ(rate.yaw(), 15.0);
}

// ============================================================================
// Const Accessor Tests
// ============================================================================

TEST(AngularRateTest, ConstAccessors)
{
  const AngularRate rate{1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(rate.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(rate.roll(), 2.0);
  EXPECT_DOUBLE_EQ(rate.yaw(), 3.0);
}

// ============================================================================
// Eigen Integration Tests
// ============================================================================

TEST(AngularRateTest, ConstructorFromEigenVector)
{
  msd_sim::Vector3D vec{1.0, 2.0, 3.0};
  AngularRate rate{vec};
  EXPECT_DOUBLE_EQ(rate.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(rate.roll(), 2.0);
  EXPECT_DOUBLE_EQ(rate.yaw(), 3.0);
}

TEST(AngularRateTest, EigenExpressionTemplateConstructor)
{
  msd_sim::Vector3D a{1.0, 2.0, 3.0};
  msd_sim::Vector3D b{4.0, 5.0, 6.0};
  AngularRate rate{a + b};
  EXPECT_DOUBLE_EQ(rate.pitch(), 5.0);
  EXPECT_DOUBLE_EQ(rate.roll(), 7.0);
  EXPECT_DOUBLE_EQ(rate.yaw(), 9.0);
}

TEST(AngularRateTest, EigenExpressionAssignment)
{
  AngularRate rate{};
  msd_sim::Vector3D vec{1.0, 2.0, 3.0};
  rate = vec;
  EXPECT_DOUBLE_EQ(rate.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(rate.roll(), 2.0);
  EXPECT_DOUBLE_EQ(rate.yaw(), 3.0);
}

// ============================================================================
// Eigen Operations Tests
// ============================================================================

TEST(AngularRateTest, Addition)
{
  AngularRate a{1.0, 2.0, 3.0};
  AngularRate b{4.0, 5.0, 6.0};
  AngularRate c = a + b;
  EXPECT_DOUBLE_EQ(c.pitch(), 5.0);
  EXPECT_DOUBLE_EQ(c.roll(), 7.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 9.0);
}

TEST(AngularRateTest, Subtraction)
{
  AngularRate a{4.0, 5.0, 6.0};
  AngularRate b{1.0, 2.0, 3.0};
  AngularRate c = a - b;
  EXPECT_DOUBLE_EQ(c.pitch(), 3.0);
  EXPECT_DOUBLE_EQ(c.roll(), 3.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 3.0);
}

TEST(AngularRateTest, ScalarMultiplication)
{
  AngularRate a{1.0, 2.0, 3.0};
  AngularRate c = a * 2.0;
  EXPECT_DOUBLE_EQ(c.pitch(), 2.0);
  EXPECT_DOUBLE_EQ(c.roll(), 4.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 6.0);
}

TEST(AngularRateTest, CrossProduct)
{
  AngularRate a{1.0, 0.0, 0.0};
  AngularRate b{0.0, 1.0, 0.0};
  AngularRate c = a.cross(b);
  EXPECT_NEAR(c[0], 0.0, 1e-10);
  EXPECT_NEAR(c[1], 0.0, 1e-10);
  EXPECT_NEAR(c[2], 1.0, 1e-10);
}

TEST(AngularRateTest, DotProduct)
{
  AngularRate a{1.0, 2.0, 3.0};
  AngularRate b{4.0, 5.0, 6.0};
  double dotProd = a.dot(b);
  EXPECT_DOUBLE_EQ(dotProd, 32.0);
}

TEST(AngularRateTest, Norm)
{
  AngularRate a{3.0, 4.0, 0.0};
  double magnitude = a.norm();
  EXPECT_DOUBLE_EQ(magnitude, 5.0);
}

// ============================================================================
// Rule of Zero Tests
// ============================================================================

TEST(AngularRateTest, CopyConstructor)
{
  AngularRate a{1.0, 2.0, 3.0};
  AngularRate b{a};
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularRateTest, MoveConstructor)
{
  AngularRate a{1.0, 2.0, 3.0};
  AngularRate b{std::move(a)};
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularRateTest, CopyAssignment)
{
  AngularRate a{1.0, 2.0, 3.0};
  AngularRate b{};
  b = a;
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularRateTest, MoveAssignment)
{
  AngularRate a{1.0, 2.0, 3.0};
  AngularRate b{};
  b = std::move(a);
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

// ============================================================================
// std::format Support Tests
// ============================================================================

TEST(AngularRateTest, FormatDefault)
{
  AngularRate rate{1.5, 2.5, 3.5};
  std::string formatted = std::format("{}", rate);
  EXPECT_TRUE(formatted.find("1.5") != std::string::npos);
  EXPECT_TRUE(formatted.find("2.5") != std::string::npos);
  EXPECT_TRUE(formatted.find("3.5") != std::string::npos);
}

TEST(AngularRateTest, FormatCustomPrecision)
{
  AngularRate rate{1.5, 2.5, 3.5};
  std::string formatted = std::format("{:.2f}", rate);
  EXPECT_TRUE(formatted.find("1.50") != std::string::npos);
  EXPECT_TRUE(formatted.find("2.50") != std::string::npos);
  EXPECT_TRUE(formatted.find("3.50") != std::string::npos);
}

// ============================================================================
// Physics Integration Tests
// ============================================================================

TEST(AngularRateTest, AngularVelocityIntegration)
{
  AngularRate angularVelocity{0.0, 0.0, 2.0 * M_PI};  // 1 revolution per second
  double deltaTime = 0.5;                             // 0.5 seconds
  AngularRate deltaOrientation = angularVelocity * deltaTime;
  // Half revolution = π radians
  EXPECT_DOUBLE_EQ(deltaOrientation.yaw(), M_PI);
}

TEST(AngularRateTest, TorqueCalculation)
{
  // r × F for torque calculation
  msd_sim::Vector3D r{1.0, 0.0, 0.0};   // Moment arm
  msd_sim::Vector3D F{0.0, 10.0, 0.0};  // Force
  AngularRate torque{r.cross(F)};
  // Torque = r × F = (1, 0, 0) × (0, 10, 0) = (0, 0, 10)
  EXPECT_DOUBLE_EQ(torque.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(torque.roll(), 0.0);
  EXPECT_DOUBLE_EQ(torque.yaw(), 10.0);
}

// ============================================================================
// Memory Size Test
// ============================================================================

TEST(AngularRateTest, MemoryFootprint)
{
  // Should be same size as msd_sim::Vector3D (24 bytes)
  EXPECT_EQ(sizeof(AngularRate), sizeof(msd_sim::Vector3D));
  EXPECT_EQ(sizeof(AngularRate), 24);
}
