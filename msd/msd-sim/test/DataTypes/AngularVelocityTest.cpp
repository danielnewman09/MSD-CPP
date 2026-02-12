// Ticket: 0024_angular_coordinate

#include <gtest/gtest.h>
#include <cmath>
#include <format>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"

using namespace msd_sim;

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(AngularVelocityTest, DefaultConstructorInitializesToZero)
{
  AngularVelocity vel{};
  EXPECT_DOUBLE_EQ(vel.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(vel.roll(), 0.0);
  EXPECT_DOUBLE_EQ(vel.yaw(), 0.0);
}

TEST(AngularVelocityTest, ValueConstructorStoresRates)
{
  AngularVelocity vel{1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(vel.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(vel.roll(), 2.0);
  EXPECT_DOUBLE_EQ(vel.yaw(), 3.0);
}

// ============================================================================
// No Normalization Tests
// ============================================================================

TEST(AngularVelocityTest, LargePositiveRatesStoredAsIs)
{
  AngularVelocity vel{100 * M_PI, 0, 0};
  // Value should NOT be normalized
  EXPECT_DOUBLE_EQ(vel.pitch(), 100 * M_PI);
}

TEST(AngularVelocityTest, LargeNegativeRatesStoredAsIs)
{
  AngularVelocity vel{-100 * M_PI, 0, 0};
  // Value should NOT be normalized
  EXPECT_DOUBLE_EQ(vel.pitch(), -100 * M_PI);
}

TEST(AngularVelocityTest, RatesExceedingTwoPi)
{
  double fourPi = 4.0 * M_PI;  // 720 deg/s
  AngularVelocity vel{0, 0, fourPi};
  // Value should remain 4pi, not normalized to 0
  EXPECT_DOUBLE_EQ(vel.yaw(), fourPi);
}

// ============================================================================
// Mutable Accessor Tests
// ============================================================================

TEST(AngularVelocityTest, ModifyPitchViaReference)
{
  AngularVelocity vel{};
  vel.pitch() = 5.0;
  EXPECT_DOUBLE_EQ(vel.pitch(), 5.0);
}

TEST(AngularVelocityTest, ModifyRollViaReference)
{
  AngularVelocity vel{};
  vel.roll() = 10.0;
  EXPECT_DOUBLE_EQ(vel.roll(), 10.0);
}

TEST(AngularVelocityTest, ModifyYawViaReference)
{
  AngularVelocity vel{};
  vel.yaw() = 15.0;
  EXPECT_DOUBLE_EQ(vel.yaw(), 15.0);
}

// ============================================================================
// Const Accessor Tests
// ============================================================================

TEST(AngularVelocityTest, ConstAccessors)
{
  const AngularVelocity vel{1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(vel.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(vel.roll(), 2.0);
  EXPECT_DOUBLE_EQ(vel.yaw(), 3.0);
}

// ============================================================================
// Eigen Integration Tests
// ============================================================================

TEST(AngularVelocityTest, ConstructorFromEigenVector)
{
  msd_sim::Vector3D vec{1.0, 2.0, 3.0};
  AngularVelocity vel{vec};
  EXPECT_DOUBLE_EQ(vel.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(vel.roll(), 2.0);
  EXPECT_DOUBLE_EQ(vel.yaw(), 3.0);
}

TEST(AngularVelocityTest, EigenExpressionTemplateConstructor)
{
  msd_sim::Vector3D a{1.0, 2.0, 3.0};
  msd_sim::Vector3D b{4.0, 5.0, 6.0};
  AngularVelocity vel{a + b};
  EXPECT_DOUBLE_EQ(vel.pitch(), 5.0);
  EXPECT_DOUBLE_EQ(vel.roll(), 7.0);
  EXPECT_DOUBLE_EQ(vel.yaw(), 9.0);
}

TEST(AngularVelocityTest, EigenExpressionAssignment)
{
  AngularVelocity vel{};
  msd_sim::Vector3D vec{1.0, 2.0, 3.0};
  vel = vec;
  EXPECT_DOUBLE_EQ(vel.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(vel.roll(), 2.0);
  EXPECT_DOUBLE_EQ(vel.yaw(), 3.0);
}

// ============================================================================
// Eigen Operations Tests
// ============================================================================

TEST(AngularVelocityTest, Addition)
{
  AngularVelocity a{1.0, 2.0, 3.0};
  AngularVelocity b{4.0, 5.0, 6.0};
  AngularVelocity c = a + b;
  EXPECT_DOUBLE_EQ(c.pitch(), 5.0);
  EXPECT_DOUBLE_EQ(c.roll(), 7.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 9.0);
}

TEST(AngularVelocityTest, Subtraction)
{
  AngularVelocity a{4.0, 5.0, 6.0};
  AngularVelocity b{1.0, 2.0, 3.0};
  AngularVelocity c = a - b;
  EXPECT_DOUBLE_EQ(c.pitch(), 3.0);
  EXPECT_DOUBLE_EQ(c.roll(), 3.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 3.0);
}

TEST(AngularVelocityTest, ScalarMultiplication)
{
  AngularVelocity a{1.0, 2.0, 3.0};
  AngularVelocity c = a * 2.0;
  EXPECT_DOUBLE_EQ(c.pitch(), 2.0);
  EXPECT_DOUBLE_EQ(c.roll(), 4.0);
  EXPECT_DOUBLE_EQ(c.yaw(), 6.0);
}

TEST(AngularVelocityTest, CrossProduct)
{
  AngularVelocity a{1.0, 0.0, 0.0};
  AngularVelocity b{0.0, 1.0, 0.0};
  AngularVelocity c = a.cross(b);
  EXPECT_NEAR(c[0], 0.0, 1e-10);
  EXPECT_NEAR(c[1], 0.0, 1e-10);
  EXPECT_NEAR(c[2], 1.0, 1e-10);
}

TEST(AngularVelocityTest, DotProduct)
{
  AngularVelocity a{1.0, 2.0, 3.0};
  AngularVelocity b{4.0, 5.0, 6.0};
  double dotProd = a.dot(b);
  EXPECT_DOUBLE_EQ(dotProd, 32.0);
}

TEST(AngularVelocityTest, Norm)
{
  AngularVelocity a{3.0, 4.0, 0.0};
  double magnitude = a.norm();
  EXPECT_DOUBLE_EQ(magnitude, 5.0);
}

// ============================================================================
// Rule of Five Tests
// ============================================================================

TEST(AngularVelocityTest, CopyConstructor)
{
  AngularVelocity a{1.0, 2.0, 3.0};
  AngularVelocity b{a};
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularVelocityTest, MoveConstructor)
{
  AngularVelocity a{1.0, 2.0, 3.0};
  AngularVelocity b{std::move(a)};
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularVelocityTest, CopyAssignment)
{
  AngularVelocity a{1.0, 2.0, 3.0};
  AngularVelocity b{};
  b = a;
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

TEST(AngularVelocityTest, MoveAssignment)
{
  AngularVelocity a{1.0, 2.0, 3.0};
  AngularVelocity b{};
  b = std::move(a);
  EXPECT_DOUBLE_EQ(b.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(b.roll(), 2.0);
  EXPECT_DOUBLE_EQ(b.yaw(), 3.0);
}

// ============================================================================
// std::format Support Tests
// ============================================================================

TEST(AngularVelocityTest, FormatDefault)
{
  AngularVelocity vel{1.5, 2.5, 3.5};
  std::string formatted = std::format("{}", vel);
  EXPECT_TRUE(formatted.find("1.5") != std::string::npos);
  EXPECT_TRUE(formatted.find("2.5") != std::string::npos);
  EXPECT_TRUE(formatted.find("3.5") != std::string::npos);
}

TEST(AngularVelocityTest, FormatCustomPrecision)
{
  AngularVelocity vel{1.5, 2.5, 3.5};
  std::string formatted = std::format("{:.2f}", vel);
  EXPECT_TRUE(formatted.find("1.50") != std::string::npos);
  EXPECT_TRUE(formatted.find("2.50") != std::string::npos);
  EXPECT_TRUE(formatted.find("3.50") != std::string::npos);
}

// ============================================================================
// Physics Integration Tests
// ============================================================================

TEST(AngularVelocityTest, AngularVelocityIntegration)
{
  AngularVelocity angularVelocity{0.0, 0.0, 2.0 * M_PI};  // 1 rev per second
  double deltaTime = 0.5;                                   // 0.5 seconds
  AngularVelocity deltaOrientation = angularVelocity * deltaTime;
  // Half revolution = pi radians
  EXPECT_DOUBLE_EQ(deltaOrientation.yaw(), M_PI);
}

TEST(AngularVelocityTest, TorqueCalculation)
{
  // r x F for torque calculation
  msd_sim::Vector3D r{1.0, 0.0, 0.0};   // Moment arm
  msd_sim::Vector3D F{0.0, 10.0, 0.0};  // Force
  AngularVelocity torque{r.cross(F)};
  // Torque = r x F = (1, 0, 0) x (0, 10, 0) = (0, 0, 10)
  EXPECT_DOUBLE_EQ(torque.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(torque.roll(), 0.0);
  EXPECT_DOUBLE_EQ(torque.yaw(), 10.0);
}

// ============================================================================
// Memory Size Test
// ============================================================================

TEST(AngularVelocityTest, MemoryFootprint)
{
  // Should be same size as Eigen::Vector3d (24 bytes)
  EXPECT_EQ(sizeof(AngularVelocity), sizeof(Eigen::Vector3d));
  EXPECT_EQ(sizeof(AngularVelocity), 24);
}
