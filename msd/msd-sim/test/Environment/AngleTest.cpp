#include <gtest/gtest.h>
#include <cmath>
#include "msd-sim/src/Environment/Angle.hpp"

using namespace msd_sim;

// Helper function for comparing doubles with tolerance
constexpr double TOLERANCE = 1e-10;

bool almostEqual(double a, double b, double tolerance = TOLERANCE)
{
  return std::abs(a - b) < tolerance;
}

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(AngleTest, DefaultConstructor)
{
  Angle angle;
  EXPECT_DOUBLE_EQ(angle.getRad(), 0.0);
  EXPECT_EQ(angle.getNormalization(), Angle::Norm::PI);
}

TEST(AngleTest, ConstructorWithRadians)
{
  Angle angle(M_PI / 4, Angle::Norm::PI);
  EXPECT_DOUBLE_EQ(angle.getRad(), M_PI / 4);
  EXPECT_EQ(angle.getNormalization(), Angle::Norm::PI);
}

TEST(AngleTest, ConstructorWithNormalizationPI)
{
  // Test angle that needs normalization to (-pi, pi]
  Angle angle(5.0, Angle::Norm::PI);
  double expected = 5.0 - 2.0 * M_PI;  // Should wrap to (-pi, pi]
  EXPECT_TRUE(almostEqual(angle.getRad(), expected));
}

TEST(AngleTest, ConstructorWithNormalizationTwoPI)
{
  // Test angle that needs normalization to [0, 2pi)
  Angle angle(-M_PI / 2, Angle::Norm::TWO_PI);
  double expected = -M_PI / 2 + 2.0 * M_PI;  // Should wrap to [0, 2pi)
  EXPECT_TRUE(almostEqual(angle.getRad(), expected));
}

TEST(AngleTest, FromRadiansFactory)
{
  Angle angle = Angle::fromRadians(M_PI / 3, Angle::Norm::PI);
  EXPECT_DOUBLE_EQ(angle.getRad(), M_PI / 3);
}

TEST(AngleTest, FromDegreesFactory)
{
  Angle angle = Angle::fromDegrees(90.0, Angle::Norm::PI);
  EXPECT_TRUE(almostEqual(angle.getRad(), M_PI / 2));
}

// ============================================================================
// Conversion Tests
// ============================================================================

TEST(AngleTest, ToRadians)
{
  Angle angle = Angle::fromRadians(M_PI / 4);
  EXPECT_DOUBLE_EQ(angle.getRad(), M_PI / 4);
}

TEST(AngleTest, ToDegrees)
{
  Angle angle = Angle::fromRadians(M_PI);
  EXPECT_TRUE(almostEqual(angle.toDeg(), 180.0)) << angle.toDeg();
}

TEST(AngleTest, DegreesRoundTrip)
{
  double degInput = 45.0;
  Angle angle = Angle::fromDegrees(degInput);
  EXPECT_TRUE(almostEqual(angle.toDeg(), degInput));
}

TEST(AngleTest, RadiansRoundTrip)
{
  double radInput = M_PI / 6;
  Angle angle = Angle::fromRadians(radInput);
  EXPECT_DOUBLE_EQ(angle.getRad(), radInput);
}

// ============================================================================
// Normalization Tests
// ============================================================================

TEST(AngleTest, NormalizationPIPositive)
{
  // Test that angles > pi wrap correctly
  Angle angle(4.0, Angle::Norm::PI);
  double normalized = angle.getRad();
  EXPECT_TRUE(normalized > -M_PI);
  EXPECT_TRUE(normalized <= M_PI);
}

TEST(AngleTest, NormalizationPINegative)
{
  // Test that angles < -pi wrap correctly
  Angle angle{-5.0, Angle::Norm::PI};
  double normalized = angle.getRad();
  EXPECT_TRUE(normalized > -M_PI) << normalized;
  EXPECT_TRUE(normalized <= M_PI);
}

TEST(AngleTest, NormalizationTwoPIPositive)
{
  // Test that angles >= 2pi wrap correctly
  Angle angle(7.0, Angle::Norm::TWO_PI);
  double normalized = angle.getRad();
  EXPECT_TRUE(normalized >= 0.0);
  EXPECT_TRUE(normalized < 2.0 * M_PI);
}

TEST(AngleTest, NormalizationTwoPINegative)
{
  // Test that negative angles wrap to [0, 2pi)
  Angle angle(-1.0, Angle::Norm::TWO_PI);
  double normalized = angle.getRad();
  EXPECT_TRUE(normalized >= 0.0);
  EXPECT_TRUE(normalized < 2.0 * M_PI);
}

TEST(AngleTest, SetNormalization)
{
  Angle angle(M_PI / 2, Angle::Norm::PI);
  angle.setNormalization(Angle::Norm::TWO_PI);
  EXPECT_EQ(angle.getNormalization(), Angle::Norm::TWO_PI);
  // Value should still be valid in new normalization
  EXPECT_TRUE(angle.getRad() >= 0.0);
  EXPECT_TRUE(angle.getRad() < 2.0 * M_PI);
}

// ============================================================================
// Arithmetic Operator Tests
// ============================================================================

TEST(AngleTest, Addition)
{
  Angle a1 = Angle::fromRadians(M_PI / 4);
  Angle a2 = Angle::fromRadians(M_PI / 4);
  Angle result = a1 + a2;
  EXPECT_TRUE(almostEqual(result.getRad(), M_PI / 2));
}

TEST(AngleTest, AdditionWithNormalization)
{
  Angle a1 = Angle::fromRadians(M_PI, Angle::Norm::PI);
  Angle a2 = Angle::fromRadians(M_PI, Angle::Norm::PI);
  Angle result = a1 + a2;
  // Result should be normalized to (-pi, pi]
  EXPECT_TRUE(result.getRad() > -M_PI);
  EXPECT_TRUE(result.getRad() <= M_PI);
}

TEST(AngleTest, Subtraction)
{
  Angle a1 = Angle::fromRadians(M_PI / 2);
  Angle a2 = Angle::fromRadians(M_PI / 4);
  Angle result = a1 - a2;
  EXPECT_TRUE(almostEqual(result.getRad(), M_PI / 4));
}

TEST(AngleTest, SubtractionWithNormalization)
{
  Angle a1 = Angle::fromRadians(-M_PI, Angle::Norm::PI);
  Angle a2 = Angle::fromRadians(M_PI / 2, Angle::Norm::PI);
  Angle result = a1 - a2;
  // Result should be normalized to (-pi, pi]
  EXPECT_TRUE(result.getRad() > -M_PI);
  EXPECT_TRUE(result.getRad() <= M_PI);
}

TEST(AngleTest, MultiplicationByScalar)
{
  Angle angle = Angle::fromRadians(M_PI / 4);
  Angle result = angle * 2.0;
  EXPECT_TRUE(almostEqual(result.getRad(), M_PI / 2));
}

TEST(AngleTest, ScalarMultiplication)
{
  Angle angle = Angle::fromRadians(M_PI / 4);
  Angle result = 2.0 * angle;
  EXPECT_TRUE(almostEqual(result.getRad(), M_PI / 2));
}

TEST(AngleTest, DivisionByScalar)
{
  Angle angle = Angle::fromRadians(M_PI);
  Angle result = angle / 2.0;
  EXPECT_TRUE(almostEqual(result.getRad(), M_PI / 2));
}

TEST(AngleTest, UnaryMinus)
{
  Angle angle = Angle::fromRadians(M_PI / 4);
  Angle result = -angle;
  EXPECT_TRUE(almostEqual(result.getRad(), -M_PI / 4));
}

// ============================================================================
// Compound Assignment Tests
// ============================================================================

TEST(AngleTest, CompoundAddition)
{
  Angle angle = Angle::fromRadians(M_PI / 4);
  angle += Angle::fromRadians(M_PI / 4);
  EXPECT_TRUE(almostEqual(angle.getRad(), M_PI / 2));
}

TEST(AngleTest, CompoundSubtraction)
{
  Angle angle = Angle::fromRadians(M_PI / 2);
  angle -= Angle::fromRadians(M_PI / 4);
  EXPECT_TRUE(almostEqual(angle.getRad(), M_PI / 4));
}

TEST(AngleTest, CompoundMultiplication)
{
  Angle angle = Angle::fromRadians(M_PI / 4);
  angle *= 2.0;
  EXPECT_TRUE(almostEqual(angle.getRad(), M_PI / 2));
}

TEST(AngleTest, CompoundDivision)
{
  Angle angle = Angle::fromRadians(M_PI);
  angle /= 2.0;
  EXPECT_TRUE(almostEqual(angle.getRad(), M_PI / 2));
}

// ============================================================================
// Assignment Tests
// ============================================================================

TEST(AngleTest, AssignmentOperator)
{
  Angle a1 = Angle::fromRadians(M_PI / 4);
  Angle a2;
  a2 = a1;
  EXPECT_DOUBLE_EQ(a2.getRad(), a1.getRad());
  EXPECT_EQ(a2.getNormalization(), a1.getNormalization());
}

TEST(AngleTest, SelfAssignment)
{
  Angle angle = Angle::fromRadians(M_PI / 4);
  angle = angle;
  EXPECT_DOUBLE_EQ(angle.getRad(), M_PI / 4);
}

// ============================================================================
// Comparison Tests
// ============================================================================

TEST(AngleTest, Equality)
{
  Angle a1 = Angle::fromRadians(M_PI / 4);
  Angle a2 = Angle::fromRadians(M_PI / 4);
  EXPECT_TRUE(a1 == a2);
}

TEST(AngleTest, Inequality)
{
  Angle a1 = Angle::fromRadians(M_PI / 4);
  Angle a2 = Angle::fromRadians(M_PI / 3);
  EXPECT_TRUE(a1 != a2);
}

TEST(AngleTest, LessThan)
{
  Angle a1 = Angle::fromRadians(M_PI / 4);
  Angle a2 = Angle::fromRadians(M_PI / 2);
  EXPECT_TRUE(a1 < a2);
  EXPECT_FALSE(a2 < a1);
}

TEST(AngleTest, GreaterThan)
{
  Angle a1 = Angle::fromRadians(M_PI / 2);
  Angle a2 = Angle::fromRadians(M_PI / 4);
  EXPECT_TRUE(a1 > a2);
  EXPECT_FALSE(a2 > a1);
}

TEST(AngleTest, LessThanOrEqual)
{
  Angle a1 = Angle::fromRadians(M_PI / 4);
  Angle a2 = Angle::fromRadians(M_PI / 4);
  Angle a3 = Angle::fromRadians(M_PI / 2);
  EXPECT_TRUE(a1 <= a2);
  EXPECT_TRUE(a1 <= a3);
  EXPECT_FALSE(a3 <= a1);
}

TEST(AngleTest, GreaterThanOrEqual)
{
  Angle a1 = Angle::fromRadians(M_PI / 4);
  Angle a2 = Angle::fromRadians(M_PI / 4);
  Angle a3 = Angle::fromRadians(M_PI / 2);
  EXPECT_TRUE(a1 >= a2);
  EXPECT_TRUE(a3 >= a1);
  EXPECT_FALSE(a1 >= a3);
}

TEST(AngleTest, ZeroAngle)
{
  Angle angle = Angle::fromRadians(0.0);
  EXPECT_DOUBLE_EQ(angle.getRad(), 0.0);
  EXPECT_DOUBLE_EQ(angle.toDeg(), 0.0);
}

TEST(AngleTest, FullCircleTwoPI)
{
  Angle angle = Angle::fromRadians(2.0 * M_PI, Angle::Norm::TWO_PI);
  EXPECT_TRUE(almostEqual(angle.getRad(), 0.0));
}

TEST(AngleTest, FullCirclePI)
{
  Angle angle = Angle::fromRadians(2.0 * M_PI, Angle::Norm::PI);
  EXPECT_TRUE(almostEqual(angle.getRad(), 0.0));
}

TEST(AngleTest, NegativeFullCircle)
{
  Angle angle = Angle::fromRadians(-2.0 * M_PI, Angle::Norm::TWO_PI);
  EXPECT_TRUE(almostEqual(angle.getRad(), 0.0));
}

TEST(AngleTest, VeryLargeAngle)
{
  Angle angle = Angle::fromRadians(10.0 * M_PI, Angle::Norm::PI);
  // Should be normalized to (-pi, pi]
  EXPECT_TRUE(angle.getRad() > -M_PI);
  EXPECT_TRUE(angle.getRad() <= M_PI);
}

TEST(AngleTest, VerySmallAngle)
{
  Angle angle = Angle::fromRadians(-10.0 * M_PI, Angle::Norm::PI);
  // Should be normalized to (-pi, pi]
  EXPECT_TRUE(angle.getRad() > -M_PI);
  EXPECT_TRUE(angle.getRad() <= M_PI);
}

// ============================================================================
// Practical Use Case Tests
// ============================================================================

TEST(AngleTest, AngleDifferenceWraparound)
{
  // Test case: difference between 10 degrees and 350 degrees
  Angle a1 = Angle::fromDegrees(10.0, Angle::Norm::TWO_PI);
  Angle a2 = Angle::fromDegrees(350.0, Angle::Norm::TWO_PI);
  Angle diff = a1 - a2;
  // The difference should be 20 degrees (wrapping around)
  EXPECT_TRUE(almostEqual(std::abs(diff.toDeg()), 20.0, 1.0));
}

TEST(AngleTest, HeadingCalculation)
{
  // Test typical heading/yaw angle in [0, 2pi)
  Angle heading = Angle::fromDegrees(270.0, Angle::Norm::TWO_PI);
  EXPECT_TRUE(almostEqual(heading.toDeg(), 270.0));
  EXPECT_TRUE(almostEqual(heading.getRad(), 3.0 * M_PI / 2.0));
}

TEST(AngleTest, RollPitchInPIRange)
{
  // Roll and pitch typically use (-pi, pi] range
  Angle roll = Angle::fromDegrees(45.0, Angle::Norm::PI);
  EXPECT_TRUE(almostEqual(roll.toDeg(), 45.0));

  Angle pitch = Angle::fromDegrees(-30.0, Angle::Norm::PI);
  EXPECT_TRUE(almostEqual(pitch.toDeg(), -30.0));
}
