// Ticket: 0035a_tangent_basis_and_friction_constraint
// Design: docs/designs/0035a_tangent_basis_and_friction_constraint/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Collision/TangentBasis.hpp"
#include <cmath>
#include <random>

using namespace msd_sim;

TEST(TangentBasis, OrthonormalityForAllCoordinateAxes)
{
  constexpr double kTolerance = 1e-6;

  // Test all 6 coordinate-aligned normals: ±ex, ±ey, ±ez
  std::vector<Coordinate> normals = {
    Coordinate{1.0, 0.0, 0.0},   // +x
    Coordinate{-1.0, 0.0, 0.0},  // -x
    Coordinate{0.0, 1.0, 0.0},   // +y
    Coordinate{0.0, -1.0, 0.0},  // -y
    Coordinate{0.0, 0.0, 1.0},   // +z
    Coordinate{0.0, 0.0, -1.0}   // -z
  };

  for (const auto& normal : normals) {
    TangentFrame frame = TangentBasis::computeTangentBasis(normal);

    // Verify unit length: ||t1|| = 1, ||t2|| = 1
    EXPECT_NEAR(frame.t1.norm(), 1.0, kTolerance);
    EXPECT_NEAR(frame.t2.norm(), 1.0, kTolerance);

    // Verify orthogonality: t1 · t2 = 0
    EXPECT_NEAR(frame.t1.dot(frame.t2), 0.0, kTolerance);

    // Verify perpendicular to normal: ti · n = 0
    EXPECT_NEAR(frame.t1.dot(normal), 0.0, kTolerance);
    EXPECT_NEAR(frame.t2.dot(normal), 0.0, kTolerance);
  }
}

TEST(TangentBasis, Determinism_RepeatedCallsProduceIdenticalOutput)
{
  Coordinate normal{0.577350, 0.577350, 0.577350};  // Normalized (1,1,1)

  // Compute tangent basis multiple times
  TangentFrame frame1 = TangentBasis::computeTangentBasis(normal);
  TangentFrame frame2 = TangentBasis::computeTangentBasis(normal);
  TangentFrame frame3 = TangentBasis::computeTangentBasis(normal);

  constexpr double kTolerance = 1e-15;  // Machine precision for identical calls

  // Verify all results are identical
  EXPECT_NEAR((frame1.t1 - frame2.t1).norm(), 0.0, kTolerance);
  EXPECT_NEAR((frame1.t2 - frame2.t2).norm(), 0.0, kTolerance);
  EXPECT_NEAR((frame1.t1 - frame3.t1).norm(), 0.0, kTolerance);
  EXPECT_NEAR((frame1.t2 - frame3.t2).norm(), 0.0, kTolerance);
}

TEST(TangentBasis, Continuity_SmallPerturbationsProduceSmallChanges)
{
  // Base normal
  Coordinate normal{0.707107, 0.707107, 0.0};  // Normalized (1, 1, 0)
  TangentFrame frame = TangentBasis::computeTangentBasis(normal);

  // Perturb normal by epsilon = 1e-4
  constexpr double epsilon = 1e-4;
  Coordinate perturbedNormal{normal.x() + epsilon, normal.y(), normal.z()};
  perturbedNormal = perturbedNormal.normalized();
  TangentFrame perturbedFrame = TangentBasis::computeTangentBasis(perturbedNormal);

  // Verify changes in tangents are O(epsilon): ||Δt|| < 2ε
  double deltaT1 = (frame.t1 - perturbedFrame.t1).norm();
  double deltaT2 = (frame.t2 - perturbedFrame.t2).norm();

  EXPECT_LT(deltaT1, 2.0 * epsilon);
  EXPECT_LT(deltaT2, 2.0 * epsilon);
}

TEST(TangentBasis, Degeneracy_HandlesCoordinateAlignedNormalsWithoutSingularities)
{
  constexpr double kTolerance = 1e-6;

  // Test exact coordinate alignment (potential singularities)
  std::vector<Coordinate> normals = {
    Coordinate{1.0, 0.0, 0.0},   // Exactly aligned with +x
    Coordinate{0.0, 1.0, 0.0},   // Exactly aligned with +y
    Coordinate{0.0, 0.0, 1.0}    // Exactly aligned with +z
  };

  for (const auto& normal : normals) {
    // Should not throw or produce NaN/inf
    TangentFrame frame = TangentBasis::computeTangentBasis(normal);

    // Verify valid outputs (no NaN/inf)
    EXPECT_TRUE(std::isfinite(frame.t1.norm()));
    EXPECT_TRUE(std::isfinite(frame.t2.norm()));

    // Verify orthonormality still holds
    EXPECT_NEAR(frame.t1.norm(), 1.0, kTolerance);
    EXPECT_NEAR(frame.t2.norm(), 1.0, kTolerance);
    EXPECT_NEAR(frame.t1.dot(frame.t2), 0.0, kTolerance);
    EXPECT_NEAR(frame.t1.dot(normal), 0.0, kTolerance);
    EXPECT_NEAR(frame.t2.dot(normal), 0.0, kTolerance);
  }
}

TEST(TangentBasis, InvalidInput_NonUnitNormalThrowsException)
{
  // Non-unit normal (length = 2.0)
  Coordinate nonUnitNormal{2.0, 0.0, 0.0};

  // Should throw std::invalid_argument
  EXPECT_THROW(TangentBasis::computeTangentBasis(nonUnitNormal), std::invalid_argument);

  // Nearly zero normal (length << 1)
  Coordinate nearlyZeroNormal{1e-8, 0.0, 0.0};
  EXPECT_THROW(TangentBasis::computeTangentBasis(nearlyZeroNormal), std::invalid_argument);
}

TEST(TangentBasis, ArbitraryNormals_ValidatesOrthonormalityForRandomVectors)
{
  constexpr double kTolerance = 1e-6;
  constexpr int kNumTests = 100;

  std::mt19937 rng{42};  // Fixed seed for reproducibility
  std::uniform_real_distribution<double> dist{-1.0, 1.0};

  for (int i = 0; i < kNumTests; ++i) {
    // Generate random unit normal
    Coordinate normal{dist(rng), dist(rng), dist(rng)};
    normal = normal.normalized();

    TangentFrame frame = TangentBasis::computeTangentBasis(normal);

    // Verify orthonormality properties
    EXPECT_NEAR(frame.t1.norm(), 1.0, kTolerance);
    EXPECT_NEAR(frame.t2.norm(), 1.0, kTolerance);
    EXPECT_NEAR(frame.t1.dot(frame.t2), 0.0, kTolerance);
    EXPECT_NEAR(frame.t1.dot(normal), 0.0, kTolerance);
    EXPECT_NEAR(frame.t2.dot(normal), 0.0, kTolerance);
  }
}

TEST(TangentFrame, ConstructionWithValidUnitTangentsSucceeds)
{
  Coordinate t1{1.0, 0.0, 0.0};
  Coordinate t2{0.0, 1.0, 0.0};

  // Should not throw
  EXPECT_NO_THROW((TangentFrame{t1, t2}));

  TangentFrame frame{t1, t2};
  EXPECT_LT((frame.t1 - t1).norm(), 1e-15);
  EXPECT_LT((frame.t2 - t2).norm(), 1e-15);
}

TEST(TangentFrame, ConstructionWithNonUnitTangentThrowsException)
{
  Coordinate t1Unit{1.0, 0.0, 0.0};
  Coordinate t2NonUnit{2.0, 0.0, 0.0};  // Length = 2.0

  // Non-unit t1
  EXPECT_THROW(TangentFrame(t2NonUnit, t1Unit), std::invalid_argument);

  // Non-unit t2
  EXPECT_THROW(TangentFrame(t1Unit, t2NonUnit), std::invalid_argument);

  // Both non-unit
  EXPECT_THROW(TangentFrame(t2NonUnit, t2NonUnit), std::invalid_argument);
}
