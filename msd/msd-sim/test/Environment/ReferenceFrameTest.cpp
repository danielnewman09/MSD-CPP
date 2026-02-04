// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#include <gtest/gtest.h>
#include <cmath>
#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Utils/utils.hpp"

using namespace msd_sim;


bool coordinatesEqual(const Coordinate& c1,
                      const Coordinate& c2,
                      double tolerance = TOLERANCE)
{
  return msd_sim::almostEqual(c1.x(), c2.x(), tolerance) &&
         msd_sim::almostEqual(c1.y(), c2.y(), tolerance) &&
         msd_sim::almostEqual(c1.z(), c2.z(), tolerance);
}

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(ReferenceFrameTest, DefaultConstructor)
{
  ReferenceFrame frame;
  Coordinate origin = frame.getOrigin();

  EXPECT_DOUBLE_EQ(origin.x(), 0.0);
  EXPECT_DOUBLE_EQ(origin.y(), 0.0);
  EXPECT_DOUBLE_EQ(origin.z(), 0.0);

  AngularCoordinate angular = frame.getAngularCoordinate();
  EXPECT_DOUBLE_EQ(angular.roll(), 0.0);
  EXPECT_DOUBLE_EQ(angular.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(angular.yaw(), 0.0);
}

TEST(ReferenceFrameTest, ConstructorWithOrigin)
{
  Coordinate origin{1.0, 2.0, 3.0};
  ReferenceFrame frame{origin};

  Coordinate frameOrigin = frame.getOrigin();
  EXPECT_DOUBLE_EQ(frameOrigin.x(), 1.0);
  EXPECT_DOUBLE_EQ(frameOrigin.y(), 2.0);
  EXPECT_DOUBLE_EQ(frameOrigin.z(), 3.0);

  AngularCoordinate angular = frame.getAngularCoordinate();
  EXPECT_DOUBLE_EQ(angular.roll(), 0.0);
  EXPECT_DOUBLE_EQ(angular.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(angular.yaw(), 0.0);
}

TEST(ReferenceFrameTest, ConstructorWithOriginAndRotation)
{
  Coordinate origin{1.0, 2.0, 3.0};
  AngularCoordinate angular{M_PI / 4, M_PI / 6, M_PI / 3};  // pitch, roll, yaw

  ReferenceFrame frame{origin, angular};

  Coordinate frameOrigin = frame.getOrigin();
  EXPECT_DOUBLE_EQ(frameOrigin.x(), 1.0);
  EXPECT_DOUBLE_EQ(frameOrigin.y(), 2.0);
  EXPECT_DOUBLE_EQ(frameOrigin.z(), 3.0);

  AngularCoordinate frameAngular = frame.getAngularCoordinate();
  EXPECT_DOUBLE_EQ(frameAngular.pitch(), M_PI / 4);
  EXPECT_DOUBLE_EQ(frameAngular.roll(), M_PI / 6);
  EXPECT_DOUBLE_EQ(frameAngular.yaw(), M_PI / 3);
}

// ============================================================================
// Constructor from X and Z Axes Tests
// ============================================================================

TEST(ReferenceFrameTest, ConstructorFromAxes_IdentityAxes)
{
  // Standard world axes should produce identity rotation
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{1.0, 0.0, 0.0};
  Coordinate zDir{0.0, 0.0, 1.0};

  ReferenceFrame frame{origin, xDir, zDir};

  // Verify rotation matrix is identity (or very close)
  const Eigen::Matrix3d& rotation = frame.getRotation();
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 0), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(1, 1), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(2, 2), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 1), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 2), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(1, 0), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, ConstructorFromAxes_90DegYaw)
{
  // X pointing in +Y direction, Z pointing in +Z direction
  // This should be a 90 degree yaw rotation
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{0.0, 1.0, 0.0};
  Coordinate zDir{0.0, 0.0, 1.0};

  ReferenceFrame frame{origin, xDir, zDir};

  // Point at (1, 0, 0) in local frame should map to (0, 1, 0) in global
  Coordinate localPoint{1.0, 0.0, 0.0};
  Coordinate globalPoint = frame.localToGlobal(localPoint);

  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.y(), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.z(), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, ConstructorFromAxes_90DegPitch)
{
  // X pointing in -Z direction, Z pointing in +X direction
  // This should be a -90 degree pitch rotation
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{0.0, 0.0, -1.0};
  Coordinate zDir{1.0, 0.0, 0.0};

  ReferenceFrame frame{origin, xDir, zDir};

  // Point at (1, 0, 0) in local frame should map to (0, 0, -1) in global
  Coordinate localPoint{1.0, 0.0, 0.0};
  Coordinate globalPoint = frame.localToGlobal(localPoint);

  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.y(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.z(), -1.0, 1e-9));
}

TEST(ReferenceFrameTest, ConstructorFromAxes_UnnormalizedInputs)
{
  // Vectors that are not unit length should still work
  Coordinate origin{5.0, 10.0, 15.0};
  Coordinate xDir{3.0, 0.0, 0.0};    // Length 3
  Coordinate zDir{0.0, 0.0, 100.0};  // Length 100

  ReferenceFrame frame{origin, xDir, zDir};

  // Should still produce identity rotation (just with normalization)
  Coordinate localPoint{1.0, 2.0, 3.0};
  Coordinate globalPoint = frame.localToGlobal(localPoint);

  // Expected: localPoint + origin = (6, 12, 18)
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.x(), 6.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.y(), 12.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.z(), 18.0, 1e-9));
}

TEST(ReferenceFrameTest, ConstructorFromAxes_NonOrthogonalInputs)
{
  // X and Z are not perfectly orthogonal - X should be orthogonalized against Z
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{1.0, 0.0, 0.5};  // Not orthogonal to Z
  Coordinate zDir{0.0, 0.0, 1.0};  // Z takes precedence

  ReferenceFrame frame{origin, xDir, zDir};

  // After orthogonalization, X should be purely in +X direction
  // Z should be +Z
  // Y should be +Y (from Z × X)
  const Eigen::Matrix3d& rotation = frame.getRotation();

  // X-axis column should be (1, 0, 0)
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 0), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(1, 0), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(2, 0), 0.0, 1e-9));

  // Z-axis column should be (0, 0, 1)
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 2), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(1, 2), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(2, 2), 1.0, 1e-9));
}

TEST(ReferenceFrameTest, ConstructorFromAxes_RightHandRule)
{
  // Verify the Y-axis follows right-hand rule: Y = Z × X
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{1.0, 0.0, 0.0};
  Coordinate zDir{0.0, 0.0, 1.0};

  ReferenceFrame frame{origin, xDir, zDir};

  const Eigen::Matrix3d& rotation = frame.getRotation();

  // Y-axis should be (0, 1, 0) following right-hand rule
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 1), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(1, 1), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(2, 1), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, ConstructorFromAxes_RoundTrip)
{
  // Arbitrary frame - verify round-trip transformation
  Coordinate origin{5.0, 10.0, 15.0};
  Coordinate xDir{1.0, 1.0, 0.0};  // X at 45 degrees in XY plane
  Coordinate zDir{0.0, 0.0, 1.0};

  ReferenceFrame frame{origin, xDir, zDir};

  Coordinate original{100.0, 200.0, 300.0};
  Coordinate local = frame.globalToLocal(original);
  Coordinate backToGlobal = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(backToGlobal, original, 1e-9));
}

TEST(ReferenceFrameTest, ConstructorFromAxes_CollisionResponseUseCase)
{
  // Typical collision response scenario:
  // - Contact normal (Z) pointing from surface
  // - Tangent direction (X) along surface
  Coordinate contactPoint{10.0, 5.0, 0.0};
  Coordinate normal{0.0, 0.0, 1.0};   // Pointing up
  Coordinate tangent{1.0, 0.0, 0.0};  // Along X-axis

  ReferenceFrame contactFrame{contactPoint, tangent, normal};

  // A velocity in world frame
  Coordinate worldVelocity{
    2.0, 3.0, -5.0};  // Moving with some downward component

  // Transform to contact frame
  Eigen::Vector3d localVelocity =
    contactFrame.globalToLocal(Eigen::Vector3d{worldVelocity});

  // In contact frame:
  // - Z component should be the normal component (penetrating = negative)
  // - X component should be the tangent component
  EXPECT_TRUE(msd_sim::almostEqual(localVelocity.x(), 2.0, 1e-9));  // Tangent X
  EXPECT_TRUE(msd_sim::almostEqual(localVelocity.y(), 3.0, 1e-9));  // Tangent Y
  EXPECT_TRUE(msd_sim::almostEqual(
    localVelocity.z(), -5.0, 1e-9));  // Normal (penetrating)
}

TEST(ReferenceFrameTest, ConstructorFromAxes_ZeroXDirectionThrows)
{
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{0.0, 0.0, 0.0};  // Zero vector
  Coordinate zDir{0.0, 0.0, 1.0};

  EXPECT_THROW(ReferenceFrame(origin, xDir, zDir), std::invalid_argument);
}

TEST(ReferenceFrameTest, ConstructorFromAxes_ZeroZDirectionThrows)
{
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{1.0, 0.0, 0.0};
  Coordinate zDir{0.0, 0.0, 0.0};  // Zero vector

  EXPECT_THROW(ReferenceFrame(origin, xDir, zDir), std::invalid_argument);
}

TEST(ReferenceFrameTest, ConstructorFromAxes_ParallelVectorsThrows)
{
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{1.0, 0.0, 0.0};
  Coordinate zDir{2.0, 0.0, 0.0};  // Parallel to X

  EXPECT_THROW(ReferenceFrame(origin, xDir, zDir), std::invalid_argument);
}

TEST(ReferenceFrameTest, ConstructorFromAxes_AntiParallelVectorsThrows)
{
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{1.0, 0.0, 0.0};
  Coordinate zDir{-5.0, 0.0, 0.0};  // Anti-parallel to X

  EXPECT_THROW(ReferenceFrame(origin, xDir, zDir), std::invalid_argument);
}

TEST(ReferenceFrameTest, ConstructorFromAxes_DiagonalNormal)
{
  // Diagonal normal - tests more complex Euler angle extraction
  Coordinate origin{0.0, 0.0, 0.0};
  Coordinate xDir{1.0, 0.0, 0.0};
  Coordinate zDir{1.0, 1.0, 1.0};  // Diagonal normal

  ReferenceFrame frame{origin, xDir, zDir};

  // Verify the frame is valid by checking orthonormality
  const Eigen::Matrix3d& R = frame.getRotation();

  // Check orthogonality: R^T * R should be identity
  Eigen::Matrix3d product = R.transpose() * R;
  EXPECT_TRUE(msd_sim::almostEqual(product(0, 0), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(1, 1), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(2, 2), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(0, 1), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(0, 2), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(1, 2), 0.0, 1e-9));

  // Check determinant is +1 (proper rotation, not reflection)
  EXPECT_TRUE(msd_sim::almostEqual(R.determinant(), 1.0, 1e-9));
}

TEST(ReferenceFrameTest, ConstructorFromAxes_EulerAnglesConsistent)
{
  // Create frame from axes, then verify the Euler angles are consistent
  // by creating another frame from those Euler angles and comparing rotations
  Coordinate origin{5.0, 10.0, 15.0};
  Coordinate xDir{1.0, 1.0, 0.0};  // 45 degree yaw
  Coordinate zDir{0.0, 0.0, 1.0};

  ReferenceFrame frameFromAxes{origin, xDir, zDir};
  AngularCoordinate extractedAngles = frameFromAxes.getAngularCoordinate();

  // Create another frame from the extracted Euler angles
  ReferenceFrame frameFromAngles{origin, extractedAngles};

  // Both frames should have the same rotation matrix
  const Eigen::Matrix3d& R1 = frameFromAxes.getRotation();
  const Eigen::Matrix3d& R2 = frameFromAngles.getRotation();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_TRUE(msd_sim::almostEqual(R1(i, j), R2(i, j), 1e-9))
        << "Mismatch at (" << i << ", " << j << "): " << R1(i, j) << " vs "
        << R2(i, j);
    }
  }
}

// ============================================================================
// Setter Tests
// ============================================================================

TEST(ReferenceFrameTest, SetOrigin)
{
  ReferenceFrame frame;
  Coordinate newOrigin{5.0, 6.0, 7.0};

  frame.setOrigin(newOrigin);

  Coordinate frameOrigin = frame.getOrigin();
  EXPECT_DOUBLE_EQ(frameOrigin.x(), 5.0);
  EXPECT_DOUBLE_EQ(frameOrigin.y(), 6.0);
  EXPECT_DOUBLE_EQ(frameOrigin.z(), 7.0);
}

TEST(ReferenceFrameTest, SetRotation)
{
  // Normal case (away from gimbal lock): individual Euler angles round-trip
  // exactly
  ReferenceFrame frame;
  AngularCoordinate angular{M_PI / 6, M_PI / 4, M_PI / 3};  // pitch, roll, yaw

  Eigen::Quaterniond q =
    Eigen::AngleAxisd{angular.yaw(), Eigen::Vector3d::UnitZ()} *
    Eigen::AngleAxisd{angular.pitch(), Eigen::Vector3d::UnitY()} *
    Eigen::AngleAxisd{angular.roll(), Eigen::Vector3d::UnitX()};
  frame.setQuaternion(q);

  AngularCoordinate frameAngular = frame.getAngularCoordinate();
  EXPECT_DOUBLE_EQ(frameAngular.pitch(), M_PI / 6);
  EXPECT_DOUBLE_EQ(frameAngular.roll(), M_PI / 4);
  EXPECT_DOUBLE_EQ(frameAngular.yaw(), M_PI / 3);
}

TEST(ReferenceFrameTest, SetRotationGimbalLock)
{
  // Gimbal lock case (pitch = π/2): roll and yaw are coupled, so individual
  // angles cannot round-trip. Verify the resulting rotation matrix is correct.
  ReferenceFrame frame;
  AngularCoordinate angular{M_PI / 2, M_PI / 4, M_PI / 6};  // pitch, roll, yaw

  Eigen::Quaterniond q =
    Eigen::AngleAxisd{angular.yaw(), Eigen::Vector3d::UnitZ()} *
    Eigen::AngleAxisd{angular.pitch(), Eigen::Vector3d::UnitY()} *
    Eigen::AngleAxisd{angular.roll(), Eigen::Vector3d::UnitX()};
  Eigen::Matrix3d expectedRotation = q.toRotationMatrix();

  frame.setQuaternion(q);

  // Pitch is still recoverable at gimbal lock
  AngularCoordinate frameAngular = frame.getAngularCoordinate();
  EXPECT_DOUBLE_EQ(frameAngular.pitch(), M_PI / 2);

  // Roll and yaw individually are ambiguous, but the rotation matrix
  // reconstructed from the extracted angles must match the original
  const Eigen::Matrix3d& actualRotation = frame.getRotation();
  EXPECT_TRUE(actualRotation.isApprox(expectedRotation, 1e-10));
}

// ============================================================================
// Identity Transformation Tests (No Translation, No Rotation)
// ============================================================================

TEST(ReferenceFrameTest, IdentityTransformGlobalToLocal)
{
  ReferenceFrame frame;  // Identity frame at origin
  Coordinate globalCoord{1.0, 2.0, 3.0};

  Coordinate localCoord = frame.globalToLocal(globalCoord);

  EXPECT_TRUE(coordinatesEqual(localCoord, globalCoord));
}

TEST(ReferenceFrameTest, IdentityTransformLocalToGlobal)
{
  ReferenceFrame frame;  // Identity frame at origin
  Coordinate localCoord{1.0, 2.0, 3.0};

  Coordinate globalCoord = frame.localToGlobal(localCoord);

  EXPECT_TRUE(coordinatesEqual(globalCoord, localCoord));
}

TEST(ReferenceFrameTest, IdentityRoundTrip)
{
  ReferenceFrame frame;
  Coordinate original{1.0, 2.0, 3.0};

  Coordinate local = frame.globalToLocal(original);
  Coordinate global = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(global, original));
}

// ============================================================================
// Translation Only Tests
// ============================================================================

TEST(ReferenceFrameTest, TranslationOnlyGlobalToLocal)
{
  Coordinate origin{10.0, 20.0, 30.0};
  ReferenceFrame frame{origin};

  Coordinate globalCoord{15.0, 25.0, 35.0};
  Coordinate localCoord = frame.globalToLocal(globalCoord);

  // Local coordinate should be global - origin
  EXPECT_DOUBLE_EQ(localCoord.x(), 5.0);
  EXPECT_DOUBLE_EQ(localCoord.y(), 5.0);
  EXPECT_DOUBLE_EQ(localCoord.z(), 5.0);
}

TEST(ReferenceFrameTest, TranslationOnlyLocalToGlobal)
{
  Coordinate origin{10.0, 20.0, 30.0};
  ReferenceFrame frame{origin};

  Coordinate localCoord{5.0, 5.0, 5.0};
  Coordinate globalCoord = frame.localToGlobal(localCoord);

  // Global coordinate should be local + origin
  EXPECT_DOUBLE_EQ(globalCoord.x(), 15.0);
  EXPECT_DOUBLE_EQ(globalCoord.y(), 25.0);
  EXPECT_DOUBLE_EQ(globalCoord.z(), 35.0);
}

TEST(ReferenceFrameTest, TranslationOnlyRoundTrip)
{
  Coordinate origin{100.0, 200.0, 300.0};
  ReferenceFrame frame{origin};

  Coordinate original{150.0, 250.0, 350.0};

  Coordinate local = frame.globalToLocal(original);
  Coordinate global = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(global, original));
}

// ============================================================================
// Rotation Only Tests (About Each Axis)
// ============================================================================

TEST(ReferenceFrameTest, YawRotation90Degrees)
{
  // Rotate 90 degrees about Z-axis (yaw)
  AngularCoordinate angular{
    0.0,      // pitch
    0.0,      // roll
    M_PI / 2  // yaw = 90 degrees
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, angular};

  // Point at (1, 0, 0) in global frame
  Coordinate globalCoord{1.0, 0.0, 0.0};
  Coordinate localCoord = frame.globalToLocal(globalCoord);

  // After 90 degree yaw rotation, should be at (0, -1, 0) in local frame
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.y(), -1.0, 1e-9))
    << localCoord.y();
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.z(), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, PitchRotation90Degrees)
{
  // Rotate 90 degrees about Y-axis (pitch)
  AngularCoordinate angular{
    M_PI / 2,  // pitch = 90 degrees
    0.0,       // roll
    0.0        // yaw
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, angular};

  // Point at (1, 0, 0) in global frame
  Coordinate globalCoord{1.0, 0.0, 0.0};
  Coordinate localCoord = frame.globalToLocal(globalCoord);

  // After 90 degree pitch rotation, should be at (0, 0, 1) in local frame
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.y(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.z(), 1.0, 1e-9));
}

TEST(ReferenceFrameTest, RollRotation90Degrees)
{
  // Rotate 90 degrees about X-axis (roll)
  AngularCoordinate angular{
    0.0,       // pitch
    M_PI / 2,  // roll = 90 degrees
    0.0        // yaw
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, angular};

  // Point at (0, 1, 0) in global frame
  Coordinate globalCoord{0.0, 1.0, 0.0};
  Coordinate localCoord = frame.globalToLocal(globalCoord);

  // After 90 degree roll rotation, should be at (0, 0, -1) in local frame
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.y(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.z(), -1.0, 1e-9));
}

TEST(ReferenceFrameTest, RotationOnlyRoundTrip)
{
  AngularCoordinate angular{
    (0.3),  // pitch
    (0.5),  // roll
    (0.7)   // yaw
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, angular};

  Coordinate original{1.5, 2.5, 3.5};

  Coordinate local = frame.globalToLocal(original);
  Coordinate global = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(global, original, 1e-9));
}

// ============================================================================
// Combined Translation and Rotation Tests
// ============================================================================

TEST(ReferenceFrameTest, TranslationAndRotationGlobalToLocal)
{
  Coordinate origin{10.0, 20.0, 30.0};
  AngularCoordinate angular{
    0.0,      // pitch
    0.0,      // roll
    M_PI / 2  // yaw = 90 degrees
  };
  ReferenceFrame frame{origin, angular};

  // Point at (11, 20, 30) in global frame
  Coordinate globalCoord{11.0, 20.0, 30.0};
  Coordinate localCoord = frame.globalToLocal(globalCoord);

  // First translate: (11, 20, 30) - (10, 20, 30) = (1, 0, 0)
  // Then rotate 90 degrees about Z: (1, 0, 0) -> (0, -1, 0)
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.y(), -1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.z(), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, TranslationAndRotationLocalToGlobal)
{
  Coordinate origin{10.0, 20.0, 30.0};
  AngularCoordinate angular{
    0.0,      // pitch
    0.0,      // roll
    M_PI / 2  // yaw = 90 degrees
  };
  ReferenceFrame frame{origin, angular};

  // Point at (0, -1, 0) in local frame
  Coordinate localCoord{0.0, -1.0, 0.0};
  Coordinate globalCoord = frame.localToGlobal(localCoord);

  // First rotate back: (0, -1, 0) -> (1, 0, 0)
  // Then translate: (1, 0, 0) + (10, 20, 30) = (11, 20, 30)
  EXPECT_TRUE(msd_sim::almostEqual(globalCoord.x(), 11.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalCoord.y(), 20.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalCoord.z(), 30.0, 1e-9));
}

TEST(ReferenceFrameTest, ComplexTransformationRoundTrip)
{
  Coordinate origin{5.5, 10.3, -2.7};
  AngularCoordinate angular{
    (0.4),   // pitch
    (-0.3),  // roll
    (1.2)    // yaw
  };
  ReferenceFrame frame{origin, angular};

  Coordinate original{100.0, 200.0, 300.0};

  Coordinate local = frame.globalToLocal(original);
  Coordinate global = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(global, original, 1e-9));
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(ReferenceFrameTest, OriginPoint)
{
  Coordinate origin{5.0, 10.0, 15.0};
  ReferenceFrame frame{origin};

  // Transform the origin point itself
  Coordinate localCoord = frame.globalToLocal(origin);

  // Should be at (0, 0, 0) in local frame
  EXPECT_DOUBLE_EQ(localCoord.x(), 0.0);
  EXPECT_DOUBLE_EQ(localCoord.y(), 0.0);
  EXPECT_DOUBLE_EQ(localCoord.z(), 0.0);
}

TEST(ReferenceFrameTest, NegativeCoordinates)
{
  Coordinate origin{-5.0, -10.0, -15.0};
  ReferenceFrame frame{origin};

  Coordinate globalCoord{-3.0, -8.0, -12.0};
  Coordinate localCoord = frame.globalToLocal(globalCoord);

  EXPECT_DOUBLE_EQ(localCoord.x(), 2.0);
  EXPECT_DOUBLE_EQ(localCoord.y(), 2.0);
  EXPECT_DOUBLE_EQ(localCoord.z(), 3.0);
}

TEST(ReferenceFrameTest, LargeRotationAngles)
{
  // Test with angles that require normalization
  AngularCoordinate angular{
    (3.5 * M_PI),   // pitch
    (-2.7 * M_PI),  // roll
    (5.1 * M_PI)    // yaw
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, angular};

  Coordinate original{1.0, 2.0, 3.0};

  Coordinate local = frame.globalToLocal(original);
  Coordinate global = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(global, original, 1e-9));
}

TEST(ReferenceFrameTest, ZeroVector)
{
  Coordinate origin{5.0, 10.0, 15.0};
  AngularCoordinate angular{(0.5), (0.3), (0.7)};
  ReferenceFrame frame{origin, angular};

  Coordinate zeroVec{0.0, 0.0, 0.0};

  Coordinate local = frame.globalToLocal(zeroVec);
  Coordinate global = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(global, zeroVec, 1e-9));
}

// ============================================================================
// Multiple Frame Chaining Tests
// ============================================================================

TEST(ReferenceFrameTest, ChainedFrameTransformations)
{
  // Create two reference frames
  Coordinate origin1{10.0, 0.0, 0.0};
  AngularCoordinate angular1{0.0, 0.0, M_PI / 4};
  ReferenceFrame frame1{origin1, angular1};

  Coordinate origin2{5.0, 0.0, 0.0};  // Relative to frame1
  // euler2 declared but unused - test focuses on frame1 operations
  // EulerAngles euler2{...}

  // Point in global frame
  Coordinate globalPoint{20.0, 5.0, 0.0};

  // Transform to frame1
  Coordinate inFrame1 = frame1.globalToLocal(globalPoint);

  // The origin of frame2 in frame1's local coordinates
  Coordinate frame2Origin = origin2;

  // Transform back to global should match
  Coordinate backToGlobal = frame1.localToGlobal(inFrame1);

  EXPECT_TRUE(coordinatesEqual(backToGlobal, globalPoint, 1e-9));
}

// ============================================================================
// Practical Use Case Tests
// ============================================================================

TEST(ReferenceFrameTest, AircraftBodyFrame)
{
  // Simulate an aircraft at position (1000, 2000, 500) meters
  // with heading 45 degrees (yaw), pitch up 10 degrees, no roll
  Coordinate aircraftPos{1000.0, 2000.0, 500.0};
  AngularCoordinate aircraftAttitude{
    10.0 * M_PI / 180.0,  // pitch up 10 degrees
    0.0,                  // no roll
    45.0 * M_PI / 180.0   // heading 45 degrees
  };
  ReferenceFrame bodyFrame{aircraftPos, aircraftAttitude};

  // A point 10 meters ahead of the aircraft in its body frame
  Coordinate pointInBody{10.0, 0.0, 0.0};

  // Transform to global frame
  Coordinate pointInGlobal = bodyFrame.localToGlobal(pointInBody);

  // Should be displaced from aircraft position
  Coordinate displacement = pointInGlobal - aircraftPos;

  // Check that displacement is not zero
  double distanceFromAircraft = displacement.norm();
  EXPECT_TRUE(msd_sim::almostEqual(distanceFromAircraft, 10.0, 0.1));
}

TEST(ReferenceFrameTest, SensorMountOnRobot)
{
  // Robot at position (5, 5, 0) facing 90 degrees (north)
  Coordinate robotPos{5.0, 5.0, 0.0};
  AngularCoordinate robotOrientation{0.0, 0.0, 90.0 * M_PI / 180.0};
  ReferenceFrame robotFrame{robotPos, robotOrientation};

  // Sensor mounted 1 meter in front and 0.5 meters up
  Coordinate sensorInRobotFrame{1.0, 0.0, 0.5};

  // Get sensor position in global frame
  Coordinate sensorInGlobal = robotFrame.localToGlobal(sensorInRobotFrame);

  // Sensor should be roughly at (5, 6, 0.5) in global frame
  // (1 meter north of robot position)
  EXPECT_TRUE(msd_sim::almostEqual(sensorInGlobal.x(), 5.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(sensorInGlobal.y(), 6.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(sensorInGlobal.z(), 0.5, 1e-9));
}

// ============================================================================
// Quaternion Constructor and Getter Tests
// ============================================================================

TEST(ReferenceFrameTest, QuaternionConstructor_IdentityQuaternion)
{
  // Identity quaternion should produce identity rotation
  Coordinate origin{0.0, 0.0, 0.0};
  Eigen::Quaterniond identity{Eigen::Quaterniond::Identity()};

  ReferenceFrame frame{origin, identity};

  // Verify rotation matrix is identity
  const Eigen::Matrix3d& rotation = frame.getRotation();
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 0), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(1, 1), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(2, 2), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 1), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(0, 2), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(1, 0), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(1, 2), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(2, 0), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(rotation(2, 1), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, QuaternionConstructor_90DegYaw)
{
  // 90 degree rotation about Z-axis (yaw)
  Coordinate origin{0.0, 0.0, 0.0};
  Eigen::Quaterniond quat{
    Eigen::AngleAxisd{M_PI / 2, Eigen::Vector3d::UnitZ()}};

  ReferenceFrame frame{origin, quat};

  // Point at (1, 0, 0) in local frame should map to (0, 1, 0) in global
  Coordinate localPoint{1.0, 0.0, 0.0};
  Coordinate globalPoint = frame.localToGlobal(localPoint);

  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.y(), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.z(), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, QuaternionConstructor_90DegPitch)
{
  // 90 degree rotation about Y-axis (pitch)
  Coordinate origin{0.0, 0.0, 0.0};
  Eigen::Quaterniond quat{
    Eigen::AngleAxisd{M_PI / 2, Eigen::Vector3d::UnitY()}};

  ReferenceFrame frame{origin, quat};

  // Point at (1, 0, 0) in local frame should map to (0, 0, -1) in global
  Coordinate localPoint{1.0, 0.0, 0.0};
  Coordinate globalPoint = frame.localToGlobal(localPoint);

  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.y(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.z(), -1.0, 1e-9));
}

TEST(ReferenceFrameTest, QuaternionConstructor_90DegRoll)
{
  // 90 degree rotation about X-axis (roll)
  Coordinate origin{0.0, 0.0, 0.0};
  Eigen::Quaterniond quat{
    Eigen::AngleAxisd{M_PI / 2, Eigen::Vector3d::UnitX()}};

  ReferenceFrame frame{origin, quat};

  // Point at (0, 1, 0) in local frame should map to (0, 0, 1) in global
  Coordinate localPoint{0.0, 1.0, 0.0};
  Coordinate globalPoint = frame.localToGlobal(localPoint);

  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.y(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.z(), 1.0, 1e-9));
}

TEST(ReferenceFrameTest, QuaternionConstructor_ArbitraryRotation)
{
  // Arbitrary rotation: combine roll, pitch, yaw
  Coordinate origin{5.0, 10.0, 15.0};

  // Create quaternion from combined rotations
  Eigen::Quaterniond qRoll{Eigen::AngleAxisd{0.3, Eigen::Vector3d::UnitX()}};
  Eigen::Quaterniond qPitch{Eigen::AngleAxisd{0.5, Eigen::Vector3d::UnitY()}};
  Eigen::Quaterniond qYaw{Eigen::AngleAxisd{0.7, Eigen::Vector3d::UnitZ()}};
  Eigen::Quaterniond quat = qYaw * qPitch * qRoll;  // ZYX order

  ReferenceFrame frame{origin, quat};

  // Verify rotation matrix is orthonormal
  const Eigen::Matrix3d& R = frame.getRotation();
  Eigen::Matrix3d product = R.transpose() * R;
  EXPECT_TRUE(msd_sim::almostEqual(product(0, 0), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(1, 1), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(2, 2), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(0, 1), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(0, 2), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(product(1, 2), 0.0, 1e-9));

  // Check determinant is +1 (proper rotation)
  EXPECT_TRUE(msd_sim::almostEqual(R.determinant(), 1.0, 1e-9));
}

TEST(ReferenceFrameTest, QuaternionConstructor_RoundTrip)
{
  // Create frame from quaternion, transform point, transform back
  Coordinate origin{5.0, 10.0, 15.0};
  Eigen::Quaterniond quat{
    Eigen::AngleAxisd{M_PI / 3, Eigen::Vector3d{1, 1, 1}.normalized()}};

  ReferenceFrame frame{origin, quat};

  Coordinate original{100.0, 200.0, 300.0};
  Coordinate local = frame.globalToLocal(original);
  Coordinate backToGlobal = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(backToGlobal, original, 1e-9));
}

TEST(ReferenceFrameTest, QuaternionConstructor_UnnormalizedInput)
{
  // Constructor should normalize the input quaternion
  Coordinate origin{0.0, 0.0, 0.0};

  // Create an unnormalized quaternion (scaled by 3)
  Eigen::Quaterniond normalizedQuat{
    Eigen::AngleAxisd{M_PI / 2, Eigen::Vector3d::UnitZ()}};
  Eigen::Quaterniond unnormalizedQuat{normalizedQuat.w() * 3.0,
                                      normalizedQuat.x() * 3.0,
                                      normalizedQuat.y() * 3.0,
                                      normalizedQuat.z() * 3.0};

  ReferenceFrame frame{origin, unnormalizedQuat};

  // Should still produce correct rotation
  Coordinate localPoint{1.0, 0.0, 0.0};
  Coordinate globalPoint = frame.localToGlobal(localPoint);

  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.y(), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalPoint.z(), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, GetQuaternion_Identity)
{
  // Default frame should return identity quaternion
  ReferenceFrame frame;

  Eigen::Quaterniond quat = frame.getQuaternion();

  // Identity quaternion is (w=1, x=0, y=0, z=0) or (w=-1, x=0, y=0, z=0)
  // Both represent the same rotation
  EXPECT_TRUE(msd_sim::almostEqual(std::abs(quat.w()), 1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(quat.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(quat.y(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(quat.z(), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, GetQuaternion_FromEulerAngles)
{
  // Create frame from Euler angles, get quaternion, verify rotation equivalence
  Coordinate origin{0.0, 0.0, 0.0};
  AngularCoordinate angular{0.3, 0.5, 0.7};  // pitch, roll, yaw

  ReferenceFrame frame{origin, angular};
  Eigen::Quaterniond quat = frame.getQuaternion();

  // The quaternion should represent the same rotation as the rotation matrix
  Eigen::Matrix3d quatRotation = quat.toRotationMatrix();
  const Eigen::Matrix3d& frameRotation = frame.getRotation();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_TRUE(
        msd_sim::almostEqual(quatRotation(i, j), frameRotation(i, j), 1e-9))
        << "Mismatch at (" << i << ", " << j << "): " << quatRotation(i, j)
        << " vs " << frameRotation(i, j);
    }
  }
}

TEST(ReferenceFrameTest, QuaternionRoundTrip)
{
  // Create frame from quaternion, get quaternion back, verify equivalence
  Coordinate origin{5.0, 10.0, 15.0};
  Eigen::Quaterniond originalQuat{
    Eigen::AngleAxisd{M_PI / 3, Eigen::Vector3d{1, 2, 3}.normalized()}};

  ReferenceFrame frame{origin, originalQuat};
  Eigen::Quaterniond retrievedQuat = frame.getQuaternion();

  // Quaternions q and -q represent the same rotation
  // Check that they produce the same rotation matrix
  Eigen::Matrix3d originalRotation =
    originalQuat.normalized().toRotationMatrix();
  Eigen::Matrix3d retrievedRotation = retrievedQuat.toRotationMatrix();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_TRUE(msd_sim::almostEqual(
        originalRotation(i, j), retrievedRotation(i, j), 1e-9))
        << "Mismatch at (" << i << ", " << j << ")";
    }
  }
}

TEST(ReferenceFrameTest, QuaternionConstructor_GimbalLockPitchPositive)
{
  // Test gimbal lock at pitch = +π/2
  Coordinate origin{0.0, 0.0, 0.0};

  // Create quaternion for pitch = +90 degrees
  Eigen::Quaterniond quat{
    Eigen::AngleAxisd{M_PI / 2, Eigen::Vector3d::UnitY()}};

  ReferenceFrame frame{origin, quat};

  // Round-trip should still work
  Coordinate original{1.0, 2.0, 3.0};
  Coordinate local = frame.globalToLocal(original);
  Coordinate backToGlobal = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(backToGlobal, original, 1e-9));

  // Quaternion round-trip should also work
  Eigen::Quaterniond retrieved = frame.getQuaternion();
  Eigen::Matrix3d originalRotation = quat.toRotationMatrix();
  Eigen::Matrix3d retrievedRotation = retrieved.toRotationMatrix();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_TRUE(msd_sim::almostEqual(
        originalRotation(i, j), retrievedRotation(i, j), 1e-9));
    }
  }
}

TEST(ReferenceFrameTest, QuaternionConstructor_GimbalLockPitchNegative)
{
  // Test gimbal lock at pitch = -π/2
  Coordinate origin{0.0, 0.0, 0.0};

  // Create quaternion for pitch = -90 degrees
  Eigen::Quaterniond quat{
    Eigen::AngleAxisd{-M_PI / 2, Eigen::Vector3d::UnitY()}};

  ReferenceFrame frame{origin, quat};

  // Round-trip should still work
  Coordinate original{1.0, 2.0, 3.0};
  Coordinate local = frame.globalToLocal(original);
  Coordinate backToGlobal = frame.localToGlobal(local);

  EXPECT_TRUE(coordinatesEqual(backToGlobal, original, 1e-9));

  // Quaternion round-trip should also work
  Eigen::Quaterniond retrieved = frame.getQuaternion();
  Eigen::Matrix3d originalRotation = quat.toRotationMatrix();
  Eigen::Matrix3d retrievedRotation = retrieved.toRotationMatrix();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_TRUE(msd_sim::almostEqual(
        originalRotation(i, j), retrievedRotation(i, j), 1e-9));
    }
  }
}

TEST(ReferenceFrameTest, EulerQuaternionConsistency)
{
  // Create frame from Euler angles, compare with equivalent quaternion-based
  // frame
  Coordinate origin{10.0, 20.0, 30.0};
  double pitch = 0.4;
  double roll = -0.3;
  double yaw = 1.2;

  // Frame from Euler angles
  AngularCoordinate angular{pitch, roll, yaw};
  ReferenceFrame frameFromEuler{origin, angular};

  // Equivalent quaternion (ZYX convention: yaw * pitch * roll)
  Eigen::Quaterniond qRoll{Eigen::AngleAxisd{roll, Eigen::Vector3d::UnitX()}};
  Eigen::Quaterniond qPitch{Eigen::AngleAxisd{pitch, Eigen::Vector3d::UnitY()}};
  Eigen::Quaterniond qYaw{Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}};
  Eigen::Quaterniond quat = qYaw * qPitch * qRoll;
  ReferenceFrame frameFromQuat{origin, quat};

  // Both should produce the same rotation matrix
  const Eigen::Matrix3d& R1 = frameFromEuler.getRotation();
  const Eigen::Matrix3d& R2 = frameFromQuat.getRotation();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_TRUE(msd_sim::almostEqual(R1(i, j), R2(i, j), 1e-9))
        << "Mismatch at (" << i << ", " << j << "): " << R1(i, j) << " vs "
        << R2(i, j);
    }
  }
}
