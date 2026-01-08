#include <gtest/gtest.h>
#include <cmath>
#include "msd-sim/src/Environment/Angle.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"
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

  EulerAngles euler = frame.getEulerAngles();
  EXPECT_DOUBLE_EQ(euler.roll.getRad(), 0.0);
  EXPECT_DOUBLE_EQ(euler.pitch.getRad(), 0.0);
  EXPECT_DOUBLE_EQ(euler.yaw.getRad(), 0.0);
}

TEST(ReferenceFrameTest, ConstructorWithOrigin)
{
  Coordinate origin{1.0, 2.0, 3.0};
  ReferenceFrame frame{origin};

  Coordinate frameOrigin = frame.getOrigin();
  EXPECT_DOUBLE_EQ(frameOrigin.x(), 1.0);
  EXPECT_DOUBLE_EQ(frameOrigin.y(), 2.0);
  EXPECT_DOUBLE_EQ(frameOrigin.z(), 3.0);

  EulerAngles euler = frame.getEulerAngles();
  EXPECT_DOUBLE_EQ(euler.roll.getRad(), 0.0);
  EXPECT_DOUBLE_EQ(euler.pitch.getRad(), 0.0);
  EXPECT_DOUBLE_EQ(euler.yaw.getRad(), 0.0);
}

TEST(ReferenceFrameTest, ConstructorWithOriginAndRotation)
{
  Coordinate origin{1.0, 2.0, 3.0};
  EulerAngles euler{
    Angle::fromRadians(M_PI / 4),  // pitch
    Angle::fromRadians(M_PI / 6),  // roll
    Angle::fromRadians(M_PI / 3)   // yaw
  };

  ReferenceFrame frame{origin, euler};

  Coordinate frameOrigin = frame.getOrigin();
  EXPECT_DOUBLE_EQ(frameOrigin.x(), 1.0);
  EXPECT_DOUBLE_EQ(frameOrigin.y(), 2.0);
  EXPECT_DOUBLE_EQ(frameOrigin.z(), 3.0);

  EulerAngles frameEuler = frame.getEulerAngles();
  EXPECT_DOUBLE_EQ(frameEuler.pitch.getRad(), M_PI / 4);
  EXPECT_DOUBLE_EQ(frameEuler.roll.getRad(), M_PI / 6);
  EXPECT_DOUBLE_EQ(frameEuler.yaw.getRad(), M_PI / 3);
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
  ReferenceFrame frame;
  EulerAngles euler{
    Angle::fromRadians(M_PI / 2),  // pitch
    Angle::fromRadians(M_PI / 4),  // roll
    Angle::fromRadians(M_PI / 6)   // yaw
  };

  frame.setRotation(euler);

  EulerAngles frameEuler = frame.getEulerAngles();
  EXPECT_DOUBLE_EQ(frameEuler.pitch.getRad(), M_PI / 2);
  EXPECT_DOUBLE_EQ(frameEuler.roll.getRad(), M_PI / 4);
  EXPECT_DOUBLE_EQ(frameEuler.yaw.getRad(), M_PI / 6);
}

// ============================================================================
// Identity Transformation Tests (No Translation, No Rotation)
// ============================================================================

TEST(ReferenceFrameTest, IdentityTransformGlobalToLocal)
{
  ReferenceFrame frame;  // Identity frame at origin
  Coordinate globalCoord{1.0, 2.0, 3.0};

  Coordinate localCoord = frame.globalToLocalAbsolute(globalCoord);

  EXPECT_TRUE(coordinatesEqual(localCoord, globalCoord));
}

TEST(ReferenceFrameTest, IdentityTransformLocalToGlobal)
{
  ReferenceFrame frame;  // Identity frame at origin
  Coordinate localCoord{1.0, 2.0, 3.0};

  Coordinate globalCoord = frame.localToGlobalAbsolute(localCoord);

  EXPECT_TRUE(coordinatesEqual(globalCoord, localCoord));
}

TEST(ReferenceFrameTest, IdentityRoundTrip)
{
  ReferenceFrame frame;
  Coordinate original{1.0, 2.0, 3.0};

  Coordinate local = frame.globalToLocalAbsolute(original);
  Coordinate global = frame.localToGlobalAbsolute(local);

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
  Coordinate localCoord = frame.globalToLocalAbsolute(globalCoord);

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
  Coordinate globalCoord = frame.localToGlobalAbsolute(localCoord);

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

  Coordinate local = frame.globalToLocalAbsolute(original);
  Coordinate global = frame.localToGlobalAbsolute(local);

  EXPECT_TRUE(coordinatesEqual(global, original));
}

// ============================================================================
// Rotation Only Tests (About Each Axis)
// ============================================================================

TEST(ReferenceFrameTest, YawRotation90Degrees)
{
  // Rotate 90 degrees about Z-axis (yaw)
  EulerAngles euler{
    Angle::fromRadians(0.0),      // pitch
    Angle::fromRadians(0.0),      // roll
    Angle::fromRadians(M_PI / 2)  // yaw = 90 degrees
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, euler};

  // Point at (1, 0, 0) in global frame
  Coordinate globalCoord{1.0, 0.0, 0.0};
  Coordinate localCoord = frame.globalToLocalAbsolute(globalCoord);

  // After 90 degree yaw rotation, should be at (0, -1, 0) in local frame
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.y(), -1.0, 1e-9))
    << localCoord.y();
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.z(), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, PitchRotation90Degrees)
{
  // Rotate 90 degrees about Y-axis (pitch)
  EulerAngles euler{
    Angle::fromRadians(M_PI / 2),  // pitch = 90 degrees
    Angle::fromRadians(0.0),       // roll
    Angle::fromRadians(0.0)        // yaw
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, euler};

  // Point at (1, 0, 0) in global frame
  Coordinate globalCoord{1.0, 0.0, 0.0};
  Coordinate localCoord = frame.globalToLocalAbsolute(globalCoord);

  // After 90 degree pitch rotation, should be at (0, 0, 1) in local frame
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.y(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.z(), 1.0, 1e-9));
}

TEST(ReferenceFrameTest, RollRotation90Degrees)
{
  // Rotate 90 degrees about X-axis (roll)
  EulerAngles euler{
    Angle::fromRadians(0.0),       // pitch
    Angle::fromRadians(M_PI / 2),  // roll = 90 degrees
    Angle::fromRadians(0.0)        // yaw
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, euler};

  // Point at (0, 1, 0) in global frame
  Coordinate globalCoord{0.0, 1.0, 0.0};
  Coordinate localCoord = frame.globalToLocalAbsolute(globalCoord);

  // After 90 degree roll rotation, should be at (0, 0, -1) in local frame
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.y(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.z(), -1.0, 1e-9));
}

TEST(ReferenceFrameTest, RotationOnlyRoundTrip)
{
  EulerAngles euler{
    Angle::fromRadians(0.3),  // pitch
    Angle::fromRadians(0.5),  // roll
    Angle::fromRadians(0.7)   // yaw
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, euler};

  Coordinate original{1.5, 2.5, 3.5};

  Coordinate local = frame.globalToLocalAbsolute(original);
  Coordinate global = frame.localToGlobalAbsolute(local);

  EXPECT_TRUE(coordinatesEqual(global, original, 1e-9));
}

// ============================================================================
// Combined Translation and Rotation Tests
// ============================================================================

TEST(ReferenceFrameTest, TranslationAndRotationGlobalToLocal)
{
  Coordinate origin{10.0, 20.0, 30.0};
  EulerAngles euler{
    Angle::fromRadians(0.0),      // pitch
    Angle::fromRadians(0.0),      // roll
    Angle::fromRadians(M_PI / 2)  // yaw = 90 degrees
  };
  ReferenceFrame frame{origin, euler};

  // Point at (11, 20, 30) in global frame
  Coordinate globalCoord{11.0, 20.0, 30.0};
  Coordinate localCoord = frame.globalToLocalAbsolute(globalCoord);

  // First translate: (11, 20, 30) - (10, 20, 30) = (1, 0, 0)
  // Then rotate 90 degrees about Z: (1, 0, 0) -> (0, -1, 0)
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.x(), 0.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.y(), -1.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(localCoord.z(), 0.0, 1e-9));
}

TEST(ReferenceFrameTest, TranslationAndRotationLocalToGlobal)
{
  Coordinate origin{10.0, 20.0, 30.0};
  EulerAngles euler{
    Angle::fromRadians(0.0),      // pitch
    Angle::fromRadians(0.0),      // roll
    Angle::fromRadians(M_PI / 2)  // yaw = 90 degrees
  };
  ReferenceFrame frame{origin, euler};

  // Point at (0, -1, 0) in local frame
  Coordinate localCoord{0.0, -1.0, 0.0};
  Coordinate globalCoord = frame.localToGlobalAbsolute(localCoord);

  // First rotate back: (0, -1, 0) -> (1, 0, 0)
  // Then translate: (1, 0, 0) + (10, 20, 30) = (11, 20, 30)
  EXPECT_TRUE(msd_sim::almostEqual(globalCoord.x(), 11.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalCoord.y(), 20.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(globalCoord.z(), 30.0, 1e-9));
}

TEST(ReferenceFrameTest, ComplexTransformationRoundTrip)
{
  Coordinate origin{5.5, 10.3, -2.7};
  EulerAngles euler{
    Angle::fromRadians(0.4),   // pitch
    Angle::fromRadians(-0.3),  // roll
    Angle::fromRadians(1.2)    // yaw
  };
  ReferenceFrame frame{origin, euler};

  Coordinate original{100.0, 200.0, 300.0};

  Coordinate local = frame.globalToLocalAbsolute(original);
  Coordinate global = frame.localToGlobalAbsolute(local);

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
  Coordinate localCoord = frame.globalToLocalAbsolute(origin);

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
  Coordinate localCoord = frame.globalToLocalAbsolute(globalCoord);

  EXPECT_DOUBLE_EQ(localCoord.x(), 2.0);
  EXPECT_DOUBLE_EQ(localCoord.y(), 2.0);
  EXPECT_DOUBLE_EQ(localCoord.z(), 3.0);
}

TEST(ReferenceFrameTest, LargeRotationAngles)
{
  // Test with angles that require normalization
  EulerAngles euler{
    Angle::fromRadians(3.5 * M_PI),   // pitch
    Angle::fromRadians(-2.7 * M_PI),  // roll
    Angle::fromRadians(5.1 * M_PI)    // yaw
  };
  ReferenceFrame frame{Coordinate{0, 0, 0}, euler};

  Coordinate original{1.0, 2.0, 3.0};

  Coordinate local = frame.globalToLocalAbsolute(original);
  Coordinate global = frame.localToGlobalAbsolute(local);

  EXPECT_TRUE(coordinatesEqual(global, original, 1e-9));
}

TEST(ReferenceFrameTest, ZeroVector)
{
  Coordinate origin{5.0, 10.0, 15.0};
  EulerAngles euler{
    Angle::fromRadians(0.5), Angle::fromRadians(0.3), Angle::fromRadians(0.7)};
  ReferenceFrame frame{origin, euler};

  Coordinate zeroVec{0.0, 0.0, 0.0};

  Coordinate local = frame.globalToLocalAbsolute(zeroVec);
  Coordinate global = frame.localToGlobalAbsolute(local);

  EXPECT_TRUE(coordinatesEqual(global, zeroVec, 1e-9));
}

// ============================================================================
// Multiple Frame Chaining Tests
// ============================================================================

TEST(ReferenceFrameTest, ChainedFrameTransformations)
{
  // Create two reference frames
  Coordinate origin1{10.0, 0.0, 0.0};
  EulerAngles euler1{Angle::fromRadians(0.0),
                     Angle::fromRadians(0.0),
                     Angle::fromRadians(M_PI / 4)};
  ReferenceFrame frame1{origin1, euler1};

  Coordinate origin2{5.0, 0.0, 0.0};  // Relative to frame1
  // euler2 declared but unused - test focuses on frame1 operations
  // EulerAngles euler2{...}

  // Point in global frame
  Coordinate globalPoint{20.0, 5.0, 0.0};

  // Transform to frame1
  Coordinate inFrame1 = frame1.globalToLocalAbsolute(globalPoint);

  // The origin of frame2 in frame1's local coordinates
  Coordinate frame2Origin = origin2;

  // Transform back to global should match
  Coordinate backToGlobal = frame1.localToGlobalAbsolute(inFrame1);

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
  EulerAngles aircraftAttitude{
    Angle::fromDegrees(10.0),  // pitch up 10 degrees
    Angle::fromDegrees(0.0),   // no roll
    Angle::fromDegrees(45.0)   // heading 45 degrees
  };
  ReferenceFrame bodyFrame{aircraftPos, aircraftAttitude};

  // A point 10 meters ahead of the aircraft in its body frame
  Coordinate pointInBody{10.0, 0.0, 0.0};

  // Transform to global frame
  Coordinate pointInGlobal = bodyFrame.localToGlobalAbsolute(pointInBody);

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
  EulerAngles robotOrientation{
    Angle::fromDegrees(0.0), Angle::fromDegrees(0.0), Angle::fromDegrees(90.0)};
  ReferenceFrame robotFrame{robotPos, robotOrientation};

  // Sensor mounted 1 meter in front and 0.5 meters up
  Coordinate sensorInRobotFrame{1.0, 0.0, 0.5};

  // Get sensor position in global frame
  Coordinate sensorInGlobal =
    robotFrame.localToGlobalAbsolute(sensorInRobotFrame);

  // Sensor should be roughly at (5, 6, 0.5) in global frame
  // (1 meter north of robot position)
  EXPECT_TRUE(msd_sim::almostEqual(sensorInGlobal.x(), 5.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(sensorInGlobal.y(), 6.0, 1e-9));
  EXPECT_TRUE(msd_sim::almostEqual(sensorInGlobal.z(), 0.5, 1e-9));
}
