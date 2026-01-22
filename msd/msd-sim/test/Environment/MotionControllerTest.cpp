// Ticket: 0005_camera_controller_sim
// Tests for MotionController functionality

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>

#include "msd-sim/src/Agent/InputCommands.hpp"
#include "msd-sim/src/Environment/Angle.hpp"
#include "msd-sim/src/Environment/AngularCoordinate.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/MotionController.hpp"
#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Utils/utils.hpp"

using namespace msd_sim;

namespace
{
// Helper to create a minimal valid ConvexHull for Platform construction
ConvexHull createTestHull()
{
  // Minimal tetrahedron for a valid 3D hull
  std::vector<Coordinate> points = {Coordinate{0.0, 0.0, 0.0},
                                    Coordinate{1.0, 0.0, 0.0},
                                    Coordinate{0.5, 1.0, 0.0},
                                    Coordinate{0.5, 0.5, 1.0}};
  return ConvexHull{points};
}
}  // namespace

// Motion controller uses float for move speed but double for delta time
// calculations, which introduces minor floating point errors. Use a relaxed
// tolerance for position tests.
constexpr double MOTION_TOLERANCE = 1e-7;

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(MotionControllerTest, ConstructorWithParameters)
{
  double rotSpeed = 0.1;
  double moveSpeed = 5.0;

  MotionController controller{rotSpeed, moveSpeed};

  EXPECT_DOUBLE_EQ(controller.getMoveSpeed(), 5.0);
  EXPECT_DOUBLE_EQ(controller.getRotationSpeed(), 0.1);
  EXPECT_DOUBLE_EQ(controller.getSensitivity(), 1.0);
}

// ============================================================================
// Movement Tests
// ============================================================================

TEST(MotionControllerTest, UpdateTransform_MoveForward_TranslatesInLocalZ)
{
  MotionController controller{0.0, 1.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveForward = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();

  // Move forward in local -Z direction (camera convention)
  // deltaTime = 100ms = 0.1s, moveSpeed = 1.0, sensitivity = 1.0
  // Expected movement = 1.0 * 1.0 * 0.1 = 0.1 in -Z
  EXPECT_NEAR(newOrigin.x(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.y(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.z(), -0.1, MOTION_TOLERANCE);
}

TEST(MotionControllerTest,
     UpdateTransform_MoveBackward_TranslatesInPositiveLocalZ)
{
  MotionController controller{0.0, 1.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveBackward = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();

  // Move backward in local +Z direction
  EXPECT_NEAR(newOrigin.x(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.y(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.z(), 0.1, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, UpdateTransform_MoveLeft_TranslatesInNegativeLocalX)
{
  MotionController controller{0.0, 1.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveLeft = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();

  // Move left in local -X direction
  EXPECT_NEAR(newOrigin.x(), -0.1, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.y(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.z(), 0.0, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, UpdateTransform_MoveRight_TranslatesInPositiveLocalX)
{
  MotionController controller{0.0, 1.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveRight = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();

  // Move right in local +X direction
  EXPECT_NEAR(newOrigin.x(), 0.1, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.y(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.z(), 0.0, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, UpdateTransform_MoveUp_TranslatesInPositiveLocalY)
{
  MotionController controller{0.0, 1.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveUp = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();

  // Move up in local +Y direction
  EXPECT_NEAR(newOrigin.x(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.y(), 0.1, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.z(), 0.0, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, UpdateTransform_MoveDown_TranslatesInNegativeLocalY)
{
  MotionController controller{0.0, 1.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveDown = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();

  // Move down in local -Y direction
  EXPECT_NEAR(newOrigin.x(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.y(), -0.1, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.z(), 0.0, MOTION_TOLERANCE);
}

// ============================================================================
// Rotation Tests
// ============================================================================

TEST(MotionControllerTest, UpdateTransform_PitchUp_IncreasesEulerPitch)
{
  MotionController controller{1.0, 0.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.pitchUp = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  AngularCoordinate newAngles = frame.getAngularCoordinate();

  // rotSpeed = 1.0 rad/s, deltaTime = 100ms = 0.1s
  // Expected pitch change = 1.0 * 0.1 = 0.1 rad
  EXPECT_NEAR(newAngles.pitch(), 0.1, MOTION_TOLERANCE);
  EXPECT_NEAR(newAngles.roll(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newAngles.yaw(), 0.0, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, UpdateTransform_PitchDown_DecreasesEulerPitch)
{
  MotionController controller{1.0, 0.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.pitchDown = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  AngularCoordinate newAngles = frame.getAngularCoordinate();

  // Expected pitch change = -1.0 * 0.1 = -0.1 rad
  EXPECT_NEAR(newAngles.pitch(), -0.1, MOTION_TOLERANCE);
  EXPECT_NEAR(newAngles.roll(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newAngles.yaw(), 0.0, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, UpdateTransform_YawLeft_IncreasesEulerYaw)
{
  MotionController controller{1.0, 0.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.yawLeft = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  AngularCoordinate newAngles = frame.getAngularCoordinate();

  // Expected yaw change = 1.0 * 0.1 = 0.1 rad
  EXPECT_NEAR(newAngles.pitch(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newAngles.roll(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newAngles.yaw(), 0.1, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, UpdateTransform_YawRight_DecreasesEulerYaw)
{
  MotionController controller{1.0, 0.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.yawRight = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  AngularCoordinate newAngles = frame.getAngularCoordinate();

  // Expected yaw change = -1.0 * 0.1 = -0.1 rad
  EXPECT_NEAR(newAngles.pitch(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newAngles.roll(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newAngles.yaw(), -0.1, MOTION_TOLERANCE);
}

// ============================================================================
// Frame-Rate Independence Tests
// ============================================================================

TEST(MotionControllerTest,
     UpdateTransform_ScaledByDeltaTime_FrameRateIndependent)
{
  MotionController controller{0.0, 10.0};
  ReferenceFrame frame1;
  frame1.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame1.setRotation(AngularCoordinate{});

  ReferenceFrame frame2;
  frame2.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame2.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveForward = true;

  // Simulate 1 second with different frame rates
  // Case 1: 10 updates at 100ms each (10 Hz)
  for (int i = 0; i < 10; ++i)
  {
    controller.updateTransform(
      frame1, commands, std::chrono::milliseconds{100});
  }

  // Case 2: 100 updates at 10ms each (100 Hz)
  for (int i = 0; i < 100; ++i)
  {
    controller.updateTransform(frame2, commands, std::chrono::milliseconds{10});
  }

  Coordinate pos1 = frame1.getOrigin();
  Coordinate pos2 = frame2.getOrigin();

  // Both should have moved the same total distance
  // Total time = 1.0s, speed = 10.0
  // Expected distance = 10.0
  EXPECT_NEAR(pos1.x(), pos2.x(), 0.01);
  EXPECT_NEAR(pos1.y(), pos2.y(), 0.01);
  EXPECT_NEAR(pos1.z(), pos2.z(), 0.01);

  // Check approximate expected value
  EXPECT_NEAR(pos1.z(), -10.0, 0.01);
}

// ============================================================================
// Setter/Getter Tests
// ============================================================================

TEST(MotionControllerTest, SetMoveSpeed_UpdatesSpeed_AffectsTranslation)
{
  MotionController controller{0.0, 1.0};
  controller.setMoveSpeed(5.0);

  EXPECT_DOUBLE_EQ(controller.getMoveSpeed(), 5.0);

  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveForward = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();

  // moveSpeed = 5.0, deltaTime = 0.1s, sensitivity = 1.0
  // Expected movement = 5.0 * 1.0 * 0.1 = 0.5
  EXPECT_NEAR(newOrigin.z(), -0.5, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, SetRotationSpeed_UpdatesSpeed_AffectsRotation)
{
  MotionController controller{0.5, 0.0};
  controller.setRotationSpeed(2.0);

  EXPECT_DOUBLE_EQ(controller.getRotationSpeed(), 2.0);

  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.pitchUp = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  AngularCoordinate newAngles = frame.getAngularCoordinate();

  // rotSpeed = 2.0 rad/s, deltaTime = 0.1s
  // Expected rotation = 2.0 * 0.1 = 0.2 rad
  EXPECT_NEAR(newAngles.pitch(), 0.2, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, SetSensitivity_UpdatesSensitivity_ScalesMovement)
{
  MotionController controller{0.0, 1.0};
  controller.setSensitivity(2.0);

  EXPECT_DOUBLE_EQ(controller.getSensitivity(), 2.0);

  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveForward = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();

  // moveSpeed = 1.0, sensitivity = 2.0, deltaTime = 0.1s
  // Expected movement = 1.0 * 2.0 * 0.1 = 0.2
  EXPECT_NEAR(newOrigin.z(), -0.2, MOTION_TOLERANCE);
}

// ============================================================================
// Platform Integration Tests
// ============================================================================

TEST(MotionControllerTest, Platform_GetMotionController_ReturnsReference)
{
  ConvexHull hull = createTestHull();
  ReferenceFrame frame;
  Platform platform{0, 0, 0, hull, 1.0, frame};
  MotionController& controller = platform.getMotionController();

  // Verify we can modify through reference
  controller.setMoveSpeed(99.0);

  EXPECT_DOUBLE_EQ(platform.getMotionController().getMoveSpeed(), 99.0);
}

TEST(MotionControllerTest,
     Platform_GetMotionController_ConstVersion_ReturnsConstReference)
{
  ConvexHull hull = createTestHull();
  ReferenceFrame frame;
  Platform platform{0, 0, 0, hull, 1.0, frame};
  const Platform& constPlatform = platform;

  const MotionController& controller = constPlatform.getMotionController();

  // Should be able to read default values
  EXPECT_DOUBLE_EQ(controller.getMoveSpeed(), 0.1);
  EXPECT_DOUBLE_EQ(controller.getRotationSpeed(), 0.05);
}

// ============================================================================
// Combined Movement Tests
// ============================================================================

TEST(MotionControllerTest, UpdateTransform_CombinedMovement_AppliesAllCommands)
{
  MotionController controller{1.0, 1.0};
  ReferenceFrame frame;
  frame.setOrigin(Coordinate{0.0, 0.0, 0.0});
  frame.setRotation(AngularCoordinate{});

  InputCommands commands;
  commands.moveForward = true;
  commands.moveRight = true;
  commands.pitchUp = true;

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();
  AngularCoordinate newAngles = frame.getAngularCoordinate();

  // Movement: forward (-Z) and right (+X)
  EXPECT_NEAR(newOrigin.x(), 0.1, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.y(), 0.0, MOTION_TOLERANCE);
  EXPECT_NEAR(newOrigin.z(), -0.1, MOTION_TOLERANCE);

  // Rotation: pitch up
  EXPECT_NEAR(newAngles.pitch(), 0.1, MOTION_TOLERANCE);
}

TEST(MotionControllerTest, UpdateTransform_NoCommands_NoChange)
{
  MotionController controller{1.0, 1.0};
  ReferenceFrame frame;
  Coordinate initialOrigin{5.0, 10.0, 15.0};
  AngularCoordinate initialAngles{
    10.0 * M_PI / 180.0, 20.0 * M_PI / 180.0, 30.0 * M_PI / 180.0};
  frame.setOrigin(initialOrigin);
  frame.setRotation(initialAngles);

  InputCommands commands;  // All false

  auto deltaTime = std::chrono::milliseconds{100};
  controller.updateTransform(frame, commands, deltaTime);

  Coordinate newOrigin = frame.getOrigin();
  AngularCoordinate newAngles = frame.getAngularCoordinate();

  // No change expected
  EXPECT_NEAR(newOrigin.x(), 5.0, TOLERANCE);
  EXPECT_NEAR(newOrigin.y(), 10.0, TOLERANCE);
  EXPECT_NEAR(newOrigin.z(), 15.0, TOLERANCE);
  EXPECT_NEAR(newAngles.pitch(), initialAngles.pitch(), TOLERANCE);
  EXPECT_NEAR(newAngles.roll(), initialAngles.roll(), TOLERANCE);
  EXPECT_NEAR(newAngles.yaw(), initialAngles.yaw(), TOLERANCE);
}
