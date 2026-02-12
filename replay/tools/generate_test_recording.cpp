// Ticket: 0056k_example_workflow_test_recording
// Generate test recording database for replay system demonstration

#include <chrono>
#include <iostream>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// Create a simple cube as a point cloud
std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate(-half, -half, -half),
          Coordinate(half, -half, -half),
          Coordinate(half, half, -half),
          Coordinate(-half, half, -half),
          Coordinate(-half, -half, half),
          Coordinate(half, -half, half),
          Coordinate(half, half, half),
          Coordinate(-half, half, half)};
}

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <output.db>\n";
    return 1;
  }

  std::string outputPath = argv[1];
  std::cout << "Generating test recording: " << outputPath << "\n";

  // Create world with floor and falling cube
  WorldModel world;

  // Floor: large cube at z=-50 so top face is at z=0
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(0, floorHull, floorFrame);

  // Falling cube: 1×1×1 cube at z=2
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 2.0}};
  world.spawnObject(1, cubeHull, cubeFrame);

  // Enable recording with explicit flush interval
  world.enableRecording(outputPath, std::chrono::milliseconds{100});

  // Run simulation: 300 timesteps at 10ms = 3 seconds
  // Cube should fall, bounce, and settle
  std::cout << "Running simulation (300 frames @ 10ms)...\n";
  for (int i = 0; i < 300; ++i) {
    world.update(std::chrono::milliseconds{(i + 1) * 10});

    if ((i + 1) % 50 == 0) {
      std::cout << "  Frame " << (i + 1) << "/300\n";
    }
  }

  // Disable recording (flushes pending records)
  world.disableRecording();

  std::cout << "Recording complete: " << outputPath << "\n";
  return 0;
}
