// Ticket: 0056k_example_workflow_test_recording
// Ticket: 0056e_threejs_core_visualization (R0b - Use Engine + AssetRegistry)
// Generate test recording database for replay system demonstration

#include <chrono>
#include <filesystem>
#include <iostream>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/Engine.hpp"

using namespace msd_sim;

int main(int argc, char** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <assets.db> <output.db>\n";
    std::cerr << "  <assets.db>: Path to asset database (input)\n";
    std::cerr << "  <output.db>: Path to recording database (output)\n";
    return 1;
  }

  std::string assetsDbPath = argv[1];
  std::string outputPath = argv[2];

  // Verify assets database exists
  if (!std::filesystem::exists(assetsDbPath)) {
    std::cerr << "Error: Asset database not found: " << assetsDbPath << "\n";
    std::cerr << "Please run generate_assets first to create the asset database.\n";
    return 1;
  }

  std::cout << "Asset database: " << assetsDbPath << "\n";
  std::cout << "Generating test recording: " << outputPath << "\n";

  // Create engine with asset database
  // Note: Engine constructor automatically spawns a floor at z=-60
  Engine engine{assetsDbPath};

  // Enable recording with explicit flush interval
  engine.getWorldModel().enableRecording(outputPath, std::chrono::milliseconds{100});

  // Spawn falling cube: 1×1×1 cube at z=2 above the floor
  engine.spawnInertialObject("cube", Coordinate{0.0, 0.0, 2.0}, AngularCoordinate{});

  // Run simulation: 300 timesteps at 10ms = 3 seconds
  // Cube should fall, bounce, and settle
  std::cout << "Running simulation (300 frames @ 10ms)...\n";
  for (int i = 0; i < 300; ++i) {
    engine.update(std::chrono::milliseconds{(i + 1) * 10});

    if ((i + 1) % 50 == 0) {
      std::cout << "  Frame " << (i + 1) << "/300\n";
    }
  }

  // Disable recording (flushes pending records)
  engine.getWorldModel().disableRecording();

  std::cout << "Recording complete: " << outputPath << "\n";
  return 0;
}
