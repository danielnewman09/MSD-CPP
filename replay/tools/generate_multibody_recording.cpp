// Ticket: 0075-unified-contact-constraint
// Generate multi-body collision recording databases for replay analysis.
// Uses Engine + test asset database so recordings render correctly in the
// replay tool (assets resolved by ID from the same database).
//
// Generates all three scenarios (stack, cluster, grid) into replay/recordings/.
// Engine auto-spawns a floor (100m cube at z=-60, top face at z=-10), so all
// spawn heights are relative to z=-10.
//
// Usage: generate_multibody_recording [num_bodies]
//   num_bodies defaults to 8 if not specified.

#include <cmath>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>

#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Engine.hpp"

using namespace msd_sim;

constexpr int kDefaultNumBodies = 8;
constexpr int kNumFrames = 100;       // Frames to simulate (1.6s at 16ms timestep)
constexpr double kCubeHalfSize = 0.5; // unit_cube is 1.0m → half-extent 0.5m
constexpr double kFloorZ = -10.0;     // Engine floor top face z-coordinate
constexpr double kMass = 10.0;
constexpr double kRestitution = 0.5;
constexpr double kFriction = 0.5;
constexpr unsigned int kRandomSeed = 42;

// ============================================================================
// Scenario setup functions (Engine-based)
// ============================================================================

// N cubes in a vertical stack with slight random offsets making it unstable.
// Stack collapses and bodies settle — sustained multi-contact resting + tumbling.
void setupStackCollapse(Engine& engine, int numBodies)
{
  std::mt19937 rng{kRandomSeed};
  std::uniform_real_distribution<double> offsetDist{-0.15, 0.15};

  for (int i = 0; i < numBodies; ++i)
  {
    double const stackZ =
      kFloorZ + kCubeHalfSize + static_cast<double>(i) * 2.0 * kCubeHalfSize;
    Coordinate position{offsetDist(rng), offsetDist(rng), stackZ};

    engine.spawnInertialObject(
      "unit_cube", position, AngularCoordinate{}, kMass, kRestitution, kFriction);
  }
}

// N cubes spawned in a tight random cluster above the floor.
// Each has a small random velocity. Creates dense body-body + body-floor contacts.
void setupClusterDrop(Engine& engine, int numBodies)
{
  double const clusterRadius =
    kCubeHalfSize * 2.0 * std::cbrt(static_cast<double>(numBodies));

  std::mt19937 rng{kRandomSeed};
  std::uniform_real_distribution<double> posDist{-clusterRadius, clusterRadius};
  std::uniform_real_distribution<double> velDist{-1.0, 1.0};

  for (int i = 0; i < numBodies; ++i)
  {
    double const dropHeight = kFloorZ + 3.0 + kCubeHalfSize;
    Coordinate position{
      posDist(rng), posDist(rng), dropHeight + std::abs(posDist(rng))};

    const auto& asset = engine.spawnInertialObject(
      "unit_cube", position, AngularCoordinate{}, kMass, kRestitution, kFriction);
    uint32_t id = asset.getInstanceId();
    engine.getWorldModel().getObject(id).getInertialState().velocity =
      Velocity{velDist(rng), velDist(rng), velDist(rng)};
  }
}

// N cubes in a grid pattern settling onto the floor.
// Many parallel body-floor contacts, minimal body-body interaction.
void setupGridSettle(Engine& engine, int numBodies)
{
  int const gridSide = static_cast<int>(std::ceil(std::sqrt(numBodies)));
  double const spacing = 3.0 * kCubeHalfSize * 2.0;
  double const gridOffset = -static_cast<double>(gridSide - 1) * spacing / 2.0;

  int spawned = 0;
  for (int row = 0; row < gridSide && spawned < numBodies; ++row)
  {
    for (int col = 0; col < gridSide && spawned < numBodies; ++col)
    {
      double const dropHeight = kFloorZ + 2.0 + kCubeHalfSize;
      Coordinate position{gridOffset + static_cast<double>(col) * spacing,
                          gridOffset + static_cast<double>(row) * spacing,
                          dropHeight};

      engine.spawnInertialObject(
        "unit_cube", position, AngularCoordinate{}, kMass, kRestitution, kFriction);
      ++spawned;
    }
  }
}

// ============================================================================
// Scenario table
// ============================================================================

struct Scenario
{
  std::string name;
  void (*setup)(Engine&, int);
};

static const Scenario kScenarios[] = {
  {"stack", setupStackCollapse},
  {"cluster", setupClusterDrop},
  {"grid", setupGridSettle},
};

// ============================================================================
// Scenario runner
// ============================================================================

void runScenario(const Scenario& scenario,
                 int numBodies,
                 const std::string& assetsDbPath,
                 const std::filesystem::path& outputDir)
{
  std::string filename =
    scenario.name + "_" + std::to_string(numBodies) + ".db";
  std::filesystem::path outputPath = outputDir / filename;

  std::cout << "\n=== " << scenario.name << " (" << numBodies
            << " bodies) ===\n";
  std::cout << "Output: " << outputPath << "\n";

  // Fresh engine per scenario (provides its own floor via constructor)
  Engine engine{assetsDbPath};
  engine.getWorldModel().enableRecording(outputPath.string());

  scenario.setup(engine, numBodies);

  std::cout << "Running simulation (" << kNumFrames << " frames @ 16ms)...\n";
  for (int i = 0; i < kNumFrames; ++i)
  {
    engine.update(std::chrono::milliseconds{(i + 1) * 16});

    if ((i + 1) % 20 == 0)
    {
      std::cout << "  Frame " << (i + 1) << "/" << kNumFrames << "\n";
    }
  }

  engine.getWorldModel().disableRecording();
  std::cout << "Done: " << outputPath << "\n";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv)
{
  if (argc > 2)
  {
    std::cerr << "Usage: " << argv[0] << " [num_bodies]\n";
    std::cerr << "  num_bodies: number of dynamic cubes (default: "
              << kDefaultNumBodies << ")\n";
    return 1;
  }

  int numBodies = kDefaultNumBodies;
  if (argc == 2)
  {
    numBodies = std::stoi(argv[1]);
    if (numBodies < 1)
    {
      std::cerr << "Error: num_bodies must be >= 1\n";
      return 1;
    }
  }

  // Locate test_assets.db relative to the executable
  // Executable is in build/{type}/debug/, test_assets.db is in build/{type}/
  std::filesystem::path exeDir =
    std::filesystem::path{argv[0]}.parent_path();
  std::filesystem::path assetsDbPath =
    std::filesystem::weakly_canonical(exeDir / "../test_assets.db");

  if (!std::filesystem::exists(assetsDbPath))
  {
    std::cerr << "Error: Asset database not found: " << assetsDbPath << "\n";
    std::cerr << "Please build the test_assets_db target first.\n";
    return 1;
  }

  // Output to replay/recordings/ relative to repo root
  std::filesystem::path outputDir =
    std::filesystem::weakly_canonical(exeDir / "../../../replay/recordings");
  std::filesystem::create_directories(outputDir);

  std::cout << "Generating multi-body recordings\n";
  std::cout << "Assets DB: " << assetsDbPath << "\n";
  std::cout << "Bodies:    " << numBodies << "\n";
  std::cout << "Directory: " << outputDir << "\n";

  for (const auto& scenario : kScenarios)
  {
    runScenario(scenario, numBodies, assetsDbPath.string(), outputDir);
  }

  std::cout << "\nAll recordings complete.\n";
  return 0;
}
