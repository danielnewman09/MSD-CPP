// Ticket: 0038_simulation_data_recorder
// Purpose: Profile the actual cost of toRecord() and data recording operations

#include <benchmark/benchmark.h>
#include <filesystem>
#include <random>

#include "msd-sim/src/DataRecorder/DataRecorder.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

// Generate a randomized InertialState for realistic benchmarking
InertialState generateRandomState(std::mt19937& rng)
{
  std::uniform_real_distribution<double> posDist{-1000.0, 1000.0};
  std::uniform_real_distribution<double> velDist{-100.0, 100.0};
  std::uniform_real_distribution<double> accDist{-10.0, 10.0};
  std::uniform_real_distribution<double> quatDist{-1.0, 1.0};

  InertialState state;
  state.position = Coordinate{posDist(rng), posDist(rng), posDist(rng)};
  state.velocity = Vector3D{velDist(rng), velDist(rng), velDist(rng)};
  state.acceleration = Vector3D{accDist(rng), accDist(rng), accDist(rng)};

  // Generate random quaternion and normalize
  double w = quatDist(rng);
  double x = quatDist(rng);
  double y = quatDist(rng);
  double z = quatDist(rng);
  double norm = std::sqrt(w * w + x * x + y * y + z * z);
  state.orientation = QuaternionD{w / norm, x / norm, y / norm, z / norm};

  state.quaternionRate =
    Vector4D{quatDist(rng), quatDist(rng), quatDist(rng), quatDist(rng)};
  state.angularAcceleration =
    AngularRate{accDist(rng), accDist(rng), accDist(rng)};

  return state;
}

// Create temp database path for benchmarks
std::filesystem::path getTempDbPath()
{
  static int counter = 0;
  return std::filesystem::temp_directory_path() /
         ("bench_recorder_" + std::to_string(counter++) + ".db");
}

}  // namespace

// ============================================================================
// Benchmarks: Isolate toRecord() Cost
// ============================================================================

/**
 * @brief Benchmark InertialState::toRecord() in isolation
 *
 * This measures ONLY the cost of converting InertialState to InertialStateRecord.
 * This is a pure data copy operation with no I/O or synchronization.
 *
 * @ticket 0038_simulation_data_recorder
 */
static void BM_InertialState_ToRecord(benchmark::State& state)
{
  std::mt19937 rng{42};
  InertialState inertialState = generateRandomState(rng);

  for (auto _ : state)
  {
    auto record = inertialState.toRecord();
    benchmark::DoNotOptimize(record);
  }
}
BENCHMARK(BM_InertialState_ToRecord);

/**
 * @brief Benchmark toRecord() for multiple states (batch conversion)
 *
 * Measures the cost of converting N InertialStates to records.
 * Simulates the per-frame cost for worlds with varying object counts.
 *
 * @ticket 0038_simulation_data_recorder
 */
static void BM_InertialState_ToRecord_Batch(benchmark::State& state)
{
  std::mt19937 rng{42};
  const auto count = static_cast<size_t>(state.range(0));

  // Pre-generate states
  std::vector<InertialState> states;
  states.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    states.push_back(generateRandomState(rng));
  }

  for (auto _ : state)
  {
    for (const auto& s : states)
    {
      auto record = s.toRecord();
      benchmark::DoNotOptimize(record);
    }
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_InertialState_ToRecord_Batch)
  ->Args({1})
  ->Args({10})
  ->Args({100})
  ->Args({1000})
  ->Complexity();

// ============================================================================
// Benchmarks: addToBuffer() Cost (Mutex Acquisition)
// ============================================================================

/**
 * @brief Benchmark addToBuffer() in isolation (single record)
 *
 * Measures the cost of adding a single record to the DAO buffer.
 * This includes mutex acquisition and vector push_back.
 *
 * @ticket 0038_simulation_data_recorder
 */
static void BM_AddToBuffer_Single(benchmark::State& state)
{
  auto dbPath = getTempDbPath();
  DataRecorder::Config config;
  config.databasePath = dbPath.string();
  config.flushInterval = std::chrono::hours{1};  // Prevent background flush

  DataRecorder recorder{config};
  auto& dao = recorder.getDAO<msd_transfer::InertialStateRecord>();

  std::mt19937 rng{42};
  InertialState inertialState = generateRandomState(rng);
  auto record = inertialState.toRecord();
  record.frame.id = 1;

  for (auto _ : state)
  {
    dao.addToBuffer(record);
  }

  // Cleanup
  std::filesystem::remove(dbPath);
}
BENCHMARK(BM_AddToBuffer_Single);

/**
 * @brief Benchmark addToBuffer() for batch of records
 *
 * Measures the cost of adding N records to the DAO buffer sequentially.
 * Each call acquires the mutex independently.
 *
 * @ticket 0038_simulation_data_recorder
 */
static void BM_AddToBuffer_Batch(benchmark::State& state)
{
  auto dbPath = getTempDbPath();
  DataRecorder::Config config;
  config.databasePath = dbPath.string();
  config.flushInterval = std::chrono::hours{1};

  DataRecorder recorder{config};
  auto& dao = recorder.getDAO<msd_transfer::InertialStateRecord>();

  std::mt19937 rng{42};
  const auto count = static_cast<size_t>(state.range(0));

  // Pre-generate records
  std::vector<msd_transfer::InertialStateRecord> records;
  records.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    auto record = generateRandomState(rng).toRecord();
    record.frame.id = 1;
    records.push_back(record);
  }

  for (auto _ : state)
  {
    for (const auto& r : records)
    {
      dao.addToBuffer(r);
    }
    state.PauseTiming();
    dao.clearBuffer();
    state.ResumeTiming();
  }

  std::filesystem::remove(dbPath);
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_AddToBuffer_Batch)
  ->Args({1})
  ->Args({10})
  ->Args({100})
  ->Args({1000})
  ->Complexity();

// ============================================================================
// Benchmarks: Full Recording Pipeline
// ============================================================================

/**
 * @brief Benchmark complete recording pipeline (toRecord + addToBuffer)
 *
 * Measures the total cost of the current blocking implementation:
 * 1. Convert InertialState to InertialStateRecord
 * 2. Set frame ID
 * 3. Add to buffer
 *
 * This is what recordCurrentFrame() does per asset.
 *
 * @ticket 0038_simulation_data_recorder
 */
static void BM_FullRecordingPipeline_PerAsset(benchmark::State& state)
{
  auto dbPath = getTempDbPath();
  DataRecorder::Config config;
  config.databasePath = dbPath.string();
  config.flushInterval = std::chrono::hours{1};

  DataRecorder recorder{config};
  auto& dao = recorder.getDAO<msd_transfer::InertialStateRecord>();

  std::mt19937 rng{42};
  InertialState inertialState = generateRandomState(rng);

  uint32_t frameId = 1;
  for (auto _ : state)
  {
    // This is exactly what recordCurrentFrame() does per asset
    auto record = inertialState.toRecord();
    record.frame.id = frameId;
    dao.addToBuffer(record);
    benchmark::DoNotOptimize(record);
  }

  std::filesystem::remove(dbPath);
}
BENCHMARK(BM_FullRecordingPipeline_PerAsset);

/**
 * @brief Benchmark complete frame recording with N assets
 *
 * Simulates recordCurrentFrame() for worlds with varying object counts.
 * Includes recordFrame() call for the frame record itself.
 *
 * @ticket 0038_simulation_data_recorder
 */
static void BM_FullRecordingPipeline_Frame(benchmark::State& state)
{
  auto dbPath = getTempDbPath();
  DataRecorder::Config config;
  config.databasePath = dbPath.string();
  config.flushInterval = std::chrono::hours{1};

  DataRecorder recorder{config};
  auto& dao = recorder.getDAO<msd_transfer::InertialStateRecord>();

  std::mt19937 rng{42};
  const auto assetCount = static_cast<size_t>(state.range(0));

  // Pre-generate states (simulating assets)
  std::vector<InertialState> states;
  states.reserve(assetCount);
  for (size_t i = 0; i < assetCount; ++i)
  {
    states.push_back(generateRandomState(rng));
  }

  double simTime = 0.0;
  for (auto _ : state)
  {
    // Record frame (atomic ID + addToBuffer for frame record)
    uint32_t frameId = recorder.recordFrame(simTime);

    // Record all asset states
    for (const auto& s : states)
    {
      auto record = s.toRecord();
      record.frame.id = frameId;
      dao.addToBuffer(record);
    }

    simTime += 0.016;  // 60 FPS

    // Prevent buffer from growing unbounded
    state.PauseTiming();
    dao.clearBuffer();
    state.ResumeTiming();
  }

  std::filesystem::remove(dbPath);
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_FullRecordingPipeline_Frame)
  ->Args({1})
  ->Args({10})
  ->Args({100})
  ->Args({1000})
  ->Complexity();

// ============================================================================
// Benchmarks: Comparison - What Non-Blocking Could Achieve
// ============================================================================

/**
 * @brief Benchmark local batching (no mutex per record)
 *
 * This simulates what a non-blocking approach could achieve by:
 * 1. Building records locally without mutex
 * 2. Single batch insert to buffer
 *
 * Compares against BM_FullRecordingPipeline_Frame to show potential gains.
 *
 * Note: This requires a hypothetical addBatchToBuffer() API that doesn't exist
 * yet in cpp_sqlite. For now, we simulate the cost without the batch API.
 *
 * @ticket 0038_simulation_data_recorder
 */
static void BM_LocalBatching_RecordConversion(benchmark::State& state)
{
  std::mt19937 rng{42};
  const auto assetCount = static_cast<size_t>(state.range(0));

  // Pre-generate states
  std::vector<InertialState> states;
  states.reserve(assetCount);
  for (size_t i = 0; i < assetCount; ++i)
  {
    states.push_back(generateRandomState(rng));
  }

  uint32_t frameId = 1;
  for (auto _ : state)
  {
    // Build records locally (no mutex)
    std::vector<msd_transfer::InertialStateRecord> records;
    records.reserve(assetCount);

    for (const auto& s : states)
    {
      auto record = s.toRecord();
      record.frame.id = frameId;
      records.push_back(std::move(record));
    }

    benchmark::DoNotOptimize(records);
  }

  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_LocalBatching_RecordConversion)
  ->Args({1})
  ->Args({10})
  ->Args({100})
  ->Args({1000})
  ->Complexity();
