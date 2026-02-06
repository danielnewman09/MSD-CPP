// Ticket: 0038_simulation_data_recorder
// Test: DataRecorder unit tests

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <thread>

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>

#include "msd-sim/src/DataRecorder/DataRecorder.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"

namespace msd_sim
{
namespace test
{

class DataRecorderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create unique temp file for each test
    testDbPath_ = std::filesystem::temp_directory_path() /
                  ("data_recorder_test_" + std::to_string(testCounter_++) +
                   ".db");
  }

  void TearDown() override
  {
    // Clean up test database
    if (std::filesystem::exists(testDbPath_))
    {
      std::filesystem::remove(testDbPath_);
    }
  }

  std::filesystem::path testDbPath_;
  static int testCounter_;
};

int DataRecorderTest::testCounter_ = 0;

// ========== Construction/Destruction Tests ==========

TEST_F(DataRecorderTest, Constructor_CreatesDatabase)
{
  DataRecorder::Config config;
  config.databasePath = testDbPath_.string();
  config.flushInterval = std::chrono::milliseconds{50};

  DataRecorder recorder{config};

  // Database file should exist
  EXPECT_TRUE(std::filesystem::exists(testDbPath_));
}

TEST_F(DataRecorderTest, Constructor_InvalidPath_Throws)
{
  DataRecorder::Config config;
  config.databasePath = "/nonexistent/path/to/database.db";
  config.flushInterval = std::chrono::milliseconds{50};

  EXPECT_THROW(DataRecorder{config}, std::runtime_error);
}

TEST_F(DataRecorderTest, Destructor_FlushesRemainingRecords)
{
  {
    DataRecorder::Config config;
    config.databasePath = testDbPath_.string();
    config.flushInterval = std::chrono::milliseconds{1000};  // Long interval

    DataRecorder recorder{config};

    // Record a frame
    recorder.recordFrame(1.0);
    recorder.recordFrame(2.0);

    // Destructor should flush without waiting for interval
  }

  // Verify records were written
  cpp_sqlite::Database db{testDbPath_.string(), true};
  auto& frameDAO = db.getDAO<msd_transfer::SimulationFrameRecord>();
  auto frames = frameDAO.selectAll();

  EXPECT_EQ(frames.size(), 2);
}

// ========== recordFrame Tests ==========

TEST_F(DataRecorderTest, RecordFrame_ReturnsSequentialIds)
{
  DataRecorder::Config config;
  config.databasePath = testDbPath_.string();
  config.flushInterval = std::chrono::milliseconds{50};

  DataRecorder recorder{config};

  uint32_t id1 = recorder.recordFrame(0.0);
  uint32_t id2 = recorder.recordFrame(0.016);
  uint32_t id3 = recorder.recordFrame(0.032);

  EXPECT_EQ(id1, 1);
  EXPECT_EQ(id2, 2);
  EXPECT_EQ(id3, 3);
}

TEST_F(DataRecorderTest, RecordFrame_StoresCorrectTimestamp)
{
  DataRecorder::Config config;
  config.databasePath = testDbPath_.string();
  config.flushInterval = std::chrono::milliseconds{50};

  {
    DataRecorder recorder{config};
    recorder.recordFrame(1.5);
    recorder.recordFrame(2.5);
  }

  // Verify timestamps
  cpp_sqlite::Database db{testDbPath_.string(), true};
  auto& frameDAO = db.getDAO<msd_transfer::SimulationFrameRecord>();

  auto frame1 = frameDAO.selectById(1);
  auto frame2 = frameDAO.selectById(2);

  ASSERT_TRUE(frame1.has_value());
  ASSERT_TRUE(frame2.has_value());

  EXPECT_DOUBLE_EQ(frame1->simulation_time, 1.5);
  EXPECT_DOUBLE_EQ(frame2->simulation_time, 2.5);
}

// ========== getDAO Tests ==========

TEST_F(DataRecorderTest, GetDAO_ReturnsValidDAO)
{
  DataRecorder::Config config;
  config.databasePath = testDbPath_.string();
  config.flushInterval = std::chrono::milliseconds{50};

  DataRecorder recorder{config};

  // Should not throw
  auto& stateDAO = recorder.getDAO<msd_transfer::InertialStateRecord>();

  // Add a record to verify DAO works
  msd_transfer::InertialStateRecord record;
  record.position.x = 1.0;
  record.position.y = 2.0;
  record.position.z = 3.0;
  record.frame.id = 1;

  stateDAO.addToBuffer(record);

  // Flush and verify
  recorder.flush();

  auto selected = stateDAO.selectById(1);
  EXPECT_TRUE(selected.has_value());
}

// ========== flush Tests ==========

TEST_F(DataRecorderTest, Flush_WritesBufferedRecords)
{
  DataRecorder::Config config;
  config.databasePath = testDbPath_.string();
  config.flushInterval = std::chrono::milliseconds{10000};  // Very long

  DataRecorder recorder{config};

  // Record frames
  recorder.recordFrame(1.0);
  recorder.recordFrame(2.0);
  recorder.recordFrame(3.0);

  // Explicit flush
  recorder.flush();

  // Verify immediately (don't wait for background thread)
  auto& frameDAO =
    const_cast<cpp_sqlite::Database&>(recorder.getDatabase())
      .getDAO<msd_transfer::SimulationFrameRecord>();

  auto frames = frameDAO.selectAll();
  EXPECT_EQ(frames.size(), 3);
}

// ========== Background Thread Tests ==========

TEST_F(DataRecorderTest, BackgroundThread_FlushesAutomatically)
{
  DataRecorder::Config config;
  config.databasePath = testDbPath_.string();
  config.flushInterval = std::chrono::milliseconds{50};

  DataRecorder recorder{config};

  // Record frames
  recorder.recordFrame(1.0);
  recorder.recordFrame(2.0);

  // Wait for background thread to flush (2x interval to be safe)
  std::this_thread::sleep_for(std::chrono::milliseconds{150});

  // Verify records were written by background thread
  auto& frameDAO =
    const_cast<cpp_sqlite::Database&>(recorder.getDatabase())
      .getDAO<msd_transfer::SimulationFrameRecord>();

  auto frames = frameDAO.selectAll();
  EXPECT_EQ(frames.size(), 2);
}

// ========== Integration Tests ==========

TEST_F(DataRecorderTest, Integration_RecordFrameWithState)
{
  DataRecorder::Config config;
  config.databasePath = testDbPath_.string();
  config.flushInterval = std::chrono::milliseconds{50};

  {
    DataRecorder recorder{config};

    // Record frame and get ID
    uint32_t frameId = recorder.recordFrame(0.5);

    // Record state with FK to frame
    msd_transfer::InertialStateRecord state;
    state.position.x = 10.0;
    state.position.y = 20.0;
    state.position.z = 30.0;
    state.velocity.x = 1.0;
    state.velocity.y = 2.0;
    state.velocity.z = 3.0;
    state.frame.id = frameId;

    recorder.getDAO<msd_transfer::InertialStateRecord>().addToBuffer(state);
  }

  // Verify FK relationship
  cpp_sqlite::Database db{testDbPath_.string(), true};

  auto& frameDAO = db.getDAO<msd_transfer::SimulationFrameRecord>();
  auto& stateDAO = db.getDAO<msd_transfer::InertialStateRecord>();

  auto frame = frameDAO.selectById(1);
  auto state = stateDAO.selectById(1);

  ASSERT_TRUE(frame.has_value());
  ASSERT_TRUE(state.has_value());

  EXPECT_EQ(state->frame.id, frame->id);
  EXPECT_DOUBLE_EQ(state->position.x, 10.0);
}

TEST_F(DataRecorderTest, Integration_MultipleFramesWithStates)
{
  DataRecorder::Config config;
  config.databasePath = testDbPath_.string();
  config.flushInterval = std::chrono::milliseconds{50};

  constexpr int kNumFrames = 10;

  {
    DataRecorder recorder{config};

    for (int i = 0; i < kNumFrames; ++i)
    {
      double simTime = static_cast<double>(i) * 0.016;  // 60 FPS
      uint32_t frameId = recorder.recordFrame(simTime);

      msd_transfer::InertialStateRecord state;
      state.position.x = static_cast<double>(i);
      state.position.y = static_cast<double>(i) * 2;
      state.position.z = static_cast<double>(i) * 3;
      state.frame.id = frameId;

      recorder.getDAO<msd_transfer::InertialStateRecord>().addToBuffer(state);
    }
  }

  // Verify all records
  cpp_sqlite::Database db{testDbPath_.string(), true};

  auto& frameDAO = db.getDAO<msd_transfer::SimulationFrameRecord>();
  auto& stateDAO = db.getDAO<msd_transfer::InertialStateRecord>();

  auto frames = frameDAO.selectAll();
  auto states = stateDAO.selectAll();

  EXPECT_EQ(frames.size(), kNumFrames);
  EXPECT_EQ(states.size(), kNumFrames);

  // Verify FK references
  for (size_t i = 0; i < static_cast<size_t>(kNumFrames); ++i)
  {
    EXPECT_EQ(states[i].frame.id, frames[i].id);
  }
}

}  // namespace test
}  // namespace msd_sim
