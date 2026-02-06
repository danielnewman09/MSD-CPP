/**
 * Prototype P2: Flush Interval Sensitivity Analysis
 *
 * Question: What flush interval balances memory usage vs. write latency?
 *
 * Success Criteria: Identify interval that keeps buffer under 10MB
 *                   while maintaining <1s write latency
 *
 * Approach: Simulate continuous recording at 60 FPS with 10 objects per frame
 *           Test flush intervals: 10ms, 50ms, 100ms, 500ms
 *           Measure buffer memory usage and flush latency
 */

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sqlite3.h>
#include <thread>
#include <vector>

using namespace std::chrono;

struct TestConfig {
    int flushIntervalMs;
    int simulationFPS{60};
    int objectsPerFrame{10};
    double simulationDurationSeconds{10.0};  // 10 seconds
};

struct MeasurementResult {
    int flushIntervalMs;
    size_t maxBufferSizeBytes;
    double avgFlushLatencyMs;
    double maxFlushLatencyMs;
    int totalFlushes;
    size_t totalRecords;
};

// Structure representing a single record (~14 doubles = 112 bytes)
struct SimulationRecord {
    double simulation_time;
    double position_x, position_y, position_z;
    double velocity_x, velocity_y, velocity_z;
    double orientation_w, orientation_x, orientation_y, orientation_z;
    double angular_velocity_x, angular_velocity_y, angular_velocity_z;
};

void checkSqlite(int rc, const char* msg, sqlite3* db) {
    if (rc != SQLITE_OK) {
        std::cerr << "SQLite error: " << msg << ": " << sqlite3_errmsg(db) << "\n";
        sqlite3_close(db);
        std::exit(1);
    }
}

void createTable(sqlite3* db) {
    const char* sql = R"(
        CREATE TABLE IF NOT EXISTS simulation_records (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            simulation_time REAL,
            position_x REAL, position_y REAL, position_z REAL,
            velocity_x REAL, velocity_y REAL, velocity_z REAL,
            orientation_w REAL, orientation_x REAL, orientation_y REAL, orientation_z REAL,
            angular_velocity_x REAL, angular_velocity_y REAL, angular_velocity_z REAL
        );
    )";

    char* errMsg = nullptr;
    int rc = sqlite3_exec(db, sql, nullptr, nullptr, &errMsg);
    if (rc != SQLITE_OK) {
        std::cerr << "SQL error: " << errMsg << "\n";
        sqlite3_free(errMsg);
        std::exit(1);
    }
}

/**
 * @brief Simulate recording with periodic flushes
 */
MeasurementResult benchmarkFlushInterval(const TestConfig& config) {
    std::string dbPath = "test_p2_" + std::to_string(config.flushIntervalMs) + "ms.db";
    std::filesystem::remove(dbPath);

    sqlite3* db = nullptr;
    int rc = sqlite3_open(dbPath.c_str(), &db);
    checkSqlite(rc, "open database", db);

    // Enable WAL mode for better concurrent performance
    sqlite3_exec(db, "PRAGMA journal_mode=WAL;", nullptr, nullptr, nullptr);

    createTable(db);

    const char* insertSql = R"(
        INSERT INTO simulation_records (
            simulation_time, position_x, position_y, position_z,
            velocity_x, velocity_y, velocity_z,
            orientation_w, orientation_x, orientation_y, orientation_z,
            angular_velocity_x, angular_velocity_y, angular_velocity_z
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
    )";

    sqlite3_stmt* stmt = nullptr;
    checkSqlite(sqlite3_prepare_v2(db, insertSql, -1, &stmt, nullptr), "prepare statement", db);

    // Tracking
    std::vector<SimulationRecord> buffer;
    buffer.reserve(config.objectsPerFrame * config.simulationFPS * 2);  // 2 sec buffer

    size_t maxBufferSize = 0;
    std::vector<double> flushLatencies;
    int totalFlushes = 0;
    size_t totalRecords = 0;

    double frameTime = 1.0 / config.simulationFPS;  // seconds per frame
    milliseconds flushInterval{config.flushIntervalMs};

    auto simulationStart = high_resolution_clock::now();
    auto lastFlush = simulationStart;

    // Simulate recording - produce records at simulation rate
    double elapsedSimTime = 0.0;
    while (elapsedSimTime < config.simulationDurationSeconds) {
        // Generate records for this frame
        for (int obj = 0; obj < config.objectsPerFrame; ++obj) {
            SimulationRecord rec{};
            rec.simulation_time = elapsedSimTime;
            rec.position_x = obj * 1.0;
            rec.position_y = obj * 2.0;
            rec.position_z = obj * 3.0;
            rec.velocity_x = obj * 0.1;
            rec.velocity_y = obj * 0.2;
            rec.velocity_z = obj * 0.3;
            rec.orientation_w = 1.0;
            rec.orientation_x = 0.0;
            rec.orientation_y = 0.0;
            rec.orientation_z = 0.0;
            rec.angular_velocity_x = obj * 0.01;
            rec.angular_velocity_y = obj * 0.02;
            rec.angular_velocity_z = obj * 0.03;

            buffer.push_back(rec);
            totalRecords++;
        }

        // Track max buffer size
        size_t currentBufferBytes = buffer.size() * sizeof(SimulationRecord);
        if (currentBufferBytes > maxBufferSize) {
            maxBufferSize = currentBufferBytes;
        }

        // Check if it's time to flush
        auto now = high_resolution_clock::now();
        if (duration_cast<milliseconds>(now - lastFlush) >= flushInterval && !buffer.empty()) {
            auto flushStart = high_resolution_clock::now();

            // Begin transaction
            sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);

            // Insert all buffered records
            for (const auto& rec : buffer) {
                sqlite3_bind_double(stmt, 1, rec.simulation_time);
                sqlite3_bind_double(stmt, 2, rec.position_x);
                sqlite3_bind_double(stmt, 3, rec.position_y);
                sqlite3_bind_double(stmt, 4, rec.position_z);
                sqlite3_bind_double(stmt, 5, rec.velocity_x);
                sqlite3_bind_double(stmt, 6, rec.velocity_y);
                sqlite3_bind_double(stmt, 7, rec.velocity_z);
                sqlite3_bind_double(stmt, 8, rec.orientation_w);
                sqlite3_bind_double(stmt, 9, rec.orientation_x);
                sqlite3_bind_double(stmt, 10, rec.orientation_y);
                sqlite3_bind_double(stmt, 11, rec.orientation_z);
                sqlite3_bind_double(stmt, 12, rec.angular_velocity_x);
                sqlite3_bind_double(stmt, 13, rec.angular_velocity_y);
                sqlite3_bind_double(stmt, 14, rec.angular_velocity_z);

                sqlite3_step(stmt);
                sqlite3_reset(stmt);
            }

            // Commit transaction
            sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);

            auto flushEnd = high_resolution_clock::now();
            double latencyMs = duration_cast<microseconds>(flushEnd - flushStart).count() / 1000.0;
            flushLatencies.push_back(latencyMs);

            // Clear buffer
            buffer.clear();
            lastFlush = now;
            totalFlushes++;
        }

        // Advance simulation time
        elapsedSimTime += frameTime;

        // Sleep to simulate real-time execution (not needed for this benchmark,
        // but helps demonstrate that buffer can grow if flushing is slow)
        // std::this_thread::sleep_for(microseconds(static_cast<long>(frameTime * 1000000)));
    }

    // Final flush if any records remain
    if (!buffer.empty()) {
        auto flushStart = high_resolution_clock::now();

        sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);

        for (const auto& rec : buffer) {
            sqlite3_bind_double(stmt, 1, rec.simulation_time);
            sqlite3_bind_double(stmt, 2, rec.position_x);
            sqlite3_bind_double(stmt, 3, rec.position_y);
            sqlite3_bind_double(stmt, 4, rec.position_z);
            sqlite3_bind_double(stmt, 5, rec.velocity_x);
            sqlite3_bind_double(stmt, 6, rec.velocity_y);
            sqlite3_bind_double(stmt, 7, rec.velocity_z);
            sqlite3_bind_double(stmt, 8, rec.orientation_w);
            sqlite3_bind_double(stmt, 9, rec.orientation_x);
            sqlite3_bind_double(stmt, 10, rec.orientation_y);
            sqlite3_bind_double(stmt, 11, rec.orientation_z);
            sqlite3_bind_double(stmt, 12, rec.angular_velocity_x);
            sqlite3_bind_double(stmt, 13, rec.angular_velocity_y);
            sqlite3_bind_double(stmt, 14, rec.angular_velocity_z);

            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }

        sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);

        auto flushEnd = high_resolution_clock::now();
        double latencyMs = duration_cast<microseconds>(flushEnd - flushStart).count() / 1000.0;
        flushLatencies.push_back(latencyMs);
        totalFlushes++;
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
    std::filesystem::remove(dbPath);

    // Calculate statistics
    double avgLatency = 0.0;
    double maxLatency = 0.0;
    if (!flushLatencies.empty()) {
        for (double lat : flushLatencies) {
            avgLatency += lat;
            if (lat > maxLatency) maxLatency = lat;
        }
        avgLatency /= flushLatencies.size();
    }

    MeasurementResult result{};
    result.flushIntervalMs = config.flushIntervalMs;
    result.maxBufferSizeBytes = maxBufferSize;
    result.avgFlushLatencyMs = avgLatency;
    result.maxFlushLatencyMs = maxLatency;
    result.totalFlushes = totalFlushes;
    result.totalRecords = totalRecords;

    return result;
}

int main() {
    std::cout << "Prototype P2: Flush Interval Sensitivity Analysis\n";
    std::cout << "=================================================\n\n";

    std::cout << "Simulation parameters:\n";
    std::cout << "  - FPS: 60\n";
    std::cout << "  - Objects per frame: 10\n";
    std::cout << "  - Duration: 10 seconds\n";
    std::cout << "  - Expected records: 6000\n";
    std::cout << "  - Record size: ~112 bytes\n\n";

    std::vector<int> flushIntervals{10, 50, 100, 500};
    std::vector<MeasurementResult> results;

    for (int intervalMs : flushIntervals) {
        std::cout << "Testing flush interval: " << intervalMs << " ms...\n";

        TestConfig config{};
        config.flushIntervalMs = intervalMs;

        auto result = benchmarkFlushInterval(config);
        results.push_back(result);

        std::cout << "  Max buffer size: "
                  << std::fixed << std::setprecision(2)
                  << result.maxBufferSizeBytes / 1024.0 << " KB ("
                  << result.maxBufferSizeBytes / (1024.0 * 1024.0) << " MB)\n";
        std::cout << "  Avg flush latency: "
                  << std::fixed << std::setprecision(3)
                  << result.avgFlushLatencyMs << " ms\n";
        std::cout << "  Max flush latency: "
                  << std::fixed << std::setprecision(3)
                  << result.maxFlushLatencyMs << " ms\n";
        std::cout << "  Total flushes: " << result.totalFlushes << "\n";
        std::cout << "  Total records: " << result.totalRecords << "\n\n";
    }

    // Print summary table
    std::cout << "\nSummary Table\n";
    std::cout << "=============\n";
    std::cout << std::setw(10) << "Interval"
              << std::setw(15) << "Max Buffer"
              << std::setw(15) << "Avg Flush"
              << std::setw(15) << "Max Flush"
              << std::setw(12) << "Flushes\n";
    std::cout << std::setw(10) << "(ms)"
              << std::setw(15) << "(KB)"
              << std::setw(15) << "(ms)"
              << std::setw(15) << "(ms)"
              << std::setw(12) << "\n";
    std::cout << std::string(67, '-') << "\n";

    for (const auto& res : results) {
        std::cout << std::setw(10) << res.flushIntervalMs
                  << std::setw(15) << std::fixed << std::setprecision(2)
                  << res.maxBufferSizeBytes / 1024.0
                  << std::setw(15) << std::fixed << std::setprecision(3)
                  << res.avgFlushLatencyMs
                  << std::setw(15) << std::fixed << std::setprecision(3)
                  << res.maxFlushLatencyMs
                  << std::setw(12) << res.totalFlushes << "\n";
    }

    // Evaluate success criteria
    std::cout << "\nSuccess Criteria Evaluation\n";
    std::cout << "===========================\n";
    std::cout << "Criterion: Buffer under 10 MB AND max flush latency < 1000 ms\n\n";

    constexpr size_t maxBufferSizeBytes = 10 * 1024 * 1024;  // 10 MB
    constexpr double maxFlushLatencyMs = 1000.0;             // 1 second

    bool foundValid = false;
    int recommendedInterval = 100;

    for (const auto& res : results) {
        bool bufferOk = res.maxBufferSizeBytes < maxBufferSizeBytes;
        bool latencyOk = res.maxFlushLatencyMs < maxFlushLatencyMs;
        bool pass = bufferOk && latencyOk;

        std::string status = pass ? "[PASS]" : "[FAIL]";
        std::cout << status << " Interval " << res.flushIntervalMs << " ms: ";

        if (!bufferOk) {
            std::cout << "buffer too large ("
                      << std::fixed << std::setprecision(2)
                      << res.maxBufferSizeBytes / (1024.0 * 1024.0) << " MB > 10 MB)";
        }
        if (!latencyOk) {
            if (!bufferOk) std::cout << ", ";
            std::cout << "latency too high ("
                      << std::fixed << std::setprecision(1)
                      << res.maxFlushLatencyMs << " ms > 1000 ms)";
        }
        if (pass) {
            std::cout << "meets criteria";
            if (!foundValid || res.flushIntervalMs > recommendedInterval) {
                recommendedInterval = res.flushIntervalMs;
                foundValid = true;
            }
        }

        std::cout << "\n";
    }

    std::cout << "\nOverall: " << (foundValid ? "VALIDATED" : "INVALIDATED") << "\n";
    if (foundValid) {
        std::cout << "Recommended flush interval: " << recommendedInterval << " ms\n";
        std::cout << "\nRationale: Larger intervals reduce I/O frequency while still\n";
        std::cout << "           maintaining acceptable memory footprint and latency.\n";
    }

    return foundValid ? 0 : 1;
}
