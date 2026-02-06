/**
 * Prototype P1: Transaction Performance Assessment
 *
 * Question: Does withTransaction() provide meaningful speedup over auto-commit
 *           for batch sizes 10-1000?
 *
 * Success Criteria: Transaction mode at least 2x faster for batch size >= 100
 *
 * Approach: Measure insert throughput with and without transactions for various
 *           batch sizes, using cpp_sqlite DAOs and buffered writes.
 */

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <vector>

#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <spdlog/spdlog.h>

#include "test_record.hpp"

using namespace std::chrono;

struct BenchmarkResult {
    int batchSize;
    double autocommitTimeMs;
    double transactionTimeMs;
    double speedup;
};

/**
 * @brief Benchmark auto-commit mode (no explicit transaction)
 *
 * Each DAO.insert() call is auto-committed individually.
 */
double benchmarkAutocommit(cpp_sqlite::Database& db, int batchSize) {
    auto& dao = db.getDAO<TestRecord>();
    dao.clearBuffer();  // Start fresh

    // Prepare test records
    std::vector<TestRecord> records;
    records.reserve(batchSize);
    for (int i = 0; i < batchSize; ++i) {
        TestRecord rec{};
        rec.simulation_time = i * 0.01;
        rec.position_x = i * 1.0;
        rec.position_y = i * 2.0;
        rec.position_z = i * 3.0;
        records.push_back(rec);
    }

    // Measure time to buffer and insert
    auto start = high_resolution_clock::now();

    for (const auto& rec : records) {
        dao.addToBuffer(rec);
    }
    dao.insert();  // Auto-commit each buffered record

    auto end = high_resolution_clock::now();
    return duration_cast<microseconds>(end - start).count() / 1000.0;  // Convert to ms
}

/**
 * @brief Benchmark transaction mode
 *
 * All inserts wrapped in a single transaction via withTransaction().
 */
double benchmarkTransaction(cpp_sqlite::Database& db, int batchSize) {
    auto& dao = db.getDAO<TestRecord>();
    dao.clearBuffer();  // Start fresh

    // Prepare test records
    std::vector<TestRecord> records;
    records.reserve(batchSize);
    for (int i = 0; i < batchSize; ++i) {
        TestRecord rec{};
        rec.simulation_time = i * 0.01;
        rec.position_x = i * 1.0;
        rec.position_y = i * 2.0;
        rec.position_z = i * 3.0;
        records.push_back(rec);
    }

    // Measure time to buffer and insert within transaction
    auto start = high_resolution_clock::now();

    db.withTransaction([&]() {
        for (const auto& rec : records) {
            dao.addToBuffer(rec);
        }
        dao.insert();  // Flush within transaction
    });

    auto end = high_resolution_clock::now();
    return duration_cast<microseconds>(end - start).count() / 1000.0;  // Convert to ms
}

int main() {
    std::cout << "Prototype P1: Transaction Performance Assessment\n";
    std::cout << "================================================\n\n";

    // Test batch sizes: 10, 50, 100, 250, 500, 1000
    std::vector<int> batchSizes{10, 50, 100, 250, 500, 1000};
    std::vector<BenchmarkResult> results;

    for (int batchSize : batchSizes) {
        std::cout << "Testing batch size: " << batchSize << "...\n";

        // Create temporary database
        std::string dbPath = "test_p1_" + std::to_string(batchSize) + ".db";
        std::filesystem::remove(dbPath);  // Clean slate

        // Run autocommit benchmark
        {
            cpp_sqlite::Database db{dbPath, true};
            double autocommitTime = benchmarkAutocommit(db, batchSize);
            std::cout << "  Auto-commit: " << std::fixed << std::setprecision(3)
                      << autocommitTime << " ms\n";

            // Store result
            BenchmarkResult res{};
            res.batchSize = batchSize;
            res.autocommitTimeMs = autocommitTime;

            // Run transaction benchmark (reuse same DB)
            double transactionTime = benchmarkTransaction(db, batchSize);
            std::cout << "  Transaction: " << std::fixed << std::setprecision(3)
                      << transactionTime << " ms\n";

            res.transactionTimeMs = transactionTime;
            res.speedup = (transactionTime > 0) ? (autocommitTime / transactionTime) : 0.0;

            std::cout << "  Speedup: " << std::fixed << std::setprecision(2)
                      << res.speedup << "x\n\n";

            results.push_back(res);
        }

        // Clean up
        std::filesystem::remove(dbPath);
    }

    // Print summary table
    std::cout << "\nSummary Table\n";
    std::cout << "=============\n";
    std::cout << std::setw(12) << "Batch Size"
              << std::setw(18) << "Auto-commit (ms)"
              << std::setw(18) << "Transaction (ms)"
              << std::setw(12) << "Speedup\n";
    std::cout << std::string(60, '-') << "\n";

    for (const auto& res : results) {
        std::cout << std::setw(12) << res.batchSize
                  << std::setw(18) << std::fixed << std::setprecision(3) << res.autocommitTimeMs
                  << std::setw(18) << std::fixed << std::setprecision(3) << res.transactionTimeMs
                  << std::setw(12) << std::fixed << std::setprecision(2) << res.speedup << "x\n";
    }

    // Evaluate success criteria
    std::cout << "\nSuccess Criteria Evaluation\n";
    std::cout << "===========================\n";
    std::cout << "Criterion: Transaction mode at least 2x faster for batch size >= 100\n\n";

    bool success = true;
    for (const auto& res : results) {
        if (res.batchSize >= 100) {
            std::string status = (res.speedup >= 2.0) ? "[PASS]" : "[FAIL]";
            std::cout << status << " Batch size " << res.batchSize
                      << ": " << std::fixed << std::setprecision(2) << res.speedup << "x speedup";
            if (res.speedup < 2.0) {
                std::cout << " (expected >= 2.0x)";
                success = false;
            }
            std::cout << "\n";
        }
    }

    std::cout << "\nOverall: " << (success ? "VALIDATED" : "INVALIDATED") << "\n";

    return success ? 0 : 1;
}
