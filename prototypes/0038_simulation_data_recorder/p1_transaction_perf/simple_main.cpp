/**
 * Prototype P1: Transaction Performance Assessment (Simplified)
 *
 * Question: Does using a SQL transaction provide meaningful speedup over auto-commit
 *           for batch sizes 10-1000?
 *
 * Success Criteria: Transaction mode at least 2x faster for batch size >= 100
 *
 * Note: This simplified version uses sqlite3 C API directly to avoid Conan/CMake complexity
 */

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sqlite3.h>
#include <vector>

using namespace std::chrono;

struct BenchmarkResult {
    int batchSize;
    double autocommitTimeMs;
    double transactionTimeMs;
    double speedup;
};

void checkSqlite(int rc, const char* msg, sqlite3* db) {
    if (rc != SQLITE_OK) {
        std::cerr << "SQLite error: " << msg << ": "
                  << sqlite3_errmsg(db) << "\n";
        sqlite3_close(db);
        std::exit(1);
    }
}

void createTable(sqlite3* db) {
    const char* sql = R"(
        CREATE TABLE IF NOT EXISTS test_records (
            id INTEGER PRIMARY KEY,
            simulation_time REAL,
            position_x REAL,
            position_y REAL,
            position_z REAL,
            velocity_x REAL,
            velocity_y REAL,
            velocity_z REAL,
            orientation_w REAL,
            orientation_x REAL,
            orientation_y REAL,
            orientation_z REAL,
            angular_velocity_x REAL,
            angular_velocity_y REAL,
            angular_velocity_z REAL
        );
    )";

    char* errMsg = nullptr;
    int rc = sqlite3_exec(db, sql, nullptr, nullptr, &errMsg);
    if (rc != SQLITE_OK) {
        std::cerr << "SQL error: " << errMsg << "\n";
        sqlite3_free(errMsg);
        sqlite3_close(db);
        std::exit(1);
    }
}

/**
 * @brief Benchmark auto-commit mode
 */
double benchmarkAutocommit(sqlite3* db, int batchSize) {
    // Clear table
    sqlite3_exec(db, "DELETE FROM test_records;", nullptr, nullptr, nullptr);

    const char* insertSql = R"(
        INSERT INTO test_records (
            simulation_time, position_x, position_y, position_z,
            velocity_x, velocity_y, velocity_z,
            orientation_w, orientation_x, orientation_y, orientation_z,
            angular_velocity_x, angular_velocity_y, angular_velocity_z
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
    )";

    sqlite3_stmt* stmt = nullptr;
    checkSqlite(sqlite3_prepare_v2(db, insertSql, -1, &stmt, nullptr),
                "prepare statement", db);

    auto start = high_resolution_clock::now();

    for (int i = 0; i < batchSize; ++i) {
        sqlite3_bind_double(stmt, 1, i * 0.01);
        sqlite3_bind_double(stmt, 2, i * 1.0);
        sqlite3_bind_double(stmt, 3, i * 2.0);
        sqlite3_bind_double(stmt, 4, i * 3.0);
        sqlite3_bind_double(stmt, 5, i * 0.1);
        sqlite3_bind_double(stmt, 6, i * 0.2);
        sqlite3_bind_double(stmt, 7, i * 0.3);
        sqlite3_bind_double(stmt, 8, 1.0);
        sqlite3_bind_double(stmt, 9, 0.0);
        sqlite3_bind_double(stmt, 10, 0.0);
        sqlite3_bind_double(stmt, 11, 0.0);
        sqlite3_bind_double(stmt, 12, i * 0.01);
        sqlite3_bind_double(stmt, 13, i * 0.02);
        sqlite3_bind_double(stmt, 14, i * 0.03);

        int rc = sqlite3_step(stmt);
        if (rc != SQLITE_DONE) {
            std::cerr << "Insert failed: " << sqlite3_errmsg(db) << "\n";
        }

        sqlite3_reset(stmt);  // Auto-commit happens here
    }

    auto end = high_resolution_clock::now();
    sqlite3_finalize(stmt);

    return duration_cast<microseconds>(end - start).count() / 1000.0;
}

/**
 * @brief Benchmark transaction mode
 */
double benchmarkTransaction(sqlite3* db, int batchSize) {
    // Clear table
    sqlite3_exec(db, "DELETE FROM test_records;", nullptr, nullptr, nullptr);

    const char* insertSql = R"(
        INSERT INTO test_records (
            simulation_time, position_x, position_y, position_z,
            velocity_x, velocity_y, velocity_z,
            orientation_w, orientation_x, orientation_y, orientation_z,
            angular_velocity_x, angular_velocity_y, angular_velocity_z
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
    )";

    sqlite3_stmt* stmt = nullptr;
    checkSqlite(sqlite3_prepare_v2(db, insertSql, -1, &stmt, nullptr),
                "prepare statement", db);

    auto start = high_resolution_clock::now();

    // Begin transaction
    sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);

    for (int i = 0; i < batchSize; ++i) {
        sqlite3_bind_double(stmt, 1, i * 0.01);
        sqlite3_bind_double(stmt, 2, i * 1.0);
        sqlite3_bind_double(stmt, 3, i * 2.0);
        sqlite3_bind_double(stmt, 4, i * 3.0);
        sqlite3_bind_double(stmt, 5, i * 0.1);
        sqlite3_bind_double(stmt, 6, i * 0.2);
        sqlite3_bind_double(stmt, 7, i * 0.3);
        sqlite3_bind_double(stmt, 8, 1.0);
        sqlite3_bind_double(stmt, 9, 0.0);
        sqlite3_bind_double(stmt, 10, 0.0);
        sqlite3_bind_double(stmt, 11, 0.0);
        sqlite3_bind_double(stmt, 12, i * 0.01);
        sqlite3_bind_double(stmt, 13, i * 0.02);
        sqlite3_bind_double(stmt, 14, i * 0.03);

        int rc = sqlite3_step(stmt);
        if (rc != SQLITE_DONE) {
            std::cerr << "Insert failed: " << sqlite3_errmsg(db) << "\n";
        }

        sqlite3_reset(stmt);
    }

    // Commit transaction
    sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);

    auto end = high_resolution_clock::now();
    sqlite3_finalize(stmt);

    return duration_cast<microseconds>(end - start).count() / 1000.0;
}

int main() {
    std::cout << "Prototype P1: Transaction Performance Assessment\n";
    std::cout << "================================================\n\n";

    std::vector<int> batchSizes{10, 50, 100, 250, 500, 1000};
    std::vector<BenchmarkResult> results;

    for (int batchSize : batchSizes) {
        std::cout << "Testing batch size: " << batchSize << "...\n";

        std::string dbPath = "test_p1_" + std::to_string(batchSize) + ".db";
        std::filesystem::remove(dbPath);

        sqlite3* db = nullptr;
        int rc = sqlite3_open(dbPath.c_str(), &db);
        checkSqlite(rc, "open database", db);

        createTable(db);

        // Run auto-commit benchmark
        double autocommitTime = benchmarkAutocommit(db, batchSize);
        std::cout << "  Auto-commit: " << std::fixed << std::setprecision(3)
                  << autocommitTime << " ms\n";

        // Run transaction benchmark
        double transactionTime = benchmarkTransaction(db, batchSize);
        std::cout << "  Transaction: " << std::fixed << std::setprecision(3)
                  << transactionTime << " ms\n";

        double speedup = (transactionTime > 0) ? (autocommitTime / transactionTime) : 0.0;
        std::cout << "  Speedup: " << std::fixed << std::setprecision(2)
                  << speedup << "x\n\n";

        BenchmarkResult res{batchSize, autocommitTime, transactionTime, speedup};
        results.push_back(res);

        sqlite3_close(db);
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
