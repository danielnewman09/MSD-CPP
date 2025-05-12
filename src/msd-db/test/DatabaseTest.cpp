#include <gtest/gtest.h>
#include "msd-db/src/Database.hpp"
#include <filesystem>
#include <spdlog/sinks/null_sink.h>

namespace msd_db_test {

class DatabaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a null logger that doesn't actually output anything
        // to avoid cluttering test output
        auto null_sink = std::make_shared<spdlog::sinks::null_sink_mt>();
        test_logger = std::make_shared<spdlog::logger>("test_logger", null_sink);
        
        // Use a temporary file for testing
        temp_db_path = "test_db_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".db";
    }

    void TearDown() override {
        // Clean up the test database file
        if (std::filesystem::exists(temp_db_path)) {
            std::filesystem::remove(temp_db_path);
        }
    }

    std::shared_ptr<spdlog::logger> test_logger;
    std::string temp_db_path;
};

TEST_F(DatabaseTest, OpenAndCreateDatabase) {
    // Test creating a new database
    auto db = msd_db::Database(temp_db_path, test_logger, msd_db::DBOpenCondition::OpenCreate);
    
    // Verify the database was created
    ASSERT_TRUE(std::filesystem::exists(temp_db_path));
    
    // Verify we can execute a simple query
    ASSERT_TRUE(db.executeQuery("SELECT 1"));
}

TEST_F(DatabaseTest, CreateTableAndInsertData) {
    // Create a new database
    auto db = msd_db::Database(temp_db_path, test_logger, msd_db::DBOpenCondition::OpenCreate);
    
    // Create a test table
    ASSERT_TRUE(db.executeQuery(R"(
        CREATE TABLE test_table (
            id INTEGER PRIMARY KEY,
            name TEXT,
            value REAL
        )
    )"));
    
    // Insert data
    ASSERT_TRUE(db.executeQuery(R"(
        INSERT INTO test_table (name, value) VALUES ('test1', 1.1)
    )"));
    
    // Verify data was inserted using a SELECT query
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db.getRawDb(), "SELECT COUNT(*) FROM test_table", -1, &stmt, nullptr);
    ASSERT_EQ(rc, SQLITE_OK);
    
    rc = sqlite3_step(stmt);
    ASSERT_EQ(rc, SQLITE_ROW);
    
    int count = sqlite3_column_int(stmt, 0);
    ASSERT_EQ(count, 1);
    
    sqlite3_finalize(stmt);
}

TEST_F(DatabaseTest, TransactionCommit) {
    // Create a new database
    auto db = msd_db::Database(temp_db_path, test_logger, msd_db::DBOpenCondition::OpenCreate);
    
    // Create a test table
    ASSERT_TRUE(db.executeQuery("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value TEXT)"));
    
    // Start a transaction
    ASSERT_TRUE(db.beginTransaction());
    
    // Insert multiple rows
    ASSERT_TRUE(db.executeQuery("INSERT INTO test_table (value) VALUES ('row1')"));
    ASSERT_TRUE(db.executeQuery("INSERT INTO test_table (value) VALUES ('row2')"));
    ASSERT_TRUE(db.executeQuery("INSERT INTO test_table (value) VALUES ('row3')"));
    
    // Commit the transaction
    ASSERT_TRUE(db.commitTransaction());
    
    // Verify all rows were inserted
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db.getRawDb(), "SELECT COUNT(*) FROM test_table", -1, &stmt, nullptr);
    ASSERT_EQ(rc, SQLITE_OK);
    
    rc = sqlite3_step(stmt);
    ASSERT_EQ(rc, SQLITE_ROW);
    
    int count = sqlite3_column_int(stmt, 0);
    ASSERT_EQ(count, 3);
    
    sqlite3_finalize(stmt);
}

TEST_F(DatabaseTest, TransactionRollback) {
    // Create a new database
    auto db = msd_db::Database(temp_db_path, test_logger, msd_db::DBOpenCondition::OpenCreate);
    
    // Create a test table
    ASSERT_TRUE(db.executeQuery("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value TEXT)"));
    
    // Insert one row outside transaction
    ASSERT_TRUE(db.executeQuery("INSERT INTO test_table (value) VALUES ('permanent')"));
    
    // Start a transaction
    ASSERT_TRUE(db.beginTransaction());
    
    // Insert rows in transaction
    ASSERT_TRUE(db.executeQuery("INSERT INTO test_table (value) VALUES ('will be rolled back 1')"));
    ASSERT_TRUE(db.executeQuery("INSERT INTO test_table (value) VALUES ('will be rolled back 2')"));
    
    // Rollback the transaction
    ASSERT_TRUE(db.rollbackTransaction());
    
    // Verify only the first row remains
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db.getRawDb(), "SELECT COUNT(*) FROM test_table", -1, &stmt, nullptr);
    ASSERT_EQ(rc, SQLITE_OK);
    
    rc = sqlite3_step(stmt);
    ASSERT_EQ(rc, SQLITE_ROW);
    
    int count = sqlite3_column_int(stmt, 0);
    ASSERT_EQ(count, 1);
    
    sqlite3_finalize(stmt);
}

TEST_F(DatabaseTest, BuildDatabaseFactory) {
    // Test the factory function
    auto db_opt = msd_db::buildDatabase(temp_db_path, msd_db::DBOpenCondition::OpenCreate);
    
    ASSERT_TRUE(db_opt.has_value());
    ASSERT_TRUE(std::filesystem::exists(temp_db_path));
    
    // Verify the database works
    ASSERT_TRUE(db_opt->executeQuery("CREATE TABLE test (id INTEGER PRIMARY KEY)"));
}

TEST_F(DatabaseTest, InvalidQuery) {
    // Create a new database
    auto db = msd_db::Database(temp_db_path, test_logger, msd_db::DBOpenCondition::OpenCreate);
    
    // Execute an invalid query
    ASSERT_FALSE(db.executeQuery("CREATE TABLES invalid_syntax"));
}

TEST_F(DatabaseTest, ForeignKeyConstraints) {
    // Create a new database
    auto db = msd_db::Database(temp_db_path, test_logger, msd_db::DBOpenCondition::OpenCreate);
    
    // Create tables with foreign key relationship
    ASSERT_TRUE(db.executeQuery(R"(
        CREATE TABLE parent (
            id INTEGER PRIMARY KEY,
            name TEXT
        )
    )"));
    
    ASSERT_TRUE(db.executeQuery(R"(
        CREATE TABLE child (
            id INTEGER PRIMARY KEY,
            parent_id INTEGER,
            value TEXT,
            FOREIGN KEY (parent_id) REFERENCES parent(id)
        )
    )"));
    
    // Make sure foreign keys are enabled
    ASSERT_TRUE(db.enableForeignKeys(true));
    
    // Insert parent row
    ASSERT_TRUE(db.executeQuery("INSERT INTO parent (id, name) VALUES (1, 'parent1')"));
    
    // Insert valid child row should succeed
    ASSERT_TRUE(db.executeQuery("INSERT INTO child (parent_id, value) VALUES (1, 'valid')"));
    
    // Insert invalid child row should fail
    ASSERT_FALSE(db.executeQuery("INSERT INTO child (parent_id, value) VALUES (999, 'invalid')"));
}

} // namespace msd_db_test