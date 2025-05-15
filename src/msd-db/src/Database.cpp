// src/msd-db/src/Database.cpp
#include "Database.hpp"
#include <iostream>
#include <filesystem>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace msd_db
{

Database::Database(std::string dbUrl, 
                  std::shared_ptr<spdlog::logger> logger,
                  DBOpenCondition openCond)
    : logger_(logger), dbUrl_(dbUrl)
{
  logger_->debug("Opening database: {}", dbUrl);
  
  sqlite3* rawDb = nullptr;
  int rc = sqlite3_open_v2(dbUrl.c_str(), &rawDb, static_cast<int>(openCond), nullptr);
  
  if (rc != SQLITE_OK) {
    const char* errMsg = sqlite3_errmsg(rawDb);
    std::string errorStr = errMsg ? errMsg : "Unknown error";
    
    logger_->error("Failed to open database: {}", errorStr);
    
    // Close the database if it was partially opened
    if (rawDb) {
      sqlite3_close(rawDb);
    }
    
    throw std::runtime_error("Failed to open database: " + errorStr);
  }
  
  // Move ownership to the smart pointer
  db_.reset(rawDb);
  
  logger_->info("Database opened successfully: {}", dbUrl);
  
  // Enable foreign keys by default
  enableForeignKeys(true);
  
  // Enable WAL mode for better concurrency
  executeQuery("PRAGMA journal_mode = WAL;");
}

Database::~Database()
{
  if (db_) {
    logger_->debug("Closing database: {}", dbUrl_);
  }
}

Database::Database(Database&& other) noexcept
    : db_(std::move(other.db_)), 
      logger_(std::move(other.logger_)),
      dbUrl_(std::move(other.dbUrl_))
{
  logger_->trace("Database moved via move constructor");
}

Database& Database::operator=(Database&& other) noexcept
{
  if (this != &other) {
    // First log with our current logger before we lose it
    if (logger_) {
      logger_->trace("Database moved via move assignment from: {}", other.dbUrl_);
    }
    
    // Move all members
    db_ = std::move(other.db_);
    logger_ = std::move(other.logger_);
    dbUrl_ = std::move(other.dbUrl_);
  }
  return *this;
}

bool Database::executeQuery(const std::string& query)
{
  logger_->debug("Executing query: {}", query);
  
  char* errMsg = nullptr;
  int rc = sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, &errMsg);
  
  if (rc != SQLITE_OK) {
    if (errMsg) {
      logger_->error("SQL error: {}", errMsg);
      sqlite3_free(errMsg);
    } else {
      logger_->error("Unknown SQL error");
    }
    return false;
  }
  
  logger_->trace("Query executed successfully");
  return true;
}

bool Database::enableForeignKeys(bool enable)
{
  logger_->debug("Setting foreign keys to: {}", enable ? "enabled" : "disabled");
  
  std::string query = "PRAGMA foreign_keys = ";
  query += enable ? "ON" : "OFF";
  query += ";";
  
  return executeQuery(query);
}

bool Database::beginTransaction()
{
  logger_->debug("Beginning transaction");
  return executeQuery("BEGIN TRANSACTION;");
}

bool Database::commitTransaction()
{
  logger_->debug("Committing transaction");
  return executeQuery("COMMIT;");
}

bool Database::rollbackTransaction()
{
  logger_->debug("Rolling back transaction");
  return executeQuery("ROLLBACK;");
}

std::optional<Database> buildDatabase(
    const std::string& dbUrl,
    DBOpenCondition openCond,
    std::optional<std::string> loggerName)
{
  try {
    // Create a default logger name if none provided
    std::string actualLoggerName;
    if (loggerName) {
      actualLoggerName = *loggerName;
    } else {
      // Extract filename from path for default logger name
      std::filesystem::path filePath(dbUrl);
      std::string fileName = filePath.filename().string();
      actualLoggerName = "db-" + fileName;
    }
    
    // Create and configure logger
    auto logger = spdlog::stdout_color_mt(actualLoggerName);
    logger->set_level(spdlog::level::debug);  // Set default level
    
    // Directly construct the Database in the optional
    return Database(dbUrl, logger, openCond);
  } catch (const spdlog::spdlog_ex& e) {
    std::cerr << "Logger initialization failed: " << e.what() << std::endl;
    return std::nullopt;
  } catch (const std::exception& e) {
    std::cerr << "Failed to build database: " << e.what() << std::endl;
    return std::nullopt;
  }
}

} // namespace msd_db