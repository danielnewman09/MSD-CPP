// src/msd-db/include/msd-db/Database.hpp
#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <optional>
#include <string>
#include <memory>
#include <stdexcept>

#include <sqlite3.h>
#include <spdlog/spdlog.h>

namespace msd_db
{

/*!
 * @brief Enum class for SQLite open conditions
 */
enum class DBOpenCondition : int
{
  OpenReadOnly = SQLITE_OPEN_READONLY,
  OpenReadWrite = SQLITE_OPEN_READWRITE,
  OpenCreate = SQLITE_OPEN_CREATE | SQLITE_OPEN_READWRITE,
};

/**
 * @brief Wraps a SQLite database connection with integrated logging
 */
class Database
{
public:
  /**
   * @brief Constructs a database connection
   * 
   * @param dbUrl URL to the SQLite database
   * @param logger Shared pointer to a logger instance
   * @param openCond Condition to open the database with
   * @throws std::runtime_error if the database cannot be opened
   */
  Database(std::string dbUrl, 
           std::shared_ptr<spdlog::logger> logger,
           DBOpenCondition openCond = DBOpenCondition::OpenReadWrite);
  
  /**
   * @brief Destructor automatically closes the database connection
   */
  ~Database();
  
  // Delete copy constructor and copy assignment
  Database(const Database&) = delete;
  Database& operator=(const Database&) = delete;
  
  // Allow move constructor and move assignment
  Database(Database&& other) noexcept;
  Database& operator=(Database&& other) noexcept;
  
  /**
   * @brief Get the raw SQLite database pointer
   * 
   * @return Raw SQLite database pointer
   */
  sqlite3* getRawDb() const { return db_.get(); }
  
  /**
   * @brief Execute a raw SQL query
   * 
   * @param query SQL query to execute
   * @return true if successful, false otherwise
   */
  bool executeQuery(const std::string& query);

  /**
   * @brief Enable or disable foreign key constraints
   * 
   * @param enable Whether to enable foreign keys
   * @return true if successful, false otherwise
   */
  bool enableForeignKeys(bool enable = true);

  /**
   * @brief Begin a transaction
   * 
   * @return true if successful, false otherwise
   */
  bool beginTransaction();

  /**
   * @brief Commit the current transaction
   * 
   * @return true if successful, false otherwise
   */
  bool commitTransaction();

  /**
   * @brief Rollback the current transaction
   * 
   * @return true if successful, false otherwise
   */
  bool rollbackTransaction();

  /**
   * @brief Get the logger instance
   * 
   * @return The logger instance
   */
  std::shared_ptr<spdlog::logger> getLogger() const { return logger_; }

private:
  // Custom deleter for sqlite3 pointer
  struct Sqlite3Deleter {
    void operator()(sqlite3* db) const {
      if (db) {
        sqlite3_close(db);
      }
    }
  };

  // SQLite database smart pointer with custom deleter
  std::unique_ptr<sqlite3, Sqlite3Deleter> db_;
  
  // Logger instance
  std::shared_ptr<spdlog::logger> logger_;
  
  // Database URL (useful for logging)
  std::string dbUrl_;
};

/**
 * @brief Factory function to build a database connection
 * 
 * @param dbUrl URL to the SQLite database
 * @param openCond Condition to open the database with
 * @param loggerName Name to use for the logger (defaults to "db-[filename]")
 * @return A Database object wrapped in std::optional if successful, or std::nullopt otherwise
 */
std::optional<Database> buildDatabase(
    const std::string& dbUrl,
    DBOpenCondition openCond = DBOpenCondition::OpenReadWrite,
    std::optional<std::string> loggerName = std::nullopt);

} // namespace msd_db

#endif // DATABASE_HPP