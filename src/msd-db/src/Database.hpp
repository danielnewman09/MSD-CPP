// Database.hpp
#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <sqlite3.h>

namespace msd_db
{

enum class DBOpenCondition : int
{
  OpenReadOnly = SQLITE_OPEN_READONLY,
  OpenReadWrite = SQLITE_OPEN_READWRITE,
  OpenCreate = SQLITE_OPEN_CREATE,
};

/**
 * @brief Wraps a SQLite database connection
 */
class Database
{
public:
  /**
   * @brief Constructs a database connection
   *
   * @param dbUrl URL to the SQLite database
   * @param openCond Condition to open the database with
   * @throws std::runtime_error if the database cannot be opened
   */
  Database(std::string dbUrl,
           DBOpenCondition openCond = DBOpenCondition::OpenReadWrite);

  // Delete copy constructor and copy assignment
  Database(const Database &) = delete;
  Database &operator=(const Database &) = delete;

  // Allow move constructor and move assignment
  Database(Database &&other) noexcept = default;
  Database &operator=(Database &&other) noexcept = default;

  /**
   * @brief Destructor automatically closes the database connection
   */
  ~Database() = default;

  /**
   * @brief Get the raw SQLite database pointer
   *
   * @return Raw SQLite database pointer
   */
  sqlite3 *getRawDb() const
  {
    return db_.get();
  }

  /**
   * @brief Execute a raw SQL query
   *
   * @param query SQL query to execute
   * @return true if successful, false otherwise
   */
  bool executeQuery(const std::string &query);

private:
  // Custom deleter for sqlite3 pointer
  struct Sqlite3Deleter
  {
    void operator()(sqlite3 *db) const
    {
      if (db)
        {
          sqlite3_close(db);
        }
    }
  };

  // SQLite database smart pointer with custom deleter
  std::unique_ptr<sqlite3, Sqlite3Deleter> db_;
};

/**
 * @brief Factory function to build a database connection
 *
 * @param dbUrl URL to the SQLite database
 * @param openCond Condition to open the database with
 * @return A Database object wrapped in std::optional if successful, or
 * std::nullopt otherwise
 */
std::optional<Database>
buildDatabase(const std::string &dbUrl,
              DBOpenCondition openCond = DBOpenCondition::OpenReadWrite);

} // namespace msd_db

#endif // DATABASE_HPP