#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <optional>
#include <string>
#include <memory>

#include <sqlite3.h>

namespace msd_db
{

enum class DBOpenCondition : int
{
  OpenReadOnly = SQLITE_OPEN_READONLY,
  OpenReadWrite = SQLITE_OPEN_READWRITE,
  OpenCreate = SQLITE_OPEN_CREATE,
};

class Database
{
public:
  Database(std::string dbUrl);

private:
  // Base SQLite object
  std::unique_ptr<sqlite3> db_;
};

/*!
 * \brief Build a database connection.
 *
 * \param dbUrl The URL of the database to connect to.
 * \param openCond The condition under which to open the database.
 * \return A Database object if successful, or nullopt otherwise.
 */
std::optional<Database> buildDatabase(const std::string &dbUrl,
                                      DBOpenCondition openCond);

} // namespace msd_db

#endif // DATABASE_HPP