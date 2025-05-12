#include "msd-db/src/Database.hpp"
#include <iostream>

namespace msd_db
{

Database::Database(std::string dbUrl, DBOpenCondition openCond)
{
  sqlite3* rawDb = nullptr;
  int rc = sqlite3_open_v2(dbUrl.c_str(), &rawDb, static_cast<int>(openCond), nullptr);
  
  if (rc != SQLITE_OK) {
    const char* errMsg = sqlite3_errmsg(rawDb);
    std::string errorStr = errMsg ? errMsg : "Unknown error";
    
    // Close the database if it was partially opened
    if (rawDb) {
      sqlite3_close(rawDb);
    }
    
    throw std::runtime_error("Failed to open database: " + errorStr);
  }
  
  // Move ownership to the smart pointer
  db_.reset(rawDb);
}

bool Database::executeQuery(const std::string& query)
{
  char* errMsg = nullptr;
  int rc = sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, &errMsg);
  
  if (rc != SQLITE_OK) {
    if (errMsg) {
      std::cerr << "SQL error: " << errMsg << std::endl;
      sqlite3_free(errMsg);
    } else {
      std::cerr << "Unknown SQL error" << std::endl;
    }
    return false;
  }
  
  return true;
}

std::optional<Database> buildDatabase(const std::string& dbUrl, DBOpenCondition openCond)
{
  try {
    // Directly construct the Database in the optional
    return Database(dbUrl, openCond);
  } catch (const std::exception& e) {
    std::cerr << "Failed to build database: " << e.what() << std::endl;
    return std::nullopt;
  }
}

} // namespace msd_db
