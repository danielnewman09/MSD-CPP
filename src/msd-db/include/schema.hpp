#ifndef MSD_DB_SCHEMA_HPP
#define MSD_DB_SCHEMA_HPP
#include <chrono>
#include <memory>
#include <optional>
#include <sqlite3.h>
#include <string>
#include <type_traits>
#include <vector>

namespace msd_db
{

// Base types for ORM
enum class ColumnType
{
  INTEGER,
  REAL,
  TEXT,
  BLOB
};

// Foreign key constraint info
struct ForeignKeyConstraint
{
  std::string reference_table;
  std::string reference_column;

  ForeignKeyConstraint(const std::string &table, const std::string &column)
      : reference_table(table),
        reference_column(column)
  {
  }
};

// Enhanced member info with foreign key support
template <typename Class, typename Member> struct MemberInfo
{
  Member Class::*ptr;
  ColumnType type;
  std::string column_name;
  bool primary_key;
  bool auto_increment;
  bool nullable;
  std::optional<ForeignKeyConstraint> foreign_key;

  MemberInfo(Member Class::*p, const std::string &name, ColumnType t,
             bool pk = false, bool ai = false, bool null = false,
             std::optional<ForeignKeyConstraint> fk = std::nullopt)
      : ptr(p),
        column_name(name),
        type(t),
        primary_key(pk),
        auto_increment(ai),
        nullable(null),
        foreign_key(fk)
  {
  }
};

// Compile-time metadata builder
template <typename TO> class TableSchema
{
public:
  using transfer_object_type = TO;
  using member_info_list = std::vector<std::unique_ptr<void>>;

  template <typename Member>
  TableSchema &add_member(Member TO::*ptr, const std::string &name,
                          ColumnType type, bool primary_key = false,
                          bool auto_increment = false, bool nullable = false)
  {
    members_.push_back(std::make_unique<MemberInfo<TO, Member>>(
        ptr, name, type, primary_key, auto_increment, nullable));
    return *this;
  }

  template <typename Member>
  TableSchema &add_foreign_key(Member TO::*ptr, const std::string &name,
                               ColumnType type, const std::string &ref_table,
                               const std::string &ref_column,
                               bool nullable = false)
  {
    ForeignKeyConstraint fk{ref_table, ref_column};
    members_.push_back(std::make_unique<MemberInfo<TO, Member>>(
        ptr, name, type, false, false, nullable, fk));
    return *this;
  }

  const std::string &table_name() const
  {
    return table_name_;
  }
  
  TableSchema &set_table_name(const std::string &name)
  {
    table_name_ = name;
    return *this;
  }

private:
  std::string table_name_;
  member_info_list members_;

  friend class DataAccessObject<TO>;
};

} // namespace msd_db

#endif // MSD_DB_SCHEMA_HPP