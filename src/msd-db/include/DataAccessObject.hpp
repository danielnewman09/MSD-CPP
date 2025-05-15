
// Base Data Access Object with foreign key support
template<typename TO>
class DataAccessObject {
public:
    explicit DataAccessObject(sqlite3* db, const TableSchema<TO>& schema)
        : db_(db), schema_(schema) {
        create_table();
        prepare_statements();
    }
    
    ~DataAccessObject() {
        for (auto& stmt : prepared_statements_) {
            if (stmt) sqlite3_finalize(stmt);
        }
    }
    
    // Basic CRUD operations
    bool insert(const TO& obj) {
        sqlite3_reset(insert_stmt_);
        bind_values(insert_stmt_, obj, /*skip_auto_increment=*/true);
        return sqlite3_step(insert_stmt_) == SQLITE_DONE;
    }
    
    // Insert and return the auto-generated ID
    std::optional<int64_t> insert_returning_id(const TO& obj) {
        if (insert(obj)) {
            return sqlite3_last_insert_rowid(db_);
        }
        return std::nullopt;
    }
    
    bool update(const TO& obj) {
        sqlite3_reset(update_stmt_);
        bind_values(update_stmt_, obj, /*skip_auto_increment=*/false);
        return sqlite3_step(update_stmt_) == SQLITE_DONE;
    }
    
    bool remove(int64_t id) {
        sqlite3_reset(delete_stmt_);
        sqlite3_bind_int64(delete_stmt_, 1, id);
        return sqlite3_step(delete_stmt_) == SQLITE_DONE;
    }
    
    std::unique_ptr<TO> find_by_id(int64_t id) {
        sqlite3_reset(select_stmt_);
        sqlite3_bind_int64(select_stmt_, 1, id);
        
        if (sqlite3_step(select_stmt_) == SQLITE_ROW) {
            return extract_object(select_stmt_);
        }
        return nullptr;
    }
    
    std::vector<TO> find_all() {
        std::vector<TO> results;
        sqlite3_reset(select_all_stmt_);
        
        while (sqlite3_step(select_all_stmt_) == SQLITE_ROW) {
            if (auto obj = extract_object(select_all_stmt_)) {
                results.push_back(*obj);
            }
        }
        return results;
    }
    
    // Foreign key queries
    template<typename FK_TO>
    std::vector<FK_TO> find_by_foreign_key(const std::string& fk_column, int64_t fk_value,
                                           DataAccessObject<FK_TO>& fk_dao) {
        std::string query = "SELECT * FROM " + fk_dao.schema_.table_name_ + 
                           " WHERE " + fk_column + " = ?";
        return fk_dao.execute_query_with_param(query, fk_value);
    }
    
    // Execute query with parameter binding
    std::vector<TO> execute_query_with_param(const std::string& query, int64_t param) {
        std::vector<TO> results;
        sqlite3_stmt* stmt;
        
        if (sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
            sqlite3_bind_int64(stmt, 1, param);
            while (sqlite3_step(stmt) == SQLITE_ROW) {
                if (auto obj = extract_object(stmt)) {
                    results.push_back(*obj);
                }
            }
            sqlite3_finalize(stmt);
        }
        return results;
    }
    
    // Join operations
    std::vector<TO> find_with_join(const std::string& join_clause, const std::string& where_clause) {
        std::string query = "SELECT * FROM " + schema_.table_name_ + 
                           " " + join_clause + 
                           " WHERE " + where_clause;
        
        std::vector<TO> results;
        sqlite3_stmt* stmt;
        
        if (sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
            while (sqlite3_step(stmt) == SQLITE_ROW) {
                if (auto obj = extract_object(stmt)) {
                    results.push_back(*obj);
                }
            }
            sqlite3_finalize(stmt);
        }
        return results;
    }
    
private:
    sqlite3* db_;
    const TableSchema<TO>& schema_;
    
    // Statement management
    sqlite3_stmt* insert_stmt_ = nullptr;
    sqlite3_stmt* update_stmt_ = nullptr;
    sqlite3_stmt* delete_stmt_ = nullptr;
    sqlite3_stmt* select_stmt_ = nullptr;
    sqlite3_stmt* select_all_stmt_ = nullptr;
    std::vector<sqlite3_stmt*> prepared_statements_;
    
    void create_table() {
        std::string create_sql = "CREATE TABLE IF NOT EXISTS " + schema_.table_name_ + " (";
        bool first = true;
        
        for (const auto& member_ptr : schema_.members_) {
            if (!first) create_sql += ", ";
            
            auto info = get_member_info_base(member_ptr.get());
            std::string column_def = info->column_name + " " + get_sql_type(info->type);
            
            if (info->primary_key) column_def += " PRIMARY KEY";
            if (info->auto_increment) column_def += " AUTOINCREMENT";
            if (!info->nullable) column_def += " NOT NULL";
            
            create_sql += column_def;
            first = false;
        }
        
        // Add foreign key constraints
        for (const auto& member_ptr : schema_.members_) {
            auto info = get_member_info_base(member_ptr.get());
            if (info->foreign_key.has_value()) {
                const auto& fk = info->foreign_key.value();
                create_sql += ", FOREIGN KEY(" + info->column_name + ") REFERENCES " +
                             fk.reference_table + "(" + fk.reference_column + ")";
            }
        }
        
        create_sql += ");";
        
        char* error_msg;
        sqlite3_exec(db_, create_sql.c_str(), nullptr, nullptr, &error_msg);
        if (error_msg) {
            sqlite3_free(error_msg);
        }
        
        // Create indices for foreign keys
        for (const auto& member_ptr : schema_.members_) {
            auto info = get_member_info_base(member_ptr.get());
            if (info->foreign_key.has_value()) {
                std::string index_sql = "CREATE INDEX IF NOT EXISTS idx_" + 
                                       schema_.table_name_ + "_" + info->column_name +
                                       " ON " + schema_.table_name_ + "(" + info->column_name + ");";
                sqlite3_exec(db_, index_sql.c_str(), nullptr, nullptr, &error_msg);
                if (error_msg) {
                    sqlite3_free(error_msg);
                }
            }
        }
    }
    
    // ... [Previous helper functions with foreign key support] ...
    
    std::string get_sql_type(ColumnType type) {
        switch (type) {
            case ColumnType::INTEGER: return "INTEGER";
            case ColumnType::REAL: return "REAL";
            case ColumnType::TEXT: return "TEXT";
            case ColumnType::BLOB: return "BLOB";
            default: return "TEXT";
        }
    }
    
    // Type-erased helpers
    struct MemberInfoBase {
        std::string column_name;
        ColumnType type;
        bool primary_key;
        bool auto_increment;
        bool nullable;
        std::optional<ForeignKeyConstraint> foreign_key;
    };
    
    MemberInfoBase* get_member_info_base(void* member_ptr) {
        return reinterpret_cast<MemberInfoBase*>(member_ptr);
    }
};
