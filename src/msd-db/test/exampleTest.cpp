
// Example: Simulation Session and Platform States

// Transfer Object for simulation sessions (parent table)
struct SimulationSessionTO {
    int64_t id;
    double start_time;
    double end_time;
    std::string session_name;
    std::string metadata;
};

// Transfer Object for platform states (child table with foreign key)
struct PlatformStateTO {
    int64_t id;
    int64_t session_id;  // Foreign key to SimulationSessionTO
    double timestamp;
    std::string platform_id;
    double position_x;
    double position_y;
    double position_z;
    double velocity_x;
    double velocity_y;
    double velocity_z;
    double orientation_qw;
    double orientation_qx;
    double orientation_qy;
    double orientation_qz;
};

// Schema definitions with foreign keys
inline TableSchema<SimulationSessionTO> create_session_schema() {
    TableSchema<SimulationSessionTO> schema;
    schema.set_table_name("simulation_sessions")
        .add_member(&SimulationSessionTO::id, "id", ColumnType::INTEGER, true, true)
        .add_member(&SimulationSessionTO::start_time, "start_time", ColumnType::REAL)
        .add_member(&SimulationSessionTO::end_time, "end_time", ColumnType::REAL, false, false, true)
        .add_member(&SimulationSessionTO::session_name, "session_name", ColumnType::TEXT)
        .add_member(&SimulationSessionTO::metadata, "metadata", ColumnType::TEXT, false, false, true);
    return schema;
}

inline TableSchema<PlatformStateTO> create_platform_state_schema() {
    TableSchema<PlatformStateTO> schema;
    schema.set_table_name("platform_states")
        .add_member(&PlatformStateTO::id, "id", ColumnType::INTEGER, true, true)
        .add_foreign_key(&PlatformStateTO::session_id, "session_id", ColumnType::INTEGER,
                        "simulation_sessions", "id")
        .add_member(&PlatformStateTO::timestamp, "timestamp", ColumnType::REAL)
        .add_member(&PlatformStateTO::platform_id, "platform_id", ColumnType::TEXT)
        .add_member(&PlatformStateTO::position_x, "position_x", ColumnType::REAL)
        .add_member(&PlatformStateTO::position_y, "position_y", ColumnType::REAL)
        .add_member(&PlatformStateTO::position_z, "position_z", ColumnType::REAL)
        .add_member(&PlatformStateTO::velocity_x, "velocity_x", ColumnType::REAL)
        .add_member(&PlatformStateTO::velocity_y, "velocity_y", ColumnType::REAL)
        .add_member(&PlatformStateTO::velocity_z, "velocity_z", ColumnType::REAL)
        .add_member(&PlatformStateTO::orientation_qw, "orientation_qw", ColumnType::REAL)
        .add_member(&PlatformStateTO::orientation_qx, "orientation_qx", ColumnType::REAL)
        .add_member(&PlatformStateTO::orientation_qy, "orientation_qy", ColumnType::REAL)
        .add_member(&PlatformStateTO::orientation_qz, "orientation_qz", ColumnType::REAL);
    return schema;
}

// Usage example with foreign keys
class SimulationRecorder {
private:
    sqlite3* db_;
    std::unique_ptr<DataAccessObject<SimulationSessionTO>> session_dao_;
    std::unique_ptr<DataAccessObject<PlatformStateTO>> platform_dao_;
    int64_t current_session_id_ = -1;
    
public:
    SimulationRecorder(const std::string& db_path) {
        sqlite3_open(db_path.c_str(), &db_);
        
        // Enable foreign key support
        sqlite3_exec(db_, "PRAGMA foreign_keys = ON;", nullptr, nullptr, nullptr);
        
        auto session_schema = create_session_schema();
        auto platform_schema = create_platform_state_schema();
        
        session_dao_ = std::make_unique<DataAccessObject<SimulationSessionTO>>(db_, session_schema);
        platform_dao_ = std::make_unique<DataAccessObject<PlatformStateTO>>(db_, platform_schema);
    }
    
    ~SimulationRecorder() {
        if (db_) sqlite3_close(db_);
    }
    
    bool start_new_session(const std::string& session_name) {
        SimulationSessionTO session;
        session.start_time = get_current_time();
        session.session_name = session_name;
        session.metadata = "{}"; // Empty JSON metadata
        
        if (auto id = session_dao_->insert_returning_id(session)) {
            current_session_id_ = *id;
            return true;
        }
        return false;
    }
    
    bool end_current_session() {
        if (current_session_id_ == -1) return false;
        
        if (auto session = session_dao_->find_by_id(current_session_id_)) {
            session->end_time = get_current_time();
            return session_dao_->update(*session);
        }
        return false;
    }
    
    bool record_platform_state(const std::string& platform_id, const PlatformState& state) {
        if (current_session_id_ == -1) return false;
        
        PlatformStateTO platform_to;
        platform_to.session_id = current_session_id_;  // Foreign key reference
        platform_to.timestamp = state.timestamp;
        platform_to.platform_id = platform_id;
        platform_to.position_x = state.position.x;
        platform_to.position_y = state.position.y;
        platform_to.position_z = state.position.z;
        platform_to.velocity_x = state.velocity.x;
        platform_to.velocity_y = state.velocity.y;
        platform_to.velocity_z = state.velocity.z;
        platform_to.orientation_qw = state.orientation.w;
        platform_to.orientation_qx = state.orientation.x;
        platform_to.orientation_qy = state.orientation.y;
        platform_to.orientation_qz = state.orientation.z;
        
        return platform_dao_->insert(platform_to);
    }
    
    // Query methods using foreign keys
    std::vector<PlatformStateTO> get_platform_states_for_session(int64_t session_id, 
                                                                const std::string& platform_id = "") {
        std::string query = "SELECT * FROM platform_states WHERE session_id = ?";
        if (!platform_id.empty()) {
            query += " AND platform_id = '" + platform_id + "'";
        }
        return platform_dao_->execute_query_with_param(query, session_id);
    }
    
    // Get states for a specific time range with join
    std::vector<PlatformStateTO> get_states_in_time_range(double start_time, double end_time,
                                                          const std::string& session_name) {
        std::string join_clause = "JOIN simulation_sessions ss ON platform_states.session_id = ss.id";
        std::string where_clause = "ss.session_name = '" + session_name + 
                                  "' AND platform_states.timestamp BETWEEN " + 
                                  std::to_string(start_time) + " AND " + std::to_string(end_time);
        
        return platform_dao_->find_with_join(join_clause, where_clause);
    }
    
private:
    double get_current_time() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }
};

// Additional Transfer Objects for other relationships

// Transfer Object for simulation events
struct SimulationEventTO {
    int64_t id;
    int64_t session_id;  // Foreign key to SimulationSessionTO
    double timestamp;
    std::string event_type;
    std::string description;
    std::string platform_id;  // Optional: which platform generated the event
    std::vector<uint8_t> event_data;
};

// Example schema for events with foreign key
inline TableSchema<SimulationEventTO> create_event_schema() {
    TableSchema<SimulationEventTO> schema;
    schema.set_table_name("simulation_events")
        .add_member(&SimulationEventTO::id, "id", ColumnType::INTEGER, true, true)
        .add_foreign_key(&SimulationEventTO::session_id, "session_id", ColumnType::INTEGER,
                        "simulation_sessions", "id")
        .add_member(&SimulationEventTO::timestamp, "timestamp", ColumnType::REAL)
        .add_member(&SimulationEventTO::event_type, "event_type", ColumnType::TEXT)
        .add_member(&SimulationEventTO::description, "description", ColumnType::TEXT)
        .add_member(&SimulationEventTO::platform_id, "platform_id", ColumnType::TEXT, false, false, true)
        .add_member(&SimulationEventTO::event_data, "event_data", ColumnType::BLOB, false, false, true);
    return schema;
}