# Project Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/designs/` for detailed component relationships.

## Project Overview

This is a high-performance database client library providing connection pooling, query caching, and async operations for C++ applications.

## Architecture Overview

### High-Level Architecture

See: [`docs/architecture/overview.puml`](docs/architecture/overview.puml)

The system consists of three main layers:
- **Client API** — Public interface for application code
- **Core Services** — Connection pooling, caching, query execution
- **Transport** — Network communication and protocol handling

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| ConnectionPool | `include/database/` | Manages database connections | [`connection-pool.puml`](docs/designs/connection-pool/connection-pool.puml) |
| QueryCache | `include/cache/` | LRU cache for query results | [`lru-cache.puml`](docs/designs/lru-cache/lru-cache.puml) |
| AsyncExecutor | `include/async/` | Async query execution | [`async-executor.puml`](docs/designs/async-executor/async-executor.puml) |

---

## Component Details

### Connection Pool

**Location**: `include/database/`, `src/database/`  
**Diagram**: [`docs/designs/connection-pool/connection-pool.puml`](docs/designs/connection-pool/connection-pool.puml)  
**Introduced**: [Ticket: connection-pool](tickets/connection-pool.md)

#### Purpose
Manages a pool of reusable database connections to avoid the overhead of establishing new connections for each query.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `ConnectionPool` | `connection_pool.hpp` | Pool lifecycle and connection distribution |
| `PooledConnection` | `pooled_connection.hpp` | RAII wrapper for borrowed connections |
| `ConnectionConfig` | `connection_config.hpp` | Pool configuration parameters |

#### Key Interfaces
```cpp
class IConnectionPool {
public:
    virtual ~IConnectionPool() = default;
    virtual PooledConnection acquire() = 0;
    virtual PooledConnection tryAcquire(std::chrono::milliseconds timeout) = 0;
    virtual PoolStats stats() const = 0;
};
```

#### Usage Example
```cpp
auto pool = ConnectionPool::create(config);
{
    auto conn = pool->acquire();  // Borrows connection
    auto result = conn->execute("SELECT * FROM users");
}  // Connection automatically returned to pool
```

#### Thread Safety
- `acquire()` and `tryAcquire()` are thread-safe
- `PooledConnection` is not thread-safe (single-thread use)
- `stats()` provides atomic snapshot

#### Error Handling
- Throws `ConnectionPoolExhausted` if pool is empty and timeout exceeded
- Throws `ConnectionError` for underlying connection failures

#### Dependencies
- `Transport` — For creating raw connections
- `std::mutex`, `std::condition_variable` — For synchronization

---

### Query Cache (LRU)

**Location**: `include/cache/`, `src/cache/`  
**Diagram**: [`docs/designs/lru-cache/lru-cache.puml`](docs/designs/lru-cache/lru-cache.puml)  
**Introduced**: [Ticket: lru-cache](tickets/lru-cache.md)

#### Purpose
Caches query results using an LRU eviction policy to reduce redundant database calls and improve response times.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `LRUCache<K,V>` | `lru_cache.hpp` | Generic thread-safe LRU cache |
| `QueryCache` | `query_cache.hpp` | Specialized cache for query results |
| `CacheStats` | `cache_stats.hpp` | Hit/miss statistics |

#### Key Interfaces
```cpp
template<typename K, typename V>
class LRUCache {
public:
    explicit LRUCache(size_t capacity, std::chrono::milliseconds ttl);
    
    std::optional<V> get(const K& key);
    void put(const K& key, V value);
    void invalidate(const K& key);
    void clear();
    
    CacheStats stats() const;
};
```

#### Thread Safety
- All public methods are thread-safe
- Uses `std::shared_mutex` for reader-writer locking
- Statistics use atomic counters

#### Error Handling
- Returns `std::optional<V>` for cache miss (no exceptions)
- `put()` silently evicts oldest entry if at capacity

---

## Design Patterns in Use

### RAII Connection Wrapper
**Used in**: `PooledConnection`  
**Purpose**: Ensures connections are returned to pool even on exceptions

See implementation: [`docs/designs/connection-pool/connection-pool.puml`](docs/designs/connection-pool/connection-pool.puml)

### Template-Based Policy
**Used in**: `LRUCache<K,V>`  
**Purpose**: Generic cache usable with different key/value types

---

## Cross-Cutting Concerns

### Error Handling Strategy
- Use exceptions for exceptional conditions (connection failures, pool exhausted)
- Use `std::optional` or `std::expected` for expected failures (cache miss)
- All exceptions derive from `DatabaseException` base class

### Logging
- Use `spdlog` for structured logging
- Levels: TRACE (query details), DEBUG (connection lifecycle), INFO (pool stats), WARN (retries), ERROR (failures)

### Memory Management
- `std::unique_ptr` for exclusive ownership
- `std::shared_ptr` only where shared ownership is required
- Raw pointers for non-owning references (always document lifetime)

### Thread Safety Conventions
- Document thread safety in class Doxygen comment
- Use `std::shared_mutex` for read-heavy, write-light scenarios
- Prefer `std::atomic` for simple counters/flags

---

## Build & Configuration

### Build Requirements
- C++ Standard: C++17
- Compiler: GCC 9+, Clang 10+, MSVC 2019+
- Dependencies: spdlog, Catch2 (tests)

### Key Build Targets
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

### Configuration Options
| Option | Default | Description |
|--------|---------|-------------|
| `DB_ENABLE_CACHE` | `ON` | Enable query caching |
| `DB_CACHE_SIZE` | `10000` | Default cache capacity |
| `DB_POOL_SIZE` | `10` | Default connection pool size |

---

## Testing

### Test Organization
```
test/
├── unit/           # Unit tests (isolated, fast)
│   ├── cache/
│   └── database/
├── integration/    # Integration tests (real DB)
└── benchmark/      # Performance benchmarks
```

### Running Tests
```bash
ctest --test-dir build                    # All tests
ctest --test-dir build -L unit            # Unit tests only
ctest --test-dir build -L integration     # Integration tests
```

### Test Conventions
- Test files mirror source structure: `src/cache/lru_cache.cpp` → `test/unit/cache/lru_cache_test.cpp`
- Ticket references in test names: `TEST_CASE("LRUCache: evicts oldest on capacity [lru-cache]")`
- Use `[unit]` or `[integration]` tags for filtering

---

## Change History

For architectural change history, design decision rationale, and symbol-level evolution, use the traceability database MCP tools:

- `get_ticket_impact("NNNN")` — All commits, file changes, and decisions for a ticket
- `search_decisions("query")` — Search design decision rationale and trade-offs
- `why_symbol("qualified_name")` — Design decision(s) that created or modified a symbol
- `get_symbol_history("qualified_name")` — Timeline of changes to a symbol across commits
- `get_commit_context("sha")` — Full context for a commit (ticket, phase, file/symbol changes)

See [`scripts/traceability/README.md`](../../scripts/traceability/README.md) for details.

---

## Diagrams Index

| Diagram | Description | Last Updated |
|---------|-------------|--------------|
| [`overview.puml`](docs/architecture/overview.puml) | High-level system architecture | 2024-01-20 |
| [`connection-pool.puml`](docs/designs/connection-pool/connection-pool.puml) | Connection pool component | 2024-01-10 |
| [`lru-cache.puml`](docs/designs/lru-cache/lru-cache.puml) | LRU cache component | 2024-01-20 |

---

## Conventions

### Naming Conventions
- Classes: `PascalCase`
- Functions/Methods: `camelCase`
- Member variables: `snake_case_` (trailing underscore)
- Constants: `kPascalCase`
- Namespaces: `snake_case`

### Code Organization
- One class per header (generally)
- Implementation in `.cpp` unless template/inline
- Public headers in `include/`, private in `src/`

### Documentation
- Public APIs: Doxygen-style comments with `@brief`, `@param`, `@return`
- Ticket references: `@ticket {ticket-name}` in class docs, `// Ticket: {name}` at file top
- PlantUML diagrams for architectural components

---

## Getting Help

### For AI Assistants
1. Start with this document for architectural context
2. Reference the linked PlantUML diagrams for component relationships
3. Check `tickets/` for feature history and design decisions
4. Look at `docs/designs/{feature}/design.md` for detailed design rationale

### For Developers
- Design documents: `docs/designs/`
- API documentation: `docs/api/` (generated by Doxygen)
- Tickets with full context: `tickets/`
