# msd-assets Library Architecture

> Architectural context for asset management and geometry handling.
> For API details, use MCP: `find_class AssetRegistry`, `get_class_members Geometry`, etc.

## Architecture Overview

```
AssetRegistry (Singleton cache for assets)
    └── Asset (Complete asset with visual/collision geometry)
        ├── VisualGeometry (Vertex, color, normal data)
        └── CollisionGeometry (Raw 3D coordinates)

GeometryFactory (Procedural primitive generation)
    └── Cube, Pyramid, Wireframe shapes

STLLoader (File import)
    └── Binary and ASCII STL formats
```

---

## Design Decisions

### Why Template-Based Geometry?

`BaseGeometry<T>` uses compile-time specialization to handle both visual geometry (with normals/colors) and collision geometry (raw vertices) with a single implementation. This avoids code duplication while maintaining type safety.

### Why `std::optional<std::reference_wrapper>`?

Cache lookups return `std::optional<std::reference_wrapper<const Asset>>` because:
1. **Cache miss is expected**: Asset may not exist in database
2. **No ownership transfer**: Registry owns assets, callers get non-owning access
3. **Clear semantics**: `std::nullopt` vs exception for "not found"

### Why Singleton AssetRegistry?

- Single database connection per application
- Centralized asset cache prevents duplicate loads
- Thread-safe via internal mutex

---

## Cross-Cutting Concerns

### Thread Safety

| Component | Safety |
|-----------|--------|
| AssetRegistry | Thread-safe (mutex-protected cache) |
| Asset, Geometry | Immutable after construction |
| GeometryFactory, STLLoader | Stateless static methods |

### Error Handling

- **Cache miss**: Returns `std::optional` (no exception)
- **Database error**: Throws during construction
- **Invalid data**: Throws `std::runtime_error`
- **File not found**: STLLoader returns `nullptr`

### Memory Ownership

| Data | Owner |
|------|-------|
| Cached assets | AssetRegistry (value in map) |
| Geometry vertices | Geometry instance (`std::vector`) |
| Loaded STL data | Caller (`std::unique_ptr`) |

---

## Integration with Other Libraries

```
msd-assets
    ├── Depends on: msd-transfer (database DTOs)
    └── Used by: msd-sim (collision geometry), msd-gui (visual geometry)
```

**Key usage pattern**:
1. `AssetRegistry` loads assets from SQLite database
2. `msd-sim` uses `CollisionGeometry` for ConvexHull construction
3. `msd-gui` uses `VisualGeometry` for GPU rendering

---

## Querying This Module

Use MCP tools for API details:
- `find_class AssetRegistry` — Registry interface and caching
- `find_class Asset` — Asset container structure
- `find_class BaseGeometry` — Template geometry container
- `find_class GeometryFactory` — Primitive generation methods
- `find_class STLLoader` — File loading interface
- `search_documentation geometry` — Find geometry-related docs

---

## Related Documentation

- **Database DTOs**: [msd-transfer/CLAUDE.md](../msd-transfer/CLAUDE.md)
- **Collision usage**: [msd-sim/src/Physics/Collision/CLAUDE.md](../msd-sim/src/Physics/Collision/CLAUDE.md)
- **Diagrams**: `docs/msd/msd-assets/` (asset-registry.puml, asset-geometry.puml, etc.)
