# PhysicalAsset Usage Guide

## Overview

The `PhysicalAsset` class efficiently combines visual geometry (for rendering) with physics representations (convex hull for collision detection). It implements a **lazy computation** pattern with **shared ownership** for optimal performance.

## Architecture

```
┌─────────────────────────────┐
│     msd_assets::Geometry    │  ← Visual mesh (detailed triangulation)
│   (shared across instances) │
└──────────────┬──────────────┘
               │
               │ used by
               ▼
┌─────────────────────────────┐
│   msd_sim::PhysicalAsset    │  ← Hybrid container
│                             │
│  ├─ visualGeometry_         │  ← shared_ptr to Geometry
│  ├─ collisionHull_          │  ← ConvexHull (lazy computed)
│  └─ physicsProps_           │  ← RigidBodyProperties
└─────────────────────────────┘
               │
               ├─────────────────┐
               ▼                 ▼
     ┌──────────────┐   ┌────────────────┐
     │  ConvexHull  │   │ RigidBody      │
     │              │   │ Properties     │
     │ • Collision  │   │ • Mass         │
     │ • Intersection│  │ • Inertia      │
     └──────────────┘   │ • Center of    │
                        │   Mass         │
                        └────────────────┘
```

## Basic Usage

### 1. Creating a PhysicalAsset

```cpp
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-sim/src/Physics/PhysicalAsset.hpp"

// Create visual geometry
auto cubeGeometry = std::make_shared<msd_assets::Geometry>(
    msd_assets::GeometryFactory::createCube(2.0)
);

// Create physical asset (100 kg mass)
// convex hull computed lazily on first physics access
auto asset = msd_sim::PhysicalAsset::create(cubeGeometry, 100.0f);

// OR: Pre-compute hull immediately (useful for loading screens)
auto assetPrecomputed = msd_sim::PhysicalAsset::create(
    cubeGeometry, 100.0f, /* computeHull= */ true
);
```

### 2. Using for Rendering

```cpp
// Get visual geometry for rendering (no hull computation)
const auto& geometry = asset->getVisualGeometry();

// Pass to renderer
renderer.draw(geometry);
```

### 3. Using for Physics/Collision

```cpp
// First access triggers hull computation (if lazy)
const auto& hull = asset->getCollisionHull();

// Point containment test
msd_sim::Coordinate testPoint(1.0f, 0.5f, 0.3f);
bool isInside = hull.contains(testPoint);

// Distance query
float distance = hull.signedDistance(testPoint);

// Get physics properties (auto-computed from hull)
const auto& props = asset->getPhysicsProperties();
float mass = props.getMass();
const auto& inertia = props.getInertiaTensor();
```

### 4. Sharing Assets Across Multiple Entities

```cpp
// Create asset once
auto boxAsset = msd_sim::PhysicalAsset::create(boxGeometry, 50.0f);

// Share across multiple platforms
Platform platform1(id1);
platform1.setAsset(boxAsset);  // shares visual + physics

Platform platform2(id2);
platform2.setAsset(boxAsset);  // reuses same convex hull

Platform platform3(id3);
platform3.setAsset(boxAsset);  // memory efficient!

// Each platform has independent InertialState but shares geometry/hull
```

### 5. Custom Collision Hulls

Sometimes you want a simplified collision hull that differs from the visual mesh:

```cpp
// Complex visual mesh
auto detailedMesh = loadComplexSTL("detailed_model.stl");

// Simple collision hull (box approximation)
auto simpleBoxGeometry = std::make_shared<msd_assets::Geometry>(
    msd_assets::GeometryFactory::createCube(2.0)
);
auto simpleHull = std::make_shared<msd_sim::ConvexHull>(
    msd_sim::ConvexHull::fromGeometry(*simpleBoxGeometry)
);

// Combine: detailed visuals + simple collision
auto asset = msd_sim::PhysicalAsset::createWithCustomHull(
    detailedMesh,
    simpleHull,
    100.0f
);

// Renders detailed, collides with simple box
```

### 6. Updating Mass Dynamically

```cpp
auto asset = msd_sim::PhysicalAsset::create(geometry, 100.0f);

// Later, change mass (inertia tensor automatically recomputed)
asset->setMass(150.0f);

// Physics properties reflect new mass
const auto& updatedProps = asset->getPhysicsProperties();
```

## Integration with Platform Class

Here's how to integrate PhysicalAsset with your existing Platform class:

```cpp
// In Platform.hpp
#include "msd-sim/src/Physics/PhysicalAsset.hpp"

class Platform {
public:
    Platform(uint32_t id, std::shared_ptr<PhysicalAsset> asset)
        : id_(id)
        , asset_(asset)
        , state_(/* initial state */)
    {
    }

    // Rendering
    const msd_assets::Geometry& getVisualGeometry() const {
        return asset_->getVisualGeometry();
    }

    // Physics/Collision
    bool checkCollision(const Coordinate& point) const {
        return asset_->getCollisionHull().contains(point);
    }

    // Mass properties for dynamics
    void applyForce(const Force& force) {
        const auto& props = asset_->getPhysicsProperties();
        // Use props.getMass(), props.getInertiaTensor(), etc.
    }

private:
    std::shared_ptr<PhysicalAsset> asset_;  // Shared geometry + physics
    InertialState state_;                    // Instance-specific state
    uint32_t id_;
};
```

## Performance Characteristics

### Memory Efficiency
```cpp
// Create 1000 platforms with same geometry
auto asset = PhysicalAsset::create(geometry, 100.0f);

std::vector<Platform> platforms;
for (int i = 0; i < 1000; ++i) {
    platforms.emplace_back(i, asset);  // All share ONE hull!
}

// Memory usage:
// - 1x Geometry (visual mesh)
// - 1x ConvexHull (collision)
// - 1x RigidBodyProperties (mass/inertia template)
// - 1000x InertialState (per-instance state)
```

### Computation Cost

| Operation | Cost | When |
|-----------|------|------|
| `create(..., computeHull=false)` | O(1) | Immediate (default) |
| `create(..., computeHull=true)` | O(n log n) | Immediate (optional) |
| First `getCollisionHull()` call | O(n log n) | Lazy (if not precomputed) |
| Subsequent `getCollisionHull()` | O(1) | Always (cached) |
| `getVisualGeometry()` | O(1) | Always |
| `setMass()` | O(1) | When called |

Where `n` = number of vertices in geometry

## Best Practices

### 1. **Preload during asset loading**
```cpp
// During level load / asset initialization
auto asset = PhysicalAsset::create(geometry, mass, /* computeHull= */ true);
assetManager.store("player_ship", asset);

// No hitches during gameplay
```

### 2. **Share assets aggressively**
```cpp
// BAD: Creates duplicate hulls
for (int i = 0; i < 100; ++i) {
    auto asset = PhysicalAsset::create(cubeGeom, 50.0f);
    enemies.push_back(Enemy(asset));  // 100 duplicate hulls!
}

// GOOD: Share single asset
auto enemyAsset = PhysicalAsset::create(cubeGeom, 50.0f, true);
for (int i = 0; i < 100; ++i) {
    enemies.push_back(Enemy(enemyAsset));  // 1 shared hull!
}
```

### 3. **Use custom hulls for complex meshes**
```cpp
// If visual mesh has 10,000+ vertices, use simplified collision
auto detailedVisual = loadSTL("complex_model.stl");
auto simplifiedCollision = createBoxApproximation(detailedVisual);
auto asset = PhysicalAsset::createWithCustomHull(
    detailedVisual, simplifiedCollision, mass);
```

### 4. **Check memory usage**
```cpp
size_t bytes = asset->estimateMemoryUsage();
std::cout << "Asset uses ~" << bytes / 1024 << " KB\n";
```

## Advanced: Asset Manager Pattern

For larger applications, create an asset management system:

```cpp
class AssetManager {
public:
    std::shared_ptr<PhysicalAsset> load(
        const std::string& name,
        const std::string& geometryPath,
        float mass)
    {
        // Check cache
        if (assets_.count(name)) {
            return assets_[name];
        }

        // Load geometry
        auto geometry = loadGeometry(geometryPath);

        // Create and cache asset (precompute hull)
        auto asset = PhysicalAsset::create(geometry, mass, true);
        assets_[name] = asset;

        return asset;
    }

    std::shared_ptr<PhysicalAsset> get(const std::string& name) {
        return assets_.at(name);
    }

private:
    std::unordered_map<std::string, std::shared_ptr<PhysicalAsset>> assets_;
};

// Usage
AssetManager assetMgr;
auto playerShip = assetMgr.load("player", "models/ship.stl", 1000.0f);
auto enemy1 = assetMgr.get("player");  // Reuses same asset
```

## Summary

**PhysicalAsset** provides:
- ✅ **Efficient memory**: Share geometry and hulls across instances
- ✅ **Lazy computation**: Compute convex hulls only when needed
- ✅ **Clean API**: Separate visual from physics concerns
- ✅ **Flexible**: Support custom collision hulls
- ✅ **Automatic**: Physics properties computed from geometry

Use it whenever you need both rendering and physics for the same object!
