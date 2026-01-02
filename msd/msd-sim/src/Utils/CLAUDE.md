# Utils Module Architecture Guide

> This document provides architectural context for AI assistants and developers.

## Project Overview

**Utils** is a utility module within `msd-sim` that provides common helper functions used throughout the simulation library. Currently contains numerical comparison utilities for floating-point values.

## Architecture Overview

### High-Level Architecture

The Utils module provides standalone utility functions:

```
msd_sim namespace
    └── Utils
        ├── almostEqual()     # Floating-point comparison
        └── TOLERANCE         # Default tolerance constant
```

### Core Components

| Component | Location | Purpose |
|-----------|----------|---------|
| almostEqual | `utils.hpp` | Floating-point comparison with tolerance |
| TOLERANCE | `utils.hpp` | Default comparison tolerance (1e-10) |

---

## Component Details

### almostEqual

**Location**: `utils.hpp`
**Type**: Header-only template function

#### Purpose
Helper function for comparing floating-point numbers with a configurable tolerance. Essential for numerical comparisons in physics simulations where exact equality is unreliable.

#### Key Interfaces
```cpp
// Default tolerance for floating-point comparisons
constexpr double TOLERANCE = 1e-10;

/**
 * @brief Compare two floating-point values with tolerance.
 *
 * @tparam T Floating-point type (float, double, long double)
 * @param a First value
 * @param b Second value
 * @param tolerance Comparison tolerance (default: 1e-10)
 * @return true if |a - b| < tolerance
 */
template <std::floating_point T>
bool almostEqual(T a, T b, double tolerance = TOLERANCE);
```

#### Usage Example
```cpp
#include "msd-sim/src/Utils/utils.hpp"

double computed = 1.0000000001;
double expected = 1.0;

if (msd_sim::almostEqual(computed, expected)) {
  std::cout << "Values are equal within tolerance" << std::endl;
}

// Custom tolerance
if (msd_sim::almostEqual(computed, expected, 1e-6)) {
  std::cout << "Values are equal within 1e-6" << std::endl;
}

// Works with float
float a = 0.1f + 0.2f;
float b = 0.3f;
if (msd_sim::almostEqual(a, b, 1e-6)) {
  std::cout << "Floats are equal within tolerance" << std::endl;
}
```

#### Thread Safety
**Thread-safe** — Pure function with no state.

#### Error Handling
No error handling — simple mathematical comparison.

---

## Design Patterns in Use

### Template Metaprogramming
**Used in**: almostEqual function
**Purpose**: Works with any floating-point type while maintaining type safety via `std::floating_point` concept.

---

## Build & Configuration

### Build Requirements
- **C++ Standard**: C++20 (for `std::floating_point` concept)
- **Dependencies**: None (uses only standard library)

### Building This Component

This module is built as part of `msd-sim`:

```bash
# From project root
cmake --preset conan-debug
cmake --build --preset debug-sim-only
```

---

## Future Development

### Potential Additions
- **Unit conversions**: Degrees to radians, etc.
- **Mathematical utilities**: Clamp, lerp, smoothstep
- **Numerical integration**: Euler, RK4 helpers
- **Random number utilities**: For stochastic simulations

---

## Coding Standards

This module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards).

Key standards applied in this module:
- **Naming**: `camelCase` for functions, `SCREAMING_CASE` for constants
- **Templates**: Uses C++20 concepts for type constraints

See the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Getting Help

### For AI Assistants
1. This document provides complete architectural context for the Utils module
2. Review [msd-sim/CLAUDE.md](../../CLAUDE.md) for overall simulation architecture
3. Check [root CLAUDE.md](../../../../CLAUDE.md) for project-wide conventions

### For Developers
- **Floating-point comparison**: Use `almostEqual()` instead of `==`
- **Custom tolerance**: Pass explicit tolerance for specific precision requirements
