# Prototype P2: Horizon Edge Construction Robustness

## Purpose
Validate that the horizon edge detection and visible face removal maintain valid polytope topology under various expansion scenarios.

## Question
Does EPA's `buildHorizonEdges()` correctly identify the boundary between visible and non-visible faces, ensuring:
- No duplicate edges in the horizon
- All new faces have valid normals (non-zero, finite)
- Polytope remains watertight after each expansion (each edge shared by exactly 2 faces)

## Success Criteria
- **No duplicate edges**: All horizon edges are unique
- **Valid normals**: All face normals are non-zero and finite
- **Watertight topology**: Every edge shared by exactly 2 faces (manifold mesh)
- **No corruption**: Passes validation for 100+ expansion iterations

## Build Instructions

```bash
cd prototypes/0027a_expanding_polytope_algorithm/p2_horizon/
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
./prototype
```

## Test Cases

1. **Simple expansion** (4 points): Basic horizon validation
2. **Many expansions** (20 points): Stress test for topology corruption
3. **Near-coplanar expansion**: Edge case for numerical precision

## Validation Checks

For each expansion, the prototype validates:
1. **Pre-expansion**: Polytope topology is valid
2. **Horizon uniqueness**: No duplicate edges in horizon
3. **Post-expansion**: Polytope remains valid (watertight, valid normals)

## Implementation Notes

- **Epsilon**: 1e-6 (matches design specification)
- **Watertight check**: Every edge must be shared by exactly 2 faces
- **Edge normalization**: Edges stored with min vertex first for consistent comparison

## Expected Outcome

If successful, this confirms that EPA's topology management is robust. Failure would indicate need for:
- `std::set` with custom comparator for horizon edges
- Explicit duplicate edge detection
- More robust face normal computation

## Time Box
2 hours
