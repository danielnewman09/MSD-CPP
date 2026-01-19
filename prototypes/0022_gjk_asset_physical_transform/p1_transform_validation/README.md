# Prototype P1: Transformation Correctness Validation

## Question
Does the transformation pipeline (`globalToLocalRelative` → vertex search → `localToGlobal`) produce correct world-space support vertices for known geometries and transforms?

## Success Criteria
- Support vertices match analytical predictions for unit cube with translation-only transform
- Support vertices match analytical predictions for unit cube with rotation-only transform (90° about z-axis)
- Support vertices match analytical predictions for unit cube with combined translation + rotation
- Identity transform produces identical results to raw ConvexHull support function

## Building

```bash
cd prototypes/0022_gjk_asset_physical_transform/p1_transform_validation
mkdir build && cd build
cmake ..
make
```

## Running

```bash
./p1_transform_validation
```

## What This Validates
This prototype validates the core transformation logic that GJK will use:
1. World-space search direction transformed to local space (rotation only)
2. Support function finds vertex in local space
3. Support vertex transformed back to world space (rotation + translation)

This ensures that the transformation pipeline produces mathematically correct results before implementing it in the actual GJK algorithm.
