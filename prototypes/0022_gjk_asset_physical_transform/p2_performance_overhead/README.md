# Prototype P2: Performance Overhead Measurement

## Question
What is the performance overhead of on-the-fly transformation compared to identity-transform baseline, and does it meet the <20% threshold for typical hull sizes?

## Success Criteria
- Transformation overhead < 10% for simple hulls (< 20 vertices)
- Transformation overhead < 20% for typical hulls (20-100 vertices)
- Transformation overhead < 30% for complex hulls (100-1000 vertices)

## Building

```bash
cd prototypes/0022_gjk_asset_physical_transform/p2_performance_overhead
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

## Running

```bash
./p2_performance_overhead
```

## What This Measures
This prototype measures the computational overhead of applying transformations on-the-fly during support function queries. It compares:

1. **Identity baseline**: Support function with identity transform (no rotation/translation cost)
2. **Transformed**: Support function with non-trivial transform (rotation + translation)

The overhead is measured across various hull complexities (10 to 1000 vertices) to ensure the approach scales acceptably.
