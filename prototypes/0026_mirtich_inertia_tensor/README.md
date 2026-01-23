# Prototype: Mirtich Inertia Tensor Algorithm

This prototype validates the C++ implementation of Brian Mirtich's algorithm for computing polyhedral mass properties.

## Purpose

Validate that the Mirtich algorithm (from volInt.c) can be correctly implemented in C++ and produces results matching analytical solutions within floating-point precision.

## Building

```bash
cd prototypes/0026_mirtich_inertia_tensor
cmake -S . -B build
cmake --build build
```

## Running

```bash
./build/mirtich_prototype
```

## Expected Output

All three test cases should pass:
- Unit Cube: Inertia tensor matches I = (m/6)*Identity
- Rectangular Box: Inertia tensor matches analytical formula for box
- Regular Tetrahedron: Diagonal elements equal, matching I = (m*L²/20)*Identity

All errors should be exactly 0.0 (within floating-point representation).

## Files

- `mirtich_prototype.cpp` - Standalone implementation of Mirtich algorithm
- `CMakeLists.txt` - Build configuration
- `prototype-results.md` - Detailed validation results and findings
- `README.md` - This file

## Reference

- Source algorithm: `docs/tutorials/inertia-tensor-convex-hull/source_code/volInt.c`
- Paper: Brian Mirtich, "Fast and Accurate Computation of Polyhedral Mass Properties", Journal of Graphics Tools, Vol 1, No 1, 1996
