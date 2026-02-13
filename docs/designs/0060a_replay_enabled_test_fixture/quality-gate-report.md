# Quality Gate Report: 0060a_replay_enabled_test_fixture

**Date**: 2026-02-13 14:45
**Overall Status**: FAILED

---

## Gate 1: Build Verification

**Status**: FAILED
**Exit Code**: 2

### Warnings/Errors

**Error 1**: `replay/tools/generate_test_assets.cpp:39:63`
```
error: implicit conversion increases floating-point precision: 'float' to 'double' [-Werror,-Wdouble-promotion]
   39 |   auto visualRecord = msd_assets::GeometryFactory::createCube(size);
```

**Error 2**: `replay/tools/generate_test_assets.cpp:46:66`
```
error: implicit conversion increases floating-point precision: 'float' to 'double' [-Werror,-Wdouble-promotion]
   46 |   auto collisionRecord = msd_assets::GeometryFactory::createCube(size);
```

**Root Cause**: The `createCubeAsset()` function parameter `float size` is implicitly promoted to `double` when passed to `GeometryFactory::createCube(double size)`. In Release mode, `-Wdouble-promotion` is treated as an error via `-Werror`.

**Fix**: Change the `size` parameter type from `float` to `double` in `createCubeAsset()` function signature.

---

## Gate 2: Test Verification

**Status**: NOT RUN
**Reason**: Build failed, cannot run tests

---

## Gate 3: Static Analysis (clang-tidy)

**Status**: NOT RUN
**Reason**: Build failed

---

## Gate 4: Benchmark Regression Detection

**Status**: N/A
**Reason**: No benchmarks specified in design (direct-specification ticket)

---

## Summary

| Gate | Status | Notes |
|------|--------|-------|
| Build | FAILED | 2 double-promotion errors in generate_test_assets.cpp |
| Tests | NOT RUN | Blocked by build failure |
| Static Analysis | NOT RUN | Blocked by build failure |
| Benchmarks | N/A | No benchmarks for this feature |

**Overall**: FAILED

---

## Next Steps

Quality gate failed. Return to implementer to address:

1. **Fix double-promotion errors in `replay/tools/generate_test_assets.cpp`**:
   - Change line 33 from `float size` to `double size`
   - This will eliminate implicit float-to-double conversions on lines 39 and 46

After fixes are applied:
1. Rebuild with `cmake --build --preset conan-release`
2. Re-run quality gate to verify all gates pass
3. Proceed to implementation review once quality gate passes
