# ConvexHull Unit Tests

Comprehensive unit tests for the `ConvexHull` class using Google Test.

## Test Coverage

### Constructor Tests
- ✅ Default constructor
- ✅ Constructor with valid points
- ✅ Interior point removal (ensures only hull vertices retained)
- ✅ Error handling for empty/insufficient points
- ✅ Degenerate cases (coplanar points)

### Factory Methods
- ✅ `fromGeometry()` with Geometry objects
- ✅ `fromPoints()` with point clouds
- ✅ Integration with GeometryFactory (cube, pyramid)

### Geometric Properties
- ✅ **Volume calculation**
  - Cube volume accuracy
  - Tetrahedron volume
  - Volume caching
- ✅ **Surface area calculation**
  - Cube surface area accuracy
  - Caching verification
- ✅ **Centroid computation**
  - Origin-centered objects
  - Offset objects

### Point Containment
- ✅ Points inside hull
- ✅ Points outside hull
- ✅ Points on boundary
- ✅ Epsilon tolerance testing
- ✅ All vertices contained

### Signed Distance Queries
- ✅ Negative distance for interior points
- ✅ Positive distance for exterior points
- ✅ Zero distance for surface points
- ✅ Distance magnitude accuracy

### Bounding Box
- ✅ Correct min/max calculation
- ✅ Contains all vertices

### Facet Properties
- ✅ Triangulation (all facets are triangles)
- ✅ Normal vector normalization
- ✅ Valid vertex indices
- ✅ Expected facet counts (12 for cube, 4 for tetrahedron)

### Copy/Move Semantics
- ✅ Copy constructor
- ✅ Copy assignment
- ✅ Move constructor
- ✅ Move assignment

### Edge Cases
- ✅ Duplicate points handling
- ✅ Coplanar point rejection

## Running the Tests

### Build and Run All Tests

```bash
# From project root
cmake --build --preset conan-release
ctest --preset conan-release --output-on-failure
```

### Run Only ConvexHull Tests

```bash
# Build first
cmake --build --preset conan-release

# Run with filter
./build/Release/msd/msd-sim/msd_sim_test --gtest_filter="ConvexHullTest.*"
```

### Run Specific Test Case

```bash
./build/Release/msd/msd-sim/msd_sim_test --gtest_filter="ConvexHullTest.VolumeOfCube"
```

### Verbose Output

```bash
./build/Release/msd/msd-sim/msd_sim_test --gtest_filter="ConvexHullTest.*" --gtest_brief=0
```

## Test Statistics

- **Total Test Cases**: 47
- **Test Suites**: 11 categories
- **Coverage Areas**:
  - Constructors & Factory Methods
  - Volume & Surface Area
  - Centroid Calculation
  - Point Containment
  - Distance Queries
  - Bounding Box
  - Facet Properties
  - Copy/Move Semantics
  - Edge Cases
  - Integration Tests

## Expected Test Output

```
[==========] Running 47 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 47 tests from ConvexHullTest
[ RUN      ] ConvexHullTest.DefaultConstructor
[       OK ] ConvexHullTest.DefaultConstructor (0 ms)
[ RUN      ] ConvexHullTest.ConstructorWithValidPoints
[       OK ] ConvexHullTest.ConstructorWithValidPoints (1 ms)
...
[----------] 47 tests from ConvexHullTest (X ms total)

[----------] Global test environment tear-down
[==========] 47 tests from 1 test suite ran. (X ms total)
[  PASSED  ] 47 tests.
```

## Adding New Tests

To add new ConvexHull tests:

1. **Add test function** to `ConvexHullTest.cpp`:
```cpp
TEST(ConvexHullTest, YourNewTestName) {
    // Arrange
    auto points = createCubePoints(2.0f);
    ConvexHull hull(points);

    // Act
    float result = hull.someMethod();

    // Assert
    EXPECT_FLOAT_EQ(result, expectedValue);
}
```

2. **Rebuild and run**:
```bash
cmake --build --preset conan-release
ctest --preset conan-release -R ConvexHullTest
```

## Test Helpers

The test file includes helper functions for creating common geometries:

- `createCubePoints(size)` - 8 corner points of a cube
- `createTetrahedronPoints()` - 4 vertices of a tetrahedron
- `createPointsWithInterior()` - Cube points + interior point

Example usage:
```cpp
auto points = createCubePoints(2.0f);
ConvexHull hull(points);
EXPECT_EQ(hull.getVertexCount(), 8);
```

## Debugging Failed Tests

### Get detailed output:
```bash
./build/Release/msd/msd-sim/msd_sim_test \
    --gtest_filter="ConvexHullTest.FailingTest" \
    --gtest_print_time=1
```

### Enable Qhull debugging:
Add to test temporarily:
```cpp
// In your test case
std::cout << "Vertices: " << hull.getVertexCount() << std::endl;
std::cout << "Volume: " << hull.volume() << std::endl;
```

### Check for memory leaks (macOS):
```bash
leaks --atExit -- ./build/Release/msd/msd-sim/msd_sim_test \
    --gtest_filter="ConvexHullTest.*"
```

## Continuous Integration

These tests should be run in CI on every commit. Expected behavior:

- ✅ All 47 tests pass
- ✅ No memory leaks
- ✅ Run time < 5 seconds
- ✅ No warnings from Qhull

## Known Limitations

1. **Floating-point precision**: Tests use `EXPECT_NEAR` with epsilon values for geometric calculations
2. **Qhull dependency**: Tests require Qhull to be properly installed via Conan
3. **Platform differences**: Slight numerical differences may occur across platforms (handled by epsilon tolerances)

## Future Test Additions

Consider adding tests for:
- [ ] Very large point clouds (performance)
- [ ] Nearly-degenerate configurations
- [ ] Stress testing with random point clouds
- [ ] Comparison with analytical formulas for standard shapes
- [ ] Thread-safety tests
