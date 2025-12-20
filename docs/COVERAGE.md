# Code Coverage Guide

This guide explains how to generate and view code coverage reports for the MSD-CPP project.

## Prerequisites

### macOS (using Homebrew)
```bash
brew install lcov
```

### Linux (Ubuntu/Debian)
```bash
sudo apt-get install lcov
```

### Linux (Fedora/RHEL)
```bash
sudo dnf install lcov
```

## Building with Coverage Enabled

### Using Conan (Recommended)

1. Install dependencies with coverage enabled:
```bash
# For Debug build with coverage
conan install . --build=missing -s build_type=Debug -o "&:enable_coverage=True"
```

2. Configure CMake using the coverage preset:
```bash
cmake --preset coverage-debug
```

3. Build the project:
```bash
cmake --build build/Debug-Coverage
```

### Manual CMake Configuration (Alternative)

1. Configure CMake with coverage enabled:
```bash
cmake -S . -B build -DENABLE_COVERAGE=ON
```

2. Build the project:
```bash
cmake --build build
```

## Generating Coverage Reports

### For msd-sim tests

Run the coverage target:
```bash
# With Conan build
cmake --build build/Debug-Coverage --target coverage-msd-sim

# With manual CMake build
cmake --build build --target coverage-msd-sim
```

This will:
- Run all msd-sim tests
- Collect coverage data
- Generate an HTML report in `build/coverage/msd-sim/` (or `build/Debug-Coverage/coverage/msd-sim/`)

### Viewing the Report

Open the HTML report in your browser:
```bash
# macOS (Conan build)
open build/Debug-Coverage/coverage/msd-sim/index.html

# macOS (Manual build)
open build/coverage/msd-sim/index.html

# Linux (Conan build)
xdg-open build/Debug-Coverage/coverage/msd-sim/index.html

# Linux (Manual build)
xdg-open build/coverage/msd-sim/index.html
```

## Understanding Coverage Output

The coverage report shows:
- **Line coverage**: Percentage of lines executed during tests
- **Function coverage**: Percentage of functions called during tests
- **Branch coverage**: Percentage of conditional branches taken

### Coverage Summary

A text summary is printed to the console when you run the coverage target. Look for output like:
```
Overall coverage rate:
  lines......: 85.2% (1234 of 1448 lines)
  functions..: 78.5% (123 of 157 functions)
```

## Coverage Best Practices

1. **Aim for high coverage**: Generally 80%+ line coverage is a good target
2. **Focus on critical paths**: Ensure error handling and edge cases are tested
3. **Don't obsess over 100%**: Some code (defensive checks, unreachable code) may not need coverage
4. **Review uncovered code**: Use the HTML report to identify gaps in your tests

## Troubleshooting

### "lcov: command not found"
Install lcov using the package manager for your platform (see Prerequisites).

### No coverage data generated
- Ensure you built with `-DENABLE_COVERAGE=ON`
- Verify tests are actually running: `ctest --test-dir build`
- Check that the compiler is GCC or Clang (coverage doesn't work with MSVC)

### Low coverage numbers
- Add more test cases to exercise different code paths
- Check that tests are actually calling the code you think they are
- Review the HTML report to see which specific lines aren't covered

## Integration with CI/CD

You can integrate coverage into your CI pipeline:

```bash
# In your CI script (using Conan)
conan install . --build=missing -s build_type=Debug -o "&:enable_coverage=True"
cmake --preset coverage-debug
cmake --build build/Debug-Coverage
cmake --build build/Debug-Coverage --target coverage-msd-sim

# Or without Conan
cmake -S . -B build -DENABLE_COVERAGE=ON
cmake --build build
cmake --build build --target coverage-msd-sim
```

Consider uploading coverage reports to services like:
- Codecov (https://codecov.io)
- Coveralls (https://coveralls.io)
- SonarQube

## Advanced Usage

### Filtering Coverage Results

The coverage target automatically excludes:
- System headers (`/usr/*`)
- Test files (`*/test/*`)
- Build artifacts (`*/build/*`)

To modify filters, edit the `LCOV_PATH --remove` command in [msd/msd-sim/test/CMakeLists.txt](../msd/msd-sim/test/CMakeLists.txt:50).

### Coverage for Specific Tests

Run specific tests and generate coverage:
```bash
cd build
lcov --directory . --zerocounters
ctest -R YourSpecificTest
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/usr/*' '*/test/*' --output-file filtered.info
genhtml filtered.info --output-directory custom-coverage
```

## Notes

- Coverage adds overhead, so don't use it for production builds
- The `-O0` flag disables optimizations for accurate coverage tracking
- Coverage data is stored in `.gcda` and `.gcno` files in the build directory
