# Ticket: 0011_add_google_benchmark
# Design: docs/designs/0011_add_google_benchmark/design.md

# Find Google Benchmark package
find_package(benchmark REQUIRED)

# Create benchmark executable
add_executable(msd_sim_bench
  ConvexHullBench.cpp
  GJKBench.cpp
)

# Link dependencies
target_link_libraries(msd_sim_bench
  PRIVATE
    msd_sim
    benchmark::benchmark
    benchmark::benchmark_main
)

# Ensure benchmarks compile in Release mode for accurate measurements
# Even if overall build is Debug
target_compile_options(msd_sim_bench PRIVATE
  $<$<CONFIG:Debug>:-O2 -DNDEBUG>
)

# Include directories
target_include_directories(msd_sim_bench PRIVATE
  ${CMAKE_SOURCE_DIR}
)

# Set C++20 standard
target_compile_features(msd_sim_bench PRIVATE cxx_std_20)
