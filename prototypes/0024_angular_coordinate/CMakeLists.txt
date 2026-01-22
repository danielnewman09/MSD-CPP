cmake_minimum_required(VERSION 3.15)
project(0024_angular_coordinate_prototypes)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use Conan-generated packages from main project
# Build with: conan install ../.. --build=missing -s build_type=Release -o "&:enable_benchmarks=True"
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../build/Release/generators")

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(benchmark REQUIRED)

# Compiler flags for optimization
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
endif()

# P1: Normalization Performance (Google Benchmark)
add_executable(p1_normalization_performance p1_normalization_performance.cpp)
target_link_libraries(p1_normalization_performance
    Eigen3::Eigen
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(p1_normalization_performance PRIVATE -O3)

# P2: Internal Storage Comparison (Google Benchmark)
add_executable(p2_internal_storage p2_internal_storage.cpp)
target_link_libraries(p2_internal_storage
    Eigen3::Eigen
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(p2_internal_storage PRIVATE -O3)

# P3: Numerical Stability (correctness test, not benchmark)
add_executable(p3_numerical_stability p3_numerical_stability.cpp)
target_link_libraries(p3_numerical_stability Eigen3::Eigen)
target_compile_options(p3_numerical_stability PRIVATE -O3)

# P4: Eigen Integration (Google Benchmark)
add_executable(p4_eigen_integration p4_eigen_integration.cpp)
target_link_libraries(p4_eigen_integration
    Eigen3::Eigen
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(p4_eigen_integration PRIVATE -O3)

# P5: Shared Interface Pattern (Google Benchmark)
add_executable(p5_shared_interface p5_shared_interface.cpp)
target_link_libraries(p5_shared_interface
    Eigen3::Eigen
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(p5_shared_interface PRIVATE -O3)

# P1b: Lazy Normalization Optimization (Google Benchmark)
add_executable(p1b_lazy_optimization p1b_lazy_optimization.cpp)
target_link_libraries(p1b_lazy_optimization
    Eigen3::Eigen
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(p1b_lazy_optimization PRIVATE -O3)

# P1c: Hybrid Eager Normalization (Google Benchmark)
add_executable(p1c_hybrid_eager p1c_hybrid_eager.cpp)
target_link_libraries(p1c_hybrid_eager
    Eigen3::Eigen
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(p1c_hybrid_eager PRIVATE -O3)

# P1d: Deferred Writeback Normalization (Google Benchmark)
add_executable(p1d_deferred_writeback p1d_deferred_writeback.cpp)
target_link_libraries(p1d_deferred_writeback
    Eigen3::Eigen
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(p1d_deferred_writeback PRIVATE -O3)

# P1e: Complete Normalization Coverage (Google Benchmark)
add_executable(p1e_complete_coverage p1e_complete_coverage.cpp)
target_link_libraries(p1e_complete_coverage
    Eigen3::Eigen
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(p1e_complete_coverage PRIVATE -O3)
