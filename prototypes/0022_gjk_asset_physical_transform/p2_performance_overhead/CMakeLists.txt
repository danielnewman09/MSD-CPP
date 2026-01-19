cmake_minimum_required(VERSION 3.16)
project(prototype_p2_performance_overhead)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories from main codebase
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../msd
)

add_executable(p2_performance_overhead
  main.cpp
)

target_link_libraries(p2_performance_overhead
  Eigen3::Eigen
)
