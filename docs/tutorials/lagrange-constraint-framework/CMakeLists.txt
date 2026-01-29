# Tutorial: Lagrange Multiplier Constraint Framework
# Ticket: 0031_generalized_lagrange_constraints
#
# Standalone build - no external dependencies required.
# This tutorial demonstrates constraint-based physics using
# Lagrange multipliers with Baumgarte stabilization.
#
# Build:
#   mkdir build && cd build
#   cmake ..
#   make
#
# Run:
#   ./example

cmake_minimum_required(VERSION 3.14)
project(lagrange_constraint_tutorial VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable warnings for educational quality
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Main tutorial executable
add_executable(example example.cpp)

# Optional: Add a test target that validates constraints
enable_testing()
add_test(NAME constraint_tutorial_runs COMMAND example)
