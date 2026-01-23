# CMakeLists.txt for Mirtich Algorithm Tutorial
#
# This builds the standalone tutorial example without any project dependencies.
# The example demonstrates inertia tensor computation using Mirtich's algorithm.
#
# Build instructions:
#   mkdir -p build && cd build
#   cmake ..
#   make
#   ./example

cmake_minimum_required(VERSION 3.15)
project(MirtichInertiaTenosrTutorial LANGUAGES CXX)

# Require C++17 for std::array initializer lists
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create the example executable
add_executable(example example.cpp)

# Enable warnings for educational purposes
target_compile_options(example PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
