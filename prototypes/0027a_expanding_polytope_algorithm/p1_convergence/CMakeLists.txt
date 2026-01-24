cmake_minimum_required(VERSION 3.16)
project(prototype_p1_convergence)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Use Conan-installed Eigen3 from project root
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/Debug/generators")

# Find Eigen3
find_package(Eigen3 REQUIRED)

add_executable(prototype main.cpp epa.cpp)
target_link_libraries(prototype PRIVATE Eigen3::Eigen)

add_executable(prototype_manual main_v2.cpp)
target_link_libraries(prototype_manual PRIVATE Eigen3::Eigen)
