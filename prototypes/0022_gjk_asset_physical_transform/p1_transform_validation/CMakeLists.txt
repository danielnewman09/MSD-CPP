cmake_minimum_required(VERSION 3.16)
project(prototype_p1_transform_validation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories from main codebase
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../msd
)

add_executable(p1_transform_validation
  main.cpp
)

target_link_libraries(p1_transform_validation
  Eigen3::Eigen
)
