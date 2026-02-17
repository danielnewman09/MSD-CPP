cmake_minimum_required(VERSION 3.16)
project(prototype_p2_performance)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(Eigen3 REQUIRED)
find_package(nlopt REQUIRED)

add_executable(p2_performance main.cpp)

target_link_libraries(p2_performance
  PRIVATE
    Eigen3::Eigen
    nlopt::nlopt
)
