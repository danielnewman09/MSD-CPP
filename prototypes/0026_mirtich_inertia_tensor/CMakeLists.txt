cmake_minimum_required(VERSION 3.16)
project(prototype_mirtich_inertia_tensor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

add_executable(mirtich_prototype mirtich_prototype.cpp)
