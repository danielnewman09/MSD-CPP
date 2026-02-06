cmake_minimum_required(VERSION 3.16)
project(prototype_p1_transaction_perf)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Use Conan toolchain from main project
if(EXISTS "${CMAKE_SOURCE_DIR}/../../../../build/Release/generators/conan_toolchain.cmake")
    include("${CMAKE_SOURCE_DIR}/../../../../build/Release/generators/conan_toolchain.cmake")
endif()

# Find dependencies
find_package(cpp_sqlite REQUIRED CONFIG)
find_package(spdlog REQUIRED CONFIG)
find_package(Boost REQUIRED)
find_package(SQLite3 REQUIRED)

add_executable(p1_transaction_perf main.cpp)

target_link_libraries(p1_transaction_perf PRIVATE
    cpp_sqlite::cpp_sqlite
    spdlog::spdlog
    Boost::headers
    SQLite3::SQLite3
)

# Include local headers
target_include_directories(p1_transaction_perf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
