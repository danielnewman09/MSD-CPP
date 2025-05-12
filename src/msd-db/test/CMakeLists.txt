# msd-db/test/CMakeLists.txt
find_package(GTest REQUIRED)

set(MSD_DB_TEST_NAME msd_db_test)

add_executable(${MSD_DB_TEST_NAME} 
    DatabaseTest.cpp
)

target_link_libraries(${MSD_DB_TEST_NAME}
    PRIVATE
        msd_db
    PUBLIC
        GTest::gtest
        GTest::gtest_main
)

target_include_directories(${MSD_DB_TEST_NAME} PUBLIC
    ${GTEST_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Set C++17 standard for filesystem support
target_compile_features(${MSD_DB_TEST_NAME} PUBLIC cxx_std_17)

# Add test to CTest
include(GoogleTest)
gtest_discover_tests(${MSD_DB_TEST_NAME})