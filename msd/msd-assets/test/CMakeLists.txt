
enable_testing()

find_package(GTest REQUIRED)
add_executable(${MSD_ASSETS_TEST_NAME})

# Add test source files
target_sources(${MSD_ASSETS_TEST_NAME} PRIVATE
    GeometrySerializationTest.cpp
    GeometryDatabaseTest.cpp
)

set_target_properties(${MSD_ASSETS_TEST_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${MSD_ASSETS_TEST_NAME}
    PRIVATE
        msd_assets
        GTest::GTest
        GTest::Main)

target_compile_features(${MSD_ASSETS_TEST_NAME} PRIVATE cxx_std_20)

target_include_directories(${MSD_ASSETS_TEST_NAME} PUBLIC
    ${GTEST_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
)

# Discover and register tests with CTest
include(GoogleTest)
gtest_discover_tests(${MSD_ASSETS_TEST_NAME})
