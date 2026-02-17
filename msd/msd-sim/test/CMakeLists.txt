
enable_testing()

find_package(GTest REQUIRED)
add_executable(${MSD_SIM_TEST_NAME})

# Top-level integration tests
target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    EngineIntegrationTest.cpp
)

add_subdirectory(DataRecorder)
add_subdirectory(Diagnostics)
add_subdirectory(Environment)
add_subdirectory(Physics)
add_subdirectory(DataTypes)
add_subdirectory(Replay)

set_target_properties(${MSD_SIM_TEST_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${MSD_SIM_TEST_NAME}
    PRIVATE
      msd_sim
      GTest::GTest
      GTest::Main
      NLopt::nlopt)  # Ticket: 0068a_nlopt_conan_dependency

target_compile_features(${MSD_SIM_TEST_NAME} PRIVATE cxx_std_20)
target_compile_options(${MSD_SIM_TEST_NAME} PRIVATE -Wno-unused-result)

target_include_directories(${MSD_SIM_TEST_NAME} PUBLIC
${GTEST_INCLUDE_DIRS}
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
)

# Ticket: 0060a_replay_enabled_test_fixture
# Add compile-time paths for test asset database and recordings directory
target_compile_definitions(${MSD_SIM_TEST_NAME} PRIVATE
  MSD_TEST_ASSETS_DB="${CMAKE_BINARY_DIR}/test_assets.db"
  MSD_RECORDINGS_DIR="${CMAKE_SOURCE_DIR}/replay/recordings"
)

# Ensure test asset database is built before tests
add_dependencies(${MSD_SIM_TEST_NAME} test_assets_db)

# Discover and register tests with CTest
include(GoogleTest)
gtest_discover_tests(${MSD_SIM_TEST_NAME})
