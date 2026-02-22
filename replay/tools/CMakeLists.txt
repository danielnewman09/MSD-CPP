# Ticket: 0056k_example_workflow_test_recording
# Build configuration for test recording generator

add_executable(generate_test_recording generate_test_recording.cpp)

# Link against msd_sim (provides all transitive dependencies)
target_link_libraries(generate_test_recording PRIVATE msd_sim)

# Set output directory to build/Debug/debug for consistency
set_target_properties(generate_test_recording PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/debug"
)

# Ticket: 0060a_replay_enabled_test_fixture
# Build configuration for test asset generator

add_executable(generate_test_assets generate_test_assets.cpp)

# Link against msd_assets only (lightweight - no msd_sim dependency)
target_link_libraries(generate_test_assets PRIVATE msd_assets)

# Set output directory to build/Debug/debug for consistency
set_target_properties(generate_test_assets PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/debug"
)

# Run at build time to create test asset database
set(TEST_ASSETS_DB "${CMAKE_BINARY_DIR}/test_assets.db")
add_custom_command(
  OUTPUT "${TEST_ASSETS_DB}"
  COMMAND generate_test_assets "${TEST_ASSETS_DB}"
  DEPENDS generate_test_assets
  COMMENT "Generating test asset database"
  VERBATIM
)
add_custom_target(test_assets_db ALL DEPENDS "${TEST_ASSETS_DB}")

# Ticket: 0075-unified-contact-constraint
# Multi-body collision recording generator for replay analysis

add_executable(generate_multibody_recording generate_multibody_recording.cpp)

target_link_libraries(generate_multibody_recording PRIVATE msd_sim)

set_target_properties(generate_multibody_recording PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/debug"
)
