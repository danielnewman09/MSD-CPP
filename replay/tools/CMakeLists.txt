# Ticket: 0056k_example_workflow_test_recording
# Build configuration for test recording generator

add_executable(generate_test_recording generate_test_recording.cpp)

# Link against msd_sim (provides all transitive dependencies)
target_link_libraries(generate_test_recording PRIVATE msd_sim)

# Set output directory to build/Debug/debug for consistency
set_target_properties(generate_test_recording PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/debug"
)
