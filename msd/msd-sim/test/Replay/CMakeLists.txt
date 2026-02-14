# Ticket: 0060a_replay_enabled_test_fixture
# Replay test fixture sources

# Add Replay fixture sources to the test executable
target_sources(${MSD_SIM_TEST_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/ReplayEnabledTest.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ReplayEnabledTestTests.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DropTest.cpp              # Ticket: 0060d
  ${CMAKE_CURRENT_SOURCE_DIR}/CollisionTest.cpp          # Ticket: 0060d
)
