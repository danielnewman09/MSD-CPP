# Test helper utilities for collision solver lambda test suite
# Ticket: 0087_collision_solver_lambda_test_suite

target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/CollisionScenario.cpp
)

target_include_directories(${MSD_SIM_TEST_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
