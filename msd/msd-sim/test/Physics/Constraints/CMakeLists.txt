# Constraints module tests
# Ticket: 0031_generalized_lagrange_constraints
# Ticket: 0032a_two_body_constraint_infrastructure
# Ticket: 0033_constraint_solver_contact_tests

target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ConstraintTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ContactConstraintTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ContactConstraintFactoryTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TwoBodyConstraintTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConstraintSolverContactTest.cpp
)
