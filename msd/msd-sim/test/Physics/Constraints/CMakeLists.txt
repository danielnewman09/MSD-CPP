# Constraints module tests
# Ticket: 0031_generalized_lagrange_constraints
# Ticket: 0032a_two_body_constraint_infrastructure
# Ticket: 0033_constraint_solver_contact_tests
# Ticket: 0034_active_set_method_contact_solver
# Ticket: 0035a_tangent_basis_and_friction_constraint
# Ticket: 0035b_box_constrained_asm_solver
# Ticket: 0039b_linear_collision_test_suite

add_subdirectory(ECOS)

target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ConstraintTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ContactConstraintTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ContactConstraintFactoryTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TwoBodyConstraintTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConstraintSolverContactTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConstraintSolverASMTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FrictionConstraintTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/JacobianLinearTest.cpp
)
