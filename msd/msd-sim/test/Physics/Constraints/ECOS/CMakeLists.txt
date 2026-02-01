# ECOS utilities tests
# Ticket: 0035b_box_constrained_asm_solver
# Ticket: 0035b3_ecos_problem_construction

target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ECOSSparseMatrixTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FrictionConeSpecTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ECOSDataTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ECOSProblemBuilderTest.cpp
)
