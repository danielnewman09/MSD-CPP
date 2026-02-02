# Physics module tests

target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ConvexHullTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GJKTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ForceApplicationScaffoldingTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InertialCalculationsTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/EPATest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CollisionHandlerTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AssetInertialTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Integration/QuaternionPhysicsTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TangentBasisTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/EnergyMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/M8ScenarioBuilder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FrictionEnergyTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FrictionValidationTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FrictionStabilityTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FrictionEnergyDiagnosticTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ObliqueContactDiagnosticTest.cpp
)

# Include subdirectories with additional tests
add_subdirectory(Constraints)
add_subdirectory(RigidBody)
