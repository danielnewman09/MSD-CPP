# Physics module tests

target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/GJKTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ForceApplicationScaffoldingTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InertialCalculationsTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/EPATest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CollisionHandlerTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AssetInertialTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Integration/QuaternionPhysicsTest.cpp
)

# Include subdirectories with additional tests
add_subdirectory(Constraints)
add_subdirectory(RigidBody)
add_subdirectory(Collision)

