# Physics module tests

target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ContactManifoldStabilityTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/EdgeContactTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/EnergyAccountingTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/LinearCollisionTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParameterIsolationTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PerContactDepthTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RotationalCollisionTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RotationalEnergyTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RotationDampingTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TangentBasisTest.cpp
)


