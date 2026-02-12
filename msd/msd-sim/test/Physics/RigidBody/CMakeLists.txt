# RigidBody module tests
# Ticket: 0032a_two_body_constraint_infrastructure

target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/AssetDynamicStateTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AssetDynamicStateTransferTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AssetEnvironmentTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AssetInertialStaticRecordTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AssetPhysicalStaticRecordTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConvexHullTest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InertialStateTransferTest.cpp
)
