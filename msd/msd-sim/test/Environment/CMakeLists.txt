target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    EnvironmentTest.cpp
    MotionControllerTest.cpp
    ReferenceFrameTest.cpp
    ReferenceFrameTransferTest.cpp
    WorldModelCollisionTest.cpp
    WorldModelContactIntegrationTest.cpp
)

set(
    HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Engine.hpp
# Add more headers here as needed
    PARENT_SCOPE
)