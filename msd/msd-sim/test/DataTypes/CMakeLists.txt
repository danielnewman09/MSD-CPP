target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    AngularCoordinateTest.cpp
    AngularCoordinateTransferTest.cpp
    AngularAccelerationTest.cpp
    AngularAccelerationTransferTest.cpp
    AngularVelocityTest.cpp
    AngularVelocityTransferTest.cpp
    CoordinateTransferTest.cpp
    Vector3DTransferTest.cpp
    Vector4DTransferTest.cpp
)

set(
    HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Engine.hpp
# Add more headers here as needed
    PARENT_SCOPE
)