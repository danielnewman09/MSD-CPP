target_sources(${MSD_SIM_TEST_NAME} PRIVATE
    AngularCoordinateTest.cpp
    AngularCoordinateTransferTest.cpp
    AngularRateTest.cpp
    AngularRateTransferTest.cpp
    CoordinateTransferTest.cpp
)

set(
    HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Engine.hpp
# Add more headers here as needed
    PARENT_SCOPE
)