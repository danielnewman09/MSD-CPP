target_sources(${MSD_SIM_NAME} PRIVATE
    # ${CMAKE_CURRENT_SOURCE_DIR}/WorldModel.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Acceleration.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AngularAcceleration.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AngularCoordinate.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AngularVelocity.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Coordinate.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Facet.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Velocity.hpp
# Add more headers here as needed
    PARENT_SCOPE
)
