target_sources(${MSD_SIM_NAME} PRIVATE
    # ${CMAKE_CURRENT_SOURCE_DIR}/WorldModel.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/AngularCoordinate.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AngularRate.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Coordinate.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Facet.hpp
# Add more headers here as needed
    PARENT_SCOPE
)
