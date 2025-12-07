target_sources(${MSD_ASSETS_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/STLLoader.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryFactory.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/STLLoader.hpp
    PARENT_SCOPE
)
