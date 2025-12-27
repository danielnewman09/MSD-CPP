target_sources(${MSD_ASSETS_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/STLLoader.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/CoordianteTO.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryTO.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryFactory.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryTraits.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/STLLoader.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/VertexTO.hpp
    PARENT_SCOPE
)
