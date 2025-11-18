target_sources(${MSD_SIM_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryFactory.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utils.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryFactory.hpp
# Add more headers here as needed
    PARENT_SCOPE
)
