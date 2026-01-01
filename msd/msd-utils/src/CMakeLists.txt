target_sources(${MSD_UTILS_NAME} PRIVATE
    PathUtils.cpp
)

set(
    HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/PathUtils.hpp
# Add more headers here as needed
    PARENT_SCOPE
)