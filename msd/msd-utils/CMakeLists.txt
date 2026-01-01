# Define library name
set(MSD_UTILS_NAME msd_utils)
set(MSD_UTILS_TEST_NAME msd_utils_test)

# Create the library
add_library(${MSD_UTILS_NAME})
set_target_properties(${MSD_UTILS_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Add subdirectories to collect sources from each module
add_subdirectory(src)
add_subdirectory(test)

# Link libraries after finding them
target_link_libraries(${MSD_UTILS_NAME}
        PUBLIC
        )

# Add include directories to support "msd-sim/src/..." includes
target_include_directories(${MSD_UTILS_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include>
)

# Installation rules
install(TARGETS ${MSD_UTILS_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

# Install headers using the variable
install(FILES ${HEADER_FILES}
        DESTINATION include/${MSD_UTILS_NAME})