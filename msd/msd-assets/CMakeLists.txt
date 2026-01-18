# Define library name
set(MSD_ASSETS_NAME msd_assets)
set(MSD_ASSETS_TEST_NAME msd_assets_test)

# Create the library
add_library(${MSD_ASSETS_NAME})
set_target_properties(${MSD_ASSETS_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Find required packages first
# Note: cpp_sqlite and spdlog are found transitively via msd_transfer → cpp_sqlite
find_package(Eigen3 REQUIRED NO_MODULE)

# Add subdirectories to collect sources from each module
add_subdirectory(src)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# Link libraries after finding them
# Note: cpp_sqlite::cpp_sqlite and spdlog::spdlog are provided transitively via msd_transfer
target_link_libraries(${MSD_ASSETS_NAME}
        PUBLIC
          msd_transfer
          Eigen3::Eigen)

# Add include directories to support "msd-assets/src/..." includes
target_include_directories(${MSD_ASSETS_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include>
)


# Installation rules
install(TARGETS ${MSD_ASSETS_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

# Install headers using the variable
install(FILES ${HEADER_FILES}
        DESTINATION include/${MSD_ASSETS_NAME}) 

