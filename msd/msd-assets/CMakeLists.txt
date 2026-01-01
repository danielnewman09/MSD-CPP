# Define library name
set(MSD_ASSETS_NAME msd_assets)
set(MSD_SIM_TEST_NAME msd_assets_test)

# Create the library
add_library(${MSD_ASSETS_NAME})
set_target_properties(${MSD_ASSETS_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Find required packages first
find_package(spdlog REQUIRED)
find_package(cpp_sqlite REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Add subdirectories to collect sources from each module
add_subdirectory(src)
add_subdirectory(test)

# Link libraries after finding them
target_link_libraries(${MSD_ASSETS_NAME}
        PUBLIC
          msd_transfer
          cpp_sqlite::cpp_sqlite
          Eigen3::Eigen
        PRIVATE
          spdlog::spdlog)

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

