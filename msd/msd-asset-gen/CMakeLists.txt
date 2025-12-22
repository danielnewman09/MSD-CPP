# Define library name
set(MSD_ASSET_GEN_NAME msd_asset_gen)
set(MSD_ASSET_GEN_TEST_NAME msd_asset_gen_test)

# Create the library
add_library(${MSD_ASSET_GEN_NAME})
set_target_properties(${MSD_ASSET_GEN_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Find required packages first
find_package(spdlog REQUIRED)

# Add subdirectories to collect sources from each module
add_subdirectory(src)
add_subdirectory(test)

# Link libraries after finding them
target_link_libraries(${MSD_ASSET_GEN_NAME}
        PUBLIC
          msd_sim
          msd_assets
        PRIVATE
          spdlog::spdlog)

# Add include directories to support "msd-assets/src/..." includes
target_include_directories(${MSD_ASSET_GEN_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include>
)


# Installation rules
install(TARGETS ${MSD_ASSET_GEN_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

# Install headers using the variable
install(FILES ${HEADER_FILES}
        DESTINATION include/${MSD_ASSET_GEN_NAME}) 

