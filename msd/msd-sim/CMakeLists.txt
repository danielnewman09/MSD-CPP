# Define library name
set(MSD_SIM_NAME msd_sim)
set(MSD_SIM_TEST_NAME msd_sim_test)

# Create the library
add_library(${MSD_SIM_NAME})
set_target_properties(${MSD_SIM_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Find required packages first
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(qhull REQUIRED CONFIG)

# Add subdirectories to collect sources from each module
add_subdirectory(src)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# Link libraries after finding them
target_link_libraries(${MSD_SIM_NAME}
        PUBLIC
          msd_assets
          spdlog::spdlog
          Eigen3::Eigen
          qhull::qhullstatic_r)

# Add include directories to support "msd-sim/src/..." includes
target_include_directories(${MSD_SIM_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include>
)

# Installation rules
install(TARGETS ${MSD_SIM_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

# Install headers using the variable
install(FILES ${HEADER_FILES}
        DESTINATION include/${MSD_SIM_NAME}) 

