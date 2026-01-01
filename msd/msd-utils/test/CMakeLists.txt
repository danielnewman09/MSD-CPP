add_executable(${MSD_UTILS_TEST_NAME}
  test_PathUtils.cpp
)

target_link_libraries(${MSD_UTILS_TEST_NAME}
  PRIVATE
    msd_utils
)


# Add include directories to support "msd-sim/src/..." includes
target_include_directories(${MSD_UTILS_TEST_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
        $<INSTALL_INTERFACE:include>
)
