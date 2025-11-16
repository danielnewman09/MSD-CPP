
enable_testing()

find_package(GTest REQUIRED)
add_executable(${MSD_SIM_TEST_NAME})

add_subdirectory(Environment)
    
set_target_properties(${MSD_SIM_TEST_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${MSD_SIM_TEST_NAME} 
    PUBLIC
        spdlog::spdlog
    PRIVATE
      msd_sim
      GTest::GTest
      GTest::Main)

target_compile_features(${MSD_SIM_TEST_NAME} PRIVATE cxx_std_20)

target_include_directories(${MSD_SIM_TEST_NAME} PUBLIC
${GTEST_INCLUDE_DIRS}
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src> 
)