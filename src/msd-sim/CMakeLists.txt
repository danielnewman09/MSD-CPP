add_subdirectory(Agent)
add_subdirectory(Algorithms)
add_subdirectory(Environment)
# add_subdirectory(Logging)

set(MSD_SIM_NAME msd_sim)
set(MSD_TEST_NAME msd_sim_test)

include_directories(msd_sim ${MSD_SIM_DIR}../)
include_directories(msd_sim_test ${MSD_SIM_DIR}../)



add_library(${MSD_SIM_NAME})
set_target_properties(${MSD_SIM_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Add all relevant include directories
target_include_directories(${MSD_SIM_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Environment/src>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Agent/src>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Algorithms/src>  
)

# Add all sources (adjust as needed)
target_sources(${MSD_SIM_NAME} PRIVATE
    Engine.cpp
    Engine.hpp
    Agent/src/BaseAgent.cpp
    Environment/src/Platform.cpp
    Environment/src/PlatformState.cpp
    Environment/src/WorldModel.cpp
    Agent/src/BaseAgent.hpp
    Environment/src/Platform.hpp
    Environment/src/PlatformState.hpp
    Environment/src/WorldModel.hpp
    )


enable_testing()

find_package(GTest REQUIRED)
add_executable(msd_sim_test 
    Environment/test/EnvironmentTest.cpp)
set_target_properties(msd_sim_test PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(msd_sim_test 
    PRIVATE
        msd_sim
    PUBLIC
        GTest::GTest
        GTest::Main)
target_compile_features(msd_sim_test PRIVATE cxx_std_17)

target_include_directories(msd_sim_test PUBLIC
${GTEST_INCLUDE_DIRS}
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Environment>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Environment/test>  
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Agent>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Algorithms> 
)
