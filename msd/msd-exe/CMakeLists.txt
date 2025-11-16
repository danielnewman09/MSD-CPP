
set(MSD_EXE_NAME msd_exe)

add_executable(${MSD_EXE_NAME})
set_target_properties(${MSD_EXE_NAME} PROPERTIES LINKER_LANGUAGE CXX)

find_package(spdlog REQUIRED)

target_link_libraries(${MSD_EXE_NAME} 
    PRIVATE 
      msd_sim
      spdlog::spdlog)

# Add all relevant include directories
target_include_directories(${MSD_EXE_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>  
)

# Add all sources (adjust as needed)
target_sources(${MSD_EXE_NAME} PRIVATE
src/main.cpp
    )

