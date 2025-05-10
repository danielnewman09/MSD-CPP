add_subdirectory(test)
add_subdirectory(src)

set(MSD_DB_NAME msd_db)

add_library(${MSD_DB_NAME} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(SQLite3 REQUIRED)
target_link_libraries(${MSD_DB_NAME} 
  PUBLIC
    SQLite::SQLite3
  )


set_target_properties(${MSD_DB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
# Add all relevant include directories
target_include_directories(${MSD_DB_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>  
)

# Add all sources (adjust as needed)
target_sources(${MSD_DB_NAME} PRIVATE
src/Database.cpp
    )

