# Add subdirectories to collect sources from each module
add_subdirectory(src)
if(BUILD_TESTING)
  add_subdirectory(test)
endif() 

