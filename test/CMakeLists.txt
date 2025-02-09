enable_testing()

# Add tests for project-sim
add_executable(test-msd-sim msd-sim/test-msd-sim.cpp)
target_link_libraries(test-msd-sim PRIVATE msd-sim)

add_test(NAME TestMSDSim COMMAND test-msd-sim)
