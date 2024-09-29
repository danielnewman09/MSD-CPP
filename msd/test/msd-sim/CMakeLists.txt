add_executable(test_msd_sim
    # List your test .cpp files here
    test_simulation.cpp
)

target_link_libraries(test_msd_sim
    gtest
    gtest_main
    msd-sim
)

add_test(NAME TestMsdSim COMMAND test_msd_sim)
