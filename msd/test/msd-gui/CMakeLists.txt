add_executable(test_msd_gui
    # List your test .cpp files here
    test_gui.cpp
)

target_link_libraries(test_msd_gui
    gtest
    gtest_main
    msd-gui
)

add_test(NAME TestMSDGui COMMAND test_msd_gui)
