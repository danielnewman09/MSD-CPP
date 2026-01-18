set(MSD_GUI_TEST_NAME msd_gui_test)

find_package(GTest REQUIRED)

add_executable(${MSD_GUI_TEST_NAME}
    ShaderTransformTest.cpp
    unit/shader_policy_test.cpp
    unit/gpu_instance_manager_test.cpp
)

target_link_libraries(${MSD_GUI_TEST_NAME}
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        msd_gui
)

target_include_directories(${MSD_GUI_TEST_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

include(GoogleTest)
gtest_discover_tests(${MSD_GUI_TEST_NAME})
