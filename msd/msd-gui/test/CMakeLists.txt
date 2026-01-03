set(MSD_GUI_TEST_NAME msd_gui_test)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${MSD_GUI_TEST_NAME}
    ShaderTransformTest.cpp
    unit/shader_policy_test.cpp
)

target_link_libraries(${MSD_GUI_TEST_NAME}
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
        msd_sim
        msd_gui
        msd_assets
)

target_include_directories(${MSD_GUI_TEST_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

include(GoogleTest)
gtest_discover_tests(${MSD_GUI_TEST_NAME})
