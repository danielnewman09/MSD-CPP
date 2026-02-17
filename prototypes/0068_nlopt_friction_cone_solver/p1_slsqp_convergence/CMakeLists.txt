cmake_minimum_required(VERSION 3.16)
project(prototype_p1_slsqp_convergence)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find Conan-installed packages
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlopt REQUIRED)

# Find Eigen include directory from Conan package
get_target_property(EIGEN_INCLUDE Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

add_executable(p1_convergence
    main.cpp
)

target_include_directories(p1_convergence
    PRIVATE
        ${EIGEN_INCLUDE}
)

target_link_libraries(p1_convergence
    PRIVATE
        Eigen3::Eigen
        NLopt::nlopt
)
