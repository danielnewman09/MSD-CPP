from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os

class QhullRecipe(ConanFile):
    name = "qhull"
    version = "8.0.2"
    package_type = "library"

    # Optional metadata
    license = "Qhull"
    author = "C.B. Barber and The Geometry Center"
    url = "https://github.com/qhull/qhull"
    description = "Qhull computes the convex hull, Delaunay triangulation, Voronoi diagram, and more"
    topics = ("qhull", "convex-hull", "computational-geometry", "delaunay", "voronoi")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
    }

    def configure(self):
        # Qhull 8.0.2 is not C++20 compatible (QhullSet.h uses ClassName<T>()
        # constructor syntax which is invalid in C++20). Force C++17 for compilation;
        # the resulting library links fine with C++20 consumers.
        self.settings.compiler.cppstd = "17"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = "ON"
        tc.variables["BUILD_SHARED_LIBS"] = self.options.shared
        # Disable building applications and tests
        tc.variables["BUILD_APPLICATIONS"] = "OFF"
        tc.variables["BUILD_TESTING"] = "OFF"
        tc.generate()

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/qhull/qhull.git", target=".")
        # Checkout the 2020.2 release tag
        git.checkout("v8.0.2")

        # Patch CMakeLists.txt to fix minimum CMake version
        import os
        cmake_file = os.path.join(self.source_folder, "CMakeLists.txt")
        with open(cmake_file, 'r') as f:
            content = f.read()
        # Replace outdated cmake_minimum_required with modern version
        content = content.replace('cmake_minimum_required(VERSION 3.0)',
                                  'cmake_minimum_required(VERSION 3.15)')
        with open(cmake_file, 'w') as f:
            f.write(content)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

        # Copy headers if not already copied by install
        copy(self, "*.h", src=os.path.join(self.source_folder, "src", "libqhull_r"),
             dst=os.path.join(self.package_folder, "include", "libqhull_r"), keep_path=False)
        copy(self, "*.h", src=os.path.join(self.source_folder, "src", "libqhullcpp"),
             dst=os.path.join(self.package_folder, "include", "libqhullcpp"), keep_path=False)

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "qhull")

        # Qhull uses different library names for Debug builds (adds 'd' or '_d' suffix)
        debug_suffix = "d" if self.settings.build_type == "Debug" else ""

        # qhullcpp component (C++ interface)
        self.cpp_info.components["qhullcpp"].set_property("cmake_target_name", "qhull::qhullcpp")
        self.cpp_info.components["qhullcpp"].libs = [f"qhullcpp_{debug_suffix}" if debug_suffix else "qhullcpp"]
        self.cpp_info.components["qhullcpp"].requires = ["qhullstatic_r"]

        # qhullstatic_r component (reentrant C library)
        self.cpp_info.components["qhullstatic_r"].set_property("cmake_target_name", "qhull::qhullstatic_r")
        self.cpp_info.components["qhullstatic_r"].libs = [f"qhullstatic_r{debug_suffix}"]

        # Add math library on Linux
        if self.settings.os == "Linux":
            self.cpp_info.components["qhullstatic_r"].system_libs = ["m"]

    def layout(self):
        cmake_layout(self)
