from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os


class NloptRecipe(ConanFile):
    name = "nlopt"
    version = "2.10.0"
    package_type = "library"

    # Metadata
    license = "MIT"
    author = "Steven G. Johnson"
    url = "https://github.com/stevengj/nlopt"
    description = "NLopt - nonlinear optimization library"
    topics = ("nlopt", "optimization", "nonlinear", "constrained", "slsqp")

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

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/stevengj/nlopt.git", target=".")
        git.checkout("v2.10.0")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = "ON"
        tc.variables["BUILD_SHARED_LIBS"] = self.options.shared
        # Disable language bindings we don't need
        tc.variables["NLOPT_PYTHON"] = "OFF"
        tc.variables["NLOPT_OCTAVE"] = "OFF"
        tc.variables["NLOPT_MATLAB"] = "OFF"
        tc.variables["NLOPT_GUILE"] = "OFF"
        tc.variables["NLOPT_SWIG"] = "OFF"
        tc.variables["NLOPT_TESTS"] = "OFF"
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

        # Copy headers if not already installed
        copy(self, "*.h",
             src=os.path.join(self.source_folder, "src", "api"),
             dst=os.path.join(self.package_folder, "include"),
             keep_path=False)
        copy(self, "*.hpp",
             src=os.path.join(self.source_folder, "src", "api"),
             dst=os.path.join(self.package_folder, "include"),
             keep_path=False)

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "NLopt")
        self.cpp_info.set_property("cmake_target_name", "NLopt::nlopt")
        self.cpp_info.libs = ["nlopt"]
        self.cpp_info.includedirs = ["include"]

        if self.settings.os != "Windows":
            self.cpp_info.system_libs = ["m"]

    def layout(self):
        cmake_layout(self)
