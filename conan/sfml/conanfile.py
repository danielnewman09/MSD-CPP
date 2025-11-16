from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os

class SFMLRecipe(ConanFile):
    name = "sfml"
    version = "3.0.1"
    package_type = "library"

    # Optional metadata
    license = "zlib/png"
    author = "SFML Team"
    url = "https://github.com/SFML/SFML"
    description = "Simple and Fast Multimedia Library"
    topics = ("sfml", "multimedia", "graphics", "audio")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "window": [True, False],
        "graphics": [True, False],
        "audio": [True, False],
        "network": [True, False]
    }
    default_options = {
        "shared": True,
        "fPIC": True,
        "window": True,
        "graphics": True,
        "audio": True,
        "network": True
    }

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = "ON"
        tc.variables["BUILD_SHARED_LIBS"] = self.options.shared
        tc.variables["SFML_BUILD_EXAMPLES"] = "OFF"
        tc.variables["SFML_BUILD_TEST_SUITE"] = "OFF"
        tc.variables["SFML_ENABLE_STDLIB_ASSERTIONS"] = "OFF"
        tc.variables["SFML_WARNINGS_AS_ERRORS"] = "OFF"
        tc.variables['CMAKE_CXX_EXTENSIONS'] = "OFF"
        tc.generate()

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/SFML/SFML.git", target=".")
        git.checkout("3.0.1")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        copy(self, "*.hpp", src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include"), keep_path=True)

    def package_info(self):
        suffix = "" if self.options.shared else "-s"

        # Define components in dependency order
        self.cpp_info.components["system"].libs = [f"sfml-system{suffix}"]
        self.cpp_info.components["system"].set_property("cmake_target_name", "SFML::System")

        self.cpp_info.components["window"].libs = [f"sfml-window{suffix}"]
        self.cpp_info.components["window"].requires = ["system"]
        self.cpp_info.components["window"].set_property("cmake_target_name", "SFML::Window")

        self.cpp_info.components["graphics"].libs = [f"sfml-graphics{suffix}"]
        self.cpp_info.components["graphics"].requires = ["window", "system"]
        self.cpp_info.components["graphics"].set_property("cmake_target_name", "SFML::Graphics")

        self.cpp_info.components["audio"].libs = [f"sfml-audio{suffix}"]
        self.cpp_info.components["audio"].requires = ["system"]
        self.cpp_info.components["audio"].set_property("cmake_target_name", "SFML::Audio")

        self.cpp_info.components["network"].libs = [f"sfml-network{suffix}"]
        self.cpp_info.components["network"].requires = ["system"]
        self.cpp_info.components["network"].set_property("cmake_target_name", "SFML::Network")

        # Add system-specific frameworks and libraries
        if self.settings.os == "Macos":
            self.cpp_info.components["system"].frameworks = ["Foundation"]
            self.cpp_info.components["window"].frameworks = ["AppKit", "IOKit", "Carbon"]
            self.cpp_info.components["graphics"].frameworks = ["CoreGraphics"]
        elif self.settings.os == "Linux":
            self.cpp_info.components["window"].system_libs = ["X11", "Xrandr", "Xcursor", "Xi"]
            self.cpp_info.components["system"].system_libs = ["pthread"]

    def layout(self):
        cmake_layout(self)
