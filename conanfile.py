from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git

class msd(ConanFile):
    name = "msd"
    version = "1.0"
    package_type = "application"

    # Optional metadata
    license = "<Put the package license here>"
    author = "<Put your name here> <And your email here>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of foo package here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    # Options
    options = {
        "enable_coverage": [True, False]
    }
    default_options = {
        "enable_coverage": False
    }

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "../../CMakeLists.txt", "../../src/*", "../../test/*", "../../*.cmake"

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_CXX_STANDARD"] = "20"
        tc.variables["CMAKE_CXX_STANDARD_REQUIRED"] = "ON"

        # Pass coverage option to CMake
        tc.variables["ENABLE_COVERAGE"] = self.options.enable_coverage

        build_type = str(self.settings.build_type).lower()
        tc.variables["CMAKE_RUNTIME_OUTPUT_DIRECTORY"] = \
            f"${{CMAKE_BINARY_DIR}}/{build_type}"
        tc.variables["CMAKE_LIBRARY_OUTPUT_DIRECTORY"] = \
            f"${{CMAKE_BINARY_DIR}}/{build_type}"
        tc.variables["CMAKE_ARCHIVE_OUTPUT_DIRECTORY"] = \
            f"${{CMAKE_BINARY_DIR}}/{build_type}"

        # Set install prefix to repository root's installs directory
        import os
        install_prefix = os.path.abspath(os.path.join(self.recipe_folder, "..", "..", "installs"))
        tc.variables["CMAKE_INSTALL_PREFIX"] = install_prefix

        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def requirements(self):
        self.requires("gtest/1.15.0")
        self.requires("eigen/3.4.0")
        self.requires('sqlite3/3.47.0')
        self.requires('spdlog/1.12.0')
        self.requires('sdl/3.3.3')
        self.requires("sdl_image/3.2.4")
        self.requires("sdl_mixer/3.1.0")
        self.requires("sdl_ttf/3.2.2")
        self.requires("qhull/8.0.2")

    def build_requirements(self):
        self.tool_requires("cmake/3.22.6")

    def layout(self):
        cmake_layout(self)
        self.folders.base_install = "installs"

    def package_info(self):
        self.cpp_info.libs = ["msd-sim", "msd-exe"]  # Adjust based on actual library names
