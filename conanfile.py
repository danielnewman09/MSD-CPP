from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git

class fooRecipe(ConanFile):
    name = "msd-exe"
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

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "../../CMakeLists.txt", "../../src/*"

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()


    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def requirements(self):
        self.requires("glew/2.2.0")
        self.requires("gtest/1.15.0")
        self.requires("opengl/system")
        self.requires("glfw/3.4")
        self.requires("glm/cci.20230113")
        self.requires("sfml/2.6.1")
        self.requires("eigen/3.4.0")
        self.requires('sqlite3/3.47.0')

    def build_requirements(self):
        self.tool_requires("cmake/3.22.6")

    def layout(self):
        cmake_layout(self)

    def package_info(self):
        self.cpp_info.libs = ["msd-sim", "msd-exe"]  # Adjust based on actual library names
