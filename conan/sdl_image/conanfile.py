from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os

class SDL3ImageRecipe(ConanFile):
    name = "sdl_image"
    version = "3.2.4"
    package_type = "library"

    # Optional metadata
    license = "Zlib"
    author = "Sam Lantinga and SDL Contributors"
    url = "https://github.com/libsdl-org/SDL_image"
    description = "SDL_image - Image loading library for SDL3"
    topics = ("sdl", "sdl3", "image", "png", "jpg", "svg")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
    }
    default_options = {
        "shared": True,
        "fPIC": True,
    }

    def requirements(self):
        self.requires("sdl/3.3.3")

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = "ON"
        tc.variables["BUILD_SHARED_LIBS"] = self.options.shared
        tc.variables["SDL3IMAGE_TESTS"] = "OFF"
        tc.variables["SDL3IMAGE_SAMPLES"] = "OFF"
        tc.generate()

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/libsdl-org/SDL_image.git", target=".")
        git.checkout(f"release-{self.version}")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        copy(self, "*.h", src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include"), keep_path=True)

    def package_info(self):
        if self.options.shared:
            self.cpp_info.libs = ["SDL3_image"]
            self.cpp_info.set_property("cmake_target_name", "SDL3_image::SDL3_image-shared")
            self.cpp_info.set_property("cmake_target_aliases", ["SDL3_image::SDL3_image"])
        else:
            self.cpp_info.libs = ["SDL3_image-static"]
            self.cpp_info.set_property("cmake_target_name", "SDL3_image::SDL3_image-static")
            self.cpp_info.set_property("cmake_target_aliases", ["SDL3_image::SDL3_image"])

        self.cpp_info.set_property("cmake_file_name", "SDL3_image")

    def layout(self):
        cmake_layout(self)
