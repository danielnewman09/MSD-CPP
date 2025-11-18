from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os

class SDLRecipe(ConanFile):
    name = "sdl"
    version = "3.3.3"
    package_type = "library"

    # Optional metadata
    license = "Zlib"
    author = "Sam Lantinga and SDL Contributors"
    url = "https://github.com/libsdl-org/SDL"
    description = "Simple DirectMedia Layer - Cross-platform multimedia library"
    topics = ("sdl", "multimedia", "graphics", "audio", "input")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "vulkan": [True, False],
        "opengl": [True, False],
        "opengles": [True, False],
        "wayland": [True, False],
        "x11": [True, False]
    }
    default_options = {
        "shared": True,
        "fPIC": True,
        "vulkan": True,
        "opengl": True,
        "opengles": False,
        "wayland": False,
        "x11": True
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
        tc.variables["SDL_TEST"] = "OFF"
        tc.variables["SDL_TESTS"] = "OFF"
        tc.variables["SDL_DISABLE_INSTALL_DOCS"] = "ON"
        tc.variables["SDL_VULKAN"] = self.options.vulkan
        tc.variables["SDL_OPENGL"] = self.options.opengl
        tc.variables["SDL_OPENGLES"] = self.options.opengles
        if self.settings.os == "Linux":
            tc.variables["SDL_WAYLAND"] = self.options.wayland
            tc.variables["SDL_X11"] = self.options.x11
        tc.generate()

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/libsdl-org/SDL.git", target=".")
        git.checkout(f"main")

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
        self.cpp_info.set_property("cmake_file_name", "SDL3")

        # Create Headers component (header-only interface)
        self.cpp_info.components["headers"].set_property("cmake_target_name", "SDL3::Headers")
        self.cpp_info.components["headers"].bindirs = []
        self.cpp_info.components["headers"].libdirs = []

        # Create main SDL3 component
        if self.options.shared:
            self.cpp_info.components["sdl3"].libs = ["SDL3"]
            self.cpp_info.components["sdl3"].set_property("cmake_target_name", "SDL3::SDL3-shared")
            self.cpp_info.components["sdl3"].set_property("cmake_target_aliases", ["SDL3::SDL3"])
        else:
            self.cpp_info.components["sdl3"].libs = ["SDL3-static"]
            self.cpp_info.components["sdl3"].set_property("cmake_target_name", "SDL3::SDL3-static")
            self.cpp_info.components["sdl3"].set_property("cmake_target_aliases", ["SDL3::SDL3"])

        self.cpp_info.components["sdl3"].requires = ["headers"]

        # Add system-specific frameworks and libraries to the main component
        if self.settings.os == "Macos":
            self.cpp_info.components["sdl3"].frameworks = [
                "Cocoa", "IOKit", "CoreVideo", "CoreAudio",
                "AudioToolbox", "ForceFeedback", "Metal", "GameController"
            ]
        elif self.settings.os == "Linux":
            self.cpp_info.components["sdl3"].system_libs = ["pthread", "dl", "m"]
            if self.options.x11:
                self.cpp_info.components["sdl3"].system_libs.extend(["X11", "Xext", "Xcursor", "Xinerama", "Xi", "Xrandr", "Xss", "Xxf86vm"])
        elif self.settings.os == "Windows":
            self.cpp_info.components["sdl3"].system_libs = [
                "user32", "gdi32", "winmm", "imm32", "ole32",
                "oleaut32", "version", "uuid", "advapi32", "setupapi", "shell32"
            ]

    def layout(self):
        cmake_layout(self)
