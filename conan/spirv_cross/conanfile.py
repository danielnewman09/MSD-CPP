from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os

class SPIRVCrossRecipe(ConanFile):
    name = "spirv_cross"
    version = "main"
    package_type = "library"

    # Optional metadata
    license = "Apache-2.0"
    author = "Khronos Group"
    url = "https://github.com/KhronosGroup/SPIRV-Cross"
    description = "SPIRV-Cross is a tool for reflection and disassembly of SPIR-V"
    topics = ("spirv", "shader", "compiler", "graphics", "vulkan")

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

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = "ON"
        # SPIRV_CROSS_SHARED controls whether to build the unified C shared library
        tc.variables["SPIRV_CROSS_SHARED"] = "ON" if self.options.shared else "OFF"
        tc.variables["SPIRV_CROSS_CLI"] = "OFF"  # Don't need CLI tools
        tc.variables["SPIRV_CROSS_ENABLE_TESTS"] = "OFF"
        tc.generate()

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/KhronosGroup/SPIRV-Cross.git", target=".")
        git.checkout("main")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

        # Manually copy headers
        copy(self, "*.h", src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include"), keep_path=True)
        copy(self, "*.hpp", src=os.path.join(self.source_folder),
             dst=os.path.join(self.package_folder, "include", "spirv_cross"), keep_path=False)

        # Debug: print what cmake.install() actually installed
        import glob
        installed_files = glob.glob(os.path.join(self.package_folder, "**", "*"), recursive=True)
        self.output.info(f"Files in package folder after cmake.install(): {installed_files}")

        # Also check what's in build folder
        build_libs = glob.glob(os.path.join(self.build_folder, "**", "*.dylib"), recursive=True)
        build_libs.extend(glob.glob(os.path.join(self.build_folder, "**", "*.a"), recursive=True))
        self.output.info(f"Library files found in build folder: {build_libs}")

    def package_info(self):
        # Match what SDL_shadercross CMakeLists.txt expects
        self.cpp_info.set_property("cmake_file_name", "spirv_cross_c_shared")
        self.cpp_info.set_property("cmake_target_name", "spirv-cross-c-shared")

        # Set include directories - headers are in include/spirv_cross/
        # Match what the native SPIRV-Cross CMake config does
        self.cpp_info.includedirs = [os.path.join("include", "spirv_cross")]

        # Library configuration - the actual library name built by SPIRV-Cross
        if self.options.shared:
            # On macOS, the library is built as libspirv-cross-c-shared.dylib
            # We need to tell Conan the base name without lib prefix or extension
            self.cpp_info.libs = ["spirv-cross-c-shared"]
            # Also inform the system about the actual library file
            self.cpp_info.libdirs = ["lib"]
        else:
            self.cpp_info.libs = ["spirv-cross-core", "spirv-cross-glsl", "spirv-cross-hlsl", "spirv-cross-msl", "spirv-cross-c"]
            self.cpp_info.libdirs = ["lib"]

    def layout(self):
        cmake_layout(self)