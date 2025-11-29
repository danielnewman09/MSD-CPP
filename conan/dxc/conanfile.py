from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os

class DXCRecipe(ConanFile):
    name = "dxc"
    version = "main"
    package_type = "library"

    # Optional metadata
    license = "LLVM"
    author = "Microsoft"
    url = "https://github.com/microsoft/DirectXShaderCompiler"
    description = "DirectX Shader Compiler"
    topics = ("directx", "shader", "compiler", "graphics", "hlsl")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
    }
    default_options = {
        "shared": False,  # DXC must be built as static libraries due to circular dependencies
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

        # DXC build options
        tc.variables["LLVM_ENABLE_PROJECTS"] = "clang"
        tc.variables["LLVM_TARGETS_TO_BUILD"] = "None"
        tc.variables["CLANG_ENABLE_ARCMT"] = "OFF"
        tc.variables["CLANG_ENABLE_STATIC_ANALYZER"] = "OFF"
        tc.variables["LLVM_ENABLE_TERMINFO"] = "OFF"
        tc.variables["LLVM_INCLUDE_TESTS"] = "OFF"
        tc.variables["CLANG_INCLUDE_TESTS"] = "OFF"
        tc.variables["LLVM_INCLUDE_DOCS"] = "OFF"
        tc.variables["LLVM_INCLUDE_EXAMPLES"] = "OFF"
        tc.variables["LLVM_OPTIMIZED_TABLEGEN"] = "ON"

        # Enable C++ exceptions (required by DXC)
        tc.variables["LLVM_ENABLE_EH"] = "ON"
        tc.variables["LLVM_ENABLE_RTTI"] = "ON"

        # Force static libraries to avoid circular dependency issues
        tc.variables["BUILD_SHARED_LIBS"] = "OFF"
        tc.variables["LIBCLANG_BUILD_STATIC"] = "ON"

        # Build type settings
        if self.settings.build_type == "Release":
            tc.variables["LLVM_ENABLE_ASSERTIONS"] = "OFF"

        tc.generate()

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/microsoft/DirectXShaderCompiler.git", target=".")
        git.checkout("main")
        # DXC requires LLVM and other submodules
        git.run("submodule update --init --recursive")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        # Don't use cmake.install() as it tries to install disabled components
        # Instead, manually copy what we need

        build_bin = os.path.join(self.build_folder, "build", str(self.settings.build_type), "bin")
        build_lib = os.path.join(self.build_folder, "build", str(self.settings.build_type), "lib")

        # Copy the DXC executable to Conan package
        copy(self, "dxc*",
             src=build_bin,
             dst=os.path.join(self.package_folder, "bin"),
             keep_path=False)

        # Copy static libraries (we're building static)
        copy(self, "*.a",
             src=build_lib,
             dst=os.path.join(self.package_folder, "lib"),
             keep_path=False)

        # Copy headers if needed
        copy(self, "*.h",
             src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include"),
             keep_path=True)

        # Get install path from environment variable
        local_install = os.getenv("DXC_INSTALL_DIR")

        if local_install:
            print(f"Installing DXC to: {local_install}")

            # Copy the binary
            copy(self, "dxc*",
                src=os.path.join(self.package_folder, "bin"),
                dst=os.path.join(local_install, "bin"))

            print(f"DXC binary installed to {os.path.join(local_install, 'bin')}")
        else:
            print("DXC_INSTALL_DIR not set, skipping local install")

    def package_info(self):
        self.cpp_info.bindirs = ["bin"]
        self.cpp_info.libdirs = ["lib"]
        self.cpp_info.includedirs = ["include"]

        # Make dxc available in PATH
        self.buildenv_info.prepend_path("PATH", os.path.join(self.package_folder, "bin"))
        self.runenv_info.prepend_path("PATH", os.path.join(self.package_folder, "bin"))

    def layout(self):
        cmake_layout(self)
