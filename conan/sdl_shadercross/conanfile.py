from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os

from pathlib import Path

# Get the directory containing the current file
current_file_directory = str(Path(__file__).parent.absolute())
print(f"Directory of the current file: {current_file_directory}")

class SDLShadercrossRecipe(ConanFile):
    name = "sdl_shadercross"
    version = "main"
    package_type = "application"

    # Optional metadata
    license = "Zlib"
    author = "SDL Contributors"
    url = "https://github.com/libsdl-org/SDL_shadercross"
    description = "SDL shader cross-compiler for HLSL, SPIR-V, MSL, and DXIL"
    topics = ("sdl", "shader", "compiler", "graphics")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.requires("sdl/3.3.3")
        self.requires("spirv_cross/main")
        # DXC is used as an external binary tool, not linked as a library
        # self.requires("dxc/main")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = "ON"

        # DXC/DXBC require DirectX headers — disable on Linux
        if self.settings.os == "Linux":
            tc.variables["SDLSHADERCROSS_DXC"] = "OFF"
            tc.variables["SDLSHADERCROSS_DXBC"] = "OFF"
            tc.variables["SDLSHADERCROSS_VENDORED"] = "OFF"
            # Workaround: suppress __stdcall attribute warnings on aarch64 GCC
            tc.extra_cflags = ["-Wno-attributes"]
        else:
            # macOS/Windows: enable DXC with vendored mode (builds DXC from source)
            tc.variables["SDLSHADERCROSS_DXC"] = "ON"
            tc.variables["SDLSHADERCROSS_VENDORED"] = "ON"

        # Enable installation
        tc.variables["SDLSHADERCROSS_INSTALL"] = "ON"

        # Set install prefix to ../../installs/sdl_shadercross
        install_prefix = os.path.abspath(os.path.join(self.recipe_folder, "..", "..", "installs", "sdl_shadercross"))
        tc.variables["CMAKE_INSTALL_PREFIX"] = install_prefix
        tc.generate()

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/libsdl-org/SDL_shadercross.git", target=".")
        git.checkout("main")
        # Initialize submodules for vendored dependencies (SPIRV-Cross, DirectXShaderCompiler)
        git.run("submodule update --init --recursive")
        
    def build(self):
        if self.settings.os == "Linux":
            self._patch_elf_note_semicolon()
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def _patch_elf_note_semicolon(self):
        """Patch SDL_shadercross.c to add missing semicolons after SDL_ELF_NOTE_DLOPEN.

        SDL 3.3.3's SDL_ELF_NOTE_DLOPEN macro expands to a struct definition
        without a trailing semicolon, causing parse errors with GCC on Linux.
        """
        import re
        src_file = os.path.join(self.source_folder, "src", "SDL_shadercross.c")
        with open(src_file, "r") as f:
            content = f.read()
        # Add semicolon after SDL_ELF_NOTE_DLOPEN(...) invocations that lack one
        content = re.sub(
            r'(SDL_ELF_NOTE_DLOPEN\([^)]*\))\s*\n#endif',
            r'\1;\n#endif',
            content
        )
        with open(src_file, "w") as f:
            f.write(content)
    
    def package(self):
        cmake = CMake(self)
        cmake.install()

        # Copy the shadercross binary to Conan package
        copy(self, "shadercross*",
             src=os.path.join(self.build_folder, "build", str(self.settings.build_type)),
             dst=os.path.join(self.package_folder, "bin"),
             keep_path=False)

        # Get install path from environment variable
        local_install = os.getenv("SHADERCROSS_INSTALL_DIR")

        if local_install:
            print(f"Installing shadercross to: {local_install}")

            # Copy the binary
            copy(self, "shadercross*",
                src=os.path.join(self.package_folder, "bin"),
                dst=os.path.join(local_install, "bin"))

            # Copy required dynamic libraries from dependencies
            lib_dir = os.path.join(local_install, "lib")
            os.makedirs(lib_dir, exist_ok=True)

            # Copy all libraries from package lib directory
            copy(self, "*",
                    src=os.path.join(self.package_folder, "lib"),
                    dst=os.path.join(local_install, "lib"))

            # Copy dependency shared libraries (platform-appropriate extensions)
            lib_glob = "*.dylib*" if self.settings.os == "Macos" else "*.so*"
            for dep in self.dependencies.values():
                dep_cpp_info = dep.cpp_info.aggregated_components()
                for libdir in dep_cpp_info.libdirs:
                    copy(self, lib_glob,
                         src=libdir,
                         dst=lib_dir,
                         keep_path=False)

            # Fix the rpath in the shadercross binary to look in ../lib
            import subprocess
            shadercross_path = os.path.join(local_install, "bin", "shadercross")
            if self.settings.os == "Macos":
                subprocess.run(["install_name_tool", "-add_rpath", "@executable_path/../lib", shadercross_path])
            elif self.settings.os == "Linux":
                subprocess.run(["patchelf", "--set-rpath", "$ORIGIN/../lib", shadercross_path])
            print(f"Fixed rpath for {shadercross_path}")
        else:
            print("SHADERCROSS_INSTALL_DIR not set, skipping local install")

    def package_info(self):
        self.cpp_info.bindirs = ["bin"]
        # Make shadercross available in PATH
        self.buildenv_info.prepend_path("PATH", os.path.join(self.package_folder, "bin"))
        self.runenv_info.prepend_path("PATH", os.path.join(self.package_folder, "bin"))
    
    def layout(self):
        cmake_layout(self)
