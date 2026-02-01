from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os


class EcosRecipe(ConanFile):
    name = "ecos"
    version = "2.0.10"
    package_type = "library"

    # Metadata
    license = "GPLv3"
    author = "Alexander Domahidi, embotech AG"
    url = "https://github.com/embotech/ecos"
    description = "Embedded Conic Solver (ECOS) - an interior-point solver for second-order cone programs"
    topics = ("ecos", "socp", "optimization", "interior-point", "conic-solver")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "use_long": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "use_long": True,
    }

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/embotech/ecos.git", target=".")
        git.checkout("v2.0.10")

        # Patch CMakeLists.txt:
        # 1. Replace hardcoded SHARED with configurable BUILD_SHARED_LIBS
        # 2. Update cmake_minimum_required to suppress policy warnings
        cmake_file = os.path.join(self.source_folder, "CMakeLists.txt")
        with open(cmake_file, 'r') as f:
            content = f.read()

        content = content.replace(
            'cmake_minimum_required(VERSION 3.5)',
            'cmake_minimum_required(VERSION 3.15)')
        content = content.replace(
            'add_library(ecos SHARED ${ecos_headers} ${ecos_sources})',
            'add_library(ecos ${ecos_headers} ${ecos_sources})')

        with open(cmake_file, 'w') as f:
            f.write(content)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = "ON"
        tc.variables["BUILD_SHARED_LIBS"] = self.options.shared
        tc.variables["USE_LONG"] = self.options.use_long
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

        # Ensure headers from external/ are also available
        copy(self, "*.h",
             src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include", "ecos"),
             keep_path=False)
        copy(self, "SuiteSparse_config.h",
             src=os.path.join(self.source_folder, "external", "SuiteSparse_config"),
             dst=os.path.join(self.package_folder, "include", "ecos"),
             keep_path=False)
        copy(self, "*.h",
             src=os.path.join(self.source_folder, "external", "amd", "include"),
             dst=os.path.join(self.package_folder, "include", "ecos"),
             keep_path=False)
        copy(self, "*.h",
             src=os.path.join(self.source_folder, "external", "ldl", "include"),
             dst=os.path.join(self.package_folder, "include", "ecos"),
             keep_path=False)

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "ecos")
        self.cpp_info.set_property("cmake_target_name", "ecos::ecos")
        self.cpp_info.libs = ["ecos"]
        self.cpp_info.includedirs = ["include"]

        # Propagate compile definitions
        self.cpp_info.defines = ["CTRLC=1"]
        if self.options.use_long:
            self.cpp_info.defines.extend(["LDL_LONG", "DLONG"])

        # Link math library on non-Windows
        if self.settings.os != "Windows":
            self.cpp_info.system_libs = ["m"]

    def layout(self):
        cmake_layout(self)
