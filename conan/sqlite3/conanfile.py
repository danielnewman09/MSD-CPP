from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.files import download, unzip, copy, save
import os


class SQLite3Recipe(ConanFile):
    name = "sqlite3"
    version = "3.47.0-local"
    package_type = "library"

    # Metadata
    license = "Public Domain"
    author = "SQLite Developers"
    url = "https://www.sqlite.org"
    description = "SQLite is a C-language library that implements a small, fast, self-contained SQL database engine"
    topics = ("sqlite", "database", "sql", "embedded")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "threadsafe": [0, 1, 2],
        "enable_fts5": [True, False],
        "enable_json1": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "threadsafe": 1,
        "enable_fts5": True,
        "enable_json1": True,
    }

    exports_sources = "CMakeLists.txt"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")
        # SQLite is C, not C++
        self.settings.rm_safe("compiler.libcxx")
        self.settings.rm_safe("compiler.cppstd")

    def source(self):
        # Download SQLite amalgamation
        year = "2024"
        version_str = "3470000"  # 3.47.0 -> 3470000
        url = f"https://www.sqlite.org/{year}/sqlite-amalgamation-{version_str}.zip"
        download(self, url, "sqlite.zip")
        unzip(self, "sqlite.zip")
        # Move files to source root
        copy(self, "*", src=f"sqlite-amalgamation-{version_str}", dst=".")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()

        # Create CMakeLists.txt for building sqlite3
        threadsafe_val = int(self.options.threadsafe)
        enable_fts5 = "1" if self.options.enable_fts5 else "0"
        enable_json1 = "1" if self.options.enable_json1 else "0"

        cmakelists = f"""
cmake_minimum_required(VERSION 3.15)
project(sqlite3 C)

add_library(sqlite3 sqlite3.c)

target_include_directories(sqlite3 PUBLIC
    $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(sqlite3 PRIVATE
    SQLITE_THREADSAFE={threadsafe_val}
    SQLITE_ENABLE_FTS5={enable_fts5}
    SQLITE_ENABLE_JSON1={enable_json1}
)

if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
    target_link_libraries(sqlite3 PUBLIC pthread dl m)
endif()

install(TARGETS sqlite3
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES sqlite3.h sqlite3ext.h DESTINATION include)
"""
        save(self, os.path.join(self.source_folder, "CMakeLists.txt"), cmakelists)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def layout(self):
        cmake_layout(self)

    def package_info(self):
        self.cpp_info.libs = ["sqlite3"]
        # Explicitly set includedirs - required because configure() removes compiler settings
        # which affects how cmake_layout() computes the package layout
        self.cpp_info.includedirs = ["include"]
        self.cpp_info.libdirs = ["lib"]
        self.cpp_info.set_property("cmake_file_name", "SQLite3")
        self.cpp_info.set_property("cmake_target_name", "SQLite3::SQLite3")

        if self.settings.os in ["Linux", "FreeBSD"]:
            self.cpp_info.system_libs = ["pthread", "dl", "m"]