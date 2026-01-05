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

    def configure(self):
        # Set C++20 standard for all dependencies and this package
        self.settings.compiler.cppstd = "20"

        # Disable problematic Boost components for Emscripten
        if self.settings.os == "Emscripten":
            # Disable stacktrace components that don't work with Emscripten
            self.options["boost/*"].without_stacktrace = True
            # Disable other components that may have issues with Emscripten
            self.options["boost/*"].without_fiber = True
            self.options["boost/*"].without_context = True
            self.options["boost/*"].without_coroutine = True
            # Disable iostreams to avoid bzip2 build issues with Emscripten
            self.options["boost/*"].without_iostreams = True

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "../../CMakeLists.txt", "../../src/*", "../../test/*", "../../*.cmake"

    def generate(self):
        import os
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)

        # Customize preset name for Emscripten to avoid conflicts with native builds
        if self.settings.os == "Emscripten":
            tc.presets_prefix = "conan-emscripten"
            # Workaround: When using Emscripten toolchain, Conan doesn't generate
            # conan_toolchain.cmake, so CMAKE_PREFIX_PATH isn't set automatically.
            # We need to explicitly set it so CMake can find dependency configs.
            generators_folder = os.path.join(
                self.recipe_folder, "build", "Emscripten", "build",
                str(self.settings.build_type), "generators"
            )
            tc.cache_variables["CMAKE_PREFIX_PATH"] = generators_folder
            # Also set individual package directories for robustness
            tc.cache_variables["cpp_sqlite_DIR"] = generators_folder
            tc.cache_variables["spdlog_DIR"] = generators_folder
            tc.cache_variables["fmt_DIR"] = generators_folder
            tc.cache_variables["Boost_DIR"] = generators_folder
            tc.cache_variables["Eigen3_DIR"] = generators_folder
            tc.cache_variables["qhull_DIR"] = generators_folder
            tc.cache_variables["SQLite3_DIR"] = generators_folder
            # Workaround for fmt/spdlog consteval issue with Emscripten 3.1.68+
            # See: https://github.com/emscripten-core/emscripten/issues/22795
            tc.cache_variables["CMAKE_CXX_FLAGS"] = "-DFMT_USE_CONSTEVAL=0 -DFMT_CONSTEVAL="

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
        # Skip gtest for Emscripten - tests don't run in browser
        if self.settings.os != "Emscripten":
            self.requires("gtest/1.15.0")

        self.requires("eigen/3.4.0")
        self.requires('spdlog/1.14.1')
        self.requires("qhull/8.0.2")

        # Build cpp_sqlite with same build_type as project for debugging
        self.requires("cpp_sqlite/0.1.0")

        self.requires("boost/1.86.0")

        # Platform-conditional SDL dependencies
        # For Emscripten, SDL3 comes from Emscripten ports (USE_SDL=3)
        if self.settings.os != "Emscripten":
            self.requires('sdl/3.3.3')
            self.requires("sdl_image/3.2.4")
            self.requires("sdl_mixer/3.1.0")  # Skip for web (minimal scope)
            self.requires("sdl_ttf/3.2.2")    # Skip for web (minimal scope)

    def build_requirements(self):
        self.tool_requires("cmake/3.22.6")

    def layout(self):
        cmake_layout(self)
        self.folders.base_install = "installs"

    def package_info(self):
        self.cpp_info.libs = ["msd-sim", "msd-exe"]  # Adjust based on actual library names
