from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git
from conan.tools.files import copy
import os

class SDL3MixerRecipe(ConanFile):
    name = "sdl_mixer"
    version = "3.1.0"
    package_type = "library"

    # Optional metadata
    license = "Zlib"
    author = "Sam Lantinga and SDL Contributors"
    url = "https://github.com/libsdl-org/SDL_mixer"
    description = "SDL_mixer - Audio mixing library for SDL3"
    topics = ("sdl", "sdl3", "audio", "mixer", "music")

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
        tc.variables["SDL3MIXER_TESTS"] = "OFF"
        tc.variables["SDL3MIXER_SAMPLES"] = "OFF"
        tc.variables["SDL3MIXER_INSTALL"] = "ON"
        tc.variables["SDL3MIXER_DEPS_SHARED"] = "OFF"
        tc.variables["SDL3MIXER_VENDORED"] = "ON"

        # Explicitly disable ALL external codec dependencies
        tc.variables["SDL3MIXER_FLAC"] = "OFF"
        tc.variables["SDL3MIXER_FLAC_LIBFLAC"] = "OFF"
        tc.variables["SDL3MIXER_FLAC_DRFLAC"] = "OFF"
        tc.variables["SDL3MIXER_MOD"] = "OFF"
        tc.variables["SDL3MIXER_MOD_MODPLUG"] = "OFF"
        tc.variables["SDL3MIXER_MOD_XMP"] = "OFF"
        tc.variables["SDL3MIXER_MOD_XMP_LITE"] = "OFF"
        tc.variables["SDL3MIXER_MP3"] = "OFF"
        tc.variables["SDL3MIXER_MP3_DRMP3"] = "OFF"
        tc.variables["SDL3MIXER_MP3_MPG123"] = "OFF"
        tc.variables["SDL3MIXER_MIDI"] = "OFF"
        tc.variables["SDL3MIXER_MIDI_FLUIDSYNTH"] = "OFF"
        tc.variables["SDL3MIXER_MIDI_NATIVE"] = "OFF"
        tc.variables["SDL3MIXER_MIDI_TIMIDITY"] = "OFF"
        tc.variables["SDL3MIXER_OPUS"] = "OFF"
        tc.variables["SDL3MIXER_VORBIS"] = "OFF"
        tc.variables["SDL3MIXER_VORBIS_STB"] = "OFF"
        tc.variables["SDL3MIXER_VORBIS_TREMOR"] = "OFF"
        tc.variables["SDL3MIXER_VORBIS_VORBISFILE"] = "OFF"
        tc.variables["SDL3MIXER_WAVPACK"] = "OFF"

        # Enable only WAV support (built-in, no external dependencies)
        tc.variables["SDL3MIXER_WAV"] = "ON"

        tc.generate()

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/libsdl-org/SDL_mixer.git", target=".")
        git.checkout(f"main")

    def build(self):
        cmake = CMake(self)
        # Pass additional defines at configure time to ensure codecs are disabled
        cmake.configure(cli_args=[
            "-DCMAKE_DISABLE_FIND_PACKAGE_FLAC=ON",
            "-DCMAKE_DISABLE_FIND_PACKAGE_libxmp=ON",
            "-DCMAKE_DISABLE_FIND_PACKAGE_modplug=ON",
            "-DCMAKE_DISABLE_FIND_PACKAGE_MPG123=ON",
            "-DCMAKE_DISABLE_FIND_PACKAGE_Opus=ON",
            "-DCMAKE_DISABLE_FIND_PACKAGE_Vorbis=ON",
            "-DCMAKE_DISABLE_FIND_PACKAGE_vorbisfile=ON",
            "-DCMAKE_DISABLE_FIND_PACKAGE_WavPack=ON"
        ])
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        copy(self, "*.h", src=os.path.join(self.source_folder, "include"),
             dst=os.path.join(self.package_folder, "include"), keep_path=True)

    def package_info(self):
        if self.options.shared:
            self.cpp_info.libs = ["SDL3_mixer"]
            self.cpp_info.set_property("cmake_target_name", "SDL3_mixer::SDL3_mixer-shared")
            self.cpp_info.set_property("cmake_target_aliases", ["SDL3_mixer::SDL3_mixer"])
        else:
            self.cpp_info.libs = ["SDL3_mixer-static"]
            self.cpp_info.set_property("cmake_target_name", "SDL3_mixer::SDL3_mixer-static")
            self.cpp_info.set_property("cmake_target_aliases", ["SDL3_mixer::SDL3_mixer"])

        self.cpp_info.set_property("cmake_file_name", "SDL3_mixer")

    def layout(self):
        cmake_layout(self)
