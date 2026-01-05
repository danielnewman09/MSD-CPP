from conan import ConanFile
from conan.tools.scm import Git
from conan.tools.files import save, copy
from conan.errors import ConanInvalidConfiguration
import os


class EmsdkRecipe(ConanFile):
    name = "emsdk"
    version = "4.0.22"
    package_type = "application"

    # Metadata
    license = "MIT"
    author = "Emscripten Contributors"
    url = "https://github.com/emscripten-core/emsdk"
    description = "Emscripten SDK - Compile C/C++ to WebAssembly"
    topics = ("emscripten", "wasm", "webassembly", "compiler", "toolchain")

    # This is a build tool, not a library
    settings = "os", "arch"

    # No binary configuration needed - this is a toolchain
    exports_sources = []

    def validate(self):
        # emsdk only works on certain host platforms
        if self.settings.os not in ["Linux", "Macos", "Windows"]:
            raise ConanInvalidConfiguration(
                f"emsdk is not supported on {self.settings.os}"
            )

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/emscripten-core/emsdk.git", target=".")
        git.checkout(self.version)

    def build(self):
        # Install the specific Emscripten version
        emsdk_path = self.source_folder

        if self.settings.os == "Windows":
            emsdk_cmd = os.path.join(emsdk_path, "emsdk.bat")
        else:
            emsdk_cmd = os.path.join(emsdk_path, "emsdk")

        # Install and activate the SDK
        self.run(f"{emsdk_cmd} install {self.version}")
        self.run(f"{emsdk_cmd} activate {self.version}")

    def package(self):
        # Copy the entire emsdk directory to package
        copy(self, "*", src=self.source_folder, dst=self.package_folder,
             excludes=["*.git*"])

    def package_info(self):
        # Set up environment variables for using Emscripten
        emsdk_path = self.package_folder
        upstream_path = os.path.join(emsdk_path, "upstream", "emscripten")
        node_path = os.path.join(emsdk_path, "node", f"{self._node_version}_64bit", "bin")
        python_path = os.path.join(emsdk_path, "python", f"{self._python_version}_64bit", "bin")

        # Add to PATH - Python must come first so emcc uses bundled Python 3.13+
        self.buildenv_info.prepend_path("PATH", python_path)
        self.buildenv_info.prepend_path("PATH", node_path)
        self.buildenv_info.prepend_path("PATH", upstream_path)
        self.buildenv_info.prepend_path("PATH", emsdk_path)

        # Set EMSDK environment variables
        self.buildenv_info.define("EMSDK", emsdk_path)
        self.buildenv_info.define("EMSDK_NODE", os.path.join(node_path, "node"))
        self.buildenv_info.define("EMSDK_PYTHON", os.path.join(python_path, "python3"))
        self.buildenv_info.define("EM_CONFIG", os.path.join(emsdk_path, ".emscripten"))

        # CMake integration
        self.conf_info.define("tools.cmake.cmaketoolchain:toolchain_file",
                              os.path.join(upstream_path, "cmake", "Modules", "Platform", "Emscripten.cmake"))

    @property
    def _node_version(self):
        # Node version bundled with emsdk 4.0.22
        return "22.16.0"

    @property
    def _python_version(self):
        # Python version bundled with emsdk 4.0.22
        return "3.13.3"