from conan import ConanFile
from conan.tools.cmake import CMake
from conan.tools.files import copy
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps

from pathlib import Path

local_script_abs_path = Path(__file__)
_WORKSPACE_ROOT_DIR = local_script_abs_path.parent.absolute()


class MyProjectConan(ConanFile):
    name = "MyProject"
    version = "1.0"
    settings = "os", "arch", "compiler", "build_type"
    
    exports_sources = "CMakeLists.txt", "msd/**"
    
    requires = [
        "sfml/2.6.1",
        "eigen/3.4.0",
        "gtest/1.15.0"
    ]
 
        
    def generate(self):
        for dep in self.dependencies.values():
            
            if len(dep.cpp_info.libdirs) > 0:
                installs_dir = _WORKSPACE_ROOT_DIR.joinpath('./installs/')
                import pdb
                pdb.set_trace()
                copy(self, "*",dep.cpp_info.libdirs[0], dst=installs_dir.absolute(),  excludes=('conaninfo*', 'conanmanifest*'))
                
        copy(self, "*",self.cpp_info.libdirs[0], dst=installs_dir.absolute(),  excludes=('conaninfo*', 'conanmanifest*'))
               
        
    def package(self):
        cmake = CMake(self)
        cmake.install()
    
    def build_requirements(self):
        self.tool_requires("cmake/3.22.6")
            
    def layout(self):
        cmake_layout(self)
        
    # def package(self):
    #     copy(self, "*", dst="include", src="include")
    #     copy(self, "*", dst="lib", src="lib")
