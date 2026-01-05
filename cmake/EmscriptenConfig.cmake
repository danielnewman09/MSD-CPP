# EmscriptenConfig.cmake - CMake configuration for Emscripten/WebAssembly builds
# This file is included when EMSCRIPTEN is detected
#
# Note: Memory configuration (ALLOW_MEMORY_GROWTH, INITIAL_MEMORY, MAXIMUM_MEMORY)
# is set in the Conan profile (profiles/emscripten.profile) to avoid duplication.

if(EMSCRIPTEN)
    message(STATUS "Configuring for Emscripten/WebAssembly with WebGPU backend")

    # Output HTML files instead of executables
    set(CMAKE_EXECUTABLE_SUFFIX ".html")

    # SDL3 from Emscripten ports
    add_compile_options(-sUSE_SDL=3)
    add_link_options(-sUSE_SDL=3)

    # WebGPU support via Dawn (emdawnwebgpu port)
    # Note: -sUSE_WEBGPU=1 is deprecated in Emscripten 4.0+
    # Use --use-port=emdawnwebgpu instead
    add_compile_options(--use-port=emdawnwebgpu)
    add_link_options(--use-port=emdawnwebgpu)

    # Asyncify for WebGPU async operations (adapter/device requests)
    add_link_options(-sASYNCIFY)
    add_link_options(-sASYNCIFY_STACK_SIZE=32768)

    # Export functions for JavaScript interop
    add_link_options(-sEXPORTED_FUNCTIONS=['_main'])
    add_link_options(-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap'])

    # Enable filesystem for asset loading
    add_link_options(-sFORCE_FILESYSTEM=1)

    # Disable tests for web builds
    set(BUILD_TESTING OFF CACHE BOOL "Disable testing for Emscripten" FORCE)

    # Define preprocessor macro for conditional compilation
    add_compile_definitions(__EMSCRIPTEN_BUILD__)
endif()