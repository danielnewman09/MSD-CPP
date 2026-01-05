#!/bin/bash
# Requires shadercross CLI installed from SDL_shadercross
# Optionally requires naga CLI for WGSL conversion (for WebGPU/Emscripten builds)
# Install naga: cargo install naga-cli

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check if naga is available for WGSL conversion
NAGA_AVAILABLE=false
if command -v naga &> /dev/null; then
    NAGA_AVAILABLE=true
    echo "naga found - WGSL shaders will be generated for WebGPU"
else
    echo "naga not found - skipping WGSL shader generation"
    echo "To enable WGSL: cargo install naga-cli"
fi

compile_shader() {
    local filename="$1"
    local basename="${filename/.hlsl/}"

    # Native backends
    shadercross "$filename" -o "../Compiled/SPIRV/${basename}.spv"
    shadercross "$filename" -o "../Compiled/MSL/${basename}.msl"
    shadercross "$filename" -o "../Compiled/DXIL/${basename}.dxil"

    # WebGPU backend (SPIRV -> WGSL via naga)
    if [ "$NAGA_AVAILABLE" = true ]; then
        naga "../Compiled/SPIRV/${basename}.spv" "../Compiled/WGSL/${basename}.wgsl" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "  Generated WGSL: ${basename}.wgsl"
        else
            echo "  Warning: Failed to generate WGSL for ${basename}"
        fi
    fi
}

for filename in *.vert.hlsl; do
    if [ -f "$filename" ]; then
        echo "Compiling vertex shader: $filename"
        compile_shader "$filename"
    fi
done

for filename in *.frag.hlsl; do
    if [ -f "$filename" ]; then
        echo "Compiling fragment shader: $filename"
        compile_shader "$filename"
    fi
done

for filename in *.comp.hlsl; do
    if [ -f "$filename" ]; then
        echo "Compiling compute shader: $filename"
        compile_shader "$filename"
    fi
done

echo "Shader compilation complete."
