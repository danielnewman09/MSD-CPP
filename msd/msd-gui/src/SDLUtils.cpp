#include "msd-gui/src/SDLUtils.hpp"

#include <fstream>

namespace msd_gui
{
UniqueShader loadShader(const std::string& shaderFilename,
                        SDL_GPUDevice& device,
                        const std::string& basePath,
                        uint32_t samplerCount,
                        uint32_t uniformBufferCount,
                        uint32_t storageBufferCount,
                        uint32_t storageTextureCount)
{
  // Auto-detect the shader stage from the file name for convenience
  SDL_GPUShaderStage stage;
  if (SDL_strstr(shaderFilename.c_str(), ".vert"))
  {
    stage = SDL_GPU_SHADERSTAGE_VERTEX;
  }
  else if (SDL_strstr(shaderFilename.c_str(), ".frag"))
  {
    stage = SDL_GPU_SHADERSTAGE_FRAGMENT;
  }
  else
  {
    SDL_Log("Invalid shader stage!");
    return nullptr;
  }

  std::string fullPath;
  SDL_GPUShaderFormat backendFormats = SDL_GetGPUShaderFormats(&device);
  SDL_GPUShaderFormat format = SDL_GPU_SHADERFORMAT_INVALID;
  std::string entrypoint;

  if (backendFormats & SDL_GPU_SHADERFORMAT_SPIRV)
  {
    fullPath =
      basePath + "Content/Shaders/Compiled/SPIRV/" + shaderFilename + ".spv";
    format = SDL_GPU_SHADERFORMAT_SPIRV;
    entrypoint = "main";
  }
  else if (backendFormats & SDL_GPU_SHADERFORMAT_MSL)
  {
    fullPath =
      basePath + "Content/Shaders/Compiled/MSL/" + shaderFilename + ".msl";
    format = SDL_GPU_SHADERFORMAT_MSL;
    entrypoint = "main0";
  }
  else if (backendFormats & SDL_GPU_SHADERFORMAT_DXIL)
  {
    fullPath =
      basePath + "Content/Shaders/Compiled/DXIL/" + shaderFilename + ".dxil";
    format = SDL_GPU_SHADERFORMAT_DXIL;
    entrypoint = "main";
  }
  else
  {
    SDL_Log("%s", "Unrecognized backend shader format!");
    return nullptr;
  }

  std::ifstream file(fullPath, std::ios::binary);
  if (!file)
  {
    SDL_Log("Failed to load shader from disk! %s", fullPath.c_str());
    return nullptr;
  }

  std::vector<uint8_t> code{std::istreambuf_iterator<char>(file),
                            std::istreambuf_iterator<char>()};

  SDL_GPUShaderCreateInfo shaderInfo = {
    .code = code.data(),
    .code_size = code.size(),
    .entrypoint = entrypoint.c_str(),
    .format = format,
    .stage = stage,
    .num_samplers = samplerCount,
    .num_uniform_buffers = uniformBufferCount,
    .num_storage_buffers = storageBufferCount,
    .num_storage_textures = storageTextureCount};

  auto shader = UniqueShader(SDL_CreateGPUShader(&device, &shaderInfo),
                             ShaderDeleter{&device});
  if (!shader)
  {
    SDL_Log("Failed to create shader!");
    return nullptr;
  }

  return shader;
}
}  // namespace msd_gui