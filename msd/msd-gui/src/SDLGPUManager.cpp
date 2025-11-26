#include "msd-gui/src/SDLGPUManager.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

#include <SDL3/SDL.h>

namespace msd_gui
{

class SDLException final : public std::runtime_error
{
public:
  explicit SDLException(const std::string& message)
    : std::runtime_error(message + ": " + SDL_GetError())
  {
  }
};

GPUManager::GPUManager(SDL_Window& window, const std::string& basePath)
  : window_{window}, basePath_{basePath}
{
  device_.reset(SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_MSL |
                                      SDL_GPU_SHADERFORMAT_DXIL |
                                      SDL_GPU_SHADERFORMAT_SPIRV,
                                    true,
                                    NULL));

  if (!device_.get())
  {
    throw SDLException("Failed to create SDL GPU device");
  }

  if (!SDL_ClaimWindowForGPUDevice(device_.get(), &window))
  {
    throw SDLException("Failed to claim window with GPU Device");
  }

  std::cout << "Using GPU device driver: "
            << SDL_GetGPUDeviceDriver(device_.get()) << std::endl;

  vertexShader_ =
    UniqueShader(loadShader(device_.get(), "RawTriangle.vert", 0, 0, 0, 0),
                 ShaderDeleter{device_.get()});

  if (!vertexShader_.get())
  {
    throw SDLException("Failed to load vertexShader");
  }

  fragmentShader_ =
    UniqueShader(loadShader(device_.get(), "SolidColor.frag", 0, 0, 0, 0),
                 ShaderDeleter{device_.get()});

  if (!fragmentShader_.get())
  {
    throw SDLException("Failed to load fragmentShader");
  }
}

void GPUManager::render()
{
  SDL_GPUCommandBuffer* commandBuffer{
    SDL_AcquireGPUCommandBuffer(device_.get())};

  if (!commandBuffer)
  {
    throw SDLException("Couldn't acquire command buffer");
  }

  SDL_GPUTexture* swapchainTexture;

  SDL_WaitAndAcquireGPUSwapchainTexture(
    commandBuffer, &window_, &swapchainTexture, nullptr, nullptr);

  if (swapchainTexture)
  {
    SDL_GPUColorTargetInfo colorTarget{};
    colorTarget.texture = swapchainTexture;
    colorTarget.store_op = SDL_GPU_STOREOP_STORE;
    colorTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    colorTarget.clear_color = SDL_FColor{1.0f, 0.0f, 0.0f, 1.0f};
    std::vector<SDL_GPUColorTargetInfo> colorTargets{colorTarget};
    SDL_GPURenderPass* renderPass{SDL_BeginGPURenderPass(
      commandBuffer, colorTargets.data(), colorTargets.size(), NULL)};

    SDL_EndGPURenderPass(renderPass);
  }

  if (!SDL_SubmitGPUCommandBuffer(commandBuffer))
  {
    throw SDLException("Couldn't submit command buffer to GPU");
  }
}

SDL_GPUShader* GPUManager::loadShader(SDL_GPUDevice* device,
                                      const char* shaderFilename,
                                      uint32_t samplerCount,
                                      uint32_t uniformBufferCount,
                                      uint32_t storageBufferCount,
                                      uint32_t storageTextureCount)
{
  // Auto-detect the shader stage from the file name for convenience
  SDL_GPUShaderStage stage;
  if (SDL_strstr(shaderFilename, ".vert"))
  {
    stage = SDL_GPU_SHADERSTAGE_VERTEX;
  }
  else if (SDL_strstr(shaderFilename, ".frag"))
  {
    stage = SDL_GPU_SHADERSTAGE_FRAGMENT;
  }
  else
  {
    SDL_Log("Invalid shader stage!");
    return nullptr;
  }

  char fullPath[256];
  SDL_GPUShaderFormat backendFormats = SDL_GetGPUShaderFormats(device_.get());
  SDL_GPUShaderFormat format = SDL_GPU_SHADERFORMAT_INVALID;
  const char* entrypoint;

  if (backendFormats & SDL_GPU_SHADERFORMAT_SPIRV)
  {
    SDL_snprintf(fullPath,
                 sizeof(fullPath),
                 "%sContent/Shaders/Compiled/SPIRV/%s.spv",
                 basePath_.c_str(),
                 shaderFilename);
    format = SDL_GPU_SHADERFORMAT_SPIRV;
    entrypoint = "main";
  }
  else if (backendFormats & SDL_GPU_SHADERFORMAT_MSL)
  {
    SDL_snprintf(fullPath,
                 sizeof(fullPath),
                 "%sContent/Shaders/Compiled/MSL/%s.msl",
                 basePath_.c_str(),
                 shaderFilename);
    format = SDL_GPU_SHADERFORMAT_MSL;
    entrypoint = "main0";
  }
  else if (backendFormats & SDL_GPU_SHADERFORMAT_DXIL)
  {
    SDL_snprintf(fullPath,
                 sizeof(fullPath),
                 "%sContent/Shaders/Compiled/DXIL/%s.dxil",
                 basePath_.c_str(),
                 shaderFilename);
    format = SDL_GPU_SHADERFORMAT_DXIL;
    entrypoint = "main";
  }
  else
  {
    SDL_Log("%s", "Unrecognized backend shader format!");
    return nullptr;
  }

  size_t codeSize;
  void* code = SDL_LoadFile(fullPath, &codeSize);
  if (code == nullptr)
  {
    SDL_Log("Failed to load shader from disk! %s", fullPath);
    return nullptr;
  }

  SDL_GPUShaderCreateInfo shaderInfo = {
    .code = static_cast<const uint8_t*>(code),
    .code_size = codeSize,
    .entrypoint = entrypoint,
    .format = format,
    .stage = stage,
    .num_samplers = samplerCount,
    .num_uniform_buffers = uniformBufferCount,
    .num_storage_buffers = storageBufferCount,
    .num_storage_textures = storageTextureCount};

  SDL_GPUShader* shader = SDL_CreateGPUShader(device, &shaderInfo);
  if (shader == NULL)
  {
    SDL_Log("Failed to create shader!");
    SDL_free(code);
    return NULL;
  }

  SDL_free(code);
  return shader;
}

}  // namespace msd_gui