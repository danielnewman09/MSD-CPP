#ifndef SDL_GPU_MANAGER_HPP
#define SDL_GPU_MANAGER_HPP

#include <memory>
#include <string>

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>

namespace msd_gui
{

class SDLException;  // Forward declaration

class GPUManager
{
public:
  explicit GPUManager(SDL_Window& window, const std::string& basePath);
  ~GPUManager() = default;

  // Delete copy constructor and assignment operator
  GPUManager(const GPUManager&) = delete;
  GPUManager& operator=(const GPUManager&) = delete;

  // Delete move constructor and assignment operator
  GPUManager(GPUManager&&) = delete;
  GPUManager& operator=(GPUManager&&) = delete;

  void render();

private:
  struct SDLDeviceDeleter
  {
    void operator()(SDL_GPUDevice* d) const
    {
      SDL_DestroyGPUDevice(d);
    }
  };

  struct ShaderDeleter
  {
    SDL_GPUDevice* device;
    void operator()(SDL_GPUShader* s) const
    {
      SDL_ReleaseGPUShader(device, s);
    }
  };

  using UniqueShader = std::unique_ptr<SDL_GPUShader, ShaderDeleter>;

  SDL_Window& window_;

  SDL_GPUShader* loadShader(SDL_GPUDevice* device,
                            const char* shaderFilename,
                            uint32_t samplerCount,
                            uint32_t uniformBufferCount,
                            uint32_t storageBufferCount,
                            uint32_t storageTextureCount);

  std::unique_ptr<SDL_GPUDevice, SDLDeviceDeleter> device_;
  UniqueShader vertexShader_;
  UniqueShader fragmentShader_;
  std::string basePath_;
};

}  // namespace msd_gui

#endif