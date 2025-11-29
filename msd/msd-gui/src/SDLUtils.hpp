#ifndef SDL_UTILS_HPP
#define SDL_UTILS_HPP

#include <memory>
#include <stdexcept>
#include <string>

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


struct ShaderDeleter
{
  SDL_GPUDevice* device;
  void operator()(SDL_GPUShader* s) const
  {
    SDL_ReleaseGPUShader(device, s);
  }
};

using UniqueShader = std::unique_ptr<SDL_GPUShader, ShaderDeleter>;

UniqueShader loadShader(const std::string& shaderFilename,
                        SDL_GPUDevice& device,
                        const std::string& basePath,
                        uint32_t samplerCount,
                        uint32_t uniformBufferCount,
                        uint32_t storageBufferCount,
                        uint32_t storageTextureCount);


}  // namespace msd_gui

#endif