#ifndef SDL_GPU_MANAGER_HPP
#define SDL_GPU_MANAGER_HPP

#include <memory>
#include <string>

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>

namespace msd_gui
{

class SDLException;  // Forward declaration

struct Vertex
{
  float x, y, z;        // Position (3D for shader compatibility)
  float r, g, b, a;     // Color (RGBA for shader compatibility)
};

// Transform uniform buffer data (must match shader layout)
struct TransformData
{
  float offsetX;
  float offsetY;
  float padding[2];  // Align to 16 bytes for uniform buffer
};

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

  // Set the position offset for the triangle
  void setPosition(float x, float y);

private:
  struct SDLDeviceDeleter
  {
    void operator()(SDL_GPUDevice* d) const
    {
      SDL_DestroyGPUDevice(d);
    }
  };

  struct PipelineDeleter
  {
    SDL_GPUDevice* device;
    void operator()(SDL_GPUGraphicsPipeline* p) const
    {
      SDL_ReleaseGPUGraphicsPipeline(device, p);
    }
  };

  struct BufferDeleter
  {
    SDL_GPUDevice* device;
    void operator()(SDL_GPUBuffer* b) const
    {
      SDL_ReleaseGPUBuffer(device, b);
    }
  };

  using UniquePipeline =
    std::unique_ptr<SDL_GPUGraphicsPipeline, PipelineDeleter>;
  using UniqueBuffer = std::unique_ptr<SDL_GPUBuffer, BufferDeleter>;

  SDL_Window& window_;

  UniquePipeline pipeline_;
  UniqueBuffer vertexBuffer_;
  UniqueBuffer uniformBuffer_;
  TransformData transform_;

  std::unique_ptr<SDL_GPUDevice, SDLDeviceDeleter> device_;
  std::string basePath_;
};

}  // namespace msd_gui

#endif