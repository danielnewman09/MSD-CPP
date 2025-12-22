#ifndef SDL_GPU_MANAGER_HPP
#define SDL_GPU_MANAGER_HPP

#include <memory>
#include <string>

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include <msd-assets/src/Geometry.hpp>
#include <msd-gui/src/Camera3D.hpp>
#include <msd-sim/src/Environment/Coordinate.hpp>
#include <msd-sim/src/Environment/ReferenceFrame.hpp>

namespace msd_gui
{

class SDLException;  // Forward declaration

// Per-instance data for instanced rendering
struct InstanceData
{
  float position[3];  // World position offset for this instance
  float color[3];     // Color for this instance
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

  Camera3D& getCamera();

  // Instance management methods
  void addInstance(float posX,
                   float posY,
                   float posZ,
                   float r,
                   float g,
                   float b);
  void removeInstance(size_t index);
  void updateInstance(size_t index,
                      float posX,
                      float posY,
                      float posZ,
                      float r,
                      float g,
                      float b);
  void clearInstances();
  size_t getInstanceCount() const
  {
    return instances_.size();
  }

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
  UniqueBuffer vertexBuffer_;    // Vertex buffer for base mesh geometry
  UniqueBuffer instanceBuffer_;  // Instance buffer for per-instance data
  UniqueBuffer uniformBuffer_;

  std::vector<InstanceData> instances_;  // CPU-side instance data
  size_t pyramidVertexCount_{0};  // Number of vertices in base pyramid mesh
  bool instanceBufferNeedsUpdate_{
    false};  // Flag to track if instance buffer needs upload

  std::unique_ptr<SDL_GPUDevice, SDLDeviceDeleter> device_;
  std::string basePath_;

  Camera3D camera_;

  void uploadInstanceBuffer();
};

}  // namespace msd_gui

#endif