#ifndef SDL_GPU_MANAGER_HPP
#define SDL_GPU_MANAGER_HPP

#include <memory>
#include <string>

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include <msd-sim/src/Environment/Coordinate.hpp>
#include <msd-assets/src/Geometry.hpp>

namespace msd_gui
{

class SDLException;  // Forward declaration

struct Vertex
{
  float position[3];  // Position (x, y, z)
  float color[3];     // Color (r, g, b)
  float normal[3];    // Normal vector (x, y, z)
};

// Transform uniform buffer data (must match shader layout)
struct TransformData
{
  float mvpMatrix[16];  // 4x4 model-view-projection matrix (column-major)
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

  // Set the camera position and rotation
  void setCameraTransform(float posX, float posY, float posZ, float rotX, float rotY, float rotZ);

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

  float cameraPosX_{0.0f};
  float cameraPosY_{0.0f};
  float cameraPosZ_{5.0f};  // Camera starts 5 units away on positive Z, looking toward origin
  float cameraRotX_{0.0f};
  float cameraRotY_{0.0f};
  float cameraRotZ_{0.0f};

  void updateTransformMatrix();

  /**
   * @brief Convert Geometry to Vertex vector with colors
   * @param geometry The geometry to convert
   * @param r Red component (0.0-1.0)
   * @param g Green component (0.0-1.0)
   * @param b Blue component (0.0-1.0)
   * @return Vector of Vertex structs ready for GPU upload
   */
  static std::vector<Vertex> geometryToVertices(const msd_assets::Geometry& geometry,
                                                  float r = 1.0f,
                                                  float g = 1.0f,
                                                  float b = 1.0f);
};

}  // namespace msd_gui

#endif