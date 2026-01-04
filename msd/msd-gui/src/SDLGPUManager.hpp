// Ticket: 0001_link-gui-sim-object
// Design: docs/designs/generalize-gui-object-rendering/design.md

#ifndef SDL_GPU_MANAGER_HPP
#define SDL_GPU_MANAGER_HPP

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include <Eigen/Dense>
#include <msd-assets/src/Geometry.hpp>
#include <msd-gui/src/Camera3D.hpp>
#include <msd-sim/src/Environment/Coordinate.hpp>
#include <msd-sim/src/Environment/Object.hpp>
#include <msd-sim/src/Environment/ReferenceFrame.hpp>

namespace msd_gui
{

class SDLException;  // Forward declaration

/**
 * @brief Geometry registry entry tracking a geometry type's location within
 * the unified vertex buffer
 * @ticket 0001_link-gui-sim-object
 */
struct GeometryInfo
{
  uint32_t baseVertex{0};   // Starting vertex index in unified buffer
  uint32_t vertexCount{0};  // Number of vertices for this geometry
};

/**
 * @brief Per-instance data for instanced rendering
 * @ticket 0001_link-gui-sim-object
 *
 * Stores a full 4x4 model matrix (translation + rotation), color, and geometry
 * index for each rendered instance. Size: 84 bytes (64 + 12 + 4 + 4)
 */
struct InstanceData
{
  float modelMatrix[16];   // 4x4 transform matrix (translation + rotation)
  float color[3];          // RGB color
  uint32_t geometryIndex;  // Index into geometryRegistry_
  uint32_t padding[4] = {0, 0, 0, 0};  // Alignment padding (16-byte boundary)
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

  // Object management methods
  size_t addObject(const msd_sim::Object& object);
  void removeObject(size_t index);
  void updateObjects(const std::vector<msd_sim::Object>& objects);
  void clearObjects();
  size_t getObjectCount() const
  {
    return objectIndices_.size();
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
  std::vector<size_t> objectIndices_;    // Indices into external object vector

  // Geometry registry for unified vertex buffer
  std::vector<GeometryInfo> geometryRegistry_;
  std::unordered_map<std::string, uint32_t> geometryNameToIndex_;
  size_t totalVertexCount_{0};

  std::unique_ptr<SDL_GPUDevice, SDLDeviceDeleter> device_;
  std::string basePath_;

  Camera3D camera_;

  void uploadInstanceBuffer();

  // Geometry registration system
  uint32_t registerGeometry(const std::string& name,
                            const std::vector<msd_assets::Vertex>& vertices);

  // Object → InstanceData conversion
  InstanceData buildInstanceData(const msd_sim::Object& object);
  Eigen::Matrix4f createModelMatrix(const msd_sim::ReferenceFrame& transform);
};

}  // namespace msd_gui

#endif