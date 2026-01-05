// Ticket: 0002_remove_rotation_from_gpu
// Design: docs/designs/modularize-gpu-shader-system/design.md
// Previous ticket: 0001_link-gui-sim-object

#ifndef SDL_GPU_MANAGER_HPP
#define SDL_GPU_MANAGER_HPP

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include <Eigen/Dense>
#include <msd-assets/src/Geometry.hpp>
#include <msd-assets/src/GeometryFactory.hpp>
#include <msd-gui/src/Camera3D.hpp>
#include <msd-gui/src/ShaderPolicy.hpp>
#include <msd-gui/src/SDLUtils.hpp>
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
 * @brief Templated GPU manager for SDL GPU rendering with configurable shader policies
 * @tparam ShaderPolicy Shader policy type (e.g., PositionOnlyShaderPolicy, FullTransformShaderPolicy)
 * @ticket 0002_remove_rotation_from_gpu
 * @see docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml
 *
 * Manages GPU device, pipeline, buffers, and rendering for instanced 3D objects.
 * The shader policy determines vertex attributes, instance data layout, and shader files.
 *
 * Thread safety: Not thread-safe - all operations must occur on the main thread.
 */
template<typename ShaderPolicy>
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
  using InstanceDataType = typename ShaderPolicy::InstanceDataType;

  SDL_Window& window_;

  UniquePipeline pipeline_;
  UniqueBuffer vertexBuffer_;    // Vertex buffer for base mesh geometry
  UniqueBuffer instanceBuffer_;  // Instance buffer for per-instance data
  UniqueBuffer uniformBuffer_;

  ShaderPolicy shaderPolicy_;  // Shader policy instance

  std::vector<InstanceDataType> instances_;  // CPU-side instance data
  std::vector<size_t> objectIndices_;        // Indices into external object vector

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
  InstanceDataType buildInstanceData(const msd_sim::Object& object);
};

//=============================================================================
// Template Implementation
//=============================================================================

template<typename ShaderPolicy>
GPUManager<ShaderPolicy>::GPUManager(SDL_Window& window,
                                     const std::string& basePath)
  : window_{window},
    basePath_{basePath},
    camera_{msd_sim::Coordinate{0., 0., 5.}}
{
  SDL_Log("GPUManager: Starting initialization, basePath=%s", basePath_.c_str());

  SDL_Log("GPUManager: Creating GPU device...");
  device_.reset(SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_MSL |
                                      SDL_GPU_SHADERFORMAT_DXIL |
                                      SDL_GPU_SHADERFORMAT_SPIRV,
                                    true,
                                    NULL));

  if (!device_.get())
  {
    SDL_Log("GPUManager: ERROR - Failed to create SDL GPU device: %s", SDL_GetError());
    throw SDLException("Failed to create SDL GPU device");
  }
  SDL_Log("GPUManager: GPU device created successfully");

  SDL_Log("GPUManager: Claiming window for GPU device...");
  if (!SDL_ClaimWindowForGPUDevice(device_.get(), &window))
  {
    SDL_Log("GPUManager: ERROR - Failed to claim window: %s", SDL_GetError());
    throw SDLException("Failed to claim window with GPU Device");
  }
  SDL_Log("GPUManager: Window claimed successfully");

  SDL_Log("Using GPU device driver: %s", SDL_GetGPUDeviceDriver(device_.get()));

  // Load shaders via shader policy
  SDL_Log("GPUManager: Loading vertex shader: %s", shaderPolicy_.getVertexShaderFile().c_str());
  auto vertexShader = loadShader(shaderPolicy_.getVertexShaderFile(),
                                 *device_,
                                 basePath_,
                                 0,
                                 1,
                                 0,
                                 0);

  if (!vertexShader.get())
  {
    SDL_Log("ERROR: Failed to load vertex shader: %s",
            shaderPolicy_.getVertexShaderFile().c_str());
    throw SDLException("Failed to load vertexShader");
  }
  SDL_Log("Successfully loaded vertex shader: %s",
          shaderPolicy_.getVertexShaderFile().c_str());

  auto fragmentShader = loadShader(shaderPolicy_.getFragmentShaderFile(),
                                   *device_,
                                   basePath_,
                                   0,
                                   0,
                                   0,
                                   0);

  if (!fragmentShader.get())
  {
    SDL_Log("ERROR: Failed to load fragment shader: %s",
            shaderPolicy_.getFragmentShaderFile().c_str());
    throw SDLException("Failed to load fragmentShader");
  }
  SDL_Log("Successfully loaded fragment shader: %s",
          shaderPolicy_.getFragmentShaderFile().c_str());

  // Create unified vertex buffer for multiple geometry types
  // Register pyramid geometry
  auto pyramidMeshRecord = msd_assets::GeometryFactory::createPyramid(1.0, 1.0);
  msd_assets::VisualGeometry pyramidGeometry{pyramidMeshRecord, 0};
  const auto& pyramidVertices = pyramidGeometry.getVertices();

  // Register cube geometry
  auto cubeMeshRecord = msd_assets::GeometryFactory::createCube(1.0);
  msd_assets::VisualGeometry cubeGeometry{cubeMeshRecord, 0};
  const auto& cubeVertices = cubeGeometry.getVertices();

  // Combine all vertices into unified buffer
  std::vector<msd_assets::Vertex> allVertices;
  allVertices.reserve(pyramidVertices.size() + cubeVertices.size());

  // Register geometries and build unified vertex buffer
  registerGeometry("pyramid", pyramidVertices);
  registerGeometry("cube", cubeVertices);

  // Append all vertices
  allVertices.insert(
    allVertices.end(), pyramidVertices.begin(), pyramidVertices.end());
  allVertices.insert(
    allVertices.end(), cubeVertices.begin(), cubeVertices.end());

  // Create unified vertex buffer
  SDL_GPUBufferCreateInfo vertexBufferCreateInfo = {
    .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
    .size =
      static_cast<uint32_t>(allVertices.size() * sizeof(msd_assets::Vertex))};

  vertexBuffer_ =
    UniqueBuffer(SDL_CreateGPUBuffer(device_.get(), &vertexBufferCreateInfo),
                 BufferDeleter{device_.get()});

  if (!vertexBuffer_)
  {
    throw SDLException("Failed to create vertex buffer!");
  }

  // Upload unified vertex data to GPU
  SDL_GPUTransferBufferCreateInfo vertexTransferCreateInfo = {
    .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
    .size = vertexBufferCreateInfo.size};

  SDL_GPUTransferBuffer* vertexTransferBuffer =
    SDL_CreateGPUTransferBuffer(device_.get(), &vertexTransferCreateInfo);

  if (!vertexTransferBuffer)
  {
    throw SDLException("Failed to create vertex transfer buffer!");
  }

  void* vertexTransferData =
    SDL_MapGPUTransferBuffer(device_.get(), vertexTransferBuffer, false);
  std::memcpy(
    vertexTransferData, allVertices.data(), vertexBufferCreateInfo.size);
  SDL_UnmapGPUTransferBuffer(device_.get(), vertexTransferBuffer);

  SDL_GPUCommandBuffer* vertexUploadCmd =
    SDL_AcquireGPUCommandBuffer(device_.get());
  SDL_GPUCopyPass* vertexCopyPass = SDL_BeginGPUCopyPass(vertexUploadCmd);

  SDL_GPUTransferBufferLocation vertexTransferLocation = {
    .transfer_buffer = vertexTransferBuffer, .offset = 0};

  SDL_GPUBufferRegion vertexBufferRegion = {
    .buffer = vertexBuffer_.get(),
    .offset = 0,
    .size = vertexBufferCreateInfo.size};

  SDL_UploadToGPUBuffer(
    vertexCopyPass, &vertexTransferLocation, &vertexBufferRegion, false);
  SDL_EndGPUCopyPass(vertexCopyPass);
  SDL_SubmitGPUCommandBuffer(vertexUploadCmd);

  SDL_ReleaseGPUTransferBuffer(device_.get(), vertexTransferBuffer);

  SDL_Log("Registered %zu geometry types, total vertices: %zu",
          geometryRegistry_.size(),
          totalVertexCount_);

  // Create instance buffer (pre-allocate space for many instances)
  const size_t maxInstances = 1000;
  SDL_GPUBufferCreateInfo instanceBufferCreateInfo = {
    .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
    .size = static_cast<uint32_t>(maxInstances * shaderPolicy_.getInstanceDataSize())};

  instanceBuffer_ =
    UniqueBuffer(SDL_CreateGPUBuffer(device_.get(), &instanceBufferCreateInfo),
                 BufferDeleter{device_.get()});

  if (!instanceBuffer_)
  {
    throw SDLException("Failed to create instance buffer!");
  }

  // Upload initial instance data
  uploadInstanceBuffer();

  SDL_GPUBufferCreateInfo uniformBufferCreateInfo = {
    .usage = SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ,
    .size = sizeof(Eigen::Matrix4f)};

  uniformBuffer_ =
    UniqueBuffer(SDL_CreateGPUBuffer(device_.get(), &uniformBufferCreateInfo),
                 BufferDeleter{device_.get()});

  if (!uniformBuffer_)
  {
    throw SDLException("Failed to create uniform buffer!");
  }

  // Create the pipelines
  SDL_GPUColorTargetDescription colorTargetDesc = {
    .format = SDL_GetGPUSwapchainTextureFormat(device_.get(), &window)};

  // Get vertex input state from shader policy
  SDL_GPUVertexInputState vertexInputState = shaderPolicy_.getVertexInputState();

  SDL_GPUGraphicsPipelineCreateInfo pipelineCreateInfo = {
    .vertex_input_state = vertexInputState,
    .target_info =
      {
        .num_color_targets = 1,
        .color_target_descriptions = &colorTargetDesc,
        .has_depth_stencil_target = true,
        .depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D16_UNORM,
      },
    .primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
    .vertex_shader = vertexShader.get(),
    .fragment_shader = fragmentShader.get(),
  };
  pipelineCreateInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
  pipelineCreateInfo.rasterizer_state.cull_mode =
    SDL_GPU_CULLMODE_NONE;  // Disable backface culling
  pipelineCreateInfo.depth_stencil_state.enable_depth_test = true;
  pipelineCreateInfo.depth_stencil_state.enable_depth_write = true;
  pipelineCreateInfo.depth_stencil_state.compare_op =
    SDL_GPU_COMPAREOP_LESS;  // Standard depth test

  pipeline_ = UniquePipeline(
    SDL_CreateGPUGraphicsPipeline(device_.get(), &pipelineCreateInfo),
    PipelineDeleter{device_.get()});

  if (!pipeline_)
  {
    SDL_Log("ERROR: Failed to create graphics pipeline!");
    throw SDLException("Failed to create line pipeline!");
  }
  SDL_Log("Successfully created graphics pipeline with %s shader policy",
          ShaderPolicy::kShaderName);
  SDL_Log("Total vertex count: %zu, Geometry types registered: %zu, "
          "Vertex size: %zu bytes, Instance size: %zu bytes",
          totalVertexCount_,
          geometryRegistry_.size(),
          sizeof(msd_assets::Vertex),
          shaderPolicy_.getInstanceDataSize());
  for (const auto& [name, index] : geometryNameToIndex_)
  {
    const auto& geomInfo = geometryRegistry_[index];
    SDL_Log("  Geometry '%s': baseVertex=%u, vertexCount=%u",
            name.c_str(),
            geomInfo.baseVertex,
            geomInfo.vertexCount);
  }
}

template<typename ShaderPolicy>
Camera3D& GPUManager<ShaderPolicy>::getCamera()
{
  return camera_;
}

template<typename ShaderPolicy>
void GPUManager<ShaderPolicy>::render()
{
  {
    // Update camera aspect ratio based on current window size
    int width, height;
    SDL_GetWindowSize(&window_, &width, &height);
    if (height > 0)
    {
      camera_.setAspectRatio(static_cast<float>(width) /
                             static_cast<float>(height));
    }
  }

  SDL_GPUCommandBuffer* commandBuffer{
    SDL_AcquireGPUCommandBuffer(device_.get())};

  if (!commandBuffer)
  {
    throw SDLException("Couldn't acquire command buffer");
  }

  // Get the View-Projection matrix from the camera (model is per-instance now)
  Eigen::Matrix4f viewProjectionMatrix{camera_.getMVPMatrix()};

  // Upload view-projection matrix to uniform buffer
  SDL_PushGPUVertexUniformData(
    commandBuffer, 0, viewProjectionMatrix.data(), sizeof(Eigen::Matrix4f));

  SDL_GPUTexture* swapchainTexture;
  uint32_t width, height;

  SDL_WaitAndAcquireGPUSwapchainTexture(
    commandBuffer, &window_, &swapchainTexture, &width, &height);

  if (swapchainTexture)
  {
    // Create depth texture
    SDL_GPUTextureCreateInfo depthTextureInfo = {
      .type = SDL_GPU_TEXTURETYPE_2D,
      .format = SDL_GPU_TEXTUREFORMAT_D16_UNORM,
      .usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET,
      .width = width,
      .height = height,
      .layer_count_or_depth = 1,
      .num_levels = 1,
    };
    SDL_GPUTexture* depthTexture =
      SDL_CreateGPUTexture(device_.get(), &depthTextureInfo);

    SDL_GPUDepthStencilTargetInfo depthTarget{};
    depthTarget.texture = depthTexture;
    depthTarget.clear_depth = 1.0f;  // Standard depth clear (far = 1.0)
    depthTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    depthTarget.store_op = SDL_GPU_STOREOP_DONT_CARE;
    depthTarget.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depthTarget.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    depthTarget.cycle = true;

    SDL_GPUColorTargetInfo colorTarget{};
    colorTarget.texture = swapchainTexture;
    colorTarget.store_op = SDL_GPU_STOREOP_STORE;
    colorTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    colorTarget.clear_color =
      SDL_FColor{0.0f, 0.0f, 0.5f, 1.0f};  // Blue background

    std::vector<SDL_GPUColorTargetInfo> colorTargets{colorTarget};
    SDL_GPURenderPass* renderPass{SDL_BeginGPURenderPass(
      commandBuffer, colorTargets.data(), colorTargets.size(), &depthTarget)};

    SDL_BindGPUGraphicsPipeline(renderPass, pipeline_.get());

    // Bind vertex buffer (slot 0) and instance buffer (slot 1)
    SDL_GPUBufferBinding bufferBindings[] = {
      {.buffer = vertexBuffer_.get(), .offset = 0},   // Slot 0: vertex data
      {.buffer = instanceBuffer_.get(), .offset = 0}  // Slot 1: instance data
    };
    SDL_BindGPUVertexBuffers(renderPass, 0, bufferBindings, 2);

    // Draw instances - for PositionOnly, all instances use the same geometry
    // For FullTransform, instances are sorted by geometryIndex
    if (!instances_.empty())
    {
      // For PositionOnlyShaderPolicy: no geometryIndex, all instances rendered together
      // For FullTransformShaderPolicy: instances sorted by geometryIndex
      if constexpr (std::is_same_v<ShaderPolicy, PositionOnlyShaderPolicy>)
      {
        // Simple case: all instances use geometry index 0 (pyramid)
        const auto& geomInfo = geometryRegistry_[0];
        SDL_DrawGPUPrimitives(renderPass,
                              geomInfo.vertexCount,       // Vertices
                              instances_.size(),          // Instance count
                              geomInfo.baseVertex,        // Base vertex
                              0);                         // First instance
      }
      else
      {
        // Complex case: group by geometry index
        uint32_t currentGeomIndex = instances_[0].geometryIndex;
        uint32_t rangeStart = 0;

        for (uint32_t i = 1; i <= instances_.size(); ++i)
        {
          // Check if we've reached the end or found a new geometry type
          if (i == instances_.size() ||
              instances_[i].geometryIndex != currentGeomIndex)
          {
            // Draw the current range
            uint32_t rangeCount = i - rangeStart;
            const auto& geomInfo = geometryRegistry_[currentGeomIndex];

            SDL_DrawGPUPrimitives(
              renderPass,
              geomInfo.vertexCount,  // Vertices for this geometry type
              rangeCount,            // Number of instances in this range
              geomInfo.baseVertex,   // Starting vertex in unified buffer
              rangeStart);           // First instance in this range

            // Start new range if not at end
            if (i < instances_.size())
            {
              currentGeomIndex = instances_[i].geometryIndex;
              rangeStart = i;
            }
          }
        }
      }
    }

    SDL_EndGPURenderPass(renderPass);

    SDL_ReleaseGPUTexture(device_.get(), depthTexture);
  }

  if (!SDL_SubmitGPUCommandBuffer(commandBuffer))
  {
    throw SDLException("Couldn't submit command buffer to GPU");
  }
}

template<typename ShaderPolicy>
void GPUManager<ShaderPolicy>::uploadInstanceBuffer()
{
  if (instances_.empty())
  {
    return;
  }

  SDL_GPUTransferBufferCreateInfo transferCreateInfo = {
    .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
    .size = static_cast<uint32_t>(instances_.size() * sizeof(InstanceDataType))};

  SDL_GPUTransferBuffer* transferBuffer =
    SDL_CreateGPUTransferBuffer(device_.get(), &transferCreateInfo);

  if (!transferBuffer)
  {
    throw SDLException("Failed to create instance transfer buffer!");
  }

  void* transferData =
    SDL_MapGPUTransferBuffer(device_.get(), transferBuffer, false);
  std::memcpy(transferData, instances_.data(), transferCreateInfo.size);
  SDL_UnmapGPUTransferBuffer(device_.get(), transferBuffer);

  SDL_GPUCommandBuffer* uploadCmd = SDL_AcquireGPUCommandBuffer(device_.get());
  SDL_GPUCopyPass* copyPass = SDL_BeginGPUCopyPass(uploadCmd);

  SDL_GPUTransferBufferLocation transferLocation = {
    .transfer_buffer = transferBuffer, .offset = 0};

  SDL_GPUBufferRegion bufferRegion = {.buffer = instanceBuffer_.get(),
                                      .offset = 0,
                                      .size = transferCreateInfo.size};

  SDL_UploadToGPUBuffer(copyPass, &transferLocation, &bufferRegion, false);
  SDL_EndGPUCopyPass(copyPass);
  SDL_SubmitGPUCommandBuffer(uploadCmd);

  SDL_ReleaseGPUTransferBuffer(device_.get(), transferBuffer);
}

template<typename ShaderPolicy>
uint32_t GPUManager<ShaderPolicy>::registerGeometry(
  const std::string& name,
  const std::vector<msd_assets::Vertex>& vertices)
{
  GeometryInfo info{};
  info.baseVertex = static_cast<uint32_t>(totalVertexCount_);
  info.vertexCount = static_cast<uint32_t>(vertices.size());

  uint32_t index = static_cast<uint32_t>(geometryRegistry_.size());
  geometryRegistry_.push_back(info);
  geometryNameToIndex_[name] = index;

  totalVertexCount_ += vertices.size();

  SDL_Log("Registered geometry '%s': index=%u, baseVertex=%u, vertexCount=%u",
          name.c_str(),
          index,
          info.baseVertex,
          info.vertexCount);

  return index;
}

template<typename ShaderPolicy>
typename GPUManager<ShaderPolicy>::InstanceDataType
GPUManager<ShaderPolicy>::buildInstanceData(const msd_sim::Object& object)
{
  InstanceDataType data{};

  if constexpr (std::is_same_v<ShaderPolicy, PositionOnlyShaderPolicy>)
  {
    // PositionOnly: extract position and color
    const auto& origin = object.getTransform().getOrigin();
    data.position[0] = static_cast<float>(origin.x());
    data.position[1] = static_cast<float>(origin.y());
    data.position[2] = static_cast<float>(origin.z());

    float r, g, b;
    object.getColor(r, g, b);
    data.color[0] = r;
    data.color[1] = g;
    data.color[2] = b;
  }
  else if constexpr (std::is_same_v<ShaderPolicy, FullTransformShaderPolicy>)
  {
    // FullTransform: create model matrix and extract geometry index
    // Use the shader policy's createModelMatrix method
    FullTransformShaderPolicy tempPolicy;
    auto instanceBytes = tempPolicy.buildInstanceData(object, geometryNameToIndex_);
    std::memcpy(&data, instanceBytes.data(), sizeof(InstanceDataType));
  }

  return data;
}

template<typename ShaderPolicy>
size_t GPUManager<ShaderPolicy>::addObject(const msd_sim::Object& object)
{
  InstanceDataType instance = buildInstanceData(object);
  instances_.push_back(instance);

  size_t index = objectIndices_.size();
  objectIndices_.push_back(index);  // Store object index

  uploadInstanceBuffer();

  if constexpr (std::is_same_v<ShaderPolicy, FullTransformShaderPolicy>)
  {
    SDL_Log("Added object %zu (geometry index: %u). Total objects: %zu",
            index,
            instance.geometryIndex,
            objectIndices_.size());
  }
  else
  {
    SDL_Log("Added object %zu. Total objects: %zu", index, objectIndices_.size());
  }

  return index;
}

template<typename ShaderPolicy>
void GPUManager<ShaderPolicy>::removeObject(size_t index)
{
  if (index >= objectIndices_.size())
  {
    SDL_Log("ERROR: Cannot remove object %zu, only %zu objects exist",
            index,
            objectIndices_.size());
    return;
  }

  objectIndices_.erase(objectIndices_.begin() + index);
  instances_.erase(instances_.begin() + index);

  uploadInstanceBuffer();

  SDL_Log(
    "Removed object %zu. Remaining objects: %zu", index, objectIndices_.size());
}

template<typename ShaderPolicy>
void GPUManager<ShaderPolicy>::updateObjects(
  const std::vector<msd_sim::Object>& objects)
{
  instances_.clear();
  instances_.reserve(objects.size());

  // Build instance data from objects
  for (const auto& object : objects)
  {
    if (object.hasVisualGeometry())
    {
      instances_.push_back(buildInstanceData(object));
    }
  }

  // Sort instances by geometry index for efficient rendering (FullTransform only)
  if constexpr (std::is_same_v<ShaderPolicy, FullTransformShaderPolicy>)
  {
    std::sort(instances_.begin(),
              instances_.end(),
              [](const InstanceDataType& a, const InstanceDataType& b)
              { return a.geometryIndex < b.geometryIndex; });
  }

  SDL_Log("DEBUG updateObjects: %zu objects -> %zu instances",
          objects.size(),
          instances_.size());

  uploadInstanceBuffer();
}

template<typename ShaderPolicy>
void GPUManager<ShaderPolicy>::clearObjects()
{
  instances_.clear();
  objectIndices_.clear();
  SDL_Log("Cleared all objects");
}

}  // namespace msd_gui

#endif  // SDL_GPU_MANAGER_HPP
