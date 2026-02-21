// Ticket: 0002_remove_rotation_from_gpu
// Ticket: 0005_camera_controller_sim
// Design: docs/designs/modularize-gpu-shader-system/design.md
// Design: docs/designs/0005_camera_controller_sim/design.md
// Previous ticket: 0001_link-gui-sim-object

#ifndef SDL_GPU_MANAGER_HPP
#define SDL_GPU_MANAGER_HPP

#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include <Eigen/Dense>

#include <msd-assets/src/Geometry.hpp>
#include <msd-gui/src/Camera3D.hpp>
#include <msd-gui/src/GPUInstanceManager.hpp>
#include <msd-gui/src/SDLUtils.hpp>
#include <msd-gui/src/ShaderPolicy.hpp>
#include <msd-sim/src/Engine.hpp>
#include <msd-sim/src/Environment/ReferenceFrame.hpp>

namespace msd_gui
{

/**
 * @brief Templated GPU manager for SDL GPU rendering with configurable shader
 * policies
 * @tparam ShaderPolicy Shader policy type (e.g., PositionOnlyShaderPolicy,
 * FullTransformShaderPolicy)
 * @ticket 0002_remove_rotation_from_gpu
 * @see
 * docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml
 *
 * Manages GPU device, pipeline, buffers, and rendering for instanced 3D
 * objects. The shader policy determines vertex attributes, instance data
 * layout, and shader files.
 *
 * Thread safety: Not thread-safe - all operations must occur on the main
 * thread.
 */
template <typename ShaderPolicy>
class GPUManager
{
public:
  /**
   * @brief Construct a GPUManager with camera reference frame
   * @param window SDL window reference (non-owning)
   * @param cameraFrame Reference frame for camera (non-owning)
   * @param basePath Base path for shader files
   *
   * NOTE: The provided ReferenceFrame must outlive this GPUManager instance.
   * Typically the ReferenceFrame is owned by a Platform's visual Object.
   *
   * @ticket 0005_camera_controller_sim
   */
  explicit GPUManager(SDL_Window& window,
                      msd_sim::ReferenceFrame& cameraFrame,
                      std::string basePath)
    : window_{window}, basePath_{std::move(basePath)}, camera_{cameraFrame}
  {
    SDL_Log("GPUManager: Starting initialization, basePath=%s",
            basePath_.c_str());

    SDL_Log("GPUManager: Creating GPU device...");
    device_.reset(SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_MSL |
                                        SDL_GPU_SHADERFORMAT_DXIL |
                                        SDL_GPU_SHADERFORMAT_SPIRV,
                                      true,
                                      nullptr));

    if (!device_.get())
    {
      SDL_Log("GPUManager: ERROR - Failed to create SDL GPU device: %s",
              SDL_GetError());
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
    SDL_Log("Using GPU device driver: %s",
            SDL_GetGPUDeviceDriver(device_.get()));

    // Load shaders via shader policy
    SDL_Log("GPUManager: Loading vertex shader: %s",
            shaderPolicy_.getVertexShaderFile().c_str());
    auto vertexShader = loadShader(
      shaderPolicy_.getVertexShaderFile(), *device_, basePath_, 0, 1, 0, 0);

    if (!vertexShader.get())
    {
      SDL_Log("ERROR: Failed to load vertex shader: %s",
              shaderPolicy_.getVertexShaderFile().c_str());
      throw SDLException("Failed to load vertexShader");
    }
    SDL_Log("Successfully loaded vertex shader: %s",
            shaderPolicy_.getVertexShaderFile().c_str());

    auto fragmentShader = loadShader(
      shaderPolicy_.getFragmentShaderFile(), *device_, basePath_, 0, 0, 0, 0);

    if (!fragmentShader.get())
    {
      SDL_Log("ERROR: Failed to load fragment shader: %s",
              shaderPolicy_.getFragmentShaderFile().c_str());
      throw SDLException("Failed to load fragmentShader");
    }
    SDL_Log("Successfully loaded fragment shader: %s",
            shaderPolicy_.getFragmentShaderFile().c_str());

    const SDL_GPUBufferCreateInfo uniformBufferCreateInfo = {
      .usage = SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ,
      .size = sizeof(Eigen::Matrix4f),
      .props = 0};

    uniformBuffer_ =
      UniqueBuffer(SDL_CreateGPUBuffer(device_.get(), &uniformBufferCreateInfo),
                   BufferDeleter{device_.get()});

    if (!uniformBuffer_)
    {
      throw SDLException("Failed to create uniform buffer!");
    }

    // Create the pipelines
    const SDL_GPUColorTargetDescription colorTargetDesc = {
      .format = SDL_GetGPUSwapchainTextureFormat(device_.get(), &window),
      .blend_state = {}};

    // Get vertex input state from shader policy
    const SDL_GPUVertexInputState vertexInputState =
      shaderPolicy_.getVertexInputState();

    SDL_GPURasterizerState rasterizerState{};
    rasterizerState.fill_mode = SDL_GPU_FILLMODE_FILL;
    rasterizerState.cull_mode = SDL_GPU_CULLMODE_NONE;  // Disable backface culling

    SDL_GPUDepthStencilState depthStencilState{};
    depthStencilState.enable_depth_test = true;
    depthStencilState.enable_depth_write = true;
    depthStencilState.compare_op = SDL_GPU_COMPAREOP_LESS;  // Standard depth test

    SDL_GPUGraphicsPipelineCreateInfo pipelineCreateInfo = {
      .vertex_shader = vertexShader.get(),
      .fragment_shader = fragmentShader.get(),
      .vertex_input_state = vertexInputState,
      .primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
      .rasterizer_state = rasterizerState,
      .multisample_state = {},
      .depth_stencil_state = depthStencilState,
      .target_info =
        {
          .color_target_descriptions = &colorTargetDesc,
          .num_color_targets = 1,
          .depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D16_UNORM,
          .has_depth_stencil_target = true,
        },
      .props = 0,
    };

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
  }

  void updateGeometryBuffer()
  {
    // Create unified vertex buffer
    const SDL_GPUBufferCreateInfo vertexBufferCreateInfo = {
      .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
      .size = static_cast<uint32_t>(allVertices_.size() *
                                    sizeof(msd_assets::Vertex)),
      .props = 0};

    vertexBuffer_ =
      UniqueBuffer(SDL_CreateGPUBuffer(device_.get(), &vertexBufferCreateInfo),
                   BufferDeleter{device_.get()});

    if (!vertexBuffer_)
    {
      throw SDLException("Failed to create vertex buffer!");
    }

    // Upload unified vertex data to GPU
    const SDL_GPUTransferBufferCreateInfo vertexTransferCreateInfo = {
      .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
      .size = vertexBufferCreateInfo.size,
      .props = 0};

    SDL_GPUTransferBuffer* vertexTransferBuffer =
      SDL_CreateGPUTransferBuffer(device_.get(), &vertexTransferCreateInfo);

    if (vertexTransferBuffer == nullptr)
    {
      throw SDLException("Failed to create vertex transfer buffer!");
    }

    void* vertexTransferData =
      SDL_MapGPUTransferBuffer(device_.get(), vertexTransferBuffer, false);
    std::memcpy(
      vertexTransferData, allVertices_.data(), vertexBufferCreateInfo.size);
    SDL_UnmapGPUTransferBuffer(device_.get(), vertexTransferBuffer);

    SDL_GPUCommandBuffer* vertexUploadCmd =
      SDL_AcquireGPUCommandBuffer(device_.get());
    SDL_GPUCopyPass* vertexCopyPass = SDL_BeginGPUCopyPass(vertexUploadCmd);

    SDL_GPUTransferBufferLocation const vertexTransferLocation = {
      .transfer_buffer = vertexTransferBuffer, .offset = 0};

    const SDL_GPUBufferRegion vertexBufferRegion = {
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
    const SDL_GPUBufferCreateInfo instanceBufferCreateInfo = {
      .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
      .size = static_cast<uint32_t>(maxInstances *
                                    shaderPolicy_.getInstanceDataSize()),
      .props = 0};

    instanceBuffer_ = UniqueBuffer(
      SDL_CreateGPUBuffer(device_.get(), &instanceBufferCreateInfo),
      BufferDeleter{device_.get()});

    if (!instanceBuffer_)
    {
      throw SDLException("Failed to create instance buffer!");
    }
  }

  uint32_t registerGeometry(uint32_t assetId,
                            const std::vector<msd_assets::Vertex>& vertices)
  {
    GeometryInfo info{};
    info.baseVertex = static_cast<uint32_t>(totalVertexCount_);
    info.vertexCount = static_cast<uint32_t>(vertices.size());

    auto index = static_cast<uint32_t>(geometryRegistry_.size());
    geometryRegistry_.push_back(info);
    assetIdToGeometryIndex_[assetId] = index;

    totalVertexCount_ += vertices.size();

    SDL_Log("Registered geometry for asset ID %u: index=%u, baseVertex=%u, "
            "vertexCount=%u",
            assetId,
            index,
            info.baseVertex,
            info.vertexCount);

    // Append all vertices
    allVertices_.insert(allVertices_.end(), vertices.begin(), vertices.end());

    // Upload vertices to GPU and create/resize buffers
    updateGeometryBuffer();

    return index;
  }

  std::unordered_map<uint32_t, uint32_t> getGeometryIdMap()
  {
    return assetIdToGeometryIndex_;
  }

  void addObject(const msd_sim::AssetInertial& object,
                 float r,
                 float g,
                 float b)
  {
    uint32_t geometryId = 0;
    auto it = assetIdToGeometryIndex_.find(object.getAssetId());
    if (it != assetIdToGeometryIndex_.end())
    {
      geometryId = it->second;
    }
    else
    {
      throw SDLException("Couldn't map object to geometry id");
    }

    instanceManager_.addObject(
      getDevice(), getInstanceBuffer(), object, geometryId, r, g, b);
  }

  ~GPUManager() = default;

  // Delete copy constructor and assignment operator
  GPUManager(const GPUManager&) = delete;
  GPUManager& operator=(const GPUManager&) = delete;

  // Delete move constructor and assignment operator
  GPUManager(GPUManager&&) = delete;
  GPUManager& operator=(GPUManager&&) = delete;

  InstanceManager<ShaderPolicy>& getInstanceManager()
  {
    return instanceManager_;
  }

  Camera3D& getCamera()
  {
    return camera_;
  }

  SDL_GPUDevice& getDevice()
  {
    return *device_.get();
  }

  SDL_GPUBuffer& getInstanceBuffer()
  {
    return *instanceBuffer_.get();
  }

  void update(const msd_sim::Engine& engine)
  {
    instanceManager_.update(engine);
    instanceManager_.uploadInstanceBuffer(getDevice(), getInstanceBuffer());
    render();
  }

  void render()
  {
    {
      // Update camera aspect ratio based on current window size
      int width = 0;
      int height = 0;
      SDL_GetWindowSize(&window_, &width, &height);
      if (height > 0)
      {
        camera_.setAspectRatio(static_cast<float>(width) /
                               static_cast<float>(height));
      }
    }

    SDL_GPUCommandBuffer* commandBuffer{
      SDL_AcquireGPUCommandBuffer(device_.get())};

    if (commandBuffer == nullptr)
    {
      throw SDLException("Couldn't acquire command buffer");
    }

    // Get the View-Projection matrix from the camera (model is per-instance
    // now)
    Eigen::Matrix4f viewProjectionMatrix{camera_.getMVPMatrix()};

    // Upload view-projection matrix to uniform buffer
    SDL_PushGPUVertexUniformData(
      commandBuffer, 0, viewProjectionMatrix.data(), sizeof(Eigen::Matrix4f));

    SDL_GPUTexture* swapchainTexture = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;

    SDL_WaitAndAcquireGPUSwapchainTexture(
      commandBuffer, &window_, &swapchainTexture, &width, &height);

    if (swapchainTexture != nullptr)
    {
      // Create depth texture
      const SDL_GPUTextureCreateInfo depthTextureInfo = {
        .type = SDL_GPU_TEXTURETYPE_2D,
        .format = SDL_GPU_TEXTUREFORMAT_D16_UNORM,
        .usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET,
        .width = width,
        .height = height,
        .layer_count_or_depth = 1,
        .num_levels = 1,
        .sample_count = SDL_GPU_SAMPLECOUNT_1,
        .props = 0,
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
      SDL_GPURenderPass* renderPass{
        SDL_BeginGPURenderPass(commandBuffer,
                               colorTargets.data(),
                               static_cast<uint32_t>(colorTargets.size()),
                               &depthTarget)};

      SDL_BindGPUGraphicsPipeline(renderPass, pipeline_.get());

      // Only bind and draw if we have geometry registered
      if (vertexBuffer_ && instanceBuffer_)
      {
        // Bind vertex buffer (slot 0) and instance buffer (slot 1)
        SDL_GPUBufferBinding bufferBindings[] = {
          {.buffer = vertexBuffer_.get(), .offset = 0},  // Slot 0: vertex data
          {.buffer = instanceBuffer_.get(), .offset = 0}
          // Slot 1: instance data
        };
        SDL_BindGPUVertexBuffers(renderPass, 0, bufferBindings, 2);

        const auto& instances = instanceManager_.getInstances();

        // Draw instances - for PositionOnly, all instances use the same
        // geometry For FullTransform, instances are sorted by geometryIndex
        if (!instances.empty())
        {
          // Complex case: group by geometry index
          uint32_t currentGeomIndex = instances[0].geometryIndex;
          uint32_t rangeStart = 0;

          for (uint32_t i = 1; i <= instances.size(); ++i)
          {
            // Check if we've reached the end or found a new geometry type
            if (i == instances.size() ||
                instances[i].geometryIndex != currentGeomIndex)
            {
              // Draw the current range
              uint32_t const rangeCount = i - rangeStart;
              const auto& geomInfo = geometryRegistry_[currentGeomIndex];

              SDL_DrawGPUPrimitives(
                renderPass,
                geomInfo.vertexCount,  // Vertices for this geometry type
                rangeCount,            // Number of instances in this range
                geomInfo.baseVertex,   // Starting vertex in unified buffer
                rangeStart);           // First instance in this range

              // Start new range if not at end
              if (i < instances.size())
              {
                currentGeomIndex = instances[i].geometryIndex;
                rangeStart = i;
              }
            }
          }
        }
      }  // end if (vertexBuffer_ && instanceBuffer_)

      SDL_EndGPURenderPass(renderPass);

      SDL_ReleaseGPUTexture(device_.get(), depthTexture);
    }

    if (!SDL_SubmitGPUCommandBuffer(commandBuffer))
    {
      throw SDLException("Couldn't submit command buffer to GPU");
    }
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

  using InstanceDataType = typename ShaderPolicy::InstanceDataType;

  size_t totalVertexCount_{0};
  std::vector<msd_assets::Vertex> allVertices_;

  // Geometry registry for unified vertex buffer
  std::vector<GeometryInfo> geometryRegistry_;

  // Maps the:
  // assetId - which is the unique identifier for the geometry asset
  //           as loaded from the database
  // to the:
  // GeometryIndex - which is the position in the geometryRegistry_
  //                 vector for the loaded geometry object in the gui
  //                 application.
  //
  // This might seem a bit weird, but because of the nature of the
  // graphical object loading and management, we have the lightweight
  // GeometryInfo object keeping track of the geometry assets in this
  // manager. Instead of duplicating geometry information or having
  // complex reference behavior, we copy the asset id when we register
  // the geometric object with the Manager.
  std::unordered_map<uint32_t, uint32_t> assetIdToGeometryIndex_;

  InstanceManager<ShaderPolicy> instanceManager_;
  ShaderPolicy shaderPolicy_;

  std::unique_ptr<SDL_GPUDevice, SDLDeviceDeleter> device_;
  std::string basePath_;

  Camera3D camera_;
};

}  // namespace msd_gui

#endif  // SDL_GPU_MANAGER_HPP
