// Ticket: 0001_link-gui-sim-object
// Design: docs/designs/generalize-gui-object-rendering/design.md

#include "msd-gui/src/SDLGPUManager.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-gui/src/SDLUtils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <SDL3/SDL.h>

namespace msd_gui
{

GPUManager::GPUManager(SDL_Window& window, const std::string& basePath)
  : window_{window},
    basePath_{basePath},
    camera_{msd_sim::Coordinate{0., 0., 5.}}
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

  auto vertexShader = loadShader(
    "PositionRotation3DColorTransform.vert", *device_, basePath_, 0, 1, 0, 0);

  if (!vertexShader.get())
  {
    SDL_Log(
      "ERROR: Failed to load PositionRotation3DColorTransform.vert shader");
    throw SDLException("Failed to load vertexShader");
  }
  SDL_Log(
    "Successfully loaded vertex shader: PositionRotation3DColorTransform.vert");

  auto fragmentShader =
    loadShader("SolidColor.frag", *device_, basePath_, 0, 0, 0, 0);

  if (!fragmentShader.get())
  {
    SDL_Log("ERROR: Failed to load SolidColor.frag shader");
    throw SDLException("Failed to load fragmentShader");
  }
  SDL_Log("Successfully loaded fragment shader: SolidColor.frag");

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
    .size = static_cast<uint32_t>(maxInstances * sizeof(InstanceData))};

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

  // Define vertex input layout for 3D vertices and instanced data
  SDL_GPUVertexAttribute vertexAttributes[] = {
    // Per-vertex attributes (buffer slot 0)
    {.location = 0,
     .buffer_slot = 0,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // position (x, y, z)
     .offset = 0},
    {.location = 1,
     .buffer_slot = 0,
     .format =
       SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // color (r, g, b) - unused but kept
                                            // for compatibility
     .offset = sizeof(float) * 3},
    {.location = 2,
     .buffer_slot = 0,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // normal (x, y, z)
     .offset = sizeof(float) * 6},
    // Per-instance attributes (buffer slot 1)
    // Model matrix (4x4 = 4 vec4s, occupies locations 3-6)
    {.location = 3,
     .buffer_slot = 1,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,  // model matrix row 0
     .offset = 0},
    {.location = 4,
     .buffer_slot = 1,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,  // model matrix row 1
     .offset = sizeof(float) * 4},
    {.location = 5,
     .buffer_slot = 1,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,  // model matrix row 2
     .offset = sizeof(float) * 8},
    {.location = 6,
     .buffer_slot = 1,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,  // model matrix row 3
     .offset = sizeof(float) * 12},
    {.location = 7,
     .buffer_slot = 1,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // instance color
     .offset = sizeof(float) * 16},
    {.location = 8,
     .buffer_slot = 1,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_UINT,  // geometry index
     .offset = sizeof(float) * 19}};

  SDL_GPUVertexBufferDescription bufferDescriptions[] = {
    // Slot 0: Per-vertex data
    {.slot = 0,
     .pitch = sizeof(msd_assets::Vertex),
     .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX,
     .instance_step_rate = 0},
    // Slot 1: Per-instance data
    {.slot = 1,
     .pitch = sizeof(InstanceData),
     .input_rate = SDL_GPU_VERTEXINPUTRATE_INSTANCE,
     .instance_step_rate = 0}};

  SDL_GPUVertexInputState vertexInputState = {
    .vertex_buffer_descriptions = bufferDescriptions,
    .num_vertex_buffers = 2,
    .vertex_attributes = vertexAttributes,
    .num_vertex_attributes = 9};

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
  SDL_Log("Successfully created graphics pipeline");
  SDL_Log("Total vertex count: %zu, Geometry types registered: %zu, "
          "Vertex size: %zu bytes, Instance size: %zu bytes",
          totalVertexCount_,
          geometryRegistry_.size(),
          sizeof(msd_assets::Vertex),
          sizeof(InstanceData));
  for (const auto& [name, index] : geometryNameToIndex_)
  {
    const auto& geomInfo = geometryRegistry_[index];
    SDL_Log("  Geometry '%s': baseVertex=%u, vertexCount=%u",
            name.c_str(),
            geomInfo.baseVertex,
            geomInfo.vertexCount);
  }
}


Camera3D& GPUManager::getCamera()
{
  return camera_;
}

void GPUManager::render()
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
  Eigen::Matrix4f viewProjectionMatrix = camera_.getMVPMatrix();

  // Debug: log the VP matrix periodically
  static int vpLogCounter = 0;
  if (vpLogCounter++ % 120 == 0)
  {
    SDL_Log("DEBUG VP matrix row0: [%.2f, %.2f, %.2f, %.2f]",
            viewProjectionMatrix(0, 0),
            viewProjectionMatrix(0, 1),
            viewProjectionMatrix(0, 2),
            viewProjectionMatrix(0, 3));
    SDL_Log("DEBUG VP matrix row3: [%.2f, %.2f, %.2f, %.2f]",
            viewProjectionMatrix(3, 0),
            viewProjectionMatrix(3, 1),
            viewProjectionMatrix(3, 2),
            viewProjectionMatrix(3, 3));
  }

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

    // Draw instances grouped by geometry type
    // Instances are sorted by geometryIndex in updateObjects(), so we can make
    // efficient contiguous draw calls
    static int frameCounter = 0;
    frameCounter++;
    if (!instances_.empty())
    {
      if (frameCounter % 60 == 0)
      {
        SDL_Log("DEBUG render: %zu instances to draw", instances_.size());
      }

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

          if (frameCounter % 60 == 0)
          {
            SDL_Log("DEBUG draw: geomIndex=%u, vertexCount=%u, "
                    "instanceCount=%u, baseVertex=%u, firstInstance=%u",
                    currentGeomIndex,
                    geomInfo.vertexCount,
                    rangeCount,
                    geomInfo.baseVertex,
                    rangeStart);
          }

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

    SDL_EndGPURenderPass(renderPass);

    SDL_ReleaseGPUTexture(device_.get(), depthTexture);
  }

  if (!SDL_SubmitGPUCommandBuffer(commandBuffer))
  {
    throw SDLException("Couldn't submit command buffer to GPU");
  }
}


void GPUManager::uploadInstanceBuffer()
{
  if (instances_.empty())
  {
    return;
  }

  SDL_GPUTransferBufferCreateInfo transferCreateInfo = {
    .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
    .size = static_cast<uint32_t>(instances_.size() * sizeof(InstanceData))};

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

uint32_t GPUManager::registerGeometry(
  const std::string& name,
  const std::vector<msd_assets::Vertex>& vertices)
{
  GeometryInfo info;
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

Eigen::Matrix4f GPUManager::createModelMatrix(
  const msd_sim::ReferenceFrame& transform)
{
  // Start with identity matrix
  Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();

  // TEMPORARY: Use identity rotation to debug visibility issue
  // const Eigen::Matrix3d& rotation = transform.getRotation();
  // modelMatrix.block<3, 3>(0, 0) = rotation.cast<float>();
  // Identity rotation is already set by Matrix4f::Identity()

  // Get origin AFTER rotation to avoid reference invalidation from lazy
  // computation
  const auto& origin = transform.getOrigin();

  // Set translation (last column, first 3 rows)
  modelMatrix(0, 3) = static_cast<float>(origin.x());
  modelMatrix(1, 3) = static_cast<float>(origin.y());
  modelMatrix(2, 3) = static_cast<float>(origin.z());

  // Debug: log the full model matrix
  SDL_Log("DEBUG modelMatrix row0: [%.2f, %.2f, %.2f, %.2f]",
          modelMatrix(0, 0),
          modelMatrix(0, 1),
          modelMatrix(0, 2),
          modelMatrix(0, 3));
  SDL_Log("DEBUG modelMatrix row1: [%.2f, %.2f, %.2f, %.2f]",
          modelMatrix(1, 0),
          modelMatrix(1, 1),
          modelMatrix(1, 2),
          modelMatrix(1, 3));
  SDL_Log("DEBUG modelMatrix row2: [%.2f, %.2f, %.2f, %.2f]",
          modelMatrix(2, 0),
          modelMatrix(2, 1),
          modelMatrix(2, 2),
          modelMatrix(2, 3));
  SDL_Log("DEBUG modelMatrix row3: [%.2f, %.2f, %.2f, %.2f]",
          modelMatrix(3, 0),
          modelMatrix(3, 1),
          modelMatrix(3, 2),
          modelMatrix(3, 3));

  return modelMatrix;
}

InstanceData GPUManager::buildInstanceData(const msd_sim::Object& object)
{
  InstanceData data{};

  // Create model matrix from object's transform
  Eigen::Matrix4f modelMatrix = createModelMatrix(object.getTransform());

  // Copy model matrix - use Eigen's column-major data directly
  // Metal shader compiles mul(M,v) to v*M, which expects column-major layout
  // Eigen stores column-major, so copy directly
  for (int i = 0; i < 16; ++i)
  {
    data.modelMatrix[i] = modelMatrix.data()[i];
  }

  // Debug: log the model matrix translation (column-major: Tx at [12], Ty at
  // [13], Tz at [14])
  SDL_Log("DEBUG: Model matrix translation: (%.2f, %.2f, %.2f)",
          data.modelMatrix[12],
          data.modelMatrix[13],
          data.modelMatrix[14]);

  // Get object color
  float r, g, b;
  object.getColor(r, g, b);
  data.color[0] = r;
  data.color[1] = g;
  data.color[2] = b;

  // Get geometry index from asset name
  if (object.hasVisualGeometry())
  {
    auto assetOpt = object.getAsset();
    if (assetOpt.has_value())
    {
      const auto& asset = assetOpt->get();
      const std::string& geometryName = asset.getName();

      auto it = geometryNameToIndex_.find(geometryName);
      if (it != geometryNameToIndex_.end())
      {
        data.geometryIndex = it->second;
      }
      else
      {
        SDL_Log("WARNING: Geometry '%s' not found in registry, using index 0",
                geometryName.c_str());
        data.geometryIndex = 0;
      }
    }
    else
    {
      SDL_Log(
        "WARNING: Object has visual geometry but no asset, using index 0");
      data.geometryIndex = 0;
    }
  }
  else
  {
    SDL_Log("WARNING: Object has no visual geometry, using index 0");
    data.geometryIndex = 0;
  }

  return data;
}

size_t GPUManager::addObject(const msd_sim::Object& object)
{
  InstanceData instance = buildInstanceData(object);
  instances_.push_back(instance);

  size_t index = objectIndices_.size();
  objectIndices_.push_back(index);  // Store object index

  uploadInstanceBuffer();

  SDL_Log("Added object %zu (geometry index: %u). Total objects: %zu",
          index,
          instance.geometryIndex,
          objectIndices_.size());

  return index;
}

void GPUManager::removeObject(size_t index)
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

void GPUManager::updateObjects(const std::vector<msd_sim::Object>& objects)
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
    else
    {
      SDL_Log("DEBUG: Object does NOT have visual geometry!");
    }
  }

  // Sort instances by geometry index for efficient rendering
  // This allows us to make fewer draw calls (one per geometry type)
  std::sort(instances_.begin(),
            instances_.end(),
            [](const InstanceData& a, const InstanceData& b)
            { return a.geometryIndex < b.geometryIndex; });

  SDL_Log("DEBUG updateObjects: %zu objects -> %zu instances",
          objects.size(),
          instances_.size());

  uploadInstanceBuffer();
}

void GPUManager::clearObjects()
{
  instances_.clear();
  objectIndices_.clear();
  SDL_Log("Cleared all objects");
}

}  // namespace msd_gui