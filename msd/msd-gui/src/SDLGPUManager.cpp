#include "msd-gui/src/SDLGPUManager.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-gui/src/SDLUtils.hpp"

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
    "Position3DColorTransform.vert", *device_, basePath_, 0, 1, 0, 0);

  if (!vertexShader.get())
  {
    SDL_Log("ERROR: Failed to load Position3DColorTransform.vert shader");
    throw SDLException("Failed to load vertexShader");
  }
  SDL_Log("Successfully loaded vertex shader: Position3DColorTransform.vert");

  auto fragmentShader =
    loadShader("SolidColor.frag", *device_, basePath_, 0, 0, 0, 0);

  if (!fragmentShader.get())
  {
    SDL_Log("ERROR: Failed to load SolidColor.frag shader");
    throw SDLException("Failed to load fragmentShader");
  }
  SDL_Log("Successfully loaded fragment shader: SolidColor.frag");

  // Create base pyramid geometry (will be instanced)
  // This creates a pyramid with base size 1.0 and height 1.0
  auto pyramidGeometry = msd_assets::GeometryFactory::createPyramid(1.0, 1.0);

  // Convert geometry to vertices (color will come from instance data, so use
  // white)
  auto pyramidVertices = pyramidGeometry.toGUIVertices(1.0f, 1.0f, 1.0f);

  pyramidVertexCount_ = pyramidVertices.size();

  // Create vertex buffer for base pyramid mesh
  SDL_GPUBufferCreateInfo vertexBufferCreateInfo = {
    .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
    .size = static_cast<uint32_t>(pyramidVertices.size() *
                                  sizeof(msd_assets::Vertex))};

  vertexBuffer_ =
    UniqueBuffer(SDL_CreateGPUBuffer(device_.get(), &vertexBufferCreateInfo),
                 BufferDeleter{device_.get()});

  if (!vertexBuffer_)
  {
    throw SDLException("Failed to create vertex buffer!");
  }

  // Upload pyramid vertex data to GPU
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
    vertexTransferData, pyramidVertices.data(), vertexBufferCreateInfo.size);
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

  // Initialize instances with two pyramids (for compatibility with existing
  // demo)
  instances_.push_back(
    {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}});  // Red at origin
  instances_.push_back(
    {{2.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}});  // Green at x=2

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
    {.location = 3,
     .buffer_slot = 1,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // instance position
     .offset = 0},
    {.location = 4,
     .buffer_slot = 1,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // instance color
     .offset = sizeof(float) * 3}};

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
    .num_vertex_attributes = 5};

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
  SDL_Log("Pyramid vertex count: %zu, Initial instance count: %zu, "
          "Vertex size: %zu bytes, Instance size: %zu bytes",
          pyramidVertexCount_,
          instances_.size(),
          sizeof(msd_assets::Vertex),
          sizeof(InstanceData));
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

  // Get the MVP matrix from the camera
  Eigen::Matrix4f mvpMatrix = camera_.getMVPMatrix();

  // Upload transform data to uniform buffer
  SDL_PushGPUVertexUniformData(
    commandBuffer, 0, mvpMatrix.data(), sizeof(Eigen::Matrix4f));

  static int frameCount = 0;

  frameCount++;

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

    // Draw all instances with a single instanced draw call
    // SDL_DrawGPUPrimitives parameters: (renderPass, vertexCount,
    // instanceCount, firstVertex, firstInstance)
    if (!instances_.empty())
    {
      SDL_DrawGPUPrimitives(
        renderPass,
        static_cast<uint32_t>(
          pyramidVertexCount_),  // Number of vertices per instance
        static_cast<uint32_t>(instances_.size()),  // Number of instances
        0,                                         // First vertex
        0);                                        // First instance
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

  instanceBufferNeedsUpdate_ = false;
}

void GPUManager::addInstance(float posX,
                             float posY,
                             float posZ,
                             float r,
                             float g,
                             float b)
{
  instances_.push_back({{posX, posY, posZ}, {r, g, b}});
  uploadInstanceBuffer();
  SDL_Log("Added instance at (%.2f, %.2f, %.2f) with color (%.2f, %.2f, %.2f). "
          "Total instances: %zu",
          posX,
          posY,
          posZ,
          r,
          g,
          b,
          instances_.size());
}

void GPUManager::removeInstance(size_t index)
{
  if (index >= instances_.size())
  {
    SDL_Log("ERROR: Cannot remove instance %zu, only %zu instances exist",
            index,
            instances_.size());
    return;
  }

  instances_.erase(instances_.begin() + index);
  uploadInstanceBuffer();
  SDL_Log(
    "Removed instance %zu. Remaining instances: %zu", index, instances_.size());
}

void GPUManager::updateInstance(size_t index,
                                float posX,
                                float posY,
                                float posZ,
                                float r,
                                float g,
                                float b)
{
  if (index >= instances_.size())
  {
    SDL_Log("ERROR: Cannot update instance %zu, only %zu instances exist",
            index,
            instances_.size());
    return;
  }

  instances_[index] = {{posX, posY, posZ}, {r, g, b}};
  uploadInstanceBuffer();
  SDL_Log("Updated instance %zu to position (%.2f, %.2f, %.2f) with color "
          "(%.2f, %.2f, %.2f)",
          index,
          posX,
          posY,
          posZ,
          r,
          g,
          b);
}

void GPUManager::clearInstances()
{
  instances_.clear();
  SDL_Log("Cleared all instances");
}

}  // namespace msd_gui