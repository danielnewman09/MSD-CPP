#include "msd-gui/src/SDLGPUManager.hpp"
#include "msd-gui/src/SDLUtils.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <SDL3/SDL.h>

namespace msd_gui
{

GPUManager::GPUManager(SDL_Window& window, const std::string& basePath)
  : window_{window}, basePath_{basePath}
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

  auto vertexShader =
    loadShader("PositionColorOffset.vert", *device_, basePath_, 0, 1, 0, 0);

  if (!vertexShader.get())
  {
    throw SDLException("Failed to load vertexShader");
  }

  auto fragmentShader =
    loadShader("SolidColor.frag", *device_, basePath_, 0, 0, 0, 0);

  if (!fragmentShader.get())
  {
    throw SDLException("Failed to load fragmentShader");
  }

  // Create vertex buffer with customizable triangle vertices
  std::vector<Vertex> vertices = {
    {-0.25f, -0.5f, 1.0f, 0.0f, 0.0f},  // Bottom-left: Red
    {0.25f, -0.5f, 0.0f, 1.0f, 0.0f},   // Bottom-right: Green
    {0.0f, 0.5f, 0.0f, 0.0f, 1.0f}      // Top-center: Blue
  };

  SDL_GPUBufferCreateInfo bufferCreateInfo = {
    .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
    .size = static_cast<uint32_t>(vertices.size() * sizeof(Vertex))};

  vertexBuffer_ =
    UniqueBuffer(SDL_CreateGPUBuffer(device_.get(), &bufferCreateInfo),
                 BufferDeleter{device_.get()});

  if (!vertexBuffer_)
  {
    throw SDLException("Failed to create vertex buffer!");
  }

  // Upload vertex data to GPU
  SDL_GPUTransferBufferCreateInfo transferBufferCreateInfo = {
    .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, .size = bufferCreateInfo.size};

  SDL_GPUTransferBuffer* transferBuffer =
    SDL_CreateGPUTransferBuffer(device_.get(), &transferBufferCreateInfo);

  if (!transferBuffer)
  {
    throw SDLException("Failed to create transfer buffer!");
  }

  void* transferData =
    SDL_MapGPUTransferBuffer(device_.get(), transferBuffer, false);
  std::memcpy(transferData, vertices.data(), bufferCreateInfo.size);
  SDL_UnmapGPUTransferBuffer(device_.get(), transferBuffer);

  SDL_GPUCommandBuffer* uploadCmdBuffer =
    SDL_AcquireGPUCommandBuffer(device_.get());
  SDL_GPUCopyPass* copyPass = SDL_BeginGPUCopyPass(uploadCmdBuffer);

  SDL_GPUTransferBufferLocation transferBufferLocation = {
    .transfer_buffer = transferBuffer, .offset = 0};

  SDL_GPUBufferRegion bufferRegion = {
    .buffer = vertexBuffer_.get(), .offset = 0, .size = bufferCreateInfo.size};

  SDL_UploadToGPUBuffer(
    copyPass, &transferBufferLocation, &bufferRegion, false);
  SDL_EndGPUCopyPass(copyPass);
  SDL_SubmitGPUCommandBuffer(uploadCmdBuffer);

  SDL_ReleaseGPUTransferBuffer(device_.get(), transferBuffer);

  // Create uniform buffer for transform
  transform_ = {0.0f, 0.0f, {0.0f, 0.0f}};  // Initialize at origin

  SDL_GPUBufferCreateInfo uniformBufferCreateInfo = {
    .usage = SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ,
    .size = sizeof(TransformData)};

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

  // Define vertex input layout
  SDL_GPUVertexAttribute vertexAttributes[] = {
    {.location = 0,
     .buffer_slot = 0,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2,  // x, y position
     .offset = 0},
    {.location = 1,
     .buffer_slot = 0,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // r, g, b color
     .offset = sizeof(float) * 2}};

  SDL_GPUVertexBufferDescription vertexBufferDesc = {
    .slot = 0,
    .pitch = sizeof(Vertex),
    .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX,
    .instance_step_rate = 0};

  SDL_GPUVertexInputState vertexInputState = {
    .vertex_buffer_descriptions = &vertexBufferDesc,
    .num_vertex_buffers = 1,
    .vertex_attributes = vertexAttributes,
    .num_vertex_attributes = 2};

  SDL_GPUGraphicsPipelineCreateInfo pipelineCreateInfo = {
    .vertex_input_state = vertexInputState,
    .target_info =
      {
        .num_color_targets = 1,
        .color_target_descriptions = &colorTargetDesc,
      },
    .primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
    .vertex_shader = vertexShader.get(),
    .fragment_shader = fragmentShader.get(),
  };
  pipelineCreateInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;

  pipeline_ = UniquePipeline(
    SDL_CreateGPUGraphicsPipeline(device_.get(), &pipelineCreateInfo),
    PipelineDeleter{device_.get()});

  if (!pipeline_)
  {
    throw SDLException("Failed to create line pipeline!");
  }
}

void GPUManager::render()
{
  SDL_GPUCommandBuffer* commandBuffer{
    SDL_AcquireGPUCommandBuffer(device_.get())};

  if (!commandBuffer)
  {
    throw SDLException("Couldn't acquire command buffer");
  }

  // Upload transform data to uniform buffer
  SDL_PushGPUVertexUniformData(commandBuffer, 0, &transform_, sizeof(TransformData));

  SDL_GPUTexture* swapchainTexture;

  SDL_WaitAndAcquireGPUSwapchainTexture(
    commandBuffer, &window_, &swapchainTexture, nullptr, nullptr);

  if (swapchainTexture)
  {
    SDL_GPUColorTargetInfo colorTarget{};
    colorTarget.texture = swapchainTexture;
    colorTarget.store_op = SDL_GPU_STOREOP_STORE;
    colorTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    colorTarget.clear_color = SDL_FColor{1.0f, 0.0f, 0.0f, 1.0f};
    std::vector<SDL_GPUColorTargetInfo> colorTargets{colorTarget};
    SDL_GPURenderPass* renderPass{SDL_BeginGPURenderPass(
      commandBuffer, colorTargets.data(), colorTargets.size(), NULL)};

    SDL_BindGPUGraphicsPipeline(renderPass, pipeline_.get());

    SDL_GPUBufferBinding vertexBufferBinding = {.buffer = vertexBuffer_.get(),
                                                .offset = 0};
    SDL_BindGPUVertexBuffers(renderPass, 0, &vertexBufferBinding, 1);

    SDL_DrawGPUPrimitives(renderPass, 3, 1, 0, 0);
    SDL_EndGPURenderPass(renderPass);
  }

  if (!SDL_SubmitGPUCommandBuffer(commandBuffer))
  {
    throw SDLException("Couldn't submit command buffer to GPU");
  }
}

void GPUManager::setPosition(float x, float y)
{
  transform_.offsetX = x;
  transform_.offsetY = y;
}


}  // namespace msd_gui