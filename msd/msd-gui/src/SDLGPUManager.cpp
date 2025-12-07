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

  // Create pyramid geometry using GeometryFactory
  // This creates a pyramid with base size 1.0 and height 1.0
  auto pyramidGeometry = msd_assets::GeometryFactory::createPyramid(1.0, 1.0);

  // Convert geometry to vertices with white color
  std::vector<Vertex> vertices =
    geometryToVertices(pyramidGeometry, 1.0f, 0.0f, 0.0f);

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
  // TEMPORARY: Use simple orthographic-like projection for testing
  // Just scale down by 2 to fit in NDC space
  for (int i = 0; i < 16; ++i)
  {
    transform_.mvpMatrix[i] = 0.0f;
  }
  transform_.mvpMatrix[0] = 0.5f;   // Scale X
  transform_.mvpMatrix[5] = 0.5f;   // Scale Y
  transform_.mvpMatrix[10] = 0.5f;  // Scale Z
  transform_.mvpMatrix[15] = 1.0f;  // W (homogeneous)

  SDL_Log("Using simple test matrix (scale by 0.5):");
  SDL_Log("  [%.2f %.2f %.2f %.2f]",
          transform_.mvpMatrix[0],
          transform_.mvpMatrix[1],
          transform_.mvpMatrix[2],
          transform_.mvpMatrix[3]);
  SDL_Log("  [%.2f %.2f %.2f %.2f]",
          transform_.mvpMatrix[4],
          transform_.mvpMatrix[5],
          transform_.mvpMatrix[6],
          transform_.mvpMatrix[7]);
  SDL_Log("  [%.2f %.2f %.2f %.2f]",
          transform_.mvpMatrix[8],
          transform_.mvpMatrix[9],
          transform_.mvpMatrix[10],
          transform_.mvpMatrix[11]);
  SDL_Log("  [%.2f %.2f %.2f %.2f]",
          transform_.mvpMatrix[12],
          transform_.mvpMatrix[13],
          transform_.mvpMatrix[14],
          transform_.mvpMatrix[15]);

  // TEMPORARILY COMMENT OUT THE CAMERA TRANSFORM
  updateTransformMatrix();

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

  // Define vertex input layout for 3D vertices
  SDL_GPUVertexAttribute vertexAttributes[] = {
    {.location = 0,
     .buffer_slot = 0,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // x, y, z position
     .offset = 0},
    {.location = 1,
     .buffer_slot = 0,
     .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // r, g, b color
     .offset = sizeof(float) * 3}};

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
  SDL_Log("Vertex count: %zu, Vertex size: %zu bytes",
          vertices.size(),
          sizeof(Vertex));
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
  SDL_PushGPUVertexUniformData(
    commandBuffer, 0, &transform_, sizeof(TransformData));

  static int frameCount = 0;
  if (frameCount == 0)
  {
    SDL_Log("First frame - Matrix being sent to shader:");
    SDL_Log("  [%.3f %.3f %.3f %.3f]",
            transform_.mvpMatrix[0],
            transform_.mvpMatrix[1],
            transform_.mvpMatrix[2],
            transform_.mvpMatrix[3]);
    SDL_Log("  [%.3f %.3f %.3f %.3f]",
            transform_.mvpMatrix[4],
            transform_.mvpMatrix[5],
            transform_.mvpMatrix[6],
            transform_.mvpMatrix[7]);
    SDL_Log("  [%.3f %.3f %.3f %.3f]",
            transform_.mvpMatrix[8],
            transform_.mvpMatrix[9],
            transform_.mvpMatrix[10],
            transform_.mvpMatrix[11]);
    SDL_Log("  [%.3f %.3f %.3f %.3f]",
            transform_.mvpMatrix[12],
            transform_.mvpMatrix[13],
            transform_.mvpMatrix[14],
            transform_.mvpMatrix[15]);
  }
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

    SDL_GPUBufferBinding vertexBufferBinding = {.buffer = vertexBuffer_.get(),
                                                .offset = 0};
    SDL_BindGPUVertexBuffers(renderPass, 0, &vertexBufferBinding, 1);

    // Draw all pyramid faces (18 vertices total: 4 side faces + 2 base
    // triangles)
    SDL_DrawGPUPrimitives(renderPass, 18, 1, 0, 0);

    SDL_EndGPURenderPass(renderPass);

    SDL_ReleaseGPUTexture(device_.get(), depthTexture);
  }

  if (!SDL_SubmitGPUCommandBuffer(commandBuffer))
  {
    throw SDLException("Couldn't submit command buffer to GPU");
  }
}

void GPUManager::setCameraTransform(float posX,
                                    float posY,
                                    float posZ,
                                    float rotX,
                                    float rotY,
                                    float rotZ)
{
  cameraPosX_ = posX;
  cameraPosY_ = posY;
  cameraPosZ_ = posZ;
  cameraRotX_ = rotX;
  cameraRotY_ = rotY;
  cameraRotZ_ = rotZ;
  updateTransformMatrix();
}

void GPUManager::updateTransformMatrix()
{
  // Get window dimensions for aspect ratio
  int width, height;
  SDL_GetWindowSize(&window_, &width, &height);
  float aspect = static_cast<float>(width) / static_cast<float>(height);

  static bool firstCall = true;
  if (firstCall)
  {
    SDL_Log("Initial camera: pos=(%.2f, %.2f, %.2f), rot=(%.2f, %.2f)",
            cameraPosX_,
            cameraPosY_,
            cameraPosZ_,
            cameraRotX_,
            cameraRotY_);
    firstCall = false;
  }

  // Create perspective projection matrix (column-major for Metal/HLSL)
  float fov = 60.0f * 3.14159f / 180.0f;
  float nearPlane = 0.1f;
  float farPlane = 100.0f;
  float f = 1.0f / std::tan(fov / 2.0f);

  // Projection matrix (column-major) - Fixed for reverse Z
  float proj[16] = {f / aspect,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    (nearPlane + farPlane) / (nearPlane - farPlane),
                    -1.0f,
                    0.0f,
                    0.0f,
                    (2.0f * nearPlane * farPlane) / (nearPlane - farPlane),
                    0.0f};

  // Create view matrix with full camera translation
  // In column-major format, translation is in the last row (elements 12, 13,
  // 14) We negate camera position to move world in opposite direction
  float view[16] = {1.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    1.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    1.0f,
                    0.0f,
                    -cameraPosX_,
                    -cameraPosY_,
                    -cameraPosZ_,
                    1.0f};

  // Apply rotations
  float cosX = std::cos(cameraRotX_);
  float sinX = std::sin(cameraRotX_);
  float cosY = std::cos(cameraRotY_);
  float sinY = std::sin(cameraRotY_);

  // Rotation around Y axis (yaw) - column-major
  float rotY[16] = {cosY,
                    0.0f,
                    -sinY,
                    0.0f,
                    0.0f,
                    1.0f,
                    0.0f,
                    0.0f,
                    sinY,
                    0.0f,
                    cosY,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    1.0f};

  // Rotation around X axis (pitch) - column-major
  float rotX[16] = {1.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    cosX,
                    sinX,
                    0.0f,
                    0.0f,
                    -sinX,
                    cosX,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    1.0f};

  // Matrix multiplication helper
  auto multiplyMatrices = [](const float* a, const float* b, float* result)
  {
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        result[j * 4 + i] = 0.0f;
        for (int k = 0; k < 4; ++k)
        {
          result[j * 4 + i] += a[k * 4 + i] * b[j * 4 + k];
        }
      }
    }
  };

  // MVP = Projection * View (skip rotations for now)
  multiplyMatrices(proj, view, transform_.mvpMatrix);

  if (firstCall)
  {
    SDL_Log("MVP Matrix (first 4 values): [%.3f, %.3f, %.3f, %.3f]",
            transform_.mvpMatrix[0],
            transform_.mvpMatrix[1],
            transform_.mvpMatrix[2],
            transform_.mvpMatrix[3]);
  }
}

std::vector<Vertex> GPUManager::geometryToVertices(
  const msd_assets::Geometry& geometry,
  float r,
  float g,
  float b)
{
  const auto& coords = geometry.getVertices();
  std::vector<Vertex> vertices;
  vertices.reserve(coords.size());

  for (const auto& coord : coords)
  {
    vertices.push_back({coord, r, g, b});
  }

  return vertices;
}

}  // namespace msd_gui