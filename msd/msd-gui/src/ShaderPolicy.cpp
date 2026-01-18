// Ticket: 0002_remove_rotation_from_gpu
// Design: docs/designs/modularize-gpu-shader-system/design.md

#include "msd-gui/src/ShaderPolicy.hpp"

#include <cstring>

#include <SDL3/SDL.h>

namespace msd_gui
{

//=============================================================================
// PositionOnlyShaderPolicy Implementation
//=============================================================================

std::vector<SDL_GPUVertexAttribute>
PositionOnlyShaderPolicy::getVertexAttributes() const
{
  return {
    // Per-vertex attributes (buffer slot 0)
    SDL_GPUVertexAttribute{
      .location = 0,
      .buffer_slot = 0,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // position (x, y, z)
      .offset = 0},
    SDL_GPUVertexAttribute{
      .location = 1,
      .buffer_slot = 0,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // color (r, g, b) - unused
      .offset = sizeof(float) * 3},
    SDL_GPUVertexAttribute{
      .location = 2,
      .buffer_slot = 0,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // normal (x, y, z)
      .offset = sizeof(float) * 6},

    // Per-instance attributes (buffer slot 1)
    SDL_GPUVertexAttribute{
      .location = 3,
      .buffer_slot = 1,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // instance position
      .offset = 0},
    SDL_GPUVertexAttribute{
      .location = 4,
      .buffer_slot = 1,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // instance color
      .offset = sizeof(float) * 3}};
}

std::vector<SDL_GPUVertexBufferDescription>
PositionOnlyShaderPolicy::getVertexBufferDescriptions() const
{
  return {
    // Slot 0: Per-vertex data
    SDL_GPUVertexBufferDescription{.slot = 0,
                                   .pitch = sizeof(msd_assets::Vertex),
                                   .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX,
                                   .instance_step_rate = 0},

    // Slot 1: Per-instance data
    SDL_GPUVertexBufferDescription{
      .slot = 1,
      .pitch = sizeof(PositionOnlyInstanceData),
      .input_rate = SDL_GPU_VERTEXINPUTRATE_INSTANCE,
      .instance_step_rate = 0}};
}

void PositionOnlyShaderPolicy::initializeCache() const
{
  if (!cacheInitialized_)
  {
    cachedAttributes_ = getVertexAttributes();
    cachedBufferDescs_ = getVertexBufferDescriptions();
    cacheInitialized_ = true;
  }
}

SDL_GPUVertexInputState PositionOnlyShaderPolicy::getVertexInputState() const
{
  initializeCache();

  SDL_GPUVertexInputState state{};
  state.vertex_buffer_descriptions = cachedBufferDescs_.data();
  state.num_vertex_buffers = static_cast<uint32_t>(cachedBufferDescs_.size());
  state.vertex_attributes = cachedAttributes_.data();
  state.num_vertex_attributes = static_cast<uint32_t>(cachedAttributes_.size());

  return state;
}

std::vector<uint8_t> PositionOnlyShaderPolicy::buildInstanceData(
  const msd_sim::AssetInertial& object,
  float r,
  float b,
  float g) const
{
  PositionOnlyInstanceData data{};

  // Extract position from object transform
  const auto& origin = object.getReferenceFrame().getOrigin();
  data.position[0] = static_cast<float>(origin.x());
  data.position[1] = static_cast<float>(origin.y());
  data.position[2] = static_cast<float>(origin.z());

  data.color[0] = r;
  data.color[1] = g;
  data.color[2] = b;

  // Padding is already zero-initialized

  // Serialize to bytes
  std::vector<uint8_t> bytes(sizeof(PositionOnlyInstanceData));
  std::memcpy(bytes.data(), &data, sizeof(PositionOnlyInstanceData));

  return bytes;
}

//=============================================================================
// FullTransformShaderPolicy Implementation
//=============================================================================

std::vector<SDL_GPUVertexAttribute>
FullTransformShaderPolicy::getVertexAttributes() const
{
  return {
    // Per-vertex attributes (buffer slot 0)
    SDL_GPUVertexAttribute{
      .location = 0,
      .buffer_slot = 0,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // position (x, y, z)
      .offset = 0},
    SDL_GPUVertexAttribute{
      .location = 1,
      .buffer_slot = 0,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // color (r, g, b) - unused
      .offset = sizeof(float) * 3},
    SDL_GPUVertexAttribute{
      .location = 2,
      .buffer_slot = 0,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // normal (x, y, z)
      .offset = sizeof(float) * 6},

    // Per-instance attributes (buffer slot 1)
    // Model matrix (4x4 = 4 vec4s, occupies locations 3-6)
    SDL_GPUVertexAttribute{
      .location = 3,
      .buffer_slot = 1,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,  // model matrix row 0
      .offset = 0},
    SDL_GPUVertexAttribute{
      .location = 4,
      .buffer_slot = 1,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,  // model matrix row 1
      .offset = sizeof(float) * 4},
    SDL_GPUVertexAttribute{
      .location = 5,
      .buffer_slot = 1,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,  // model matrix row 2
      .offset = sizeof(float) * 8},
    SDL_GPUVertexAttribute{
      .location = 6,
      .buffer_slot = 1,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,  // model matrix row 3
      .offset = sizeof(float) * 12},
    SDL_GPUVertexAttribute{
      .location = 7,
      .buffer_slot = 1,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,  // instance color
      .offset = sizeof(float) * 16},
    SDL_GPUVertexAttribute{
      .location = 8,
      .buffer_slot = 1,
      .format = SDL_GPU_VERTEXELEMENTFORMAT_UINT,  // geometry index
      .offset = sizeof(float) * 19}};
}

std::vector<SDL_GPUVertexBufferDescription>
FullTransformShaderPolicy::getVertexBufferDescriptions() const
{
  return {
    // Slot 0: Per-vertex data
    SDL_GPUVertexBufferDescription{.slot = 0,
                                   .pitch = sizeof(msd_assets::Vertex),
                                   .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX,
                                   .instance_step_rate = 0},

    // Slot 1: Per-instance data
    SDL_GPUVertexBufferDescription{
      .slot = 1,
      .pitch = sizeof(FullTransformInstanceData),
      .input_rate = SDL_GPU_VERTEXINPUTRATE_INSTANCE,
      .instance_step_rate = 0}};
}

void FullTransformShaderPolicy::initializeCache() const
{
  if (!cacheInitialized_)
  {
    cachedAttributes_ = getVertexAttributes();
    cachedBufferDescs_ = getVertexBufferDescriptions();
    cacheInitialized_ = true;
  }
}

SDL_GPUVertexInputState FullTransformShaderPolicy::getVertexInputState() const
{
  initializeCache();

  SDL_GPUVertexInputState state{};
  state.vertex_buffer_descriptions = cachedBufferDescs_.data();
  state.num_vertex_buffers = static_cast<uint32_t>(cachedBufferDescs_.size());
  state.vertex_attributes = cachedAttributes_.data();
  state.num_vertex_attributes = static_cast<uint32_t>(cachedAttributes_.size());

  return state;
}

Eigen::Matrix4f FullTransformShaderPolicy::createModelMatrix(
  const msd_sim::ReferenceFrame& transform) const
{
  // Start with identity matrix
  Eigen::Matrix4f modelMatrix{Eigen::Matrix4f::Identity()};

  // Apply rotation (upper-left 3x3)
  const Eigen::Matrix3d& rotation = transform.getRotation();
  modelMatrix.block<3, 3>(0, 0) = rotation.cast<float>();

  // Get origin AFTER rotation to avoid reference invalidation from lazy
  // computation
  const auto& origin = transform.getOrigin();

  // Set translation (last column, first 3 rows)
  modelMatrix(0, 3) = static_cast<float>(origin.x());
  modelMatrix(1, 3) = static_cast<float>(origin.y());
  modelMatrix(2, 3) = static_cast<float>(origin.z());

  return modelMatrix;
}

std::vector<uint8_t> FullTransformShaderPolicy::buildInstanceData(
  const msd_sim::AssetInertial& object,
  float r,
  float b,
  float g,
  const std::unordered_map<std::string, uint32_t>& geometryNameToIndex) const
{
  FullTransformInstanceData data{};

  // Create model matrix from object's transform1
  Eigen::Matrix4f modelMatrix{createModelMatrix(object.getReferenceFrame())};

  // Copy model matrix - use Eigen's column-major data directly
  // Metal shader compiles mul(M,v) to v*M, which expects column-major layout
  // Eigen stores column-major, so copy directly
  for (int i = 0; i < 16; ++i)
  {
    data.modelMatrix[i] = modelMatrix.data()[i];
  }

  data.color[0] = r;
  data.color[1] = g;
  data.color[2] = b;

  // Look up geometry index by asset ID
  auto assetId = object.getAssetId();
  auto it = geometryNameToIndex.find(std::to_string(assetId));
  if (it != geometryNameToIndex.end())
  {
    data.geometryIndex = it->second;
  }
  else
  {
    SDL_Log("WARNING: Asset ID %u not found in registry, using index 0",
            assetId);
    data.geometryIndex = 0;
  }

  // Padding is already zero-initialized

  // Serialize to bytes
  std::vector<uint8_t> bytes(sizeof(FullTransformInstanceData));
  std::memcpy(bytes.data(), &data, sizeof(FullTransformInstanceData));

  return bytes;
}

}  // namespace msd_gui
