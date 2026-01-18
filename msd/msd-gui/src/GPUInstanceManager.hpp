#ifndef GPU_INSTANCE_MANAGER_HPP
#define GPU_INSTANCE_MANAGER_HPP

#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>

#include "msd-assets/src/Geometry.hpp"
#include "msd-gui/src/SDLUtils.hpp"
#include "msd-gui/src/ShaderPolicy.hpp"
#include "msd-sim/src/Engine.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"

namespace msd_gui
{
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

template <typename ShaderPolicy>
class InstanceManager
{
public:
  using InstanceDataType = typename ShaderPolicy::InstanceDataType;

  InstanceManager() = default;

  void clearObjects()
  {
    instances_.clear();
    indexMap_.clear();
    SDL_Log("Cleared all objects");
  }


  void removeObject(uint32_t instanceId)
  {
    auto it = indexMap_.find(instanceId);
    if (it == indexMap_.end())
    {
      SDL_Log("ERROR: Cannot remove object with instanceId %u, not found",
              instanceId);
      return;
    }

    size_t removedIdx = it->second;
    indexMap_.erase(it);
    instances_.erase(instances_.begin() + static_cast<ptrdiff_t>(removedIdx));

    SDL_Log("Removed object with instanceId %u. Remaining objects: %zu",
            instanceId,
            instances_.size());
  }


  size_t buildInstanceData(const msd_sim::AssetInertial& object,
                           uint32_t geometryId,
                           float r,
                           float g,
                           float b)
  {
    InstanceDataType data{};

    if constexpr (std::is_same_v<ShaderPolicy, PositionOnlyShaderPolicy>)
    {
      // PositionOnly: extract position and color
      const auto& origin = object.getReferenceFrame().getOrigin();
      data.position[0] = static_cast<float>(origin.x());
      data.position[1] = static_cast<float>(origin.y());
      data.position[2] = static_cast<float>(origin.z());

      data.color[0] = r;
      data.color[1] = g;
      data.color[2] = b;
    }
    else if constexpr (std::is_same_v<ShaderPolicy, FullTransformShaderPolicy>)
    {
      // FullTransform: create model matrix and extract geometry index
      // Use the shader policy's createModelMatrix method
      FullTransformShaderPolicy tempPolicy;
      auto instanceBytes =
        tempPolicy.buildInstanceData(object, r, g, b, geometryId);
      std::memcpy(&data, instanceBytes.data(), sizeof(InstanceDataType));
    }

    auto currentIdx = instances_.size();
    instances_.emplace_back(data);
    indexMap_.emplace(object.getInstanceId(), currentIdx);


    if constexpr (std::is_same_v<ShaderPolicy, FullTransformShaderPolicy>)
    {
      SDL_Log("Added object (geometry index: %u). Total objects: %zu",
              data.geometryIndex,
              instances_.size());
    }
    else
    {
      SDL_Log(
        "Added object %zu. Total objects: %zu", currentIdx, instances_.size());
    }

    return currentIdx;
  }

  size_t addObject(SDL_GPUDevice& device,
                   SDL_GPUBuffer& instanceBuffer,
                   const msd_sim::AssetInertial& object,
                   uint32_t geometryId,
                   float r,
                   float g,
                   float b)
  {
    auto index = buildInstanceData(object, geometryId, r, g, b);

    uploadInstanceBuffer(device, instanceBuffer);

    return index;
  }

  void update(const msd_sim::Engine& engine)
  {
    const auto& wm = engine.getWorldModel();

    for (const auto& [instanceId, idx] : indexMap_)
    {
      const auto& object = wm.getObject(instanceId);
      const auto& frame = object.getReferenceFrame();
      auto& data = instances_[idx];

      if constexpr (std::is_same_v<ShaderPolicy, PositionOnlyShaderPolicy>)
      {
        const auto& origin = frame.getOrigin();
        data.position[0] = static_cast<float>(origin.x());
        data.position[1] = static_cast<float>(origin.y());
        data.position[2] = static_cast<float>(origin.z());
      }
      else if constexpr (std::is_same_v<ShaderPolicy,
                                        FullTransformShaderPolicy>)
      {
        // Build model matrix from reference frame
        const auto& rotation = frame.getRotation();
        const auto& origin = frame.getOrigin();

        // Column-major 4x4 matrix: [R | t]
        //                          [0 | 1]
        // Column 0
        data.modelMatrix[0] = static_cast<float>(rotation(0, 0));
        data.modelMatrix[1] = static_cast<float>(rotation(1, 0));
        data.modelMatrix[2] = static_cast<float>(rotation(2, 0));
        data.modelMatrix[3] = 0.0f;
        // Column 1
        data.modelMatrix[4] = static_cast<float>(rotation(0, 1));
        data.modelMatrix[5] = static_cast<float>(rotation(1, 1));
        data.modelMatrix[6] = static_cast<float>(rotation(2, 1));
        data.modelMatrix[7] = 0.0f;
        // Column 2
        data.modelMatrix[8] = static_cast<float>(rotation(0, 2));
        data.modelMatrix[9] = static_cast<float>(rotation(1, 2));
        data.modelMatrix[10] = static_cast<float>(rotation(2, 2));
        data.modelMatrix[11] = 0.0f;
        // Column 3 (translation)
        data.modelMatrix[12] = static_cast<float>(origin.x());
        data.modelMatrix[13] = static_cast<float>(origin.y());
        data.modelMatrix[14] = static_cast<float>(origin.z());
        data.modelMatrix[15] = 1.0f;
      }
    }
  }

  const std::vector<InstanceDataType>& getInstances() const
  {
    return instances_;
  }

  void uploadInstanceBuffer(SDL_GPUDevice& device,
                            SDL_GPUBuffer& instanceBuffer)
  {
    if (instances_.empty())
    {
      return;
    }

    SDL_GPUTransferBufferCreateInfo transferCreateInfo = {
      .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
      .size =
        static_cast<uint32_t>(instances_.size() * sizeof(InstanceDataType))};

    SDL_GPUTransferBuffer* transferBuffer =
      SDL_CreateGPUTransferBuffer(&device, &transferCreateInfo);

    if (!transferBuffer)
    {
      throw SDLException("Failed to create instance transfer buffer!");
    }

    void* transferData =
      SDL_MapGPUTransferBuffer(&device, transferBuffer, false);
    std::memcpy(transferData, instances_.data(), transferCreateInfo.size);
    SDL_UnmapGPUTransferBuffer(&device, transferBuffer);

    SDL_GPUCommandBuffer* uploadCmd = SDL_AcquireGPUCommandBuffer(&device);
    SDL_GPUCopyPass* copyPass = SDL_BeginGPUCopyPass(uploadCmd);

    SDL_GPUTransferBufferLocation transferLocation = {
      .transfer_buffer = transferBuffer, .offset = 0};

    SDL_GPUBufferRegion bufferRegion = {
      .buffer = &instanceBuffer, .offset = 0, .size = transferCreateInfo.size};

    SDL_UploadToGPUBuffer(copyPass, &transferLocation, &bufferRegion, false);
    SDL_EndGPUCopyPass(copyPass);
    SDL_SubmitGPUCommandBuffer(uploadCmd);

    SDL_ReleaseGPUTransferBuffer(&device, transferBuffer);
  }

private:
  ShaderPolicy shaderPolicy_;  // Shader policy instance

  // Stores the unique instance data for each object to be rendered
  std::vector<InstanceDataType> instances_;

  // Maps the index identifier from the simulation backend
  // to the index identifier for the gui
  std::unordered_map<uint32_t, size_t> indexMap_;
};

}  // namespace msd_gui

#endif