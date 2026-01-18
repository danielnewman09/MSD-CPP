// Ticket: 0002_remove_rotation_from_gpu
// Design: docs/designs/modularize-gpu-shader-system/design.md

#ifndef SHADER_POLICY_HPP
#define SHADER_POLICY_HPP

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <SDL3/SDL_gpu.h>
#include <Eigen/Dense>
#include <msd-assets/src/Geometry.hpp>
#include <msd-sim/src/Environment/ReferenceFrame.hpp>
#include <msd-sim/src/Physics/RigidBody/AssetInertial.hpp>

namespace msd_gui
{


/**
 * @brief Instance data for Position3DColorTransform shader (position-only
 * rendering)
 * @ticket 0002_remove_rotation_from_gpu
 *
 * Stores only position offset and color for each instance. No rotation support.
 * Size: 32 bytes (24 bytes data + 8 bytes padding for 16-byte alignment)
 */
struct PositionOnlyInstanceData
{
  float position[3];       // World position offset (12 bytes)
  float color[3];          // RGB color (12 bytes)
  uint32_t padding[2]{0};  // Padding for 16-byte alignment (8 bytes)
};

/**
 * @brief Instance data for PositionRotation3DColorTransform shader (full
 * transform rendering)
 * @ticket 0002_remove_rotation_from_gpu
 *
 * Stores full 4x4 model matrix, color, and geometry index for each instance.
 * Size: 96 bytes (80 bytes data + 16 bytes padding for 16-byte alignment)
 */
struct FullTransformInstanceData
{
  float modelMatrix[16];      // 4x4 transform matrix (64 bytes)
  float color[3];             // RGB color (12 bytes)
  uint32_t geometryIndex{0};  // Index into geometry registry (4 bytes)
  uint32_t padding[4]{0};     // Padding for 16-byte alignment (16 bytes)
};

/**
 * @brief Shader policy for Position3DColorTransform shader
 * @ticket 0002_remove_rotation_from_gpu
 * @see
 * docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml
 *
 * Provides configuration for simple position-offset rendering without rotation.
 * Uses Position3DColorTransform.vert shader which applies only translation to
 * geometry.
 *
 * Thread safety: All methods are const and stateless (thread-safe)
 */
class PositionOnlyShaderPolicy
{
public:
  using InstanceDataType = PositionOnlyInstanceData;

  static constexpr const char* kShaderName = "PositionOnly";
  static constexpr const char* kVertexShaderFile =
    "Position3DColorTransform.vert";
  static constexpr const char* kFragmentShaderFile = "SolidColor.frag";

  /**
   * @brief Get SDL GPU vertex attribute configuration
   * @return Vector of vertex attributes for pipeline creation
   *
   * Configures:
   * - Location 0-2: Per-vertex attributes (position, color, normal) from buffer
   * slot 0
   * - Location 3-4: Per-instance attributes (position, color) from buffer slot
   * 1
   */
  std::vector<SDL_GPUVertexAttribute> getVertexAttributes() const;

  /**
   * @brief Get vertex buffer descriptions for both per-vertex and per-instance
   * data
   * @return Vector of buffer descriptions (2 elements: vertex buffer, instance
   * buffer)
   */
  std::vector<SDL_GPUVertexBufferDescription> getVertexBufferDescriptions()
    const;

  /**
   * @brief Get complete vertex input state for pipeline creation
   * @return Vertex input state combining attributes and buffer descriptions
   *
   * Note: Returned structure contains pointers to internal vectors.
   * The policy object must remain alive while the returned state is in use.
   */
  SDL_GPUVertexInputState getVertexInputState() const;

  /**
   * @brief Build instance data from simulation object
   * @param object Source object containing transform and color
   * @return Serialized instance data ready for GPU upload
   *
   * Extracts position and color from object, ignoring rotation.
   */
  std::vector<uint8_t> buildInstanceData(const msd_sim::AssetInertial& object,
                                         float r,
                                         float b,
                                         float g) const;

  /**
   * @brief Get vertex shader filename
   * @return Shader filename (without path or backend extension)
   */
  std::string getVertexShaderFile() const
  {
    return kVertexShaderFile;
  }

  /**
   * @brief Get fragment shader filename
   * @return Shader filename (without path or backend extension)
   */
  std::string getFragmentShaderFile() const
  {
    return kFragmentShaderFile;
  }

  /**
   * @brief Get size of instance data structure
   * @return Size in bytes (32 bytes for PositionOnlyInstanceData)
   */
  size_t getInstanceDataSize() const
  {
    return sizeof(PositionOnlyInstanceData);
  }

private:
  // Cached attribute and buffer description vectors for getVertexInputState()
  mutable std::vector<SDL_GPUVertexAttribute> cachedAttributes_;
  mutable std::vector<SDL_GPUVertexBufferDescription> cachedBufferDescs_;
  mutable bool cacheInitialized_{false};

  void initializeCache() const;
};

/**
 * @brief Shader policy for PositionRotation3DColorTransform shader
 * @ticket 0002_remove_rotation_from_gpu
 * @see
 * docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml
 *
 * Provides configuration for full transform rendering with 4x4 model matrices.
 * Uses PositionRotation3DColorTransform.vert shader which supports translation
 * and rotation.
 *
 * Thread safety: All methods are const and stateless (thread-safe)
 */
class FullTransformShaderPolicy
{
public:
  using InstanceDataType = FullTransformInstanceData;

  static constexpr const char* kShaderName = "FullTransform";
  static constexpr const char* kVertexShaderFile =
    "PositionRotation3DColorTransform.vert";
  static constexpr const char* kFragmentShaderFile = "SolidColor.frag";

  /**
   * @brief Get SDL GPU vertex attribute configuration
   * @return Vector of vertex attributes for pipeline creation
   *
   * Configures:
   * - Location 0-2: Per-vertex attributes (position, color, normal) from buffer
   * slot 0
   * - Location 3-6: Per-instance model matrix (4x vec4) from buffer slot 1
   * - Location 7: Per-instance color from buffer slot 1
   * - Location 8: Per-instance geometry index from buffer slot 1
   */
  std::vector<SDL_GPUVertexAttribute> getVertexAttributes() const;

  /**
   * @brief Get vertex buffer descriptions for both per-vertex and per-instance
   * data
   * @return Vector of buffer descriptions (2 elements: vertex buffer, instance
   * buffer)
   */
  std::vector<SDL_GPUVertexBufferDescription> getVertexBufferDescriptions()
    const;

  /**
   * @brief Get complete vertex input state for pipeline creation
   * @return Vertex input state combining attributes and buffer descriptions
   *
   * Note: Returned structure contains pointers to internal vectors.
   * The policy object must remain alive while the returned state is in use.
   */
  SDL_GPUVertexInputState getVertexInputState() const;

  /**
   * @brief Build instance data from simulation object
   * @param object Source object containing transform, color, and geometry
   * reference
   * @param geometryNameToIndex Mapping from geometry names to registry indices
   * @return Serialized instance data ready for GPU upload
   *
   * Extracts full transform (4x4 matrix), color, and geometry index from
   * object.
   */
  std::vector<uint8_t> buildInstanceData(
    const msd_sim::AssetInertial& object,
    float r,
    float b,
    float g,
    const std::unordered_map<std::string, uint32_t>& geometryNameToIndex) const;

  /**
   * @brief Get vertex shader filename
   * @return Shader filename (without path or backend extension)
   */
  std::string getVertexShaderFile() const
  {
    return kVertexShaderFile;
  }

  /**
   * @brief Get fragment shader filename
   * @return Shader filename (without path or backend extension)
   */
  std::string getFragmentShaderFile() const
  {
    return kFragmentShaderFile;
  }

  /**
   * @brief Get size of instance data structure
   * @return Size in bytes (96 bytes for FullTransformInstanceData)
   */
  size_t getInstanceDataSize() const
  {
    return sizeof(FullTransformInstanceData);
  }

private:
  // Cached attribute and buffer description vectors for getVertexInputState()
  mutable std::vector<SDL_GPUVertexAttribute> cachedAttributes_;
  mutable std::vector<SDL_GPUVertexBufferDescription> cachedBufferDescs_;
  mutable bool cacheInitialized_{false};

  void initializeCache() const;

  /**
   * @brief Create 4x4 model matrix from reference frame
   * @param transform Reference frame containing position and orientation
   * @return Column-major 4x4 transformation matrix
   */
  Eigen::Matrix4f createModelMatrix(
    const msd_sim::ReferenceFrame& transform) const;
};

}  // namespace msd_gui

#endif  // SHADER_POLICY_HPP
