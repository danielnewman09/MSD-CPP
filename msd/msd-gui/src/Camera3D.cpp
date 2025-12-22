#include "msd-gui/src/Camera3D.hpp"
#include <cmath>

namespace msd_gui
{

Camera3D::Camera3D(const msd_sim::Coordinate& position,
                   float fovDegrees,
                   float aspectRatio,
                   float nearPlane,
                   float farPlane)
  : frame_(position),
    fovRadians_(fovDegrees * M_PI / 180.0f),
    aspectRatio_(aspectRatio),
    nearPlane_(nearPlane),
    farPlane_(farPlane)
{
}

void Camera3D::setAspectRatio(float aspectRatio)
{
  aspectRatio_ = aspectRatio;
}

void Camera3D::setFieldOfView(float fovDegrees)
{
  fovRadians_ = fovDegrees * M_PI / 180.0f;
}

void Camera3D::setClippingPlanes(float nearPlane, float farPlane)
{
  nearPlane_ = nearPlane;
  farPlane_ = farPlane;
}

Eigen::Matrix4f Camera3D::getViewMatrix() const
{
  // The view matrix transforms from world space to camera space
  // It's the inverse of the camera's world transform
  //
  // Camera world transform: T(origin) × R(rotation)
  // View matrix = inverse(T × R) = R^T × T^(-1)
  //
  // For a rotation matrix, inverse = transpose
  // For translation, inverse(T(p)) = T(-p)

  const Eigen::Matrix3d& rotation = frame_.getRotation();
  const msd_sim::Coordinate& origin = frame_.getOrigin();

  // Transpose the rotation (inverse for rotation matrices)
  Eigen::Matrix3f rotationTranspose = rotation.transpose().cast<float>();

  // Calculate -R^T × origin (rotated negative origin)
  Eigen::Vector3f translatedOrigin = -rotationTranspose * origin.cast<float>();

  // Build the 4x4 view matrix
  Eigen::Matrix4f viewMatrix = Eigen::Matrix4f::Identity();

  // Top-left 3x3: transposed rotation
  viewMatrix.block<3, 3>(0, 0) = rotationTranspose;

  // Top-right column: translated origin
  viewMatrix.block<3, 1>(0, 3) = translatedOrigin;

  return viewMatrix;
}

Eigen::Matrix4f Camera3D::getProjectionMatrix() const
{
  // Create a perspective projection matrix
  // This matches the standard OpenGL/Vulkan perspective projection
  //
  // Maps the view frustum to normalized device coordinates (NDC)
  // After projection, points are in clip space with w != 1
  // Perspective division (x/w, y/w, z/w) happens in hardware

  const float f = 1.0f / std::tan(fovRadians_ / 2.0f);
  const float rangeInv = 1.0f / (nearPlane_ - farPlane_);

  Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();

  // Column-major layout for HLSL/Metal/Vulkan
  projection(0, 0) = f / aspectRatio_;  // X scale (accounts for aspect ratio)
  projection(1, 1) = f;                 // Y scale (from FOV)
  projection(2, 2) =
    (farPlane_ + nearPlane_) * rangeInv;  // Z mapping (depth remapping)
  projection(2, 3) =
    2.0f * farPlane_ * nearPlane_ * rangeInv;  // Z translation (depth offset)
  projection(3, 2) = -1.0f;  // W = -Z (enables perspective division)

  return projection;
}

Eigen::Matrix4f Camera3D::getMVPMatrix(const Eigen::Matrix4f& modelMatrix) const
{
  // Combine Model, View, and Projection matrices
  // Transformations are applied right-to-left: MVP × vertex
  // So: vertex → Model → View → Projection
  return getProjectionMatrix() * getViewMatrix() * modelMatrix;
}

}  // namespace msd_gui
