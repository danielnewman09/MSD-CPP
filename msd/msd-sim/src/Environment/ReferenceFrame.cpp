// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include <cmath>
#include <stdexcept>

namespace msd_sim
{

ReferenceFrame::ReferenceFrame()
  : origin_{0.0, 0.0, 0.0},
    angular_{},
    rotation_{Eigen::Matrix3d::Identity()},
    updated_{false}
{
  updateRotationMatrix();
}

ReferenceFrame::ReferenceFrame(const Coordinate& origin)
  : origin_{origin},
    angular_{},
    rotation_{Eigen::Matrix3d::Identity()},
    updated_{false}
{
  updateRotationMatrix();
}

ReferenceFrame::ReferenceFrame(const Coordinate& origin,
                               const AngularCoordinate& angular)
  : origin_{origin},
    angular_{angular},
    rotation_{Eigen::Matrix3d::Identity()},
    updated_{false}
{
  updateRotationMatrix();
}

ReferenceFrame::ReferenceFrame(const Coordinate& origin,
                               const Eigen::Quaterniond& quaternion)
  : origin_{origin},
    angular_{},
    rotation_{Eigen::Matrix3d::Identity()},
    updated_{false}
{
  // Normalize the quaternion for robustness and convert to rotation matrix
  rotation_ = quaternion.normalized().toRotationMatrix();

  // Extract Euler angles for consistency with class design
  angular_ = extractEulerAngles(rotation_);
  updated_ = true;
}

ReferenceFrame::ReferenceFrame(const Coordinate& origin,
                               const Coordinate& xDirection,
                               const Coordinate& zDirection)
  : origin_{origin},
    angular_{},
    rotation_{Eigen::Matrix3d::Identity()},
    updated_{false}
{
  // Validate input vectors are non-zero
  const double xNorm = xDirection.norm();
  const double zNorm = zDirection.norm();

  if (xNorm < 1e-10)
  {
    throw std::invalid_argument(
      "ReferenceFrame: xDirection vector has zero magnitude");
  }
  if (zNorm < 1e-10)
  {
    throw std::invalid_argument(
      "ReferenceFrame: zDirection vector has zero magnitude");
  }

  // Normalize Z first (primary direction / normal)
  Eigen::Vector3d zAxis = zDirection.normalized();

  // Orthogonalize X against Z: remove component of X parallel to Z
  Eigen::Vector3d xProjection = xDirection - xDirection.dot(zAxis) * zAxis;
  const double xProjectionNorm = xProjection.norm();

  if (xProjectionNorm < 1e-10)
  {
    throw std::invalid_argument(
      "ReferenceFrame: xDirection and zDirection are parallel");
  }

  Eigen::Vector3d xAxis = xProjection.normalized();

  // Compute Y using right-hand rule: Y = Z × X
  Eigen::Vector3d yAxis = zAxis.cross(xAxis);

  // Build rotation matrix from unit vectors (columns are local axes in world
  // coords)
  rotation_.col(0) = xAxis;
  rotation_.col(1) = yAxis;
  rotation_.col(2) = zAxis;

  // Extract Euler angles for consistency with class design
  angular_ = extractEulerAngles(rotation_);
  updated_ = true;
}

AngularCoordinate ReferenceFrame::extractEulerAngles(
  const Eigen::Matrix3d& rotation)
{
  // Extract ZYX Euler angles from rotation matrix
  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  //
  // For a ZYX rotation matrix:
  // R = | cy*cp        cy*sp*sr - sy*cr    cy*sp*cr + sy*sr |
  //     | sy*cp        sy*sp*sr + cy*cr    sy*sp*cr - cy*sr |
  //     | -sp          cp*sr               cp*cr            |
  //
  // Where: cy = cos(yaw), sy = sin(yaw), cp = cos(pitch), sp = sin(pitch),
  //        cr = cos(roll), sr = sin(roll)

  double pitch{};
  double roll{};
  double yaw{};

  // Check for gimbal lock (pitch = ±π/2, where cos(pitch) ≈ 0)
  const double sinPitch = -rotation(2, 0);

  if (std::abs(sinPitch) > 0.99999)
  {
    // Gimbal lock: pitch is ±π/2
    // In this case, roll and yaw become coupled - we can only determine their
    // sum/difference. Set roll to 0 and absorb everything into yaw.
    //
    // At pitch = +π/2: R(0,1) = sin(roll - yaw), R(1,1) = cos(roll - yaw)
    //   atan2(R(0,1), R(1,1)) = roll - yaw → with roll=0, yaw = -atan2(...)
    // At pitch = -π/2: R(0,1) = -sin(roll + yaw), R(1,1) = cos(roll + yaw)
    //   atan2(R(0,1), R(1,1)) = -(roll + yaw) → with roll=0, yaw = -atan2(...)
    pitch = (sinPitch > 0) ? M_PI / 2.0 : -M_PI / 2.0;
    roll = 0.0;
    yaw = -std::atan2(rotation(0, 1), rotation(1, 1));
  }
  else
  {
    // Normal case: extract all three angles
    pitch = std::asin(sinPitch);
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  }

  return AngularCoordinate{pitch, roll, yaw};
}

void ReferenceFrame::globalToLocalInPlace(Coordinate& globalCoord) const
{
  // Translate to frame origin, then rotate to local orientation
  globalCoord -= origin_;
  globalCoord.applyOnTheLeft(rotation_.transpose());
}

void ReferenceFrame::globalToLocalBatch(Eigen::Matrix3Xd& globalCoords) const
{
  // Translate all coordinates to frame origin
  globalCoords.colwise() -= origin_;
  // Rotate all coordinates to local orientation in one matrix multiply
  globalCoords.applyOnTheLeft(rotation_.transpose());
}

void ReferenceFrame::localToGlobalInPlace(Coordinate& localCoord) const
{
  // Rotate to global orientation, then translate to global position
  localCoord.applyOnTheLeft(rotation_);
  localCoord += origin_;
}

void ReferenceFrame::localToGlobalBatch(Eigen::Matrix3Xd& localCoords) const
{
  // Rotate all coordinates to global orientation in one matrix multiply
  localCoords.applyOnTheLeft(rotation_);
  // Translate all coordinates to global position
  localCoords.colwise() += origin_;
}

Eigen::Vector3d ReferenceFrame::globalToLocal(
  const Eigen::Vector3d& globalVector) const
{
  // Apply only rotation (transpose for inverse), no translation
  return rotation_.transpose() * globalVector;
}

Eigen::Vector3d ReferenceFrame::localToGlobal(
  const Eigen::Vector3d& localVector) const
{
  // Apply only rotation, no translation
  return rotation_ * localVector;
}


AngularRate ReferenceFrame::localToGlobal(const AngularRate& localVector) const
{
  // Apply only rotation, no translation
  return AngularRate{rotation_ * localVector};
}


Coordinate ReferenceFrame::globalToLocal(const Coordinate& globalPoint) const
{
  // Translate to frame origin, then rotate to local orientation
  Coordinate translated = globalPoint - origin_;
  return rotation_.transpose() * translated;
}

Coordinate ReferenceFrame::localToGlobal(const Coordinate& localPoint) const
{
  // Rotate to global orientation, then translate to global position
  Coordinate rotated = rotation_ * localPoint;
  return rotated + origin_;
}

void ReferenceFrame::setOrigin(const Coordinate& origin)
{
  origin_ = origin;
}

void ReferenceFrame::setRotation(const AngularCoordinate& angular)
{
  angular_ = angular;
  updateRotationMatrix();
}

void ReferenceFrame::setQuaternion(const Eigen::Quaterniond& quaternion)
{
  // Normalize for robustness and convert to rotation matrix
  rotation_ = quaternion.normalized().toRotationMatrix();

  // Extract Euler angles for consistency with class design
  angular_ = extractEulerAngles(rotation_);
  updated_ = true;
  // Ticket: 0030_lagrangian_quaternion_physics
}

AngularCoordinate& ReferenceFrame::getAngularCoordinate()
{
  updated_ = false;
  return angular_;
}

const AngularCoordinate& ReferenceFrame::getAngularCoordinate() const
{
  return angular_;
}

Eigen::Quaterniond ReferenceFrame::getQuaternion() const
{
  if (!updated_)
  {
    updateRotationMatrix();
  }
  return Eigen::Quaterniond{rotation_};
}

void ReferenceFrame::updateRotationMatrix() const
{
  // Create rotation matrix using ZYX Euler angle convention
  // Using Eigen's AngleAxis for clarity and efficiency
  Eigen::AngleAxisd rollAngle{angular_.roll(), Eigen::Vector3d::UnitX()};
  Eigen::AngleAxisd pitchAngle{angular_.pitch(), Eigen::Vector3d::UnitY()};
  Eigen::AngleAxisd yawAngle{angular_.yaw(), Eigen::Vector3d::UnitZ()};

  // Combine rotations: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  rotation_ = q.matrix();
  updated_ = true;
}

}  // namespace msd_sim
