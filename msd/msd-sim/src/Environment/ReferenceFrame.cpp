// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include <cmath>

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

Coordinate ReferenceFrame::globalToLocal(
  const CoordinateRate& globalVector) const
{
  // Apply only rotation (transpose for inverse), no translation
  return rotation_.transpose() * globalVector;
}

Coordinate ReferenceFrame::localToGlobal(
  const CoordinateRate& localVector) const
{
  // Apply only rotation, no translation
  return rotation_ * localVector;
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

AngularCoordinate& ReferenceFrame::getAngularCoordinate()
{
  updated_ = false;
  return angular_;
}

const AngularCoordinate& ReferenceFrame::getAngularCoordinate() const
{
  return angular_;
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
