#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include <cmath>

namespace msd_sim
{

ReferenceFrame::ReferenceFrame()
  : origin_{0.0f, 0.0f, 0.0f}, euler_{}, rotation_{Eigen::Matrix3f::Identity()}
{
}

ReferenceFrame::ReferenceFrame(const Coordinate& origin)
  : origin_{origin}, euler_{}, rotation_{Eigen::Matrix3f::Identity()}
{
}

ReferenceFrame::ReferenceFrame(const Coordinate& origin,
                               const EulerAngles& euler)
  : origin_{origin}, euler_{euler}, rotation_{Eigen::Matrix3f::Identity()}
{
  updateRotationMatrix();
}

void ReferenceFrame::globalToLocalInPlace(Coordinate& globalCoord) const
{
  // Translate to frame origin, then rotate to local orientation
  globalCoord -= origin_;
  globalCoord.applyOnTheLeft(rotation_.transpose());
}

Coordinate ReferenceFrame::globalToLocal(const Coordinate& globalCoord) const
{
  // Translate to frame origin, then rotate to local orientation
  Coordinate translated = globalCoord - origin_;
  return rotation_.transpose() * translated;
}

void ReferenceFrame::globalToLocalBatch(Eigen::Matrix3Xf& globalCoords) const
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

Coordinate ReferenceFrame::localToGlobal(const Coordinate& localCoord) const
{
  // Rotate to global orientation, then translate to global position
  Coordinate rotated = rotation_ * localCoord;
  return rotated + origin_;
}

void ReferenceFrame::localToGlobalBatch(Eigen::Matrix3Xf& localCoords) const
{
  // Rotate all coordinates to global orientation in one matrix multiply
  localCoords.applyOnTheLeft(rotation_);
  // Translate all coordinates to global position
  localCoords.colwise() += origin_;
}

void ReferenceFrame::setOrigin(const Coordinate& origin)
{
  origin_ = origin;
}

void ReferenceFrame::setRotation(const EulerAngles& euler)
{
  euler_ = euler;
  updateRotationMatrix();
}

EulerAngles& ReferenceFrame::getEulerAngles()
{
  return euler_;
}

void ReferenceFrame::updateRotationMatrix()
{
  // Create rotation matrix using ZYX Euler angle convention
  // Using Eigen's AngleAxis for clarity and efficiency
  Eigen::AngleAxisf rollAngle{static_cast<float>(euler_.roll.getRad()), Eigen::Vector3f::UnitX()};
  Eigen::AngleAxisf pitchAngle{static_cast<float>(euler_.pitch.getRad()), Eigen::Vector3f::UnitY()};
  Eigen::AngleAxisf yawAngle{static_cast<float>(euler_.yaw.getRad()), Eigen::Vector3f::UnitZ()};

  // Combine rotations: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
  rotation_ = q.matrix();
}

}  // namespace msd_sim
