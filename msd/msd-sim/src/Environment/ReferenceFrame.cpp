#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include <cmath>

namespace msd_sim
{

ReferenceFrame::ReferenceFrame()
  : origin_{0.0, 0.0, 0.0}, euler_{}, rotation_{Eigen::Matrix3d::Identity()}
{
}

ReferenceFrame::ReferenceFrame(const Coordinate& origin)
  : origin_{origin}, euler_{}, rotation_{Eigen::Matrix3d::Identity()}
{
}

ReferenceFrame::ReferenceFrame(const Coordinate& origin,
                               const EulerAngles& euler)
  : origin_{origin}, euler_{euler}, rotation_{Eigen::Matrix3d::Identity()}
{
  updateRotationMatrix();
}

Coordinate ReferenceFrame::globalToLocal(const Coordinate& globalCoord) const
{
  // Translate to frame origin, then rotate to local orientation
  Coordinate translated = globalCoord - origin_;
  return rotation_ * translated;
}

Coordinate ReferenceFrame::localToGlobal(const Coordinate& localCoord) const
{
  // Rotate to global orientation, then translate to global position
  Coordinate rotated = rotation_.transpose() * localCoord;
  return rotated + origin_;
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
  Eigen::AngleAxisd rollAngle{euler_.roll.getRad(), Eigen::Vector3d::UnitX()};
  Eigen::AngleAxisd pitchAngle{euler_.pitch.getRad(), Eigen::Vector3d::UnitY()};
  Eigen::AngleAxisd yawAngle{euler_.yaw.getRad(), Eigen::Vector3d::UnitZ()};

  // Combine rotations: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  rotation_ = q.matrix();
}

}  // namespace msd_sim
