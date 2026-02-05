// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>

#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"


namespace msd_sim
{

AngularRate InertialState::getAngularVelocity() const
{
  return quaternionRateToOmega(quaternionRate, orientation);
}

void InertialState::setAngularVelocity(const AngularRate& omega)
{
  quaternionRate = omegaToQuaternionRate(omega, orientation);
}

Eigen::Vector4d InertialState::omegaToQuaternionRate(
  const AngularRate& omega,
  const Eigen::Quaterniond& Q)
{
  // Q̇ = ½ * Q ⊗ [0, ω]
  // Quaternion multiplication: Q ⊗ [0, ω] where [0, ω] = (0, ωx, ωy, ωz)
  //
  // Quaternion product formula:
  // (w1, x1, y1, z1) ⊗ (w2, x2, y2, z2) =
  //   ( w1*w2 - x1*x2 - y1*y2 - z1*z2,
  //     w1*x2 + x1*w2 + y1*z2 - z1*y2,
  //     w1*y2 - x1*z2 + y1*w2 + z1*x2,
  //     w1*z2 + x1*y2 - y1*x2 + z1*w2 )
  //
  // For [0, ω] = (0, ωx, ωy, ωz), w2=0:
  // Q ⊗ [0, ω] = ( -x*ωx - y*ωy - z*ωz,       [w component]
  //                 w*ωx + y*ωz - z*ωy,        [x component]
  //                 w*ωy - x*ωz + z*ωx,        [y component]
  //                 w*ωz + x*ωy - y*ωx )       [z component]
  //
  // IMPORTANT: Eigen stores quaternion coeffs as (x, y, z, w), not (w, x, y,
  // z)! So our Vector4d should store in the same order for consistency.

  double const qw = Q.w();
  double const qx = Q.x();
  double const qy = Q.y();
  double const qz = Q.z();

  double const wx = omega.x();
  double const wy = omega.y();
  double const wz = omega.z();

  // Result in Eigen's coefficient order: (x, y, z, w)
  Eigen::Vector4d qdot;
  qdot(0) = 0.5 * (qw * wx + qy * wz - qz * wy);   // x component
  qdot(1) = 0.5 * (qw * wy - qx * wz + qz * wx);   // y component
  qdot(2) = 0.5 * (qw * wz + qx * wy - qy * wx);   // z component
  qdot(3) = 0.5 * (-qx * wx - qy * wy - qz * wz);  // w component

  return qdot;
}

Eigen::Vector3d InertialState::quaternionRateToOmega(
  const Eigen::Vector4d& Qdot,
  const Eigen::Quaterniond& Q)
{
  // ω = 2 * Q̄ ⊗ Q̇
  // where Q̄ is the conjugate quaternion (w, -x, -y, -z)
  //
  // Quaternion product formula:
  // (w1, x1, y1, z1) ⊗ (w2, x2, y2, z2) =
  //   ( w1*w2 - x1*x2 - y1*y2 - z1*z2,
  //     w1*x2 + x1*w2 + y1*z2 - z1*y2,
  //     w1*y2 - x1*z2 + y1*w2 + z1*x2,
  //     w1*z2 + x1*y2 - y1*x2 + z1*w2 )
  //
  // For Q̄ = (w, -x, -y, -z), substitute x1→-x, y1→-y, z1→-z:
  // Q̄ ⊗ Q̇ = ( w*ẇ - (-x)*ẋ - (-y)*ẏ - (-z)*ż,
  //           w*ẋ + (-x)*ẇ + (-y)*ż - (-z)*ẏ,
  //           w*ẏ - (-x)*ż + (-y)*ẇ + (-z)*ẋ,
  //           w*ż + (-x)*ẏ - (-y)*ẋ + (-z)*ẇ )
  //       = ( w*ẇ + x*ẋ + y*ẏ + z*ż,     // Scalar part (should be ~0 for valid
  //       Q)
  //           w*ẋ - x*ẇ - y*ż + z*ẏ,     // ωx
  //           w*ẏ + x*ż - y*ẇ - z*ẋ,     // ωy
  //           w*ż - x*ẏ + y*ẋ - z*ẇ )    // ωz
  //
  // IMPORTANT: Eigen stores quaternion coeffs as (x, y, z, w), not (w, x, y,
  // z)! So Qdot is also stored as (x, y, z, w) for consistency.

  double const qw = Q.w();
  double const qx = Q.x();
  double const qy = Q.y();
  double const qz = Q.z();

  // Qdot in Eigen's coefficient order: (x, y, z, w)
  double const qdotX = Qdot(0);
  double const qdotY = Qdot(1);
  double const qdotZ = Qdot(2);
  double const qdotW = Qdot(3);

  // Extract vector part of Q̄ ⊗ Q̇, then multiply by 2
  double const wx = 2.0 * (qw * qdotX - qx * qdotW - qy * qdotZ + qz * qdotY);
  double const wy = 2.0 * (qw * qdotY + qx * qdotZ - qy * qdotW - qz * qdotX);
  double const wz = 2.0 * (qw * qdotZ - qx * qdotY + qy * qdotX - qz * qdotW);

  return AngularRate{wx, wy, wz};
}

AngularCoordinate InertialState::getEulerAngles() const
{
  // Extract Euler angles from quaternion using Eigen's conversion
  // Returns ZYX Euler angles (yaw-pitch-roll)
  Eigen::Matrix3d rotation = orientation.toRotationMatrix();

  // Extract ZYX Euler angles from rotation matrix
  // This is the same algorithm as ReferenceFrame::extractEulerAngles
  double pitch{};
  double roll{};
  double yaw{};

  const double sinPitch = -rotation(2, 0);

  if (std::abs(sinPitch) > 0.99999)
  {
    // Gimbal lock: pitch is ±π/2
    pitch = (sinPitch > 0) ? M_PI / 2.0 : -M_PI / 2.0;
    roll = 0.0;
    yaw = std::atan2(rotation(0, 1), rotation(1, 1));
    if (sinPitch < 0)
    {
      yaw = -yaw;
    }
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

}  // namespace msd_sim
