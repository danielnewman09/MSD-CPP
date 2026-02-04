// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#ifndef REFERENCE_FRAME_HPP
#define REFERENCE_FRAME_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"

namespace msd_sim
{

/**
 * @brief A reference frame for coordinate transformations
 *
 * This class represents a reference frame that can be translated and rotated
 * relative to a global frame. It uses AngularCoordinate (pitch, roll, yaw) for
 * rotation specification and provides methods to transform coordinates between
 * the global frame and the local frame.
 *
 * Rotation order: ZYX (yaw-pitch-roll) intrinsic rotations
 * - Roll: rotation around X-axis (component 1)
 * - Pitch: rotation around Y-axis (component 0)
 * - Yaw: rotation around Z-axis (component 2)
 *
 * @see docs/designs/0024_angular_coordinate/0024_angular_coordinate.puml
 * @ticket 0024_angular_coordinate
 */
class ReferenceFrame
{
public:
  /**
   * @brief Default constructor - creates identity frame at origin
   */
  ReferenceFrame();

  /**
   * @brief Constructor with translation only
   * @param origin The origin of this frame in global coordinates
   */
  explicit ReferenceFrame(const Coordinate& origin);

  /**
   * @brief Constructor with translation and rotation
   * @param origin The origin of this frame in global coordinates
   * @param angular The angular coordinate defining the rotation
   */
  ReferenceFrame(const Coordinate& origin, const AngularCoordinate& angular);

  /**
   * @brief Constructor from local X and Z axes (right-hand rule)
   *
   * Constructs a reference frame from two vectors representing the local X-axis
   * (tangent direction) and Z-axis (normal direction). The Y-axis is computed
   * using the cross product Y = Z × X to follow the right-hand rule.
   *
   * The input vectors do not need to be normalized or perfectly orthogonal -
   * they will be processed as follows:
   * 1. Z is normalized (takes precedence as the primary/normal direction)
   * 2. X is orthogonalized against Z and normalized
   * 3. Y is computed as Z × X
   *
   * This is useful for constructing a local coordinate frame based on a surface
   * normal (Z) and tangent direction (X), common in collision response.
   *
   * @param origin The origin of this frame in global coordinates
   * @param xDirection Vector representing the local positive X-axis direction
   * @param zDirection Vector representing the local positive Z-axis direction
   * @throws std::invalid_argument if either vector is zero or if vectors are
   *         parallel
   */
  ReferenceFrame(const Coordinate& origin,
                 const Coordinate& xDirection,
                 const Coordinate& zDirection);

  /**
   * @brief Constructor with origin and quaternion orientation
   *
   * Constructs a reference frame using a quaternion to specify orientation.
   * The quaternion will be normalized internally for robustness.
   *
   * @param origin The origin of this frame in global coordinates
   * @param quaternion The orientation as a unit quaternion (will be normalized)
   */
  ReferenceFrame(const Coordinate& origin,
                 const Eigen::Quaterniond& quaternion);

  /**
   * @brief Transform a coordinate from global frame to this local frame
   * @param globalCoord Coordinate in global frame
   * @return Coordinate in this local frame
   */
  void globalToLocalInPlace(Coordinate& globalCoord) const;

  /**
   * @brief Batch transform coordinates from global frame to this local frame
   *
   * Efficiently transforms all coordinates in a single matrix operation.
   * Each column represents a 3D coordinate.
   *
   * @param globalCoords 3xN matrix of coordinates in global frame (modified in
   * place)
   */
  void globalToLocalBatch(Eigen::Matrix3Xd& globalCoords) const;

  /**
   * @brief Transform a coordinate from this local frame to global frame
   * @param localCoord Coordinate in this local frame
   * @return Coordinate in global frame
   */
  void localToGlobalInPlace(Coordinate& localCoord) const;

  /**
   * @brief Batch transform coordinates from this local frame to global frame
   *
   * Efficiently transforms all coordinates in a single matrix operation.
   * Each column represents a 3D coordinate.
   *
   * @param localCoords 3xN matrix of coordinates in local frame (modified in
   * place)
   */
  void localToGlobalBatch(Eigen::Matrix3Xd& localCoords) const;

  /**
   * @brief Transform a direction vector from global frame to local frame
   * (relative transformation)
   *
   * This applies only rotation, not translation. Use this for transforming
   * direction vectors, velocities, or any vector that represents a direction
   * rather than a position.
   *
   * @param globalVector Direction vector in global frame
   * @return Direction vector in local frame
   */
  Eigen::Vector3d globalToLocal(const Eigen::Vector3d& globalVector) const;

  /**
   * @brief Transform a direction vector from global frame to local frame
   * (relative transformation)
   *
   * This applies only rotation, not translation. Use this for transforming
   * direction vectors, velocities, or any vector that represents a direction
   * rather than a position.
   *
   * @param globalVector Direction vector in global frame
   * @return Direction vector in local frame
   */
  AngularRate globalToLocal(const AngularRate& globalVector) const;

  /**
   * @brief Transform a direction vector from local frame to global frame
   * (relative transformation)
   *
   * This applies only rotation, not translation. Use this for transforming
   * direction vectors, velocities, or any vector that represents a direction
   * rather than a position.
   *
   * @param localVector Direction vector in local frame
   * @return Direction vector in global frame
   */
  Eigen::Vector3d localToGlobal(const Eigen::Vector3d& localVector) const;

  /**
   * @brief Transform a direction vector from local frame to global frame
   * (relative transformation)
   *
   * This applies only rotation, not translation. Use this for transforming
   * direction vectors, velocities, or any vector that represents a direction
   * rather than a position.
   *
   * @param localVector Direction vector in local frame
   * @return Direction vector in global frame
   */
  AngularRate localToGlobal(const AngularRate& localVector) const;

  /**
   * @brief Transform a point from global frame to local frame (absolute
   * transformation)
   *
   * This applies both rotation and translation. Use this for transforming
   * positions/points. This is an alias for globalToLocal() for clarity.
   *
   * @param globalPoint Point in global frame
   * @return Point in local frame
   */
  Coordinate globalToLocal(const Coordinate& globalPoint) const;

  /**
   * @brief Transform a point from local frame to global frame (absolute
   * transformation)
   *
   * This applies both rotation and translation. Use this for transforming
   * positions/points. This is an alias for localToGlobal() for clarity.
   *
   * @param localPoint Point in local frame
   * @return Point in global frame
   */
  Coordinate localToGlobal(const Coordinate& localPoint) const;

  /**
   * @brief Set the origin of this frame in global coordinates
   * @param origin New origin position
   */
  void setOrigin(const Coordinate& origin);

  /**
   * @brief Set the rotation using AngularCoordinate
   * @param angular Orientation angles (pitch, roll, yaw) in radians
   * @deprecated Use setQuaternion() for gimbal-lock-free orientation
   */
  [[deprecated("Use setQuaternion() for gimbal-lock-free orientation")]]
  void setRotation(const AngularCoordinate& angular);

  /**
   * @brief Set the orientation using a quaternion
   * @param quaternion The orientation as a unit quaternion (will be normalized)
   *
   * @ticket 0030_lagrangian_quaternion_physics
   */
  void setQuaternion(const Eigen::Quaterniond& quaternion);

  /**
   * @brief Get the origin of this frame in global coordinates
   */
  Coordinate& getOrigin()
  {
    return origin_;
  }

  /**
   * @brief Get the orientation as AngularCoordinate
   * @return AngularCoordinate representing current rotation
   */
  AngularCoordinate& getAngularCoordinate();

  /**
   * @brief Get the orientation as AngularCoordinate (const version)
   * @return Const reference to AngularCoordinate representing current rotation
   */
  const AngularCoordinate& getAngularCoordinate() const;

  /**
   * @brief Get the orientation as a quaternion
   * @return Unit quaternion representing the frame's orientation
   */
  [[nodiscard]] Eigen::Quaterniond getQuaternion() const;

  /**
   * @brief Get the rotation matrix
   * @return Const reference to the 3x3 rotation matrix
   */
  const Eigen::Matrix3d& getRotation() const
  {
    if (!updated_)
    {
      updateRotationMatrix();
    }
    return rotation_;
  }

  /**
   * @brief Get the origin (const version)
   * @return Const reference to the origin coordinate
   */
  const Coordinate& getOrigin() const
  {
    return origin_;
  }

private:
  /*!
   * @brief Update rotation matrix from AngularCoordinate
   */
  void updateRotationMatrix() const;

  /**
   * @brief Extract Euler angles (ZYX convention) from a rotation matrix
   *
   * Extracts pitch, roll, yaw angles from a rotation matrix using the
   * ZYX intrinsic rotation convention (R = Rz(yaw) * Ry(pitch) * Rx(roll)).
   * Handles gimbal lock at pitch = ±π/2.
   *
   * @param rotation The 3x3 rotation matrix to decompose
   * @return AngularCoordinate containing (pitch, roll, yaw) in radians
   */
  static AngularCoordinate extractEulerAngles(const Eigen::Matrix3d& rotation);

  Coordinate origin_;          ///< Origin of this frame in global coordinates
  AngularCoordinate angular_;  ///< Orientation angles (pitch, roll, yaw)

  mutable Eigen::Matrix3d rotation_;

  mutable bool updated_;
};

}  // namespace msd_sim

#endif  // REFERENCE_FRAME_HPP