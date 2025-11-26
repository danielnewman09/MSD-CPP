#ifndef REFERENCE_FRAME_HPP
#define REFERENCE_FRAME_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"

namespace msd_sim
{

/**
 * @brief A reference frame for coordinate transformations
 *
 * This class represents a reference frame that can be translated and rotated
 * relative to a global frame. It uses Eulerian angles (roll, pitch, yaw) for
 * rotation specification and provides methods to transform coordinates between
 * the global frame and the local frame.
 *
 * Rotation order: ZYX (yaw-pitch-roll) intrinsic rotations
 * - Roll: rotation around X-axis
 * - Pitch: rotation around Y-axis
 * - Yaw: rotation around Z-axis
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
   * @param euler The euler angles defining the roation
   */
  ReferenceFrame(const Coordinate& origin, const EulerAngles& euler);

  /**
   * @brief Transform a coordinate from global frame to this local frame
   * @param globalCoord Coordinate in global frame
   * @return Coordinate in this local frame
   */
  void globalToLocalInPlace(Coordinate& globalCoord) const;

  /**
   * @brief Transform a coordinate from global frame to this local frame
   * @param globalCoord Coordinate in global frame
   * @return Coordinate in this local frame
   */
  Coordinate globalToLocal(const Coordinate& globalCoord) const;

  /**
   * @brief Batch transform coordinates from global frame to this local frame
   *
   * Efficiently transforms all coordinates in a single matrix operation.
   * Each column represents a 3D coordinate.
   *
   * @param globalCoords 3xN matrix of coordinates in global frame (modified in place)
   */
  void globalToLocalBatch(Eigen::Matrix3Xd& globalCoords) const;

  /**
   * @brief Transform a coordinate from this local frame to global frame
   * @param localCoord Coordinate in this local frame
   * @return Coordinate in global frame
   */
  void localToGlobalInPlace(Coordinate& localCoord) const;

  /**
   * @brief Transform a coordinate from this local frame to global frame
   * @param localCoord Coordinate in this local frame
   * @return Coordinate in global frame
   */
  Coordinate localToGlobal(const Coordinate& localCoord) const;

  /**
   * @brief Batch transform coordinates from this local frame to global frame
   *
   * Efficiently transforms all coordinates in a single matrix operation.
   * Each column represents a 3D coordinate.
   *
   * @param localCoords 3xN matrix of coordinates in local frame (modified in place)
   */
  void localToGlobalBatch(Eigen::Matrix3Xd& localCoords) const;

  /**
   * @brief Set the origin of this frame in global coordinates
   * @param origin New origin position
   */
  void setOrigin(const Coordinate& origin);

  /**
   * @brief Set the rotation using Eulerian angles
   * @param roll Rotation around X-axis (radians)
   * @param pitch Rotation around Y-axis (radians)
   * @param yaw Rotation around Z-axis (radians)
   */
  void setRotation(const EulerAngles& euler);


  /**
   * @brief Get the origin of this frame in global coordinates
   */
  Coordinate& getOrigin()
  {
    return origin_;
  }

  /**
   * @brief Get the Eulerian angles (roll, pitch, yaw) in radians
   * @return Eigen::Vector3d with (roll, pitch, yaw)
   */
  EulerAngles& getEulerAngles();

  /**
   * @brief Get the rotation matrix
   * @return Const reference to the 3x3 rotation matrix
   */
  const Eigen::Matrix3d& getRotation() const { return rotation_; }

  /**
   * @brief Get the origin (const version)
   * @return Const reference to the origin coordinate
   */
  const Coordinate& getOrigin() const { return origin_; }

private:
  /*!
   * @brief Update rotation matrix from Eulerian angles
   */
  void updateRotationMatrix();

  Coordinate origin_;  ///< Origin of this frame in global coordinates
  EulerAngles euler_;  ///< Rotation matrix from global to local frame

  Eigen::Matrix3d rotation_;
};

}  // namespace msd_sim

#endif  // REFERENCE_FRAME_HPP