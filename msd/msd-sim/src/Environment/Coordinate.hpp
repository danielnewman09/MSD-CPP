#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include <Eigen/Dense>

namespace msd_sim
{

/**
 * @brief A 3D coordinate class that inherits from Eigen::Vector3d
 *
 * This class provides a convenient wrapper around Eigen::Vector3d with
 * additional constructors and methods specific to coordinate operations.
 */
class Coordinate : public Eigen::Vector3d
{
public:
  // Default constructor - initializes to (0, 0, 0)
  Coordinate() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  // Constructor with x, y, z values
  Coordinate(double x, double y, double z) : Eigen::Vector3d{x, y, z}
  {
  }

  // Constructor from Eigen::Vector3d
  Coordinate(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec}
  {
  }

  // This constructor allows you to construct Coordinate from Eigen expressions
  template <typename OtherDerived>
  Coordinate(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
  }

  // This method allows you to assign Eigen expressions to Coordinate
  template <typename OtherDerived>
  Coordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    return *this;
  }
};

}  // namespace msd_sim

#endif  // COORDINATE_HPP