#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include <Eigen/Dense>

namespace msd_sim
{

/**
 * @brief A 3D coordinate class that inherits from Eigen::Vector3f
 *
 * This class provides a convenient wrapper around Eigen::Vector3f with
 * additional constructors and methods specific to coordinate operations.
 * Uses float precision which is sufficient for rendering and most simulations.
 */
class Coordinate : public Eigen::Vector3f
{
public:
  // Default constructor - initializes to (0, 0, 0)
  Coordinate() : Eigen::Vector3f{0.0f, 0.0f, 0.0f}
  {
  }

  // Constructor with x, y, z values
  Coordinate(float x, float y, float z) : Eigen::Vector3f{x, y, z}
  {
  }

  // Constructor from Eigen::Vector3f
  Coordinate(const Eigen::Vector3f& vec) : Eigen::Vector3f{vec}
  {
  }

  // This constructor allows you to construct Coordinate from Eigen expressions
  template <typename OtherDerived>
  Coordinate(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3f{other}
  {
  }

  // This method allows you to assign Eigen expressions to Coordinate
  template <typename OtherDerived>
  Coordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3f::operator=(other);
    return *this;
  }
};

}  // namespace msd_sim

#endif  // COORDINATE_HPP