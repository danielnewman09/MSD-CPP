// Ticket: 0026_mirtich_inertia_tensor
// Design: docs/designs/0026_mirtich_inertia_tensor/design.md

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialCalculations.hpp"

using namespace msd_sim;

namespace
{

/**
 * @brief Create unit cube centered at origin with side length 1.
 * @ticket 0026_mirtich_inertia_tensor
 */
std::vector<Coordinate> createUnitCubePoints()
{
  return {Coordinate{-0.5, -0.5, -0.5},
          Coordinate{0.5, -0.5, -0.5},
          Coordinate{0.5, 0.5, -0.5},
          Coordinate{-0.5, 0.5, -0.5},
          Coordinate{-0.5, -0.5, 0.5},
          Coordinate{0.5, -0.5, 0.5},
          Coordinate{0.5, 0.5, 0.5},
          Coordinate{-0.5, 0.5, 0.5}};
}

/**
 * @brief Create rectangular box centered at origin.
 * @param a Width (x dimension)
 * @param b Height (y dimension)
 * @param c Depth (z dimension)
 * @ticket 0026_mirtich_inertia_tensor
 */
std::vector<Coordinate> createRectangularBoxPoints(double a, double b, double c)
{
  double hx = a / 2.0;
  double hy = b / 2.0;
  double hz = c / 2.0;

  return {Coordinate{-hx, -hy, -hz},
          Coordinate{hx, -hy, -hz},
          Coordinate{hx, hy, -hz},
          Coordinate{-hx, hy, -hz},
          Coordinate{-hx, -hy, hz},
          Coordinate{hx, -hy, hz},
          Coordinate{hx, hy, hz},
          Coordinate{-hx, hy, hz}};
}

/**
 * @brief Create regular tetrahedron centered at origin.
 * @param edgeLength Length of each edge
 * @ticket 0026_mirtich_inertia_tensor
 */
std::vector<Coordinate> createRegularTetrahedronPoints(double edgeLength)
{
  // Regular tetrahedron with vertices at:
  // v0 = (1, 1, 1), v1 = (1, -1, -1), v2 = (-1, 1, -1), v3 = (-1, -1, 1)
  // These are equidistant from origin and form regular tetrahedron
  double scale = edgeLength / (2.0 * std::sqrt(2.0));

  return {Coordinate{scale, scale, scale},
          Coordinate{scale, -scale, -scale},
          Coordinate{-scale, scale, -scale},
          Coordinate{-scale, -scale, scale}};
}

}  // anonymous namespace

TEST(InertialCalculationsTest, UnitCubeAnalytical)
{
  // Arrange: Unit cube (side = 1, density = 1, centered at origin)
  auto points = createUnitCubePoints();
  ConvexHull hull{points};
  double density = 1.0;

  // Analytical solution for unit cube:
  // Ixx = Iyy = Izz = m/6 ≈ 0.166666666667
  // Ixy = Iyz = Izx = 0
  double expected_diagonal = density / 6.0;

  // Act
  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density);

  // Assert: Diagonal elements
  EXPECT_NEAR(I(0, 0), expected_diagonal, 1e-10);
  EXPECT_NEAR(I(1, 1), expected_diagonal, 1e-10);
  EXPECT_NEAR(I(2, 2), expected_diagonal, 1e-10);

  // Assert: Off-diagonal elements
  EXPECT_NEAR(I(0, 1), 0.0, 1e-10);
  EXPECT_NEAR(I(0, 2), 0.0, 1e-10);
  EXPECT_NEAR(I(1, 2), 0.0, 1e-10);

  // Assert: Symmetry
  EXPECT_NEAR(I(1, 0), I(0, 1), 1e-10);
  EXPECT_NEAR(I(2, 0), I(0, 2), 1e-10);
  EXPECT_NEAR(I(2, 1), I(1, 2), 1e-10);
}

TEST(InertialCalculationsTest, RectangularBoxAnalytical)
{
  // Arrange: Rectangular box (2x3x4, density = 1, centered at origin)
  double a = 2.0, b = 3.0, c = 4.0;
  auto points = createRectangularBoxPoints(a, b, c);
  ConvexHull hull{points};
  double density = 1.0;
  double volume = a * b * c;

  // Analytical solution:
  // Ixx = m(b² + c²)/12 = (9 + 16)/12 = 25/12 ≈ 2.083333333333
  // Iyy = m(a² + c²)/12 = (4 + 16)/12 = 20/12 ≈ 1.666666666667
  // Izz = m(a² + b²)/12 = (4 + 9)/12 = 13/12 ≈ 1.083333333333
  // Off-diagonal: 0
  double expected_xx = density * volume * (b * b + c * c) / 12.0;
  double expected_yy = density * volume * (a * a + c * c) / 12.0;
  double expected_zz = density * volume * (a * a + b * b) / 12.0;

  // Act
  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density);

  // Assert: Diagonal elements
  EXPECT_NEAR(I(0, 0), expected_xx, 1e-10);
  EXPECT_NEAR(I(1, 1), expected_yy, 1e-10);
  EXPECT_NEAR(I(2, 2), expected_zz, 1e-10);

  // Assert: Off-diagonal elements
  EXPECT_NEAR(I(0, 1), 0.0, 1e-10);
  EXPECT_NEAR(I(0, 2), 0.0, 1e-10);
  EXPECT_NEAR(I(1, 2), 0.0, 1e-10);

  // Assert: Symmetry
  EXPECT_NEAR(I(1, 0), I(0, 1), 1e-10);
  EXPECT_NEAR(I(2, 0), I(0, 2), 1e-10);
  EXPECT_NEAR(I(2, 1), I(1, 2), 1e-10);
}

TEST(InertialCalculationsTest, RegularTetrahedronAnalytical)
{
  // Arrange: Regular tetrahedron (edge = 2.0, density = 1)
  double edgeLength = 2.0;
  auto points = createRegularTetrahedronPoints(edgeLength);
  ConvexHull hull{points};
  auto volume = hull.getVolume();
  double density = 1.0;

  // Analytical solution for regular tetrahedron:
  // Ixx = Iyy = Izz = m*L²/20 = 1.0 * 4.0 / 20.0 = 0.2
  // (when vertices are symmetrically placed as we construct them)
  double expected_diagonal = density * volume * edgeLength * edgeLength / 20.0;

  // Act
  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density);

  // Assert: Diagonal elements should all be equal (symmetric geometry)
  double avg_diagonal = (I(0, 0) + I(1, 1) + I(2, 2)) / 3.0;
  EXPECT_NEAR(avg_diagonal, expected_diagonal, 1e-10);

  // Assert: Individual diagonal elements match average
  EXPECT_NEAR(I(0, 0), avg_diagonal, 1e-10);
  EXPECT_NEAR(I(1, 1), avg_diagonal, 1e-10);
  EXPECT_NEAR(I(2, 2), avg_diagonal, 1e-10);

  // Assert: Tensor is symmetric
  EXPECT_NEAR(I(1, 0), I(0, 1), 1e-10);
  EXPECT_NEAR(I(2, 0), I(0, 2), 1e-10);
  EXPECT_NEAR(I(2, 1), I(1, 2), 1e-10);
}

TEST(InertialCalculationsTest, VolumeByproduct)
{
  // Arrange: Unit cube
  auto points = createUnitCubePoints();
  ConvexHull hull{points};
  double density = 1.0;

  // Expected volume = 1.0
  double expected_volume = 1.0;
  double hull_volume = hull.getVolume();

  EXPECT_NEAR(hull_volume, expected_volume, 1e-10);

  // Act: Should not throw due to volume mismatch
  EXPECT_NO_THROW(
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density));
}

TEST(InertialCalculationsTest, CentroidByproduct)
{
  // Arrange: Rectangular box centered at origin
  double a = 2.0, b = 3.0, c = 4.0;
  auto points = createRectangularBoxPoints(a, b, c);
  ConvexHull hull{points};

  // Centroid should be at origin for centered box
  Coordinate centroid = hull.getCentroid();
  EXPECT_NEAR(centroid.x(), 0.0, 1e-10);
  EXPECT_NEAR(centroid.y(), 0.0, 1e-10);
  EXPECT_NEAR(centroid.z(), 0.0, 1e-10);

  // Compute inertia (internally computes centroid as byproduct)
  double density = 1.0;
  EXPECT_NO_THROW(
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density));
}

TEST(InertialCalculationsTest, SymmetryProperty)
{
  // Arrange: Unit cube
  auto points = createUnitCubePoints();
  ConvexHull hull{points};
  double density = 1.0;

  // Act
  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density);

  // Assert: Tensor is symmetric (I_ij == I_ji)
  EXPECT_NEAR(I(0, 1), I(1, 0), 1e-10);
  EXPECT_NEAR(I(0, 2), I(2, 0), 1e-10);
  EXPECT_NEAR(I(1, 2), I(2, 1), 1e-10);
}

TEST(InertialCalculationsTest, PositiveDefinite)
{
  // Arrange: Unit cube
  auto points = createUnitCubePoints();
  ConvexHull hull{points};
  double density = 1.0;

  // Act
  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density);

  // Assert: All eigenvalues > 0 (positive definite)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(I);
  msd_sim::Vector3D eigenvalues = solver.eigenvalues();

  EXPECT_GT(eigenvalues(0), 0.0);
  EXPECT_GT(eigenvalues(1), 0.0);
  EXPECT_GT(eigenvalues(2), 0.0);
}

TEST(InertialCalculationsTest, LargeCoordinateOffset)
{
  // Arrange: Unit cube offset far from origin
  auto points = createUnitCubePoints();
  Coordinate offset{1000.0, 2000.0, 3000.0};
  for (auto& p : points)
  {
    p = p + offset;
  }

  ConvexHull hull{points};
  double density = 1.0;

  // Expected: Same inertia as centered cube (about centroid)
  double expected_diagonal = density / 6.0;

  // Act
  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density);

  // Assert: Inertia about centroid is independent of translation
  // Note: Tolerance relaxed to 1e-8 due to floating-point precision degradation
  // at large coordinate offsets. The algorithm accumulates products involving
  // coordinates in the thousands, which reduces relative precision.
  EXPECT_NEAR(I(0, 0), expected_diagonal, 1e-8);
  EXPECT_NEAR(I(1, 1), expected_diagonal, 1e-8);
  EXPECT_NEAR(I(2, 2), expected_diagonal, 1e-8);
}

TEST(InertialCalculationsTest, ExtremeAspectRatio)
{
  // Arrange: Very thin rectangular box (1:100:100 aspect ratio)
  double a = 0.01, b = 1.0, c = 1.0;
  auto points = createRectangularBoxPoints(a, b, c);
  ConvexHull hull{points};
  auto volume = hull.getVolume();
  double density = 1.0;

  // Analytical solution (box formulas still apply)
  double expected_xx = density * volume * (b * b + c * c) / 12.0;
  double expected_yy = density * volume * (a * a + c * c) / 12.0;
  double expected_zz = density * volume * (a * a + b * b) / 12.0;

  // Act
  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density);

  // Assert: Even with extreme aspect ratio, accuracy maintained
  EXPECT_NEAR(I(0, 0), expected_xx, 1e-9);
  EXPECT_NEAR(I(1, 1), expected_yy, 1e-9);
  EXPECT_NEAR(I(2, 2), expected_zz, 1e-9);
}

TEST(InertialCalculationsTest, SingleTetrahedron)
{
  // Arrange: Regular tetrahedron (minimum convex hull)
  double edgeLength = 1.0;
  auto points = createRegularTetrahedronPoints(edgeLength);
  ConvexHull hull{points};
  auto volume = hull.getVolume();
  double density = 1.0;

  // Expected: Valid inertia tensor
  double expected_diagonal = density * volume * edgeLength * edgeLength / 20.0;

  // Act
  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density);

  // Assert: Produces valid result for minimal hull
  double avg_diagonal = (I(0, 0) + I(1, 1) + I(2, 2)) / 3.0;
  EXPECT_NEAR(avg_diagonal, expected_diagonal, 1e-10);

  // Assert: Positive definite
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(I);
  msd_sim::Vector3D eigenvalues = solver.eigenvalues();
  EXPECT_GT(eigenvalues(0), 0.0);
  EXPECT_GT(eigenvalues(1), 0.0);
  EXPECT_GT(eigenvalues(2), 0.0);
}

TEST(InertialCalculationsTest, Invaliddensity)
{
  // Arrange: Unit cube
  auto points = createUnitCubePoints();
  ConvexHull hull{points};

  // Act & Assert: Zero density
  EXPECT_THROW(
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, 0.0),
    std::invalid_argument);

  // Act & Assert: Negative density
  EXPECT_THROW(
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, -1.0),
    std::invalid_argument);
}

TEST(InertialCalculationsTest, InvalidHull)
{
  // Arrange: Empty hull
  ConvexHull hull;
  double density = 1.0;

  // Act & Assert
  EXPECT_THROW(
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density),
    std::runtime_error);
}

TEST(InertialCalculationsTest, densityScaling)
{
  // Arrange: Unit cube with two different densityes
  auto points = createUnitCubePoints();
  ConvexHull hull{points};

  double density1 = 1.0;
  double density2 = 10.0;

  // Act
  Eigen::Matrix3d I1 =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density1);
  Eigen::Matrix3d I2 =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, density2);

  // Assert: Inertia scales linearly with density
  double scale = density2 / density1;
  EXPECT_NEAR(I2(0, 0), I1(0, 0) * scale, 1e-10);
  EXPECT_NEAR(I2(1, 1), I1(1, 1) * scale, 1e-10);
  EXPECT_NEAR(I2(2, 2), I1(2, 2) * scale, 1e-10);
}

// Compare the inertial calculation agains the known reference from the original
// C source
TEST(InertialCalculationsTest, Tetrahedron)
{
  std::vector<msd_sim::Coordinate> points{
    {0, 0, 0}, {5, 0, 0}, {0, 4, 0}, {0, 0, 3}};

  ConvexHull hull{points};

  Eigen::Matrix3d I =
    inertial_calculations::computeInertiaTensorAboutCentroid(hull, 1.0);

  EXPECT_NEAR(I(0, 0), 9.375, 1e-10);
  EXPECT_NEAR(I(0, 1), 2.50, 1E-10);
  EXPECT_NEAR(I(0, 2), 1.875, 1e-10);

  EXPECT_NEAR(I(1, 0), 2.5, 1e-10);
  EXPECT_NEAR(I(1, 1), 12.75, 1e-10);
  EXPECT_NEAR(I(1, 2), 1.5, 1e-10);

  EXPECT_NEAR(I(2, 0), 1.875, 1e-10);
  EXPECT_NEAR(I(2, 1), 1.500, 1e-10);
  EXPECT_NEAR(I(2, 2), 15.375, 1e-10);
}
