// Prototype P1: EPA Convergence Validation
// Question: Does EPA reliably converge within 64 iterations for typical convex
// hulls? Success criteria: 95%+ success rate with <32 iterations average

#ifndef EPA_HPP
#define EPA_HPP

#include <Eigen/Dense>
#include <array>
#include <limits>
#include <vector>

// Minimal types recreated from codebase
using Coordinate = msd_sim::Vector3D;

struct EPAFace
{
  std::array<size_t, 3> vertexIndices;
  Coordinate normal;
  double distance;

  EPAFace() = default;
  EPAFace(size_t v0, size_t v1, size_t v2, const Coordinate& n, double d)
    : vertexIndices{v0, v1, v2}, normal{n}, distance{d}
  {
  }
};

struct EPAEdge
{
  size_t v0;
  size_t v1;

  EPAEdge() = default;
  EPAEdge(size_t a, size_t b) : v0{a}, v1{b}
  {
  }

  bool operator==(const EPAEdge& other) const
  {
    return (v0 == other.v0 && v1 == other.v1) ||
           (v0 == other.v1 && v1 == other.v0);
  }
};

struct CollisionResult
{
  Coordinate normal;
  double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};
  Coordinate contactPoint;

  CollisionResult() = default;
  CollisionResult(const Coordinate& n, double depth, const Coordinate& point)
    : normal{n}, penetrationDepth{depth}, contactPoint{point}
  {
  }
};

struct ConvergenceMetrics
{
  bool converged{false};
  int iterations{0};
  double finalDistance{std::numeric_limits<double>::quiet_NaN()};
  double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};
};

class EPA
{
public:
  EPA(double epsilon = 1e-6);

  // Returns metrics for validation (not just result)
  ConvergenceMetrics computeContactInfoWithMetrics(
    const std::vector<Coordinate>& simplex,
    int maxIterations = 64);

  CollisionResult computeContactInfo(const std::vector<Coordinate>& simplex,
                                     int maxIterations = 64);

private:
  double epsilon_;
  std::vector<Coordinate> vertices_;
  std::vector<EPAFace> faces_;

  bool expandPolytope(int maxIterations, ConvergenceMetrics& metrics);
  size_t findClosestFace() const;
  bool isVisible(const EPAFace& face, const Coordinate& point) const;
  std::vector<EPAEdge> buildHorizonEdges(const Coordinate& newVertex);
  void addFace(size_t v0, size_t v1, size_t v2);
  Coordinate computeContactPoint(const EPAFace& face) const;

  // Simplified support function for prototyping (returns point in direction)
  Coordinate supportMinkowski(const Coordinate& dir) const;
};

#endif  // EPA_HPP
