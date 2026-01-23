// Ticket: 0022_gjk_asset_physical_transform
// Design: docs/designs/0022_gjk_asset_physical_transform/design.md

#include "msd-sim/src/Physics/GJK.hpp"
#include <algorithm>
#include <limits>
#include <vector>

namespace msd_sim
{

// GJK class implementation

GJK::GJK(const AssetPhysical& assetA,
         const AssetPhysical& assetB,
         double epsilon)
  : assetA_{assetA},
    assetB_{assetB},
    epsilon_{epsilon},
    simplex_{},
    direction_{0.0, 0.0, 0.0}
{
}

bool GJK::intersects(int maxIterations)
{
  // Get collision hulls from assets
  const ConvexHull& hullA = assetA_.getCollisionHull();
  const ConvexHull& hullB = assetB_.getCollisionHull();

  // Get reference frames for transformation
  const ReferenceFrame& frameA = assetA_.getReferenceFrame();
  const ReferenceFrame& frameB = assetB_.getReferenceFrame();

  // Quick bounding box check first (cheap early-out)
  // Transform bounding boxes to world space before comparison
  auto bboxA_local = hullA.getBoundingBox();
  auto bboxB_local = hullB.getBoundingBox();

  // Transform bounding box corners to world space
  Coordinate bboxA_min_world = frameA.localToGlobal(bboxA_local.min);
  Coordinate bboxA_max_world = frameA.localToGlobal(bboxA_local.max);
  Coordinate bboxB_min_world = frameB.localToGlobal(bboxB_local.min);
  Coordinate bboxB_max_world = frameB.localToGlobal(bboxB_local.max);

  // Recompute axis-aligned bounding box in world space
  // (rotation may change which corners are min/max)
  auto recomputeAABB = [](const Coordinate& c1, const Coordinate& c2)
  {
    return std::make_pair(Coordinate{std::min(c1.x(), c2.x()),
                                     std::min(c1.y(), c2.y()),
                                     std::min(c1.z(), c2.z())},
                          Coordinate{std::max(c1.x(), c2.x()),
                                     std::max(c1.y(), c2.y()),
                                     std::max(c1.z(), c2.z())});
  };

  auto [bboxA_min, bboxA_max] = recomputeAABB(bboxA_min_world, bboxA_max_world);
  auto [bboxB_min, bboxB_max] = recomputeAABB(bboxB_min_world, bboxB_max_world);

  if (bboxA_max.x() < bboxB_min.x() || bboxA_min.x() > bboxB_max.x() ||
      bboxA_max.y() < bboxB_min.y() || bboxA_min.y() > bboxB_max.y() ||
      bboxA_max.z() < bboxB_min.z() || bboxA_min.z() > bboxB_max.z())
  {
    return false;  // Bounding boxes don't overlap
  }

  // Initialize: pick initial search direction (from A toward B in world space)
  Coordinate centroidA_world = frameA.localToGlobal(hullA.getCentroid());
  Coordinate centroidB_world = frameB.localToGlobal(hullB.getCentroid());
  direction_ = centroidB_world - centroidA_world;

  // Handle edge case where centroids are identical
  if (direction_.norm() < epsilon_)
  {
    direction_ = Coordinate{1.0, 0.0, 0.0};
  }

  // Get first support point
  simplex_.clear();
  simplex_.push_back(supportMinkowski(direction_));

  // New direction points toward origin
  direction_ = -simplex_[0];

  // GJK main loop
  for (int iteration = 0; iteration < maxIterations; ++iteration)
  {
    // Get support point in search direction
    Coordinate newPoint = supportMinkowski(direction_);

    // Check if we've passed the origin
    // If the new point isn't past the origin in our search direction,
    // then the Minkowski difference doesn't contain the origin
    if (newPoint.dot(direction_) < epsilon_)
    {
      return false;  // No intersection
    }

    // Add new point to simplex
    simplex_.push_back(newPoint);

    // Update simplex and direction
    if (updateSimplex())
    {
      return true;  // Origin is contained in simplex → collision!
    }

    // Safety check: direction should never be zero
    if (direction_.norm() < epsilon_)
    {
      // If direction is near-zero, we're likely at the origin
      return true;
    }
  }

  // Maximum iterations reached without conclusion
  // This should be very rare for well-formed convex hulls
  return false;
}

Coordinate GJK::support(const ConvexHull& hull, const Coordinate& dir) const
{
  const auto& vertices = hull.getVertices();

  double maxDot = -std::numeric_limits<double>::infinity();
  Coordinate furthest{0.0, 0.0, 0.0};

  for (const auto& vertex : vertices)
  {
    double dotProduct = vertex.dot(dir);
    if (dotProduct > maxDot)
    {
      maxDot = dotProduct;
      furthest = vertex;
    }
  }

  return furthest;
}

Coordinate GJK::supportMinkowski(const CoordinateRate& dir) const
{
  // Get collision hulls and reference frames from assets
  const ConvexHull& hullA = assetA_.getCollisionHull();
  const ConvexHull& hullB = assetB_.getCollisionHull();
  const ReferenceFrame& frameA = assetA_.getReferenceFrame();
  const ReferenceFrame& frameB = assetB_.getReferenceFrame();

  // Transform search direction from world space to local space for asset A
  // (rotation only - direction vectors don't translate)
  Coordinate dirA_local = frameA.globalToLocal(dir);

  // Get support vertex in local space for asset A
  Coordinate supportA_local = support(hullA, dirA_local);

  // Transform support vertex from local space to world space
  // (rotation + translation - positions transform fully)
  Coordinate supportA_world = frameA.localToGlobal(supportA_local);

  // Same process for asset B with negated direction
  Coordinate dirB_local = frameB.globalToLocal(CoordinateRate{-dir});
  Coordinate supportB_local = support(hullB, dirB_local);
  Coordinate supportB_world = frameB.localToGlobal(supportB_local);

  // Return Minkowski difference in world space
  return supportA_world - supportB_world;
}

bool GJK::updateSimplex()
{
  switch (simplex_.size())
  {
    case 2:
      return handleLine();
    case 3:
      return handleTriangle();
    case 4:
      return handleTetrahedron();
    default:
      // Should never happen
      return false;
  }
}

bool GJK::handleLine()
{
  const Coordinate& a = simplex_[1];  // Newest point
  const Coordinate& b = simplex_[0];  // Previous point

  Coordinate ab = b - a;  // Vector from A to B
  Coordinate ao = -a;     // Vector from A to origin

  // Direction perpendicular to AB, pointing toward origin
  // This is the triple product: (AB × AO) × AB
  Coordinate abPerp = ab.cross(ao).cross(ab);

  direction_ = abPerp;
  // Keep both points in simplex
  return false;
}

bool GJK::handleTriangle()
{
  const Coordinate& a = simplex_[2];  // Newest point
  const Coordinate& b = simplex_[1];
  const Coordinate& c = simplex_[0];

  Coordinate ab = b - a;
  Coordinate ac = c - a;
  Coordinate ao = -a;

  Coordinate abc = ab.cross(ac);  // Triangle normal

  // Check if origin is outside triangle in direction of edge AC
  Coordinate acPerp = abc.cross(ac);
  if (sameDirection(acPerp, ao))
  {
    // Origin is in Voronoi region of AC edge
    simplex_ = {c, a};
    direction_ = ac.cross(ao).cross(ac);
    return false;
  }

  // Check if origin is outside triangle in direction of edge AB
  Coordinate abPerp = ab.cross(abc);
  if (sameDirection(abPerp, ao))
  {
    // Origin is in Voronoi region of AB edge
    simplex_ = {b, a};
    direction_ = ab.cross(ao).cross(ab);
    return false;
  }

  // Origin is in Voronoi region of triangle face
  // Check which side of the triangle the origin is on
  if (sameDirection(abc, ao))
  {
    // Origin is above triangle (toward ABC normal)
    direction_ = abc;
  }
  else
  {
    // Origin is below triangle (away from ABC normal)
    // Reorder simplex to maintain correct winding
    simplex_ = {b, c, a};
    direction_ = -abc;
  }

  return false;
}

bool GJK::handleTetrahedron()
{
  const Coordinate& a = simplex_[3];  // Newest point
  const Coordinate& b = simplex_[2];
  const Coordinate& c = simplex_[1];
  const Coordinate& d = simplex_[0];

  Coordinate ab = b - a;
  Coordinate ac = c - a;
  Coordinate ad = d - a;
  Coordinate ao = -a;

  // Normals of tetrahedron faces (pointing outward from A)
  Coordinate abc = ab.cross(ac);
  Coordinate acd = ac.cross(ad);
  Coordinate adb = ad.cross(ab);

  // Check each face to see if origin is beyond it

  // Check face ABC
  if (sameDirection(abc, ao))
  {
    // Origin is beyond ABC face
    simplex_ = {c, b, a};
    direction_ = abc;
    return false;
  }

  // Check face ACD
  if (sameDirection(acd, ao))
  {
    // Origin is beyond ACD face
    simplex_ = {d, c, a};
    direction_ = acd;
    return false;
  }

  // Check face ADB
  if (sameDirection(adb, ao))
  {
    // Origin is beyond ADB face
    simplex_ = {b, d, a};
    direction_ = adb;
    return false;
  }

  // Origin is not beyond any face → it's inside the tetrahedron!
  return true;
}

bool GJK::sameDirection(const Coordinate& direction, const Coordinate& ao)
{
  return direction.dot(ao) > 0.0;
}

// Convenience function implementation

bool gjkIntersects(const AssetPhysical& assetA,
                   const AssetPhysical& assetB,
                   double epsilon,
                   int maxIterations)
{
  GJK gjk{assetA, assetB, epsilon};
  return gjk.intersects(maxIterations);
}

}  // namespace msd_sim
