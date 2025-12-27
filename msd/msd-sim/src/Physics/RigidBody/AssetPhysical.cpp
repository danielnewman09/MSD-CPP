#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include <sstream>
#include <stdexcept>
#include "msd-assets/src/Geometry.hpp"

namespace msd_sim
{

AssetPhysical::AssetPhysical(std::shared_ptr<msd_assets::Geometry> geometry,
                             const ReferenceFrame& frame)
  : visualGeometry_(*geometry),
    referenceFrame_(frame),
    collisionHull_{
      std::make_unique<ConvexHull>(ConvexHull::fromGeometry(visualGeometry_))}
{
  if (!geometry)
  {
    throw std::invalid_argument(
      "Cannot create AssetPhysical with null geometry");
  }

  if (geometry->getVertexCount() == 0)
  {
    throw std::invalid_argument(
      "Cannot create AssetPhysical with empty geometry");
  }
}

const ConvexHull& AssetPhysical::getCollisionHull() const
{
  return collisionGeometry_.get().;
}

const ReferenceFrame& AssetPhysical::getReferenceFrame() const
{
  return referenceFrame_;
}

ReferenceFrame& AssetPhysical::getReferenceFrame()
{
  return referenceFrame_;
}


}  // namespace msd_sim
