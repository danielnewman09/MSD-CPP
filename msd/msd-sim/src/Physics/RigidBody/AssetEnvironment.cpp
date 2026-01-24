
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"

namespace msd_sim
{

AssetEnvironment::AssetEnvironment(uint32_t assetId,
                                   uint32_t instanceId,
                                   ConvexHull& hull,
                                   const ReferenceFrame& frame)
  : AssetPhysical{assetId, instanceId, hull, frame}
{
}


}  // namespace msd_sim
