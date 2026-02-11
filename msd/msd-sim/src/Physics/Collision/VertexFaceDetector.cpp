// Ticket: 0055c_friction_direction_fix
// Design: docs/designs/0055c_friction_direction_fix/design.md

#include "VertexFaceDetector.hpp"

namespace msd_sim
{

ContactType VertexFaceDetector::detectContactType(size_t refVertCount,
                                                   size_t incVertCount) const
{
  // Face-face: Both sides have >= 3 vertices
  if (refVertCount >= 3 && incVertCount >= 3)
  {
    return ContactType::FaceFace;
  }

  // Edge-edge: Both sides have exactly 2 vertices
  if (refVertCount == 2 && incVertCount == 2)
  {
    return ContactType::EdgeEdge;
  }

  // Vertex-face: One side has 1 vertex, other has >= 3
  if ((refVertCount == 1 && incVertCount >= 3) ||
      (incVertCount == 1 && refVertCount >= 3))
  {
    return ContactType::VertexFace;
  }

  // Vertex-vertex: Both sides have exactly 1 vertex
  if (refVertCount == 1 && incVertCount == 1)
  {
    return ContactType::VertexVertex;
  }

  // Unknown: Unexpected geometry (should be rare)
  return ContactType::Unknown;
}

bool VertexFaceDetector::isVertexFaceContact(size_t refVertCount,
                                              size_t incVertCount) const
{
  return detectContactType(refVertCount, incVertCount) == ContactType::VertexFace;
}

}  // namespace msd_sim
