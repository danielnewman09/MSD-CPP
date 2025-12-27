#include "msd-assets/src/GeometryFactory.hpp"
#include <array>

namespace msd_assets
{

std::array<Eigen::Vector3d, 8> GeometryFactory::getCubeCorners(double size)
{
  float half = static_cast<float>(size) / 2.0f;

  // Define 8 corners of the cube
  // Naming: (Front/Back)(Top/Bottom)(Left/Right)
  return {
    Eigen::Vector3d{-half, -half, -half},  // 0: FTL (Front Top Left)
    Eigen::Vector3d{half, -half, -half},   // 1: FTR (Front Top Right)
    Eigen::Vector3d{half, half, -half},    // 2: FBR (Front Bottom Right)
    Eigen::Vector3d{-half, half, -half},   // 3: FBL (Front Bottom Left)
    Eigen::Vector3d{-half, -half, half},   // 4: BTL (Back Top Left)
    Eigen::Vector3d{half, -half, half},    // 5: BTR (Back Top Right)
    Eigen::Vector3d{half, half, half},     // 6: BBR (Back Bottom Right)
    Eigen::Vector3d{-half, half, half}     // 7: BBL (Back Bottom Left)
  };
}

// Template implementations are now in the header file

}  // namespace msd_assets
