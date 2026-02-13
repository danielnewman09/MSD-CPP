// Ticket: 0056e_threejs_core_visualization
// Python bindings for AssetRegistry — geometry lookup by asset ID

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <tuple>
#include <vector>

#include "msd-assets/src/AssetRegistry.hpp"
#include "msd-assets/src/Geometry.hpp"

namespace py = pybind11;

/**
 * @brief Python wrapper for AssetRegistry
 *
 * Provides geometry lookup by ObjectRecord.id (asset_id),
 * matching the C++ AssetRegistry::loadCollisionGeometry() interface.
 */
class AssetRegistryWrapper
{
public:
  explicit AssetRegistryWrapper(const std::string& dbPath)
    : registry_{dbPath}
  {
  }

  /**
   * @brief Get collision vertices for an asset by ObjectRecord.id
   * @param assetId ObjectRecord.id (maps to AssetPhysicalStaticRecord.asset_id)
   * @return List of (x, y, z) tuples, or empty list if asset not found
   */
  std::vector<std::tuple<double, double, double>>
  getCollisionVertices(uint32_t assetId)
  {
    auto geoOpt = registry_.loadCollisionGeometry(assetId);
    if (!geoOpt.has_value())
    {
      return {};
    }

    const auto& geometry = geoOpt->get();
    const auto& vertices = geometry.getVertices();

    std::vector<std::tuple<double, double, double>> result;
    result.reserve(vertices.size());

    for (const auto& v : vertices)
    {
      result.emplace_back(v.x(), v.y(), v.z());
    }

    return result;
  }

  /**
   * @brief Get asset name by ObjectRecord.id
   * @param assetId ObjectRecord.id
   * @return Asset name, or empty string if not found
   */
  std::string getAssetName(uint32_t assetId)
  {
    auto assetOpt = registry_.getAsset(assetId);
    if (!assetOpt.has_value())
    {
      return "";
    }
    return assetOpt->get().getName();
  }

  /**
   * @brief List all available asset IDs and names
   * @return List of (id, name) pairs
   */
  std::vector<std::tuple<uint32_t, std::string>> listAssets()
  {
    const auto& cache = registry_.getAssetCache();
    std::vector<std::tuple<uint32_t, std::string>> result;
    result.reserve(cache.size());
    for (const auto& asset : cache)
    {
      result.emplace_back(asset.getId(), asset.getName());
    }
    return result;
  }

private:
  msd_assets::AssetRegistry registry_;
};

void bind_asset_registry(py::module_& m)
{
  py::class_<AssetRegistryWrapper>(m, "AssetRegistry")
    .def(py::init<const std::string&>(), py::arg("db_path"),
         "Create AssetRegistry from asset database path")
    .def("get_collision_vertices", &AssetRegistryWrapper::getCollisionVertices,
         py::arg("asset_id"),
         "Get collision geometry vertices as list of (x, y, z) tuples")
    .def("get_asset_name", &AssetRegistryWrapper::getAssetName,
         py::arg("asset_id"),
         "Get asset name by ObjectRecord.id")
    .def("list_assets", &AssetRegistryWrapper::listAssets,
         "List all available asset IDs and names");
}
