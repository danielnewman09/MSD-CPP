#ifndef MSD_ASSETS_ASSET_HPP
#define MSD_ASSETS_ASSET_HPP

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include "msd-assets/src/Geometry.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace msd_assets
{


/**
 * @brief Represents a complete asset with optional visual and collision
 * geometry
 *
 * Asset encapsulates an object from the database along with its associated
 * visual and/or collision geometry. Assets are immutable once created and
 * provide access to their geometry components.
 */
class Asset
{
public:
  Asset() = delete;

  // The objectrecord contains the foreign key which facilitates resolving the
  // records for both the visual geometry and the collision geometry
  static Asset fromObjectRecord(msd_transfer::ObjectRecord& record,
                                cpp_sqlite::Database& db);

  [[nodiscard]] uint32_t getId() const;
  [[nodiscard]] const std::string& getName() const;
  [[nodiscard]] const std::string& getCategory() const;

  [[nodiscard]] std::optional<std::reference_wrapper<const VisualGeometry>>
  getVisualGeometry() const;
  [[nodiscard]] std::optional<std::reference_wrapper<const CollisionGeometry>>
  getCollisionGeometry() const;

  [[nodiscard]] bool hasVisualGeometry() const;
  [[nodiscard]] bool hasCollisionGeometry() const;

private:
  // The visualgeometry and collisiongeometry objects can be build from the
  // factory methods in the geometry.hpp header after retrieving the foreign
  // keys from the objectrecord.
  Asset(uint32_t id,
        std::string name,
        std::string category,
        std::optional<VisualGeometry> visualGeom,
        std::optional<CollisionGeometry> collisionGeom);

  // Identifier from the object record
  uint32_t id_;
  std::string name_;
  std::string category_;

  // Optional geometry components
  std::optional<VisualGeometry> visualGeometry_;
  std::optional<CollisionGeometry> collisionGeometry_;
};

}  // namespace msd_assets

#endif  // MSD_ASSETS_ASSET_HPP