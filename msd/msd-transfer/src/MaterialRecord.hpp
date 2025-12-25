#ifndef MSD_TRANSFER_MATERIAL_RECORD_HPP
#define MSD_TRANSFER_MATERIAL_RECORD_HPP

#include <cstdint>
#include <string>

#include <boost/describe.hpp>
#include <cpp_sqlite/sqlite_db/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for rendering materials
 *
 * Stores shader references and default material properties for rendering.
 * Future extension point for PBR materials, textures, etc.
 */
struct MaterialRecord : public cpp_sqlite::BaseTransferObject
{
  std::string name;  // Unique material name (e.g., "solid_color")

  // Shader references (file paths or names)
  std::string shader_vertex;
  std::string shader_fragment;

  // Default color (if not overridden per-instance)
  float default_color_r{1.0f};
  float default_color_g{1.0f};
  float default_color_b{1.0f};

  // Material properties
  float shininess{32.0f};
  float metallic{0.0f};
  float roughness{1.0f};
};

}  // namespace msd_transfer

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(msd_transfer::MaterialRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (name,
                       shader_vertex,
                       shader_fragment,
                       default_color_r,
                       default_color_g,
                       default_color_b,
                       shininess,
                       metallic,
                       roughness));

#endif  // MSD_TRANSFER_MATERIAL_RECORD_HPP
