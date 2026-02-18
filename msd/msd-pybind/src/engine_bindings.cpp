// Ticket: 0072a_engine_pybind_bindings
// Python bindings for msd_sim::Engine — live simulation control from Python

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <string>
#include <tuple>
#include <vector>

#include "msd-assets/src/AssetRegistry.hpp"
#include "msd-sim/src/Engine.hpp"
#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/AngularVelocity.hpp"

namespace py = pybind11;

/**
 * @brief Python wrapper for msd_sim::Engine
 *
 * Converts C++ types (Coordinate, AngularCoordinate, Eigen quaternions) to
 * Python-native dicts/tuples internally. No Eigen type casters are exposed.
 *
 * Follows the DatabaseWrapper / AssetRegistryWrapper pattern: all conversion
 * happens inside the C++ wrapper so the Python API stays simple.
 *
 * @ticket 0072a_engine_pybind_bindings
 */
class EngineWrapper
{
public:
  /**
   * @brief Construct an Engine from an assets database path
   * @param dbPath Path to assets.db (created by generate_assets)
   * @throws std::runtime_error if database cannot be opened
   */
  explicit EngineWrapper(const std::string& dbPath)
    : engine_{dbPath}
  {
  }

  /**
   * @brief Spawn a dynamic inertial object into the simulation
   * @param assetName Name of the asset in the registry
   * @param x,y,z World position [m]
   * @param pitch,roll,yaw Initial orientation [rad]
   * @param mass Mass [kg]
   * @param restitution Coefficient of restitution [0, 1]
   * @param friction Friction coefficient [0, inf)
   * @return dict with "instance_id" and "asset_id"
   * @throws std::runtime_error if asset not found or has no collision geometry
   */
  py::dict spawnInertialObject(const std::string& assetName,
                               double x,
                               double y,
                               double z,
                               double pitch,
                               double roll,
                               double yaw,
                               double mass,
                               double restitution,
                               double friction)
  {
    const msd_sim::Coordinate position{x, y, z};
    const msd_sim::AngularCoordinate orientation{pitch, roll, yaw};

    const auto& asset = engine_.spawnInertialObject(
      assetName, position, orientation, mass, restitution, friction);

    py::dict result;
    result["instance_id"] = asset.getInstanceId();
    result["asset_id"] = asset.getAssetId();
    return result;
  }

  /**
   * @brief Spawn a static environment object into the simulation
   * @param assetName Name of the asset in the registry
   * @param x,y,z World position [m]
   * @param pitch,roll,yaw Initial orientation [rad]
   * @return dict with "instance_id" and "asset_id"
   * @throws std::runtime_error if asset not found or has no collision geometry
   */
  py::dict spawnEnvironmentObject(const std::string& assetName,
                                  double x,
                                  double y,
                                  double z,
                                  double pitch,
                                  double roll,
                                  double yaw)
  {
    const msd_sim::Coordinate position{x, y, z};
    const msd_sim::AngularCoordinate orientation{pitch, roll, yaw};

    const auto& asset =
      engine_.spawnEnvironmentObject(assetName, position, orientation);

    py::dict result;
    result["instance_id"] = asset.getInstanceId();
    result["asset_id"] = asset.getAssetId();
    return result;
  }

  /**
   * @brief Set the current simulation time (absolute, not delta)
   *
   * The underlying WorldModel::update(simTime) receives the absolute simulation
   * time, not a delta. The caller is responsible for tracking cumulative time
   * and passing an increasing value on each call.
   *
   * Example (60 FPS):
   *   for i in range(1, 61):
   *       engine.update(i * 16)  # 16ms, 32ms, 48ms, ...
   *
   * @param milliseconds Absolute simulation time [ms]
   */
  void update(int milliseconds)
  {
    engine_.update(std::chrono::milliseconds{milliseconds});
  }

  /**
   * @brief Get the current simulation state for all dynamic bodies
   *
   * Returns the simulation time plus the state of every inertial asset
   * spawned into the world. Position, velocity, orientation (quaternion),
   * and angular velocity are extracted from each AssetInertial.
   *
   * @return dict with:
   *   "simulation_time": float (seconds)
   *   "states": list of dicts each containing:
   *     "body_id":          int
   *     "asset_id":         int
   *     "position":         {"x": float, "y": float, "z": float}
   *     "velocity":         {"x": float, "y": float, "z": float}
   *     "orientation":      {"w": float, "x": float, "y": float, "z": float}
   *     "angular_velocity": {"x": float, "y": float, "z": float}
   */
  py::dict getFrameState()
  {
    const auto& worldModel = engine_.getWorldModel();

    // Simulation time in seconds
    const double simTimeSec =
      static_cast<double>(worldModel.getTime().count()) / 1000.0;

    py::list states;

    // Iterate over all dynamic (inertial) bodies
    for (const auto& asset : worldModel.getInertialAssets())
    {
      const auto& state = asset.getInertialState();
      const auto& frame = asset.getReferenceFrame();

      // Position from reference frame origin
      const auto& origin = frame.getOrigin();
      py::dict position;
      position["x"] = origin.x();
      position["y"] = origin.y();
      position["z"] = origin.z();

      // Linear velocity from kinematic state
      py::dict velocity;
      velocity["x"] = state.velocity.x();
      velocity["y"] = state.velocity.y();
      velocity["z"] = state.velocity.z();

      // Orientation quaternion (w, x, y, z)
      const auto& q = state.orientation;
      py::dict orientation;
      orientation["w"] = q.w();
      orientation["x"] = q.x();
      orientation["y"] = q.y();
      orientation["z"] = q.z();

      // Angular velocity derived from quaternion rate
      const auto omega = state.getAngularVelocity();
      py::dict angularVelocity;
      angularVelocity["x"] = omega.x();
      angularVelocity["y"] = omega.y();
      angularVelocity["z"] = omega.z();

      py::dict bodyState;
      bodyState["body_id"] = asset.getInstanceId();
      bodyState["asset_id"] = asset.getAssetId();
      bodyState["position"] = position;
      bodyState["velocity"] = velocity;
      bodyState["orientation"] = orientation;
      bodyState["angular_velocity"] = angularVelocity;

      states.append(bodyState);
    }

    py::dict frame;
    frame["simulation_time"] = simTimeSec;
    frame["states"] = states;
    return frame;
  }

  /**
   * @brief List all assets available in the registry
   * @return List of (asset_id, asset_name) tuples
   */
  std::vector<std::tuple<uint32_t, std::string>> listAssets()
  {
    const auto& cache = engine_.getAssetRegistry().getAssetCache();
    std::vector<std::tuple<uint32_t, std::string>> result;
    result.reserve(cache.size());
    for (const auto& asset : cache)
    {
      result.emplace_back(asset.getId(), asset.getName());
    }
    return result;
  }

  /**
   * @brief Get the collision hull vertices for an asset by ID
   * @param assetId ObjectRecord.id (asset type identifier)
   * @return List of (x, y, z) tuples, empty if asset not found
   */
  std::vector<std::tuple<double, double, double>>
  getCollisionVertices(uint32_t assetId)
  {
    auto geoOpt = engine_.getAssetRegistry().loadCollisionGeometry(assetId);
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

private:
  msd_sim::Engine engine_;
};

void bind_engine(py::module_& m)
{
  py::class_<EngineWrapper>(m, "Engine")
    .def(py::init<const std::string&>(),
         py::arg("db_path"),
         "Create Engine from assets database path")

    .def("spawn_inertial_object",
         &EngineWrapper::spawnInertialObject,
         py::arg("asset_name"),
         py::arg("x"),
         py::arg("y"),
         py::arg("z"),
         py::arg("pitch") = 0.0,
         py::arg("roll") = 0.0,
         py::arg("yaw") = 0.0,
         py::arg("mass") = 10.0,
         py::arg("restitution") = 0.5,
         py::arg("friction") = 0.5,
         "Spawn a dynamic inertial object. Returns dict with instance_id and "
         "asset_id.")

    .def("spawn_environment_object",
         &EngineWrapper::spawnEnvironmentObject,
         py::arg("asset_name"),
         py::arg("x"),
         py::arg("y"),
         py::arg("z"),
         py::arg("pitch") = 0.0,
         py::arg("roll") = 0.0,
         py::arg("yaw") = 0.0,
         "Spawn a static environment object. Returns dict with instance_id and "
         "asset_id.")

    .def("update",
         &EngineWrapper::update,
         py::arg("milliseconds"),
         "Set absolute simulation time in milliseconds. Pass increasing values "
         "each call (e.g. 16, 32, 48...) — this is NOT a delta.")

    .def("get_frame_state",
         &EngineWrapper::getFrameState,
         "Get the current simulation state for all dynamic bodies. Returns "
         "dict with simulation_time (float, seconds) and states (list of body "
         "state dicts).")

    .def("list_assets",
         &EngineWrapper::listAssets,
         "List all available assets as a list of (asset_id, name) tuples.")

    .def("get_collision_vertices",
         &EngineWrapper::getCollisionVertices,
         py::arg("asset_id"),
         "Get collision hull vertices for an asset as a list of (x, y, z) "
         "tuples.");
}
