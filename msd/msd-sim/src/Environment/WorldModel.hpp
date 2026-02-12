#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <chrono>
#include <memory>
#include <vector>

#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include "msd-sim/src/Physics/Integration/Integrator.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"

namespace msd_sim
{

// Forward declare DataRecorder to avoid including in header
class DataRecorder;

/**
 * @brief Container and manager for all simulation objects
 *
 * WorldModel uses a unified storage approach with index-based iteration
 * for efficient physics updates and collision detection.
 *
 * The WorldModel maintains:
 * - A single vector of all objects (graphical, inertial, environmental,
 * boundary)
 * - Cached index lists for efficient iteration over specific object types
 * - Current simulation time
 *
 * Efficient iteration patterns:
 * @code
 * // Physics update - only iterate objects with physics
 * for (size_t idx : worldModel.getPhysicsObjectIndices()) {
 *   Object& obj = worldModel.getObject(idx);
 *   // Update physics...
 * }
 *
 * // Collision detection - only iterate objects with collision hulls
 * for (size_t idx : worldModel.getCollisionObjectIndices()) {
 *   // Check collisions...
 * }
 *
 * // Rendering - only iterate objects with visual geometry
 * for (size_t idx : worldModel.getRenderObjectIndices()) {
 *   // Render...
 * }
 * @endcode
 */
class WorldModel
{
public:
  /**
   * @brief Construct WorldModel with default integrator and gravity
   *
   * Initializes:
   * - SemiImplicitEulerIntegrator as default integrator
   * - GravityPotential with (0, 0, -9.81) m/s² as default potential
   *
   * @ticket 0030_lagrangian_quaternion_physics
   */
  WorldModel();
  ~WorldModel();  // Defined in .cpp for unique_ptr forward-declared type

  // ========== Object Management ==========

  /**
   * @brief Spawn a new object in the world
   *
   * Adds the object to the world and updates internal index caches.
   *
   * @param object Object to add (uses move semantics)
   * @return Index of added object
   */
  const AssetInertial& spawnObject(uint32_t assetId,
                                   ConvexHull& hull,
                                   const ReferenceFrame& origin);

  /**
   * @brief Spawn a new object with specified mass.
   *
   * @param assetId Asset type identifier
   * @param hull Collision hull for the object
   * @param mass Mass in kilograms [kg]
   * @param origin Initial reference frame
   * @return Reference to the spawned object
   *
   * @ticket 0039b_linear_collision_test_suite
   */
  const AssetInertial& spawnObject(uint32_t assetId,
                                   ConvexHull& hull,
                                   double mass,
                                   const ReferenceFrame& origin);

  /**
   * @brief Spawn a new static environment object in the world
   *
   * Environment objects are static (immovable) and participate in collision
   * detection but have no dynamics. Dynamic objects will bounce off them.
   *
   * @param assetId Asset type identifier
   * @param hull Collision hull for the object
   * @param origin Initial position and orientation
   * @return Reference to the spawned environment object
   */
  const AssetEnvironment& spawnEnvironmentObject(uint32_t assetId,
                                                 ConvexHull& hull,
                                                 const ReferenceFrame& origin);

  /**
   * @brief Get all objects (const)
   */
  const std::vector<AssetInertial>& getInertialAssets() const
  {
    return inertialAssets_;
  }

  const AssetInertial& getObject(uint32_t instanceId) const;

  AssetInertial& getObject(uint32_t instanceId);

  /**
   * @brief Remove object by index
   *
   * Note: This invalidates indices > removed index.
   * Consider marking objects for removal and cleaning up between frames.
   *
   * @param index Index of object to remove
   * @throws std::out_of_range if index is invalid
   */
  void removeObject(uint32_t instanceId);

  /**
   * @brief Get object count
   */
  size_t getObjectCount() const
  {
    return inertialAssets_.size();
  }

  /**
   * @brief Clear all objects from the world
   */
  void clearObjects();

  // ========== Efficient Iteration Helpers ==========

  /**
   * @brief Get indices of all objects with physics components
   *
   * Use this for physics integration loops.
   */
  const std::vector<AssetEnvironment>& getEnvironmentalObjects() const
  {
    return environmentalAssets_;
  }

  // ========== Simulation Update ==========

  /**
   * @brief Update world simulation
   *
   * Performs:
   * 1. Physics integration for objects with PhysicsComponent
   * 2. Collision detection for objects with collision hulls
   * 3. Collision resolution
   * 4. Updates simulation time
   *
   * @param deltaTime Time step for integration
   */
  void update(std::chrono::milliseconds simTime);

  /**
   * @brief Get current simulation time
   */
  std::chrono::milliseconds getTime() const
  {
    return time_;
  }

  // ========== NEW: Potential Energy Configuration (ticket 0030) ==========

  /**
   * @brief Add a potential energy field to the world.
   *
   * Potential energies compute generalized forces from energy gradients.
   * Multiple potentials can be added and their forces are summed.
   *
   * @param energy Potential energy implementation (ownership transferred)
   *
   * @ticket 0030_lagrangian_quaternion_physics
   */
  void addPotentialEnergy(std::unique_ptr<PotentialEnergy> energy);

  /**
   * @brief Clear all potential energies.
   *
   * @ticket 0030_lagrangian_quaternion_physics
   */
  void clearPotentialEnergies();

  // ========== NEW: Integrator Configuration (ticket 0030) ==========

  /**
   * @brief Set the numerical integrator for physics updates.
   *
   * The integrator handles the mathematical integration of equations
   * of motion. Allows swapping integration schemes (Euler, RK4, etc.).
   *
   * @param integrator Integrator implementation (ownership transferred)
   *
   * @ticket 0030_lagrangian_quaternion_physics
   */
  void setIntegrator(std::unique_ptr<Integrator> integrator);

  /**
   * @brief Get the world gravity vector (deprecated).
   * @return Gravity acceleration vector [m/s²]
   * @deprecated Use PotentialEnergy interface instead
   */
  [[deprecated("Use PotentialEnergy interface instead")]]
  const Coordinate& getGravity() const;

  // ========== Platform Management (Legacy) ==========

  /**
   * @brief Add a platform to the world
   * @param platform Platform to add
   */
  void addPlatform(Platform&& platform);

  /**
   * @brief Get all platforms (const)
   */
  const std::vector<Platform>& getPlatforms() const
  {
    return platforms_;
  }

  /**
   * @brief Get all platforms (mutable)
   *
   * Allows modification of platforms for input command updates.
   */
  std::vector<Platform>& getPlatforms()
  {
    return platforms_;
  }

  /**
   * @brief Get next platform ID
   * @return Next available platform ID
   */
  uint32_t getNextPlatformId() const
  {
    return nextPlatformId_++;
  }

  /**
   * Retrieve the next inertial asset id and increment the counter
   */
  uint32_t getInertialAssetId();

  // ========== Data Recording (ticket 0038) ==========

  /**
   * @brief Enable simulation data recording to SQLite database
   *
   * Creates a DataRecorder instance that operates on a background thread,
   * periodically flushing buffered records. Recording begins immediately
   * and continues until disableRecording() is called or WorldModel is destroyed.
   *
   * @param dbPath Path to SQLite database file (created if doesn't exist)
   * @param flushInterval How often to flush buffered records to disk
   *
   * @throws std::runtime_error if database cannot be opened
   * @ticket 0038_simulation_data_recorder
   */
  void enableRecording(const std::string& dbPath,
                       std::chrono::milliseconds flushInterval =
                         std::chrono::milliseconds{100});

  /**
   * @brief Disable simulation data recording
   *
   * Flushes all pending records and stops the recorder thread.
   * After this call, no more recording occurs until enableRecording() is called.
   *
   * @ticket 0038_simulation_data_recorder
   */
  void disableRecording();

private:
  // ========== Internal Update Methods ==========

  /**
   * @brief Update physics for all dynamic objects
   * @param dt Time step in seconds
   */
  void updatePhysics(double dt);

  /**
   * @brief Detect and resolve collisions using constraint-based approach
   *
   * Creates transient contact constraints from collision results, solves them
   * via Projected Gauss-Seidel (PGS), and applies constraint forces to bodies.
   *
   * @param dt Timestep [s] (needed for PGS solver force conversion)
   *
   * @ticket 0032_contact_constraint_refactor
   */
  void updateCollisions(double dt);

  /**
   * @brief Record current frame to database if recording is enabled
   *
   * Creates a SimulationFrameRecord with timestamp, then records all inertial
   * assets' states with FK reference to the frame.
   *
   * @ticket 0038_simulation_data_recorder
   */
  void recordCurrentFrame();

  /**
   * @brief Record collision results for this frame
   *
   * Reads collision pairs directly from CollisionPipeline::getCollisions()
   * and writes CollisionResultRecord for each pair.
   *
   * @param frameId Frame ID for FK reference
   * @ticket 0056b1_eliminate_snapshot_layer
   */
  void recordCollisions(uint32_t frameId);

  /**
   * @brief Record solver diagnostics for this frame
   *
   * Reads solver data directly from CollisionPipeline::getSolverData()
   * and writes SolverDiagnosticRecord.
   *
   * @param frameId Frame ID for FK reference
   * @ticket 0056b1_eliminate_snapshot_layer
   */
  void recordSolverDiagnostics(uint32_t frameId);

  /**
   * @brief Record static asset data for a single asset
   *
   * Creates AssetInertialStaticRecord from asset's static state and writes
   * to database via DataRecorder. The record's auto-incremented id becomes
   * the FK target for per-frame records.
   *
   * @param asset The asset to record static data for
   * @ticket 0056i_static_asset_recording_and_fk
   */
  void recordStaticData(const AssetInertial& asset);

  /**
   * @brief Backfill static records for all existing assets
   *
   * Called when recording is enabled after assets have already been spawned.
   * Iterates all inertial assets and records static data for each.
   *
   * @ticket 0056i_static_asset_recording_and_fk
   */
  void backfillStaticData();

  // ========== Data ==========


  // Physical assets are those that are "dumb" objects which are
  // affected by physics, but do not have any intelligence
  // e.g. a ball that bounces around
  std::vector<AssetInertial> inertialAssets_;

  // Environmental assets are those that are immovable, but
  // still affect other assets that interact with them.
  // e.g. a floor that a bounces off of
  std::vector<AssetEnvironment> environmentalAssets_;

  // Legacy platform support
  std::vector<Platform> platforms_;
  mutable uint32_t nextPlatformId_{0};  // Mutable for getNextPlatformId() const

  // Current simulation time
  std::chrono::milliseconds time_{0};

  uint32_t inertialAssetIdCounter_{0};
  uint32_t environmentAssetIdCounter_{0};

  // NEW: Gravity (deprecated, ticket 0023a, replaced by potentialEnergies_ in
  // ticket 0030)
  [[deprecated("Use potentialEnergies_ instead")]]
  Coordinate gravity_{0.0, 0.0, -9.81};

  // Collision detection and response (ticket 0044 — integrated pipeline)
  CollisionPipeline collisionPipeline_;

  // NEW: Potential energies and integrator (ticket 0030)
  std::vector<std::unique_ptr<PotentialEnergy>> potentialEnergies_;
  std::unique_ptr<Integrator> integrator_;

  // NEW: Data recorder (ticket 0038)
  std::unique_ptr<DataRecorder> dataRecorder_;

  // NEW: Energy tracking (ticket 0039a)
  double previousSystemEnergy_{0.0};
  bool collisionActiveThisFrame_{false};
};

}  // namespace msd_sim

#endif  // WORLD_MODEL_HPP
