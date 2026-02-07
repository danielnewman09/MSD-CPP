// Ticket: 0040d_contact_persistence_warm_starting

#ifndef MSD_SIM_PHYSICS_CONTACT_CACHE_HPP
#define MSD_SIM_PHYSICS_CONTACT_CACHE_HPP

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"

namespace msd_sim
{

/**
 * @brief Cached contact data for a single body pair
 *
 * Stores the solved lambda values, contact normal, and contact point
 * locations from the previous frame. Used for warm-starting the
 * Active Set Method solver.
 *
 * @ticket 0040d_contact_persistence_warm_starting
 */
struct CachedContact
{
  uint32_t bodyA_id;
  uint32_t bodyB_id;
  Vector3D normal;                    // Previous frame contact normal
  std::vector<double> lambdas;        // Previous frame solved lambda values
  std::vector<Coordinate> points;     // Previous frame contact midpoints
  uint32_t age{0};                    // Frames since last refresh
};

/**
 * @brief Contact cache for warm-starting the constraint solver
 *
 * Maps body pairs to their previous frame's solved contact data (lambda
 * values, contact normals, contact points). When contacts persist between
 * frames, the solver starts from the previous lambda, requiring only small
 * corrections. When contacts change significantly (normal rotation >15°),
 * the cache is invalidated.
 *
 * Cache keying uses a symmetric body pair key: (min(id_A, id_B), max(id_A,
 * id_B)) to ensure body order independence.
 *
 * Thread safety: Not thread-safe (single-threaded simulation)
 *
 * @ticket 0040d_contact_persistence_warm_starting
 */
class ContactCache
{
public:
  using BodyPairKey = std::pair<uint32_t, uint32_t>;

  ContactCache() = default;

  /**
   * @brief Look up cached lambdas for a body pair
   *
   * Matches current contact points to cached contact points by proximity
   * (nearest-neighbor within threshold). Returns reordered lambdas matching
   * current contact ordering. Unmatched contacts get lambda = 0.
   *
   * Invalidates cache entry if contact normal changed >15 degrees.
   *
   * @param bodyA Body A identifier
   * @param bodyB Body B identifier
   * @param currentNormal Current frame contact normal
   * @param currentPoints Current frame contact midpoints
   * @return Cached lambdas reordered to match current contacts, or empty if
   *         no valid cache entry exists
   */
  [[nodiscard]] std::vector<double> getWarmStart(
    uint32_t bodyA,
    uint32_t bodyB,
    const Vector3D& currentNormal,
    const std::vector<Coordinate>& currentPoints) const;

  /**
   * @brief Store solved lambdas for a body pair
   *
   * Overwrites any existing entry for this body pair.
   * Resets age to 0 (freshly updated).
   *
   * @param bodyA Body A identifier
   * @param bodyB Body B identifier
   * @param normal Contact normal for this frame
   * @param lambdas Solved lambda values
   * @param points Contact midpoints for this frame
   */
  void update(uint32_t bodyA,
              uint32_t bodyB,
              const Vector3D& normal,
              const std::vector<double>& lambdas,
              const std::vector<Coordinate>& points);

  /**
   * @brief Remove entries older than maxAge frames
   *
   * Called once per frame after advanceFrame(). Entries that have not been
   * refreshed (via update()) for maxAge frames are removed.
   *
   * @param maxAge Maximum frames without refresh before expiry (default: 10)
   */
  void expireOldEntries(uint32_t maxAge = 10);

  /**
   * @brief Increment age of all entries
   *
   * Call once per frame at the end of updateCollisions(). Entries refreshed
   * via update() in the current frame will have age 0; others will increment.
   */
  void advanceFrame();

  /**
   * @brief Clear all cached data
   */
  void clear();

  /**
   * @brief Number of cached body pair entries
   */
  [[nodiscard]] size_t size() const;

  // Rule of Five
  ContactCache(const ContactCache&) = default;
  ContactCache& operator=(const ContactCache&) = default;
  ContactCache(ContactCache&&) noexcept = default;
  ContactCache& operator=(ContactCache&&) noexcept = default;
  ~ContactCache() = default;

private:
  /**
   * @brief Create symmetric key from body pair
   * @return (min(a,b), max(a,b))
   */
  [[nodiscard]] static BodyPairKey makeKey(uint32_t a, uint32_t b);

  struct PairHash
  {
    size_t operator()(const BodyPairKey& p) const
    {
      size_t seed = std::hash<uint32_t>{}(p.first);
      // Golden ratio hash_combine (Boost pattern) for better distribution
      seed ^= std::hash<uint32_t>{}(p.second) + 0x9e3779b9 +
              (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  std::unordered_map<BodyPairKey, CachedContact, PairHash> cache_;

  static constexpr double kNormalThreshold{0.966};   // cos(15 degrees)
  static constexpr double kPointMatchRadius{0.02};    // 2cm matching radius
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONTACT_CACHE_HPP
