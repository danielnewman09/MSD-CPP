// Prototype P3: Numerical Stability
// Ticket: 0024_angular_coordinate
// Purpose: Test long-duration simulation to compare angle drift with different
// normalization frequencies

#include <Eigen/Dense>
#include <cmath>
#include <format>
#include <iostream>
#include <vector>

// Normalization function
inline double normalizeAngle(double rad)
{
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

// Strategy 1: Normalize every access (lazy)
class AngularCoordinate_Lazy : public msd_sim::Vector3D
{
public:
  AngularCoordinate_Lazy() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularCoordinate_Lazy(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Lazy(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Lazy& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  double pitch() const
  {
    return normalizeAngle((*this)[0]);
  }
  double roll() const
  {
    return normalizeAngle((*this)[1]);
  }
  double yaw() const
  {
    return normalizeAngle((*this)[2]);
  }
};

// Strategy 2: Normalize periodically (every N steps)
class AngularCoordinate_Periodic : public msd_sim::Vector3D
{
public:
  AngularCoordinate_Periodic()
    : msd_sim::Vector3D{0.0, 0.0, 0.0}, normalizePeriod_{1000}, stepCount_{0}
  {
  }

  AngularCoordinate_Periodic(double pitch,
                             double roll,
                             double yaw,
                             size_t period = 1000)
    : msd_sim::Vector3D{pitch, roll, yaw},
      normalizePeriod_{period},
      stepCount_{0}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Periodic(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}, normalizePeriod_{1000}, stepCount_{0}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Periodic& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    stepCount_++;
    if (stepCount_ >= normalizePeriod_)
    {
      normalizeInPlace();
      stepCount_ = 0;
    }
    return *this;
  }

  double pitch() const
  {
    return (*this)[0];
  }  // No normalization on access
  double roll() const
  {
    return (*this)[1];
  }
  double yaw() const
  {
    return (*this)[2];
  }

  void normalizeInPlace()
  {
    (*this)[0] = normalizeAngle((*this)[0]);
    (*this)[1] = normalizeAngle((*this)[1]);
    (*this)[2] = normalizeAngle((*this)[2]);
  }

private:
  size_t normalizePeriod_;
  mutable size_t stepCount_;
};

// Strategy 3: Never normalize
class AngularCoordinate_Never : public msd_sim::Vector3D
{
public:
  AngularCoordinate_Never() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularCoordinate_Never(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Never(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Never& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  double pitch() const
  {
    return (*this)[0];
  }
  double roll() const
  {
    return (*this)[1];
  }
  double yaw() const
  {
    return (*this)[2];
  }
};

// Simulation harness
template <typename AngularType>
struct SimulationResult
{
  std::string name;
  double finalPitch;
  double finalRoll;
  double finalYaw;
  double maxPitch;
  double maxRoll;
  double maxYaw;
  size_t overflowCount;  // Number of times angle exceeded ±1000 radians
};

template <typename AngularType>
SimulationResult<AngularType> runSimulation(const std::string& name,
                                            size_t timeSteps)
{
  SimulationResult<AngularType> result;
  result.name = name;
  result.overflowCount = 0;

  // Initial state
  AngularType orientation{0.0, 0.0, 0.0};

  // Angular velocity (constant spin around all axes)
  msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};  // rad/s
  double dt = 0.01;                                    // 10ms timestep

  double maxPitch = 0.0;
  double maxRoll = 0.0;
  double maxYaw = 0.0;

  // Run simulation
  for (size_t i = 0; i < timeSteps; ++i)
  {
    // Euler integration: orientation += angularVelocity * dt
    orientation = orientation + angularVelocity * dt;

    // Track maximum values
    double pitch = orientation.pitch();
    double roll = orientation.roll();
    double yaw = orientation.yaw();

    maxPitch = std::max(maxPitch, std::abs(pitch));
    maxRoll = std::max(maxRoll, std::abs(roll));
    maxYaw = std::max(maxYaw, std::abs(yaw));

    // Check for overflow (angle magnitude > 1000 radians)
    if (std::abs(pitch) > 1000.0 || std::abs(roll) > 1000.0 ||
        std::abs(yaw) > 1000.0)
    {
      result.overflowCount++;
    }
  }

  result.finalPitch = orientation.pitch();
  result.finalRoll = orientation.roll();
  result.finalYaw = orientation.yaw();
  result.maxPitch = maxPitch;
  result.maxRoll = maxRoll;
  result.maxYaw = maxYaw;

  return result;
}

int main()
{
  constexpr size_t timeSteps = 10000000;             // 10 million steps
  constexpr double dt = 0.01;                        // 10ms
  constexpr double simulationTime = timeSteps * dt;  // seconds

  std::cout << "=== P3: Numerical Stability Test ===\n\n";
  std::cout << std::format("Simulation parameters:\n");
  std::cout << std::format("  Time steps: {}\n", timeSteps);
  std::cout << std::format("  Time step (dt): {:.3f}s\n", dt);
  std::cout << std::format("  Total simulation time: {:.1f}s ({:.1f} hours)\n",
                           simulationTime,
                           simulationTime / 3600.0);
  std::cout << std::format("  Angular velocity: (0.1, 0.05, 0.15) rad/s\n");
  std::cout << std::format("  Expected revolutions per axis:\n");
  std::cout << std::format("    Pitch: {:.1f} revolutions\n",
                           (0.1 * simulationTime) / (2.0 * M_PI));
  std::cout << std::format("    Roll:  {:.1f} revolutions\n",
                           (0.05 * simulationTime) / (2.0 * M_PI));
  std::cout << std::format("    Yaw:   {:.1f} revolutions\n\n",
                           (0.15 * simulationTime) / (2.0 * M_PI));

  // Run simulations
  std::cout << "Running simulations (this may take a minute)...\n\n";

  auto lazyResult = runSimulation<AngularCoordinate_Lazy>(
    "Lazy (normalize on every access)", timeSteps);
  auto periodic1000Result = runSimulation<AngularCoordinate_Periodic>(
    "Periodic (every 1000 steps)", timeSteps);
  auto periodic10000Result = runSimulation<AngularCoordinate_Periodic>(
    "Periodic (every 10000 steps)", timeSteps);
  auto neverResult =
    runSimulation<AngularCoordinate_Never>("Never normalize", timeSteps);

  // Print results
  std::cout << "=== Final Angle Values ===\n\n";
  std::cout << std::format(
    "{:<35} {:>12} {:>12} {:>12}\n", "Strategy", "Pitch", "Roll", "Yaw");
  std::cout << std::format("{:-<71}\n", "");

  auto printRow = [&](const auto& r)
  {
    std::cout << std::format("{:<35} {:>11.6f} {:>11.6f} {:>11.6f}\n",
                             r.name,
                             r.finalPitch,
                             r.finalRoll,
                             r.finalYaw);
  };

  printRow(lazyResult);
  printRow(periodic1000Result);
  printRow(periodic10000Result);
  printRow(neverResult);

  std::cout << "\n=== Maximum Angle Magnitudes ===\n\n";
  std::cout << std::format("{:<35} {:>12} {:>12} {:>12}\n",
                           "Strategy",
                           "Max Pitch",
                           "Max Roll",
                           "Max Yaw");
  std::cout << std::format("{:-<71}\n", "");

  auto printMaxRow = [&](const auto& r)
  {
    std::cout << std::format("{:<35} {:>11.6f} {:>11.6f} {:>11.6f}\n",
                             r.name,
                             r.maxPitch,
                             r.maxRoll,
                             r.maxYaw);
  };

  printMaxRow(lazyResult);
  printMaxRow(periodic1000Result);
  printMaxRow(periodic10000Result);
  printMaxRow(neverResult);

  std::cout << "\n=== Overflow Analysis ===\n\n";
  std::cout << std::format("{:<35} {:>15}\n", "Strategy", "Overflow Count");
  std::cout << std::format("{:-<51}\n", "");

  auto printOverflow = [&](const auto& r)
  { std::cout << std::format("{:<35} {:>15}\n", r.name, r.overflowCount); };

  printOverflow(lazyResult);
  printOverflow(periodic1000Result);
  printOverflow(periodic10000Result);
  printOverflow(neverResult);

  // Recommendations
  std::cout << "\n=== Stability Analysis ===\n\n";

  bool lazyStable =
    (lazyResult.maxPitch < 2.0 * M_PI && lazyResult.maxRoll < 2.0 * M_PI &&
     lazyResult.maxYaw < 2.0 * M_PI);
  bool periodic1000Stable =
    (periodic1000Result.maxPitch < 100.0 &&
     periodic1000Result.maxRoll < 100.0 && periodic1000Result.maxYaw < 100.0);
  bool periodic10000Stable = (periodic10000Result.maxPitch < 1000.0 &&
                              periodic10000Result.maxRoll < 1000.0 &&
                              periodic10000Result.maxYaw < 1000.0);

  if (lazyStable)
  {
    std::cout
      << "✅ LAZY normalization: Angles remain bounded within (-π, π]\n";
    std::cout << "   Maximum angle magnitude: "
              << std::format("{:.6f} rad\n",
                             std::max({lazyResult.maxPitch,
                                       lazyResult.maxRoll,
                                       lazyResult.maxYaw}));
  }
  else
  {
    std::cout << "❌ LAZY normalization: Angles exceeded expected bounds!\n";
  }

  if (periodic1000Stable)
  {
    std::cout
      << "✅ PERIODIC (1000 steps) normalization: Angles remain bounded\n";
    std::cout << "   Maximum angle magnitude: "
              << std::format("{:.6f} rad\n",
                             std::max({periodic1000Result.maxPitch,
                                       periodic1000Result.maxRoll,
                                       periodic1000Result.maxYaw}));
  }
  else
  {
    std::cout << "⚠️  PERIODIC (1000 steps) normalization: Angles growing, may "
                 "overflow\n";
  }

  if (periodic10000Stable)
  {
    std::cout
      << "✅ PERIODIC (10000 steps) normalization: Angles remain bounded\n";
    std::cout << "   Maximum angle magnitude: "
              << std::format("{:.6f} rad\n",
                             std::max({periodic10000Result.maxPitch,
                                       periodic10000Result.maxRoll,
                                       periodic10000Result.maxYaw}));
  }
  else
  {
    std::cout << "⚠️  PERIODIC (10000 steps) normalization: Angles growing, may "
                 "overflow\n";
  }

  std::cout << "\n❌ NEVER normalize: Angles grow unbounded ("
            << std::format("{:.1f} rad max)\n",
                           std::max({neverResult.maxPitch,
                                     neverResult.maxRoll,
                                     neverResult.maxYaw}));
  std::cout << "   This is expected - angles accumulate over "
            << std::format("{:.1f} revolutions\n",
                           std::max({0.1 * simulationTime,
                                     0.05 * simulationTime,
                                     0.15 * simulationTime}) /
                             (2.0 * M_PI));

  std::cout << "\n=== Recommendations ===\n\n";

  std::cout << "For AngularCoordinate (orientation):\n";
  if (lazyStable)
  {
    std::cout << "  ✅ Use LAZY normalization (normalize on access)\n";
    std::cout << "     Keeps angles bounded, no drift over long simulations\n";
  }
  else
  {
    std::cout
      << "  ⚠️  LAZY normalization may have issues - investigate further\n";
  }

  std::cout << "\nFor AngularRate (velocity/acceleration):\n";
  std::cout << "  ✅ NEVER normalize\n";
  std::cout << "     Angular rates should not be constrained to (-π, π]\n";
  std::cout << "     Values like 720°/s (4π rad/s) are valid and meaningful\n";

  return 0;
}
