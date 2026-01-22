// Prototype P1c: Hybrid Eager Normalization
// Ticket: 0024_angular_coordinate
// Purpose: Test eager normalization with early-exit optimization in normalize function

#include <Eigen/Dense>
#include <benchmark/benchmark.h>
#include <cmath>
#include <random>
#include <vector>

// =============================================================================
// Normalization Functions
// =============================================================================

// Original: Always calls fmod()
inline double normalizeAngle_Always(double rad)
{
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

// Optimized: Early exit if already in range (-pi, pi]
inline double normalizeAngle_EarlyExit(double rad)
{
  if (rad > -M_PI && rad <= M_PI)
  {
    return rad;  // Fast path: already normalized
  }
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

// =============================================================================
// Strategy 1: Eager Always (current design - always calls fmod)
// =============================================================================

class AngularCoordinate_EagerAlways : public Eigen::Vector3d
{
public:
  AngularCoordinate_EagerAlways() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_EagerAlways(double pitch, double roll, double yaw)
    : Eigen::Vector3d{normalizeAngle_Always(pitch), normalizeAngle_Always(roll),
                      normalizeAngle_Always(yaw)}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_EagerAlways(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
    normalizeInPlace();
  }

  template <typename OtherDerived>
  AngularCoordinate_EagerAlways& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    normalizeInPlace();
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

private:
  void normalizeInPlace()
  {
    (*this)[0] = normalizeAngle_Always((*this)[0]);
    (*this)[1] = normalizeAngle_Always((*this)[1]);
    (*this)[2] = normalizeAngle_Always((*this)[2]);
  }
};

// =============================================================================
// Strategy 2: Eager with Early Exit (hybrid - skip fmod if in range)
// =============================================================================

class AngularCoordinate_EagerEarlyExit : public Eigen::Vector3d
{
public:
  AngularCoordinate_EagerEarlyExit() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_EagerEarlyExit(double pitch, double roll, double yaw)
    : Eigen::Vector3d{normalizeAngle_EarlyExit(pitch), normalizeAngle_EarlyExit(roll),
                      normalizeAngle_EarlyExit(yaw)}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_EagerEarlyExit(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
    normalizeInPlace();
  }

  template <typename OtherDerived>
  AngularCoordinate_EagerEarlyExit& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    normalizeInPlace();
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

private:
  void normalizeInPlace()
  {
    (*this)[0] = normalizeAngle_EarlyExit((*this)[0]);
    (*this)[1] = normalizeAngle_EarlyExit((*this)[1]);
    (*this)[2] = normalizeAngle_EarlyExit((*this)[2]);
  }
};

// =============================================================================
// Strategy 3: Never Normalize (baseline)
// =============================================================================

class AngularCoordinate_Never : public Eigen::Vector3d
{
public:
  AngularCoordinate_Never() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_Never(double pitch, double roll, double yaw)
    : Eigen::Vector3d{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Never(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Never& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
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

// =============================================================================
// Test Data
// =============================================================================

constexpr size_t kDataSize = 1024;

// =============================================================================
// Benchmark: Construction with OUT OF RANGE data
// =============================================================================

static void BM_Construction_Never_OutOfRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);  // Out of range

  for (auto _ : state)
  {
    AngularCoordinate_Never a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_Never_OutOfRange);

static void BM_Construction_EagerAlways_OutOfRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);

  for (auto _ : state)
  {
    AngularCoordinate_EagerAlways a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_EagerAlways_OutOfRange);

static void BM_Construction_EagerEarlyExit_OutOfRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);

  for (auto _ : state)
  {
    AngularCoordinate_EagerEarlyExit a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_EagerEarlyExit_OutOfRange);

// =============================================================================
// Benchmark: Construction with IN RANGE data
// =============================================================================

static void BM_Construction_Never_InRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01, M_PI - 0.01);  // In range

  for (auto _ : state)
  {
    AngularCoordinate_Never a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_Never_InRange);

static void BM_Construction_EagerAlways_InRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01, M_PI - 0.01);

  for (auto _ : state)
  {
    AngularCoordinate_EagerAlways a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_EagerAlways_InRange);

static void BM_Construction_EagerEarlyExit_InRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01, M_PI - 0.01);

  for (auto _ : state)
  {
    AngularCoordinate_EagerEarlyExit a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_EagerEarlyExit_InRange);

// =============================================================================
// Benchmark: Assignment with OUT OF RANGE data
// =============================================================================

static void BM_Assignment_Never_OutOfRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  AngularCoordinate_Never a{0, 0, 0};
  Eigen::Vector3d source{0, 0, 0};

  for (auto _ : state)
  {
    source = Eigen::Vector3d{dis(gen), dis(gen), dis(gen)};
    a = source;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Assignment_Never_OutOfRange);

static void BM_Assignment_EagerAlways_OutOfRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  AngularCoordinate_EagerAlways a{0, 0, 0};
  Eigen::Vector3d source{0, 0, 0};

  for (auto _ : state)
  {
    source = Eigen::Vector3d{dis(gen), dis(gen), dis(gen)};
    a = source;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Assignment_EagerAlways_OutOfRange);

static void BM_Assignment_EagerEarlyExit_OutOfRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  AngularCoordinate_EagerEarlyExit a{0, 0, 0};
  Eigen::Vector3d source{0, 0, 0};

  for (auto _ : state)
  {
    source = Eigen::Vector3d{dis(gen), dis(gen), dis(gen)};
    a = source;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Assignment_EagerEarlyExit_OutOfRange);

// =============================================================================
// Benchmark: Assignment with IN RANGE data
// =============================================================================

static void BM_Assignment_Never_InRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01, M_PI - 0.01);
  AngularCoordinate_Never a{0, 0, 0};
  Eigen::Vector3d source{0, 0, 0};

  for (auto _ : state)
  {
    source = Eigen::Vector3d{dis(gen), dis(gen), dis(gen)};
    a = source;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Assignment_Never_InRange);

static void BM_Assignment_EagerAlways_InRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01, M_PI - 0.01);
  AngularCoordinate_EagerAlways a{0, 0, 0};
  Eigen::Vector3d source{0, 0, 0};

  for (auto _ : state)
  {
    source = Eigen::Vector3d{dis(gen), dis(gen), dis(gen)};
    a = source;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Assignment_EagerAlways_InRange);

static void BM_Assignment_EagerEarlyExit_InRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01, M_PI - 0.01);
  AngularCoordinate_EagerEarlyExit a{0, 0, 0};
  Eigen::Vector3d source{0, 0, 0};

  for (auto _ : state)
  {
    source = Eigen::Vector3d{dis(gen), dis(gen), dis(gen)};
    a = source;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Assignment_EagerEarlyExit_InRange);

// =============================================================================
// Benchmark: Realistic Physics Simulation Pattern
// Small incremental updates that keep values near the boundary
// =============================================================================

static void BM_PhysicsUpdate_EagerAlways(benchmark::State& state)
{
  AngularCoordinate_EagerAlways orientation{0.0, 0.0, 0.0};
  Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};  // rad/s
  double dt = 0.01;  // 10ms timestep

  for (auto _ : state)
  {
    // Typical physics: orientation += velocity * dt
    // This produces small increments that rarely exceed bounds
    orientation = orientation + angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsUpdate_EagerAlways);

static void BM_PhysicsUpdate_EagerEarlyExit(benchmark::State& state)
{
  AngularCoordinate_EagerEarlyExit orientation{0.0, 0.0, 0.0};
  Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation = orientation + angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsUpdate_EagerEarlyExit);

static void BM_PhysicsUpdate_Never(benchmark::State& state)
{
  AngularCoordinate_Never orientation{0.0, 0.0, 0.0};
  Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation = orientation + angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsUpdate_Never);

// =============================================================================
// Benchmark: Full Cycle (construct + 10 reads)
// =============================================================================

static void BM_FullCycle_EagerAlways_OutOfRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_EagerAlways a{dis(gen), dis(gen), dis(gen)};
    for (int i = 0; i < 10; ++i)
    {
      sum += a.pitch() + a.roll() + a.yaw();
    }
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_FullCycle_EagerAlways_OutOfRange);

static void BM_FullCycle_EagerEarlyExit_OutOfRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_EagerEarlyExit a{dis(gen), dis(gen), dis(gen)};
    for (int i = 0; i < 10; ++i)
    {
      sum += a.pitch() + a.roll() + a.yaw();
    }
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_FullCycle_EagerEarlyExit_OutOfRange);

static void BM_FullCycle_EagerAlways_InRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01, M_PI - 0.01);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_EagerAlways a{dis(gen), dis(gen), dis(gen)};
    for (int i = 0; i < 10; ++i)
    {
      sum += a.pitch() + a.roll() + a.yaw();
    }
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_FullCycle_EagerAlways_InRange);

static void BM_FullCycle_EagerEarlyExit_InRange(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01, M_PI - 0.01);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_EagerEarlyExit a{dis(gen), dis(gen), dis(gen)};
    for (int i = 0; i < 10; ++i)
    {
      sum += a.pitch() + a.roll() + a.yaw();
    }
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_FullCycle_EagerEarlyExit_InRange);

// =============================================================================
// Benchmark: Arithmetic Operations (result needs normalization)
// =============================================================================

static void BM_Arithmetic_EagerAlways(benchmark::State& state)
{
  std::vector<AngularCoordinate_EagerAlways> data;
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-2.0, 2.0);  // Within range initially
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }

  size_t idx = 0;
  AngularCoordinate_EagerAlways result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = data[idx % kDataSize];
    const auto& b = data[(idx + 1) % kDataSize];
    const auto& c = data[(idx + 2) % kDataSize];
    // Result of a + b * 2 - c might exceed bounds
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_EagerAlways);

static void BM_Arithmetic_EagerEarlyExit(benchmark::State& state)
{
  std::vector<AngularCoordinate_EagerEarlyExit> data;
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-2.0, 2.0);
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }

  size_t idx = 0;
  AngularCoordinate_EagerEarlyExit result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = data[idx % kDataSize];
    const auto& b = data[(idx + 1) % kDataSize];
    const auto& c = data[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_EagerEarlyExit);

static void BM_Arithmetic_Never(benchmark::State& state)
{
  std::vector<AngularCoordinate_Never> data;
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-2.0, 2.0);
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }

  size_t idx = 0;
  AngularCoordinate_Never result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = data[idx % kDataSize];
    const auto& b = data[(idx + 1) % kDataSize];
    const auto& c = data[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_Never);

BENCHMARK_MAIN();
