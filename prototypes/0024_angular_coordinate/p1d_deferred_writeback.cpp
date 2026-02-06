// Prototype P1d: Deferred Writeback Normalization
// Ticket: 0024_angular_coordinate
// Purpose: Test lazy normalization with writeback in assignment (not accessor)
//          and variable threshold bounds

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

// =============================================================================
// Normalization Function
// =============================================================================

inline double normalizeAngle(double rad)
{
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

// =============================================================================
// Strategy 1: Eager Always (baseline - normalize on every write)
// =============================================================================

class AngularCoordinate_EagerAlways : public msd_sim::Vector3D
{
public:
  AngularCoordinate_EagerAlways() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_EagerAlways(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{normalizeAngle(pitch),
                        normalizeAngle(roll),
                        normalizeAngle(yaw)}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_EagerAlways(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
    normalizeInPlace();
  }

  template <typename OtherDerived>
  AngularCoordinate_EagerAlways& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
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
    (*this)[0] = normalizeAngle((*this)[0]);
    (*this)[1] = normalizeAngle((*this)[1]);
    (*this)[2] = normalizeAngle((*this)[2]);
  }
};

// =============================================================================
// Strategy 2: Deferred Writeback with PI threshold
// Normalize in assignment only when |angle| > PI
// =============================================================================

class AngularCoordinate_DeferredPI : public msd_sim::Vector3D
{
public:
  static constexpr double kThreshold = M_PI;

  AngularCoordinate_DeferredPI() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_DeferredPI(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
    normalizeIfNeeded();
  }

  template <typename OtherDerived>
  AngularCoordinate_DeferredPI(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
    normalizeIfNeeded();
  }

  template <typename OtherDerived>
  AngularCoordinate_DeferredPI& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    normalizeIfNeeded();
    return *this;
  }

  // Fast accessor - no normalization, no check
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
  void normalizeIfNeeded()
  {
    // Only normalize components that exceed threshold
    if ((*this)[0] <= -kThreshold || (*this)[0] > kThreshold)
    {
      (*this)[0] = normalizeAngle((*this)[0]);
    }
    if ((*this)[1] <= -kThreshold || (*this)[1] > kThreshold)
    {
      (*this)[1] = normalizeAngle((*this)[1]);
    }
    if ((*this)[2] <= -kThreshold || (*this)[2] > kThreshold)
    {
      (*this)[2] = normalizeAngle((*this)[2]);
    }
  }
};

// =============================================================================
// Strategy 3: Deferred Writeback with 10*PI threshold
// Normalize only when |angle| > 10*PI (~5 revolutions)
// =============================================================================

class AngularCoordinate_Deferred10PI : public msd_sim::Vector3D
{
public:
  static constexpr double kThreshold = 10.0 * M_PI;

  AngularCoordinate_Deferred10PI() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_Deferred10PI(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
    normalizeIfNeeded();
  }

  template <typename OtherDerived>
  AngularCoordinate_Deferred10PI(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
    normalizeIfNeeded();
  }

  template <typename OtherDerived>
  AngularCoordinate_Deferred10PI& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    normalizeIfNeeded();
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
  void normalizeIfNeeded()
  {
    if ((*this)[0] <= -kThreshold || (*this)[0] > kThreshold)
    {
      (*this)[0] = normalizeAngle((*this)[0]);
    }
    if ((*this)[1] <= -kThreshold || (*this)[1] > kThreshold)
    {
      (*this)[1] = normalizeAngle((*this)[1]);
    }
    if ((*this)[2] <= -kThreshold || (*this)[2] > kThreshold)
    {
      (*this)[2] = normalizeAngle((*this)[2]);
    }
  }
};

// =============================================================================
// Strategy 4: Deferred Writeback with 100*PI threshold
// Normalize only when |angle| > 100*PI (~50 revolutions)
// =============================================================================

class AngularCoordinate_Deferred100PI : public msd_sim::Vector3D
{
public:
  static constexpr double kThreshold = 100.0 * M_PI;

  AngularCoordinate_Deferred100PI() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_Deferred100PI(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
    normalizeIfNeeded();
  }

  template <typename OtherDerived>
  AngularCoordinate_Deferred100PI(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
    normalizeIfNeeded();
  }

  template <typename OtherDerived>
  AngularCoordinate_Deferred100PI& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    normalizeIfNeeded();
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
  void normalizeIfNeeded()
  {
    if ((*this)[0] <= -kThreshold || (*this)[0] > kThreshold)
    {
      (*this)[0] = normalizeAngle((*this)[0]);
    }
    if ((*this)[1] <= -kThreshold || (*this)[1] > kThreshold)
    {
      (*this)[1] = normalizeAngle((*this)[1]);
    }
    if ((*this)[2] <= -kThreshold || (*this)[2] > kThreshold)
    {
      (*this)[2] = normalizeAngle((*this)[2]);
    }
  }
};

// =============================================================================
// Strategy 5: Never Normalize (baseline for accessor speed)
// =============================================================================

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

// =============================================================================
// Test Data
// =============================================================================

constexpr size_t kDataSize = 1024;

// =============================================================================
// Benchmark: Accessor Performance (should be identical for all deferred)
// =============================================================================

template <typename T>
std::vector<T> generateData()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-3.0, 3.0);  // Slightly outside PI
  std::vector<T> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}

static auto gDataEagerAlways = generateData<AngularCoordinate_EagerAlways>();
static auto gDataDeferredPI = generateData<AngularCoordinate_DeferredPI>();
static auto gDataDeferred10PI = generateData<AngularCoordinate_Deferred10PI>();
static auto gDataDeferred100PI =
  generateData<AngularCoordinate_Deferred100PI>();
static auto gDataNever = generateData<AngularCoordinate_Never>();

static void BM_Accessor_EagerAlways(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataEagerAlways[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_EagerAlways);

static void BM_Accessor_DeferredPI(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataDeferredPI[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_DeferredPI);

static void BM_Accessor_Deferred10PI(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataDeferred10PI[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Deferred10PI);

static void BM_Accessor_Deferred100PI(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataDeferred100PI[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Deferred100PI);

static void BM_Accessor_Never(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataNever[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Never);

// =============================================================================
// Benchmark: Construction with values slightly outside PI
// =============================================================================

static void BM_Construction_EagerAlways_NearPI(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-4.0, 4.0);  // Slightly > PI

  for (auto _ : state)
  {
    AngularCoordinate_EagerAlways a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_EagerAlways_NearPI);

static void BM_Construction_DeferredPI_NearPI(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-4.0, 4.0);

  for (auto _ : state)
  {
    AngularCoordinate_DeferredPI a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_DeferredPI_NearPI);

static void BM_Construction_Deferred10PI_NearPI(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-4.0, 4.0);

  for (auto _ : state)
  {
    AngularCoordinate_Deferred10PI a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_Deferred10PI_NearPI);

static void BM_Construction_Deferred100PI_NearPI(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-4.0, 4.0);

  for (auto _ : state)
  {
    AngularCoordinate_Deferred100PI a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_Deferred100PI_NearPI);

static void BM_Construction_Never_NearPI(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-4.0, 4.0);

  for (auto _ : state)
  {
    AngularCoordinate_Never a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_Never_NearPI);

// =============================================================================
// Benchmark: Physics Simulation (incremental updates)
// Values grow slowly, rarely exceed thresholds
// =============================================================================

static void BM_PhysicsUpdate_EagerAlways(benchmark::State& state)
{
  AngularCoordinate_EagerAlways orientation{0.0, 0.0, 0.0};
  msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation = orientation + angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsUpdate_EagerAlways);

static void BM_PhysicsUpdate_DeferredPI(benchmark::State& state)
{
  AngularCoordinate_DeferredPI orientation{0.0, 0.0, 0.0};
  msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation = orientation + angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsUpdate_DeferredPI);

static void BM_PhysicsUpdate_Deferred10PI(benchmark::State& state)
{
  AngularCoordinate_Deferred10PI orientation{0.0, 0.0, 0.0};
  msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation = orientation + angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsUpdate_Deferred10PI);

static void BM_PhysicsUpdate_Deferred100PI(benchmark::State& state)
{
  AngularCoordinate_Deferred100PI orientation{0.0, 0.0, 0.0};
  msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation = orientation + angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsUpdate_Deferred100PI);

static void BM_PhysicsUpdate_Never(benchmark::State& state)
{
  AngularCoordinate_Never orientation{0.0, 0.0, 0.0};
  msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation = orientation + angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsUpdate_Never);

// =============================================================================
// Benchmark: Arithmetic (a + b * 2 - c) - result may exceed bounds
// =============================================================================

static void BM_Arithmetic_EagerAlways(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_EagerAlways result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = gDataEagerAlways[idx % kDataSize];
    const auto& b = gDataEagerAlways[(idx + 1) % kDataSize];
    const auto& c = gDataEagerAlways[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_EagerAlways);

static void BM_Arithmetic_DeferredPI(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_DeferredPI result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = gDataDeferredPI[idx % kDataSize];
    const auto& b = gDataDeferredPI[(idx + 1) % kDataSize];
    const auto& c = gDataDeferredPI[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_DeferredPI);

static void BM_Arithmetic_Deferred10PI(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Deferred10PI result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = gDataDeferred10PI[idx % kDataSize];
    const auto& b = gDataDeferred10PI[(idx + 1) % kDataSize];
    const auto& c = gDataDeferred10PI[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_Deferred10PI);

static void BM_Arithmetic_Deferred100PI(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Deferred100PI result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = gDataDeferred100PI[idx % kDataSize];
    const auto& b = gDataDeferred100PI[(idx + 1) % kDataSize];
    const auto& c = gDataDeferred100PI[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_Deferred100PI);

static void BM_Arithmetic_Never(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Never result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = gDataNever[idx % kDataSize];
    const auto& b = gDataNever[(idx + 1) % kDataSize];
    const auto& c = gDataNever[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_Never);

// =============================================================================
// Benchmark: Long simulation (1000 physics steps)
// Tests how often normalization actually triggers
// =============================================================================

static void BM_LongSim_EagerAlways(benchmark::State& state)
{
  for (auto _ : state)
  {
    AngularCoordinate_EagerAlways orientation{0.0, 0.0, 0.0};
    msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i)
    {
      orientation = orientation + angularVelocity * dt;
    }
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_LongSim_EagerAlways);

static void BM_LongSim_DeferredPI(benchmark::State& state)
{
  for (auto _ : state)
  {
    AngularCoordinate_DeferredPI orientation{0.0, 0.0, 0.0};
    msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i)
    {
      orientation = orientation + angularVelocity * dt;
    }
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_LongSim_DeferredPI);

static void BM_LongSim_Deferred10PI(benchmark::State& state)
{
  for (auto _ : state)
  {
    AngularCoordinate_Deferred10PI orientation{0.0, 0.0, 0.0};
    msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i)
    {
      orientation = orientation + angularVelocity * dt;
    }
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_LongSim_Deferred10PI);

static void BM_LongSim_Deferred100PI(benchmark::State& state)
{
  for (auto _ : state)
  {
    AngularCoordinate_Deferred100PI orientation{0.0, 0.0, 0.0};
    msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i)
    {
      orientation = orientation + angularVelocity * dt;
    }
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_LongSim_Deferred100PI);

static void BM_LongSim_Never(benchmark::State& state)
{
  for (auto _ : state)
  {
    AngularCoordinate_Never orientation{0.0, 0.0, 0.0};
    msd_sim::Vector3D angularVelocity{0.1, 0.05, 0.15};
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i)
    {
      orientation = orientation + angularVelocity * dt;
    }
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_LongSim_Never);

BENCHMARK_MAIN();
