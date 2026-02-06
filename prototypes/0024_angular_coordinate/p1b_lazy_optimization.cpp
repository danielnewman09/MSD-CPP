// Prototype P1b: Lazy Normalization Optimization
// Ticket: 0024_angular_coordinate
// Purpose: Test lazy normalization with early-exit and writeback optimizations

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
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

// Optimized: Early exit if already in range
inline double normalizeAngle_EarlyExit(double rad)
{
  // Fast path: already in (-pi, pi]
  if (rad > -M_PI && rad <= M_PI)
  {
    return rad;
  }
  // Slow path: needs normalization
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

// =============================================================================
// Strategy 1: Lazy Always (current implementation)
// =============================================================================

class AngularCoordinate_LazyAlways : public msd_sim::Vector3D
{
public:
  AngularCoordinate_LazyAlways() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularCoordinate_LazyAlways(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_LazyAlways(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_LazyAlways& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  // Always calls fmod() on every access
  double pitch() const
  {
    return normalizeAngle_Always((*this)[0]);
  }
  double roll() const
  {
    return normalizeAngle_Always((*this)[1]);
  }
  double yaw() const
  {
    return normalizeAngle_Always((*this)[2]);
  }
};

// =============================================================================
// Strategy 2: Lazy with Early Exit (check bounds first)
// =============================================================================

class AngularCoordinate_LazyEarlyExit : public msd_sim::Vector3D
{
public:
  AngularCoordinate_LazyEarlyExit() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularCoordinate_LazyEarlyExit(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_LazyEarlyExit(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_LazyEarlyExit& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  // Early exit if already normalized
  double pitch() const
  {
    return normalizeAngle_EarlyExit((*this)[0]);
  }
  double roll() const
  {
    return normalizeAngle_EarlyExit((*this)[1]);
  }
  double yaw() const
  {
    return normalizeAngle_EarlyExit((*this)[2]);
  }
};

// =============================================================================
// Strategy 3: Lazy with Writeback (normalize and store on first access)
// =============================================================================

class AngularCoordinate_LazyWriteback : public msd_sim::Vector3D
{
public:
  AngularCoordinate_LazyWriteback() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularCoordinate_LazyWriteback(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_LazyWriteback(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_LazyWriteback& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  // Normalize, writeback, and return
  double pitch() const
  {
    double val = (*this)[0];
    if (val > -M_PI && val <= M_PI)
    {
      return val;  // Already normalized
    }
    double normalized = normalizeAngle_Always(val);
    const_cast<AngularCoordinate_LazyWriteback*>(this)->operator[](0) =
      normalized;
    return normalized;
  }

  double roll() const
  {
    double val = (*this)[1];
    if (val > -M_PI && val <= M_PI)
    {
      return val;
    }
    double normalized = normalizeAngle_Always(val);
    const_cast<AngularCoordinate_LazyWriteback*>(this)->operator[](1) =
      normalized;
    return normalized;
  }

  double yaw() const
  {
    double val = (*this)[2];
    if (val > -M_PI && val <= M_PI)
    {
      return val;
    }
    double normalized = normalizeAngle_Always(val);
    const_cast<AngularCoordinate_LazyWriteback*>(this)->operator[](2) =
      normalized;
    return normalized;
  }
};

// =============================================================================
// Strategy 4: Eager (baseline for comparison)
// =============================================================================

class AngularCoordinate_Eager : public msd_sim::Vector3D
{
public:
  AngularCoordinate_Eager() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_Eager(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{normalizeAngle_Always(pitch),
                        normalizeAngle_Always(roll),
                        normalizeAngle_Always(yaw)}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_Eager(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
    normalizeInPlace();
  }

  template <typename OtherDerived>
  AngularCoordinate_Eager& operator=(
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
    (*this)[0] = normalizeAngle_Always((*this)[0]);
    (*this)[1] = normalizeAngle_Always((*this)[1]);
    (*this)[2] = normalizeAngle_Always((*this)[2]);
  }
};

// =============================================================================
// Test Data Generation
// =============================================================================

constexpr size_t kDataSize = 1024;

// Data that needs normalization (outside -pi to pi)
template <typename T>
std::vector<T> generateOutOfRangeData()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);  // Outside (-pi, pi)

  std::vector<T> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}

// Data already in normalized range
template <typename T>
std::vector<T> generateInRangeData()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-M_PI + 0.01,
                                       M_PI - 0.01);  // Inside (-pi, pi)

  std::vector<T> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}

// Out of range data
static auto gDataLazyAlways_OutOfRange =
  generateOutOfRangeData<AngularCoordinate_LazyAlways>();
static auto gDataLazyEarlyExit_OutOfRange =
  generateOutOfRangeData<AngularCoordinate_LazyEarlyExit>();
static auto gDataLazyWriteback_OutOfRange =
  generateOutOfRangeData<AngularCoordinate_LazyWriteback>();
static auto gDataEager_OutOfRange =
  generateOutOfRangeData<AngularCoordinate_Eager>();

// In range data
static auto gDataLazyAlways_InRange =
  generateInRangeData<AngularCoordinate_LazyAlways>();
static auto gDataLazyEarlyExit_InRange =
  generateInRangeData<AngularCoordinate_LazyEarlyExit>();
static auto gDataLazyWriteback_InRange =
  generateInRangeData<AngularCoordinate_LazyWriteback>();
static auto gDataEager_InRange = generateInRangeData<AngularCoordinate_Eager>();

// =============================================================================
// Benchmark: Accessor with OUT OF RANGE data (needs normalization)
// =============================================================================

static void BM_Accessor_LazyAlways_OutOfRange(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataLazyAlways_OutOfRange[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_LazyAlways_OutOfRange);

static void BM_Accessor_LazyEarlyExit_OutOfRange(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataLazyEarlyExit_OutOfRange[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_LazyEarlyExit_OutOfRange);

static void BM_Accessor_LazyWriteback_OutOfRange(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataLazyWriteback_OutOfRange[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_LazyWriteback_OutOfRange);

static void BM_Accessor_Eager_OutOfRange(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataEager_OutOfRange[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Eager_OutOfRange);

// =============================================================================
// Benchmark: Accessor with IN RANGE data (already normalized)
// =============================================================================

static void BM_Accessor_LazyAlways_InRange(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataLazyAlways_InRange[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_LazyAlways_InRange);

static void BM_Accessor_LazyEarlyExit_InRange(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataLazyEarlyExit_InRange[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_LazyEarlyExit_InRange);

static void BM_Accessor_LazyWriteback_InRange(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataLazyWriteback_InRange[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_LazyWriteback_InRange);

static void BM_Accessor_Eager_InRange(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataEager_InRange[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Eager_InRange);

// =============================================================================
// Benchmark: Multiple reads of same object (tests writeback benefit)
// =============================================================================

static void BM_MultiRead_LazyAlways(benchmark::State& state)
{
  AngularCoordinate_LazyAlways a{5.0, -6.0, 7.0};  // Out of range
  double sum = 0.0;
  for (auto _ : state)
  {
    // Read each component 3 times
    sum += a.pitch() + a.roll() + a.yaw();
    sum += a.pitch() + a.roll() + a.yaw();
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_MultiRead_LazyAlways);

static void BM_MultiRead_LazyEarlyExit(benchmark::State& state)
{
  AngularCoordinate_LazyEarlyExit a{5.0, -6.0, 7.0};  // Out of range
  double sum = 0.0;
  for (auto _ : state)
  {
    sum += a.pitch() + a.roll() + a.yaw();
    sum += a.pitch() + a.roll() + a.yaw();
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_MultiRead_LazyEarlyExit);

static void BM_MultiRead_LazyWriteback(benchmark::State& state)
{
  AngularCoordinate_LazyWriteback a{5.0, -6.0, 7.0};  // Out of range
  double sum = 0.0;
  for (auto _ : state)
  {
    // First read normalizes and writes back
    // Subsequent reads hit the fast path
    sum += a.pitch() + a.roll() + a.yaw();
    sum += a.pitch() + a.roll() + a.yaw();
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_MultiRead_LazyWriteback);

static void BM_MultiRead_Eager(benchmark::State& state)
{
  AngularCoordinate_Eager a{5.0, -6.0, 7.0};  // Normalized at construction
  double sum = 0.0;
  for (auto _ : state)
  {
    sum += a.pitch() + a.roll() + a.yaw();
    sum += a.pitch() + a.roll() + a.yaw();
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_MultiRead_Eager);

// =============================================================================
// Benchmark: Construction + Single Read (typical usage pattern)
// =============================================================================

static void BM_ConstructAndRead_LazyAlways(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_LazyAlways a{dis(gen), dis(gen), dis(gen)};
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_ConstructAndRead_LazyAlways);

static void BM_ConstructAndRead_LazyWriteback(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_LazyWriteback a{dis(gen), dis(gen), dis(gen)};
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_ConstructAndRead_LazyWriteback);

static void BM_ConstructAndRead_Eager(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_Eager a{dis(gen), dis(gen), dis(gen)};
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_ConstructAndRead_Eager);

// =============================================================================
// Benchmark: Construction + Multiple Reads
// =============================================================================

static void BM_ConstructAnd10Reads_LazyAlways(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_LazyAlways a{dis(gen), dis(gen), dis(gen)};
    for (int i = 0; i < 10; ++i)
    {
      sum += a.pitch() + a.roll() + a.yaw();
    }
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_ConstructAnd10Reads_LazyAlways);

static void BM_ConstructAnd10Reads_LazyWriteback(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_LazyWriteback a{dis(gen), dis(gen), dis(gen)};
    for (int i = 0; i < 10; ++i)
    {
      sum += a.pitch() + a.roll() + a.yaw();
    }
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_ConstructAnd10Reads_LazyWriteback);

static void BM_ConstructAnd10Reads_Eager(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  double sum = 0.0;

  for (auto _ : state)
  {
    AngularCoordinate_Eager a{dis(gen), dis(gen), dis(gen)};
    for (int i = 0; i < 10; ++i)
    {
      sum += a.pitch() + a.roll() + a.yaw();
    }
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_ConstructAnd10Reads_Eager);

BENCHMARK_MAIN();
