// Prototype P1: Normalization Performance
// Ticket: 0024_angular_coordinate
// Purpose: Compare normalization strategies (lazy, eager, never) using Google
// Benchmark

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

// Normalization function matching Angle::normalize()
inline double normalizeAngle(double rad)
{
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

// Strategy 1: Lazy Normalization (normalize on access)
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

  // Lazy normalization - normalize on every access
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

// Strategy 2: Eager Normalization (normalize on construction/assignment)
class AngularCoordinate_Eager : public msd_sim::Vector3D
{
public:
  AngularCoordinate_Eager() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_Eager(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{normalizeAngle(pitch),
                        normalizeAngle(roll),
                        normalizeAngle(yaw)}
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

  // No normalization on access - already normalized
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

// Strategy 3: Never Normalize (baseline for comparison)
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

  // No normalization - just return raw value
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

// -----------------------------------------------------------------------------
// Test Data Generation
// -----------------------------------------------------------------------------

// Global test data to avoid measuring allocation in benchmarks
constexpr size_t kDataSize = 1024;

template <typename T>
std::vector<T> generateTestData()
{
  std::mt19937 gen(42);  // Fixed seed for reproducibility
  std::uniform_real_distribution<> dis(-10.0, 10.0);  // Exceed +/- 2pi

  std::vector<T> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}

// Pre-generate test data
static auto gDataLazy = generateTestData<AngularCoordinate_Lazy>();
static auto gDataEager = generateTestData<AngularCoordinate_Eager>();
static auto gDataNever = generateTestData<AngularCoordinate_Never>();

// -----------------------------------------------------------------------------
// Benchmark: Accessor Performance
// -----------------------------------------------------------------------------

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
  benchmark::DoNotOptimize(sum);
}
BENCHMARK(BM_Accessor_Never);

static void BM_Accessor_Lazy(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataLazy[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
  benchmark::DoNotOptimize(sum);
}
BENCHMARK(BM_Accessor_Lazy);

static void BM_Accessor_Eager(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataEager[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
  benchmark::DoNotOptimize(sum);
}
BENCHMARK(BM_Accessor_Eager);

// -----------------------------------------------------------------------------
// Benchmark: Arithmetic Operations (a + b * 2 - c)
// -----------------------------------------------------------------------------

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
  benchmark::DoNotOptimize(result);
}
BENCHMARK(BM_Arithmetic_Never);

static void BM_Arithmetic_Lazy(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Lazy result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataLazy[idx % kDataSize];
    const auto& b = gDataLazy[(idx + 1) % kDataSize];
    const auto& c = gDataLazy[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
  benchmark::DoNotOptimize(result);
}
BENCHMARK(BM_Arithmetic_Lazy);

static void BM_Arithmetic_Eager(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Eager result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataEager[idx % kDataSize];
    const auto& b = gDataEager[(idx + 1) % kDataSize];
    const auto& c = gDataEager[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
  benchmark::DoNotOptimize(result);
}
BENCHMARK(BM_Arithmetic_Eager);

// -----------------------------------------------------------------------------
// Benchmark: Cross Product
// -----------------------------------------------------------------------------

static void BM_CrossProduct_Never(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Never result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataNever[idx % kDataSize];
    const auto& b = gDataNever[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
  benchmark::DoNotOptimize(result);
}
BENCHMARK(BM_CrossProduct_Never);

static void BM_CrossProduct_Lazy(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Lazy result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataLazy[idx % kDataSize];
    const auto& b = gDataLazy[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
  benchmark::DoNotOptimize(result);
}
BENCHMARK(BM_CrossProduct_Lazy);

static void BM_CrossProduct_Eager(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Eager result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataEager[idx % kDataSize];
    const auto& b = gDataEager[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
  benchmark::DoNotOptimize(result);
}
BENCHMARK(BM_CrossProduct_Eager);

// -----------------------------------------------------------------------------
// Benchmark: Construction Performance
// -----------------------------------------------------------------------------

static void BM_Construction_Never(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);

  for (auto _ : state)
  {
    AngularCoordinate_Never a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_Never);

static void BM_Construction_Lazy(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);

  for (auto _ : state)
  {
    AngularCoordinate_Lazy a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_Lazy);

static void BM_Construction_Eager(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);

  for (auto _ : state)
  {
    AngularCoordinate_Eager a{dis(gen), dis(gen), dis(gen)};
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_Construction_Eager);

// -----------------------------------------------------------------------------
// Benchmark: Assignment Performance
// -----------------------------------------------------------------------------

static void BM_Assignment_Never(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Never result{0, 0, 0};
  msd_sim::Vector3D source{1.5, 2.5, 3.5};

  for (auto _ : state)
  {
    source[0] = static_cast<double>(idx % 100);
    result = source;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Assignment_Never);

static void BM_Assignment_Lazy(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Lazy result{0, 0, 0};
  msd_sim::Vector3D source{1.5, 2.5, 3.5};

  for (auto _ : state)
  {
    source[0] = static_cast<double>(idx % 100);
    result = source;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Assignment_Lazy);

static void BM_Assignment_Eager(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Eager result{0, 0, 0};
  msd_sim::Vector3D source{1.5, 2.5, 3.5};

  for (auto _ : state)
  {
    source[0] = static_cast<double>(idx % 100);
    result = source;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Assignment_Eager);
