// Prototype P1e: Complete Normalization Coverage
// Ticket: 0024_angular_coordinate
// Purpose: Test designs that normalize on ALL modifying operations

#include <Eigen/Dense>
#include <benchmark/benchmark.h>
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

static constexpr double kThreshold = 100.0 * M_PI;

inline bool needsNormalization(double val)
{
  return val <= -kThreshold || val > kThreshold;
}

// =============================================================================
// Strategy 1: Inheritance with overridden compound operators
// Override +=, -=, *= to include normalization
// =============================================================================

class AngularCoordinate_Override : public Eigen::Vector3d
{
public:
  AngularCoordinate_Override() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_Override(double pitch, double roll, double yaw)
    : Eigen::Vector3d{pitch, roll, yaw}
  {
    normalizeIfNeeded();
  }

  // Constructor from Eigen expression
  template <typename OtherDerived>
  AngularCoordinate_Override(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
    normalizeIfNeeded();
  }

  // Assignment from Eigen expression
  template <typename OtherDerived>
  AngularCoordinate_Override& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    normalizeIfNeeded();
    return *this;
  }

  // Override compound assignment operators
  template <typename OtherDerived>
  AngularCoordinate_Override& operator+=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator+=(other);
    normalizeIfNeeded();
    return *this;
  }

  template <typename OtherDerived>
  AngularCoordinate_Override& operator-=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator-=(other);
    normalizeIfNeeded();
    return *this;
  }

  AngularCoordinate_Override& operator*=(double scalar)
  {
    this->Eigen::Vector3d::operator*=(scalar);
    normalizeIfNeeded();
    return *this;
  }

  AngularCoordinate_Override& operator/=(double scalar)
  {
    this->Eigen::Vector3d::operator/=(scalar);
    normalizeIfNeeded();
    return *this;
  }

  // Accessors (fast, no normalization)
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

  // Setters with normalization
  void setPitch(double val)
  {
    (*this)[0] = needsNormalization(val) ? normalizeAngle(val) : val;
  }
  void setRoll(double val)
  {
    (*this)[1] = needsNormalization(val) ? normalizeAngle(val) : val;
  }
  void setYaw(double val)
  {
    (*this)[2] = needsNormalization(val) ? normalizeAngle(val) : val;
  }

private:
  void normalizeIfNeeded()
  {
    if (needsNormalization((*this)[0]))
      (*this)[0] = normalizeAngle((*this)[0]);
    if (needsNormalization((*this)[1]))
      (*this)[1] = normalizeAngle((*this)[1]);
    if (needsNormalization((*this)[2]))
      (*this)[2] = normalizeAngle((*this)[2]);
  }
};

// =============================================================================
// Strategy 2: Composition (private Eigen member, explicit interface)
// No inheritance leakage - full control over all operations
// =============================================================================

class AngularCoordinate_Composition
{
public:
  AngularCoordinate_Composition() : data_{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_Composition(double pitch, double roll, double yaw) : data_{pitch, roll, yaw}
  {
    normalizeIfNeeded();
  }

  // Accessors (fast)
  double pitch() const
  {
    return data_[0];
  }
  double roll() const
  {
    return data_[1];
  }
  double yaw() const
  {
    return data_[2];
  }

  // Setters with normalization
  void setPitch(double val)
  {
    data_[0] = needsNormalization(val) ? normalizeAngle(val) : val;
  }
  void setRoll(double val)
  {
    data_[1] = needsNormalization(val) ? normalizeAngle(val) : val;
  }
  void setYaw(double val)
  {
    data_[2] = needsNormalization(val) ? normalizeAngle(val) : val;
  }

  // Arithmetic operators (return new object)
  AngularCoordinate_Composition operator+(const AngularCoordinate_Composition& other) const
  {
    return AngularCoordinate_Composition{data_[0] + other.data_[0], data_[1] + other.data_[1],
                                         data_[2] + other.data_[2]};
  }

  AngularCoordinate_Composition operator-(const AngularCoordinate_Composition& other) const
  {
    return AngularCoordinate_Composition{data_[0] - other.data_[0], data_[1] - other.data_[1],
                                         data_[2] - other.data_[2]};
  }

  AngularCoordinate_Composition operator*(double scalar) const
  {
    return AngularCoordinate_Composition{data_[0] * scalar, data_[1] * scalar, data_[2] * scalar};
  }

  // Compound assignment with normalization
  AngularCoordinate_Composition& operator+=(const AngularCoordinate_Composition& other)
  {
    data_ += other.data_;
    normalizeIfNeeded();
    return *this;
  }

  AngularCoordinate_Composition& operator-=(const AngularCoordinate_Composition& other)
  {
    data_ -= other.data_;
    normalizeIfNeeded();
    return *this;
  }

  AngularCoordinate_Composition& operator*=(double scalar)
  {
    data_ *= scalar;
    normalizeIfNeeded();
    return *this;
  }

  // Allow addition with Eigen::Vector3d (common in physics)
  AngularCoordinate_Composition operator+(const Eigen::Vector3d& other) const
  {
    return AngularCoordinate_Composition{data_[0] + other[0], data_[1] + other[1],
                                         data_[2] + other[2]};
  }

  AngularCoordinate_Composition& operator+=(const Eigen::Vector3d& other)
  {
    data_ += other;
    normalizeIfNeeded();
    return *this;
  }

  // Cross product
  AngularCoordinate_Composition cross(const AngularCoordinate_Composition& other) const
  {
    Eigen::Vector3d result = data_.cross(other.data_);
    return AngularCoordinate_Composition{result[0], result[1], result[2]};
  }

  // Dot product
  double dot(const AngularCoordinate_Composition& other) const
  {
    return data_.dot(other.data_);
  }

  // Norm
  double norm() const
  {
    return data_.norm();
  }

  // Access underlying Eigen vector (const only - prevents modification)
  const Eigen::Vector3d& asEigen() const
  {
    return data_;
  }

private:
  Eigen::Vector3d data_;

  void normalizeIfNeeded()
  {
    if (needsNormalization(data_[0]))
      data_[0] = normalizeAngle(data_[0]);
    if (needsNormalization(data_[1]))
      data_[1] = normalizeAngle(data_[1]);
    if (needsNormalization(data_[2]))
      data_[2] = normalizeAngle(data_[2]);
  }
};

// =============================================================================
// Strategy 3: Never (baseline)
// =============================================================================

class AngularCoordinate_Never : public Eigen::Vector3d
{
public:
  AngularCoordinate_Never() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_Never(double pitch, double roll, double yaw) : Eigen::Vector3d{pitch, roll, yaw}
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
// Benchmarks
// =============================================================================

constexpr size_t kDataSize = 1024;

// Pre-generate test data
template <typename T>
std::vector<T> generateData()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-3.0, 3.0);
  std::vector<T> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}

static auto gDataOverride = generateData<AngularCoordinate_Override>();
static auto gDataComposition = generateData<AngularCoordinate_Composition>();
static auto gDataNever = generateData<AngularCoordinate_Never>();

// =============================================================================
// Benchmark: Accessor Performance
// =============================================================================

static void BM_Accessor_Override(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataOverride[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Override);

static void BM_Accessor_Composition(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataComposition[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Composition);

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
// Benchmark: Compound Assignment (+=)
// =============================================================================

static void BM_CompoundAdd_Override(benchmark::State& state)
{
  AngularCoordinate_Override a{0, 0, 0};
  Eigen::Vector3d delta{0.001, 0.0005, 0.0015};

  for (auto _ : state)
  {
    a += delta;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_CompoundAdd_Override);

static void BM_CompoundAdd_Composition(benchmark::State& state)
{
  AngularCoordinate_Composition a{0, 0, 0};
  Eigen::Vector3d delta{0.001, 0.0005, 0.0015};

  for (auto _ : state)
  {
    a += delta;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_CompoundAdd_Composition);

static void BM_CompoundAdd_Never(benchmark::State& state)
{
  AngularCoordinate_Never a{0, 0, 0};
  Eigen::Vector3d delta{0.001, 0.0005, 0.0015};

  for (auto _ : state)
  {
    a += delta;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_CompoundAdd_Never);

// =============================================================================
// Benchmark: Assignment (a = a + b)
// =============================================================================

static void BM_AssignAdd_Override(benchmark::State& state)
{
  AngularCoordinate_Override a{0, 0, 0};
  Eigen::Vector3d delta{0.001, 0.0005, 0.0015};

  for (auto _ : state)
  {
    a = a + delta;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_AssignAdd_Override);

static void BM_AssignAdd_Composition(benchmark::State& state)
{
  AngularCoordinate_Composition a{0, 0, 0};
  Eigen::Vector3d delta{0.001, 0.0005, 0.0015};

  for (auto _ : state)
  {
    a = a + delta;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_AssignAdd_Composition);

static void BM_AssignAdd_Never(benchmark::State& state)
{
  AngularCoordinate_Never a{0, 0, 0};
  Eigen::Vector3d delta{0.001, 0.0005, 0.0015};

  for (auto _ : state)
  {
    a = a + delta;
    benchmark::DoNotOptimize(a);
  }
}
BENCHMARK(BM_AssignAdd_Never);

// =============================================================================
// Benchmark: Physics Update via += (realistic pattern)
// =============================================================================

static void BM_PhysicsViaCompound_Override(benchmark::State& state)
{
  AngularCoordinate_Override orientation{0.0, 0.0, 0.0};
  Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation += angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsViaCompound_Override);

static void BM_PhysicsViaCompound_Composition(benchmark::State& state)
{
  AngularCoordinate_Composition orientation{0.0, 0.0, 0.0};
  Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation += angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsViaCompound_Composition);

static void BM_PhysicsViaCompound_Never(benchmark::State& state)
{
  AngularCoordinate_Never orientation{0.0, 0.0, 0.0};
  Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};
  double dt = 0.01;

  for (auto _ : state)
  {
    orientation += angularVelocity * dt;
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_PhysicsViaCompound_Never);

// =============================================================================
// Benchmark: Long Simulation via += (1000 steps)
// =============================================================================

static void BM_LongSimCompound_Override(benchmark::State& state)
{
  for (auto _ : state)
  {
    AngularCoordinate_Override orientation{0.0, 0.0, 0.0};
    Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i)
    {
      orientation += angularVelocity * dt;
    }
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_LongSimCompound_Override);

static void BM_LongSimCompound_Composition(benchmark::State& state)
{
  for (auto _ : state)
  {
    AngularCoordinate_Composition orientation{0.0, 0.0, 0.0};
    Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i)
    {
      orientation += angularVelocity * dt;
    }
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_LongSimCompound_Composition);

static void BM_LongSimCompound_Never(benchmark::State& state)
{
  for (auto _ : state)
  {
    AngularCoordinate_Never orientation{0.0, 0.0, 0.0};
    Eigen::Vector3d angularVelocity{0.1, 0.05, 0.15};
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i)
    {
      orientation += angularVelocity * dt;
    }
    benchmark::DoNotOptimize(orientation);
  }
}
BENCHMARK(BM_LongSimCompound_Never);

// =============================================================================
// Benchmark: Cross Product
// =============================================================================

static void BM_CrossProduct_Override(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Override result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = gDataOverride[idx % kDataSize];
    const auto& b = gDataOverride[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_Override);

static void BM_CrossProduct_Composition(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_Composition result{0, 0, 0};

  for (auto _ : state)
  {
    const auto& a = gDataComposition[idx % kDataSize];
    const auto& b = gDataComposition[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_Composition);

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
}
BENCHMARK(BM_CrossProduct_Never);

// =============================================================================
// Benchmark: Memory Size
// =============================================================================

static void BM_MemorySize(benchmark::State& state)
{
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(sizeof(AngularCoordinate_Override));
    benchmark::DoNotOptimize(sizeof(AngularCoordinate_Composition));
    benchmark::DoNotOptimize(sizeof(AngularCoordinate_Never));
  }
  state.SetLabel("Override=" + std::to_string(sizeof(AngularCoordinate_Override)) +
                 "B, Composition=" + std::to_string(sizeof(AngularCoordinate_Composition)) +
                 "B, Never=" + std::to_string(sizeof(AngularCoordinate_Never)) + "B");
}
BENCHMARK(BM_MemorySize);

BENCHMARK_MAIN();
