// Prototype P5: Shared Interface Pattern
// Ticket: 0024_angular_coordinate
// Purpose: Compare implementation approaches for sharing code between
// AngularCoordinate and AngularRate

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <type_traits>
#include <vector>

// Normalization function
inline double normalizeAngle(double rad)
{
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

// ==============================================================================
// Approach 1: Policy-Based Template
// ==============================================================================

namespace policy_approach
{

struct AutoNormalize
{
  static constexpr bool normalizes = true;
  static double apply(double rad)
  {
    return normalizeAngle(rad);
  }
};

struct NoNormalize
{
  static constexpr bool normalizes = false;
  static double apply(double rad)
  {
    return rad;
  }
};

template <typename NormalizationPolicy>
class AngularVector : public msd_sim::Vector3D
{
public:
  using Policy = NormalizationPolicy;

  AngularVector() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularVector(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularVector(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularVector& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  double pitch() const
  {
    return Policy::apply((*this)[0]);
  }
  double roll() const
  {
    return Policy::apply((*this)[1]);
  }
  double yaw() const
  {
    return Policy::apply((*this)[2]);
  }
};

using AngularCoordinate = AngularVector<AutoNormalize>;
using AngularRate = AngularVector<NoNormalize>;

}  // namespace policy_approach

// ==============================================================================
// Approach 2: CRTP Base Class
// ==============================================================================

namespace crtp_approach
{

template <typename Derived>
class AngularBase : public msd_sim::Vector3D
{
public:
  AngularBase() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularBase(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularBase(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularBase& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  double pitch() const
  {
    return static_cast<const Derived*>(this)->applyNormalization((*this)[0]);
  }
  double roll() const
  {
    return static_cast<const Derived*>(this)->applyNormalization((*this)[1]);
  }
  double yaw() const
  {
    return static_cast<const Derived*>(this)->applyNormalization((*this)[2]);
  }
};

class AngularCoordinate : public AngularBase<AngularCoordinate>
{
public:
  using AngularBase<AngularCoordinate>::AngularBase;

  double applyNormalization(double rad) const
  {
    return normalizeAngle(rad);
  }
};

class AngularRate : public AngularBase<AngularRate>
{
public:
  using AngularBase<AngularRate>::AngularBase;

  double applyNormalization(double rad) const
  {
    return rad;
  }
};

}  // namespace crtp_approach

// ==============================================================================
// Approach 3: Simple Inheritance (Virtual Functions)
// ==============================================================================

namespace virtual_approach
{

class AngularBase : public msd_sim::Vector3D
{
public:
  AngularBase() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularBase(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularBase(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularBase& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  virtual double pitch() const = 0;
  virtual double roll() const = 0;
  virtual double yaw() const = 0;

  virtual ~AngularBase() = default;
};

class AngularCoordinate : public AngularBase
{
public:
  using AngularBase::AngularBase;

  double pitch() const override
  {
    return normalizeAngle((*this)[0]);
  }
  double roll() const override
  {
    return normalizeAngle((*this)[1]);
  }
  double yaw() const override
  {
    return normalizeAngle((*this)[2]);
  }
};

class AngularRate : public AngularBase
{
public:
  using AngularBase::AngularBase;

  double pitch() const override
  {
    return (*this)[0];
  }
  double roll() const override
  {
    return (*this)[1];
  }
  double yaw() const override
  {
    return (*this)[2];
  }
};

}  // namespace virtual_approach

// ==============================================================================
// Approach 4: No Inheritance (Explicit Duplication)
// ==============================================================================

namespace duplicate_approach
{

class AngularCoordinate : public msd_sim::Vector3D
{
public:
  AngularCoordinate() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularCoordinate(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
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

class AngularRate : public msd_sim::Vector3D
{
public:
  AngularRate() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }
  AngularRate(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{pitch, roll, yaw}
  {
  }

  template <typename OtherDerived>
  AngularRate(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularRate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
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

}  // namespace duplicate_approach

// =============================================================================
// Test Data Generation
// =============================================================================

constexpr size_t kDataSize = 1024;

// Policy approach data
static std::vector<policy_approach::AngularCoordinate> gDataPolicy = []()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  std::vector<policy_approach::AngularCoordinate> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  return data;
}();

// CRTP approach data
static std::vector<crtp_approach::AngularCoordinate> gDataCRTP = []()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  std::vector<crtp_approach::AngularCoordinate> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  return data;
}();

// Virtual approach data
static std::vector<virtual_approach::AngularCoordinate> gDataVirtual = []()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  std::vector<virtual_approach::AngularCoordinate> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  return data;
}();

// Duplicate approach data
static std::vector<duplicate_approach::AngularCoordinate> gDataDuplicate = []()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  std::vector<duplicate_approach::AngularCoordinate> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  return data;
}();

// =============================================================================
// Benchmark: Accessor Performance
// =============================================================================

static void BM_Accessor_Policy(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataPolicy[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Policy);

static void BM_Accessor_CRTP(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataCRTP[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_CRTP);

static void BM_Accessor_Virtual(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataVirtual[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Virtual);

static void BM_Accessor_Duplicate(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataDuplicate[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_Duplicate);

// =============================================================================
// Benchmark: Arithmetic Operations
// =============================================================================

static void BM_Arithmetic_Policy(benchmark::State& state)
{
  size_t idx = 0;
  policy_approach::AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataPolicy[idx % kDataSize];
    const auto& b = gDataPolicy[(idx + 1) % kDataSize];
    const auto& c = gDataPolicy[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_Policy);

static void BM_Arithmetic_CRTP(benchmark::State& state)
{
  size_t idx = 0;
  crtp_approach::AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataCRTP[idx % kDataSize];
    const auto& b = gDataCRTP[(idx + 1) % kDataSize];
    const auto& c = gDataCRTP[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_CRTP);

static void BM_Arithmetic_Virtual(benchmark::State& state)
{
  size_t idx = 0;
  virtual_approach::AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataVirtual[idx % kDataSize];
    const auto& b = gDataVirtual[(idx + 1) % kDataSize];
    const auto& c = gDataVirtual[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_Virtual);

static void BM_Arithmetic_Duplicate(benchmark::State& state)
{
  size_t idx = 0;
  duplicate_approach::AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataDuplicate[idx % kDataSize];
    const auto& b = gDataDuplicate[(idx + 1) % kDataSize];
    const auto& c = gDataDuplicate[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_Duplicate);

// =============================================================================
// Benchmark: Cross Product
// =============================================================================

static void BM_CrossProduct_Policy(benchmark::State& state)
{
  size_t idx = 0;
  policy_approach::AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataPolicy[idx % kDataSize];
    const auto& b = gDataPolicy[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_Policy);

static void BM_CrossProduct_CRTP(benchmark::State& state)
{
  size_t idx = 0;
  crtp_approach::AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataCRTP[idx % kDataSize];
    const auto& b = gDataCRTP[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_CRTP);

static void BM_CrossProduct_Virtual(benchmark::State& state)
{
  size_t idx = 0;
  virtual_approach::AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataVirtual[idx % kDataSize];
    const auto& b = gDataVirtual[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_Virtual);

static void BM_CrossProduct_Duplicate(benchmark::State& state)
{
  size_t idx = 0;
  duplicate_approach::AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataDuplicate[idx % kDataSize];
    const auto& b = gDataDuplicate[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_Duplicate);

// =============================================================================
// Memory Size Report
// =============================================================================

static void BM_MemorySize_Report(benchmark::State& state)
{
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(sizeof(policy_approach::AngularCoordinate));
    benchmark::DoNotOptimize(sizeof(crtp_approach::AngularCoordinate));
    benchmark::DoNotOptimize(sizeof(virtual_approach::AngularCoordinate));
    benchmark::DoNotOptimize(sizeof(duplicate_approach::AngularCoordinate));
  }
  state.SetLabel(
    "Policy=" + std::to_string(sizeof(policy_approach::AngularCoordinate)) +
    "B, CRTP=" + std::to_string(sizeof(crtp_approach::AngularCoordinate)) +
    "B, Virtual=" +
    std::to_string(sizeof(virtual_approach::AngularCoordinate)) +
    "B, Duplicate=" +
    std::to_string(sizeof(duplicate_approach::AngularCoordinate)) + "B");
}
BENCHMARK(BM_MemorySize_Report);
