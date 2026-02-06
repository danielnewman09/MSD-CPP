// Prototype P2: Internal Storage Comparison
// Ticket: 0024_angular_coordinate
// Purpose: Compare Angle objects vs raw double storage using Google Benchmark

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

// =============================================================================
// Angle Class (from existing codebase)
// =============================================================================

class Angle
{
public:
  enum class Norm : uint8_t
  {
    PI,     // (-pi, pi]
    TWO_PI  // [0, 2pi)
  };

  Angle() : rad_{0.0}, normalization_{Norm::PI}
  {
  }

  explicit Angle(double radians, Norm norm = Norm::PI)
    : rad_{radians}, normalization_{norm}
  {
  }

  static Angle fromRadians(double radians, Norm norm = Norm::PI)
  {
    return Angle{radians, norm};
  }

  double getRad() const
  {
    return normalize();
  }

  void setRad(double rad)
  {
    rad_ = rad;
  }

  Angle operator+(const Angle& other) const
  {
    return Angle(rad_ + other.rad_, normalization_);
  }

  Angle operator-(const Angle& other) const
  {
    return Angle(rad_ - other.rad_, normalization_);
  }

  Angle operator*(double scalar) const
  {
    return Angle(rad_ * scalar, normalization_);
  }

  Angle& operator+=(const Angle& other)
  {
    rad_ += other.rad_;
    return *this;
  }

private:
  double rad_;
  Norm normalization_;

  double normalize() const
  {
    constexpr double twoPi = 2.0 * M_PI;
    double normAngle = rad_;
    if (normalization_ == Norm::PI)
    {
      if (std::abs(normAngle) > M_PI)
      {
        normAngle = std::fmod(rad_ + M_PI, twoPi);
        if (normAngle < 0.)
        {
          normAngle += twoPi;
        }
        normAngle -= M_PI;
      }
    }
    return normAngle;
  }
};

// =============================================================================
// Strategy 1: Angle-based storage (like EulerAngles)
// =============================================================================

class AngularCoordinate_AngleStorage
{
public:
  AngularCoordinate_AngleStorage() : pitch_{0.0}, roll_{0.0}, yaw_{0.0}
  {
  }

  AngularCoordinate_AngleStorage(double pitch, double roll, double yaw)
    : pitch_{pitch}, roll_{roll}, yaw_{yaw}
  {
  }

  double pitch() const
  {
    return pitch_.getRad();
  }
  double roll() const
  {
    return roll_.getRad();
  }
  double yaw() const
  {
    return yaw_.getRad();
  }

  void setPitch(double rad)
  {
    pitch_.setRad(rad);
  }
  void setRoll(double rad)
  {
    roll_.setRad(rad);
  }
  void setYaw(double rad)
  {
    yaw_.setRad(rad);
  }

  // Manual arithmetic (no Eigen)
  AngularCoordinate_AngleStorage operator+(
    const AngularCoordinate_AngleStorage& other) const
  {
    return AngularCoordinate_AngleStorage{
      pitch_.getRad() + other.pitch_.getRad(),
      roll_.getRad() + other.roll_.getRad(),
      yaw_.getRad() + other.yaw_.getRad()};
  }

  AngularCoordinate_AngleStorage operator-(
    const AngularCoordinate_AngleStorage& other) const
  {
    return AngularCoordinate_AngleStorage{
      pitch_.getRad() - other.pitch_.getRad(),
      roll_.getRad() - other.roll_.getRad(),
      yaw_.getRad() - other.yaw_.getRad()};
  }

  AngularCoordinate_AngleStorage operator*(double scalar) const
  {
    return AngularCoordinate_AngleStorage{pitch_.getRad() * scalar,
                                          roll_.getRad() * scalar,
                                          yaw_.getRad() * scalar};
  }

  // Manual cross product
  AngularCoordinate_AngleStorage cross(
    const AngularCoordinate_AngleStorage& other) const
  {
    double p1 = pitch_.getRad(), r1 = roll_.getRad(), y1 = yaw_.getRad();
    double p2 = other.pitch_.getRad(), r2 = other.roll_.getRad(),
           y2 = other.yaw_.getRad();
    return AngularCoordinate_AngleStorage{
      r1 * y2 - y1 * r2, y1 * p2 - p1 * y2, p1 * r2 - r1 * p2};
  }

  double norm() const
  {
    double p = pitch_.getRad(), r = roll_.getRad(), y = yaw_.getRad();
    return std::sqrt(p * p + r * r + y * y);
  }

private:
  Angle pitch_;
  Angle roll_;
  Angle yaw_;
};

// =============================================================================
// Strategy 2: Raw double storage (msd_sim::Vector3D) with eager normalization
// =============================================================================

inline double normalizeAngle(double rad)
{
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

class AngularCoordinate_DoubleStorage : public msd_sim::Vector3D
{
public:
  AngularCoordinate_DoubleStorage() : msd_sim::Vector3D{0.0, 0.0, 0.0}
  {
  }

  AngularCoordinate_DoubleStorage(double pitch, double roll, double yaw)
    : msd_sim::Vector3D{normalizeAngle(pitch),
                        normalizeAngle(roll),
                        normalizeAngle(yaw)}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_DoubleStorage(const Eigen::MatrixBase<OtherDerived>& other)
    : msd_sim::Vector3D{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate_DoubleStorage& operator=(
    const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->msd_sim::Vector3D::operator=(other);
    return *this;
  }

  // Eager normalization: values already normalized, just return them
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
// Test Data Generation
// =============================================================================

constexpr size_t kDataSize = 1024;

static std::vector<AngularCoordinate_AngleStorage> gDataAngle = []()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  std::vector<AngularCoordinate_AngleStorage> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}();

static std::vector<AngularCoordinate_DoubleStorage> gDataDouble = []()
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  std::vector<AngularCoordinate_DoubleStorage> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}();

// =============================================================================
// Benchmark: Accessor Performance
// =============================================================================

static void BM_Accessor_AngleStorage(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataAngle[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_AngleStorage);

static void BM_Accessor_DoubleStorage(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataDouble[idx++ % kDataSize];
    sum += a.pitch() + a.roll() + a.yaw();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Accessor_DoubleStorage);

// =============================================================================
// Benchmark: Arithmetic (a + b * 2 - c)
// =============================================================================

static void BM_Arithmetic_AngleStorage(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_AngleStorage result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataAngle[idx % kDataSize];
    const auto& b = gDataAngle[(idx + 1) % kDataSize];
    const auto& c = gDataAngle[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_AngleStorage);

static void BM_Arithmetic_DoubleStorage(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_DoubleStorage result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataDouble[idx % kDataSize];
    const auto& b = gDataDouble[(idx + 1) % kDataSize];
    const auto& c = gDataDouble[(idx + 2) % kDataSize];
    result = a + b * 2.0 - c;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Arithmetic_DoubleStorage);

// =============================================================================
// Benchmark: Cross Product
// =============================================================================

static void BM_CrossProduct_AngleStorage(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_AngleStorage result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataAngle[idx % kDataSize];
    const auto& b = gDataAngle[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_AngleStorage);

static void BM_CrossProduct_DoubleStorage(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate_DoubleStorage result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataDouble[idx % kDataSize];
    const auto& b = gDataDouble[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_DoubleStorage);

// =============================================================================
// Benchmark: Norm
// =============================================================================

static void BM_Norm_AngleStorage(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataAngle[idx++ % kDataSize];
    sum += a.norm();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Norm_AngleStorage);

static void BM_Norm_DoubleStorage(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataDouble[idx++ % kDataSize];
    sum += a.norm();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Norm_DoubleStorage);

// =============================================================================
// Memory Size Check (printed at startup)
// =============================================================================

static void BM_MemorySize_Report(benchmark::State& state)
{
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(sizeof(AngularCoordinate_AngleStorage));
    benchmark::DoNotOptimize(sizeof(AngularCoordinate_DoubleStorage));
  }
  state.SetLabel(
    "AngleStorage=" + std::to_string(sizeof(AngularCoordinate_AngleStorage)) +
    "B, DoubleStorage=" +
    std::to_string(sizeof(AngularCoordinate_DoubleStorage)) + "B");
}
BENCHMARK(BM_MemorySize_Report);
