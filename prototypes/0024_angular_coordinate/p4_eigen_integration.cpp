// Prototype P4: Eigen Integration
// Ticket: 0024_angular_coordinate
// Purpose: Verify SIMD optimizations are preserved when inheriting from Eigen::Vector3d

#include <Eigen/Dense>
#include <benchmark/benchmark.h>
#include <cmath>
#include <random>
#include <vector>

// Normalization function
inline double normalizeAngle(double rad)
{
  constexpr double twoPi = 2.0 * M_PI;
  double result = std::fmod(rad + M_PI, twoPi);
  return result <= 0.0 ? result + M_PI : result - M_PI;
}

// AngularCoordinate inheriting from Eigen::Vector3d (eager normalization)
class AngularCoordinate : public Eigen::Vector3d
{
public:
  AngularCoordinate() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }
  AngularCoordinate(double pitch, double roll, double yaw)
    : Eigen::Vector3d{normalizeAngle(pitch), normalizeAngle(roll), normalizeAngle(yaw)}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
  }

  template <typename OtherDerived>
  AngularCoordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
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
// Test Data Generation
// =============================================================================

constexpr size_t kDataSize = 1024;

static std::vector<Eigen::Vector3d> gDataEigen = []() {
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  std::vector<Eigen::Vector3d> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}();

static std::vector<AngularCoordinate> gDataAngular = []() {
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-10.0, 10.0);
  std::vector<AngularCoordinate> data;
  data.reserve(kDataSize);
  for (size_t i = 0; i < kDataSize; ++i)
  {
    data.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return data;
}();

static Eigen::Matrix3d gRotationMatrix =
    Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();

// =============================================================================
// Benchmark: Vector Addition
// =============================================================================

static void BM_Addition_Eigen(benchmark::State& state)
{
  size_t idx = 0;
  Eigen::Vector3d result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataEigen[idx % kDataSize];
    const auto& b = gDataEigen[(idx + 1) % kDataSize];
    result = a + b;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Addition_Eigen);

static void BM_Addition_Angular(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataAngular[idx % kDataSize];
    const auto& b = gDataAngular[(idx + 1) % kDataSize];
    result = a + b;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_Addition_Angular);

// =============================================================================
// Benchmark: Scalar Multiplication
// =============================================================================

static void BM_ScalarMultiply_Eigen(benchmark::State& state)
{
  size_t idx = 0;
  Eigen::Vector3d result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataEigen[idx++ % kDataSize];
    result = a * 2.5;
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(BM_ScalarMultiply_Eigen);

static void BM_ScalarMultiply_Angular(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataAngular[idx++ % kDataSize];
    result = a * 2.5;
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(BM_ScalarMultiply_Angular);

// =============================================================================
// Benchmark: Cross Product
// =============================================================================

static void BM_CrossProduct_Eigen(benchmark::State& state)
{
  size_t idx = 0;
  Eigen::Vector3d result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataEigen[idx % kDataSize];
    const auto& b = gDataEigen[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_Eigen);

static void BM_CrossProduct_Angular(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataAngular[idx % kDataSize];
    const auto& b = gDataAngular[(idx + 1) % kDataSize];
    result = a.cross(b);
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_CrossProduct_Angular);

// =============================================================================
// Benchmark: Dot Product
// =============================================================================

static void BM_DotProduct_Eigen(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataEigen[idx % kDataSize];
    const auto& b = gDataEigen[(idx + 1) % kDataSize];
    sum += a.dot(b);
    benchmark::DoNotOptimize(sum);
    idx++;
  }
}
BENCHMARK(BM_DotProduct_Eigen);

static void BM_DotProduct_Angular(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataAngular[idx % kDataSize];
    const auto& b = gDataAngular[(idx + 1) % kDataSize];
    sum += a.dot(b);
    benchmark::DoNotOptimize(sum);
    idx++;
  }
}
BENCHMARK(BM_DotProduct_Angular);

// =============================================================================
// Benchmark: Norm
// =============================================================================

static void BM_Norm_Eigen(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataEigen[idx++ % kDataSize];
    sum += a.norm();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Norm_Eigen);

static void BM_Norm_Angular(benchmark::State& state)
{
  double sum = 0.0;
  size_t idx = 0;
  for (auto _ : state)
  {
    const auto& a = gDataAngular[idx++ % kDataSize];
    sum += a.norm();
    benchmark::DoNotOptimize(sum);
  }
}
BENCHMARK(BM_Norm_Angular);

// =============================================================================
// Benchmark: Matrix Multiplication
// =============================================================================

static void BM_MatrixMultiply_Eigen(benchmark::State& state)
{
  size_t idx = 0;
  Eigen::Vector3d result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataEigen[idx++ % kDataSize];
    result = gRotationMatrix * a;
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(BM_MatrixMultiply_Eigen);

static void BM_MatrixMultiply_Angular(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataAngular[idx++ % kDataSize];
    result = gRotationMatrix * a;
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(BM_MatrixMultiply_Angular);

// =============================================================================
// Benchmark: Expression Templates (complex expression)
// =============================================================================

static void BM_ExpressionTemplate_Eigen(benchmark::State& state)
{
  size_t idx = 0;
  Eigen::Vector3d result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataEigen[idx % kDataSize];
    const auto& b = gDataEigen[(idx + 1) % kDataSize];
    const auto& c = gDataEigen[(idx + 2) % kDataSize];
    // Complex expression: (a + b * 2 - c).cross(a) + M * b
    result = (a + b * 2.0 - c).cross(a) + gRotationMatrix * b;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_ExpressionTemplate_Eigen);

static void BM_ExpressionTemplate_Angular(benchmark::State& state)
{
  size_t idx = 0;
  AngularCoordinate result{0, 0, 0};
  for (auto _ : state)
  {
    const auto& a = gDataAngular[idx % kDataSize];
    const auto& b = gDataAngular[(idx + 1) % kDataSize];
    const auto& c = gDataAngular[(idx + 2) % kDataSize];
    // Complex expression: (a + b * 2 - c).cross(a) + M * b
    result = (a + b * 2.0 - c).cross(a) + gRotationMatrix * b;
    benchmark::DoNotOptimize(result);
    idx++;
  }
}
BENCHMARK(BM_ExpressionTemplate_Angular);

// =============================================================================
// Benchmark: Memory Size Report
// =============================================================================

static void BM_MemorySize_Report(benchmark::State& state)
{
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(sizeof(Eigen::Vector3d));
    benchmark::DoNotOptimize(sizeof(AngularCoordinate));
  }
  state.SetLabel("Eigen::Vector3d=" + std::to_string(sizeof(Eigen::Vector3d)) +
                 "B, AngularCoordinate=" + std::to_string(sizeof(AngularCoordinate)) + "B");
}
BENCHMARK(BM_MemorySize_Report);
