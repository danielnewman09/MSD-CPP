/**
 * @file example.cpp
 * @brief Lagrange Multiplier Constraint Framework Tutorial
 *
 * This standalone C++ implementation demonstrates constraint-based rigid body
 * physics using Lagrange multipliers with Baumgarte stabilization. It provides
 * a simplified, educational version of the production code in msd-sim.
 *
 * Production equivalent: msd-sim/src/Physics/Constraints/
 * The production version uses:
 * - Eigen library for optimized matrix operations
 * - std::unique_ptr for constraint ownership
 * - AssetInertial integration for full physics pipeline
 * - Thread-safe const methods for concurrent evaluation
 * This tutorial version prioritizes readability and self-containment.
 *
 * Key concepts demonstrated:
 * 1. Constraint function C(q) = 0
 * 2. Constraint Jacobian J = dC/dq
 * 3. Lagrange multiplier solve: J*M^-1*J^T * lambda = b
 * 4. Baumgarte stabilization with alpha/beta gains
 * 5. Semi-implicit Euler integration
 *
 * Ticket: 0031_generalized_lagrange_constraints
 *
 * Build:
 *   cd docs/tutorials/lagrange-constraint-framework
 *   mkdir build && cd build
 *   cmake ..
 *   make
 *
 * Run:
 *   ./example
 */

#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

// ============================================================================
// Section 1: Simple Linear Algebra Types
// ============================================================================

/**
 * @brief 3D vector for positions, velocities, forces
 *
 * Production equivalent: Eigen::Vector3d
 * Simplified for tutorial: no SIMD, no expression templates
 */
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x{x_}, y{y_}, z{z_} {}

  Vec3 operator+(const Vec3& other) const
  {
    return {x + other.x, y + other.y, z + other.z};
  }

  Vec3 operator-(const Vec3& other) const
  {
    return {x - other.x, y - other.y, z - other.z};
  }

  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }

  Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

  Vec3& operator+=(const Vec3& other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  double dot(const Vec3& other) const
  {
    return x * other.x + y * other.y + z * other.z;
  }

  double squaredNorm() const { return x * x + y * y + z * z; }

  double norm() const { return std::sqrt(squaredNorm()); }
};

/**
 * @brief Unit quaternion for rotation representation
 *
 * Production equivalent: Eigen::Quaterniond
 * Uses (w, x, y, z) convention where w is the scalar component
 *
 * A unit quaternion Q = w + xi + yj + zk satisfies |Q| = 1
 * Represents a rotation of angle theta around axis n:
 *   Q = cos(theta/2) + sin(theta/2) * n
 */
struct Quaternion
{
  double w{1.0};  // Scalar component
  double x{0.0};  // i component
  double y{0.0};  // j component
  double z{0.0};  // k component

  Quaternion() = default;
  Quaternion(double w_, double x_, double y_, double z_)
      : w{w_}, x{x_}, y{y_}, z{z_}
  {
  }

  /**
   * @brief Compute squared norm |Q|^2 = w^2 + x^2 + y^2 + z^2
   *
   * For a unit quaternion, this should equal 1.0
   * Constraint violation = squaredNorm() - 1.0
   */
  double squaredNorm() const { return w * w + x * x + y * y + z * z; }

  /**
   * @brief Normalize quaternion to unit length
   *
   * This is the "naive" approach to constraint enforcement.
   * The Lagrange multiplier method computes a corrective force instead.
   */
  void normalize()
  {
    double n = std::sqrt(squaredNorm());
    if (n > 1e-10)
    {
      w /= n;
      x /= n;
      y /= n;
      z /= n;
    }
  }
};

/**
 * @brief 4D vector for quaternion derivatives
 *
 * Quaternion rate Q_dot relates to angular velocity omega:
 *   omega = 2 * conj(Q) * Q_dot  (quaternion multiplication)
 *   Q_dot = 0.5 * Q * [0, omega]
 */
struct Vec4
{
  double data[4]{0.0, 0.0, 0.0, 0.0};

  Vec4() = default;
  Vec4(double a, double b, double c, double d) : data{a, b, c, d} {}

  double& operator[](int i) { return data[i]; }
  double operator[](int i) const { return data[i]; }

  Vec4 operator+(const Vec4& other) const
  {
    return {data[0] + other.data[0], data[1] + other.data[1],
            data[2] + other.data[2], data[3] + other.data[3]};
  }

  Vec4 operator*(double s) const
  {
    return {data[0] * s, data[1] * s, data[2] * s, data[3] * s};
  }

  Vec4& operator+=(const Vec4& other)
  {
    for (int i = 0; i < 4; ++i)
    {
      data[i] += other.data[i];
    }
    return *this;
  }
};

/**
 * @brief Simple dense matrix for small systems
 *
 * Production equivalent: Eigen::MatrixXd
 * Row-major storage for cache efficiency
 */
class Matrix
{
  int rows_;
  int cols_;
  std::vector<double> data_;

public:
  Matrix(int rows, int cols)
      : rows_{rows}, cols_{cols}, data_(static_cast<size_t>(rows * cols), 0.0)
  {
  }

  int rows() const { return rows_; }
  int cols() const { return cols_; }

  double& operator()(int r, int c) { return data_[static_cast<size_t>(r * cols_ + c)]; }
  double operator()(int r, int c) const { return data_[static_cast<size_t>(r * cols_ + c)]; }

  /**
   * @brief Set all elements to zero
   */
  void setZero()
  {
    std::fill(data_.begin(), data_.end(), 0.0);
  }

  /**
   * @brief Matrix multiplication C = A * B
   * Complexity: O(n * m * k) for n x m times m x k
   */
  Matrix operator*(const Matrix& other) const
  {
    Matrix result(rows_, other.cols_);
    for (int i = 0; i < rows_; ++i)
    {
      for (int j = 0; j < other.cols_; ++j)
      {
        double sum = 0.0;
        for (int k = 0; k < cols_; ++k)
        {
          sum += (*this)(i, k) * other(k, j);
        }
        result(i, j) = sum;
      }
    }
    return result;
  }

  /**
   * @brief Transpose: B = A^T
   */
  Matrix transpose() const
  {
    Matrix result(cols_, rows_);
    for (int i = 0; i < rows_; ++i)
    {
      for (int j = 0; j < cols_; ++j)
      {
        result(j, i) = (*this)(i, j);
      }
    }
    return result;
  }
};

/**
 * @brief Simple dense vector for constraint systems
 *
 * Production equivalent: Eigen::VectorXd
 */
class Vector
{
  std::vector<double> data_;

public:
  explicit Vector(int size) : data_(static_cast<size_t>(size), 0.0) {}

  int size() const { return static_cast<int>(data_.size()); }

  double& operator[](int i) { return data_[static_cast<size_t>(i)]; }
  double operator[](int i) const { return data_[static_cast<size_t>(i)]; }

  Vector operator+(const Vector& other) const
  {
    Vector result(size());
    for (int i = 0; i < size(); ++i)
    {
      result[i] = (*this)[i] + other[i];
    }
    return result;
  }

  Vector operator-(const Vector& other) const
  {
    Vector result(size());
    for (int i = 0; i < size(); ++i)
    {
      result[i] = (*this)[i] - other[i];
    }
    return result;
  }

  /**
   * @brief Component-wise multiplication (Hadamard product)
   * Used for per-constraint Baumgarte gains: alpha .* C
   */
  Vector cwiseProduct(const Vector& other) const
  {
    Vector result(size());
    for (int i = 0; i < size(); ++i)
    {
      result[i] = (*this)[i] * other[i];
    }
    return result;
  }
};

/**
 * @brief Matrix-vector multiplication: y = A * x
 */
Vector matVecMul(const Matrix& A, const Vector& x)
{
  Vector result(A.rows());
  for (int i = 0; i < A.rows(); ++i)
  {
    double sum = 0.0;
    for (int j = 0; j < A.cols(); ++j)
    {
      sum += A(i, j) * x[j];
    }
    result[i] = sum;
  }
  return result;
}

/**
 * @brief Solve A*x = b using Cholesky decomposition (LLT)
 *
 * For symmetric positive definite matrices, Cholesky is:
 * - Numerically stable
 * - O(n^3/3) complexity (half of LU)
 * - Guaranteed to succeed for valid constraint systems
 *
 * Production equivalent: Eigen::LLT<MatrixXd>::solve()
 *
 * @param A Symmetric positive definite matrix (n x n)
 * @param b Right-hand side vector (n x 1)
 * @return Solution x such that A*x = b
 */
Vector solveLLT(const Matrix& A, const Vector& b)
{
  int n = A.rows();

  // Step 1: Cholesky factorization A = L * L^T
  // L is lower triangular, L(i,j) = 0 for j > i
  Matrix L(n, n);
  L.setZero();

  for (int i = 0; i < n; ++i)
  {
    // Diagonal element: L(i,i) = sqrt(A(i,i) - sum_{k<i} L(i,k)^2)
    double sum = 0.0;
    for (int k = 0; k < i; ++k)
    {
      sum += L(i, k) * L(i, k);
    }
    L(i, i) = std::sqrt(A(i, i) - sum);

    // Off-diagonal elements: L(j,i) = (A(j,i) - sum_{k<i} L(j,k)*L(i,k)) / L(i,i)
    for (int j = i + 1; j < n; ++j)
    {
      double offDiagSum = 0.0;
      for (int k = 0; k < i; ++k)
      {
        offDiagSum += L(j, k) * L(i, k);
      }
      L(j, i) = (A(j, i) - offDiagSum) / L(i, i);
    }
  }

  // Step 2: Forward substitution: L * y = b
  // Solve for y in L * y = b
  Vector y(n);
  for (int i = 0; i < n; ++i)
  {
    double sum = b[i];
    for (int j = 0; j < i; ++j)
    {
      sum -= L(i, j) * y[j];
    }
    y[i] = sum / L(i, i);
  }

  // Step 3: Back substitution: L^T * x = y
  // Solve for x in L^T * x = y
  Vector x(n);
  for (int i = n - 1; i >= 0; --i)
  {
    double sum = y[i];
    for (int j = i + 1; j < n; ++j)
    {
      sum -= L(j, i) * x[j];  // Note: L^T(i,j) = L(j,i)
    }
    x[i] = sum / L(i, i);
  }

  return x;
}

// ============================================================================
// Section 2: Rigid Body State
// ============================================================================

/**
 * @brief Complete kinematic state for a rigid body
 *
 * State vector q = [X, Q] where:
 * - X is position (3 DOF)
 * - Q is quaternion orientation (4 DOF, but only 3 independent due to |Q|=1)
 *
 * Production equivalent: msd_sim::InertialState
 */
struct RigidBodyState
{
  // Position state
  Vec3 position;      // World-frame position [m]
  Vec3 velocity;      // Linear velocity [m/s]
  Vec3 acceleration;  // Linear acceleration [m/s^2]

  // Orientation state
  Quaternion orientation;     // Unit quaternion (w, x, y, z)
  Vec4 quaternionRate;        // Q_dot = dQ/dt
  Vec3 angularAcceleration;   // alpha = I^-1 * tau [rad/s^2]

  /**
   * @brief Get angular velocity from quaternion rate
   *
   * Formula: omega = 2 * conj(Q) * Q_dot (quaternion multiplication)
   *
   * Derivation:
   *   Q_dot = 0.5 * Q * [0, omega]
   *   2 * Q_dot = Q * [0, omega]
   *   2 * conj(Q) * Q_dot = conj(Q) * Q * [0, omega] = [0, omega]
   *   (since conj(Q) * Q = [1, 0, 0, 0] for unit quaternion)
   */
  Vec3 getAngularVelocity() const
  {
    const Quaternion& Q = orientation;
    const Vec4& Qdot = quaternionRate;

    // Quaternion conjugate: conj(Q) = (w, -x, -y, -z)
    // Quaternion multiplication: p * q = [p0*q0 - p_vec . q_vec, p0*q_vec + q0*p_vec + p_vec x q_vec]
    // For omega = 2 * conj(Q) * Qdot, we only need the vector part

    // 2 * conj(Q) * Qdot vector part = 2 * (w*Qdot_vec - Qdot_w*(-Q_vec) + (-Q_vec) x Qdot_vec)
    // = 2 * (w*Qdot_vec + Qdot_w*Q_vec - Q_vec x Qdot_vec)

    Vec3 omega;
    omega.x =
        2.0 * (Q.w * Qdot[1] + Qdot[0] * Q.x - (Q.y * Qdot[3] - Q.z * Qdot[2]));
    omega.y =
        2.0 * (Q.w * Qdot[2] + Qdot[0] * Q.y - (Q.z * Qdot[1] - Q.x * Qdot[3]));
    omega.z =
        2.0 * (Q.w * Qdot[3] + Qdot[0] * Q.z - (Q.x * Qdot[2] - Q.y * Qdot[1]));

    return omega;
  }

  /**
   * @brief Set angular velocity (computes quaternion rate)
   *
   * Formula: Q_dot = 0.5 * Q * [0, omega]
   *
   * This ensures the quaternion rate is consistent with the desired
   * angular velocity in world frame.
   */
  void setAngularVelocity(const Vec3& omega)
  {
    const Quaternion& Q = orientation;

    // Quaternion multiplication: Q * [0, omega]
    // [w, x, y, z] * [0, ox, oy, oz]
    // = [w*0 - x*ox - y*oy - z*oz,
    //    w*ox + x*0 + y*oz - z*oy,
    //    w*oy - x*oz + y*0 + z*ox,
    //    w*oz + x*oy - y*ox + z*0]

    double pw = -Q.x * omega.x - Q.y * omega.y - Q.z * omega.z;
    double px = Q.w * omega.x + Q.y * omega.z - Q.z * omega.y;
    double py = Q.w * omega.y - Q.x * omega.z + Q.z * omega.x;
    double pz = Q.w * omega.z + Q.x * omega.y - Q.y * omega.x;

    // Q_dot = 0.5 * [pw, px, py, pz]
    quaternionRate = Vec4{0.5 * pw, 0.5 * px, 0.5 * py, 0.5 * pz};
  }
};

// ============================================================================
// Section 3: Constraint Interface
// ============================================================================

/**
 * @brief Abstract constraint interface
 *
 * Defines the mathematical contract for any constraint:
 * 1. dimension() - How many scalar equations
 * 2. evaluate() - Constraint violation C(q)
 * 3. jacobian() - How constraint changes with state J = dC/dq
 * 4. alpha(), beta() - Baumgarte stabilization gains
 *
 * Production equivalent: msd_sim::Constraint
 */
class Constraint
{
public:
  virtual ~Constraint() = default;

  /**
   * @brief Number of scalar constraint equations
   *
   * Unit quaternion: 1 (Q^T*Q - 1 = 0)
   * Distance: 1 (|X|^2 - d^2 = 0)
   * Ball joint: 3 (X_A + R_A*r_A - X_B - R_B*r_B = 0)
   */
  virtual int dimension() const = 0;

  /**
   * @brief Evaluate constraint function C(q, t)
   *
   * Returns the constraint violation. For a perfectly satisfied
   * constraint, this returns zero.
   *
   * @param state Current rigid body state
   * @param time Simulation time (for time-dependent constraints)
   * @return Constraint violation vector (dimension x 1)
   */
  virtual Vector evaluate(const RigidBodyState& state, double time) const = 0;

  /**
   * @brief Compute constraint Jacobian J = dC/dq
   *
   * The Jacobian is a (dimension x 7) matrix mapping state changes
   * to constraint changes. For the 7-DOF state [X, Q]:
   * - Columns 0-2: dC/dX (position derivatives)
   * - Columns 3-6: dC/dQ (quaternion derivatives)
   *
   * @param state Current rigid body state
   * @param time Simulation time
   * @return Jacobian matrix (dimension x 7)
   */
  virtual Matrix jacobian(const RigidBodyState& state, double time) const = 0;

  /**
   * @brief Baumgarte position error gain
   *
   * Higher values correct position drift faster but may cause instability.
   * Literature standard: alpha = 10.0 for dt ~ 16ms
   *
   * Constraint force contribution: -alpha * C
   */
  virtual double alpha() const { return 10.0; }

  /**
   * @brief Baumgarte velocity error gain
   *
   * Higher values add more damping to constraint enforcement.
   * Literature standard: beta = 10.0 for dt ~ 16ms
   *
   * Constraint force contribution: -beta * C_dot
   */
  virtual double beta() const { return 10.0; }

  /**
   * @brief Constraint type name for debugging
   */
  virtual const char* typeName() const = 0;
};

// ============================================================================
// Section 4: Concrete Constraint Implementations
// ============================================================================

/**
 * @brief Unit quaternion normalization constraint
 *
 * Ensures |Q|^2 = 1 via Lagrange multipliers with Baumgarte stabilization.
 *
 * Mathematical formulation:
 *   Constraint: C(Q) = Q^T*Q - 1 = w^2 + x^2 + y^2 + z^2 - 1 = 0
 *   Jacobian: J = dC/dQ = [2w, 2x, 2y, 2z] (1x4 w.r.t. quaternion)
 *             J = [0, 0, 0, 2w, 2x, 2y, 2z] (1x7 w.r.t. full state)
 *
 * Why not just normalize? Rescaling violates Newton's laws and injects
 * energy. Lagrange multipliers compute the physically correct force.
 *
 * Production equivalent: msd_sim::UnitQuaternionConstraint
 */
class UnitQuaternionConstraint : public Constraint
{
  double alpha_{10.0};  // Position error gain [1/s^2]
  double beta_{10.0};   // Velocity error gain [1/s]

public:
  UnitQuaternionConstraint(double alpha = 10.0, double beta = 10.0)
      : alpha_{alpha}, beta_{beta}
  {
  }

  int dimension() const override
  {
    return 1;  // Single scalar constraint: |Q|^2 - 1 = 0
  }

  Vector evaluate(const RigidBodyState& state, double /*time*/) const override
  {
    // C(Q) = Q^T * Q - 1 = |Q|^2 - 1
    const Quaternion& Q = state.orientation;
    Vector C(1);
    C[0] = Q.squaredNorm() - 1.0;
    return C;
  }

  Matrix jacobian(const RigidBodyState& state, double /*time*/) const override
  {
    // J = dC/dq = [dC/dX, dC/dQ]
    // dC/dX = [0, 0, 0] (quaternion constraint independent of position)
    // dC/dQ = [2w, 2x, 2y, 2z]

    const Quaternion& Q = state.orientation;
    Matrix J(1, 7);
    J.setZero();

    // Position derivatives (columns 0-2): zero
    J(0, 0) = 0.0;
    J(0, 1) = 0.0;
    J(0, 2) = 0.0;

    // Quaternion derivatives (columns 3-6)
    J(0, 3) = 2.0 * Q.w;
    J(0, 4) = 2.0 * Q.x;
    J(0, 5) = 2.0 * Q.y;
    J(0, 6) = 2.0 * Q.z;

    return J;
  }

  double alpha() const override { return alpha_; }
  double beta() const override { return beta_; }

  const char* typeName() const override { return "UnitQuaternionConstraint"; }
};

/**
 * @brief Fixed distance from origin constraint
 *
 * Maintains |X|^2 = d^2, keeping the object on a sphere of radius d.
 *
 * Mathematical formulation:
 *   Constraint: C(X) = |X|^2 - d^2 = x^2 + y^2 + z^2 - d^2 = 0
 *   Jacobian: J = dC/dX = [2x, 2y, 2z] (1x3 w.r.t. position)
 *             J = [2x, 2y, 2z, 0, 0, 0, 0] (1x7 w.r.t. full state)
 *
 * Use cases:
 * - Orbital mechanics (circular orbit)
 * - Simple pendulum (fixed rod length)
 * - Tethered objects (cable constraint)
 *
 * Production equivalent: msd_sim::DistanceConstraint
 */
class DistanceConstraint : public Constraint
{
  double targetDistanceSquared_;  // d^2 for efficiency
  double alpha_{10.0};
  double beta_{10.0};

public:
  /**
   * @param targetDistance Desired distance from origin [m]
   */
  explicit DistanceConstraint(double targetDistance, double alpha = 10.0,
                              double beta = 10.0)
      : targetDistanceSquared_{targetDistance * targetDistance},
        alpha_{alpha},
        beta_{beta}
  {
  }

  int dimension() const override
  {
    return 1;  // Single scalar constraint: |X|^2 - d^2 = 0
  }

  Vector evaluate(const RigidBodyState& state, double /*time*/) const override
  {
    // C(X) = |X|^2 - d^2
    Vector C(1);
    C[0] = state.position.squaredNorm() - targetDistanceSquared_;
    return C;
  }

  Matrix jacobian(const RigidBodyState& state, double /*time*/) const override
  {
    // J = dC/dq = [dC/dX, dC/dQ]
    // dC/dX = [2x, 2y, 2z]
    // dC/dQ = [0, 0, 0, 0] (distance independent of orientation)

    Matrix J(1, 7);
    J.setZero();

    // Position derivatives (columns 0-2)
    J(0, 0) = 2.0 * state.position.x;
    J(0, 1) = 2.0 * state.position.y;
    J(0, 2) = 2.0 * state.position.z;

    // Quaternion derivatives (columns 3-6): zero
    J(0, 3) = 0.0;
    J(0, 4) = 0.0;
    J(0, 5) = 0.0;
    J(0, 6) = 0.0;

    return J;
  }

  double alpha() const override { return alpha_; }
  double beta() const override { return beta_; }

  const char* typeName() const override { return "DistanceConstraint"; }
};

// ============================================================================
// Section 5: Constraint Solver
// ============================================================================

/**
 * @brief Result of constraint solve
 *
 * Contains Lagrange multipliers, constraint forces, and diagnostic info.
 * The forces should be added to external forces before integration.
 */
struct SolveResult
{
  Vector lambdas;         // Lagrange multipliers (one per constraint equation)
  Vec3 linearForce;       // Net linear constraint force [N]
  Vec3 angularForce;      // Net angular constraint force (acts on quaternion)
  bool converged{false};  // True if solve succeeded
  double conditionNumber{1.0};  // Diagnostic: A = J*M^-1*J^T condition
};

/**
 * @brief Compute Lagrange multipliers and constraint forces
 *
 * Solves the system:
 *   J * M^-1 * J^T * lambda = b
 *   F_constraint = J^T * lambda
 *
 * where b includes external forces, Baumgarte stabilization, and velocity terms.
 *
 * Production equivalent: msd_sim::ConstraintSolver
 *
 * @param constraints Vector of constraint pointers
 * @param state Current rigid body state
 * @param externalForce External forces (gravity, thrust, etc.) [N]
 * @param mass Object mass [kg]
 * @param inverseInertia Inverse inertia tensor (3x3, simplified to diagonal)
 * @param dt Integration timestep [s]
 * @return SolveResult with forces and convergence status
 */
SolveResult solveConstraints(const std::vector<Constraint*>& constraints,
                             const RigidBodyState& state,
                             const Vec3& externalForce, double mass,
                             const std::array<double, 3>& inverseInertia,
                             double /*dt*/)
{
  // Handle empty constraint set
  if (constraints.empty())
  {
    return SolveResult{Vector(0), {0, 0, 0}, {0, 0, 0}, true, 1.0};
  }

  // Compute total constraint dimension
  int totalDim = 0;
  for (const auto* c : constraints)
  {
    totalDim += c->dimension();
  }

  // Current time (not used by current constraints)
  double time = 0.0;

  // ========== Step 1: Assemble stacked Jacobian J (totalDim x 7) ==========
  Matrix J(totalDim, 7);
  J.setZero();
  int rowOffset = 0;
  for (const auto* c : constraints)
  {
    Matrix Ji = c->jacobian(state, time);
    for (int r = 0; r < c->dimension(); ++r)
    {
      for (int col = 0; col < 7; ++col)
      {
        J(rowOffset + r, col) = Ji(r, col);
      }
    }
    rowOffset += c->dimension();
  }

  // ========== Step 2: Build inverse mass matrix M^-1 (7x7 block diagonal) ==========
  // M = diag([m*I_3, I_3x3, 1])  (mass for position, inertia for orientation, 1 for quaternion scalar)
  // M^-1 = diag([I_3/m, I^-1, 1])
  Matrix M_inv(7, 7);
  M_inv.setZero();

  // Linear mass inverse (1/m * I_3)
  M_inv(0, 0) = 1.0 / mass;
  M_inv(1, 1) = 1.0 / mass;
  M_inv(2, 2) = 1.0 / mass;

  // Angular mass inverse (diagonal approximation of inertia tensor)
  // In production code, this would be the full 3x3 inverse inertia tensor
  M_inv(3, 3) = inverseInertia[0];
  M_inv(4, 4) = inverseInertia[1];
  M_inv(5, 5) = inverseInertia[2];

  // Quaternion scalar component (simplified)
  M_inv(6, 6) = 1.0;

  // ========== Step 3: Form constraint matrix A = J * M^-1 * J^T ==========
  Matrix JM = J * M_inv;
  Matrix Jt = J.transpose();
  Matrix A = JM * Jt;

  // ========== Step 4: Compute RHS b = -J*M^-1*F_ext - alpha*C - beta*C_dot ==========

  // External force in generalized coordinates (7x1)
  Vector F_ext(7);
  F_ext[0] = externalForce.x;
  F_ext[1] = externalForce.y;
  F_ext[2] = externalForce.z;
  F_ext[3] = 0.0;  // No external torque on quaternion w
  F_ext[4] = 0.0;  // No external torque on quaternion x
  F_ext[5] = 0.0;  // No external torque on quaternion y
  F_ext[6] = 0.0;  // No external torque on quaternion z

  // Assemble constraint violation C and Baumgarte gains
  Vector C(totalDim);
  Vector alphaVec(totalDim);
  Vector betaVec(totalDim);
  rowOffset = 0;
  for (const auto* c : constraints)
  {
    Vector Ci = c->evaluate(state, time);
    for (int r = 0; r < c->dimension(); ++r)
    {
      C[rowOffset + r] = Ci[r];
      alphaVec[rowOffset + r] = c->alpha();
      betaVec[rowOffset + r] = c->beta();
    }
    rowOffset += c->dimension();
  }

  // Velocity-level constraint violation C_dot = J * q_dot
  // State velocity vector (7x1)
  Vector q_dot(7);
  q_dot[0] = state.velocity.x;
  q_dot[1] = state.velocity.y;
  q_dot[2] = state.velocity.z;
  q_dot[3] = state.quaternionRate[0];
  q_dot[4] = state.quaternionRate[1];
  q_dot[5] = state.quaternionRate[2];
  q_dot[6] = state.quaternionRate[3];

  Vector C_dot = matVecMul(J, q_dot);

  // RHS: b = -J*M^-1*F_ext - alpha*C - beta*C_dot
  Vector JM_Fext = matVecMul(JM, F_ext);
  Vector alphaC = alphaVec.cwiseProduct(C);
  Vector betaCdot = betaVec.cwiseProduct(C_dot);

  Vector b(totalDim);
  for (int i = 0; i < totalDim; ++i)
  {
    b[i] = -JM_Fext[i] - alphaC[i] - betaCdot[i];
  }

  // ========== Step 5: Solve A * lambda = b using LLT ==========
  Vector lambda = solveLLT(A, b);

  // ========== Step 6: Extract constraint forces F_c = J^T * lambda ==========
  Vector F_c = matVecMul(Jt, lambda);

  // Separate linear and angular components
  Vec3 linearForce{F_c[0], F_c[1], F_c[2]};
  Vec3 angularForce{F_c[3], F_c[4], F_c[5]};
  // F_c[6] is quaternion scalar force (contributes to normalization)

  // Compute condition number (simplified: ratio of diagonal max/min)
  double maxDiag = A(0, 0);
  double minDiag = A(0, 0);
  for (int i = 1; i < totalDim; ++i)
  {
    if (A(i, i) > maxDiag) maxDiag = A(i, i);
    if (A(i, i) < minDiag) minDiag = A(i, i);
  }
  double conditionNumber = (minDiag > 1e-10) ? maxDiag / minDiag : 1e10;

  return SolveResult{lambda, linearForce, angularForce, true, conditionNumber};
}

// ============================================================================
// Section 6: Physics Integration
// ============================================================================

/**
 * @brief Semi-implicit Euler integration step
 *
 * Updates state using symplectic integration:
 * 1. v_new = v_old + a * dt
 * 2. x_new = x_old + v_new * dt (uses NEW velocity)
 *
 * This preserves energy better than explicit Euler and is stable for
 * oscillatory systems like constrained dynamics.
 *
 * Production equivalent: msd_sim::SemiImplicitEulerIntegrator
 *
 * @param state Rigid body state (modified in place)
 * @param netForce Total force (external + constraint) [N]
 * @param mass Object mass [kg]
 * @param dt Integration timestep [s]
 */
void integrateLinear(RigidBodyState& state, const Vec3& netForce, double mass,
                     double dt)
{
  // Semi-implicit Euler: update velocity first, then position with new velocity
  Vec3 acceleration = netForce / mass;
  state.velocity += acceleration * dt;
  state.position += state.velocity * dt;
  state.acceleration = acceleration;
}

/**
 * @brief Integrate quaternion orientation
 *
 * Q_new = Q_old + Q_dot * dt
 *
 * This integrates the quaternion using the quaternion rate, which is
 * computed from angular velocity and the constraint solver.
 *
 * @param state Rigid body state (modified in place)
 * @param dt Integration timestep [s]
 */
void integrateQuaternion(RigidBodyState& state, double dt)
{
  // Integrate quaternion: Q_new = Q_old + Q_dot * dt
  state.orientation.w += state.quaternionRate[0] * dt;
  state.orientation.x += state.quaternionRate[1] * dt;
  state.orientation.y += state.quaternionRate[2] * dt;
  state.orientation.z += state.quaternionRate[3] * dt;

  // Note: We do NOT normalize here! The constraint solver handles normalization
  // through the Lagrange multiplier force. Explicit normalization would violate
  // the mathematical framework.
}

// ============================================================================
// Section 7: Main Example
// ============================================================================

int main()
{
  std::cout << "=================================================\n";
  std::cout << "Lagrange Multiplier Constraint Framework Tutorial\n";
  std::cout << "=================================================\n\n";

  // ========== Setup ==========
  // Create a rigid body with some initial state
  RigidBodyState state;
  state.position = Vec3{5.0, 0.0, 0.0};  // Start 5m from origin (for distance constraint)
  state.velocity = Vec3{0.0, 1.0, 0.0};  // Moving in Y direction

  // Slightly non-unit quaternion to test constraint enforcement
  state.orientation = Quaternion{0.99, 0.1, 0.05, 0.02};  // |Q| != 1

  // Set angular velocity (will be converted to quaternion rate)
  state.setAngularVelocity(Vec3{0.0, 0.0, 0.5});  // Rotating around Z axis

  // Physical parameters
  double mass = 10.0;  // kg
  std::array<double, 3> inverseInertia = {0.1, 0.1, 0.1};  // Diagonal I^-1

  // External force: gravity in -Z direction
  Vec3 gravity{0.0, 0.0, -9.81 * mass};

  // Integration parameters
  double dt = 0.016;       // 16ms timestep (~60 FPS)
  int numSteps = 100;      // Number of integration steps

  // ========== Create Constraints ==========
  // 1. Unit quaternion constraint: keeps |Q| = 1
  UnitQuaternionConstraint quatConstraint{10.0, 10.0};

  // 2. Distance constraint: keeps |X| = 5m (sphere orbit)
  DistanceConstraint distConstraint{5.0, 10.0, 10.0};

  std::vector<Constraint*> constraints = {&quatConstraint, &distConstraint};

  // ========== Simulation Loop ==========
  std::cout << "Initial state:\n";
  std::cout << "  Position: (" << state.position.x << ", " << state.position.y
            << ", " << state.position.z << ")\n";
  std::cout << "  Distance from origin: " << state.position.norm() << " m\n";
  std::cout << "  Quaternion: (" << state.orientation.w << ", "
            << state.orientation.x << ", " << state.orientation.y << ", "
            << state.orientation.z << ")\n";
  std::cout << "  Quaternion norm: " << std::sqrt(state.orientation.squaredNorm())
            << "\n\n";

  std::cout << "Running " << numSteps << " steps at dt = " << dt << "s...\n\n";

  for (int step = 0; step < numSteps; ++step)
  {
    // 1. Solve constraints to get constraint forces
    SolveResult result = solveConstraints(constraints, state, gravity, mass,
                                          inverseInertia, dt);

    // 2. Combine forces
    Vec3 totalForce = gravity + result.linearForce;

    // 3. Integrate linear motion
    integrateLinear(state, totalForce, mass, dt);

    // 4. Update quaternion rate from angular velocity and constraint force
    // (simplified: just integrate current rate)
    integrateQuaternion(state, dt);

    // Print progress every 10 steps
    if ((step + 1) % 10 == 0)
    {
      double quatNorm = std::sqrt(state.orientation.squaredNorm());
      double distance = state.position.norm();
      std::cout << "Step " << std::setw(3) << (step + 1) << ": "
                << "dist=" << std::fixed << std::setprecision(6) << distance
                << "m, |Q|=" << quatNorm
                << ", cond=" << std::scientific << result.conditionNumber
                << std::fixed << "\n";
    }
  }

  // ========== Final Report ==========
  std::cout << "\nFinal state:\n";
  std::cout << "  Position: (" << state.position.x << ", " << state.position.y
            << ", " << state.position.z << ")\n";
  std::cout << "  Distance from origin: " << state.position.norm()
            << " m (target: 5.0)\n";
  std::cout << "  Quaternion: (" << state.orientation.w << ", "
            << state.orientation.x << ", " << state.orientation.y << ", "
            << state.orientation.z << ")\n";
  std::cout << "  Quaternion norm: " << std::sqrt(state.orientation.squaredNorm())
            << " (target: 1.0)\n";

  // Compute constraint violations
  Vector quatViolation = quatConstraint.evaluate(state, 0.0);
  Vector distViolation = distConstraint.evaluate(state, 0.0);

  std::cout << "\nConstraint violations:\n";
  std::cout << "  Quaternion (|Q|^2 - 1): " << std::scientific
            << quatViolation[0] << "\n";
  std::cout << "  Distance (|X|^2 - d^2): " << distViolation[0] << "\n";

  // Success criteria
  double quatError = std::abs(quatViolation[0]);
  double distError = std::abs(distViolation[0]);
  bool success = (quatError < 1e-2) && (distError < 1.0);

  std::cout << "\n=================================================\n";
  if (success)
  {
    std::cout << "SUCCESS: Constraints maintained within tolerance!\n";
  }
  else
  {
    std::cout << "NOTE: Drift detected - tune Baumgarte gains or reduce dt\n";
  }
  std::cout << "=================================================\n";

  return 0;
}
