// Prototype P1: Constraint Matrix Conditioning
// Question: Do typical constraint combinations produce well-conditioned matrices?
// Success criteria: Condition number < 1e12, graceful handling of singular matrices

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

// Mock InertialState for prototype (simplified from actual implementation)
struct MockInertialState {
    Eigen::Vector3d position{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d velocity{0.0, 0.0, 0.0};
    Eigen::Vector4d quaternionRate{0.0, 0.0, 0.0, 0.0};
};

// Mock constraint for quaternion normalization
struct QuaternionConstraintMock {
    // C(Q) = Q^T*Q - 1 = 0
    double evaluate(const MockInertialState& state) const {
        return state.orientation.squaredNorm() - 1.0;
    }

    // J = ∂C/∂Q = 2*Q^T (1x4 Jacobian w.r.t. quaternion components)
    Eigen::RowVectorXd jacobian(const MockInertialState& state) const {
        Eigen::RowVectorXd J(7);
        J.setZero();
        // Position components: no contribution
        J.segment<3>(0).setZero();
        // Quaternion components: 2*Q^T
        J.segment<4>(3) = 2.0 * state.orientation.coeffs().transpose();
        return J;
    }
};

// Mock constraint for distance from origin
struct DistanceConstraintMock {
    double targetDistance;

    explicit DistanceConstraintMock(double d) : targetDistance{d} {}

    // C(X) = |X|^2 - d^2 = 0
    double evaluate(const MockInertialState& state) const {
        return state.position.squaredNorm() - targetDistance * targetDistance;
    }

    // J = ∂C/∂X = 2*X^T (1x3 Jacobian w.r.t. position components)
    Eigen::RowVectorXd jacobian(const MockInertialState& state) const {
        Eigen::RowVectorXd J(7);
        J.setZero();
        // Position components: 2*X^T
        J.segment<3>(0) = 2.0 * state.position.transpose();
        // Quaternion components: no contribution
        J.segment<4>(3).setZero();
        return J;
    }
};

// Compute condition number using SVD
double computeConditionNumber(const Eigen::MatrixXd& A) {
    if (A.rows() == 0 || A.cols() == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
    auto singularValues = svd.singularValues();

    if (singularValues.size() == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double maxSV = singularValues(0);
    double minSV = singularValues(singularValues.size() - 1);

    if (std::abs(minSV) < 1e-15) {
        return std::numeric_limits<double>::infinity();
    }

    return maxSV / minSV;
}

// Assemble constraint matrix A = J * M^-1 * J^T
Eigen::MatrixXd assembleConstraintMatrix(
    const std::vector<Eigen::RowVectorXd>& jacobians,
    double mass,
    const Eigen::Matrix3d& inverseInertia)
{
    if (jacobians.empty()) {
        return Eigen::MatrixXd(0, 0);
    }

    int numConstraints = jacobians.size();
    Eigen::MatrixXd A(numConstraints, numConstraints);

    // Construct M^-1 (7x7 block diagonal: [m^-1*I3, I^-1])
    // Note: For quaternion DOF, we use a simplified 4x4 block with the same inertia pattern
    Eigen::MatrixXd Minv = Eigen::MatrixXd::Zero(7, 7);
    Minv.block<3,3>(0, 0) = (1.0 / mass) * Eigen::Matrix3d::Identity();

    // Quaternion inertia (simplified): use same inertia as rotation but extend to 4x4
    Eigen::Matrix4d quatInertia = Eigen::Matrix4d::Zero();
    quatInertia.topLeftCorner<3,3>() = inverseInertia;
    quatInertia(3,3) = inverseInertia(0,0);  // Use Ixx for scalar component
    Minv.block<4,4>(3, 3) = quatInertia;

    // Build A = J * M^-1 * J^T
    for (int i = 0; i < numConstraints; ++i) {
        for (int j = 0; j < numConstraints; ++j) {
            A(i, j) = jacobians[i] * Minv * jacobians[j].transpose();
        }
    }

    return A;
}

// Test LLT solve with condition number tracking
struct SolveResult {
    bool converged{false};
    double conditionNumber{std::numeric_limits<double>::quiet_NaN()};
    Eigen::VectorXd solution;
    bool hasNaN{false};
};

SolveResult testLLTSolve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    SolveResult result;

    // Compute condition number
    result.conditionNumber = computeConditionNumber(A);

    // Attempt LLT decomposition
    Eigen::LLT<Eigen::MatrixXd> llt(A);

    if (llt.info() != Eigen::Success) {
        result.converged = false;
        return result;
    }

    // Solve
    result.solution = llt.solve(b);
    result.converged = true;

    // Check for NaN
    result.hasNaN = !result.solution.allFinite();

    return result;
}

void printTestHeader(const std::string& testName) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << testName << "\n";
    std::cout << std::string(60, '=') << "\n";
}

void printResult(const std::string& metric, double value, bool pass) {
    std::cout << std::setw(30) << std::left << metric << ": ";
    std::cout << std::scientific << std::setprecision(3) << value;
    std::cout << " [" << (pass ? "PASS" : "FAIL") << "]\n";
}

void printResult(const std::string& metric, const std::string& value, bool pass) {
    std::cout << std::setw(30) << std::left << metric << ": ";
    std::cout << value;
    std::cout << " [" << (pass ? "PASS" : "FAIL") << "]\n";
}

int main() {
    std::cout << "Prototype P1: Constraint Matrix Conditioning\n";
    std::cout << "Testing constraint matrix condition numbers and LLT robustness\n";

    bool allTestsPassed = true;
    const double kMaxConditionNumber = 1e12;

    // Test 1: Single quaternion constraint
    {
        printTestHeader("Test 1: Single Quaternion Constraint");

        MockInertialState state;
        state.orientation = Eigen::Quaterniond(0.7071, 0.7071, 0.0, 0.0);  // 90° rotation
        state.position = Eigen::Vector3d(1.0, 2.0, 3.0);

        QuaternionConstraintMock qConstraint;

        std::vector<Eigen::RowVectorXd> jacobians;
        jacobians.push_back(qConstraint.jacobian(state));

        double mass = 10.0;
        Eigen::Matrix3d inverseInertia = Eigen::Matrix3d::Identity() * 0.1;

        Eigen::MatrixXd A = assembleConstraintMatrix(jacobians, mass, inverseInertia);
        Eigen::VectorXd b = Eigen::VectorXd::Ones(1);

        SolveResult result = testLLTSolve(A, b);

        std::cout << "Constraint dimension: 1\n";
        std::cout << "State dimension: 7 (3 pos + 4 quat)\n";

        bool condPass = result.conditionNumber < kMaxConditionNumber;
        bool convergePass = result.converged;
        bool nanPass = !result.hasNaN;

        printResult("Condition number", result.conditionNumber, condPass);
        printResult("LLT converged", result.converged ? "true" : "false", convergePass);
        printResult("No NaN in solution", !result.hasNaN ? "true" : "false", nanPass);

        allTestsPassed &= (condPass && convergePass && nanPass);
    }

    // Test 2: Quaternion + Distance constraint
    {
        printTestHeader("Test 2: Quaternion + Distance Constraint");

        MockInertialState state;
        state.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);  // Identity
        state.position = Eigen::Vector3d(5.0, 0.0, 0.0);  // 5m from origin

        QuaternionConstraintMock qConstraint;
        DistanceConstraintMock dConstraint{5.0};

        std::vector<Eigen::RowVectorXd> jacobians;
        jacobians.push_back(qConstraint.jacobian(state));
        jacobians.push_back(dConstraint.jacobian(state));

        double mass = 10.0;
        Eigen::Matrix3d inverseInertia = Eigen::Matrix3d::Identity() * 0.1;

        Eigen::MatrixXd A = assembleConstraintMatrix(jacobians, mass, inverseInertia);
        Eigen::VectorXd b = Eigen::VectorXd::Ones(2);

        SolveResult result = testLLTSolve(A, b);

        std::cout << "Constraint dimension: 2\n";
        std::cout << "State dimension: 7 (3 pos + 4 quat)\n";

        bool condPass = result.conditionNumber < kMaxConditionNumber;
        bool convergePass = result.converged;
        bool nanPass = !result.hasNaN;

        printResult("Condition number", result.conditionNumber, condPass);
        printResult("LLT converged", result.converged ? "true" : "false", convergePass);
        printResult("No NaN in solution", !result.hasNaN ? "true" : "false", nanPass);

        allTestsPassed &= (condPass && convergePass && nanPass);
    }

    // Test 3: Quaternion + Distance (realistic scenario)
    {
        printTestHeader("Test 3: Two Independent Constraints (Quat + Distance)");

        // Use a realistic scenario: quaternion + single distance constraint
        // These are independent since quaternion operates on orientation DOFs
        // and distance operates on position DOFs

        MockInertialState state;
        state.orientation = Eigen::Quaterniond(0.6, 0.8, 0.0, 0.0);
        state.position = Eigen::Vector3d(3.0, 4.0, 5.0);

        std::vector<Eigen::RowVectorXd> jacobians;

        // Quaternion constraint
        QuaternionConstraintMock qConstraint;
        jacobians.push_back(qConstraint.jacobian(state));

        // Single distance constraint
        DistanceConstraintMock dConstraint{std::sqrt(50.0)};  // Distance matches position magnitude
        jacobians.push_back(dConstraint.jacobian(state));

        double mass = 10.0;
        Eigen::Matrix3d inverseInertia = Eigen::Matrix3d::Identity() * 0.1;

        Eigen::MatrixXd A = assembleConstraintMatrix(jacobians, mass, inverseInertia);
        Eigen::VectorXd b = Eigen::VectorXd::Ones(2);

        SolveResult result = testLLTSolve(A, b);

        std::cout << "Constraint dimension: 2\n";
        std::cout << "State dimension: 7 (3 pos + 4 quat)\n";
        std::cout << "Note: Quaternion acts on DOFs 4-7, Distance acts on DOFs 1-3\n";

        bool condPass = result.conditionNumber < kMaxConditionNumber;
        bool convergePass = result.converged;
        bool nanPass = !result.hasNaN;

        printResult("Condition number", result.conditionNumber, condPass);
        printResult("LLT converged", result.converged ? "true" : "false", convergePass);
        printResult("No NaN in solution", !result.hasNaN ? "true" : "false", nanPass);

        allTestsPassed &= (condPass && convergePass && nanPass);
    }

    // Test 4: Deliberately singular matrix (zero inertia)
    {
        printTestHeader("Test 4: Singular Matrix (Graceful Failure)");

        MockInertialState state;
        state.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        state.position = Eigen::Vector3d(1.0, 0.0, 0.0);

        QuaternionConstraintMock qConstraint;

        std::vector<Eigen::RowVectorXd> jacobians;
        jacobians.push_back(qConstraint.jacobian(state));

        // Create singular case: zero inertia (but nonzero mass to avoid division by zero)
        double mass = 10.0;
        Eigen::Matrix3d inverseInertia = Eigen::Matrix3d::Zero();  // Singular

        Eigen::MatrixXd A = assembleConstraintMatrix(jacobians, mass, inverseInertia);
        Eigen::VectorXd b = Eigen::VectorXd::Ones(1);

        SolveResult result = testLLTSolve(A, b);

        std::cout << "Deliberately singular matrix (zero inertia)\n";

        // We expect this to fail gracefully
        bool failedAsExpected = !result.converged || std::isinf(result.conditionNumber);
        // NaN is acceptable for singular matrices as long as we detect failure
        bool failureDetected = failedAsExpected;

        printResult("Condition number", result.conditionNumber, true);  // Any result is acceptable
        printResult("Failed as expected", failedAsExpected ? "true" : "false", failedAsExpected);
        printResult("Failure detected", failureDetected ? "true" : "false", failureDetected);

        // Only require that we detect the failure, NaN is acceptable in this case
        allTestsPassed &= failureDetected;
    }

    // Test 5: Near-singular matrix (very small mass)
    {
        printTestHeader("Test 5: Near-Singular Matrix (Very Small Mass)");

        MockInertialState state;
        state.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        state.position = Eigen::Vector3d(1.0, 0.0, 0.0);

        QuaternionConstraintMock qConstraint;

        std::vector<Eigen::RowVectorXd> jacobians;
        jacobians.push_back(qConstraint.jacobian(state));

        // Very small mass (1 gram)
        double mass = 1e-3;
        Eigen::Matrix3d inverseInertia = Eigen::Matrix3d::Identity() * 1e-3;

        Eigen::MatrixXd A = assembleConstraintMatrix(jacobians, mass, inverseInertia);
        Eigen::VectorXd b = Eigen::VectorXd::Ones(1);

        SolveResult result = testLLTSolve(A, b);

        std::cout << "Very small mass: " << mass << " kg\n";

        bool condAcceptable = result.conditionNumber < kMaxConditionNumber || result.converged;
        bool noNaN = !result.hasNaN;

        printResult("Condition number", result.conditionNumber, condAcceptable);
        printResult("LLT converged", result.converged ? "true" : "false", result.converged);
        printResult("No NaN in solution", !result.hasNaN ? "true" : "false", noNaN);

        allTestsPassed &= (condAcceptable && noNaN);
    }

    // Summary
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "SUMMARY\n";
    std::cout << std::string(60, '=') << "\n";

    if (allTestsPassed) {
        std::cout << "All tests PASSED\n";
        std::cout << "Constraint matrices are well-conditioned for typical configurations.\n";
        std::cout << "LLT decomposition handles edge cases gracefully.\n";
        return 0;
    } else {
        std::cout << "Some tests FAILED\n";
        std::cout << "Further investigation required.\n";
        return 1;
    }
}
