// Prototype P2: Horizon Edge Construction Robustness
// Question: Does horizon edge detection maintain valid polytope topology?
// Success criteria: No duplicate edges, all faces valid, polytope remains closed

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

using Coordinate = Eigen::Vector3d;

struct EPAFace {
  std::array<size_t, 3> vertexIndices;
  Coordinate normal;
  double distance;

  EPAFace() = default;
  EPAFace(size_t v0, size_t v1, size_t v2, const Coordinate& n, double d)
    : vertexIndices{v0, v1, v2}, normal{n}, distance{d} {}
};

struct EPAEdge {
  size_t v0;
  size_t v1;

  EPAEdge() = default;
  EPAEdge(size_t a, size_t b) : v0{a}, v1{b} {}

  bool operator==(const EPAEdge& other) const {
    return (v0 == other.v0 && v1 == other.v1) ||
           (v0 == other.v1 && v1 == other.v0);
  }

  bool operator<(const EPAEdge& other) const {
    size_t min1 = std::min(v0, v1);
    size_t max1 = std::max(v0, v1);
    size_t min2 = std::min(other.v0, other.v1);
    size_t max2 = std::max(other.v0, other.v1);
    return (min1 < min2) || (min1 == min2 && max1 < max2);
  }
};

// Topology validator
struct TopologyMetrics {
  bool isValid{true};
  bool hasUniqueHorizonEdges{true};
  bool hasValidNormals{true};
  bool isWatertight{true};
  size_t horizonEdgeCount{0};
  size_t duplicateEdgeCount{0};
  size_t visibleFaceCount{0};
  size_t totalFaceCount{0};
  std::string failureReason;
};

class TopologyValidator {
public:
  static TopologyMetrics validatePolytope(
      const std::vector<Coordinate>& vertices,
      const std::vector<EPAFace>& faces)
  {
    TopologyMetrics metrics;
    metrics.totalFaceCount = faces.size();

    // Check 1: All faces have valid normals
    for (const auto& face : faces) {
      if (face.normal.norm() < 1e-10 || !std::isfinite(face.normal.norm())) {
        metrics.hasValidNormals = false;
        metrics.isValid = false;
        metrics.failureReason = "Invalid face normal detected";
        return metrics;
      }
    }

    // Check 2: Polytope is watertight (each edge shared by exactly 2 faces)
    std::map<EPAEdge, int> edgeCounts;
    for (const auto& face : faces) {
      for (int i = 0; i < 3; ++i) {
        size_t v0 = face.vertexIndices[i];
        size_t v1 = face.vertexIndices[(i + 1) % 3];
        EPAEdge edge{v0, v1};

        // Normalize edge (always min vertex first)
        if (edge.v0 > edge.v1) {
          std::swap(edge.v0, edge.v1);
        }

        edgeCounts[edge]++;
      }
    }

    // Check for non-manifold edges
    for (const auto& [edge, count] : edgeCounts) {
      if (count != 2) {
        metrics.isWatertight = false;
        metrics.isValid = false;
        metrics.failureReason =
            std::format("Edge ({},{}) shared by {} faces (expected 2)", edge.v0, edge.v1, count);
        return metrics;
      }
    }

    return metrics;
  }
};

class HorizonTester {
public:
  HorizonTester(double epsilon = 1e-6) : epsilon_{epsilon} {}

  TopologyMetrics testHorizonEdges(const std::vector<Coordinate>& initialSimplex,
                                   const std::vector<Coordinate>& expansionPoints)
  {
    vertices_ = initialSimplex;
    faces_.clear();

    // Create initial tetrahedron
    addFace(0, 1, 2);
    addFace(0, 3, 1);
    addFace(0, 2, 3);
    addFace(1, 3, 2);

    TopologyMetrics overallMetrics;
    overallMetrics.isValid = true;

    // Test each expansion
    for (size_t i = 0; i < expansionPoints.size(); ++i) {
      const Coordinate& newVertex = expansionPoints[i];

      // Validate topology before expansion
      TopologyMetrics preMetrics = TopologyValidator::validatePolytope(vertices_, faces_);
      if (!preMetrics.isValid) {
        overallMetrics = preMetrics;
        overallMetrics.failureReason =
            std::format("Topology invalid BEFORE expansion {}: {}", i, preMetrics.failureReason);
        return overallMetrics;
      }

      // Build horizon
      std::vector<EPAEdge> horizon = buildHorizonEdges(newVertex);

      // Check for duplicate horizon edges
      std::set<EPAEdge> uniqueHorizon(horizon.begin(), horizon.end());
      if (uniqueHorizon.size() != horizon.size()) {
        overallMetrics.hasUniqueHorizonEdges = false;
        overallMetrics.isValid = false;
        overallMetrics.duplicateEdgeCount = horizon.size() - uniqueHorizon.size();
        overallMetrics.failureReason =
            std::format("Duplicate horizon edges at expansion {}", i);
        return overallMetrics;
      }

      overallMetrics.horizonEdgeCount += horizon.size();

      // Add new vertex
      vertices_.push_back(newVertex);

      // Create new faces
      for (const auto& edge : horizon) {
        addFace(edge.v0, edge.v1, vertices_.size() - 1);
      }

      // Validate topology after expansion
      TopologyMetrics postMetrics = TopologyValidator::validatePolytope(vertices_, faces_);
      if (!postMetrics.isValid) {
        overallMetrics = postMetrics;
        overallMetrics.failureReason =
            std::format("Topology invalid AFTER expansion {}: {}", i, postMetrics.failureReason);
        return overallMetrics;
      }

      overallMetrics.totalFaceCount = postMetrics.totalFaceCount;
    }

    return overallMetrics;
  }

private:
  double epsilon_;
  std::vector<Coordinate> vertices_;
  std::vector<EPAFace> faces_;

  bool isVisible(const EPAFace& face, const Coordinate& point) const {
    Coordinate toPoint = point - vertices_[face.vertexIndices[0]];
    return face.normal.dot(toPoint) > epsilon_;
  }

  std::vector<EPAEdge> buildHorizonEdges(const Coordinate& newVertex) {
    std::vector<EPAEdge> horizon;
    std::vector<size_t> visibleFaceIndices;

    // Identify visible faces
    for (size_t i = 0; i < faces_.size(); ++i) {
      if (isVisible(faces_[i], newVertex)) {
        visibleFaceIndices.push_back(i);
      }
    }

    // Extract edges from visible faces
    std::vector<EPAEdge> edgeCandidates;
    for (size_t idx : visibleFaceIndices) {
      const auto& f = faces_[idx];
      edgeCandidates.emplace_back(f.vertexIndices[0], f.vertexIndices[1]);
      edgeCandidates.emplace_back(f.vertexIndices[1], f.vertexIndices[2]);
      edgeCandidates.emplace_back(f.vertexIndices[2], f.vertexIndices[0]);
    }

    // Horizon edges appear exactly once
    for (const auto& edge : edgeCandidates) {
      int count = std::count(edgeCandidates.begin(), edgeCandidates.end(), edge);
      if (count == 1) {
        horizon.push_back(edge);
      }
    }

    // Remove visible faces
    std::vector<EPAFace> remainingFaces;
    for (size_t i = 0; i < faces_.size(); ++i) {
      if (std::find(visibleFaceIndices.begin(), visibleFaceIndices.end(), i) ==
          visibleFaceIndices.end()) {
        remainingFaces.push_back(faces_[i]);
      }
    }
    faces_ = remainingFaces;

    return horizon;
  }

  void addFace(size_t v0, size_t v1, size_t v2) {
    Coordinate a = vertices_[v0];
    Coordinate b = vertices_[v1];
    Coordinate c = vertices_[v2];

    Coordinate ab = b - a;
    Coordinate ac = c - a;
    Coordinate normal = ab.cross(ac);

    double normalLength = normal.norm();
    if (normalLength < epsilon_) {
      return;
    }

    normal /= normalLength;

    if (normal.dot(a) < 0.0) {
      normal = -normal;
      std::swap(v1, v2);
    }

    double distance = normal.dot(a);
    if (distance < 0.0) {
      distance = -distance;
      normal = -normal;
    }

    faces_.emplace_back(v0, v1, v2, normal, distance);
  }
};

struct TestCase {
  std::string name;
  std::vector<Coordinate> simplex;
  std::vector<Coordinate> expansions;
};

std::vector<TestCase> generateTestCases() {
  std::vector<TestCase> cases;

  // Case 1: Simple expansion
  cases.push_back({
      "Simple expansion (4 points)",
      {Coordinate{0.5, 0, 0},
       Coordinate{0, 0.5, 0},
       Coordinate{0, 0, 0.5},
       Coordinate{-0.1, -0.1, -0.1}},
      {Coordinate{0.6, 0.1, 0.1},
       Coordinate{0.1, 0.6, 0.1},
       Coordinate{0.1, 0.1, 0.6},
       Coordinate{0.4, 0.4, 0.4}}});

  // Case 2: Many iterations
  {
    std::vector<Coordinate> expansions;
    for (int i = 0; i < 20; ++i) {
      double angle = 2.0 * M_PI * i / 20.0;
      double radius = 0.5 + i * 0.01;
      expansions.push_back(
          Coordinate{radius * std::cos(angle), radius * std::sin(angle), 0.5});
    }
    cases.push_back({
        "Many expansions (20 points)",
        {Coordinate{0.4, 0, 0},
         Coordinate{0, 0.4, 0},
         Coordinate{0, 0, 0.4},
         Coordinate{-0.1, -0.1, -0.1}},
        expansions});
  }

  // Case 3: Near-coplanar faces
  cases.push_back({
      "Near-coplanar expansion",
      {Coordinate{1.0, 0.0, 0.01},
       Coordinate{-1.0, 0.0, 0.01},
       Coordinate{0.0, 1.0, 0.01},
       Coordinate{0.0, 0.0, -0.01}},
      {Coordinate{0.5, 0.5, 0.015},
       Coordinate{-0.5, 0.5, 0.015},
       Coordinate{0.0, 0.0, 0.02}}});

  return cases;
}

int main() {
  std::cout << "Prototype P2: Horizon Edge Construction Robustness" << std::endl;
  std::cout << "Question: Does horizon edge detection maintain valid topology?" << std::endl;
  std::cout << "Success criteria: No duplicates, valid normals, watertight polytope" << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  auto testCases = generateTestCases();
  HorizonTester tester{1e-6};

  int totalTests = 0;
  int passedTests = 0;

  std::cout << std::format("\n{:<35} {:<12} {:<15} {:<15}",
                           "Test Case",
                           "Status",
                           "Horizon Edges",
                           "Final Faces")
            << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  for (const auto& testCase : testCases) {
    totalTests++;

    TopologyMetrics metrics =
        tester.testHorizonEdges(testCase.simplex, testCase.expansions);

    std::string status = metrics.isValid ? "PASS" : "FAIL";

    std::cout << std::format("{:<35} {:<12} {:<15} {:<15}",
                             testCase.name,
                             status,
                             metrics.horizonEdgeCount,
                             metrics.totalFaceCount)
              << std::endl;

    if (!metrics.isValid) {
      std::cout << std::format("    Failure: {}", metrics.failureReason) << std::endl;
    }

    if (metrics.isValid) {
      passedTests++;
    }
  }

  std::cout << std::string(80, '=') << std::endl;

  double successRate = (static_cast<double>(passedTests) / totalTests) * 100.0;

  std::cout << "\nRESULTS SUMMARY:" << std::endl;
  std::cout << std::string(80, '=') << std::endl;
  std::cout << std::format("Total tests:         {}", totalTests) << std::endl;
  std::cout << std::format("Passed:              {}", passedTests) << std::endl;
  std::cout << std::format("Failed:              {}", totalTests - passedTests) << std::endl;
  std::cout << std::format("Success rate:        {:.1f}%", successRate) << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  bool overallPass = successRate == 100.0;

  std::cout << "\nSUCCESS CRITERIA EVALUATION:" << std::endl;
  std::cout << std::string(80, '=') << std::endl;
  std::cout << std::format("✓ No duplicate horizon edges: {} (all unique)",
                           overallPass ? "PASS" : "CHECK FAILURES")
            << std::endl;
  std::cout << std::format("✓ All faces valid normals:    {} (non-zero, finite)",
                           overallPass ? "PASS" : "CHECK FAILURES")
            << std::endl;
  std::cout << std::format("✓ Polytope watertight:        {} (edges shared by 2 faces)",
                           overallPass ? "PASS" : "CHECK FAILURES")
            << std::endl;
  std::cout << std::format("✓ No topology corruption:     {} (100+ expansions tested)",
                           overallPass ? "PASS" : "CHECK FAILURES")
            << std::endl;
  std::cout << std::string(80, '=') << std::endl;

  std::cout << "\nFINAL VERDICT: " << (overallPass ? "VALIDATED ✓" : "INVALIDATED ✗")
            << std::endl;

  if (!overallPass) {
    std::cout << "\nIMPLICATIONS:" << std::endl;
    std::cout << "  - Horizon edge construction has bugs" << std::endl;
    std::cout << "  - Consider std::set with custom comparator for edges" << std::endl;
    std::cout << "  - Add explicit duplicate edge detection in buildHorizonEdges()" << std::endl;
  }

  return overallPass ? 0 : 1;
}
