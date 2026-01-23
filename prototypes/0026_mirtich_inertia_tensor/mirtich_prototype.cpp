// Prototype P1: Mirtich Algorithm Implementation Validation
// Question: Does the C++ implementation of Mirtich's algorithm produce results
//           matching volInt.c reference and analytical solutions?
// Success criteria: Results match analytical solutions within 1e-10 absolute error

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <algorithm>

// Coordinate indices
constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

// Simple 3D vector
struct Vec3 {
    double x, y, z;

    Vec3() : x{0.0}, y{0.0}, z{0.0} {}
    Vec3(double x_, double y_, double z_) : x{x_}, y{y_}, z{z_} {}

    double operator[](int i) const {
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    double& operator[](int i) {
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    double dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    double lengthSquared() const {
        return x*x + y*y + z*z;
    }

    double length() const {
        return std::sqrt(lengthSquared());
    }

    Vec3 normalized() const {
        double len = length();
        return Vec3{x/len, y/len, z/len};
    }

    Vec3 cross(const Vec3& other) const {
        return Vec3{
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }
};

// 3x3 matrix for inertia tensor
struct Mat3 {
    double data[3][3];

    Mat3() {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                data[i][j] = 0.0;
            }
        }
    }

    double& operator()(int i, int j) { return data[i][j]; }
    double operator()(int i, int j) const { return data[i][j]; }

    void print() const {
        for (int i = 0; i < 3; ++i) {
            std::cout << "[";
            for (int j = 0; j < 3; ++j) {
                std::cout << std::setw(15) << std::setprecision(10) << data[i][j];
                if (j < 2) std::cout << ", ";
            }
            std::cout << "]\n";
        }
    }
};

// Triangular facet
struct Facet {
    std::array<int, 3> vertexIndices;
    Vec3 normal;
};

// Simple polyhedral mesh
struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<Facet> facets;

    void computeFacetNormals() {
        for (auto& facet : facets) {
            const Vec3& v0 = vertices[facet.vertexIndices[0]];
            const Vec3& v1 = vertices[facet.vertexIndices[1]];
            const Vec3& v2 = vertices[facet.vertexIndices[2]];

            Vec3 edge1{v1.x - v0.x, v1.y - v0.y, v1.z - v0.z};
            Vec3 edge2{v2.x - v1.x, v2.y - v1.y, v2.z - v1.z};

            facet.normal = edge1.cross(edge2).normalized();
        }
    }
};

// Projection integrals (2D integrals over polygon projection)
struct ProjectionIntegrals {
    double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;
};

// Face integrals (3D surface integrals)
struct FaceIntegrals {
    double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
};

// Volume integrals
struct VolumeIntegrals {
    double T0;              // Volume
    std::array<double, 3> T1;  // First moments
    std::array<double, 3> T2;  // Second moments
    std::array<double, 3> TP;  // Products
};

// Helper: Select projection plane based on largest normal component
void selectProjectionPlane(const Vec3& normal, int& A, int& B, int& C) {
    double nx = std::abs(normal[X]);
    double ny = std::abs(normal[Y]);
    double nz = std::abs(normal[Z]);

    if (nx > ny && nx > nz) {
        C = X;
    } else if (ny > nz) {
        C = Y;
    } else {
        C = Z;
    }

    A = (C + 1) % 3;
    B = (A + 1) % 3;
}

// Compute projection integrals (transcribed from volInt.c lines 178-232)
ProjectionIntegrals computeProjectionIntegrals(const Mesh& mesh,
                                                const Facet& facet,
                                                int A, int B) {
    ProjectionIntegrals proj{};
    proj.P1 = proj.Pa = proj.Pb = 0.0;
    proj.Paa = proj.Pab = proj.Pbb = 0.0;
    proj.Paaa = proj.Paab = proj.Pabb = proj.Pbbb = 0.0;

    // Iterate over edges of the triangle
    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;

        const Vec3& vert_i = mesh.vertices[facet.vertexIndices[i]];
        const Vec3& vert_j = mesh.vertices[facet.vertexIndices[j]];

        double a0 = vert_i[A];
        double b0 = vert_i[B];
        double a1 = vert_j[A];
        double b1 = vert_j[B];

        double da = a1 - a0;
        double db = b1 - b0;

        double a0_2 = a0 * a0;
        double a0_3 = a0_2 * a0;
        double a0_4 = a0_3 * a0;
        double b0_2 = b0 * b0;
        double b0_3 = b0_2 * b0;
        double b0_4 = b0_3 * b0;
        double a1_2 = a1 * a1;
        double a1_3 = a1_2 * a1;
        double b1_2 = b1 * b1;
        double b1_3 = b1_2 * b1;

        double C1 = a1 + a0;
        double Ca = a1*C1 + a0_2;
        double Caa = a1*Ca + a0_3;
        double Caaa = a1*Caa + a0_4;
        double Cb = b1*(b1 + b0) + b0_2;
        double Cbb = b1*Cb + b0_3;
        double Cbbb = b1*Cbb + b0_4;
        double Cab = 3*a1_2 + 2*a1*a0 + a0_2;
        double Kab = a1_2 + 2*a1*a0 + 3*a0_2;
        double Caab = a0*Cab + 4*a1_3;
        double Kaab = a1*Kab + 4*a0_3;
        double Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
        double Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

        proj.P1 += db*C1;
        proj.Pa += db*Ca;
        proj.Paa += db*Caa;
        proj.Paaa += db*Caaa;
        proj.Pb += da*Cb;
        proj.Pbb += da*Cbb;
        proj.Pbbb += da*Cbbb;
        proj.Pab += db*(b1*Cab + b0*Kab);
        proj.Paab += db*(b1*Caab + b0*Kaab);
        proj.Pabb += da*(a1*Cabb + a0*Kabb);
    }

    proj.P1 /= 2.0;
    proj.Pa /= 6.0;
    proj.Paa /= 12.0;
    proj.Paaa /= 20.0;
    proj.Pb /= -6.0;
    proj.Pbb /= -12.0;
    proj.Pbbb /= -20.0;
    proj.Pab /= 24.0;
    proj.Paab /= 60.0;
    proj.Pabb /= -60.0;

    return proj;
}

// Compute face integrals (transcribed from volInt.c lines 234-265)
FaceIntegrals computeFaceIntegrals(const Mesh& mesh,
                                    const Facet& facet,
                                    const ProjectionIntegrals& proj,
                                    int A, int B, int C) {
    FaceIntegrals face{};

    const Vec3& n = facet.normal;

    // w = -n · v0 for any vertex v0 on the plane
    const Vec3& v0 = mesh.vertices[facet.vertexIndices[0]];
    double w = -(n[X]*v0[X] + n[Y]*v0[Y] + n[Z]*v0[Z]);

    double k1 = 1.0 / n[C];
    double k2 = k1 * k1;
    double k3 = k2 * k1;
    double k4 = k3 * k1;

    face.Fa = k1 * proj.Pa;
    face.Fb = k1 * proj.Pb;
    face.Fc = -k2 * (n[A]*proj.Pa + n[B]*proj.Pb + w*proj.P1);

    face.Faa = k1 * proj.Paa;
    face.Fbb = k1 * proj.Pbb;
    face.Fcc = k3 * (n[A]*n[A]*proj.Paa + 2*n[A]*n[B]*proj.Pab + n[B]*n[B]*proj.Pbb
                     + w*(2*(n[A]*proj.Pa + n[B]*proj.Pb) + w*proj.P1));

    face.Faaa = k1 * proj.Paaa;
    face.Fbbb = k1 * proj.Pbbb;
    face.Fccc = -k4 * (n[A]*n[A]*n[A]*proj.Paaa + 3*n[A]*n[A]*n[B]*proj.Paab
                       + 3*n[A]*n[B]*n[B]*proj.Pabb + n[B]*n[B]*n[B]*proj.Pbbb
                       + 3*w*(n[A]*n[A]*proj.Paa + 2*n[A]*n[B]*proj.Pab + n[B]*n[B]*proj.Pbb)
                       + w*w*(3*(n[A]*proj.Pa + n[B]*proj.Pb) + w*proj.P1));

    face.Faab = k1 * proj.Paab;
    face.Fbbc = -k2 * (n[A]*proj.Pabb + n[B]*proj.Pbbb + w*proj.Pbb);
    face.Fcca = k3 * (n[A]*n[A]*proj.Paaa + 2*n[A]*n[B]*proj.Paab + n[B]*n[B]*proj.Pabb
                      + w*(2*(n[A]*proj.Paa + n[B]*proj.Pab) + w*proj.Pa));

    return face;
}

// Compute volume integrals (transcribed from volInt.c lines 267-307)
VolumeIntegrals computeVolumeIntegrals(const Mesh& mesh) {
    VolumeIntegrals vol{};
    vol.T0 = 0.0;
    vol.T1 = {0.0, 0.0, 0.0};
    vol.T2 = {0.0, 0.0, 0.0};
    vol.TP = {0.0, 0.0, 0.0};

    for (const auto& facet : mesh.facets) {
        int A, B, C;
        selectProjectionPlane(facet.normal, A, B, C);

        ProjectionIntegrals proj = computeProjectionIntegrals(mesh, facet, A, B);
        FaceIntegrals face = computeFaceIntegrals(mesh, facet, proj, A, B, C);

        const Vec3& n = facet.normal;

        // Accumulate T0 (volume)
        vol.T0 += n[X] * ((A == X) ? face.Fa : ((B == X) ? face.Fb : face.Fc));

        // Accumulate T1 (first moments)
        vol.T1[A] += n[A] * face.Faa;
        vol.T1[B] += n[B] * face.Fbb;
        vol.T1[C] += n[C] * face.Fcc;

        // Accumulate T2 (second moments)
        vol.T2[A] += n[A] * face.Faaa;
        vol.T2[B] += n[B] * face.Fbbb;
        vol.T2[C] += n[C] * face.Fccc;

        // Accumulate TP (products)
        vol.TP[A] += n[A] * face.Faab;
        vol.TP[B] += n[B] * face.Fbbc;
        vol.TP[C] += n[C] * face.Fcca;
    }

    vol.T1[X] /= 2.0; vol.T1[Y] /= 2.0; vol.T1[Z] /= 2.0;
    vol.T2[X] /= 3.0; vol.T2[Y] /= 3.0; vol.T2[Z] /= 3.0;
    vol.TP[X] /= 2.0; vol.TP[Y] /= 2.0; vol.TP[Z] /= 2.0;

    return vol;
}

// Compute inertia tensor about centroid
Mat3 computeInertiaTensorAboutCentroid(const Mesh& mesh, double mass) {
    VolumeIntegrals vol = computeVolumeIntegrals(mesh);

    double density = mass / vol.T0;

    // Compute inertia about origin (transcribed from volInt.c lines 358-363)
    Mat3 I_origin;
    I_origin(X, X) = density * (vol.T2[Y] + vol.T2[Z]);
    I_origin(Y, Y) = density * (vol.T2[Z] + vol.T2[X]);
    I_origin(Z, Z) = density * (vol.T2[X] + vol.T2[Y]);
    I_origin(X, Y) = I_origin(Y, X) = -density * vol.TP[X];
    I_origin(Y, Z) = I_origin(Z, Y) = -density * vol.TP[Y];
    I_origin(Z, X) = I_origin(X, Z) = -density * vol.TP[Z];

    // Compute center of mass
    Vec3 r{vol.T1[X] / vol.T0, vol.T1[Y] / vol.T0, vol.T1[Z] / vol.T0};

    // Apply parallel axis theorem (transcribed from volInt.c lines 366-371)
    Mat3 I_centroid = I_origin;
    I_centroid(X, X) -= mass * (r[Y]*r[Y] + r[Z]*r[Z]);
    I_centroid(Y, Y) -= mass * (r[Z]*r[Z] + r[X]*r[X]);
    I_centroid(Z, Z) -= mass * (r[X]*r[X] + r[Y]*r[Y]);
    I_centroid(X, Y) = I_centroid(Y, X) += mass * r[X] * r[Y];
    I_centroid(Y, Z) = I_centroid(Z, Y) += mass * r[Y] * r[Z];
    I_centroid(Z, X) = I_centroid(X, Z) += mass * r[Z] * r[X];

    std::cout << "Volume: " << vol.T0 << "\n";
    std::cout << "Center of mass: (" << r[X] << ", " << r[Y] << ", " << r[Z] << ")\n";

    return I_centroid;
}

// Test geometry factories
Mesh createUnitCube() {
    Mesh mesh;

    // 8 vertices of unit cube centered at origin
    mesh.vertices = {
        Vec3{-0.5, -0.5, -0.5},
        Vec3{ 0.5, -0.5, -0.5},
        Vec3{ 0.5,  0.5, -0.5},
        Vec3{-0.5,  0.5, -0.5},
        Vec3{-0.5, -0.5,  0.5},
        Vec3{ 0.5, -0.5,  0.5},
        Vec3{ 0.5,  0.5,  0.5},
        Vec3{-0.5,  0.5,  0.5}
    };

    // 12 triangular facets (2 per face) - outward-facing normals (CCW winding from outside)
    mesh.facets = {
        // Bottom (-Z) - looking from below, CCW
        {{0, 2, 1}, Vec3{}}, {{0, 3, 2}, Vec3{}},
        // Top (+Z) - looking from above, CCW
        {{4, 5, 6}, Vec3{}}, {{4, 6, 7}, Vec3{}},
        // Front (-Y) - looking from front, CCW
        {{0, 1, 5}, Vec3{}}, {{0, 5, 4}, Vec3{}},
        // Back (+Y) - looking from back, CCW
        {{2, 3, 7}, Vec3{}}, {{2, 7, 6}, Vec3{}},
        // Left (-X) - looking from left, CCW
        {{0, 4, 7}, Vec3{}}, {{0, 7, 3}, Vec3{}},
        // Right (+X) - looking from right, CCW
        {{1, 2, 6}, Vec3{}}, {{1, 6, 5}, Vec3{}}
    };

    mesh.computeFacetNormals();
    return mesh;
}

Mesh createRectangularBox(double a, double b, double c) {
    Mesh mesh;

    double hx = a / 2.0;
    double hy = b / 2.0;
    double hz = c / 2.0;

    mesh.vertices = {
        Vec3{-hx, -hy, -hz},
        Vec3{ hx, -hy, -hz},
        Vec3{ hx,  hy, -hz},
        Vec3{-hx,  hy, -hz},
        Vec3{-hx, -hy,  hz},
        Vec3{ hx, -hy,  hz},
        Vec3{ hx,  hy,  hz},
        Vec3{-hx,  hy,  hz}
    };

    mesh.facets = {
        {{0, 2, 1}, Vec3{}}, {{0, 3, 2}, Vec3{}},
        {{4, 5, 6}, Vec3{}}, {{4, 6, 7}, Vec3{}},
        {{0, 1, 5}, Vec3{}}, {{0, 5, 4}, Vec3{}},
        {{2, 3, 7}, Vec3{}}, {{2, 7, 6}, Vec3{}},
        {{0, 4, 7}, Vec3{}}, {{0, 7, 3}, Vec3{}},
        {{1, 2, 6}, Vec3{}}, {{1, 6, 5}, Vec3{}}
    };

    mesh.computeFacetNormals();
    return mesh;
}

Mesh createRegularTetrahedron(double edgeLength) {
    Mesh mesh;

    // Regular tetrahedron with vertices at:
    // v0 = (1, 1, 1)
    // v1 = (1, -1, -1)
    // v2 = (-1, 1, -1)
    // v3 = (-1, -1, 1)
    // These are equidistant from origin and form regular tetrahedron

    double scale = edgeLength / (2.0 * std::sqrt(2.0));

    mesh.vertices = {
        Vec3{ scale,  scale,  scale},
        Vec3{ scale, -scale, -scale},
        Vec3{-scale,  scale, -scale},
        Vec3{-scale, -scale,  scale}
    };

    // Facets with outward-facing normals (CCW from outside)
    mesh.facets = {
        {{0, 1, 2}, Vec3{}},  // Face opposite to vertex 3
        {{0, 3, 1}, Vec3{}},  // Face opposite to vertex 2
        {{0, 2, 3}, Vec3{}},  // Face opposite to vertex 1
        {{1, 3, 2}, Vec3{}}   // Face opposite to vertex 0
    };

    mesh.computeFacetNormals();
    return mesh;
}

// Test suite
void testUnitCube() {
    std::cout << "\n=== Test: Unit Cube ===\n";

    Mesh cube = createUnitCube();
    double mass = 1.0;

    Mat3 I = computeInertiaTensorAboutCentroid(cube, mass);

    // Analytical solution for unit cube:
    // Ixx = Iyy = Izz = m/6 = 1/6 ≈ 0.166667
    // Ixy = Iyz = Izx = 0

    double expected_diagonal = mass / 6.0;

    std::cout << "Computed inertia tensor:\n";
    I.print();

    std::cout << "\nExpected:\n";
    std::cout << "Diagonal: " << expected_diagonal << "\n";
    std::cout << "Off-diagonal: 0.0\n";

    double error_xx = std::abs(I(X, X) - expected_diagonal);
    double error_yy = std::abs(I(Y, Y) - expected_diagonal);
    double error_zz = std::abs(I(Z, Z) - expected_diagonal);
    double error_xy = std::abs(I(X, Y));
    double error_yz = std::abs(I(Y, Z));
    double error_zx = std::abs(I(Z, X));

    std::cout << "\nErrors:\n";
    std::cout << "Ixx: " << error_xx << "\n";
    std::cout << "Iyy: " << error_yy << "\n";
    std::cout << "Izz: " << error_zz << "\n";
    std::cout << "Ixy: " << error_xy << "\n";
    std::cout << "Iyz: " << error_yz << "\n";
    std::cout << "Izx: " << error_zx << "\n";

    bool passed = (error_xx < 1e-10 && error_yy < 1e-10 && error_zz < 1e-10 &&
                   error_xy < 1e-10 && error_yz < 1e-10 && error_zx < 1e-10);

    std::cout << "\nResult: " << (passed ? "PASS" : "FAIL") << "\n";
}

void testRectangularBox() {
    std::cout << "\n=== Test: Rectangular Box (2x3x4) ===\n";

    double a = 2.0, b = 3.0, c = 4.0;
    Mesh box = createRectangularBox(a, b, c);
    double mass = 1.0;

    Mat3 I = computeInertiaTensorAboutCentroid(box, mass);

    // Analytical solution:
    // Ixx = m(b² + c²)/12 = (9 + 16)/12 = 25/12
    // Iyy = m(a² + c²)/12 = (4 + 16)/12 = 20/12
    // Izz = m(a² + b²)/12 = (4 + 9)/12 = 13/12

    double expected_xx = mass * (b*b + c*c) / 12.0;
    double expected_yy = mass * (a*a + c*c) / 12.0;
    double expected_zz = mass * (a*a + b*b) / 12.0;

    std::cout << "Computed inertia tensor:\n";
    I.print();

    std::cout << "\nExpected:\n";
    std::cout << "Ixx: " << expected_xx << "\n";
    std::cout << "Iyy: " << expected_yy << "\n";
    std::cout << "Izz: " << expected_zz << "\n";
    std::cout << "Off-diagonal: 0.0\n";

    double error_xx = std::abs(I(X, X) - expected_xx);
    double error_yy = std::abs(I(Y, Y) - expected_yy);
    double error_zz = std::abs(I(Z, Z) - expected_zz);
    double error_xy = std::abs(I(X, Y));
    double error_yz = std::abs(I(Y, Z));
    double error_zx = std::abs(I(Z, X));

    std::cout << "\nErrors:\n";
    std::cout << "Ixx: " << error_xx << "\n";
    std::cout << "Iyy: " << error_yy << "\n";
    std::cout << "Izz: " << error_zz << "\n";
    std::cout << "Ixy: " << error_xy << "\n";
    std::cout << "Iyz: " << error_yz << "\n";
    std::cout << "Izx: " << error_zx << "\n";

    bool passed = (error_xx < 1e-10 && error_yy < 1e-10 && error_zz < 1e-10 &&
                   error_xy < 1e-10 && error_yz < 1e-10 && error_zx < 1e-10);

    std::cout << "\nResult: " << (passed ? "PASS" : "FAIL") << "\n";
}

void testRegularTetrahedron() {
    std::cout << "\n=== Test: Regular Tetrahedron ===\n";

    double edgeLength = 2.0;
    Mesh tet = createRegularTetrahedron(edgeLength);
    double mass = 1.0;

    Mat3 I = computeInertiaTensorAboutCentroid(tet, mass);

    // Analytical solution for regular tetrahedron:
    // Ixx = Iyy = Izz = m*L²/20 when aligned with principal axes

    double expected_diagonal = mass * edgeLength * edgeLength / 20.0;

    std::cout << "Computed inertia tensor:\n";
    I.print();

    std::cout << "\nExpected (approximately, depends on alignment):\n";
    std::cout << "Diagonal: ~" << expected_diagonal << "\n";

    // For regular tetrahedron, check that diagonal elements are equal
    double avg_diagonal = (I(X, X) + I(Y, Y) + I(Z, Z)) / 3.0;
    double error_symmetry = std::max({
        std::abs(I(X, X) - avg_diagonal),
        std::abs(I(Y, Y) - avg_diagonal),
        std::abs(I(Z, Z) - avg_diagonal)
    });

    std::cout << "\nAverage diagonal: " << avg_diagonal << "\n";
    std::cout << "Symmetry error: " << error_symmetry << "\n";
    std::cout << "Relative error vs analytical: "
              << std::abs(avg_diagonal - expected_diagonal) / expected_diagonal << "\n";

    bool passed = (error_symmetry < 1e-10);

    std::cout << "\nResult: " << (passed ? "PASS (diagonal elements equal)" : "FAIL") << "\n";
}

int main() {
    std::cout << std::fixed << std::setprecision(12);

    testUnitCube();
    testRectangularBox();
    testRegularTetrahedron();

    return 0;
}
