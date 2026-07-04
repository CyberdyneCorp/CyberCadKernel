// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Gated tests for the TetGen-backed volume mesher (built only under
// CYBERCAD_HAS_TETGEN). They drive cc_tet_mesh_surface with a hardcoded unit cube
// (8 points, 12 triangular facets) — OCCT-free — and assert the invariants the
// CalculiX++ FE contract depends on:
//   * C3D4 and C3D10 output, non-empty, every tet positive signed volume,
//   * C3D10 mid-nodes exactly at edge midpoints in CalculiX shape10tet order,
//   * volume conservation (sum of |tet volumes| == enclosed cube volume),
//   * watertight manifold (every triangular face shared 1 or 2 times; the boundary
//     faces form a closed surface — every boundary edge shared exactly twice),
//   * quality gate: all elements have a positive scaled Jacobian.
//
#include "cybercadkernel/cc_kernel.h"

#include "harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace {

// Unit cube [0,1]^3: 8 vertices, 12 outward triangles (2 per face).
std::vector<double> cubeVerts() {
    return {
        0, 0, 0,  1, 0, 0,  1, 1, 0,  0, 1, 0,  // z=0
        0, 0, 1,  1, 0, 1,  1, 1, 1,  0, 1, 1,  // z=1
    };
}
std::vector<int> cubeTris() {
    return {
        0, 1, 2, 0, 2, 3,  // bottom z=0
        4, 6, 5, 4, 7, 6,  // top z=1
        0, 1, 5, 0, 5, 4,  // front y=0
        3, 2, 6, 3, 6, 7,  // back y=1
        0, 4, 7, 0, 7, 3,  // left x=0
        1, 2, 6, 1, 6, 5,  // right x=1
    };
}

double signedVol6(const double* n, int a, int b, int c, int d) {
    const double ax = n[b * 3 + 0] - n[a * 3 + 0], ay = n[b * 3 + 1] - n[a * 3 + 1],
                 az = n[b * 3 + 2] - n[a * 3 + 2];
    const double bx = n[c * 3 + 0] - n[a * 3 + 0], by = n[c * 3 + 1] - n[a * 3 + 1],
                 bz = n[c * 3 + 2] - n[a * 3 + 2];
    const double cx = n[d * 3 + 0] - n[a * 3 + 0], cy = n[d * 3 + 1] - n[a * 3 + 1],
                 cz = n[d * 3 + 2] - n[a * 3 + 2];
    return ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
}

std::array<int, 3> sortedTri(int a, int b, int c) {
    std::array<int, 3> t = {a, b, c};
    std::sort(t.begin(), t.end());
    return t;
}
std::array<int, 2> sortedEdge(int a, int b) {
    return a < b ? std::array<int, 2>{a, b} : std::array<int, 2>{b, a};
}

CCVolumeMeshOptions opts(int order) {
    CCVolumeMeshOptions o;
    o.order = order;
    o.target_element_size = 0.6;  // volume constraint -> a refined interior
    o.grading = 1.4;              // radius-edge quality ratio q
    o.min_scaled_jacobian = 0.1;
    return o;
}

}  // namespace

// C3D4: non-empty, every element positive signed volume.
CC_TEST(tet_c3d4_positive_volume) {
    const auto v = cubeVerts();
    const auto t = cubeTris();
    CCTetMesh m = cc_tet_mesh_surface(v.data(), 8, t.data(), 12, opts(4));
    CC_CHECK(m.elementCount > 0);
    CC_CHECK(m.nodesPerElement == 4);
    CC_CHECK(m.order == 4);
    for (int e = 0; e < m.elementCount; ++e) {
        const int* row = &m.elements[e * 4];
        CC_CHECK(signedVol6(m.nodes, row[0], row[1], row[2], row[3]) > 0.0);
    }
    cc_tet_mesh_free(m);
}

// C3D10: 10 nodes/element, mid-nodes at exact midpoints, all positive volume.
CC_TEST(tet_c3d10_midpoints) {
    const auto v = cubeVerts();
    const auto t = cubeTris();
    CCTetMesh m = cc_tet_mesh_surface(v.data(), 8, t.data(), 12, opts(10));
    CC_CHECK(m.elementCount > 0);
    CC_CHECK(m.nodesPerElement == 10);
    CC_CHECK(m.order == 10);
    // CalculiX shape10tet mid-edge order: 5=(1,2) 6=(2,3) 7=(3,1) 8=(1,4) 9=(2,4) 10=(3,4)
    const int me[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
    for (int e = 0; e < m.elementCount; ++e) {
        const int* row = &m.elements[e * 10];
        CC_CHECK(signedVol6(m.nodes, row[0], row[1], row[2], row[3]) > 0.0);
        for (int k = 0; k < 6; ++k) {
            const int ci = row[me[k][0]], cj = row[me[k][1]], mid = row[4 + k];
            for (int d = 0; d < 3; ++d) {
                const double expect = 0.5 * (m.nodes[ci * 3 + d] + m.nodes[cj * 3 + d]);
                CC_CHECK(std::fabs(m.nodes[mid * 3 + d] - expect) <= 1e-12);
            }
        }
    }
    cc_tet_mesh_free(m);
}

// Volume conservation: the tets exactly fill the unit cube (volume 1).
CC_TEST(tet_volume_conservation) {
    const auto v = cubeVerts();
    const auto t = cubeTris();
    CCTetMesh m = cc_tet_mesh_surface(v.data(), 8, t.data(), 12, opts(4));
    CC_CHECK(m.elementCount > 0);
    double total = 0.0;
    for (int e = 0; e < m.elementCount; ++e) {
        const int* row = &m.elements[e * 4];
        total += std::fabs(signedVol6(m.nodes, row[0], row[1], row[2], row[3])) / 6.0;
    }
    CC_CHECK(std::fabs(total - 1.0) <= 1e-9);
    cc_tet_mesh_free(m);
}

// Watertight manifold: every triangular face is shared 1 or 2 times, and the
// boundary (count==1) faces form a closed surface (every boundary edge shared 2x).
CC_TEST(tet_watertight_manifold) {
    const auto v = cubeVerts();
    const auto t = cubeTris();
    CCTetMesh m = cc_tet_mesh_surface(v.data(), 8, t.data(), 12, opts(4));
    CC_CHECK(m.elementCount > 0);

    std::map<std::array<int, 3>, int> faceCount;
    const int f[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
    for (int e = 0; e < m.elementCount; ++e) {
        const int* row = &m.elements[e * 4];
        for (auto& fi : f) {
            faceCount[sortedTri(row[fi[0]], row[fi[1]], row[fi[2]])]++;
        }
    }
    std::map<std::array<int, 2>, int> boundaryEdge;
    int boundaryFaces = 0;
    bool allValidCounts = true;
    for (const auto& kv : faceCount) {
        if (kv.second != 1 && kv.second != 2) allValidCounts = false;
        if (kv.second == 1) {
            ++boundaryFaces;
            const auto& tri = kv.first;
            boundaryEdge[sortedEdge(tri[0], tri[1])]++;
            boundaryEdge[sortedEdge(tri[1], tri[2])]++;
            boundaryEdge[sortedEdge(tri[0], tri[2])]++;
        }
    }
    CC_CHECK(allValidCounts);
    CC_CHECK(boundaryFaces > 0);
    bool closed = true;
    for (const auto& kv : boundaryEdge) {
        if (kv.second != 2) closed = false;
    }
    CC_CHECK(closed);  // boundary is a closed manifold surface
    cc_tet_mesh_free(m);
}

// Quality gate: the cube meshes with a strictly positive scaled Jacobian.
CC_TEST(tet_quality_positive_jacobian) {
    const auto v = cubeVerts();
    const auto t = cubeTris();
    CCTetMesh m = cc_tet_mesh_surface(v.data(), 8, t.data(), 12, opts(10));
    CC_CHECK(m.elementCount > 0);
    CCQualityReport rep = cc_mesh_quality(m, 0.1);
    CC_CHECK(rep.valid == 1);
    CC_CHECK(rep.min_scaled_jacobian > 0.0);  // no inverted elements
    CC_CHECK(rep.min_dihedral_angle > 0.0 && rep.max_dihedral_angle < 180.0);
    cc_quality_report_free(rep);
    cc_tet_mesh_free(m);
}

// Sizing control: target_element_size must actually refine the interior. A smaller
// target size must yield strictly more elements, and a fine size must produce far
// more tets than an unconstrained fill of the 8-vertex cube. This is the regression
// guard for the bug where an appended 'Y' switch silently froze the input boundary
// and made TetGen ignore the max-volume cap (cube stayed ~6 tets at any size).
CC_TEST(tet_size_refinement) {
    const auto v = cubeVerts();
    const auto t = cubeTris();

    CCVolumeMeshOptions coarse = opts(4);
    coarse.target_element_size = 0.6;
    CCVolumeMeshOptions fine = opts(4);
    fine.target_element_size = 0.15;

    CCTetMesh mc = cc_tet_mesh_surface(v.data(), 8, t.data(), 12, coarse);
    CCTetMesh mf = cc_tet_mesh_surface(v.data(), 8, t.data(), 12, fine);
    CC_CHECK(mc.elementCount > 0);
    CC_CHECK(mf.elementCount > 0);

    // A finer target size must produce strictly more elements.
    CC_CHECK(mf.elementCount > mc.elementCount);
    // A fine size on a unit cube must refine well beyond the ~6-tet trivial fill.
    CC_CHECK(mf.elementCount >= 50);

    // Tie element count to the requested physical size: TetGen's 'a<vol>' cap for
    // target size h is the regular-tet volume h^3/(6*sqrt(2)). The cap is a soft
    // target (a few quality/boundary tets exceed it), so we assert the MEAN tet
    // volume respects it — total volume is 1, so this bounds the count from below
    // and fails outright if the sizing knob is ignored (mean would be ~1/6).
    const double h = fine.target_element_size;
    const double maxvol = (h * h * h) / (6.0 * std::sqrt(2.0));
    double total = 0.0;
    for (int e = 0; e < mf.elementCount; ++e) {
        const int* row = &mf.elements[e * 4];
        total += std::fabs(signedVol6(mf.nodes, row[0], row[1], row[2], row[3])) / 6.0;
    }
    CC_CHECK(total / mf.elementCount <= maxvol);
    // Refinement must not break conservation: the tets still fill the unit cube.
    CC_CHECK(std::fabs(total - 1.0) <= 1e-9);

    cc_tet_mesh_free(mc);
    cc_tet_mesh_free(mf);
}

CC_RUN_ALL()
