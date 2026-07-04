// SPDX-License-Identifier: Apache-2.0
//
// Always-on tests for the native tet-mesh quality module (src/native/mesh). These
// build in the DEFAULT MIT config (no OCCT, no TetGen) — they construct CCTetMesh
// PODs by hand and drive cc_mesh_quality, which is pure geometry and never touches
// the optional AGPL backend. Coverage:
//   * regular-tet golden (dihedral 70.5288, scaledJ 1, aspect 1),
//   * a near-coplanar sliver flagged below a threshold,
//   * an inverted (negative-volume) tet -> scaledJ < 0,
//   * C3D10 == C3D4-of-its-corners (mid-nodes at exact midpoints),
//   * empty mesh -> valid == 0.
//
#include "cybercadkernel/cc_kernel.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace {

// Wrap flat node/element vectors as a CCTetMesh (pointers into caller storage; the
// quality path only reads them, and reports are freed with cc_quality_report_free).
CCTetMesh makeMesh(std::vector<double>& nodes, std::vector<int>& elems, int npe) {
    CCTetMesh m;
    m.nodes = nodes.data();
    m.nodeCount = static_cast<int>(nodes.size() / 3);
    m.elements = elems.data();
    m.elementCount = static_cast<int>(elems.size() / npe);
    m.nodesPerElement = npe;
    m.order = npe;
    return m;
}

bool nearlyEqual(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

}  // namespace

// Positively-oriented regular tetrahedron (edge length 2*sqrt(2)).
CC_TEST(quality_regular_tet) {
    std::vector<double> nodes = {
        1, 1, 1,
        -1, 1, -1,
        1, -1, -1,
        -1, -1, 1,
    };
    std::vector<int> elems = {0, 1, 2, 3};
    CCTetMesh mesh = makeMesh(nodes, elems, 4);

    CCQualityReport rep = cc_mesh_quality(mesh, 0.1);
    CC_CHECK(rep.valid == 1);
    CC_CHECK(nearlyEqual(rep.min_dihedral_angle, 70.5288, 1e-3));
    CC_CHECK(nearlyEqual(rep.max_dihedral_angle, 70.5288, 1e-3));
    CC_CHECK(nearlyEqual(rep.min_scaled_jacobian, 1.0, 1e-9));
    CC_CHECK(nearlyEqual(rep.mean_scaled_jacobian, 1.0, 1e-9));
    CC_CHECK(nearlyEqual(rep.max_aspect_ratio, 1.0, 1e-6));
    CC_CHECK(rep.elements_below_threshold == 0);
    cc_quality_report_free(rep);
}

// A regular tet (element 0) plus a near-coplanar sliver (element 1) -> the sliver
// is flagged below a modest threshold, and it drives the minimum scaled Jacobian.
CC_TEST(quality_sliver_flagged) {
    std::vector<double> nodes = {
        // regular tet corners (positively oriented)
        1, 1, 1,
        -1, 1, -1,
        1, -1, -1,
        -1, -1, 1,
        // near-coplanar sliver
        0, 0, 0,
        1, 0, 0,
        0, 1, 0,
        0.3, 0.3, 0.001,
    };
    std::vector<int> elems = {0, 1, 2, 3, 4, 5, 6, 7};
    CCTetMesh mesh = makeMesh(nodes, elems, 4);

    CCQualityReport rep = cc_mesh_quality(mesh, 0.2);
    CC_CHECK(rep.valid == 1);
    CC_CHECK(rep.elements_below_threshold >= 1);
    bool sliverFlagged = false;
    for (int i = 0; i < rep.elements_below_threshold; ++i) {
        if (rep.flagged_elements[i] == 1) sliverFlagged = true;
    }
    CC_CHECK(sliverFlagged);
    CC_CHECK(rep.min_scaled_jacobian < 0.2);
    CC_CHECK(rep.min_scaled_jacobian > -1e-6);  // sliver, not inverted
    cc_quality_report_free(rep);
}

// A negative-volume (inverted) tet must score a negative scaled Jacobian.
CC_TEST(quality_inverted_tet) {
    std::vector<double> nodes = {
        1, 1, 1,
        1, -1, -1,
        -1, 1, -1,
        -1, -1, 1,
    };
    std::vector<int> elems = {0, 1, 2, 3};
    CCTetMesh mesh = makeMesh(nodes, elems, 4);

    CCQualityReport rep = cc_mesh_quality(mesh, 0.0);
    CC_CHECK(rep.valid == 1);
    CC_CHECK(rep.min_scaled_jacobian < 0.0);
    CC_CHECK(rep.elements_below_threshold == 1);  // scaledJ < 0
    cc_quality_report_free(rep);
}

// A C3D10 element whose mid-nodes sit exactly at edge midpoints scores identically
// to the C3D4 of its 4 corners.
CC_TEST(quality_c3d10_matches_corners) {
    const double c[4][3] = {
        {1, 1, 1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, 1},
    };
    // corners then CalculiX mid-edge order: 5=(1,2) 6=(2,3) 7=(3,1) 8=(1,4) 9=(2,4) 10=(3,4)
    const int edge[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
    std::vector<double> nodes;
    for (auto& p : c) {
        nodes.insert(nodes.end(), {p[0], p[1], p[2]});
    }
    for (auto& e : edge) {
        nodes.push_back(0.5 * (c[e[0]][0] + c[e[1]][0]));
        nodes.push_back(0.5 * (c[e[0]][1] + c[e[1]][1]));
        nodes.push_back(0.5 * (c[e[0]][2] + c[e[1]][2]));
    }
    std::vector<int> elems10 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    CCTetMesh mesh10 = makeMesh(nodes, elems10, 10);

    std::vector<double> corners = {
        1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, 1,
    };
    std::vector<int> elems4 = {0, 1, 2, 3};
    CCTetMesh mesh4 = makeMesh(corners, elems4, 4);

    CCQualityReport r10 = cc_mesh_quality(mesh10, 0.1);
    CCQualityReport r4 = cc_mesh_quality(mesh4, 0.1);
    CC_CHECK(r10.valid == 1 && r4.valid == 1);
    CC_CHECK(nearlyEqual(r10.min_dihedral_angle, r4.min_dihedral_angle, 1e-9));
    CC_CHECK(nearlyEqual(r10.min_scaled_jacobian, r4.min_scaled_jacobian, 1e-9));
    CC_CHECK(nearlyEqual(r10.max_aspect_ratio, r4.max_aspect_ratio, 1e-9));
    // mid-nodes really lie at the midpoints
    for (int i = 0; i < 6; ++i) {
        const int base = (4 + i) * 3;
        CC_CHECK(nearlyEqual(nodes[base + 0], 0.5 * (c[edge[i][0]][0] + c[edge[i][1]][0]), 1e-12));
        CC_CHECK(nearlyEqual(nodes[base + 1], 0.5 * (c[edge[i][0]][1] + c[edge[i][1]][1]), 1e-12));
        CC_CHECK(nearlyEqual(nodes[base + 2], 0.5 * (c[edge[i][0]][2] + c[edge[i][1]][2]), 1e-12));
    }
    cc_quality_report_free(r10);
    cc_quality_report_free(r4);
}

// An empty mesh is invalid (valid == 0), never a crash.
CC_TEST(quality_empty_mesh) {
    CCTetMesh mesh;
    mesh.nodes = nullptr;
    mesh.nodeCount = 0;
    mesh.elements = nullptr;
    mesh.elementCount = 0;
    mesh.nodesPerElement = 0;
    mesh.order = 0;
    CCQualityReport rep = cc_mesh_quality(mesh, 0.1);
    CC_CHECK(rep.valid == 0);
    cc_quality_report_free(rep);
}

CC_RUN_ALL()
