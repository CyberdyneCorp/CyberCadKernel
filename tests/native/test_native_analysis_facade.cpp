// SPDX-License-Identifier: Apache-2.0
//
// test_native_analysis_facade.cpp — GATE A (host, no OCCT) for the ADDITIVE
// cc_measure_distance / cc_measure_angle / cc_surface_curvature /
// cc_edge_curvature facade over the NativeEngine (MOAT M-GS, GS3 + GS4).
//
// Drives the real C ABI on a natively-built B-rep (a 1×1×2 box) and checks the
// marshaled results against closed forms, and the HONEST-DECLINE contract
// (return 0 + cc_last_error, never a fabricated number). OCCT is not linked.
//
// Built under CYBERCAD_HAS_NUMSCI (the distance cell rides the numerics
// minimizer, compiled only in that config).
//
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

namespace {
constexpr double kPi = 3.14159265358979323846;

struct EngineGuard { ~EngineGuard() { cc_set_engine(0); } };

// A 1×1×2 native box (unit-square profile extruded by depth 2). Corners are the
// 8 points {0,1}×{0,1}×{0,2}.
CCShapeId makeBox() {
    const double profile[] = {0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0};
    return cc_solid_extrude(profile, 4, 2.0);
}
}  // namespace

// cc_surface_curvature on a planar face → K=H=k1=k2=0.
CC_TEST(facade_surface_curvature_plane_zero) {
    EngineGuard g; cc_set_engine(1);
    const CCShapeId box = makeBox();
    CC_CHECK(box != 0);
    double out4[4] = {9, 9, 9, 9};
    const int ok = cc_surface_curvature(box, 1, 0.5, 0.5, out4);
    CC_CHECK(ok == 1);
    for (double x : out4) CC_CHECK(std::fabs(x) < 1e-9);
    cc_shape_release(box);
}

// cc_edge_curvature on a straight edge → κ=0.
CC_TEST(facade_edge_curvature_line_zero) {
    EngineGuard g; cc_set_engine(1);
    const CCShapeId box = makeBox();
    double k = 9.0;
    const int ok = cc_edge_curvature(box, 1, 0.3, &k);
    CC_CHECK(ok == 1);
    CC_CHECK(std::fabs(k) < 1e-9);
    cc_shape_release(box);
}

// cc_measure_angle: a planar face with itself → 0; and a box has a face pair at 90°.
CC_TEST(facade_measure_angle_planes) {
    EngineGuard g; cc_set_engine(1);
    const CCShapeId box = makeBox();
    double self = 9.0;
    CC_CHECK(cc_measure_angle(box, 2, 1, 2, 1, &self) == 1);
    CC_CHECK(std::fabs(self) < 1e-9);  // acos(n·n)=0

    // Every distinct box-face pair is at 0, 90 or 180 degrees; at least one is 90.
    bool sawRight = false;
    for (int j = 2; j <= 6; ++j) {
        double th = -1.0;
        if (cc_measure_angle(box, 2, 1, 2, j, &th) == 1) {
            const bool valid = std::fabs(th) < 1e-6 || std::fabs(th - kPi / 2) < 1e-6 ||
                               std::fabs(th - kPi) < 1e-6;
            CC_CHECK(valid);
            if (std::fabs(th - kPi / 2) < 1e-6) sawRight = true;
        }
    }
    CC_CHECK(sawRight);
    cc_shape_release(box);
}

// cc_measure_distance between vertices: the 8 self-witnesses recover the known 8
// corners, and the measured min (adjacent) / max (space diagonal) match √1 and √6.
CC_TEST(facade_measure_distance_vertices) {
    EngineGuard g; cc_set_engine(1);
    const CCShapeId box = makeBox();

    int* ids = nullptr;
    const int nv = cc_subshape_ids(box, 0, &ids);
    CC_CHECK(nv == 8);

    // Self-distance is 0 and its witness is the vertex coordinate; collect them.
    std::vector<std::array<double, 3>> coords;
    for (int i = 0; i < nv; ++i) {
        double o[7] = {0};
        CC_CHECK(cc_measure_distance(box, 0, ids[i], 0, ids[i], o) == 1);
        CC_CHECK(std::fabs(o[0]) < 1e-9);
        CC_CHECK(std::fabs(o[1] - o[4]) < 1e-12 && std::fabs(o[2] - o[5]) < 1e-12 &&
                 std::fabs(o[3] - o[6]) < 1e-12);
        coords.push_back({o[1], o[2], o[3]});
    }
    // Each recovered corner is a member of {0,1}×{0,1}×{0,2}.
    for (const auto& c : coords) {
        const bool okx = std::fabs(c[0]) < 1e-9 || std::fabs(c[0] - 1) < 1e-9;
        const bool oky = std::fabs(c[1]) < 1e-9 || std::fabs(c[1] - 1) < 1e-9;
        const bool okz = std::fabs(c[2]) < 1e-9 || std::fabs(c[2] - 2) < 1e-9;
        CC_CHECK(okx && oky && okz);
    }

    // Min non-zero and max pairwise measured distances = adjacent edge (1) / space
    // diagonal √(1+1+4)=√6.
    double dmin = 1e30, dmax = 0.0;
    for (int i = 0; i < nv; ++i)
        for (int j = 0; j < nv; ++j)
            if (i != j) {
                double o[7] = {0};
                CC_CHECK(cc_measure_distance(box, 0, ids[i], 0, ids[j], o) == 1);
                dmin = std::min(dmin, o[0]);
                dmax = std::max(dmax, o[0]);
            }
    CC_CHECK(std::fabs(dmin - 1.0) < 1e-9);
    CC_CHECK(std::fabs(dmax - std::sqrt(6.0)) < 1e-9);

    cc_ints_free(ids);
    cc_shape_release(box);
}

// HONEST DECLINE contract: a non-line/plane angle and an out-of-range id return 0
// with cc_last_error set — never a fabricated number.
CC_TEST(facade_honest_declines) {
    EngineGuard g; cc_set_engine(1);
    const CCShapeId box = makeBox();

    double th = 42.0;
    CC_CHECK(cc_measure_angle(box, 0, 1, 0, 2, &th) == 0);  // vertex·vertex → decline
    CC_CHECK(std::strcmp(cc_last_error(), "") != 0);

    double out4[4] = {0};
    CC_CHECK(cc_surface_curvature(box, 999, 0.5, 0.5, out4) == 0);  // bad face id
    CC_CHECK(std::strcmp(cc_last_error(), "") != 0);

    double k = 0.0;
    CC_CHECK(cc_edge_curvature(box, 999, 0.5, &k) == 0);  // bad edge id
    CC_CHECK(std::strcmp(cc_last_error(), "") != 0);

    cc_shape_release(box);
}

CC_RUN_ALL()
