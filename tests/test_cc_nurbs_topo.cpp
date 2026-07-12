// Host test for the J5 NURBS topology wrappers (src/facade/cc_nurbs_topo.cpp):
// curve/curve + curve/surface intersection and the parameter-space trim-region
// boolean, asserted entirely through the public cc_* surface.
//
// Oracles (every asserted tolerance is the ACHIEVED error, never widened):
//   * line <-> circle → EXACTLY 2 transversal hits at the known parameters ≤ 1e-10;
//   * line <-> plane → the exact pierce point ≤ 1e-10;
//   * overlapping unit squares → Union / Intersect / Difference areas match the
//     closed-form values ≤ 1e-10;
//   * two coincident loops (identical squares, shared boundary) → HONEST DECLINE.
//   * a sizeof guard for every new J5 POD struct, cross-checked against a C sizeof
//     of the same header (cc_nurbs_topo_c_sizeof_* from test_cc_nurbs_topo_sizeof.c).

#include <cmath>
#include <cstdint>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

// Sizes computed by the C compiler over the same header (test_cc_nurbs_topo_sizeof.c).
extern "C" {
extern const size_t cc_nurbs_topo_c_sizeof_curvehit;
extern const size_t cc_nurbs_topo_c_sizeof_cshit;
extern const size_t cc_nurbs_topo_c_sizeof_trimloop;
}

namespace {

// A straight degree-1 line segment from p0 to p1 in the XY plane (z=0), as a
// cc_curve (2 poles, clamped knots {0,0,1,1}).
cc_curve makeLine(double x0, double y0, double x1, double y1) {
    const std::vector<double> poles = {x0, y0, 0.0, 1.0, x1, y1, 0.0, 1.0};
    const std::vector<double> knots = {0.0, 0.0, 1.0, 1.0};
    return cc_curve_create(1, poles.data(), 2, knots.data(),
                           static_cast<int>(knots.size()));
}

// A straight degree-1 line segment from (x0,y0,z0) to (x1,y1,z1) as a cc_curve.
cc_curve makeLine3(double x0, double y0, double z0, double x1, double y1, double z1) {
    const std::vector<double> poles = {x0, y0, z0, 1.0, x1, y1, z1, 1.0};
    const std::vector<double> knots = {0.0, 0.0, 1.0, 1.0};
    return cc_curve_create(1, poles.data(), 2, knots.data(),
                           static_cast<int>(knots.size()));
}

// A full unit circle of radius R centred at (cx,cy) in the XY plane as an EXACT
// rational degree-2 NURBS (9 poles, the standard 4-quarter construction).
cc_curve makeCircle(double cx, double cy, double R) {
    const double w = std::sqrt(2.0) / 2.0;
    // Square control polygon corners + edge midpoints, weight w on the corners.
    const std::vector<double> poles = {
        cx + R, cy,     0.0, 1.0,   // 0  (R,0)
        cx + R, cy + R, 0.0, w,     // 1  corner
        cx,     cy + R, 0.0, 1.0,   // 2  (0,R)
        cx - R, cy + R, 0.0, w,     // 3  corner
        cx - R, cy,     0.0, 1.0,   // 4  (-R,0)
        cx - R, cy - R, 0.0, w,     // 5  corner
        cx,     cy - R, 0.0, 1.0,   // 6  (0,-R)
        cx + R, cy - R, 0.0, w,     // 7  corner
        cx + R, cy,     0.0, 1.0,   // 8  close
    };
    const std::vector<double> knots = {0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5,
                                       0.75, 0.75, 1.0, 1.0, 1.0};
    return cc_curve_create(2, poles.data(), 9, knots.data(),
                           static_cast<int>(knots.size()));
}

// A flat degree-1 x degree-1 plane z = z0 spanning [x0,x1] x [y0,y1] as a cc_surface
// (2x2 poles). Row-major, U outer.
cc_surface makePlane(double x0, double y0, double x1, double y1, double z0) {
    const std::vector<double> poles = {
        x0, y0, z0, 1.0,  x0, y1, z0, 1.0,   // U=0 row: (v0, v1)
        x1, y0, z0, 1.0,  x1, y1, z0, 1.0,   // U=1 row
    };
    const std::vector<double> ku = {0.0, 0.0, 1.0, 1.0};
    const std::vector<double> kv = {0.0, 0.0, 1.0, 1.0};
    return cc_surface_create(1, 1, poles.data(), 2, 2, ku.data(), 4, kv.data(), 4);
}

// A CCW axis-aligned rectangle [x0,x1]x[y0,y1] as a degree-1 closed polyline
// cc_curve (5 poles, first == last). Its (x,y) poles are read as (u,v).
cc_curve makeRect(double x0, double y0, double x1, double y1) {
    const std::vector<double> poles = {
        x0, y0, 0.0, 1.0,  x1, y0, 0.0, 1.0,  x1, y1, 0.0, 1.0,
        x0, y1, 0.0, 1.0,  x0, y0, 0.0, 1.0,
    };
    // Degree-1, 5 poles → 5+1+1 = 7 knots, clamped uniform.
    const std::vector<double> knots = {0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0};
    return cc_curve_create(1, poles.data(), 5, knots.data(),
                           static_cast<int>(knots.size()));
}

}  // namespace

CC_TEST(sizeof_guards_match_c) {
    CC_CHECK_EQ(sizeof(CCCurveHit), cc_nurbs_topo_c_sizeof_curvehit);
    CC_CHECK_EQ(sizeof(CCCurveSurfaceHit), cc_nurbs_topo_c_sizeof_cshit);
    CC_CHECK_EQ(sizeof(CCTrimLoop), cc_nurbs_topo_c_sizeof_trimloop);
    // Field-count / layout sanity (the ABI these wrappers publish).
    CC_CHECK(sizeof(CCCurveHit) >= 5 * sizeof(double));       // xyz[3]+tA+tB+flag
    CC_CHECK(sizeof(CCCurveSurfaceHit) >= 6 * sizeof(double));  // xyz[3]+t+u+v+flag
    CC_CHECK(sizeof(CCTrimLoop) == 3 * sizeof(double));      // ptr + 2 int + double
}

#ifdef CYBERCAD_HAS_NUMSCI
CC_TEST(line_circle_two_hits) {
    // Line y = 0 across a unit circle centred at origin → hits at (-1,0) and (1,0).
    cc_curve line = makeLine(-2.0, 0.0, 2.0, 0.0);
    cc_curve circ = makeCircle(0.0, 0.0, 1.0);
    CC_CHECK(line.id != 0);
    CC_CHECK(circ.id != 0);

    CCCurveHit* hits = nullptr;
    const int n = cc_nurbs_intersect_cc(line, circ, 1e-12, &hits);
    CC_CHECK_EQ(n, 2);
    if (n == 2 && hits != nullptr) {
        // Sort by x so the assertions are order-independent.
        double xs[2] = {hits[0].xyz[0], hits[1].xyz[0]};
        if (xs[0] > xs[1]) {
            std::swap(xs[0], xs[1]);
        }
        CC_CHECK(std::fabs(xs[0] - (-1.0)) <= 1e-10);
        CC_CHECK(std::fabs(xs[1] - (1.0)) <= 1e-10);
        for (int i = 0; i < 2; ++i) {
            CC_CHECK(std::fabs(hits[i].xyz[1]) <= 1e-10);   // y = 0
            CC_CHECK(std::fabs(hits[i].xyz[2]) <= 1e-10);   // z = 0
            // A line crossing a circle transversally.
            CC_CHECK_EQ(hits[i].tangential, 0);
            // Line param t ∈ [0,1] maps x = -2 + 4t; recover and check the point.
            const double x = -2.0 + 4.0 * hits[i].tA;
            CC_CHECK(std::fabs(x - hits[i].xyz[0]) <= 1e-9);
        }
    }
    cc_nurbs_hits_cc_free(hits);
    cc_curve_release(line);
    cc_curve_release(circ);
}

CC_TEST(line_plane_pierce_exact) {
    // A vertical line from (0.3,0.4,-1) to (0.3,0.4,+1) pierces the plane z=0 at
    // (0.3, 0.4, 0). The plane spans [0,1]x[0,1].
    cc_curve line = makeLine3(0.3, 0.4, -1.0, 0.3, 0.4, 1.0);
    cc_surface plane = makePlane(0.0, 0.0, 1.0, 1.0, 0.0);
    CC_CHECK(line.id != 0);
    CC_CHECK(plane.id != 0);

    CCCurveSurfaceHit* hits = nullptr;
    const int n = cc_nurbs_intersect_cs(line, plane, 1e-12, &hits);
    CC_CHECK_EQ(n, 1);
    if (n == 1 && hits != nullptr) {
        CC_CHECK(std::fabs(hits[0].xyz[0] - 0.3) <= 1e-10);
        CC_CHECK(std::fabs(hits[0].xyz[1] - 0.4) <= 1e-10);
        CC_CHECK(std::fabs(hits[0].xyz[2] - 0.0) <= 1e-10);
        CC_CHECK_EQ(hits[0].tangential, 0);
    }
    cc_nurbs_hits_cs_free(hits);
    cc_curve_release(line);
    cc_surface_release(plane);
}

CC_TEST(intersect_unknown_handle_declines) {
    cc_curve bad{0};
    CCCurveHit* ccHits = reinterpret_cast<CCCurveHit*>(0x1);
    CC_CHECK(cc_nurbs_intersect_cc(bad, bad, 1e-9, &ccHits) < 0);
    CC_CHECK(ccHits == nullptr);

    cc_surface badS{0};
    CCCurveSurfaceHit* csHits = reinterpret_cast<CCCurveSurfaceHit*>(0x1);
    CC_CHECK(cc_nurbs_intersect_cs(bad, badS, 1e-9, &csHits) < 0);
    CC_CHECK(csHits == nullptr);
}
#endif  // CYBERCAD_HAS_NUMSCI

CC_TEST(trim_overlapping_squares_areas) {
    // A = [0,2]x[0,2] (area 4), B = [1,3]x[1,3] (area 4); overlap [1,2]x[1,2] = 1.
    cc_curve a = makeRect(0.0, 0.0, 2.0, 2.0);
    cc_curve b = makeRect(1.0, 1.0, 3.0, 3.0);
    CC_CHECK(a.id != 0);
    CC_CHECK(b.id != 0);

    struct Case {
        CCTrimBoolOp op;
        double expected;
    };
    const Case cases[] = {
        {CC_TRIM_UNION, 7.0},        // 4 + 4 - 1
        {CC_TRIM_INTERSECT, 1.0},    // the overlap
        {CC_TRIM_DIFFERENCE, 3.0},   // A \ B = 4 - 1
    };
    for (const Case& c : cases) {
        CCTrimLoop* loops = nullptr;
        double area = -999.0;
        const int n = cc_nurbs_trim_region_boolean(&a, 1, &b, 1, c.op, &loops, &area);
        CC_CHECK(n >= 1);
        CC_CHECK(std::fabs(std::fabs(area) - c.expected) <= 1e-10);
        cc_nurbs_trim_loops_free(loops, n);
    }
    cc_curve_release(a);
    cc_curve_release(b);
}

CC_TEST(trim_coincident_loops_decline) {
    // Two IDENTICAL squares share their whole boundary → coincident-edge overlap,
    // which is ambiguous for a region boolean → HONEST DECLINE (< 0), NULL out.
    cc_curve a = makeRect(0.0, 0.0, 2.0, 2.0);
    cc_curve b = makeRect(0.0, 0.0, 2.0, 2.0);
    CC_CHECK(a.id != 0);
    CC_CHECK(b.id != 0);

    CCTrimLoop* loops = reinterpret_cast<CCTrimLoop*>(0x1);
    double area = -999.0;
    const int n =
        cc_nurbs_trim_region_boolean(&a, 1, &b, 1, CC_TRIM_UNION, &loops, &area);
    CC_CHECK(n < 0);
    CC_CHECK(loops == nullptr);
    CC_CHECK(area == 0.0);
    cc_curve_release(a);
    cc_curve_release(b);
}

CC_TEST(trim_unknown_handle_declines) {
    // A zero (invalid) loop handle → honest decline, NULL out.
    cc_curve bad{0};
    CCTrimLoop* loops = reinterpret_cast<CCTrimLoop*>(0x1);
    double area = -999.0;
    CC_CHECK(cc_nurbs_trim_region_boolean(&bad, 1, &bad, 1, CC_TRIM_UNION, &loops,
                                          &area) < 0);
    CC_CHECK(loops == nullptr);
    CC_CHECK(area == 0.0);
}

CC_RUN_ALL()
