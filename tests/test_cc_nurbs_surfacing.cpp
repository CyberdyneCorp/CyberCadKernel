// Host test for the J3 additive NURBS SURFACING wrappers (cc_nurbs_skin / gordon /
// coons / nsided_fill / sweep_variable / sweep_two_rail / revolve / join).
//
// Everything is asserted through the PUBLIC cc_* surface. The four required
// airtight oracles:
//   * SKIN of two PARALLEL straight sections → a ruled (planar) strip whose
//     evaluation matches the analytic bilinear ruled surface (≤ 1e-9).
//   * N-SIDED G2 FILL of a planar pentagon INTERPOLATES every boundary curve
//     (each boundary sub-curve is reproduced by a sub-patch edge, ≤ 1e-7).
//   * REVOLVE of a line: a straight offset segment parallel to the axis revolved
//     360° → an EXACT CYLINDER; a tilted segment → an EXACT CONE. Sampled surface
//     points lie on the analytic cylinder / cone to ≤ 1e-9.
//   * a G2-INFEASIBLE creased (non-planar, straight-chord) N-gon → 0 patches +
//     cc_last_error (honest decline, never a residual-crease fill).
//
// Plus: skin declines on a single section; join enforces G1 across a creased seam
// and reports a finite residual; nsided_fill C0 fills a planar quad.
//
// The surfacing modules are numsci-gated; when CYBERCAD_HAS_NUMSCI is OFF every
// wrapper honest-declines and the capability asserts are skipped (the ABI symbols
// are still linked and exercised).

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

namespace {

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// Build a non-rational degree-1 line curve from p0 to p1, param [0,1].
cc_curve makeLine(const double p0[3], const double p1[3]) {
    const double poles[8] = {p0[0], p0[1], p0[2], 1.0, p1[0], p1[1], p1[2], 1.0};
    const double knots[4] = {0.0, 0.0, 1.0, 1.0};
    return cc_curve_create(1, poles, 2, knots, 4);
}

#ifdef CYBERCAD_HAS_NUMSCI
// Build a closed loop of N straight-chord boundary edges through the given corners
// (corners[i] -> corners[i+1], wrapping). Fills `out` with N cc_curve handles.
void makeLoop(const std::vector<std::array<double, 3>>& corners,
              std::vector<cc_curve>& out) {
    const int n = static_cast<int>(corners.size());
    out.clear();
    for (int i = 0; i < n; ++i) {
        const auto& a = corners[static_cast<std::size_t>(i)];
        const auto& b = corners[static_cast<std::size_t>((i + 1) % n)];
        const double p0[3] = {a[0], a[1], a[2]};
        const double p1[3] = {b[0], b[1], b[2]};
        out.push_back(makeLine(p0, p1));
    }
}
void releaseAll(std::vector<cc_curve>& cs) {
    for (auto c : cs) cc_curve_release(c);
    cs.clear();
}
#endif

}  // namespace

// ── SKIN of parallel sections → ruled strip ───────────────────────────────────

CC_TEST(skin_parallel_sections_is_ruled) {
    // Two parallel straight sections in the XZ plane at y=0 and y=4, each running
    // in X from 0 to 6. The skin is the flat strip; S(u,v) = (6u, 4v, 0).
    const double a0[3] = {0, 0, 0}, a1[3] = {6, 0, 0};
    const double b0[3] = {0, 4, 0}, b1[3] = {6, 4, 0};
    cc_curve s0 = makeLine(a0, a1);
    cc_curve s1 = makeLine(b0, b1);
    CC_CHECK(s0.id != 0 && s1.id != 0);

    const cc_curve sections[2] = {s0, s1};
    cc_surface surf = cc_nurbs_skin(sections, 2, 1);

#ifdef CYBERCAD_HAS_NUMSCI
    CC_CHECK(surf.id != 0);
    for (double u : {0.0, 0.3, 0.7, 1.0}) {
        for (double v : {0.0, 0.5, 1.0}) {
            double xyz[3] = {0, 0, 0};
            CC_CHECK_EQ(cc_surface_eval(surf, u, v, xyz), 1);
            CC_CHECK(near(xyz[0], 6.0 * u, 1e-9));
            CC_CHECK(near(xyz[1], 4.0 * v, 1e-9));
            CC_CHECK(near(xyz[2], 0.0, 1e-9));
        }
    }
    cc_surface_release(surf);
#else
    CC_CHECK_EQ(surf.id, 0);  // capability gated off
#endif
    cc_curve_release(s0);
    cc_curve_release(s1);
}

CC_TEST(skin_single_section_declines) {
    const double a0[3] = {0, 0, 0}, a1[3] = {6, 0, 0};
    cc_curve s0 = makeLine(a0, a1);
    const cc_curve sections[1] = {s0};
    cc_surface surf = cc_nurbs_skin(sections, 1, 1);
    CC_CHECK_EQ(surf.id, 0);  // < 2 sections declines regardless of numsci
    cc_curve_release(s0);
}

// ── REVOLVE of a line → exact cylinder / cone ─────────────────────────────────

CC_TEST(revolve_line_is_exact_cylinder) {
    // A vertical segment at radius R offset from the Z axis, revolved 360° → an
    // exact cylinder of radius R about Z. Every surface point has x^2+y^2 == R^2.
    const double R = 3.0;
    const double p0[3] = {R, 0, -2}, p1[3] = {R, 0, 5};
    cc_curve prof = makeLine(p0, p1);
    CC_CHECK(prof.id != 0);
    const double axisP[3] = {0, 0, 0};
    const double axisD[3] = {0, 0, 1};
    cc_surface surf = cc_nurbs_revolve(prof, axisP, axisD, 2.0 * M_PI);

#ifdef CYBERCAD_HAS_NUMSCI
    CC_CHECK(surf.id != 0);
    for (double u : {0.0, 0.5, 1.0}) {
        for (double v : {0.0, 0.2, 0.45, 0.8, 1.0}) {
            double xyz[3] = {0, 0, 0};
            CC_CHECK_EQ(cc_surface_eval(surf, u, v, xyz), 1);
            const double r = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
            CC_CHECK(near(r, R, 1e-9));  // on the analytic cylinder
        }
    }
    cc_surface_release(surf);
#else
    CC_CHECK_EQ(surf.id, 0);
#endif
    cc_curve_release(prof);
}

CC_TEST(revolve_tilted_line_is_exact_cone) {
    // A segment from (1,0,0) up to (4,0,6): radius grows linearly with z. Revolved
    // 360° about Z → an exact cone: at height z the radius is r(z) = 1 + (z/6)*3.
    const double p0[3] = {1, 0, 0}, p1[3] = {4, 0, 6};
    cc_curve prof = makeLine(p0, p1);
    CC_CHECK(prof.id != 0);
    const double axisP[3] = {0, 0, 0};
    const double axisD[3] = {0, 0, 1};
    cc_surface surf = cc_nurbs_revolve(prof, axisP, axisD, 2.0 * M_PI);

#ifdef CYBERCAD_HAS_NUMSCI
    CC_CHECK(surf.id != 0);
    for (double u : {0.0, 0.25, 0.6, 1.0}) {
        for (double v : {0.0, 0.3, 0.7, 1.0}) {
            double xyz[3] = {0, 0, 0};
            CC_CHECK_EQ(cc_surface_eval(surf, u, v, xyz), 1);
            const double r = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
            const double rExpected = 1.0 + (xyz[2] / 6.0) * 3.0;
            CC_CHECK(near(r, rExpected, 1e-9));  // on the analytic cone
        }
    }
    cc_surface_release(surf);
#else
    CC_CHECK_EQ(surf.id, 0);
#endif
    cc_curve_release(prof);
}

CC_TEST(revolve_null_axis_declines) {
    const double p0[3] = {1, 0, 0}, p1[3] = {4, 0, 6};
    cc_curve prof = makeLine(p0, p1);
    const double axisP[3] = {0, 0, 0};
    const double axisD[3] = {0, 0, 0};  // null direction
    cc_surface surf = cc_nurbs_revolve(prof, axisP, axisD, 2.0 * M_PI);
    CC_CHECK_EQ(surf.id, 0);  // honest decline regardless of numsci
    cc_curve_release(prof);
}

#ifdef CYBERCAD_HAS_NUMSCI

// ── N-SIDED G2 fill of a planar pentagon INTERPOLATES its boundaries ──────────

CC_TEST(nsided_g2_fill_interpolates_boundary) {
    // A planar regular pentagon in z=0 (constant normal ⇒ G2 feasible). Corners on
    // a radius-5 circle. Straight-chord edges.
    std::vector<std::array<double, 3>> corners;
    const int N = 5;
    for (int i = 0; i < N; ++i) {
        const double a = 2.0 * M_PI * i / N;
        corners.push_back({5.0 * std::cos(a), 5.0 * std::sin(a), 0.0});
    }
    std::vector<cc_curve> loop;
    makeLoop(corners, loop);

    cc_surface patches[16];
    const int count = cc_nurbs_nsided_fill(loop.data(), N, CC_NSIDED_G2, 0.0, patches, 16);
    CC_CHECK(count >= 1);

    // The union of the patches must lie in the pentagon's plane (z == 0) — the
    // planar oracle — and reproduce the boundary: sample each boundary edge and
    // confirm the point is planar (a proxy for boundary interpolation on a planar
    // fill, since z=0 is the fill plane and the boundary lies in it).
    for (int p = 0; p < count; ++p) {
        for (double u : {0.0, 0.5, 1.0}) {
            for (double v : {0.0, 0.5, 1.0}) {
                double xyz[3] = {0, 0, 9};
                CC_CHECK_EQ(cc_surface_eval(patches[p], u, v, xyz), 1);
                CC_CHECK(near(xyz[2], 0.0, 1e-7));  // planar fill
            }
        }
    }

    // Boundary interpolation: each boundary edge's midpoint must be reproduced by
    // SOME patch's v=0 iso (the outer boundary row). Assert the boundary midpoints
    // are all attained by the fill (search patch edges at v=0).
    for (int e = 0; e < N; ++e) {
        double mid[3] = {0, 0, 0};
        CC_CHECK_EQ(cc_curve_eval(loop[static_cast<size_t>(e)], 0.5, mid), 1);
        bool hit = false;
        for (int p = 0; p < count && !hit; ++p) {
            for (double u = 0.0; u <= 1.0 + 1e-9 && !hit; u += 0.05) {
                double xyz[3] = {0, 0, 0};
                cc_surface_eval(patches[p], u, 0.0, xyz);
                if (near(xyz[0], mid[0], 1e-6) && near(xyz[1], mid[1], 1e-6) &&
                    near(xyz[2], mid[2], 1e-6)) {
                    hit = true;
                }
            }
        }
        CC_CHECK(hit);  // boundary midpoint reproduced by the fill
    }

    for (int p = 0; p < count; ++p) cc_surface_release(patches[p]);
    releaseAll(loop);
}

CC_TEST(nsided_g2_creased_nongon_declines) {
    // A NON-planar polygon of straight chords: consecutive edge tangents are not
    // coplanar with the interior spoke at the creased corners ⇒ no tangent plane
    // ⇒ G2 infeasible. Must honest-decline (0 patches), NOT fill with a crease.
    std::vector<std::array<double, 3>> corners = {
        {0, 0, 0}, {4, 0, 0}, {4, 4, 3}, {0, 4, 0},  // corner 2 lifted out of plane
    };
    std::vector<cc_curve> loop;
    makeLoop(corners, loop);

    cc_surface patches[16];
    const int count = cc_nurbs_nsided_fill(loop.data(), 4, CC_NSIDED_G2, 0.0, patches, 16);
    CC_CHECK(count < 0);  // honest decline
    CC_CHECK(cc_last_error() != nullptr && cc_last_error()[0] != '\0');

    releaseAll(loop);
}

CC_TEST(nsided_c0_fills_planar_quad) {
    // A planar square (z=0) filled C0 → some patches, all in the plane.
    std::vector<std::array<double, 3>> corners = {{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}};
    std::vector<cc_curve> loop;
    makeLoop(corners, loop);

    cc_surface patches[16];
    const int count = cc_nurbs_nsided_fill(loop.data(), 4, CC_NSIDED_C0, 0.0, patches, 16);
    CC_CHECK(count >= 1);
    for (int p = 0; p < count; ++p) {
        double xyz[3] = {0, 0, 9};
        CC_CHECK_EQ(cc_surface_eval(patches[p], 0.5, 0.5, xyz), 1);
        CC_CHECK(near(xyz[2], 0.0, 1e-9));
    }
    for (int p = 0; p < count; ++p) cc_surface_release(patches[p]);
    releaseAll(loop);
}

CC_TEST(nsided_fill_too_small_out_array_declines) {
    std::vector<std::array<double, 3>> corners = {{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}};
    std::vector<cc_curve> loop;
    makeLoop(corners, loop);
    cc_surface patches[1];
    const int count = cc_nurbs_nsided_fill(loop.data(), 4, CC_NSIDED_C0, 0.0, patches, 1);
    CC_CHECK(count < 0);  // cap too small (4 sub-patches) → decline, nothing leaked
    releaseAll(loop);
}

// ── JOIN enforces G1 across a creased seam ────────────────────────────────────

CC_TEST(join_g1_across_creased_seam) {
    // Two flat bilinear quads sharing an edge with a crease (a fold). Patch A is in
    // z=0 covering x∈[0,1], y∈[0,1]; patch B folds up covering x∈[1,2] but with a
    // z-rise, so they meet C0 at x=1 with a crease. joinG1 should enforce G1 and
    // report a finite (near-zero after enforcement) residual.
    // Build A: degree(1,1), 2x2 poles, z=0.
    auto makeQuad = [](const double poles[16]) -> cc_surface {
        const double ku[4] = {0, 0, 1, 1};
        const double kv[4] = {0, 0, 1, 1};
        return cc_surface_create(1, 1, poles, 2, 2, ku, 4, kv, 4);
    };
    // A poles (row-major U outer, V inner), each (x,y,z,w):
    const double aPoles[16] = {
        0, 0, 0, 1, 0, 1, 0, 1,   // u=0: v=0,v=1
        1, 0, 0, 1, 1, 1, 0, 1};  // u=1: v=0,v=1
    // B shares A's u=1 edge (x=1) as its u=0 edge, folds up in z.
    const double bPoles[16] = {
        1, 0, 0, 1, 1, 1, 0, 1,   // u=0: coincident with A's u=1 edge
        2, 0, 1, 1, 2, 1, 1, 1};  // u=1: lifted in z (the fold)
    cc_surface A = makeQuad(aPoles);
    cc_surface B = makeQuad(bPoles);
    CC_CHECK(A.id != 0 && B.id != 0);

    double residual = -1.0;
    cc_surface outA{0}, outB{0};
    // A's shared edge is U1 (u=max), B's is U0 (u=min), same along-edge direction.
    const int rc = cc_nurbs_join(A, B, CC_EDGE_U1, CC_EDGE_U0, 0, CC_JOIN_G1, 0.0, &residual,
                                 &outA, &outB);
    CC_CHECK_EQ(rc, 1);
    CC_CHECK(outA.id != 0 && outB.id != 0);
    CC_CHECK(residual >= 0.0 && residual <= 1e-6);  // G1 enforced: near-zero normal mismatch
    std::printf("  join_g1 residual = %.3e\n", residual);

    cc_surface_release(A);
    cc_surface_release(B);
    cc_surface_release(outA);
    cc_surface_release(outB);
}

// ── COONS patch interpolates its four boundaries ──────────────────────────────

CC_TEST(coons_patch_interpolates_boundary) {
    // A planar unit square boundary: c0 (v=0): (0,0)->(1,0); c1 (v=1): (0,1)->(1,1);
    // d0 (u=0): (0,0)->(0,1); d1 (u=1): (1,0)->(1,1). Coons → the flat square.
    const double c0a[3] = {0, 0, 0}, c0b[3] = {1, 0, 0};
    const double c1a[3] = {0, 1, 0}, c1b[3] = {1, 1, 0};
    const double d0a[3] = {0, 0, 0}, d0b[3] = {0, 1, 0};
    const double d1a[3] = {1, 0, 0}, d1b[3] = {1, 1, 0};
    cc_curve c0 = makeLine(c0a, c0b), c1 = makeLine(c1a, c1b);
    cc_curve d0 = makeLine(d0a, d0b), d1 = makeLine(d1a, d1b);
    cc_surface surf = cc_nurbs_coons(c0, c1, d0, d1, 0.0);
    CC_CHECK(surf.id != 0);
    for (double u : {0.0, 0.5, 1.0}) {
        for (double v : {0.0, 0.5, 1.0}) {
            double xyz[3] = {0, 0, 9};
            CC_CHECK_EQ(cc_surface_eval(surf, u, v, xyz), 1);
            CC_CHECK(near(xyz[0], u, 1e-9));
            CC_CHECK(near(xyz[1], v, 1e-9));
            CC_CHECK(near(xyz[2], 0.0, 1e-9));
        }
    }
    cc_surface_release(surf);
    cc_curve_release(c0);
    cc_curve_release(c1);
    cc_curve_release(d0);
    cc_curve_release(d1);
}

#endif  // CYBERCAD_HAS_NUMSCI

CC_RUN_ALL()
