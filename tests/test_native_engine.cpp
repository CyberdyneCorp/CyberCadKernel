// Host unit test for the NativeEngine facade toggle (cc_set_engine /
// cc_active_engine) and the native construct ops it implements (Phase 4 #4,
// native-construction). OCCT-FREE — this is Gate 1 (host, analytic) of the
// two-gate model in openspec/NATIVE-REWRITE.md.
//
// The point of this suite: in the no-OCCT HOST build the DEFAULT engine is the
// stub (cc_solid_extrude returns 0). After cc_set_engine(1) the NativeEngine
// builds the SAME cc_solid_extrude / cc_solid_revolve natively — proving the
// native construct + tessellate path works with NO OCCT linked, and that the
// additive toggle leaves the default untouched until opted in.
//
// Assertions are analytic (exact volumes/areas of known solids) within the
// deflection-derived convergence bound the native tessellator guarantees.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

namespace {
constexpr double kPi = 3.14159265358979323846;

// Reset to the default engine after each toggling test so cases stay independent.
struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};
}  // namespace

// The default engine is NOT native — the toggle is opt-in, existing behaviour
// unchanged.
CC_TEST(default_engine_is_not_native) {
    EngineGuard g;
    CC_CHECK_EQ(cc_active_engine(), 0);
    cc_set_engine(1);
    CC_CHECK_EQ(cc_active_engine(), 1);
    cc_set_engine(0);
    CC_CHECK_EQ(cc_active_engine(), 0);
    CC_CHECK(std::strcmp(cc_last_error(), "") == 0);
}

// Native solid_extrude builds a prism with NO OCCT: a unit-square profile
// extruded by depth 2 → a 1×1×2 box (volume 2, surface area 2·(1) + 4·(1·2) = 10).
CC_TEST(native_extrude_unit_box_volume_and_watertight) {
    EngineGuard g;
    cc_set_engine(1);

    const double profile[] = {0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0};  // CCW unit square
    const CCShapeId id = cc_solid_extrude(profile, 4, 2.0);
    CC_CHECK(id != 0);
    if (id == 0) {
        std::printf("  last_error=%s\n", cc_last_error());
        return;
    }

    // Mass properties come from the native watertight mesh (divergence theorem).
    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(std::fabs(mp.volume - 2.0) < 1e-3);
    CC_CHECK(std::fabs(mp.area - 10.0) < 1e-3);
    // Centroid of a box spanning [0,1]×[0,1]×[0,2] is its center.
    CC_CHECK(std::fabs(mp.cx - 0.5) < 1e-3);
    CC_CHECK(std::fabs(mp.cy - 0.5) < 1e-3);
    CC_CHECK(std::fabs(mp.cz - 1.0) < 1e-3);

    // Bounding box exactly the profile extents.
    double bb[6] = {0};
    CC_CHECK_EQ(cc_bounding_box(id, bb), 1);
    CC_CHECK(std::fabs(bb[0] - 0.0) < 1e-9 && std::fabs(bb[1] - 0.0) < 1e-9 &&
             std::fabs(bb[2] - 0.0) < 1e-9);
    CC_CHECK(std::fabs(bb[3] - 1.0) < 1e-9 && std::fabs(bb[4] - 1.0) < 1e-9 &&
             std::fabs(bb[5] - 2.0) < 1e-9);

    // Native tessellation produces a non-empty mesh.
    CCMesh mesh = cc_tessellate(id, 0.1);
    CC_CHECK(mesh.vertexCount > 0);
    CC_CHECK(mesh.triangleCount >= 12);  // a box is at least 12 triangles
    cc_mesh_free(mesh);

    cc_shape_release(id);
}

// A rectangular (non-unit) profile extruded → a 6-face box: 3×2 base, depth 5 →
// volume 30, and exactly 6 faces (2 caps + 4 quads). Rectangular caps are the
// native tessellator's exact fast-path (see the isFullRectangle scope note in
// construct.h); a well-formed native prism over such a profile is watertight.
CC_TEST(native_extrude_rectangular_box) {
    EngineGuard g;
    cc_set_engine(1);

    const double profile[] = {0.0, 0.0, 3.0, 0.0, 3.0, 2.0, 0.0, 2.0};
    const CCShapeId id = cc_solid_extrude(profile, 4, 5.0);
    CC_CHECK(id != 0);
    if (id == 0) {
        std::printf("  last_error=%s\n", cc_last_error());
        return;
    }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(std::fabs(mp.volume - 30.0) < 1e-3);
    CC_CHECK(std::fabs(mp.area - 2.0 * (3.0 * 2.0 + 3.0 * 5.0 + 2.0 * 5.0)) < 1e-3);

    CCFaceMesh* faces = nullptr;
    const int nFaces = cc_face_meshes(id, 0.1, &faces);
    CC_CHECK_EQ(nFaces, 6);
    cc_face_meshes_free(faces, nFaces);

    cc_shape_release(id);
}

// A triangle profile extruded → a triangular prism with EXACTLY 5 faces (2 caps +
// 3 quads), the correct native TOPOLOGY, and — now that the tessellator's convex
// cap-fill is fixed (for a planar face trim.h isFullRectangle also requires the loop
// to hit all four box corners)
// — a WATERTIGHT mesh with the exact profile-area × depth volume. A right triangle
// with legs 2 has area 2; ×depth 5 = volume 10.
CC_TEST(native_extrude_triangle_prism_watertight_volume) {
    EngineGuard g;
    cc_set_engine(1);

    const double tri[] = {0.0, 0.0, 2.0, 0.0, 0.0, 2.0};  // right triangle, legs 2, area 2
    const CCShapeId id = cc_solid_extrude(tri, 3, 5.0);
    CC_CHECK(id != 0);
    if (id == 0) {
        std::printf("  last_error=%s\n", cc_last_error());
        return;
    }

    // 5 faces: bottom + top cap + one quad per profile edge (3).
    CCFaceMesh* faces = nullptr;
    const int nFaces = cc_face_meshes(id, 0.1, &faces);
    CC_CHECK_EQ(nFaces, 5);
    cc_face_meshes_free(faces, nFaces);

    // Native subshape query: 3 vertices per cap = 6 vertices, 5 faces.
    int* vids = nullptr;
    const int nV = cc_subshape_ids(id, 0, &vids);
    cc_ints_free(vids);
    CC_CHECK_EQ(nV, 6);
    int* fids = nullptr;
    const int nF = cc_subshape_ids(id, 2, &fids);
    cc_ints_free(fids);
    CC_CHECK_EQ(nF, 5);

    // Mass properties come from the native WATERTIGHT mesh (divergence theorem);
    // its validity flag requires a watertight mesh, and the volume is the exact
    // profile-area × depth (2 × 5 = 10) now that the convex cap meshes correctly.
    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);  // valid ⇒ mesh watertight (see NativeEngine mass_properties)
    CC_CHECK(std::fabs(mp.volume - 10.0) < 1e-6);

    cc_shape_release(id);
}

// Degenerate profiles are REJECTED by the native builder (returns null) and fall
// through to the default engine (the stub on host → 0). Covers: < 3 points, a
// zero-area (collinear) profile, and depth ≤ 0. The native path never fabricates a
// shape for a degenerate input.
CC_TEST(native_extrude_degenerate_rejected) {
    EngineGuard g;
    cc_set_engine(1);

    // Only 2 points.
    const double twoPts[] = {0.0, 0.0, 1.0, 0.0};
    CC_CHECK_EQ(cc_solid_extrude(twoPts, 2, 1.0), 0);

    // Collinear (zero-area) triple.
    const double collinear[] = {0.0, 0.0, 1.0, 0.0, 2.0, 0.0};
    CC_CHECK_EQ(cc_solid_extrude(collinear, 3, 1.0), 0);

    // Non-positive depth.
    const double square[] = {0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0};
    CC_CHECK_EQ(cc_solid_extrude(square, 4, 0.0), 0);
    CC_CHECK_EQ(cc_solid_extrude(square, 4, -1.0), 0);
}

// A cone: profile [(0,0),(2,0),(0,3)] revolved 360° about the Y axis → a cone of
// base radius 2, height 3, volume ⅓·π·r²·h = 4π.
CC_TEST(native_revolve_cone_volume) {
    EngineGuard g;
    cc_set_engine(1);

    const double profile[] = {0.0, 0.0, 2.0, 0.0, 0.0, 3.0};
    const CCShapeId id = cc_solid_revolve(profile, 3, 2.0 * kPi);
    CC_CHECK(id != 0);
    if (id == 0) {
        std::printf("  last_error=%s\n", cc_last_error());
        return;
    }
    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    const double expected = kPi * 4.0 * 3.0 / 3.0;  // 4π
    CC_CHECK(std::fabs(mp.volume - expected) / expected < 0.05);

    cc_shape_release(id);
}

// Partial-turn revolve: the same tube revolved 90° has ¼ the full-turn volume and
// is watertight (two planar caps close the opening).
CC_TEST(native_revolve_partial_turn_quarter) {
    EngineGuard g;
    cc_set_engine(1);

    const double profile[] = {1.0, 0.0, 2.0, 0.0, 2.0, 3.0, 1.0, 3.0};
    const CCShapeId id = cc_solid_revolve(profile, 4, kPi / 2.0);
    CC_CHECK(id != 0);
    if (id == 0) return;

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    const double expected = kPi * (4.0 - 1.0) * 3.0 / 4.0;  // 9π/4
    CC_CHECK(std::fabs(mp.volume - expected) / expected < 0.05);

    cc_shape_release(id);
}

// Native solid_revolve (line-segment profile) about the Y axis through the origin:
// a rectangle profile [x∈[1,2], y∈[0,3]] full-turn → a tube (annulus) of volume
// π(2²−1²)·3 = 9π. Uses the deflection-convergence bound, so a loose relative
// tolerance.
CC_TEST(native_revolve_full_turn_tube_volume) {
    EngineGuard g;
    cc_set_engine(1);

    // Rectangle in the profile plane: (1,0)-(2,0)-(2,3)-(1,3), CCW closed loop.
    const double profile[] = {1.0, 0.0, 2.0, 0.0, 2.0, 3.0, 1.0, 3.0};
    const CCShapeId id = cc_solid_revolve(profile, 4, 2.0 * kPi);
    CC_CHECK(id != 0);
    if (id == 0) {
        std::printf("  last_error=%s\n", cc_last_error());
        return;
    }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    const double expected = kPi * (4.0 - 1.0) * 3.0;  // 9π
    // Mesh convergence at deflection 0.05 → within a few percent of the true volume.
    CC_CHECK(std::fabs(mp.volume - expected) / expected < 0.05);

    cc_shape_release(id);
}

// Deferred native cases fall through to the DEFAULT engine (the stub on the host):
// a degenerate profile / a case the native builder does not handle returns 0 with
// an error, exactly as the default engine would — the native path never fakes it.
CC_TEST(native_deferred_falls_through_to_default) {
    EngineGuard g;
    cc_set_engine(1);
    CC_CHECK_EQ(cc_active_engine(), 1);

    // < 3 points: the native builder returns null; falls through to the stub, which
    // is unsupported on the host → 0.
    const double bad[] = {0.0, 0.0, 1.0, 0.0};
    const CCShapeId id = cc_solid_extrude(bad, 2, 1.0);
    CC_CHECK_EQ(id, 0);

    // A DEFERRED loft sub-case (mismatched section counts: 4-gon bottom, 3-gon top)
    // is not built natively — it forwards to the fallback (stub → 0 on host) without
    // crashing or faking. (The equal-count planar loft IS native; see below.)
    const double a[] = {0, 0, 4, 0, 4, 4, 0, 4};  // 4-gon
    const double b[] = {0, 0, 2, 0, 1, 2};        // 3-gon
    const CCShapeId loftId = cc_solid_loft(a, 4, b, 3, 1.0);
    CC_CHECK_EQ(loftId, 0);
}

// ── Tier-B (#4b): native 2-section RULED loft ──────────────────────────────────

// Native cc_solid_loft: a 4×4 bottom square lofted to a 2×2 top square at depth 6
// (both centred) → a square frustum, volume h/3·(A1+A2+√(A1·A2)) = 6/3·(16+4+8) =
// 56. Proves the op is served NATIVELY (the host stub has no loft, so a non-zero id
// can only come from the native ruled-loft builder), and the mass properties come
// from the native watertight mesh.
CC_TEST(native_loft_square_frustum) {
    EngineGuard g;
    cc_set_engine(1);

    const double bot[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double top[] = {-1, -1, 1, -1, 1, 1, -1, 1};
    const CCShapeId id = cc_solid_loft(bot, 4, top, 4, 6.0);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(std::fabs(mp.volume - 56.0) / 56.0 < 0.02);

    CCFaceMesh* faces = nullptr;
    const int nFaces = cc_face_meshes(id, 0.05, &faces);
    CC_CHECK_EQ(nFaces, 6);  // 4 ruled sides + 2 planar caps
    cc_face_meshes_free(faces, nFaces);

    cc_shape_release(id);
}

// Native cc_solid_loft_wires: two planar triangles in parallel z-planes (z=0, z=3)
// → a triangular prism, volume = area·3 = 6·3 = 18. Served natively; watertight.
CC_TEST(native_loft_wires_triangle_prism) {
    EngineGuard g;
    cc_set_engine(1);

    const double a[] = {0, 0, 0, 4, 0, 0, 2, 3, 0};
    const double b[] = {0, 0, 3, 4, 0, 3, 2, 3, 3};
    const CCShapeId id = cc_solid_loft_wires(a, 3, b, 3);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(std::fabs(mp.volume - 18.0) < 1e-2);

    cc_shape_release(id);
}

// ── Tier-A (#4b): native holed / typed-profile extrude + typed-profile revolve ──

// Native cc_solid_extrude_holes: a 10×10 square with one circular through-hole
// (centre (5,5), r=2), depth 4 → volume (100 − 4π)·4. The hole keeps a true circle
// edge; the native mesh is watertight and the mass properties are within the
// deflection bound. Proves the op is served NATIVELY (host stub has no such op, so a
// non-zero id can only come from the native builder).
CC_TEST(native_extrude_circular_hole) {
    EngineGuard g;
    cc_set_engine(1);

    const double outer[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const double holes[] = {5.0, 5.0, 2.0};  // cx,cy,r
    const CCShapeId id = cc_solid_extrude_holes(outer, 4, holes, 1, 4.0);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    const double expected = (100.0 - kPi * 4.0) * 4.0;
    CC_CHECK(std::fabs(mp.volume - expected) / expected < 0.02);
    cc_shape_release(id);
}

// Native cc_solid_extrude_polyholes: 10×10 square with a 2×2 square hole, depth 3 →
// volume (100 − 4)·3 = 288, exact (all planar). Watertight.
CC_TEST(native_extrude_polygon_hole) {
    EngineGuard g;
    cc_set_engine(1);

    const double outer[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const double holeXY[] = {4, 4, 6, 4, 6, 6, 4, 6};
    const int holeCounts[] = {4};
    const CCShapeId id = cc_solid_extrude_polyholes(outer, 4, holeXY, holeCounts, 1, 3.0);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(std::fabs(mp.volume - 288.0) < 1e-3);
    cc_shape_release(id);
}

// Native cc_solid_extrude_profile: a single full-circle typed segment (kind 2,
// centre origin, r=3) extruded depth 5 → a solid cylinder, volume πr²h = 45π,
// meshed from TRUE circle edges. No circular holes.
CC_TEST(native_extrude_profile_full_circle) {
    EngineGuard g;
    cc_set_engine(1);

    CCProfileSeg seg{};
    seg.kind = 2;
    seg.cx = 0; seg.cy = 0; seg.r = 3.0;
    const CCShapeId id = cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, 5.0);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    const double expected = kPi * 9.0 * 5.0;  // 45π
    // Facade mass_properties meshes at the fixed kPropertyDeflection (0.05); a
    // convex curved solid's mesh volume converges from below at that bound, so use
    // the same few-percent tolerance the existing curved-revolve facade tests use.
    CC_CHECK(std::fabs(mp.volume - expected) / expected < 0.03);
    cc_shape_release(id);
}

// Native cc_solid_revolve_profile: an on-axis half-circle arc (kind 1, r=3) revolved
// full 2π about the Y axis → a sphere, volume (4/3)πr³ = 36π. Confirms the typed
// arc-revolve → Sphere native path through the facade.
CC_TEST(native_revolve_profile_sphere) {
    EngineGuard g;
    cc_set_engine(1);

    CCProfileSeg seg{};
    seg.kind = 1;
    seg.cx = 0; seg.cy = 0; seg.r = 3.0;
    seg.x0 = 0; seg.y0 = -3.0; seg.x1 = 0; seg.y1 = 3.0;
    seg.a0 = -kPi / 2.0; seg.a1 = kPi / 2.0;
    const CCShapeId id =
        cc_solid_revolve_profile(&seg, 1, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    const double expected = (4.0 / 3.0) * kPi * 27.0;  // 36π
    // Sphere at the facade's fixed 0.05 deflection: mesh volume converges from below
    // (a sphere is doubly curved, so a coarser bound under-fills more than a
    // cylinder). A finer deflection tightens this (the direct native test at 0.01 is
    // < 3%); at 0.05 the honest bound is ~5%.
    CC_CHECK(std::fabs(mp.volume - expected) / expected < 0.06);
    cc_shape_release(id);
}

// Tier-1 residual: a kind-3 SPLINE outer profile edge is now NATIVE (residuals.h
// build_prism_profile_spline expands the fitted NURBS to a dense polyline and routes
// it through the watertight typed-extrude). The closed spline loop (0,0)→(1,1)→(2,0)
// extrudes to a watertight prism; the engine keeps it native (a non-zero id — the host
// stub has no extrude) with valid mass properties. Meanwhile a SINGLE off-axis quarter
// arc is NOT a closed generatrix, so its revolve does not mesh watertight and the
// engine's robustlyWatertight self-verify correctly DISCARDS it → fallback (stub → 0),
// never a leaky/faked solid.
CC_TEST(native_profile_spline_runs_native) {
    EngineGuard g;
    cc_set_engine(1);

    CCProfileSeg seg{};
    seg.kind = 3;
    seg.ptOffset = 0; seg.ptCount = 3;
    const double spline[] = {0, 0, 1, 1, 2, 0};  // 3 points = 6 doubles
    const CCShapeId id = cc_solid_extrude_profile(&seg, 1, nullptr, 0, spline, 6, 2.0);
    CC_CHECK(id != 0);  // native spline extrude on host (stub has none)
    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(mp.volume > 0.0);
    cc_shape_release(id);

    // A single off-axis quarter arc does not close into a watertight torus solid, so
    // the self-verify discards the native candidate and the op falls through (stub → 0).
    CCProfileSeg arc{};
    arc.kind = 1;
    arc.cx = 5.0; arc.cy = 0.0; arc.r = 1.0;
    arc.x0 = 6.0; arc.y0 = 0.0; arc.x1 = 5.0; arc.y1 = 1.0;
    arc.a0 = 0.0; arc.a1 = kPi / 2.0;
    const CCShapeId tid =
        cc_solid_revolve_profile(&arc, 1, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
    CC_CHECK_EQ(tid, 0);
}

// ── Tier-C (#4b): native STRAIGHT + SMOOTH-CURVED sweep, deferred tight/twisted ─

// Native cc_solid_sweep: a 4×4 square swept 10 along +Z is a 4×4×10 prism, volume
// 160, EXACT. Served natively (the host stub has no sweep, so a non-zero id can only
// come from the native straight-sweep builder); mass properties from the watertight
// native mesh. 6 faces (4 sides + 2 caps).
CC_TEST(native_sweep_straight_prism) {
    EngineGuard g;
    cc_set_engine(1);

    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double path[] = {0, 0, 0, 0, 0, 10};
    const CCShapeId id = cc_solid_sweep(prof, 4, path, 2);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(std::fabs(mp.volume - 160.0) < 1e-3);

    CCFaceMesh* faces = nullptr;
    const int nFaces = cc_face_meshes(id, 0.05, &faces);
    CC_CHECK_EQ(nFaces, 6);
    cc_face_meshes_free(faces, nFaces);

    cc_shape_release(id);
}

// Native cc_solid_sweep on a SMOOTH gentle-curved but PLANAR spine (Tier-C case (b)):
// a 2×2 square swept along a quarter-arc of radius R=20 (≫ the profile circumradius, so
// no self-intersection) builds a CONSTANT-frame ruled tube NATIVELY — a non-zero id on
// the host stub can only come from the native curved-sweep builder. This mirrors OCCT
// MakePipe's PLANAR corrected-Frenet law (a constant rotation → the section is
// translated, not rotated to stay perpendicular), so the volume is
// profile_area × |Δspine · n̂| (the spine displacement projected onto the FIXED section
// normal), NOT the Pappus arc-length volume. The native mesh (0.05) is watertight.
CC_TEST(native_sweep_smooth_arc) {
    EngineGuard g;
    cc_set_engine(1);

    const double prof[] = {-1, -1, 1, -1, 1, 1, -1, 1};  // side 2, area 4
    const double R = 20.0;
    const int N = 24;
    std::vector<double> path;
    for (int k = 0; k <= N; ++k) {
        const double th = (kPi / 2.0) * k / N;
        path.push_back(R * std::cos(th));
        path.push_back(0.0);
        path.push_back(R * std::sin(th));
    }
    const CCShapeId id = cc_solid_sweep(prof, 4, path.data(), N + 1);
    CC_CHECK(id != 0);  // native curved-sweep builder (stub has no sweep)
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    // Constant-frame swept volume: area × |Δspine · n̂|. The start section normal ≈ −Z,
    // the spine displacement is (−R,0,R), so the projection ≈ 20.64 for R=20 (R plus the
    // small in-plane start-tangent tilt) → expected ≈ 4 × 20.64 = 82.57. This is the
    // value OCCT MakePipe reports on this planar spine (verified on the simulator: native
    // vol == OCCT vol to fp precision), NOT the Pappus arc-length volume 125.7.
    const double exact = 82.5747;  // area(4) × |Δspine·n̂|; = OCCT MakePipe on this spine
    CC_CHECK(std::fabs(mp.volume - exact) / exact < 5e-2);  // deflection-bounded

    cc_shape_release(id);
}

// A TIGHT-CURVATURE / self-intersecting sweep spine forwards to the fallback (stub → 0
// on host), never faked — it needs surface-surface intersection (Tier 4). A REAL twist
// A tight-curvature sweep still defers; a REAL PURE twist now runs NATIVE (the densified
// Frenet-framed ruled tube welds watertight at every deflection and converges to the
// area-preserving volume); a twist COMBINED WITH a scale is NOT robustly weldable and is
// discarded by the engine self-verify → OCCT (stub → 0 on host). Proves the native path
// guards the self-intersecting case, serves the pure twist, and honestly declines the
// combined twist+scale (never a faked/leaky solid).
CC_TEST(native_sweep_tight_defer_and_twist_native) {
    EngineGuard g;
    cc_set_engine(1);

    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};  // half-extent 2√2 ≈ 2.83
    // Quarter-arc of radius 2 (< the profile circumradius) → self-intersecting tube.
    std::vector<double> tight;
    for (int k = 0; k <= 12; ++k) {
        const double th = (kPi / 2.0) * k / 12;
        tight.push_back(2.0 * std::cos(th));
        tight.push_back(0.0);
        tight.push_back(2.0 * std::sin(th));
    }
    CC_CHECK_EQ(cc_solid_sweep(prof, 4, tight.data(), 13), 0);  // tight → stub fallback

    // A real 90° PURE twist along a straight spine runs NATIVE (watertight, V → area·L).
    const double path[] = {0, 0, 0, 0, 0, 10};
    const CCShapeId tw = cc_twisted_sweep(prof, 4, path, 2, 1.5708, 1.0);
    CC_CHECK(tw != 0);
    if (tw != 0) {
        const CCMassProps mp = cc_mass_properties(tw);
        CC_CHECK(mp.valid != 0 && mp.volume > 0.0);
        CC_CHECK(std::fabs(mp.volume - 160.0) / 160.0 < 2e-2);  // area-preserving
        cc_shape_release(tw);
    }
    // A twist + scale is not robustly weldable → engine self-verify declines → stub 0.
    CC_CHECK_EQ(cc_twisted_sweep(prof, 4, path, 2, 1.5708, 0.5), 0);
}

// Tier-2#4 (widened envelope): a GUIDED sweep now runs NATIVE — the profile is scaled
// per station by the guide splay dist(path,guide)/d0 into a Frenet ThruSections tube
// that welds watertight (build_guided_sweep). A 4×4 square swept up +Z 10 with a guide
// splaying the section from ×3 to ×6 produces a positive-volume watertight solid the
// engine keeps native (the host stub has no guided sweep). A degenerate/self-folding
// guide would fail the self-verify and fall through (stub → 0), never faked.
CC_TEST(native_guided_sweep_runs_native) {
    EngineGuard g;
    cc_set_engine(1);

    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double path[] = {0, 0, 0, 0, 0, 10};
    const double guide[] = {3, 0, 0, 6, 0, 10};  // splays the section along the sweep
    const CCShapeId id = cc_guided_sweep(prof, 4, path, 2, guide, 2);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }
    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(mp.volume > 0.0);
    cc_shape_release(id);
}

// Tier-2#4 (widened envelope): loft-along-a-rail runs NATIVE for a STRAIGHT rail (a
// ruled loft between the two equal-count sections placed perpendicular to the rail
// tangent) AND for a SMOOTH CURVED rail with a well-sampled section (an RMF-transported
// morph densified to a bounded per-band turn — a watertight tube converging to the
// Pappus torus-sector volume). A SHARP-CORNERED section along a curved rail, or a
// tight-kinked rail, does not weld robustly → engine self-verify declines → OCCT (stub →
// 0 on host). Mismatched section counts → NULL → fallback. All verified below.
CC_TEST(native_loft_along_rail_runs_native) {
    EngineGuard g;
    cc_set_engine(1);

    const double rail[] = {0, 0, 0, 0, 0, 10};
    const double a[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double b[] = {-1, -1, 1, -1, 1, 1, -1, 1};
    const CCShapeId id = cc_loft_along_rail(rail, 2, a, 4, b, 4);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }
    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);
    CC_CHECK(mp.volume > 0.0);
    cc_shape_release(id);

    // A SMOOTH curved (quarter-arc) rail with a well-sampled (32-gon) section runs NATIVE:
    // the RMF-transported tube welds watertight and converges to Pappus polyArea·R·φ.
    const double R = 20.0, phi = kPi / 2.0;
    std::vector<double> arc;
    for (int k = 0; k < 24; ++k) {
        const double th = phi * k / 23;
        arc.push_back(R * std::cos(th));
        arc.push_back(R * std::sin(th));
        arc.push_back(0.0);
    }
    std::vector<double> circ;
    const int pn = 32;
    const double rp = 3.0;
    for (int i = 0; i < pn; ++i) {
        const double t = 2.0 * kPi * i / pn;
        circ.push_back(rp * std::cos(t));
        circ.push_back(rp * std::sin(t));
    }
    const CCShapeId cid = cc_loft_along_rail(arc.data(), 24, circ.data(), pn, circ.data(), pn);
    CC_CHECK(cid != 0);
    if (cid != 0) {
        const CCMassProps cm = cc_mass_properties(cid);
        const double polyArea = 0.5 * pn * rp * rp * std::sin(2.0 * kPi / pn);
        CC_CHECK(cm.valid != 0 && cm.volume > 0.0);
        CC_CHECK(std::fabs(cm.volume - polyArea * R * phi) / (polyArea * R * phi) < 2e-2);
        cc_shape_release(cid);
    }

    // A tight-kinked rail (a near-90° V) does not weld robustly → declines → stub 0.
    const double bentRail[] = {0, 0, 0, 3, 0, 5, 0, 0, 10};
    CC_CHECK_EQ(cc_loft_along_rail(bentRail, 3, a, 4, b, 4), 0);
    // Mismatched section counts → NULL → fallback.
    const double tri[] = {0, 0, 2, 0, 1, 2};
    CC_CHECK_EQ(cc_loft_along_rail(rail, 2, a, 4, tri, 3), 0);
}

// ── Tier-D (#4b): native tapered shank + thread fall-through ───────────────────

// Native cc_tapered_shank: r=5, fullHeight=20, taperHeight=10, ppm=1 → a silhouette
// revolved 360° about Z (cone tip + full-radius cylinder + head disk). Served
// NATIVELY (the host stub has no shank op, so a non-zero id can only come from the
// native revolve-based builder); the mass properties come from the native watertight
// mesh. Volume = cone ⅓π·25·10 + cylinder π·25·20 ≈ 1832.6.
CC_TEST(native_tapered_shank_watertight_volume) {
    EngineGuard g;
    cc_set_engine(1);

    const CCShapeId id = cc_tapered_shank(5.0, 20.0, 10.0, 1.0);
    CC_CHECK(id != 0);
    if (id == 0) { std::printf("  last_error=%s\n", cc_last_error()); return; }

    const CCMassProps mp = cc_mass_properties(id);
    CC_CHECK(mp.valid != 0);  // valid ⇒ the native mesh is watertight
    const double exact = kPi * 25.0 * 10.0 / 3.0 + kPi * 25.0 * 20.0;  // ≈ 1832.6
    CC_CHECK(std::fabs(mp.volume - exact) / exact < 3e-2);

    cc_shape_release(id);
}

// Degenerate shank parameters: the native builder returns NULL, the engine forwards
// to the fallback (stub on host → 0).
CC_TEST(native_tapered_shank_degenerate_falls_through) {
    EngineGuard g;
    cc_set_engine(1);
    CC_CHECK_EQ(cc_tapered_shank(0.0, 20.0, 10.0, 1.0), 0);  // r ≤ 0 → NULL → stub → 0
}

// Native helical_thread / tapered_thread: the radial-V tiling now welds ROBUSTLY
// watertight (the per-turn ruled-band ↔ band / band ↔ cap seams weld via the mesher's
// canonical shared-edge points — edge_mesher CanonicalEndpoints / face_mesher
// BoundaryAnchors), so the engine's robustlyWatertight self-verify PASSES and the op runs
// NATIVELY. On host there is no OCCT, so a non-zero id can ONLY come from the native
// builder (the stub has no thread op → 0); a valid mass-properties result confirms the
// native mesh is watertight (mass_properties requires a closed mesh). This closes the
// Tier-D DEFERRED thread residual — the well-formed thread is now native, not OCCT-fallen.
CC_TEST(native_thread_runs_native_watertight) {
    EngineGuard g;
    cc_set_engine(1);
    CC_CHECK_EQ(cc_active_engine(), 1);

    // Well-formed thread params (the reference helical: major5 / pitch2 / turns4 /
    // depth1). The native builder welds a robustly-watertight solid → self-verify passes
    // → the engine keeps it native (non-zero id on host, where the fallback is the stub).
    const CCShapeId h = cc_helical_thread(5.0, 2.0, 4.0, 1.0, 60.0, 1.0, 16);
    CC_CHECK(h != 0);
    if (h == 0) std::printf("  last_error=%s\n", cc_last_error());
    if (h != 0) {
        const CCMassProps mp = cc_mass_properties(h);
        CC_CHECK(mp.valid != 0);       // valid ⇒ the native thread mesh is watertight
        CC_CHECK(mp.volume > 0.0);     // a real, positively-enclosed solid
        cc_shape_release(h);
    }

    const CCShapeId t = cc_tapered_thread(6.0, 4.0, 2.0, 3.0, 1.0, 60.0, 1.0, 16);
    CC_CHECK(t != 0);
    if (t == 0) std::printf("  last_error=%s\n", cc_last_error());
    if (t != 0) {
        const CCMassProps mp = cc_mass_properties(t);
        CC_CHECK(mp.valid != 0);
        CC_CHECK(mp.volume > 0.0);
        cc_shape_release(t);
    }
}

// A GENUINELY self-intersecting thread (radial-V flanks that cross in 3D at a STEEP
// helix lead) still fails the native build: the fine-pitch resolver's root flat cannot
// un-fold a true self-intersection, and thread.h's lead-ratio guard returns a NULL Shape
// (pitch/(2π·pitchR) > kMaxLeadRatio → Tier-4 surface-surface territory), so the engine
// forwards to the fallback (stub on host → 0). This is the unchanged honest guard — no
// faked or leaky native thread ships. On the iOS sim (OCCT linked) the same call returns
// the OCCT MakePipeShell thread.
//
// NOTE: a DEEP-SPIKE fine-pitch thread (major2/pitch0.2/depth3, depth/pitch = 15) also
// defers — its native pure-radial V volume diverges from OCCT's MakePipeShell swept solid
// by ~11% (a real native-vs-oracle mismatch the watertight self-verify cannot see), so
// thread.h's depth/pitch guard (depth > kMaxDepthOverPitch·pitch) returns NULL (see
// test_native_thread deep_spike_fine_pitch_thread_deferred). The genuine steep-lead fold
// used here is a large pitch on a small pitch radius:
//   major1/pitch3/depth0.4 ⇒ pitchR 0.8, lead ~31° → the flanks self-intersect → NULL.
CC_TEST(native_fine_pitch_thread_falls_through_to_default) {
    EngineGuard g;
    cc_set_engine(1);
    CC_CHECK_EQ(cc_active_engine(), 1);
    // Steep-lead fold → thread.h defers (NULL) → engine forwards to the stub (0 on host).
    CC_CHECK_EQ(cc_helical_thread(1.0, 3.0, 3.0, 0.4, 60.0, 1.0, 16), 0);
}

// ── NATIVE boolean (Phase 4 #5) through the cc_boolean facade ────────────────────
// Two native prisms (planar polyhedra) fused / cut / commoned NATIVELY, self-verified
// watertight with the exact set-algebra volume. Two 10-cubes overlapping by 5 on x+y:
// overlap = 5×5×10 = 250 ⇒ fuse 1750, cut 750, common 250.
CC_TEST(native_boolean_fuse_cut_common_exact) {
    EngineGuard g;
    cc_set_engine(1);
    CC_CHECK_EQ(cc_active_engine(), 1);

    const double a[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const double b[] = {5, 5, 15, 5, 15, 15, 5, 15};
    const CCShapeId A = cc_solid_extrude(a, 4, 10.0);
    const CCShapeId B = cc_solid_extrude(b, 4, 10.0);
    CC_CHECK(A != 0 && B != 0);

    const CCShapeId fuse = cc_boolean(A, B, 0);
    CC_CHECK(fuse != 0);
    if (fuse == 0) std::printf("  last_error=%s\n", cc_last_error());
    const CCMassProps mf = cc_mass_properties(fuse);
    CC_CHECK(mf.valid != 0);
    CC_CHECK(std::fabs(mf.volume - 1750.0) < 1e-3);

    const CCShapeId cut = cc_boolean(A, B, 1);
    CC_CHECK(cut != 0);
    const CCMassProps mc = cc_mass_properties(cut);
    CC_CHECK(mc.valid != 0);
    CC_CHECK(std::fabs(mc.volume - 750.0) < 1e-3);

    const CCShapeId common = cc_boolean(A, B, 2);
    CC_CHECK(common != 0);
    const CCMassProps mm = cc_mass_properties(common);
    CC_CHECK(mm.valid != 0);
    CC_CHECK(std::fabs(mm.volume - 250.0) < 1e-3);

    cc_shape_release(A);
    cc_shape_release(B);
    cc_shape_release(fuse);
    cc_shape_release(cut);
    cc_shape_release(common);
}

// A boolean whose operands have a CURVED face is outside the native planar domain.
// Both operands are native voids OCCT cannot read, so the engine returns 0 with a
// clean error (never a faked/leaky solid). On the sim (OCCT linked) the SAME cc_*
// call would build both bodies under OCCT and the OCCT boolean would run — this host
// assertion is specifically the native-void honest-error path.
CC_TEST(native_boolean_curved_operand_errors_not_faked) {
    EngineGuard g;
    cc_set_engine(1);

    const double box[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId A = cc_solid_extrude(box, 4, 10.0);
    // A line-segment profile revolved 2π → a cylinder (curved faces), built native.
    const double rect[] = {0, 0, 3, 0, 3, 5, 0, 5};
    const CCShapeId cyl = cc_solid_revolve(rect, 4, 2.0 * kPi);
    CC_CHECK(A != 0 && cyl != 0);

    CC_CHECK_EQ(cc_boolean(A, cyl, 0), 0);  // curved → no verified native result → 0
    CC_CHECK(std::strlen(cc_last_error()) > 0);

    cc_shape_release(A);
    cc_shape_release(cyl);
}

// ── NATIVE curved slice (Phase 4 #5 residual): the ENGINE's analytic guard engages
// for an axis-aligned box ⟷ axis-parallel cylinder and DELEGATES honestly when the
// configuration leaves the analytic family — never faking a curved result.
//
// The full positive path (round THROUGH hole / disc segment / boss, self-verified to
// the analytic boxVol ± πr²·len) is exercised at the LIBRARY level in
// test_native_boolean (box_cylinder_cut_round_through_hole / _common_segment /
// _fuse_boss), where a body can be positioned to straddle the cylinder's z=0 centre
// line. Through the FACADE, cc_solid_revolve revolves only about world Y (cylinder
// centre z=0) and cc_solid_extrude yields z∈[0,depth], so a facade-built box cannot
// straddle z=0 and the Y-cylinder always radially breaches the box — which the guard
// must reject with an honest error (both operands are native voids OCCT cannot read),
// exactly as for any out-of-family curved case.
CC_TEST(native_box_cylinder_out_of_family_errors_not_faked) {
    EngineGuard g;
    cc_set_engine(1);
    CC_CHECK_EQ(cc_active_engine(), 1);

    // Cylinder axis Y, r=2, centre (x=0,z=0), y∈[0,10].
    const double rect[] = {0, 0, 2, 0, 2, 10, 0, 10};
    const CCShapeId cyl = cc_solid_revolve(rect, 4, 2.0 * kPi);
    CC_CHECK(cyl != 0);

    // Box x[−5,5] y[2,8] z[0,10]: the cylinder (centre z=0, r=2 → z∈[−2,2]) breaches
    // the box's z=0 face, so it is NOT radially inside → out of the analytic family →
    // honest 0 + error, not a faked leaky curved cut.
    const double bp[] = {-5, 2, 5, 2, 5, 8, -5, 8};
    const CCShapeId box = cc_solid_extrude(bp, 4, 10.0);
    CC_CHECK(box != 0);

    CC_CHECK_EQ(cc_boolean(box, cyl, 1), 0);  // z-breach → not in family → honest 0
    CC_CHECK(std::strlen(cc_last_error()) > 0);

    cc_shape_release(cyl);
    cc_shape_release(box);
}

// ── Phase 4 #6 native-blends through the cc_* facade ──────────────────────────────
// Each op runs NATIVE on a native box body under cc_set_engine(1), self-verifies
// (watertight + sane volume sign), and DISCARDS a bad result (→ 0 + error on the
// OCCT-free host, since a native void cannot be forwarded to OCCT). Every box edge is
// a convex 90° planar dihedral, so a single-edge chamfer/fillet always lands native.

// Chamfer one convex box edge: distance 2 on a 10×10×10 box removes a
// ½·2·2·10 = 20 corner prism → volume 980, watertight.
CC_TEST(native_chamfer_box_edge_volume_reduced) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(box != 0);
    // Chamfer each edge id until one lands native (all 12 box edges are convex, so
    // id 1 succeeds); assert the native result is watertight and smaller.
    const int edges[] = {1};
    const CCShapeId ch = cc_chamfer_edges(box, edges, 1, 2.0);
    CC_CHECK(ch != 0);
    if (ch == 0) { std::printf("  last_error=%s\n", cc_last_error()); }
    else {
        const CCMassProps mp = cc_mass_properties(ch);
        CC_CHECK(mp.valid != 0);
        CC_CHECK(mp.volume < 1000.0 && mp.volume > 900.0);  // one corner chamfered
        cc_shape_release(ch);
    }
    cc_shape_release(box);
}

// Fillet one convex box edge with r=2: watertight, volume reduced but LESS than the
// chamfer (the fillet keeps the quarter-cylinder material).
CC_TEST(native_fillet_box_edge_watertight) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(box != 0);
    const int edges[] = {1};
    const CCShapeId f = cc_fillet_edges(box, edges, 1, 2.0);
    CC_CHECK(f != 0);
    if (f == 0) { std::printf("  last_error=%s\n", cc_last_error()); }
    else {
        const CCMassProps mp = cc_mass_properties(f);
        CC_CHECK(mp.valid != 0);
        CC_CHECK(mp.volume < 1000.0 && mp.volume > 985.0);  // < sharp, > chamfer
        cc_shape_release(f);
    }
    cc_shape_release(box);
}

// Offset one box face outward by 5: a 10×10×10 box grows to 10×10×15 = 1500 for the
// cap face (id chosen by scanning; a side face grows the corresponding extent). We
// assert the native result is watertight and LARGER than the original.
CC_TEST(native_offset_face_grows) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(box != 0);
    // Scan face ids for one whose outward offset self-verifies as a grow.
    bool grew = false;
    for (int fid = 1; fid <= 6 && !grew; ++fid) {
        const CCShapeId gsh = cc_offset_face(box, fid, 5.0);
        if (gsh == 0) continue;
        const CCMassProps mp = cc_mass_properties(gsh);
        if (mp.valid && mp.volume > 1000.0 + 1e-6) {
            grew = true;
            CC_CHECK(std::fabs(mp.volume - 1500.0) < 1e-2);
        }
        cc_shape_release(gsh);
    }
    CC_CHECK(grew);
    cc_shape_release(box);
}

// Shell a box open on one face, thickness 1: wall volume 1000 − 8×8×9 = 424 when the
// open face is a cap; scan face ids and assert one gives a watertight wall < original.
CC_TEST(native_shell_box_wall) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(box != 0);
    bool walled = false;
    for (int fid = 1; fid <= 6 && !walled; ++fid) {
        const int faces[] = {fid};
        const CCShapeId sh = cc_shell(box, faces, 1, 1.0);
        if (sh == 0) continue;
        const CCMassProps mp = cc_mass_properties(sh);
        if (mp.valid && mp.volume > 0.0 && mp.volume < 1000.0) {
            walled = true;
            CC_CHECK(std::fabs(mp.volume - 424.0) < 1.0);  // one open cap
        }
        cc_shape_release(sh);
    }
    CC_CHECK(walled);
    cc_shape_release(box);
}

// REGRESSION (Phase 4 #6 sim-parity): a NATIVE body's edges must be QUERYABLE through
// cc_edge_polylines so an app/harness can resolve an edge id to chamfer/fillet. The
// native engine used to refuse edge_polylines on a native body (returned an error), so
// cc_edge_polylines gave 0 edges and any edge-id resolver (as the sim blend harness
// does via findAxisEdge) got id 0 → cc_chamfer_edges/cc_fillet_edges returned 0. This
// guards the native edge_polylines path: every native edge is returned as a 2-point
// straight line with a 1-based id, AND resolving one edge geometrically then chamfering
// it lands a valid native result — the exact end-to-end path the sim harness exercises.
// (The native builder emits per-face edges — vertex/edge SHARING is deferred — so a box
// yields 24 edge nodes, not OCCT's 12 shared edges; the id COUNT differs by that k=2
// factor while the GEOMETRY of every edge is exact. The test asserts the representation-
// tolerant invariants, not a hard 12.)
CC_TEST(native_edge_polylines_and_resolved_chamfer) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(box != 0);

    CCEdgePolyline* edges = nullptr;
    const int n = cc_edge_polylines(box, &edges);
    CC_CHECK(n > 0);              // native body edges MUST be queryable (the bug)
    CC_CHECK(edges != nullptr);

    bool idsOk = true, lineOk = true, lenOk = true;
    // Resolve the vertical edge at x=0,y=0 (the sim harness's chamfer/fillet pick).
    int vertEdge = 0;
    for (int i = 0; i < n && edges != nullptr; ++i) {
        const CCEdgePolyline& e = edges[i];
        if (e.edgeId != i + 1) idsOk = false;         // ids are 1..n in order
        if (e.pointCount != 2 || e.points == nullptr) { lineOk = false; continue; }
        const double* a = &e.points[0];
        const double* b = &e.points[3];
        const double len = std::sqrt((b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]) +
                                     (b[2]-a[2])*(b[2]-a[2]));
        if (std::fabs(len - 10.0) > 1e-6) lenOk = false;  // every box edge is 10 mm
        const bool onX0Y0 = std::fabs(a[0]) < 1e-6 && std::fabs(a[1]) < 1e-6 &&
                            std::fabs(b[0]) < 1e-6 && std::fabs(b[1]) < 1e-6;
        if (onX0Y0) vertEdge = e.edgeId;
    }
    CC_CHECK(idsOk);
    CC_CHECK(lineOk);
    CC_CHECK(lenOk);
    CC_CHECK(vertEdge != 0);
    if (edges) cc_edge_polylines_free(edges, n);

    // The end-to-end path: chamfer the RESOLVED edge → a valid, smaller native solid.
    const int ids[] = {vertEdge};
    const CCShapeId ch = cc_chamfer_edges(box, ids, 1, 1.0);
    CC_CHECK(ch != 0);
    if (ch == 0) { std::printf("  last_error=%s\n", cc_last_error()); }
    else {
        const CCMassProps mp = cc_mass_properties(ch);
        CC_CHECK(mp.valid != 0);
        CC_CHECK(std::fabs(mp.volume - 995.0) < 1e-3);  // 1000 − ½·1·1·10
        cc_shape_release(ch);
    }
    cc_shape_release(box);
}

// Variable-radius fillet is out of the native domain → honest error on a native body
// (never faked, never a native void handed to OCCT).
CC_TEST(native_variable_fillet_defers) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    const int edges[] = {1};
    CC_CHECK_EQ(cc_fillet_edges_variable(box, edges, 1, 1.0, 3.0), 0);
    CC_CHECK(std::strlen(cc_last_error()) > 0);
    cc_shape_release(box);
}

// ── Native STEP export (Phase 4 #7) through the facade ──────────────────────────
// A native-built solid whose geometry is in the writer's scope (a box → planar
// faces + line edges) exports NATIVELY: cc_step_export returns 1 and writes a valid
// ISO-10303-21 AP203 file (magic header + MANIFOLD_SOLID_BREP + mm SI_UNIT). The
// true correctness gate (re-read through OCCT to the same solid) runs on the sim;
// here we assert the wired native path produces a structurally-valid file, no OCCT.
CC_TEST(native_step_export_writes_valid_ap203_file) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(box != 0);

    const char* path = "/tmp/cybercad_native_step_export_test.step";
    std::remove(path);
    CC_CHECK_EQ(cc_step_export(box, path), 1);  // native success

    // Read it back as text and confirm the AP203 framing + key entities.
    std::FILE* f = std::fopen(path, "rb");
    CC_CHECK(f != nullptr);
    std::string content;
    if (f) {
        char buf[4096];
        std::size_t n;
        while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) content.append(buf, n);
        std::fclose(f);
    }
    CC_CHECK(content.rfind("ISO-10303-21;", 0) == 0);
    CC_CHECK(content.find("MANIFOLD_SOLID_BREP(") != std::string::npos);
    CC_CHECK(content.find("SI_UNIT(.MILLI.,.METRE.)") != std::string::npos);
    CC_CHECK(content.find("END-ISO-10303-21;") != std::string::npos);
    std::remove(path);
    cc_shape_release(box);
}

// Facade round-trip through the SHIPPING path: under the native engine, cc_step_export
// writes a native STEP, then cc_step_import reads it BACK through the native reader (the
// first import slice). The reconstructed body is a valid native solid whose mass matches
// the original EXACTLY (planar box). Host build has no OCCT, so this only passes if the
// NATIVE reader path ran (a decline would fall to the OCCT stub → 0).
CC_TEST(native_step_import_reads_native_file) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(box != 0);
    const CCMassProps src = cc_mass_properties(box);
    CC_CHECK(src.valid != 0);

    const char* path = "/tmp/cybercad_native_step_import_test.step";
    std::remove(path);
    CC_CHECK_EQ(cc_step_export(box, path), 1);

    const CCShapeId back = cc_step_import(path);
    CC_CHECK(back != 0);  // native reader reconstructed a valid watertight solid
    if (back != 0) {
        const CCMassProps rr = cc_mass_properties(back);
        CC_CHECK(rr.valid != 0);
        CC_CHECK(std::fabs(rr.volume - src.volume) < 1e-6);  // EXACT (planar box)
        CC_CHECK(std::fabs(rr.area - src.area) < 1e-6);
        CC_CHECK(std::fabs(rr.cx - src.cx) < 1e-6 && std::fabs(rr.cy - src.cy) < 1e-6 &&
                 std::fabs(rr.cz - src.cz) < 1e-6);
        cc_shape_release(back);
    }
    std::remove(path);
    cc_shape_release(box);
}

// ── ADDITIVE ABI: cc_step_pmi_scan recognises/classifies/counts PMI, and the
// geometry import of the SAME file stays byte-identical ────────────────────────
// Under the native engine, export a box to STEP, inject a KNOWN PMI block, then:
//   (1) cc_step_pmi_scan fills CCPmiSummary with the exact census; and
//   (2) cc_step_import of the PMI-bearing file imports the SAME solid as the
//       PMI-free file (mass properties bit-for-bit) — the scan is additive and the
//       geometry import is untouched.
CC_TEST(native_step_pmi_scan_census_and_byte_identical_import) {
    EngineGuard g;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(box != 0);

    const char* plainPath = "/tmp/cybercad_pmi_plain.step";
    const char* pmiPath = "/tmp/cybercad_pmi_augmented.step";
    std::remove(plainPath);
    std::remove(pmiPath);
    CC_CHECK_EQ(cc_step_export(box, plainPath), 1);

    // Read the exported STEP and inject a known PMI block (2 dims, 2 tols, 1 datum,
    // 1 datum target, 1 note, 1 annotation geometry, 1 unknown = total 9) before
    // ENDSEC; — the injected entities are never reached from the solid's brep.
    std::string step;
    {
        std::ifstream in(plainPath, std::ios::binary);
        std::ostringstream ss;
        ss << in.rdbuf();
        step = ss.str();
    }
    const std::string pmi =
        "#900001 = SHAPE_ASPECT('feature','',#900000,.T.);\n"
        "#900002 = DATUM_FEATURE('df','',#900000,.T.);\n"
        "#900010 = DIMENSIONAL_SIZE(#900001,'diameter');\n"
        "#900011 = DIMENSIONAL_LOCATION('loc','',#900001,#900002);\n"
        "#900020 = FLATNESS_TOLERANCE('','',#900099,#900001);\n"
        "#900021 = ( GEOMETRIC_TOLERANCE('','',#900099,#900001) FLATNESS_TOLERANCE() );\n"
        "#900030 = DATUM('','',#900070,.T.,'A');\n"
        "#900031 = DATUM_TARGET('P1','1',#900001,.T.,$);\n"
        "#900040 = DRAUGHTING_CALLOUT((#900041));\n"
        "#900050 = ANNOTATION_PLANE('',(#900051),#900052);\n"
        "#900060 = TOLERANCE_ZONE('',(#900061),#900062,#900063);\n";
    const std::size_t data = step.find("DATA;");
    const std::size_t end = step.find("ENDSEC;", data);
    CC_CHECK(end != std::string::npos);
    const std::string augmented = step.substr(0, end) + pmi + step.substr(end);
    {
        std::ofstream out(pmiPath, std::ios::binary);
        out << augmented;
    }

    // (1) PMI census via the facade.
    CCPmiSummary sum;
    std::memset(&sum, 0, sizeof(sum));
    CC_CHECK_EQ(cc_step_pmi_scan(pmiPath, &sum), 1);
    CC_CHECK_EQ(sum.dimensions, 2);
    CC_CHECK_EQ(sum.tolerances, 2);
    CC_CHECK_EQ(sum.datums, 1);
    CC_CHECK_EQ(sum.datum_targets, 1);
    CC_CHECK_EQ(sum.notes, 1);
    CC_CHECK_EQ(sum.annotation_geometry, 1);
    CC_CHECK_EQ(sum.unknown, 1);
    CC_CHECK_EQ(sum.total, 9);

    // (2) Byte-identical geometry: importing the PMI-bearing file yields the SAME
    // solid as the PMI-free file.
    const CCShapeId plain = cc_step_import(plainPath);
    const CCShapeId withPmi = cc_step_import(pmiPath);
    CC_CHECK(plain != 0);
    CC_CHECK(withPmi != 0);
    if (plain != 0 && withPmi != 0) {
        const CCMassProps a = cc_mass_properties(plain);
        const CCMassProps b = cc_mass_properties(withPmi);
        CC_CHECK(a.valid != 0 && b.valid != 0);
        CC_CHECK(std::fabs(a.volume - b.volume) < 1e-9);
        CC_CHECK(std::fabs(a.area - b.area) < 1e-9);
        CC_CHECK(std::fabs(a.cx - b.cx) < 1e-9 && std::fabs(a.cy - b.cy) < 1e-9 &&
                 std::fabs(a.cz - b.cz) < 1e-9);
    }
    if (plain != 0) cc_shape_release(plain);
    if (withPmi != 0) cc_shape_release(withPmi);

    // Failure paths: null out, and an unreadable path → 0 with an error set.
    CC_CHECK_EQ(cc_step_pmi_scan(pmiPath, nullptr), 0);
    CCPmiSummary junk;
    CC_CHECK_EQ(cc_step_pmi_scan("/no/such/pmi/file.step", &junk), 0);
    CC_CHECK(std::strcmp(cc_last_error(), "") != 0);

    std::remove(plainPath);
    std::remove(pmiPath);
    cc_shape_release(box);
}

// ── M-TX: NATIVE affine transforms on a native body (Gate a — host, analytic) ──
// A 10×10×10 box built natively (cc_solid_extrude) is transformed by each op and
// measured OCCT-FREE against the SAME closed-form invariants the OCCT runtime suite
// asserts in tests/sim/checks_booltransform.cpp: volume = |det L|·vol, bbox/centroid
// are the exact affine image, mass_properties.valid ⇒ the result is a watertight
// positive-|vol| solid, and every op's degenerate-input guard declines. These used
// to hard-error (CC_NATIVE_BODY_UNSUPPORTED) on a native body.
namespace {
constexpr double kBox10[8] = {0, 0, 10, 0, 10, 10, 0, 10};
CCShapeId nativeBox10() { return cc_solid_extrude(kBox10, 4, 10.0); }
bool bboxIs(CCShapeId id, double x0, double y0, double z0, double x1, double y1, double z1,
            double t = 1e-6) {
    double b[6] = {0};
    if (cc_bounding_box(id, b) != 1) return false;
    return std::fabs(b[0] - x0) < t && std::fabs(b[1] - y0) < t && std::fabs(b[2] - z0) < t &&
           std::fabs(b[3] - x1) < t && std::fabs(b[4] - y1) < t && std::fabs(b[5] - z1) < t;
}
}  // namespace

CC_TEST(native_translate_shape) {
    EngineGuard g;
    cc_set_engine(1);
    CCShapeId box = nativeBox10();
    CC_CHECK(box != 0);
    CCShapeId t = cc_translate_shape(box, 5, 5, 5);
    CC_CHECK(t != 0);
    const CCMassProps mp = cc_mass_properties(t);
    CC_CHECK(mp.valid != 0 && std::fabs(mp.volume - 1000.0) < 1e-4);
    CC_CHECK(std::fabs(mp.cx - 10) < 1e-3 && std::fabs(mp.cy - 10) < 1e-3 &&
             std::fabs(mp.cz - 10) < 1e-3);
    CC_CHECK(bboxIs(t, 5, 5, 5, 15, 15, 15));
    CC_CHECK_EQ(cc_translate_shape(999999, 1, 2, 3), 0);  // guard: unknown id
    cc_shape_release(t);
    cc_shape_release(box);
}

CC_TEST(native_rotate_shape_about) {
    EngineGuard g;
    cc_set_engine(1);
    CCShapeId box = nativeBox10();
    const double kHalfPi = 1.57079632679489661923;
    // 90° about +Z through origin: (x,y)->(-y,x) ⇒ bbox [-10,0,0 .. 0,10,10].
    CCShapeId r = cc_rotate_shape_about(box, 0, 0, 0, 0, 0, 1, kHalfPi);
    CC_CHECK(r != 0);
    const CCMassProps mp = cc_mass_properties(r);
    CC_CHECK(mp.valid != 0 && std::fabs(mp.volume - 1000.0) < 1e-4);  // rigid preserves vol
    CC_CHECK(bboxIs(r, -10, 0, 0, 0, 10, 10, 1e-4));
    CC_CHECK_EQ(cc_rotate_shape_about(box, 0, 0, 0, 0, 0, 0, kHalfPi), 0);  // guard: zero axis
    cc_shape_release(r);
    cc_shape_release(box);
}

CC_TEST(native_mirror_shape) {
    EngineGuard g;
    cc_set_engine(1);
    CCShapeId box = nativeBox10();
    // Mirror across x=0: reflection preserves |vol| and reflects x∈[0,10]→[-10,0]. The
    // result must stay a VALID watertight positive-|vol| solid (mp.valid ⇒ watertight
    // mesh with positive volume — the mirror handedness flip is proven parity-side in
    // native_transform_fuzz.mm; the ABI reports |volume|).
    CCShapeId m = cc_mirror_shape(box, 0, 0, 0, 1, 0, 0);
    CC_CHECK(m != 0);
    const CCMassProps mp = cc_mass_properties(m);
    CC_CHECK(mp.valid != 0 && std::fabs(mp.volume - 1000.0) < 1e-4);
    CC_CHECK(bboxIs(m, -10, 0, 0, 0, 10, 10));
    CC_CHECK_EQ(cc_mirror_shape(box, 0, 0, 0, 0, 0, 0), 0);  // guard: zero normal
    cc_shape_release(m);
    cc_shape_release(box);
}

CC_TEST(native_scale_shape) {
    EngineGuard g;
    cc_set_engine(1);
    CCShapeId box = nativeBox10();
    CCShapeId sc = cc_scale_shape(box, 2.0);
    CC_CHECK(sc != 0);
    const CCMassProps mp = cc_mass_properties(sc);
    CC_CHECK(mp.valid != 0 && std::fabs(mp.volume - 8000.0) < 1e-2);  // 1000·2³
    CC_CHECK(bboxIs(sc, 0, 0, 0, 20, 20, 20));
    // Honest decline of a zero/degenerate (non-invertible) scale, and non-positive.
    CC_CHECK_EQ(cc_scale_shape(box, 0.0), 0);
    CC_CHECK_EQ(cc_scale_shape(box, -1.0), 0);
    cc_shape_release(sc);
    cc_shape_release(box);
}

CC_TEST(native_scale_shape_about) {
    EngineGuard g;
    cc_set_engine(1);
    CCShapeId box = nativeBox10();
    CCShapeId sc = cc_scale_shape_about(box, 5, 5, 5, 2.0);
    CC_CHECK(sc != 0);
    const CCMassProps mp = cc_mass_properties(sc);
    CC_CHECK(mp.valid != 0 && std::fabs(mp.volume - 8000.0) < 1e-2);
    CC_CHECK(std::fabs(mp.cx - 5) < 1e-3 && std::fabs(mp.cy - 5) < 1e-3 &&
             std::fabs(mp.cz - 5) < 1e-3);  // scale centre is a fixed point
    CC_CHECK(bboxIs(sc, -5, -5, -5, 15, 15, 15));
    CC_CHECK_EQ(cc_scale_shape_about(box, 5, 5, 5, 0.0), 0);  // degenerate scale honest decline
    cc_shape_release(sc);
    cc_shape_release(box);
}

CC_TEST(native_place_on_frame) {
    EngineGuard g;
    cc_set_engine(1);
    CCShapeId box = nativeBox10();
    // Frame o=(10,0,0), u=+Y, v=+Z ⇒ n=u×v=+X. Local (x,y,z) → (10+z, x, y), so the
    // box maps to X∈[10,20], Y∈[0,10], Z∈[0,10] with volume preserved.
    CCShapeId p = cc_place_on_frame(box, 10, 0, 0, 0, 1, 0, 0, 0, 1);
    CC_CHECK(p != 0);
    const CCMassProps mp = cc_mass_properties(p);
    CC_CHECK(mp.valid != 0 && std::fabs(mp.volume - 1000.0) < 1e-4);
    CC_CHECK(bboxIs(p, 10, 0, 0, 20, 10, 10, 1e-4));
    CC_CHECK_EQ(cc_place_on_frame(box, 0, 0, 0, 0, 1, 0, 0, 1, 0), 0);  // guard: u ∥ v
    cc_shape_release(p);
    cc_shape_release(box);
}

// ── M-TX: cc_extrude (legacy mesh) rewired to ATTEMPT NATIVE FIRST ─────────────
// The native prism is the same build_prism solid_extrude uses (bbox-identical to the
// OCCT adapter's prism), so cc_extrude of a handled profile now builds natively; a
// degenerate profile falls through (stub on host → empty mesh), never faking.
CC_TEST(native_extrude_mesh_bbox) {
    EngineGuard g;
    cc_set_engine(1);
    CCMesh m = cc_extrude(kBox10, 4, 10.0);
    CC_CHECK(m.vertexCount > 0);
    CC_CHECK(m.triangleCount >= 12);  // a box is at least 12 triangles
    double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
    for (int i = 0; i < m.vertexCount; ++i)
        for (int k = 0; k < 3; ++k) {
            const double c = m.vertices[i * 3 + k];
            lo[k] = std::min(lo[k], c);
            hi[k] = std::max(hi[k], c);
        }
    CC_CHECK(std::fabs(lo[0]) < 1e-6 && std::fabs(lo[1]) < 1e-6 && std::fabs(lo[2]) < 1e-6);
    CC_CHECK(std::fabs(hi[0] - 10) < 1e-6 && std::fabs(hi[1] - 10) < 1e-6 &&
             std::fabs(hi[2] - 10) < 1e-6);
    cc_mesh_free(m);

    // <3 points: native build_prism declines → fallback (stub on host) → empty mesh.
    CCMesh d = cc_extrude(kBox10, 2, 10.0);
    CC_CHECK_EQ(d.vertexCount, 0);
    cc_mesh_free(d);
}

CC_RUN_ALL()
