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

#include <cmath>
#include <cstdio>
#include <cstring>
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

// A DEFERRED typed sub-case (kind-3 spline outer edge) falls through: the native
// builder returns NULL, the engine forwards to the fallback (stub on host → 0). The
// native path never fakes an unsupported profile.
CC_TEST(native_profile_spline_falls_through) {
    EngineGuard g;
    cc_set_engine(1);

    CCProfileSeg seg{};
    seg.kind = 3;
    seg.ptOffset = 0; seg.ptCount = 3;
    const double spline[] = {0, 0, 1, 1, 2, 0};  // 3 points = 6 doubles
    const CCShapeId id = cc_solid_extrude_profile(&seg, 1, nullptr, 0, spline, 6, 2.0);
    CC_CHECK_EQ(id, 0);  // stub fallback on host

    // An off-axis arc revolve (a Torus) also defers to the fallback.
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

// Deferred: a TIGHT-CURVATURE / self-intersecting sweep spine forwards to the
// fallback (stub → 0 on host), never faked. Same for a real twist. Proves the native
// path guards the self-intersecting cases and defers cleanly.
CC_TEST(native_sweep_tight_and_twisted_defer) {
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

    const double path[] = {0, 0, 0, 0, 0, 10};
    CC_CHECK_EQ(cc_twisted_sweep(prof, 4, path, 2, 1.5708, 1.0), 0);  // real twist → fallback
}

CC_RUN_ALL()
