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
#include <cstring>

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

    // A capability with NO native implementation (loft) forwards to the fallback
    // (stub → 0 on host) without crashing.
    const double a[] = {0, 0, 1, 0, 1, 1};
    const double b[] = {0, 0, 2, 0, 2, 2};
    const CCShapeId loftId = cc_solid_loft(a, 3, b, 3, 1.0);
    CC_CHECK_EQ(loftId, 0);
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

CC_RUN_ALL()
