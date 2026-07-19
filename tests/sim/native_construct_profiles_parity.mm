// SPDX-License-Identifier: Apache-2.0
//
// native_construct_profiles_parity.mm — native-vs-OCCT parity for the Tier-A
//                                        PROFILE construction ops (Phase 4 #4b),
//                                        driven entirely THROUGH the cc_* facade
//                                        (iOS simulator).
//
// Phase 4 capability #4b (`native-construction`, Tier A), simulator verification
// gate 2 (see openspec/NATIVE-REWRITE.md). This is the profile-family sibling of
// native_construct_parity.mm (#4): it exercises the SAME shipping path — the public
// cc_* entry points, once with the OCCT engine active and once with the NativeEngine
// active — and asserts native == OCCT for the Tier-A ops:
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default)
//   cc_set_engine(1)  → NativeEngine (native Tier-A profile builders — profile.h;
//                       every other op falls through to OCCT)
//
// The switch is the ADDITIVE facade toggle cc_set_engine / cc_active_engine; DEFAULT
// stays OCCT so every existing suite is unchanged. This harness restores
// cc_set_engine(0) before exit.
//
// ── Ops compared (Tier-A native, identical inputs to BOTH engines) ────────────────
//   1. HOLED SQUARE       — cc_solid_extrude_holes: a 10×10 outer square with one
//      (circular hole)      circular THROUGH-hole (centre (5,5), r=2), depth 4 →
//                           volume (100 − 4π)·4. Native keeps a TRUE Circle cap edge
//                           + a Cylinder wall, so the hole is a curved feature ⇒
//                           DEFLECTION-BOUNDED comparison, watertight native mesh.
//   2. POLYGON-HOLE       — cc_solid_extrude_polyholes: a 10×10 outer square with a
//      (planar)             2×2 square hole, depth 3 → volume (100 − 4)·3 = 288. All
//                           planar ⇒ EXACT comparison, watertight native mesh.
//   3. TYPED LINE+ARC     — cc_solid_extrude_profile: a D-shape (a straight diameter
//      PROFILE (D-shape)    line edge + a semicircle arc edge, r=2) extruded depth 3
//                           → volume (πr²/2)·3 = 6π. The arc extrudes to a TRUE
//                           Cylinder wall (not a chord polyline) ⇒ deflection-bounded,
//                           watertight native mesh.
//   4. LINE-PROFILE       — cc_solid_revolve_profile: a rectangle [x∈[1,2],y∈[0,3]]
//      REVOLVE (tube)       typed as four line segments, revolved 360° about the Y
//                           axis → an annular tube, volume π(2²−1²)·3 = 9π. Curved
//                           surfaces of revolution ⇒ deflection-bounded, watertight.
//   5. ARC-PROFILE        — cc_solid_revolve_profile: an ON-AXIS semicircle arc
//      REVOLVE (sphere)     (kind 1, r=3, meridian from (0,−3) through (3,0) to
//                           (0,3)) revolved 360° about the Y axis → a SPHERE, volume
//                           (4/3)πR³ = 36π. Curved ⇒ deflection-bounded, watertight.
//
// All five are GENUINELY NATIVE in the NativeEngine (profile.h build_prism_with_holes
// / build_prism_profile / build_revolution_profile return a non-null native Shape for
// these inputs — verified by the host test_native_profile / test_native_engine gate),
// so cc_set_engine(1) exercises the native path, NOT the OCCT fallthrough. Each is
// tagged [native] in the output.
//
// ── Per-op comparisons (native vs OCCT, through the facade) ───────────────────────
//   * cc_mass_properties : volume + area + centroid match. EXACT fp64 tol for the
//     all-planar op (polygon-hole prism); DEFLECTION-scaled relative tol for the
//     curved ops (circular hole, D-shape arc wall, both revolves), because native
//     mass_properties on a native body is mesh-derived (native_engine.cpp
//     kPropertyDeflection) so a curved solid converges from below.
//   * cc_bounding_box    : the two AABBs match (exact for planar; deflection tol for
//     curved).
//   * cc_subshape_ids    : native FACE count vs the oracle. Faces are the topology
//     entity the two engines agree on for a matching tiling (the profile ops here do
//     not tile a full periodic surface into <π patches the way a full revolve of a
//     smooth generatrix does — see below), so we assert the native FACE count is a
//     positive integer MULTIPLE of the OCCT face count (k≥1): identical when the
//     tiling matches, k× when native splits a curved wall / band into <120° angular
//     patches (a documented representational difference, NOT a geometric mismatch —
//     periodic-face construction is deferred, openspec/NATIVE-REWRITE.md). VERTEX /
//     EDGE counts are NOT compared (native emits per-face edges / per-patch vertices;
//     the watertight mesh check is the authoritative topological-closure test).
//   * cc_tessellate      : the NATIVE mesh is WATERTIGHT (every undirected edge shared
//     by exactly two triangles) with a bbox and mesh-volume that match the native
//     B-rep. Required for EVERY op (the tessellator's multi-hole cap-fill welds the
//     hole rings watertight — uv_triangulate.h).
//
// ── Fallback assertions (intentionally-deferred sub-cases) ────────────────────────
//   The NativeEngine returns a NULL native Shape for a sub-case profile.h defers and
//   FORWARDS to OCCT (never fakes). We assert two such sub-cases still return a VALID
//   result under cc_set_engine(1) and LABEL them [fallback]:
//     B. OFF-AXIS closed-circle revolve (cc_solid_revolve_profile, a full-circle
//        generatrix centred off the axis → a TORUS surface of revolution, no native
//        Torus yet) — deferred.
//   Each is built once under OCCT (the oracle) and once under native; the native
//   result must equal the OCCT result (native transparently delegated), proving the
//   fallthrough path is live and honest.
//
//   ⚠ SUB-CASE A WAS RECLASSIFIED. The kind-3 spline extrude is NO LONGER deferred —
//   residuals.h now fits the NURBS and builds a true spline cap edge + spline-ruled
//   wall, measured E=12 vs OCCT's 6 and vol 45.5547 vs 45.600000 (rel 9.92e-04). The
//   old [fallback] contract asserted bit-identity (rel < 1e-9), which held only WHILE
//   native delegated; once native started building its own solid no tolerance could
//   rescue it. It is now case 6 in the native list, deflection-bounded like the other
//   curved ops. Only sub-case B (the torus revolve) is still a genuine deferral.
//
// Output: [NCPROF] PASS/FAIL lines with per-op deltas + a native/fallback tag, then
// "== N passed, M failed ==". Flushes stdout and std::_Exit (the trimmed static-OCCT
// build's static teardown is not exit-clean — same rationale as the sibling sim
// harnesses; every id here is released before exit).
//
// Build: scripts/run-sim-native-construct-profiles.sh — compiles this harness + the
// whole facade/core/engine (incl. NativeEngine + OCCT adapter) + src/native/**, links
// the OCCT libs, spawns on a booted simulator. On run-sim-suite.sh's SKIP list (its
// own main(), competes for the entry point).

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── result accounting ─────────────────────────────────────────────────────────
int g_passed = 0;
int g_failed = 0;

void record(bool ok, const char* label, const char* detail) {
    if (ok) {
        ++g_passed;
        std::printf("[NCPROF] PASS  %-34s %s\n", label, detail);
    } else {
        ++g_failed;
        std::printf("[NCPROF] FAIL  %-34s %s\n", label, detail);
    }
}

// ── restore the default (OCCT) engine no matter how we leave scope ──────────────
struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

// ── mesh-level helpers (operate on a CCMesh returned by cc_tessellate) ──────────

// A mesh is watertight iff it is a closed 2-manifold: every undirected edge is
// shared by exactly two triangles. Keyed on an ordered vertex-index pair.
bool meshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0) return false;
    std::unordered_map<std::uint64_t, int> edgeCount;
    auto key = [](int a, int b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = m.triangles[t * 3 + 0];
        const int j = m.triangles[t * 3 + 1];
        const int k = m.triangles[t * 3 + 2];
        ++edgeCount[key(i, j)];
        ++edgeCount[key(j, k)];
        ++edgeCount[key(k, i)];
    }
    for (const auto& [e, c] : edgeCount)
        if (c != 2) return false;
    return true;
}

// Axis-aligned bounding box of a mesh's vertices → out6 = [minXYZ, maxXYZ].
bool meshBBox(const CCMesh& m, double out6[6]) {
    if (m.vertexCount <= 0) return false;
    for (int k = 0; k < 3; ++k) {
        out6[k] = m.vertices[k];
        out6[3 + k] = m.vertices[k];
    }
    for (int v = 0; v < m.vertexCount; ++v)
        for (int k = 0; k < 3; ++k) {
            const double c = m.vertices[v * 3 + k];
            if (c < out6[k]) out6[k] = c;
            if (c > out6[3 + k]) out6[3 + k] = c;
        }
    return true;
}

// Signed volume of a closed triangle mesh via the divergence theorem (⅙ Σ
// aᵢ·(bᵢ×cᵢ)); magnitude is the enclosed volume for an outward-wound solid.
double meshVolume(const CCMesh& m) {
    double v6 = 0.0;
    for (int t = 0; t < m.triangleCount; ++t) {
        const double* A = &m.vertices[m.triangles[t * 3 + 0] * 3];
        const double* B = &m.vertices[m.triangles[t * 3 + 1] * 3];
        const double* C = &m.vertices[m.triangles[t * 3 + 2] * 3];
        v6 += A[0] * (B[1] * C[2] - B[2] * C[1]) - A[1] * (B[0] * C[2] - B[2] * C[0]) +
              A[2] * (B[0] * C[1] - B[1] * C[0]);
    }
    return std::fabs(v6) / 6.0;
}

double maxBBoxCornerDelta(const double a[6], const double b[6]) {
    double d = 0.0;
    for (int k = 0; k < 6; ++k) d = std::max(d, std::fabs(a[k] - b[k]));
    return d;
}

int subCount(CCShapeId id, int kind) {
    int* ids = nullptr;
    const int n = cc_subshape_ids(id, kind, &ids);
    cc_ints_free(ids);
    return n;
}

// ── one native-vs-OCCT op case, built via a caller-supplied builder ─────────────
// The builder is invoked TWICE (once per active engine) with identical inputs; the
// active engine is set by the caller before each call. This keeps the per-op input
// marshalling (holes / typed segs / axis) inline in main() where it reads clearly,
// while the compare logic lives here once.
using Builder = CCShapeId (*)();

struct OpCase {
    const char* name;
    Builder build;
    bool planar;      // true → exact tol; false → deflection-scaled tol (curved)
    double deflection;
};

// Compare the native build of `s` against the OCCT build; emit the four PASS/FAIL
// lines (mass / bbox / faces / tessellate), each tagged [native].
void runNativeOp(const OpCase& s) {
    char detail[512];
    const double volRelTol = s.planar ? 1e-6 : 5e-2;
    const double areaRelTol = s.planar ? 1e-6 : 5e-2;
    const double linTol = s.planar ? 1e-6 : 5.0 * s.deflection;

    // 1) OCCT (oracle).
    cc_set_engine(0);
    const CCShapeId occtId = s.build();
    if (occtId == 0) {
        std::snprintf(detail, sizeof detail, "[native] OCCT build failed: %s", cc_last_error());
        record(false, s.name, detail);
        return;
    }
    const CCMassProps oM = cc_mass_properties(occtId);
    double oBB[6] = {0};
    const int oBBok = cc_bounding_box(occtId, oBB);
    const int oV = subCount(occtId, 0), oE = subCount(occtId, 1), oF = subCount(occtId, 2);

    // 2) Native (same inputs, same facade). This MUST take the native path (a Tier-A
    //    builder that returns non-null for this input) — asserted via cc_active_engine
    //    plus the numeric parity below (a silent fallthrough would still match OCCT,
    //    but the host gate proves these five inputs are native; here we additionally
    //    require the native engine to be active while building).
    cc_set_engine(1);
    const int activeIsNative = cc_active_engine();
    const CCShapeId natId = s.build();
    if (natId == 0 || activeIsNative != 1) {
        std::snprintf(detail, sizeof detail, "[native] native build failed (active=%d): %s",
                      activeIsNative, cc_last_error());
        record(false, s.name, detail);
        cc_set_engine(0);
        cc_shape_release(occtId);
        return;
    }
    const CCMassProps nM = cc_mass_properties(natId);
    double nBB[6] = {0};
    const int nBBok = cc_bounding_box(natId, nBB);
    const int nV = subCount(natId, 0), nE = subCount(natId, 1), nF = subCount(natId, 2);
    const CCMesh nMesh = cc_tessellate(natId, s.deflection);

    // ── mass properties (volume / area / centroid) ───────────────────────────────
    bool massOk = (oM.valid != 0) && (nM.valid != 0) && (oM.volume > 0.0);
    const double volRel = massOk ? std::fabs(nM.volume - oM.volume) / oM.volume : 1.0;
    const double areaRel = (massOk && oM.area > 0.0) ? std::fabs(nM.area - oM.area) / oM.area : 1.0;
    const double cMax = std::max({std::fabs(nM.cx - oM.cx), std::fabs(nM.cy - oM.cy),
                                  std::fabs(nM.cz - oM.cz)});
    massOk = massOk && volRel < volRelTol && areaRel < areaRelTol && cMax < linTol;
    std::snprintf(
        detail, sizeof detail,
        "[native] vol o=%.6g n=%.6g rel=%.2e | area rel=%.2e | centroidΔ=%.2e (tol v=%.0e c=%.0e)",
        oM.volume, nM.volume, volRel, areaRel, cMax, volRelTol, linTol);
    record(massOk, (std::string(s.name) + " mass").c_str(), detail);

    // ── bounding boxes ───────────────────────────────────────────────────────────
    const double bbDelta = maxBBoxCornerDelta(oBB, nBB);
    const bool bbOk = (oBBok == 1) && (nBBok == 1) && (bbDelta < linTol);
    std::snprintf(detail, sizeof detail, "[native] maxCornerΔ=%.2e (tol=%.0e)", bbDelta, linTol);
    record(bbOk, (std::string(s.name) + " bbox").c_str(), detail);

    // ── FACE counts vs the OCCT oracle ───────────────────────────────────────────
    // Native F is a positive INTEGER MULTIPLE of OCCT F: identical when the tiling
    // matches (planar prisms, a partial band), k× when native splits a curved wall /
    // band into <120° angular patches (periodic-face construction deferred — a
    // documented representational difference, NOT a geometric mismatch). VERTEX/EDGE
    // are NOT compared (per-face/per-patch; the watertight mesh below is authoritative).
    const bool facesOk = (oF > 0) && (nF > 0) && (nF % oF == 0) && (nF >= oF);
    std::snprintf(detail, sizeof detail,
                  "[native] F o=%d n=%d (n=%d×o) | V o=%d n=%d E o=%d n=%d (per-face, deferred)", oF,
                  nF, oF > 0 ? nF / oF : 0, oV, nV, oE, nE);
    record(facesOk, (std::string(s.name) + " faces").c_str(), detail);

    // ── native tessellation: WATERTIGHT + self-consistent bbox / volume ──────────
    double meshBB[6] = {0};
    const bool haveMesh = nMesh.triangleCount > 0 && meshBBox(nMesh, meshBB);
    const bool wt = haveMesh && meshWatertight(nMesh);
    const double meshVol = haveMesh ? meshVolume(nMesh) : 0.0;
    const double meshVolRel =
        (haveMesh && nM.volume > 0.0) ? std::fabs(meshVol - nM.volume) / nM.volume : 1.0;
    const double meshBBDelta = haveMesh ? maxBBoxCornerDelta(nBB, meshBB) : 1.0;
    const bool tessOk = haveMesh && wt && meshVolRel < volRelTol && meshBBDelta < linTol;
    std::snprintf(detail, sizeof detail,
                  "[native] watertight=%d tris=%d meshVolRel=%.2e bboxΔ=%.2e", wt ? 1 : 0,
                  nMesh.triangleCount, meshVolRel, meshBBDelta);
    record(tessOk, (std::string(s.name) + " tessellate").c_str(), detail);

    cc_mesh_free(nMesh);
    cc_set_engine(0);
    cc_shape_release(natId);
    cc_shape_release(occtId);
}

// ── one intentionally-deferred sub-case: native FALLS THROUGH to OCCT ────────────
// Build once under OCCT (the oracle) and once under native. profile.h returns a NULL
// native Shape for this sub-case so the NativeEngine forwards to OCCT — the native
// result must equal the OCCT oracle (delegated, not faked). Tagged [fallback].
void runFallbackOp(const char* name, Builder build, const char* why) {
    char detail[512];

    cc_set_engine(0);
    const CCShapeId occtId = build();
    const CCMassProps oM = occtId ? cc_mass_properties(occtId) : CCMassProps{0, 0, 0, 0, 0, 0};

    cc_set_engine(1);
    const int activeIsNative = cc_active_engine();
    const CCShapeId natId = build();
    const CCMassProps nM = natId ? cc_mass_properties(natId) : CCMassProps{0, 0, 0, 0, 0, 0};

    // The fallthrough must produce a valid result equal to the OCCT oracle: both ids
    // valid, both mass props valid, and volume matching to fp64 (same OCCT geometry).
    const bool bothValid = (occtId != 0) && (natId != 0) && (oM.valid != 0) && (nM.valid != 0);
    const double volRel = (bothValid && oM.volume > 0.0)
                              ? std::fabs(nM.volume - oM.volume) / oM.volume
                              : 1.0;
    const bool ok = (activeIsNative == 1) && bothValid && (oM.volume > 0.0) && volRel < 1e-9;
    std::snprintf(detail, sizeof detail,
                  "[fallback] %s — native active=%d vol o=%.6g n=%.6g rel=%.2e (delegated to OCCT)",
                  why, activeIsNative, oM.volume, nM.volume, volRel);
    record(ok, name, detail);

    cc_set_engine(0);
    if (natId) cc_shape_release(natId);
    if (occtId) cc_shape_release(occtId);
}

// ── per-op builders (identical inputs to both engines; the active engine picks the
//    implementation) ────────────────────────────────────────────────────────────

// 1) HOLED SQUARE: 10×10 outer square, one circular through-hole (5,5,r=2), depth 4.
CCShapeId buildHoledSquare() {
    const double outer[] = {0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0};
    const double holes[] = {5.0, 5.0, 2.0};  // cx, cy, r
    return cc_solid_extrude_holes(outer, 4, holes, 1, 4.0);
}

// 2) POLYGON-HOLE: 10×10 outer square, one 2×2 square hole (4..6), depth 3 (planar).
CCShapeId buildPolyHole() {
    const double outer[] = {0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0};
    const double holeXY[] = {4.0, 4.0, 6.0, 4.0, 6.0, 6.0, 4.0, 6.0};
    const int holeCounts[] = {4};
    return cc_solid_extrude_polyholes(outer, 4, holeXY, holeCounts, 1, 3.0);
}

// 3) TYPED LINE+ARC PROFILE (D-shape): straight diameter line + semicircle arc (r=2),
//    depth 3. The arc extrudes to a TRUE Cylinder wall.
CCShapeId buildDShapeProfile() {
    CCProfileSeg segs[2] = {};
    segs[0].kind = 0;  // straight diameter (0,2) → (0,-2)
    segs[0].x0 = 0.0; segs[0].y0 = 2.0; segs[0].x1 = 0.0; segs[0].y1 = -2.0;
    segs[1].kind = 1;  // semicircle arc back (0,-2) → (0,2) on the +x side
    segs[1].cx = 0.0; segs[1].cy = 0.0; segs[1].r = 2.0;
    segs[1].x0 = 0.0; segs[1].y0 = -2.0; segs[1].x1 = 0.0; segs[1].y1 = 2.0;
    segs[1].a0 = -kPi / 2.0; segs[1].a1 = kPi / 2.0;
    return cc_solid_extrude_profile(segs, 2, nullptr, 0, nullptr, 0, 3.0);
}

// 4) LINE-PROFILE REVOLVE (tube): rectangle [x∈[1,2],y∈[0,3]] as four line segments,
//    revolved 360° about the Y axis → an annular tube.
CCShapeId buildLineRevolveTube() {
    const double rect[8] = {1.0, 0.0, 2.0, 0.0, 2.0, 3.0, 1.0, 3.0};
    CCProfileSeg segs[4] = {};
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        segs[i].kind = 0;
        segs[i].x0 = rect[i * 2]; segs[i].y0 = rect[i * 2 + 1];
        segs[i].x1 = rect[j * 2]; segs[i].y1 = rect[j * 2 + 1];
    }
    return cc_solid_revolve_profile(segs, 4, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
}

// 5) ARC-PROFILE REVOLVE (sphere): an ON-AXIS semicircle arc (r=3, meridian from
//    (0,−3) through (3,0) to (0,3)) revolved 360° about the Y axis → a sphere.
CCShapeId buildArcRevolveSphere() {
    CCProfileSeg seg = {};
    seg.kind = 1;
    seg.cx = 0.0; seg.cy = 0.0; seg.r = 3.0;
    seg.x0 = 0.0; seg.y0 = -3.0; seg.x1 = 0.0; seg.y1 = 3.0;
    seg.a0 = -kPi / 2.0; seg.a1 = kPi / 2.0;
    return cc_solid_revolve_profile(&seg, 1, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
}

// FALLBACK A: kind-3 SPLINE outer edge (extrude) — profile.h defers → OCCT.
//   A CLOSED two-segment outer boundary: a kind-3 B-spline edge fitted through
//   (0,0)→(4,0)→(4,4)→(0,4) plus a kind-0 LINE edge (0,4)→(0,0) that closes the loop.
//   The line guarantees the wire closes regardless of the spline's endpoint handling,
//   so BRepBuilderAPI_MakeFace succeeds and OCCT builds a valid spline-face prism.
//   The spline segment makes native's resolveTypedOuter return NULL (any kind-3 defers
//   the whole profile), so native forwards to this OCCT build — the deferral is live.
CCShapeId buildSplineProfileNative() {
    CCProfileSeg segs[2] = {};
    segs[0].kind = 3;                          // B-spline arc through the 4 points
    segs[0].ptOffset = 0; segs[0].ptCount = 4;
    segs[1].kind = 0;                          // closing straight edge (4th pt) → (1st pt)
    segs[1].x0 = 0.0; segs[1].y0 = 4.0; segs[1].x1 = 0.0; segs[1].y1 = 0.0;
    // splineXYCount is the number of DOUBLES in `splineXY` (x,y pairs → 2·points):
    // OCCT's addSplineEdge bounds-guards with (ptOffset+ptCount)*2 <= splineXYCount.
    const double spline[] = {0.0, 0.0, 4.0, 0.0, 4.0, 4.0, 0.0, 4.0};
    return cc_solid_extrude_profile(segs, 2, nullptr, 0, spline, 8, 2.0);
}

// FALLBACK B: OFF-AXIS closed-circle revolve (circle centre off the axis → TORUS) —
//   deferred. A FULL circle (centre (5,0), r=1) is a closed generatrix whose entire
//   loop lies off the Y axis; revolved 360° it sweeps a TORUS. OCCT builds the closed
//   circle wire → face → revol (a valid torus solid). Native's revolve has no Torus
//   surface (and rejects a full-circle / off-axis-arc generatrix up front), returns
//   NULL, and forwards to this OCCT build — proving the deferral path is live.
//   (A kind-1 arc that is NOT a closed loop cannot be faced by either engine, so the
//   torus fixture uses the closed full-circle generatrix, the canonical torus input.)
CCShapeId buildOffAxisArcRevolveDeferred() {
    CCProfileSeg seg = {};
    seg.kind = 2;                              // full circle generatrix
    seg.cx = 5.0; seg.cy = 0.0; seg.r = 1.0;   // centre off the Y axis (r=5)
    return cc_solid_revolve_profile(&seg, 1, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT profile-construction parity (Tier A #4b, through the cc_* facade)\n");

    const std::vector<OpCase> cases = {
        // 1) Holed square: circular through-hole → curved Cylinder wall, deflection-
        //    bounded. Volume (100 − 4π)·4.
        {"extrude_holes circular", &buildHoledSquare, /*planar*/ false, /*defl*/ 0.02},
        // 2) Polygon-hole prism: all planar → EXACT. Volume (100 − 4)·3 = 288.
        {"extrude_polyholes square", &buildPolyHole, /*planar*/ true, /*defl*/ 0.05},
        // 3) Typed D-shape (line + arc) → the arc wall is a TRUE Cylinder, deflection-
        //    bounded. Volume 6π.
        {"extrude_profile line+arc", &buildDShapeProfile, /*planar*/ false, /*defl*/ 0.01},
        // 4) Line-profile revolve → annular tube, volume 9π. Curved → deflection-bounded.
        {"revolve_profile line-tube", &buildLineRevolveTube, /*planar*/ false, /*defl*/ 0.02},
        // 5) On-axis arc-profile revolve → sphere, volume 36π. Curved → deflection-bounded.
        {"revolve_profile arc-sphere", &buildArcRevolveSphere, /*planar*/ false, /*defl*/ 0.02},
        // 6) kind-3 SPLINE outer edge. RECLASSIFIED from deferred: native no longer falls
        //    through here — residuals.h fits the NURBS and builds a true spline cap edge +
        //    spline-ruled wall. Measured E=12 vs OCCT's 6 (per-patch edges), vol 45.5547 vs
        //    45.600000 (rel 9.92e-04), bboxes agreeing to ~1e-3. The old [fallback] contract
        //    asserted rel < 1e-9, which was only ever valid WHILE native delegated; once it
        //    started building its own solid that assertion could not pass. Curved spline wall
        //    ⇒ deflection-bounded, like every other curved case here.
        //    (The same fixture is already classified native in native_geomcompletion_parity's
        //    "spline extrude" case — this harness's classification was the stale one.)
        {"extrude_profile spline", &buildSplineProfileNative, /*planar*/ false, /*defl*/ 0.01},
    };

    for (const OpCase& s : cases) runNativeOp(s);

    // Intentionally-deferred sub-cases: native returns NULL → forwards to OCCT.
    runFallbackOp("revolve_profile offaxis-arc", &buildOffAxisArcRevolveDeferred,
                  "off-axis closed circle → torus surface of revolution");

    // Restore the default engine explicitly (guard also does, belt-and-braces).
    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
