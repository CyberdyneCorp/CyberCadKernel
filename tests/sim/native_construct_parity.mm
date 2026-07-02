// SPDX-License-Identifier: Apache-2.0
//
// native_construct_parity.mm — native-vs-OCCT CONSTRUCTION parity harness,
//                              driven entirely THROUGH the cc_* facade (iOS
//                              simulator).
//
// Phase 4 capability #4 (`native-construction`), simulator verification gate 2
// (see openspec/NATIVE-REWRITE.md). Unlike the earlier native_*_parity harnesses
// (which link the native library directly and bridge a TopoDS_Shape into the
// native model), this harness exercises the SHIPPING PATH: it calls the same
// public cc_* entry points the app calls, once with the OCCT engine active and
// once with the NativeEngine active, and compares the two results.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default)
//   cc_set_engine(1)  → NativeEngine (native solid_extrude / solid_revolve; every
//                       other op falls through to OCCT)
//
// The switch is the ADDITIVE facade toggle cc_set_engine / cc_active_engine
// (mirrors cc_set_parallel); DEFAULT stays OCCT so every existing suite is
// unchanged. This harness restores cc_set_engine(0) before exit.
//
// ── Shapes (identical inputs to BOTH engines) ─────────────────────────────────
//   1. BOX            — a 3×2 rectangle profile extruded by depth 5 (cc_solid_
//                       extrude). Rectangular caps: the native prism is planar-
//                       EXACT and meshes WATERTIGHT.
//   2. TRIANGLE PRISM — a triangle profile extruded by depth 5 (cc_solid_extrude).
//                       The native TOPOLOGY is exact (5 faces = 2 caps + 3 quads)
//                       and — now that the tessellator's convex cap-fill is fixed
//                       (for a planar face trim.h isFullRectangle also requires the
//                       loop to hit all four box corners, so a triangle cap is
//                       ear-clipped instead of filled as the whole UV rectangle) —
//                       the native MESH is watertight with the exact B-rep volume. We
//                       compare volume/area/centroid/bbox + face count AND require a
//                       watertight native mesh.
//   3. CYLINDER (TUBE)— a rectangle [x∈[1,2], y∈[0,3]] revolved 360° about the Y
//                       axis (cc_solid_revolve) → an annular tube, volume 9π. The
//                       native surfaces of revolution are curved, so this is a
//                       DEFLECTION-BOUNDED comparison (native mesh-derived vs OCCT
//                       exact) at a relative tolerance, and the native mesh is
//                       required watertight (curved shared edges weld — the two-
//                       stage native mesher).
//   4. PARTIAL REVOLVE— the same rectangle revolved 90° (cc_solid_revolve) → a
//                       quarter tube, volume 9π/4, closed by two planar side caps.
//                       Deflection-bounded, watertight native mesh required.
//
// ── Per-shape comparisons (native vs OCCT, through the facade) ────────────────
//   * cc_mass_properties : volume + area + centroid match (EXACT tol for the
//     planar prisms — box/triangle; DEFLECTION-scaled relative tol for the curved
//     revolves — cylinder/partial).
//   * cc_bounding_box    : the two AABBs match (exact tol for prisms; deflection
//     tol for revolves).
//   * cc_subshape_ids    : native FACE count vs the oracle. For a matching tiling
//     (prisms, partial revolve) native F == OCCT F exactly; for a full-turn revolve
//     native tiles each periodic OCCT face into angular patches (spans < π — see
//     construct.h), so native F is an integer MULTIPLE of OCCT F (a documented
//     representational difference — periodic-face construction is deferred, NOT a
//     geometric mismatch). VERTEX/EDGE counts are NOT compared to OCCT: the native
//     builder emits per-face edges / per-patch vertices (proper sharing deferred),
//     so its V/E are an inflated-but-valid representation of the SAME solid; the
//     watertight mesh check is the authoritative topological-closure test.
//   * cc_tessellate      : the NATIVE mesh is watertight (every undirected edge
//     shared by exactly two triangles) with a bbox and mesh-volume that match the
//     native B-rep. Required for EVERY shape now that the convex cap-fill is fixed
//     (trim.h) — prisms and revolves alike.
//
// ── Fallthrough assertion ─────────────────────────────────────────────────────
//   Under cc_set_engine(1), a NOT-yet-native op — cc_boolean(fuse) of two boxes —
//   still returns a VALID result, because the NativeEngine delegates it to OCCT.
//   (The two boxes are built by OCCT while native is active — cc_solid_extrude of
//   an overlapping profile is native, but the boolean itself is the fallthrough we
//   assert; to keep both operands consumable by the boolean under one engine we
//   build them with the OCCT engine and then flip to native for the boolean.)
//
// Output: [NCONS] PASS/FAIL lines with per-shape deltas, one fallthrough line,
// then "== N passed, M failed ==". Flushes stdout and std::_Exit(failed?1:0) — the
// trimmed static-OCCT build's static teardown is not exit-clean (same rationale as
// the sibling sim harnesses; every id here is released before exit).
//
// Build: scripts/run-sim-native-construct.sh — compiles this harness + the whole
// facade/core/engine (incl. NativeEngine + OCCT adapter) + src/native/**, links
// the OCCT libs, spawns on a booted simulator. On run-sim-suite.sh's SKIP list
// (its own main(), competes for the entry point).

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
        std::printf("[NCONS] PASS  %-28s %s\n", label, detail);
    } else {
        ++g_failed;
        std::printf("[NCONS] FAIL  %-28s %s\n", label, detail);
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

// ── per-shape input + build via the currently active engine ─────────────────────
enum class Op { Extrude, Revolve };

// How the native B-rep TILING relates to OCCT's for the counts check.
//   Identical    — native produces the SAME face decomposition as OCCT, so the
//                  native FACE count must equal OCCT's exactly (extrude prisms and a
//                  partial revolve: one planar face per profile edge + caps).
//   AngularTiling— native tiles a FULL-TURN surface of revolution into angular
//                  patches (spans < π to keep every patch a non-degenerate quad —
//                  see construct.h), whereas OCCT uses ONE periodic face with a seam
//                  edge. The two are geometrically identical solids (volume / area /
//                  bbox / watertight all match, asserted below) but tile the surface
//                  differently, so native F is a positive INTEGER MULTIPLE of OCCT F
//                  rather than equal. This is a documented representational
//                  difference, NOT a geometric mismatch — native periodic-face
//                  construction is deferred (openspec/NATIVE-REWRITE.md).
enum class Tiling { Identical, AngularTiling };

struct ShapeCase {
    const char* name;
    Op op;
    std::vector<double> profile;  // x,y pairs
    double param;                 // extrude depth OR revolve angle (radians)
    bool planar;                  // true → exact tol (prism); false → deflection tol
    Tiling tiling;                // how native faces relate to OCCT's (counts check)
    double deflection;            // tessellation deflection for the mesh checks
};

// Build the shape with the ACTIVE engine and return its id (0 on failure).
CCShapeId buildShape(const ShapeCase& s) {
    if (s.op == Op::Extrude)
        return cc_solid_extrude(s.profile.data(), static_cast<int>(s.profile.size() / 2), s.param);
    return cc_solid_revolve(s.profile.data(), static_cast<int>(s.profile.size() / 2), s.param);
}

int subCount(CCShapeId id, int kind) {
    int* ids = nullptr;
    const int n = cc_subshape_ids(id, kind, &ids);
    cc_ints_free(ids);
    return n;
}

// ── run one shape under both engines and compare ────────────────────────────────
void runShape(const ShapeCase& s) {
    char detail[512];

    // Tolerances. Planar prisms are exact fp64 modelling; curved revolves are
    // compared through a mesh so the tolerance is deflection-scaled.
    const double volRelTol = s.planar ? 1e-6 : 5e-2;
    const double areaRelTol = s.planar ? 1e-6 : 5e-2;
    const double linTol = s.planar ? 1e-6 : 5.0 * s.deflection;

    // 1) OCCT (oracle).
    cc_set_engine(0);
    const CCShapeId occtId = buildShape(s);
    if (occtId == 0) {
        std::snprintf(detail, sizeof detail, "OCCT build failed: %s", cc_last_error());
        record(false, s.name, detail);
        return;
    }
    const CCMassProps oM = cc_mass_properties(occtId);
    double oBB[6] = {0};
    const int oBBok = cc_bounding_box(occtId, oBB);
    const int oV = subCount(occtId, 0), oE = subCount(occtId, 1), oF = subCount(occtId, 2);

    // 2) Native (same inputs, same facade).
    cc_set_engine(1);
    const CCShapeId natId = buildShape(s);
    if (natId == 0) {
        std::snprintf(detail, sizeof detail, "native build failed: %s", cc_last_error());
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

    // ── compare: mass properties (volume / area / centroid) ──────────────────────
    bool massOk = (oM.valid != 0) && (nM.valid != 0) && (oM.volume > 0.0);
    const double volRel = massOk ? std::fabs(nM.volume - oM.volume) / oM.volume : 1.0;
    const double areaRel = (massOk && oM.area > 0.0) ? std::fabs(nM.area - oM.area) / oM.area : 1.0;
    const double dCx = std::fabs(nM.cx - oM.cx);
    const double dCy = std::fabs(nM.cy - oM.cy);
    const double dCz = std::fabs(nM.cz - oM.cz);
    const double cMax = std::max(dCx, std::max(dCy, dCz));
    massOk = massOk && volRel < volRelTol && areaRel < areaRelTol && cMax < linTol;
    std::snprintf(detail, sizeof detail,
                  "vol o=%.6g n=%.6g rel=%.2e | area rel=%.2e | centroidΔ=%.2e (tol v=%.0e c=%.0e)",
                  oM.volume, nM.volume, volRel, areaRel, cMax, volRelTol, linTol);
    record(massOk, (std::string(s.name) + " mass").c_str(), detail);

    // ── compare: bounding boxes ──────────────────────────────────────────────────
    const double bbDelta = maxBBoxCornerDelta(oBB, nBB);
    const bool bbOk = (oBBok == 1) && (nBBok == 1) && (bbDelta < linTol);
    std::snprintf(detail, sizeof detail, "maxCornerΔ=%.2e (tol=%.0e)", bbDelta, linTol);
    record(bbOk, (std::string(s.name) + " bbox").c_str(), detail);

    // ── compare: FACE counts vs the OCCT oracle ──────────────────────────────────
    // Faces are the topology entity the two engines agree on for a matching tiling.
    // VERTEX/EDGE counts are NOT compared to OCCT here: the native builder emits
    // per-face edges (a cube edge is one edge node PER adjacent face, not shared) and
    // per-patch vertices, so native V/E are an inflated-but-valid representation of
    // the SAME solid — proper edge/vertex sharing (and OCCT-style periodic faces) is
    // deferred (openspec/NATIVE-REWRITE.md). The mesh WATERTIGHT check below is the
    // authoritative topological-closure test (it proves every boundary is shared by
    // exactly two triangles regardless of how the B-rep tiles the surface).
    bool facesOk = false;
    if (s.tiling == Tiling::Identical) {
        facesOk = (oF == nF) && (oF > 0);  // same decomposition ⇒ exact face parity
        std::snprintf(detail, sizeof detail,
                      "F o=%d n=%d (identical tiling) | V o=%d n=%d E o=%d n=%d (per-face, deferred)",
                      oF, nF, oV, nV, oE, nE);
    } else {
        // Full-turn revolve: native tiles each periodic OCCT face into k angular
        // patches, so native F = k · (OCCT F) for some integer k ≥ 1.
        facesOk = (oF > 0) && (nF > 0) && (nF % oF == 0) && (nF >= oF);
        std::snprintf(detail, sizeof detail,
                      "F o=%d n=%d (angular tiling n=%d×o) | V o=%d n=%d E o=%d n=%d (deferred)", oF,
                      nF, oF > 0 ? nF / oF : 0, oV, nV, oE, nE);
    }
    record(facesOk, (std::string(s.name) + " faces").c_str(), detail);

    // ── native tessellation: WATERTIGHT + self-consistent bbox/volume ────────────
    // The tessellator's convex cap-fill is fixed (trim.h), so EVERY native solid —
    // prisms and revolves alike — must now mesh watertight (a closed 2-manifold).
    double meshBB[6] = {0};
    const bool haveMesh = nMesh.triangleCount > 0 && meshBBox(nMesh, meshBB);
    const bool wt = haveMesh && meshWatertight(nMesh);
    const double meshVol = haveMesh ? meshVolume(nMesh) : 0.0;
    const double meshVolRel =
        (haveMesh && nM.volume > 0.0) ? std::fabs(meshVol - nM.volume) / nM.volume : 1.0;
    const double meshBBDelta = haveMesh ? maxBBoxCornerDelta(nBB, meshBB) : 1.0;
    const bool tessOk = haveMesh && wt && meshVolRel < volRelTol && meshBBDelta < linTol;
    std::snprintf(detail, sizeof detail, "watertight=%d tris=%d meshVolRel=%.2e bboxΔ=%.2e",
                  wt ? 1 : 0, nMesh.triangleCount, meshVolRel, meshBBDelta);
    record(tessOk, (std::string(s.name) + " tessellate").c_str(), detail);

    cc_mesh_free(nMesh);
    cc_set_engine(0);
    cc_shape_release(natId);
    cc_shape_release(occtId);
}

// ── fallthrough: a NOT-yet-native op under the native engine still works ─────────
// cc_boolean of two boxes. The NativeEngine delegates the boolean to OCCT. Because
// a boolean needs both operands consumable by the SAME engine that runs it, we
// build the two boxes under OCCT, then activate native and fuse — proving the
// native engine forwards OCCT bodies to the OCCT boolean transparently.
void runFallthrough() {
    char detail[512];
    const double boxA[] = {0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0};
    const double boxB[] = {1.0, 1.0, 3.0, 1.0, 3.0, 3.0, 1.0, 3.0};

    cc_set_engine(0);
    const CCShapeId a = cc_solid_extrude(boxA, 4, 2.0);
    const CCShapeId b = cc_solid_extrude(boxB, 4, 2.0);

    // Flip to native and run the boolean it does NOT implement natively → OCCT.
    cc_set_engine(1);
    const int activeIsNative = cc_active_engine();
    const CCShapeId fused = cc_boolean(a, b, 0);  // op 0 = fuse
    const CCMassProps fM = fused ? cc_mass_properties(fused) : CCMassProps{0, 0, 0, 0, 0, 0};

    // Two 2×2×2 boxes overlapping in a 1×1×2 column: fused volume = 8 + 8 − 2 = 14.
    const bool ok = (activeIsNative == 1) && (fused != 0) && (fM.valid != 0) &&
                    std::fabs(fM.volume - 14.0) < 1e-3;
    std::snprintf(detail, sizeof detail,
                  "native active=%d, cc_boolean(fuse)->id=%ld vol=%.6g (expect 14) — delegated to OCCT",
                  activeIsNative, static_cast<long>(fused), fM.volume);
    record(ok, "fallthrough boolean", detail);

    cc_set_engine(0);
    if (fused) cc_shape_release(fused);
    cc_shape_release(a);
    cc_shape_release(b);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT construction parity (through the cc_* facade)\n");

    const std::vector<ShapeCase> cases = {
        // 1) Box: 3×2 rectangle extruded by 5 → volume 30. Planar-exact, watertight;
        //    same face tiling as OCCT (2 caps + 4 quads).
        {"box", Op::Extrude, {0.0, 0.0, 3.0, 0.0, 3.0, 2.0, 0.0, 2.0}, 5.0,
         /*planar*/ true, Tiling::Identical, /*defl*/ 0.05},
        // 2) Triangle prism: extruded by 5 → 5 faces (same tiling as OCCT). Watertight:
        //    the tessellator's convex cap-fill is fixed (for a planar face trim.h
        //    isFullRectangle now also requires the loop to hit all four box corners,
        //    so a triangle cap is ear-clipped and welds watertight), so the native
        //    mesh volume matches OCCT exactly.
        {"triangle-prism", Op::Extrude, {0.0, 0.0, 4.0, 0.0, 2.0, 3.0}, 5.0,
         /*planar*/ true, Tiling::Identical, /*defl*/ 0.05},
        // 3) Cylinder/tube: rectangle [1..2]×[0..3] revolved 360° → volume 9π. Curved
        //    surfaces → deflection-bounded; watertight required. Native tiles each
        //    periodic OCCT face into angular patches (AngularTiling — periodic-face
        //    construction deferred), so native F is an integer multiple of OCCT F.
        {"cylinder-tube", Op::Revolve, {1.0, 0.0, 2.0, 0.0, 2.0, 3.0, 1.0, 3.0}, 2.0 * kPi,
         /*planar*/ false, Tiling::AngularTiling, /*defl*/ 0.02},
        // 4) Partial revolve: the same rectangle revolved 90° → volume 9π/4, closed by
        //    two planar side caps. Curved → deflection-bounded; watertight required. A
        //    90° span is a single patch per segment, so the face tiling MATCHES OCCT.
        {"partial-revolve-90", Op::Revolve, {1.0, 0.0, 2.0, 0.0, 2.0, 3.0, 1.0, 3.0}, kPi / 2.0,
         /*planar*/ false, Tiling::Identical, /*defl*/ 0.02},
    };

    for (const ShapeCase& s : cases) runShape(s);
    runFallthrough();

    // Restore the default engine explicitly (guard also does, belt-and-braces).
    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
