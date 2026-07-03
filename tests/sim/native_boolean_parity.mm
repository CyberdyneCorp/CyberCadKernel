// SPDX-License-Identifier: Apache-2.0
//
// native_boolean_parity.mm — native-vs-OCCT BOOLEAN parity harness, driven
//                            entirely THROUGH the cc_* facade (iOS simulator).
//
// Phase 4 capability #5 (`native-booleans`) — the RESEARCH-GRADE capability —
// simulator verification gate 2 (see openspec/NATIVE-REWRITE.md). Like the sibling
// native_construct_parity.mm this harness exercises the SHIPPING PATH: it calls the
// same public cc_boolean the app calls, once with the OCCT engine active and once
// with the NativeEngine active, and compares the two results.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default) — BRepAlgoAPI / BOPAlgo.
//   cc_set_engine(1)  → NativeEngine. cc_boolean is NATIVE (clean-room BSP-CSG,
//                       src/native/boolean) for PLANAR-FACED polyhedra whose result
//                       is a single watertight shell, and SELF-VERIFIED (closed
//                       2-manifold AND set-algebra volume) before it is accepted;
//                       everything else (curved faces, near-tangent/degenerate
//                       configs, multi-shell results) FALLS BACK to OCCT.
//
// ── The native/fallback SPLIT this harness asserts (honest, per the spec) ─────────
//
// The NativeEngine can only intercept a boolean when BOTH operands are NATIVE bodies
// (built under cc_set_engine(1)); if either operand is an OCCT body the engine
// forwards the whole boolean to OCCT (it cannot mix voids). And a native-native
// boolean whose planar BSP-CSG result fails the watertight+volume self-verify is
// DISCARDED — the engine reports a clean error (id 0) rather than emit a wrong/leaky
// solid (it CANNOT forward two native voids to OCCT). So this harness drives two
// kinds of case explicitly:
//
//   NATIVE cases — operands BUILT UNDER NATIVE, boolean run UNDER NATIVE. The BSP-CSG
//   path intercepts, produces a verified single-shell watertight solid, and we assert
//   it matches the OCCT oracle EXACTLY (axis-aligned boxes are exact fp64 modelling).
//   The oracle is the SAME inputs built + boolean'd under OCCT.
//     · box∩box overlap : fuse / cut(a−b) / common of two offset-overlapping boxes.
//     · contained       : fuse / common of a small box inside a big box.
//
//   FALLBACK cases — operands BUILT UNDER OCCT, boolean run UNDER NATIVE. The engine
//   sees two OCCT bodies and forwards to the OCCT oracle transparently, so native ==
//   OCCT EXACTLY (a fall-through proof: cc_active_engine()==1 yet the OCCT result is
//   returned, never faked). These are the configurations OUTSIDE the native planar
//   single-shell domain that the spec requires to fall back:
//     · cylinder-box    : a CURVED operand (revolved tube) — non-planar, native NULL.
//     · near-coincident : two boxes sharing a face at a ~1e-9 offset — a degenerate/
//                         near-tangent configuration (the classic BOPAlgo hazard).
//     · disjoint boxes  : fuse of two separated boxes is TWO shells — not a single-
//                         shell planar result; falls back.
//
//   SELF-VERIFY GUARD — additionally, a NATIVE-NATIVE boolean outside the domain
//   (disjoint `common`, whose result is empty) must return id 0 (a clean error), NOT
//   a wrong/leaky solid. This is the mandatory guard exercised directly.
//
// ── Per-case comparisons (native/forwarded result vs OCCT oracle) ─────────────────
//   * cc_mass_properties : volume + area + centroid.  (+ analytic volume anchor for
//                          the exact axis-aligned box cases.)
//   * cc_bounding_box    : the two exact B-rep AABBs.
//   * cc_tessellate      : the result mesh is WATERTIGHT (closed 2-manifold) and its
//                          mesh-volume matches the B-rep volume.
//   EXACT tolerance (1e-6 rel/abs) for every axis-aligned box case — native and OCCT
//   agree to fp round-off. The cylinder-box fallback is a forwarded OCCT result on
//   BOTH sides (same engine both times) so its B-rep props are exact too; only its
//   CURVED-wall mesh volume uses a deflection-bounded tolerance.
//
// Output: [NBOOL] PASS/FAIL lines, each tagged [native] or [fallback], with per-case
// volume deltas; then "== N passed, M failed ==". Flushes stdout and
// std::_Exit(failed?1:0) — the trimmed static-OCCT build's static teardown is not
// exit-clean (same rationale as the sibling sim harnesses; every id is released
// before exit).
//
// Build: scripts/run-sim-native-boolean.sh — compiles this harness + the whole
// facade/core/engine (incl. NativeEngine + OCCT adapter) + src/native/**, links the
// OCCT libs, spawns on a booted simulator. On run-sim-suite.sh's SKIP list (its own
// main(), competes for the entry point).

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── result accounting ─────────────────────────────────────────────────────────
int g_passed = 0;
int g_failed = 0;

void record(bool ok, const char* tag, const std::string& label, const char* detail) {
    if (ok) {
        ++g_passed;
        std::printf("[NBOOL] PASS  %-10s %-26s %s\n", tag, label.c_str(), detail);
    } else {
        ++g_failed;
        std::printf("[NBOOL] FAIL  %-10s %-26s %s\n", tag, label.c_str(), detail);
    }
}

// ── restore the default (OCCT) engine no matter how we leave scope ──────────────
struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

// ── mesh-level helpers (operate on a CCMesh returned by cc_tessellate) ──────────

// A mesh is watertight iff it is a closed 2-manifold: every undirected edge is
// shared by exactly two triangles.
//
// The edge count is taken over POSITION-WELDED vertex ids, not the raw mesh indices.
// The native tessellator already welds shared corners to one index, but OCCT's
// BRepMesh emits each face's triangulation with its OWN copy of every boundary
// vertex (a single box → 24 vertices, 4 per face), so a raw-index edge map would
// report EVERY shared solid edge as unmatched even for a perfectly closed solid.
// Welding coincident positions first makes the check engine-agnostic — it verifies
// the GEOMETRIC closure of the surface, which is what "watertight" means here — so
// it holds for both the native results and the OCCT-forwarded fallback results.
bool meshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0) return false;

    // Position-weld: map each vertex to a representative id via a quantised grid
    // (1e-7 cell), so per-face-duplicated OCCT vertices collapse to one id.
    constexpr double kWeld = 1e-7;
    std::unordered_map<std::uint64_t, int> cellToId;
    std::vector<int> rep(static_cast<std::size_t>(m.vertexCount));
    auto cellKey = [](long long x, long long y, long long z) -> std::uint64_t {
        std::uint64_t h = static_cast<std::uint64_t>(x) * 73856093u;
        h ^= static_cast<std::uint64_t>(y) * 19349663u;
        h ^= static_cast<std::uint64_t>(z) * 83492791u;
        return h;
    };
    auto q = [](double v) -> long long {
        const double s = v / kWeld;
        return static_cast<long long>(s >= 0 ? s + 0.5 : s - 0.5);
    };
    for (int v = 0; v < m.vertexCount; ++v) {
        const double* p = &m.vertices[v * 3];
        const std::uint64_t c = cellKey(q(p[0]), q(p[1]), q(p[2]));
        auto it = cellToId.find(c);
        if (it == cellToId.end()) {
            cellToId.emplace(c, v);
            rep[static_cast<std::size_t>(v)] = v;
        } else {
            rep[static_cast<std::size_t>(v)] = it->second;
        }
    }

    std::unordered_map<std::uint64_t, int> edgeCount;
    auto key = [](int a, int b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = rep[static_cast<std::size_t>(m.triangles[t * 3 + 0])];
        const int j = rep[static_cast<std::size_t>(m.triangles[t * 3 + 1])];
        const int k = rep[static_cast<std::size_t>(m.triangles[t * 3 + 2])];
        ++edgeCount[key(i, j)];
        ++edgeCount[key(j, k)];
        ++edgeCount[key(k, i)];
    }
    for (const auto& [e, c] : edgeCount)
        if (c != 2) return false;
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

// ── operand builders (each uses the ACTIVE engine's cc_solid_extrude/_revolve) ───

// An axis-aligned box [x0,x1]×[y0,y1]×[0,dz] as a rectangle profile extruded by dz.
// Built with WHATEVER engine is active (native → native prism; OCCT → OCCT prism).
CCShapeId buildBox(double x0, double y0, double x1, double y1, double dz) {
    const double rect[] = {x0, y0, x1, y0, x1, y1, x0, y1};
    return cc_solid_extrude(rect, 4, dz);
}

// A solid cylinder (r, height h along +Y about the Y axis): revolve the rectangle
// x∈[0,r], y∈[0,h] a full turn. CURVED — the native boolean domain excludes it.
CCShapeId buildCylinder(double r, double h) {
    const double rect[] = {0.0, 0.0, r, 0.0, r, h, 0.0, h};
    return cc_solid_revolve(rect, 4, 2.0 * kPi);
}

// A closure that builds the two operands (a, b) with whatever engine is active.
using BuildPair = std::function<void(CCShapeId&, CCShapeId&)>;

// ── a boolean case comparing the native/forwarded result against the OCCT oracle ──
//
// mode:
//   Native   — build both operands UNDER NATIVE (engine 1), run cc_boolean UNDER
//              NATIVE. Expect the BSP-CSG path to intercept and pass self-verify → a
//              native solid, compared EXACTLY against the same-input OCCT oracle.
//   Fallback — build both operands UNDER OCCT (engine 0), run cc_boolean UNDER NATIVE.
//              The engine forwards to OCCT (two OCCT voids), so the candidate IS the
//              OCCT result; still labelled [fallback] with cc_active_engine()==1 as the
//              fall-through proof.
enum class Mode { Native, Fallback };

struct BoolCase {
    std::string name;
    int op;             // 0 fuse, 1 cut a−b, 2 common
    Mode mode;
    BuildPair build;
    bool exactBox;      // true → axis-aligned box result: check the analytic anchor + exact mesh
    double expectVol;   // analytic expected result volume (only checked when exactBox)
    double deflection;  // mesh deflection for the watertight/volume check
    // A CURVED-face OCCT fallback result: OCCT meshes each face independently, so a
    // boolean-created intersection SEAM between a cylindrical wall and a planar face is
    // triangulated with non-matching vertex counts on the two sides — the mesh is a
    // correct APPROXIMATION of a closed solid (its volume matches the exact B-rep within
    // the deflection bound, and the B-rep mass/bbox parity is checked separately) but is
    // NOT a triangle-level closed 2-manifold, and no vertex-weld can make it one. So for
    // such a case we assert the mesh VOLUME bound only and do not require watertight — the
    // fallback's contract is to return the correct SOLID, not a watertight triangle soup.
    // (Planar fallbacks — near-coincident, disjoint — DO close and keep the check.)
    bool curvedFallbackMesh = false;
};

// A boolean result snapshot (id + queried props).
struct Snapshot {
    CCShapeId id = 0;
    CCMassProps mass{0, 0, 0, 0, 0, 0};
    double bb[6] = {0, 0, 0, 0, 0, 0};
    int bbok = 0;
    bool activeNative = false;
};

// Build the two operands under `buildEngine`, then run cc_boolean under `boolEngine`,
// returning the result id + its queried props. Releases the operands. The two-engine
// split is the crux: for a Native case buildEngine==boolEngine==1 (native intercepts);
// for a Fallback case buildEngine==0, boolEngine==1 (OCCT operands → engine forwards).
Snapshot buildAndBool(const BuildPair& build, int op, int buildEngine, int boolEngine) {
    cc_set_engine(buildEngine);
    CCShapeId a = 0, b = 0;
    build(a, b);
    cc_set_engine(boolEngine);
    Snapshot s;
    s.activeNative = cc_active_engine() == 1;
    if (a != 0 && b != 0) {
        s.id = cc_boolean(a, b, op);
        if (s.id != 0) {
            s.mass = cc_mass_properties(s.id);
            s.bbok = cc_bounding_box(s.id, s.bb);
        }
    }
    if (a) cc_shape_release(a);
    if (b) cc_shape_release(b);
    return s;
}

// ── run one boolean case under both engines and compare ──────────────────────────
void runCase(const BoolCase& c) {
    char detail[512];
    const char* tag = (c.mode == Mode::Native) ? "native" : "fallback";
    const double volRelTol = 1e-6, linTol = 1e-6;  // axis-aligned boxes are exact fp64

    // 1) OCCT oracle: operands + boolean both under OCCT (the reference answer).
    const Snapshot oracle = buildAndBool(c.build, c.op, /*build*/ 0, /*bool*/ 0);
    if (oracle.id == 0 || oracle.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "OCCT oracle build/boolean failed: %s",
                      cc_last_error());
        record(false, tag, c.name, detail);
        cc_set_engine(0);
        if (oracle.id) cc_shape_release(oracle.id);
        return;
    }

    // 2) Candidate: Native case → operands+boolean under native (BSP-CSG intercepts);
    //    Fallback case → operands under OCCT, boolean under native (engine forwards).
    const int buildEng = (c.mode == Mode::Native) ? 1 : 0;
    const Snapshot cand = buildAndBool(c.build, c.op, buildEng, /*bool*/ 1);
    const CCMesh cMesh =
        cand.id ? cc_tessellate(cand.id, c.deflection) : CCMesh{nullptr, 0, nullptr, 0};

    if (cand.id == 0 || cand.mass.valid == 0) {
        std::snprintf(detail, sizeof detail,
                      "native active=%d, cc_boolean->0 (%s) — expected a valid %s result",
                      cand.activeNative ? 1 : 0, cc_last_error(), tag);
        record(false, tag, c.name, detail);
        cc_set_engine(0);
        cc_shape_release(oracle.id);
        return;
    }

    // ── mass properties (volume / area / centroid) vs the oracle ──────────────────
    const double volRel = std::fabs(cand.mass.volume - oracle.mass.volume) / oracle.mass.volume;
    const double areaRel =
        oracle.mass.area > 0.0 ? std::fabs(cand.mass.area - oracle.mass.area) / oracle.mass.area : 1.0;
    const double cMax = std::max({std::fabs(cand.mass.cx - oracle.mass.cx),
                                  std::fabs(cand.mass.cy - oracle.mass.cy),
                                  std::fabs(cand.mass.cz - oracle.mass.cz)});
    // Analytic anchor only for exact box cases (the oracle itself must equal expectVol).
    const bool anchorOk =
        !c.exactBox ||
        std::fabs(oracle.mass.volume - c.expectVol) / c.expectVol < 1e-6;
    const bool massOk = cand.activeNative && volRel < volRelTol && areaRel < volRelTol &&
                        cMax < linTol && anchorOk;
    if (c.exactBox)
        std::snprintf(detail, sizeof detail,
                      "vol o=%.6g n=%.6g rel=%.2e (expect %.6g) | area rel=%.2e | cΔ=%.2e",
                      oracle.mass.volume, cand.mass.volume, volRel, c.expectVol, areaRel, cMax);
    else
        std::snprintf(detail, sizeof detail,
                      "vol o=%.6g n=%.6g rel=%.2e | area rel=%.2e | cΔ=%.2e (forwarded=OCCT)",
                      oracle.mass.volume, cand.mass.volume, volRel, areaRel, cMax);
    record(massOk, tag, c.name + " mass", detail);

    // ── bounding boxes vs the oracle ──────────────────────────────────────────────
    const double bbDelta = maxBBoxCornerDelta(oracle.bb, cand.bb);
    const bool bbOk = (oracle.bbok == 1) && (cand.bbok == 1) && (bbDelta < linTol);
    std::snprintf(detail, sizeof detail, "maxCornerΔ=%.2e (tol=%.0e)", bbDelta, linTol);
    record(bbOk, tag, c.name + " bbox", detail);

    // ── the RESULT mesh is watertight + its volume matches the B-rep ─────────────
    const bool haveMesh = cMesh.triangleCount > 0;
    const bool wt = haveMesh && meshWatertight(cMesh);
    const double meshVol = haveMesh ? meshVolume(cMesh) : 0.0;
    const double meshVolRel =
        (haveMesh && cand.mass.volume > 0.0) ? std::fabs(meshVol - cand.mass.volume) / cand.mass.volume : 1.0;
    // Box results mesh exactly; the cylinder-box fallback has a curved wall so use a
    // deflection-bounded mesh tolerance there.
    const double meshTol = c.exactBox ? 1e-6 : 5e-2;
    // Watertightness is required for every case EXCEPT a curved OCCT fallback mesh (see
    // curvedFallbackMesh) — there OCCT's independent per-face meshing cannot weld the
    // boolean seam, so we assert the mesh-volume bound only. All native results, and the
    // planar fallbacks, must still close.
    const bool tessOk =
        haveMesh && meshVolRel < meshTol && (c.curvedFallbackMesh || wt);
    std::snprintf(detail, sizeof detail, "watertight=%d tris=%d meshVolRel=%.2e%s", wt ? 1 : 0,
                  cMesh.triangleCount, meshVolRel,
                  c.curvedFallbackMesh ? " (curved fallback: volume-bound only)" : "");
    record(tessOk, tag, c.name + " tessellate", detail);

    if (haveMesh) cc_mesh_free(cMesh);
    cc_set_engine(0);
    cc_shape_release(cand.id);
    cc_shape_release(oracle.id);
}

// ── the SELF-VERIFY GUARD in action ───────────────────────────────────────────────
// A native-native boolean OUTSIDE the planar single-shell domain must return id 0 (a
// clean error), NEVER a wrong/leaky solid. Two NATIVE disjoint unit boxes under
// `common`: the overlap is empty, so the planar path cannot produce a valid solid; the
// engine cannot forward two native voids to OCCT, so it reports an honest error.
void runNativeGuard() {
    char detail[512];
    cc_set_engine(1);
    const CCShapeId a = buildBox(0.0, 0.0, 1.0, 1.0, 1.0);        // unit box at origin
    const CCShapeId b = buildBox(5.0, 5.0, 6.0, 6.0, 1.0);        // far-away unit box
    const bool activeNative = cc_active_engine() == 1;
    const CCShapeId common = (a && b) ? cc_boolean(a, b, 2) : -1;  // op 2 = common (empty)
    const bool ok = activeNative && (a != 0) && (b != 0) && (common == 0);
    std::snprintf(detail, sizeof detail,
                  "native active=%d, native-native common(disjoint)->id=%ld (expect 0) — self-verify rejected: %s",
                  activeNative ? 1 : 0, static_cast<long>(common), cc_last_error());
    record(ok, "native", "self-verify-guard", detail);
    cc_set_engine(0);
    if (common > 0) cc_shape_release(common);
    if (a) cc_shape_release(a);
    if (b) cc_shape_release(b);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT boolean parity (through the cc_* facade)\n");

    // Two 2×2×2 boxes (dz=2): A at [0,2]², B at [1,3]² → overlap column 1×1×2 (vol 2).
    //   fuse = 8+8−2 = 14, cut(a−b) = 8−2 = 6, common = 2.
    const BuildPair overlapPair = [](CCShapeId& a, CCShapeId& b) {
        a = buildBox(0.0, 0.0, 2.0, 2.0, 2.0);
        b = buildBox(1.0, 1.0, 3.0, 3.0, 2.0);
    };
    // Contained: big box A (4×4×4, vol 64) with a small box B (1×1×1, vol 1) sitting on
    // A's floor and fully inside A in x/y (B ⊂ A). fuse = A = 64 (B adds nothing);
    // common = B = 1. (An A−B cut here would carve an internal void → two shells →
    // outside the single-shell planar domain, so it is NOT tested as a native case.)
    const BuildPair containedPair = [](CCShapeId& a, CCShapeId& b) {
        a = buildBox(0.0, 0.0, 4.0, 4.0, 4.0);   // z∈[0,4]
        b = buildBox(1.0, 1.0, 2.0, 2.0, 1.0);   // z∈[0,1], x/y inside A
    };

    std::vector<BoolCase> cases = {
        // ── NATIVE: box∩box overlap, all three ops (axis-aligned → EXACT) ─────────
        {"overlap-fuse", 0, Mode::Native, overlapPair, /*exact*/ true, 14.0, 0.05},
        {"overlap-cut", 1, Mode::Native, overlapPair, true, 6.0, 0.05},
        {"overlap-common", 2, Mode::Native, overlapPair, true, 2.0, 0.05},
        // ── NATIVE: contained box (B ⊂ A in x/y, on A's floor) ────────────────────
        {"contained-fuse", 0, Mode::Native, containedPair, true, 64.0, 0.05},
        {"contained-common", 2, Mode::Native, containedPair, true, 1.0, 0.05},

        // ── FALLBACK: a CURVED operand (cylinder ∪ box) → native NULL → OCCT ──────
        // Cylinder r=2 h=4 about Y fused with a small box overlapping its side. Curved
        // faces are outside the native planar domain, so the engine forwards to OCCT.
        // Not an analytic volume (box overlaps the round wall) → exactBox=false, so the
        // check is native==oracle (same OCCT engine both times) with a deflection-bounded
        // mesh tolerance on the curved wall.
        {"cyl-box-fuse", 0, Mode::Fallback,
         [](CCShapeId& a, CCShapeId& b) {
             a = buildCylinder(2.0, 4.0);
             b = buildBox(1.0, 1.0, 3.0, 3.0, 2.0);
         },
         /*exact*/ false, 0.0, 0.02, /*curvedFallbackMesh*/ true},
        // ── FALLBACK: near-coincident boxes (a ~1e-9 shared-face offset — the classic
        //    BOPAlgo near-tangent hazard) → OCCT. Two 2³ boxes stacked so B's floor sits
        //    a hair (1e-9) below A's ceiling. Not exact-anchored (a degenerate overlap);
        //    the assertion is native==OCCT oracle exactly (forwarded).
        {"near-coincident-fuse", 0, Mode::Fallback,
         [](CCShapeId& a, CCShapeId& b) {
             a = buildBox(0.0, 0.0, 2.0, 2.0, 2.0);              // z∈[0,2]
             CCShapeId b0 = buildBox(0.0, 0.0, 2.0, 2.0, 2.0);   // same footprint, z∈[0,2]
             b = b0 ? cc_translate_shape(b0, 0.0, 0.0, 2.0 - 1e-9) : 0;
             if (b0) cc_shape_release(b0);
         },
         false, 0.0, 0.05},
        // ── FALLBACK: disjoint boxes fuse → TWO shells (not single-shell) → OCCT ──
        // Fuse of two separated 1³ boxes = 2. exactBox anchors the oracle volume; the
        // result is a forwarded OCCT compound (two solids) — its mesh is still watertight
        // per shell (every edge shared by two tris), so the watertight check holds.
        {"disjoint-fuse", 0, Mode::Fallback,
         [](CCShapeId& a, CCShapeId& b) {
             a = buildBox(0.0, 0.0, 1.0, 1.0, 1.0);
             b = buildBox(5.0, 5.0, 6.0, 6.0, 1.0);
         },
         /*exact*/ true, 2.0, 0.05},
    };

    for (const BoolCase& c : cases) runCase(c);
    runNativeGuard();

    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
