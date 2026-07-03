// SPDX-License-Identifier: Apache-2.0
//
// native_curved_boolean_parity.mm — native-vs-OCCT CURVED-BOOLEAN parity harness,
//                                    driven THROUGH the cc_* facade (iOS simulator).
//
// Phase 4 capability #5 (`native-booleans`), DEFERRED RESIDUAL #2: the NARROW
// ANALYTIC curved-boolean slice that extends the planar #5 (native_boolean_parity.mm)
// — see openspec/NATIVE-REWRITE.md. This harness is simulator verification GATE 2 for
// that slice. Like the sibling native_boolean_parity.mm it drives the SHIPPING PATH:
// the same public cc_boolean the app calls, once with the OCCT engine active and once
// with the NativeEngine active, and compares the two results.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default) — BRepAlgoAPI / BOPAlgo.
//   cc_set_engine(1)  → NativeEngine. cc_boolean is NATIVE (clean-room ANALYTIC
//                       curved slice, src/native/boolean/curved.h) for the ONE
//                       tractable curved family: an AXIS-ALIGNED BOX ⟷ a CYLINDER
//                       whose axis is PARALLEL to a box axis (and a world axis), with
//                       the cylinder RADIALLY INSIDE the box cross-section. There the
//                       plane-cylinder intersection is ANALYTIC (a box face ⟂ the axis
//                       cuts the cylinder in a CIRCLE), so the builder RECOGNISES the
//                       (box, cylinder) pair and constructs the closed-form result
//                       B-rep directly from TRUE Cylinder walls + Circle rim edges +
//                       Plane caps. The engine then runs a MANDATORY analytic-volume
//                       self-verify (boxVol ± πr²·len, deflection-bounded) and
//                       DISCARDS anything that fails → OCCT. Everything else (sphere,
//                       cone, NON-axis-aligned cylinders, cylinder-cylinder, NURBS,
//                       near-tangent/coincident-curved, general curved) FALLS BACK to
//                       OCCT — labelled, verified, NEVER faked.
//
// ── The native/fallback SPLIT this harness asserts (honest, per the spec) ─────────
//
// The NativeEngine can only intercept a boolean when BOTH operands are NATIVE bodies
// (built under cc_set_engine(1)). If either operand is an OCCT body the engine
// forwards the whole boolean to OCCT (it cannot mix voids). So this harness drives two
// kinds of case explicitly, exactly as native_boolean_parity.mm does:
//
//   NATIVE cases — operands BUILT UNDER NATIVE, boolean run UNDER NATIVE. The curved
//   analytic path (curved::tryBoxCylinder, tried FIRST in boolean_solid) intercepts,
//   produces a verified watertight solid carrying the closed-form volume, and we
//   assert it matches the OCCT oracle within a deflection-bounded tolerance (the
//   result carries a TRUE cylindrical wall, so its watertight MESH volume only
//   approximates the analytic πr²·len — the B-rep volume is exact, the sampled mesh is
//   bounded by the deflection). The oracle is the SAME inputs built + boolean'd under
//   OCCT (BRepPrimAPI box/cyl + BRepAlgoAPI). The facade-reachable analytic cylinder is
//   a Z-AXIS cylinder built from a full-circle TYPED profile extruded along +Z
//   (cc_solid_extrude_profile, kind 2): it occupies the SAME z∈[0,dz] range as a
//   cc_solid_extrude box and can be centred anywhere in X/Y, so it sits radially inside
//   the box. (A cc_solid_revolve cylinder is pinned to z=0 and always breaches a
//   facade-built box — see the FACADE CONSTRAINT note in main().)
//     · through-hole cut  : box − axis-∥ (Z) cylinder spanning THROUGH → round hole.
//     · boss fuse         : box ∪ axis-∥ (Z) cylinder protruding past a cap → boss.
//     · common            : box ∩ axis-∥ (Z) cylinder → the clipped cylinder segment.
//
//   FALLBACK cases — operands BUILT UNDER OCCT, boolean run UNDER NATIVE. The engine
//   sees two OCCT bodies and forwards to the OCCT oracle transparently (native ==
//   OCCT EXACTLY at the B-rep level; a fall-through proof — cc_active_engine()==1 yet
//   the OCCT result is returned, never faked). These are the configurations OUTSIDE the
//   facade-native analytic slice that the spec REQUIRES to fall back:
//     · blind-hole cut    : box − axis-∥ cylinder entering one cap with its far cap
//                           STRICTLY inside → a flat-bottomed pocket (boxVol − πr²·depth).
//                           HONESTLY a fallback, not native: the analytic blind builder
//                           needs the far cap strictly inside (cyl.lo > box.lo), but every
//                           facade extrude/full-circle body starts at z=0, so a native
//                           blind pocket is not facade-reachable — a native body cannot be
//                           translated to make it so (cc_translate_shape on a native body
//                           is unsupported). We build both operands under OCCT (the far cap
//                           positioned by an OCCT-body translate) and let the engine forward.
//     · oblique-cyl cut   : a NON-axis-aligned cylinder (Z-cylinder rotated 30° about X)
//                           cut from a box — the axis is not world-aligned, so the
//                           analytic recogniser declines → OCCT.
//     · sphere-box cut    : a SPHERE cut from a box — a curved surface with no analytic
//                           plane-section family in this slice → OCCT.
//
// ── Per-case comparisons (native/forwarded result vs OCCT oracle) ─────────────────
//   * cc_mass_properties : volume + area + centroid. (+ an analytic-volume anchor for
//                          the NATIVE curved cases: boxVol ± πr²·len.)
//   * cc_bounding_box    : the two exact B-rep AABBs.
//   * cc_tessellate      : the result mesh is WATERTIGHT (closed 2-manifold) and its
//                          mesh-volume matches the B-rep volume within the deflection
//                          bound. The NATIVE curved results MUST be watertight (the
//                          shared Circle rim edges weld by the two-stage mesher). The
//                          OCCT curved fallbacks are the correct SOLID but OCCT meshes
//                          each face independently, so their boolean SEAM is not a
//                          triangle-level closed manifold (no vertex-weld can fix that)
//                          — for those we assert the mesh-VOLUME bound only.
//
// Output: [NCURVBOOL] PASS/FAIL lines, each tagged [native] or [fallback], with the
// per-case volume delta; then "== N passed, M failed ==". Flushes stdout and
// std::_Exit(failed?1:0) — the trimmed static-OCCT build's static teardown is not
// exit-clean (same rationale as the sibling sim harnesses; every id is released first).
//
// Build: scripts/run-sim-native-curved-boolean.sh — compiles this harness + the whole
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
        std::printf("[NCURVBOOL] PASS  %-10s %-24s %s\n", tag, label.c_str(), detail);
    } else {
        ++g_failed;
        std::printf("[NCURVBOOL] FAIL  %-10s %-24s %s\n", tag, label.c_str(), detail);
    }
}

// ── restore the default (OCCT) engine no matter how we leave scope ──────────────
struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

// ── mesh-level helpers (operate on a CCMesh returned by cc_tessellate) ──────────

// A mesh is watertight iff it is a closed 2-manifold: every undirected edge is shared
// by exactly two triangles, taken over POSITION-WELDED vertex ids (OCCT's BRepMesh
// emits per-face duplicate boundary vertices, so a raw-index edge map would misreport a
// perfectly closed solid; welding coincident positions first makes the check
// engine-agnostic — it verifies the GEOMETRIC closure of the surface). Same helper as
// native_boolean_parity.mm.
bool meshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0) return false;

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

// Signed volume of a closed triangle mesh via the divergence theorem; magnitude is the
// enclosed volume for an outward-wound solid.
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

// ── operand builders (each uses the ACTIVE engine's cc_solid_* + transforms) ─────

// An axis-aligned box [x0,x1]×[y0,y1]×[0,dz] as a rectangle profile extruded by dz
// (z∈[0,dz]). Built with WHATEVER engine is active (native → native prism; OCCT →
// OCCT prism).
CCShapeId buildBox(double x0, double y0, double x1, double y1, double dz) {
    const double rect[] = {x0, y0, x1, y0, x1, y1, x0, y1};
    return cc_solid_extrude(rect, 4, dz);
}

// A finite cylinder about the world Z axis, centre (cx,cy), radius r, over z∈[0,dz]:
// a single full-circle TYPED profile segment (kind 2) extruded by dz. This is the
// facade-reachable analytic cylinder: unlike cc_solid_revolve (which revolves the z=0
// profile about world Y through the origin — so the cylinder is pinned to z=0 and a
// facade-built box, z∈[0,depth], cannot straddle it → always a radial breach), a
// full-circle extrude gives a Z-AXIS cylinder over the SAME z∈[0,dz] range the box
// occupies, freely centred at (cx,cy). Its axis is parallel to the box Z axis (and the
// world Z axis) — the analytic family — and it sits radially inside the box when the
// box's X/Y cross-section brackets the disc with margin > r. Built with whatever engine
// is active (native → native cylinder-with-true-Circle-edges; OCCT → OCCT prism).
CCShapeId buildCylinderZ(double cx, double cy, double r, double dz) {
    CCProfileSeg seg{};
    seg.kind = 2;  // full circle
    seg.cx = cx;
    seg.cy = cy;
    seg.r = r;
    return cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, dz);
}

// A cylinder rotated OFF the world axes: build a Z-axis cylinder tall enough to cross
// the box when tilted (z∈[0,dz]), then rotate it `degrees` about +X through the box
// centre (cx,cy,rotCentreZ). The axis is no longer world-aligned, so the native
// analytic recogniser declines → the engine forwards to OCCT. Built under whatever
// engine is active (used only for the OCCT fallback case, so always OCCT here).
CCShapeId buildObliqueCylinder(double cx, double cy, double r, double dz, double degrees,
                               double rotCentreZ) {
    const CCShapeId c0 = buildCylinderZ(cx, cy, r, dz);
    if (c0 == 0) return 0;
    const CCShapeId rot = cc_rotate_shape_about(c0, cx, cy, rotCentreZ, /*axis +X*/ 1.0, 0.0, 0.0,
                                                degrees * kPi / 180.0);
    cc_shape_release(c0);
    return rot;
}

// A sphere of radius r centred at (0, cy, 0): revolve an on-axis semicircle arc a full
// turn about the world Y axis. Uses the TYPED-profile revolve so the arc becomes a TRUE
// Sphere surface (an on-axis-centre arc → sphere) — this is the exact form the proven
// native_revolve_profile_sphere facade test uses (a single kind-1 arc, endpoints on the
// axis at y = cy∓r, the revolve closing the meridian on-axis implicitly). CURVED with
// no analytic plane-section family in this slice → the native engine must fall back.
CCShapeId buildSphereY(double r, double cy) {
    CCProfileSeg seg{};
    seg.kind = 1;  // arc
    seg.cx = 0.0; seg.cy = cy; seg.r = r;
    seg.x0 = 0.0; seg.y0 = cy - r; seg.x1 = 0.0; seg.y1 = cy + r;
    seg.a0 = -kPi / 2.0; seg.a1 = kPi / 2.0;
    return cc_solid_revolve_profile(&seg, 1, /*ax*/ 0.0, /*ay*/ 0.0, /*adx*/ 0.0,
                                    /*ady*/ 1.0, /*splineXY*/ nullptr, 0, 2.0 * kPi);
}

// A closure that builds the two operands (a, b) with whatever engine is active.
using BuildPair = std::function<void(CCShapeId&, CCShapeId&)>;

// ── a boolean case comparing the native/forwarded result against the OCCT oracle ──
//
// mode:
//   Native   — build both operands UNDER NATIVE (engine 1), run cc_boolean UNDER
//              NATIVE. Expect the curved analytic path to intercept and pass the
//              analytic-volume self-verify → a native solid, compared against the
//              same-input OCCT oracle within a deflection-bounded tolerance.
//   Fallback — build both operands UNDER OCCT (engine 0), run cc_boolean UNDER NATIVE.
//              The engine forwards to OCCT (two OCCT voids), so the candidate IS the
//              OCCT result; still labelled [fallback] with cc_active_engine()==1 as the
//              fall-through proof.
enum class Mode { Native, Fallback };

struct BoolCase {
    std::string name;
    int op;              // 0 fuse, 1 cut a−b, 2 common
    Mode mode;
    BuildPair build;
    double expectVol;    // analytic expected result volume (boxVol ± πr²·len; 0 = skip anchor)
    double deflection;   // mesh deflection for the watertight/volume check
    // A NATIVE curved result carries a TRUE cylindrical wall, so its watertight MESH
    // volume only approximates the exact analytic B-rep volume (deflection-bounded).
    // relTol bounds BOTH the native-vs-OCCT B-rep parity (curved area/centroid differ
    // by tessellation-independent B-rep round-off, but the two engines' cylinders are
    // sampled to different vertex counts, so a small relative bound is honest) and the
    // mesh-volume check.
    double relTol;
    // An OCCT curved fallback meshes each face independently, so a boolean-created
    // SEAM between a cylindrical/spherical wall and a planar face is triangulated with
    // non-matching vertex counts on the two sides — the mesh is a correct APPROXIMATION
    // of a closed solid (its volume matches the exact B-rep within the deflection
    // bound) but is NOT a triangle-level closed 2-manifold, and no vertex-weld can make
    // it one. So for such a case we assert the mesh-VOLUME bound only, not watertight.
    bool curvedFallbackMesh = false;
};

// A boolean result snapshot (id + queried props + which engine was active).
struct Snapshot {
    CCShapeId id = 0;
    CCMassProps mass{0, 0, 0, 0, 0, 0};
    double bb[6] = {0, 0, 0, 0, 0, 0};
    int bbok = 0;
    bool activeNative = false;
};

// Build the two operands under `buildEngine`, then run cc_boolean under `boolEngine`,
// returning the result id + its queried props. Releases the operands. The two-engine
// split is the crux: for a Native case buildEngine==boolEngine==1 (curved analytic
// intercepts); for a Fallback case buildEngine==0, boolEngine==1 (OCCT operands →
// engine forwards).
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

// ── run one curved boolean case under both engines and compare ────────────────────
void runCase(const BoolCase& c) {
    char detail[512];
    const char* tag = (c.mode == Mode::Native) ? "native" : "fallback";
    // Curved results are deflection-bounded on BOTH sides, so use the case's relTol for
    // volume/area/mesh. The LINEAR tolerance (bbox corner delta, centroid delta) must
    // account for the NATIVE engine measuring mass/bbox from a MESH at the fixed
    // kPropertyDeflection (0.05) while OCCT measures the exact B-rep — a round wall's
    // sampled extreme sits a fraction of that deflection inside the true surface, so a
    // native-vs-OCCT bbox/centroid can differ by ~a tenth of a unit on a curved feature.
    // A 0.15 linear floor bounds that honestly while still catching a real mismatch (a
    // wrong hole would move the centroid / bbox by whole units). Planar extents are
    // exact, so this only ever loosens the CURVED dimensions.
    const double relTol = c.relTol;
    const double linTol = std::max(0.15, c.deflection);

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

    // 2) Candidate: Native case → operands+boolean under native (curved analytic
    //    intercepts); Fallback case → operands under OCCT, boolean under native (engine
    //    forwards to OCCT).
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
    // Analytic anchor for the NATIVE curved cases: the oracle volume itself must equal
    // the closed-form boxVol ± πr²·len (a Fallback case, or expectVol==0, skips it).
    const bool anchorOk =
        c.expectVol <= 0.0 ||
        std::fabs(oracle.mass.volume - c.expectVol) / c.expectVol < relTol;
    const bool massOk = cand.activeNative && volRel < relTol && areaRel < relTol &&
                        cMax < linTol && anchorOk;
    std::snprintf(detail, sizeof detail,
                  "vol o=%.6g n=%.6g rel=%.2e%s | area rel=%.2e | cΔ=%.2e%s",
                  oracle.mass.volume, cand.mass.volume, volRel,
                  c.expectVol > 0.0 ? (anchorOk ? " (anchor ok)" : " (ANCHOR OFF)") : "",
                  areaRel, cMax, c.mode == Mode::Fallback ? " (forwarded=OCCT)" : "");
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
        (haveMesh && cand.mass.volume > 0.0) ? std::fabs(meshVol - cand.mass.volume) / cand.mass.volume
                                             : 1.0;
    // Watertightness is required for every NATIVE curved result (the shared Circle rim
    // edges weld by the two-stage mesher) and is NOT required for a curved OCCT fallback
    // (OCCT's independent per-face meshing cannot weld the boolean seam — assert the
    // mesh-volume bound only there). All meshes are deflection-bounded.
    const bool tessOk = haveMesh && meshVolRel < relTol && (c.curvedFallbackMesh || wt);
    std::snprintf(detail, sizeof detail, "watertight=%d tris=%d meshVolRel=%.2e%s", wt ? 1 : 0,
                  cMesh.triangleCount, meshVolRel,
                  c.curvedFallbackMesh ? " (curved fallback: volume-bound only)" : "");
    record(tessOk, tag, c.name + " tessellate", detail);

    if (haveMesh) cc_mesh_free(cMesh);
    cc_set_engine(0);
    cc_shape_release(cand.id);
    cc_shape_release(oracle.id);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT CURVED boolean parity (axis-aligned box ⟷ axis-∥ "
                "cylinder, through the cc_* facade)\n");

    // ── Geometry: a 20³ box and an axis-∥ (Z) cylinder radially inside it ─────────
    //
    // The FACADE constraint that fixes the whole design (documented in the host test
    // native_box_cylinder_out_of_family_errors_not_faked): cc_solid_extrude yields a box
    // z∈[0,depth], and cc_solid_revolve revolves the z=0 profile about world Y through
    // the origin — so a revolved cylinder is PINNED to z=0 and a facade-built box (which
    // starts at z=0) can never straddle it → a Y-cylinder ALWAYS radially breaches → out
    // of the analytic family. A native body ALSO cannot be repositioned through the
    // facade (cc_translate_shape on a native body is not supported → error), so we cannot
    // translate a box to straddle z=0 either. The facade-reachable analytic cylinder is
    // therefore a Z-AXIS cylinder built from a full-circle TYPED profile extruded along
    // +Z (buildCylinderZ): it occupies the SAME z∈[0,dz] range as the box and can be
    // centred anywhere in X/Y, so it sits radially inside the box's X/Y cross-section
    // with a real margin. This mirrors the host cylinderZ / the headline 20-cube CUT.
    //
    // Box: x∈[0,20], y∈[0,20], z∈[0,20] (vol 8000). Cylinder: axis Z, centre (10,10),
    // r=5 → radial margin 5 on all four sides (well inside). All three ops are then
    // FACADE-NATIVE and self-verify against the analytic volume.
    const double r = 5.0;
    const double boxVol = 20.0 * 20.0 * 20.0;   // 8000
    const double cylArea = kPi * r * r;         // πr² = 25π
    const double cx = 10.0, cy = 10.0;          // cylinder centre, inside x/y∈[0,20]

    auto box20 = []() { return buildBox(0.0, 0.0, 20.0, 20.0, 20.0); };

    // NATIVE through-hole cut: box − Z-cylinder z∈[0,20] (spans the box depth exactly →
    // a round THROUGH hole). result = boxVol − πr²·20.
    const BuildPair throughHolePair = [box20, cx, cy, r](CCShapeId& a, CCShapeId& b) {
        a = box20();
        b = buildCylinderZ(cx, cy, r, 20.0);          // z∈[0,20], through
    };
    // NATIVE boss fuse: box (z∈[0,20]) ∪ Z-cylinder z∈[0,25] (base flush at z=0, boss
    // protrudes 5 past the z=20 cap). result = boxVol + πr²·5.
    const BuildPair bossPair = [box20, cx, cy, r](CCShapeId& a, CCShapeId& b) {
        a = box20();
        b = buildCylinderZ(cx, cy, r, 25.0);          // protrudes past z=20 by 5
    };
    // NATIVE common: box (z∈[0,20]) ∩ Z-cylinder z∈[0,14] → segment z∈[0,14].
    //   result = πr²·14.
    const BuildPair commonPair = [box20, cx, cy, r](CCShapeId& a, CCShapeId& b) {
        a = box20();
        b = buildCylinderZ(cx, cy, r, 14.0);          // overlap z∈[0,14]
    };

    // FALLBACK blind-hole cut: an OCCT box − an OCCT cylinder entering the hi(+Z) cap
    // with its far cap STRICTLY inside → a flat-bottomed pocket. A NATIVE blind pocket is
    // NOT facade-reachable: the analytic blind builder (blindThroughHi) needs the cylinder
    // to enter the HI face with its far cap strictly inside the box, i.e. cyl.lo > box.lo;
    // but every facade-built extrude/full-circle body starts at z=0, so cyl.lo == box.lo
    // and the pocket can only ever enter the LO face (deferred). Rather than fake a native
    // blind hole we build BOTH operands under OCCT and run the boolean under native — the
    // engine forwards to OCCT (a fall-through proof: cc_active_engine()==1). The far cap
    // is placed inside by making the box TALLER than the cylinder is deep and cutting from
    // the top: box z∈[0,20], cylinder z∈[8,28] (translated OCCT body) → pocket bottom at
    // z=8, depth 12. result = boxVol − πr²·12 (checked as native==OCCT, forwarded).
    const BuildPair blindHolePair = [box20, cx, cy, r](CCShapeId& a, CCShapeId& b) {
        a = box20();                                  // OCCT box z∈[0,20]
        CCShapeId c0 = buildCylinderZ(cx, cy, r, 20.0);  // OCCT cyl z∈[0,20]
        b = c0 ? cc_translate_shape(c0, 0.0, 0.0, 8.0) : 0;  // z∈[8,28], bottom inside
        if (c0) cc_shape_release(c0);
    };

    // FALLBACK oblique-cyl cut: an OCCT box − an OCCT cylinder ROTATED 30° off the Z
    // axis (about +X through the box centre). The axis is no longer world-aligned, so the
    // native analytic recogniser declines → the engine forwards to OCCT. Not an analytic
    // volume (expectVol=0); the check is native==OCCT (forwarded), curved-mesh bound.
    const BuildPair obliqueCylPair = [box20, cx, cy, r](CCShapeId& a, CCShapeId& b) {
        a = box20();
        b = buildObliqueCylinder(cx, cy, r, /*dz*/ 30.0, /*degrees*/ 30.0, /*rotCentreZ*/ 10.0);
    };
    // FALLBACK sphere-box cut: an OCCT box − an OCCT sphere (r=6, centre (0,10,0),
    // reaching into the box). A sphere is a curved surface with no analytic plane-section
    // family in this slice, so the native builder declines → OCCT.
    const BuildPair sphereBoxPair = [box20](CCShapeId& a, CCShapeId& b) {
        a = box20();
        b = buildSphereY(6.0, 10.0);                  // sphere centred on the box's y-mid
    };

    std::vector<BoolCase> cases = {
        // ── NATIVE: axis-aligned box ⟷ axis-∥ (Z) cylinder, analytic slice ─────────
        // relTol 2e-2 is the deflection-bounded curved-mesh tolerance the roadmap and
        // engine (curvedBooleanVerified, 1% analytic) use, widened a touch because the
        // native engine measures the CANDIDATE's mass/mesh from a chord-approximated
        // cylinder (mass at deflection 0.005, cc_tessellate at 0.01) while the OCCT
        // oracle is B-rep-exact — for `common` the whole result IS the cylinder, so its
        // vol carries the full chord under-fill (~0.1-0.5% here), well inside 2%.
        {"through-hole-cut", 1, Mode::Native, throughHolePair,
         /*expectVol*/ boxVol - cylArea * 20.0, /*defl*/ 0.01, /*relTol*/ 2e-2},
        {"boss-fuse", 0, Mode::Native, bossPair, boxVol + cylArea * 5.0, 0.01, 2e-2},
        {"common", 2, Mode::Native, commonPair, cylArea * 14.0, 0.01, 2e-2},

        // ── FALLBACK: cases OUTSIDE the facade-native analytic slice → OCCT (labelled).
        // blind-hole is a HONEST fallback (a native blind pocket is not facade-reachable,
        // see blindHolePair); oblique-cyl / sphere are genuinely out of the family. ────
        {"blind-hole-cut", 1, Mode::Fallback, blindHolePair,
         /*expectVol*/ boxVol - cylArea * 12.0, /*defl*/ 0.05, /*relTol*/ 5e-2,
         /*curvedFallbackMesh*/ true},
        {"oblique-cyl-cut", 1, Mode::Fallback, obliqueCylPair,
         /*expectVol*/ 0.0, /*defl*/ 0.05, /*relTol*/ 5e-2, /*curvedFallbackMesh*/ true},
        {"sphere-box-cut", 1, Mode::Fallback, sphereBoxPair, 0.0, 0.05, 5e-2, true},
    };

    for (const BoolCase& c : cases) runCase(c);

    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
