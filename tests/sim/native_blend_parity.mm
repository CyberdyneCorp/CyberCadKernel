// SPDX-License-Identifier: Apache-2.0
//
// native_blend_parity.mm — native-vs-OCCT BLEND parity harness, driven entirely
//                          THROUGH the cc_* facade (iOS simulator).
//
// Phase 4 capability #6 (`native-blends` / native fillets-offsets) — simulator
// verification gate 2 (see openspec/NATIVE-REWRITE.md). Like the sibling
// native_boolean_parity.mm this harness exercises the SHIPPING PATH: it calls the
// same public cc_fillet_edges / cc_chamfer_edges / cc_offset_face / cc_shell the app
// calls, once with the OCCT engine active and once with the NativeEngine active, and
// compares the two results.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default) — BRepFilletAPI /
//                       BRepOffsetAPI.
//   cc_set_engine(1)  → NativeEngine. The blend is NATIVE (clean-room planar-polygon
//                       edit, src/native/blend) for the TRACTABLE planar cases and
//                       SELF-VERIFIED (watertight 2-manifold AND a SANE volume sign vs
//                       the original — fillet/chamfer/shell shrink, offset grows/
//                       shrinks) before it is accepted; everything else FALLS BACK to
//                       OCCT (never a wrong/leaky/faked solid).
//
// ── The native/fallback SPLIT this harness asserts (honest, per the spec) ─────────
//
// The NativeEngine can only intercept a blend when the BODY is a NATIVE body (built
// under cc_set_engine(1)); an OCCT body forwards the whole op to OCCT unconditionally.
// And a native blend whose planar-polygon result fails the watertight+volume-sign
// self-verify is DISCARDED — the engine returns a clean error (id 0) rather than emit
// a wrong solid, because it CANNOT hand a native body to OCCT. So this harness drives
// two kinds of case explicitly:
//
//   NATIVE cases — body BUILT UNDER NATIVE, blend run UNDER NATIVE. The native
//   planar-polygon path intercepts, produces a verified watertight solid, and we
//   assert it matches the OCCT oracle. The oracle is the SAME input built + blended
//   under OCCT.
//     · chamfer-edge  : a planar chamfer on one convex box edge (planar cut). EXACT
//                       vs OCCT (axis-aligned box → fp64).
//     · fillet-edge   : a constant-radius rolling-ball fillet on one convex box edge
//                       (faceted cylinder). DEFLECTION-BOUNDED vs OCCT — the native
//                       fillet is a tiled arc, so its volume/area/mesh match the true
//                       OCCT cylinder only within a sagitta bound, not to fp64.
//     · offset-face   : push one planar cap outward along +normal (grow the slab).
//                       EXACT vs OCCT (a bigger axis-aligned box).
//     · shell         : hollow the box to a uniform wall, opening the top face
//                       (offset + BSP-CSG cut). EXACT vs OCCT (nested boxes).
//
//   FALLBACK case — body BUILT UNDER OCCT, blend run UNDER NATIVE. The engine sees an
//   OCCT body and forwards to the OCCT oracle transparently, so native == OCCT EXACTLY
//   (a fall-through proof: cc_active_engine()==1 yet the OCCT result is returned, never
//   faked). This is a configuration OUTSIDE the native tractable domain:
//     · fillet-curved-edge : a constant-radius fillet on a CURVED edge (the circular
//                            rim of a revolved cylinder). A curved adjacent face is
//                            outside the native planar-dihedral domain → the native
//                            builder returns NULL for a native body, and an OCCT body
//                            forwards outright. Verified valid OCCT result, labelled
//                            [fallback]. (Variable-radius fillet and cc_fillet_face are
//                            likewise OCCT-only; the curved-edge fillet is the
//                            representative curved-input fall-through here.)
//
//   SELF-VERIFY GUARD — additionally, a NATIVE-body blend whose native result cannot
//   pass the watertight+volume-sign self-verify must return id 0 (a clean error), NOT
//   a wrong/leaky solid. A native box shelled with a wall THICKER than half the box
//   (the cavity collapses) exercises the guard directly: the native builder cannot
//   produce a sane hollow, and it CANNOT forward a native body to OCCT, so it reports
//   an honest error.
//
// ── Per-case comparisons (native/forwarded result vs OCCT oracle) ─────────────────
//   * cc_mass_properties : volume + area + centroid.  (+ an analytic volume anchor for
//                          the exact chamfer/offset/shell box cases.)
//   * cc_bounding_box    : the two exact B-rep AABBs.
//   * cc_tessellate      : the result mesh is WATERTIGHT (closed 2-manifold) and its
//                          mesh-volume matches the B-rep volume.
//   EXACT tolerance (1e-6 rel/abs) for chamfer / offset / shell (axis-aligned box
//   results — native and OCCT agree to fp round-off). The constant-radius fillet is
//   DEFLECTION-BOUNDED: its native faceted cylinder differs from OCCT's true cylinder,
//   so vol/area/bbox/mesh use a loose sagitta-scaled tolerance and the CENTROID (which
//   the faceting barely moves) a mid tolerance. The curved-edge fallback is a forwarded
//   OCCT result on BOTH sides (same engine both times) so its B-rep props are exact;
//   only its CURVED-wall mesh volume uses a deflection-bounded tolerance and it is not
//   required to be a triangle-level watertight manifold (OCCT meshes each face
//   independently across the boolean/fillet seam).
//
// Output: [NBLEND] PASS/FAIL lines, each tagged [native] or [fallback], with per-case
// volume deltas; then "== N passed, M failed ==". Flushes stdout and
// std::_Exit(failed?1:0) — the trimmed static-OCCT build's static teardown is not
// exit-clean (same rationale as the sibling sim harnesses; every id is released before
// exit).
//
// Build: scripts/run-sim-native-blend.sh — compiles this harness + the whole facade/
// core/engine (incl. NativeEngine + OCCT adapter) + src/native/**, links the OCCT libs,
// spawns on a booted simulator. On run-sim-suite.sh's SKIP list (its own main(),
// competes for the entry point).

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
        std::printf("[NBLEND] PASS  %-10s %-30s %s\n", tag, label.c_str(), detail);
    } else {
        ++g_failed;
        std::printf("[NBLEND] FAIL  %-10s %-30s %s\n", tag, label.c_str(), detail);
    }
}

// ── restore the default (OCCT) engine no matter how we leave scope ──────────────
struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

// ── mesh-level helpers (operate on a CCMesh returned by cc_tessellate) ──────────

// A mesh is watertight iff it is a closed 2-manifold: every undirected edge is shared
// by exactly two triangles. Edges are counted over POSITION-WELDED vertex ids (a 1e-7
// quantised grid), not raw mesh indices: the native tessellator welds shared corners
// but OCCT's BRepMesh emits each face's triangulation with its OWN copy of every
// boundary vertex, so a raw-index edge map would report every shared solid edge as
// unmatched even for a perfectly closed solid. Welding coincident positions first
// makes the check engine-agnostic — it verifies GEOMETRIC surface closure, which is
// what "watertight" means here — so it holds for native results and OCCT-forwarded
// fallbacks alike. (Identical to the boolean harness helper.)
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

// Signed volume of a closed triangle mesh via the divergence theorem (⅙ Σ aᵢ·(bᵢ×cᵢ));
// magnitude is the enclosed volume for an outward-wound solid.
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

// ── operand + feature-id helpers (each uses the ACTIVE engine) ──────────────────

// An axis-aligned box [0,sx]×[0,sy]×[0,sz] as a square profile extruded by sz. Built
// with WHATEVER engine is active (native → native prism; OCCT → OCCT prism).
CCShapeId buildBox(double sx, double sy, double sz) {
    const double rect[] = {0.0, 0.0, sx, 0.0, sx, sy, 0.0, sy};
    return cc_solid_extrude(rect, 4, sz);
}

// A solid cylinder (radius r, height h along +Y about the Y axis): revolve the
// rectangle x∈[0,r], y∈[0,h] a full turn. Its top/bottom rims are CURVED edges
// (circles) — filleting one is outside the native planar-dihedral domain → fallback.
CCShapeId buildCylinder(double r, double h) {
    const double rect[] = {0.0, 0.0, r, 0.0, r, h, 0.0, h};
    return cc_solid_revolve(rect, 4, 2.0 * kPi);
}

// Face id (1-based, as cc_shell / cc_offset_face expect) of the box face whose every
// vertex lies on the plane axis==value (axis 0=x,1=y,2=z), resolved at run time from
// cc_face_meshes so the pick is independent of sub-shape ordering AND consistent
// across the two engines (both are queried the same way). 0 if none.
int findPlanarFace(CCShapeId body, int axis, double value, double tol) {
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.1, &faces);
    int found = 0;
    for (int i = 0; i < n && found == 0; ++i) {
        const CCFaceMesh& f = faces[i];
        if (f.vertexCount <= 0 || f.vertices == nullptr) continue;
        bool onPlane = true;
        for (int v = 0; v < f.vertexCount && onPlane; ++v)
            if (std::fabs(f.vertices[v * 3 + axis] - value) > tol) onPlane = false;
        if (onPlane) found = f.faceId;
    }
    cc_face_meshes_free(faces, n);
    return found;
}

// Edge id (1-based, as cc_fillet_edges / cc_chamfer_edges expect) of the edge whose
// endpoints both lie on the intersection of the two axis planes {axisA==valA} ∩
// {axisB==valB} — i.e. the box edge running along the third axis at that corner. On a
// 10×10×10 box the vertical edge at x=0,y=0 is a convex 90° planar dihedral (shared by
// the x=0 and y=0 side faces). Resolved from cc_edge_polylines so the pick is
// engine-independent. 0 if none.
int findAxisEdge(CCShapeId body, int axisA, double valA, int axisB, double valB, double tol) {
    CCEdgePolyline* edges = nullptr;
    const int n = cc_edge_polylines(body, &edges);
    int found = 0;
    for (int i = 0; i < n && found == 0; ++i) {
        const CCEdgePolyline& e = edges[i];
        if (e.pointCount < 2 || e.points == nullptr) continue;
        bool onLine = true;
        for (int p = 0; p < e.pointCount && onLine; ++p) {
            if (std::fabs(e.points[p * 3 + axisA] - valA) > tol ||
                std::fabs(e.points[p * 3 + axisB] - valB) > tol)
                onLine = false;
        }
        if (onLine) found = e.edgeId;
    }
    cc_edge_polylines_free(edges, n);
    return found;
}

// The curved rim edge of a cylinder built by buildCylinder(r,h) about +Y: the top
// circle at y==h (a full circular edge — cc_edge_polylines returns it as one polyline).
// Chosen as the CURVED input that must fall through to OCCT. 0 if none.
int findCurvedRimEdge(CCShapeId body, double yValue, double tol) {
    CCEdgePolyline* edges = nullptr;
    const int n = cc_edge_polylines(body, &edges);
    int found = 0;
    for (int i = 0; i < n && found == 0; ++i) {
        const CCEdgePolyline& e = edges[i];
        if (e.pointCount < 3 || e.points == nullptr) continue;
        bool onY = true;
        for (int p = 0; p < e.pointCount && onY; ++p)
            if (std::fabs(e.points[p * 3 + 1] - yValue) > tol) onY = false;
        if (onY) found = e.edgeId;  // a full circle on the y=yValue plane (≥3 samples)
    }
    cc_edge_polylines_free(edges, n);
    return found;
}

// ── one blend result snapshot (id + queried props) ─────────────────────────────
struct Snapshot {
    CCShapeId id = 0;
    CCMassProps mass{0, 0, 0, 0, 0, 0};
    double bb[6] = {0, 0, 0, 0, 0, 0};
    int bbok = 0;
    bool activeNative = false;
};

// A blend case: build a body with `buildEngine`, resolve the feature id ON THAT body
// (so the id is valid for the same engine), run one blend op UNDER `blendEngine`, and
// snapshot the result props.
enum class Mode { Native, Fallback };

// The op closure receives the body id and returns the blended id (0 on failure). The
// feature-id resolver runs on the freshly-built body.
using FeatureBuild = std::function<CCShapeId()>;                       // build body (active engine)
using BlendOp = std::function<CCShapeId(CCShapeId body)>;              // run the blend on it

struct BlendCase {
    std::string name;
    Mode mode;
    FeatureBuild build;   // builds the body with the active engine
    BlendOp op;           // resolves the feature id + runs the cc_* blend
    bool exact;           // true → axis-aligned box result: exact fp64 vs OCCT + anchor
    double expectVol;     // analytic expected result volume (checked only when exact)
    double deflection;    // mesh deflection for the watertight/volume check
    // A DEFLECTION-BOUNDED result (the native faceted-cylinder fillet, or a curved
    // OCCT-fallback mesh): vol/area/bbox/mesh are compared with a loose tolerance
    // rather than fp64, because the faceting/independent-meshing genuinely differs.
    double relTol = 1e-6;      // vol/area rel tolerance
    double linTol = 1e-6;      // bbox / centroid abs tolerance
    double meshTol = 1e-6;     // mesh-volume rel tolerance
    // A CURVED OCCT-fallback mesh cannot be welded into a triangle-level 2-manifold
    // across the fillet/boolean seam (OCCT meshes each face independently); assert the
    // mesh VOLUME bound only, not watertightness. Native results and planar fallbacks
    // must still close.
    bool curvedFallbackMesh = false;
};

// Build the body under `buildEngine`, then run the blend under `blendEngine`, returning
// the result id + queried props. Releases the body. For a Native case buildEngine ==
// blendEngine == 1 (native intercepts); for a Fallback case buildEngine == 0,
// blendEngine == 1 (an OCCT body → the engine forwards to OCCT).
Snapshot buildAndBlend(const FeatureBuild& build, const BlendOp& op, int buildEngine,
                       int blendEngine) {
    cc_set_engine(buildEngine);
    const CCShapeId body = build();
    cc_set_engine(blendEngine);
    Snapshot s;
    s.activeNative = cc_active_engine() == 1;
    if (body != 0) {
        s.id = op(body);
        if (s.id != 0) {
            s.mass = cc_mass_properties(s.id);
            s.bbok = cc_bounding_box(s.id, s.bb);
        }
    }
    if (body) cc_shape_release(body);
    return s;
}

// ── run one blend case under both engines and compare ───────────────────────────
void runCase(const BlendCase& c) {
    char detail[512];
    const char* tag = (c.mode == Mode::Native) ? "native" : "fallback";

    // 1) OCCT oracle: body + blend both under OCCT (the reference answer).
    const Snapshot oracle = buildAndBlend(c.build, c.op, /*build*/ 0, /*blend*/ 0);
    if (oracle.id == 0 || oracle.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "OCCT oracle build/blend failed: %s",
                      cc_last_error());
        record(false, tag, c.name, detail);
        cc_set_engine(0);
        if (oracle.id) cc_shape_release(oracle.id);
        return;
    }

    // 2) Candidate: Native case → body+blend under native (planar edit intercepts);
    //    Fallback case → body under OCCT, blend under native (the engine forwards).
    const int buildEng = (c.mode == Mode::Native) ? 1 : 0;
    const Snapshot cand = buildAndBlend(c.build, c.op, buildEng, /*blend*/ 1);
    const CCMesh cMesh =
        cand.id ? cc_tessellate(cand.id, c.deflection) : CCMesh{nullptr, 0, nullptr, 0};

    if (cand.id == 0 || cand.mass.valid == 0) {
        std::snprintf(detail, sizeof detail,
                      "native active=%d, blend->0 (%s) — expected a valid %s result",
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
        !c.exact || std::fabs(oracle.mass.volume - c.expectVol) / c.expectVol < 1e-6;
    const bool massOk = cand.activeNative && volRel < c.relTol && areaRel < c.relTol &&
                        cMax < c.linTol && anchorOk;
    if (c.exact)
        std::snprintf(detail, sizeof detail,
                      "vol o=%.6g n=%.6g rel=%.2e (expect %.6g) | area rel=%.2e | cΔ=%.2e",
                      oracle.mass.volume, cand.mass.volume, volRel, c.expectVol, areaRel, cMax);
    else
        std::snprintf(detail, sizeof detail,
                      "vol o=%.6g n=%.6g rel=%.2e | area rel=%.2e | cΔ=%.2e%s",
                      oracle.mass.volume, cand.mass.volume, volRel, areaRel, cMax,
                      c.mode == Mode::Fallback ? " (forwarded=OCCT)" : " (deflection-bounded)");
    record(massOk, tag, c.name + " mass", detail);

    // ── bounding boxes vs the oracle ──────────────────────────────────────────────
    const double bbDelta = maxBBoxCornerDelta(oracle.bb, cand.bb);
    const bool bbOk = (oracle.bbok == 1) && (cand.bbok == 1) && (bbDelta < c.linTol);
    std::snprintf(detail, sizeof detail, "maxCornerΔ=%.2e (tol=%.0e)", bbDelta, c.linTol);
    record(bbOk, tag, c.name + " bbox", detail);

    // ── the RESULT mesh is watertight + its volume matches the B-rep ─────────────
    const bool haveMesh = cMesh.triangleCount > 0;
    const bool wt = haveMesh && meshWatertight(cMesh);
    const double meshVol = haveMesh ? meshVolume(cMesh) : 0.0;
    const double meshVolRel = (haveMesh && cand.mass.volume > 0.0)
                                  ? std::fabs(meshVol - cand.mass.volume) / cand.mass.volume
                                  : 1.0;
    // Watertightness required for every case EXCEPT a curved OCCT fallback mesh (OCCT's
    // independent per-face meshing cannot weld the fillet/boolean seam — assert the
    // mesh-volume bound only). All native results, and planar fallbacks, must close.
    const bool tessOk = haveMesh && meshVolRel < c.meshTol && (c.curvedFallbackMesh || wt);
    std::snprintf(detail, sizeof detail, "watertight=%d tris=%d meshVolRel=%.2e%s", wt ? 1 : 0,
                  cMesh.triangleCount, meshVolRel,
                  c.curvedFallbackMesh ? " (curved fallback: volume-bound only)" : "");
    record(tessOk, tag, c.name + " tessellate", detail);

    if (haveMesh) cc_mesh_free(cMesh);
    cc_set_engine(0);
    cc_shape_release(cand.id);
    cc_shape_release(oracle.id);
}

// ── the SELF-VERIFY GUARD in action ─────────────────────────────────────────────
// A NATIVE-body blend whose native result cannot pass the watertight+volume-sign
// self-verify must return id 0 (a clean error), NEVER a wrong/leaky solid. A native
// 10×10×10 box shelled with wall thickness 6 (> half the 10 extent) has NO valid
// hollow cavity: the inward half-spaces cross, so the native builder produces no sane
// wall and the engine cannot forward a native body to OCCT → an honest error.
void runNativeGuard() {
    char detail[512];
    cc_set_engine(1);
    const CCShapeId box = buildBox(10.0, 10.0, 10.0);
    const int top = box ? findPlanarFace(box, 2, 10.0, 1e-6) : 0;
    const bool activeNative = cc_active_engine() == 1;
    const int faceIds[1] = {top};
    // Wall 6 mm on a 10 mm box: 6 + 6 > 10 on every axis → the inset cavity collapses.
    const CCShapeId shelled = (box && top) ? cc_shell(box, faceIds, 1, 6.0) : -1;
    const bool ok = activeNative && (box != 0) && (top != 0) && (shelled == 0);
    std::snprintf(detail, sizeof detail,
                  "native active=%d, native shell(t=6 on 10³ box)->id=%ld (expect 0) — "
                  "self-verify rejected: %s",
                  activeNative ? 1 : 0, static_cast<long>(shelled), cc_last_error());
    record(ok, "native", "self-verify-guard", detail);
    cc_set_engine(0);
    if (shelled > 0) cc_shape_release(shelled);
    if (box) cc_shape_release(box);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT blend parity (through the cc_* facade)\n");

    // Base body for the planar cases: a 10×10×10 box (vol 1000, area 600, every edge 10,
    // every dihedral a convex 90°). buildBox uses the ACTIVE engine.
    const FeatureBuild box10 = [] { return buildBox(10.0, 10.0, 10.0); };

    // ── chamfer: cut a d=1 planar chamfer off ONE convex vertical edge (x=0,y=0). The
    //    corner wedge removed per unit edge length is ½d² = 0.5, over the length-10 edge
    //    → 5. Result volume 1000 − 5 = 995 (EXACT vs OCCT, axis-aligned planar cut).
    const BlendOp chamferEdge = [](CCShapeId body) -> CCShapeId {
        const int e = findAxisEdge(body, 0, 0.0, 1, 0.0, 1e-6);  // vertical edge at x=0,y=0
        if (e == 0) return 0;
        const int ids[1] = {e};
        return cc_chamfer_edges(body, ids, 1, 1.0);
    };

    // ── fillet: constant radius r=1 rolling-ball on the SAME convex vertical edge. The
    //    removed volume per unit length is r²(1 − π/4); over length 10 → 10·(1 − π/4) ≈
    //    2.146, so the true volume ≈ 997.854. The native fillet is a FACETED cylinder, so
    //    vol/area/bbox/mesh are DEFLECTION-BOUNDED (not fp64) vs OCCT's true cylinder.
    const BlendOp filletEdge = [](CCShapeId body) -> CCShapeId {
        const int e = findAxisEdge(body, 0, 0.0, 1, 0.0, 1e-6);
        if (e == 0) return 0;
        const int ids[1] = {e};
        return cc_fillet_edges(body, ids, 1, 1.0);
    };

    // ── offset-face: push the TOP face (+Z at z=10) outward by +5 → a 10×10×15 box,
    //    volume 1500 (EXACT vs OCCT — a grown axis-aligned slab).
    const BlendOp offsetTop = [](CCShapeId body) -> CCShapeId {
        const int top = findPlanarFace(body, 2, 10.0, 1e-6);
        if (top == 0) return 0;
        return cc_offset_face(body, top, 5.0);
    };

    // ── shell: hollow to a uniform 1 mm wall with the TOP face open. The inner cavity is
    //    8×8×9 = 576 (walls of 1 on the four sides + the floor, top open), so the wall
    //    volume is 1000 − 576 = 424 (EXACT vs OCCT — nested axis-aligned boxes).
    const BlendOp shellTopOpen = [](CCShapeId body) -> CCShapeId {
        const int top = findPlanarFace(body, 2, 10.0, 1e-6);
        if (top == 0) return 0;
        const int ids[1] = {top};
        return cc_shell(body, ids, 1, 1.0);
    };

    // ── fillet on a CURVED edge → OCCT fallthrough. A cylinder r=4 h=10 about +Y: fillet
    //    its top rim (the circle at y=10) with r=1. A curved adjacent face is outside the
    //    native planar-dihedral domain, so the native builder returns NULL for a native
    //    body; here the body is BUILT UNDER OCCT so the engine forwards outright. Not an
    //    analytic volume (a toroidal blend on a round rim) → exact=false; the check is
    //    native==oracle (same OCCT engine both times) with a deflection-bounded curved
    //    mesh.
    const FeatureBuild cyl = [] { return buildCylinder(4.0, 10.0); };
    const BlendOp filletCurvedRim = [](CCShapeId body) -> CCShapeId {
        const int rim = findCurvedRimEdge(body, 10.0, 1e-6);  // top circle at y=10
        if (rim == 0) return 0;
        const int ids[1] = {rim};
        return cc_fillet_edges(body, ids, 1, 1.0);
    };

    std::vector<BlendCase> cases = {
        // ── NATIVE: planar chamfer / offset / shell — EXACT vs OCCT ──────────────
        {"chamfer-edge", Mode::Native, box10, chamferEdge, /*exact*/ true, 995.0,
         /*defl*/ 0.05},
        {"offset-face", Mode::Native, box10, offsetTop, true, 1500.0, 0.05},
        {"shell-open-top", Mode::Native, box10, shellTopOpen, true, 424.0, 0.05},

        // ── NATIVE: constant-radius fillet — DEFLECTION-BOUNDED vs OCCT ───────────
        // The native faceted cylinder differs from OCCT's true cylinder, so loosen the
        // tolerances: volume/area to ~1% (the faceting slightly over/under-fills the
        // arc), bbox/centroid to ~0.02 (facet chords sit inside the true arc), mesh to
        // ~2%. Still asserts native active + watertight.
        {"fillet-edge", Mode::Native, box10, filletEdge, /*exact*/ false, 0.0, /*defl*/ 0.02,
         /*relTol*/ 1e-2, /*linTol*/ 2e-2, /*meshTol*/ 2e-2},

        // ── FALLBACK: fillet on a CURVED edge → OCCT (native NULL for native body,
        //    forwarded for an OCCT body). native==OCCT exactly on B-rep props; the
        //    curved-wall mesh uses a deflection-bounded volume bound and is not required
        //    to be a triangle-level manifold.
        {"fillet-curved-edge", Mode::Fallback, cyl, filletCurvedRim, /*exact*/ false, 0.0,
         /*defl*/ 0.02, /*relTol*/ 1e-6, /*linTol*/ 1e-6, /*meshTol*/ 5e-2,
         /*curvedFallbackMesh*/ true},
    };

    for (const BlendCase& c : cases) runCase(c);
    runNativeGuard();

    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
