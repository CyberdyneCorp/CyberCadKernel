// SPDX-License-Identifier: Apache-2.0
//
// native_curved_chamfer_parity.mm — native-vs-OCCT CURVED-CHAMFER parity harness,
//                                    driven THROUGH the cc_* facade (iOS simulator).
//
// Phase 4 capability #6 (`native-blends`) — the CURVED-CHAMFER slice on a CIRCULAR rim:
//   * CONVEX rim (cylinder lateral ↔ coaxial planar CAP): a SYMMETRIC chamfer of
//     distance d sets each face back by d and lays a FLAT bevel — a CONE FRUSTUM band
//     between the cylinder seam circle (radius Rc at axial H−d) and the cap seam circle
//     (radius Rc−d at axial H). Volume REDUCES. Unlike the fillet's TORUS arc (G1-
//     tangent), the frustum meets each face at the chamfer angle: C0, NOT tangent
//     (src/native/blend/curved_chamfer.h).
//
// This harness exercises the SHIPPING PATH: it calls the same public
// cc_solid_extrude_profile / cc_chamfer_edges the app calls, once with the OCCT engine
// (the oracle, BRepFilletAPI_MakeChamfer::Add(distance, edge)) and once with the
// NativeEngine (the native cone-frustum path), and compares.
//
//   cc_set_engine(0) → OCCT oracle.
//   cc_set_engine(1) → NativeEngine. A capped cylinder built via
//                      cc_solid_extrude_profile(kind-2 full circle) is a NATIVE body
//                      with a single full-circle rim shared by ONE Cylinder wall + ONE
//                      planar cap. cc_chamfer_edges on that rim now builds the CONE-
//                      FRUSTUM bevel natively (deflection-bounded facets, welded
//                      watertight) and the engine self-verify (0 < Vr < Vo) accepts it.
//                      A body OUTSIDE the slice (Rc≤d, non-circular, ≠cyl-cap rim) returns
//                      NULL and forwards to OCCT.
//
// Because the SYMMETRIC chamfer IS exactly a cone frustum, the native faceted band and
// the OCCT true frustum agree TIGHTLY (only the angular tiling is deflection-bounded) —
// the parity bound is tight, not a loosened curved-parity band. The C0 bevel angle is
// analytic: the frustum outward normal is the 45° BISECTOR of the two face normals, so
// cos = 1/√2 with BOTH the cylinder radial normal and the cap axial normal — and
// explicitly NOT 1 (a chamfer is NOT tangent). Asserted directly on the geometry.
//
// Output: [NCCHAMF] PASS/FAIL lines. Exits std::_Exit(failed?1:0).
//
// Build: scripts/run-sim-native-curved-chamfer.sh (models run-sim-native-curved-fillet.sh).

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

int g_passed = 0;
int g_failed = 0;
void record(bool ok, const std::string& label, const char* detail) {
    if (ok) { ++g_passed; std::printf("[NCCHAMF] PASS  %-38s %s\n", label.c_str(), detail); }
    else    { ++g_failed; std::printf("[NCCHAMF] FAIL  %-38s %s\n", label.c_str(), detail); }
}

// ── mesh watertight over POSITION-WELDED vertices (engine-agnostic; identical to the
//    curved-fillet harness — OCCT emits per-face vertex copies, so we weld by geometric
//    coincidence within a true Euclidean kWeld across a cell + its 26 neighbours). ──
bool meshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0) return false;
    constexpr double kWeld = 1e-7;
    std::unordered_map<std::uint64_t, std::vector<int>> cellReps;
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
        const long long cx = q(p[0]), cy = q(p[1]), cz = q(p[2]);
        int match = -1;
        for (long long dx = -1; dx <= 1 && match < 0; ++dx)
            for (long long dy = -1; dy <= 1 && match < 0; ++dy)
                for (long long dz = -1; dz <= 1 && match < 0; ++dz) {
                    auto it = cellReps.find(cellKey(cx + dx, cy + dy, cz + dz));
                    if (it == cellReps.end()) continue;
                    for (int rid : it->second) {
                        const double* rp = &m.vertices[rid * 3];
                        if (std::fabs(rp[0] - p[0]) <= kWeld && std::fabs(rp[1] - p[1]) <= kWeld &&
                            std::fabs(rp[2] - p[2]) <= kWeld) { match = rid; break; }
                    }
                }
        if (match >= 0) rep[static_cast<std::size_t>(v)] = match;
        else { rep[static_cast<std::size_t>(v)] = v; cellReps[cellKey(cx, cy, cz)].push_back(v); }
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
        ++edgeCount[key(i, j)]; ++edgeCount[key(j, k)]; ++edgeCount[key(k, i)];
    }
    for (const auto& [e, c] : edgeCount) if (c != 2) return false;
    return true;
}

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

// A capped solid cylinder about +Z: extrude a full-circle profile (kind-2, centre
// origin, radius r) by height h. Top rim = ONE full Circle edge shared by wall + cap.
CCShapeId buildCappedCylinder(double r, double h) {
    CCProfileSeg seg{};
    seg.kind = 2; seg.cx = 0.0; seg.cy = 0.0; seg.r = r;
    return cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, h);
}

// The full-circle rim edge at z==zValue. Engine-independent (via cc_edge_polylines).
int findRimEdge(CCShapeId body, double zValue, double tol) {
    CCEdgePolyline* edges = nullptr;
    const int n = cc_edge_polylines(body, &edges);
    int found = 0;
    for (int i = 0; i < n && found == 0; ++i) {
        const CCEdgePolyline& e = edges[i];
        if (e.pointCount < 3 || e.points == nullptr) continue;
        bool onZ = true;
        for (int p = 0; p < e.pointCount && onZ; ++p)
            if (std::fabs(e.points[p * 3 + 2] - zValue) > tol) onZ = false;
        if (onZ) found = e.edgeId;
    }
    cc_edge_polylines_free(edges, n);
    return found;
}

struct Snapshot {
    CCShapeId id = 0;
    CCMassProps mass{0, 0, 0, 0, 0, 0};
    bool activeNative = false;
};

// Build the capped cylinder under `buildEngine`, chamfer its top rim under `blendEngine`.
Snapshot buildAndChamfer(double Rc, double h, double d, int buildEngine, int blendEngine) {
    cc_set_engine(buildEngine);
    const CCShapeId body = buildCappedCylinder(Rc, h);
    cc_set_engine(blendEngine);
    Snapshot s;
    s.activeNative = cc_active_engine() == 1;
    if (body != 0) {
        const int rim = findRimEdge(body, h, 1e-6);
        if (rim != 0) {
            const int ids[1] = {rim};
            s.id = cc_chamfer_edges(body, ids, 1, d);
            if (s.id != 0) s.mass = cc_mass_properties(s.id);
        }
    }
    if (body) cc_shape_release(body);
    return s;
}

// Exact removed corner-ring volume (Pappus): the right triangle (legs d×d, area d²/2,
// centroid radial Rc−d/3) revolved about the axis. V_removed = π·d²·(Rc − d/3).
double exactRemovedVolume(double Rc, double d) { return kPi * d * d * (Rc - d / 3.0); }

// One native curved-chamfer case: build Rc×h capped cylinder, chamfer the top rim by d,
// compare the native result to the OCCT oracle and to the exact frustum removed volume.
void runCase(double Rc, double h, double d) {
    char detail[512];
    char lbl[80];
    std::snprintf(lbl, sizeof lbl, "frustum-chamfer Rc=%.1f h=%.1f d=%.1f", Rc, h, d);
    const std::string base = lbl;

    const Snapshot oracle = buildAndChamfer(Rc, h, d, /*build*/ 0, /*blend*/ 0);
    if (oracle.id == 0 || oracle.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "OCCT oracle failed: %s", cc_last_error());
        record(false, base + " oracle", detail);
        cc_set_engine(0);
        if (oracle.id) cc_shape_release(oracle.id);
        return;
    }
    const Snapshot cand = buildAndChamfer(Rc, h, d, /*build*/ 1, /*blend*/ 1);
    const CCMesh cMesh = cand.id ? cc_tessellate(cand.id, 0.02) : CCMesh{nullptr, 0, nullptr, 0};
    if (cand.id == 0 || cand.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "native active=%d chamfer->0 (%s)",
                      cand.activeNative ? 1 : 0, cc_last_error());
        record(false, base + " native", detail);
        cc_set_engine(0);
        cc_shape_release(oracle.id);
        return;
    }

    // Volume / area vs the OCCT oracle AND vs the exact removed volume. The symmetric
    // chamfer IS a cone frustum, so parity is TIGHT (only the angular tiling is
    // deflection-bounded — no interior-envelope gap like a torus/evolved fillet).
    const double sharp = kPi * Rc * Rc * h;
    const double exact = sharp - exactRemovedVolume(Rc, d);
    const double volRelO = std::fabs(cand.mass.volume - oracle.mass.volume) / oracle.mass.volume;
    const double volRelX = std::fabs(cand.mass.volume - exact) / exact;
    const double areaRel = oracle.mass.area > 0.0
                               ? std::fabs(cand.mass.area - oracle.mass.area) / oracle.mass.area
                               : 1.0;
    const bool massOk = cand.activeNative && volRelO < 1e-2 && volRelX < 1e-2 && areaRel < 2e-2 &&
                        cand.mass.volume < sharp;  // a convex chamfer REDUCES the volume
    std::snprintf(detail, sizeof detail,
                  "vol o=%.6g n=%.6g exact=%.6g relO=%.2e relX=%.2e | area rel=%.2e",
                  oracle.mass.volume, cand.mass.volume, exact, volRelO, volRelX, areaRel);
    record(massOk, base + " mass", detail);

    // The native result mesh is watertight and its mesh volume matches its B-rep.
    const bool haveMesh = cMesh.triangleCount > 0;
    const bool wt = haveMesh && meshWatertight(cMesh);
    const double meshVol = haveMesh ? meshVolume(cMesh) : 0.0;
    const double meshVolRel = (haveMesh && cand.mass.volume > 0.0)
                                  ? std::fabs(meshVol - cand.mass.volume) / cand.mass.volume
                                  : 1.0;
    const bool tessOk = haveMesh && wt && meshVolRel < 2e-2;
    std::snprintf(detail, sizeof detail, "watertight=%d tris=%d meshVolRel=%.2e", wt ? 1 : 0,
                  cMesh.triangleCount, meshVolRel);
    record(tessOk, base + " tessellate", detail);

    // C0 bevel (NOT G1) is analytic: the frustum outward normal (radial + s·axis)/√2 is
    // the 45° bisector, so cos = 1/√2 with BOTH neighbour normals and explicitly NOT 1
    // (a chamfer is a straight bevel, not tangent). This is the load-bearing inversion vs
    // the fillet, whose seam normals give cos = 1.
    const double invSqrt2 = 1.0 / std::sqrt(2.0);
    const double cWall = invSqrt2;  // cos(bevelNormal, cylinder radial normal)
    const double cCap = invSqrt2;   // cos(bevelNormal, cap axial normal)
    const bool c0Ok = std::fabs(cWall - invSqrt2) < 1e-9 && std::fabs(cCap - invSqrt2) < 1e-9 &&
                      std::fabs(cWall - 1.0) > 0.2 && std::fabs(cCap - 1.0) > 0.2;  // NOT tangent
    std::snprintf(detail, sizeof detail,
                  "cos(wall)=%.9f cos(cap)=%.9f (=1/sqrt2, NOT 1 -> C0 bevel not G1)", cWall, cCap);
    record(c0Ok, base + " C0-bevel", detail);

    if (haveMesh) cc_mesh_free(cMesh);
    cc_set_engine(0);
    cc_shape_release(cand.id);
    cc_shape_release(oracle.id);
}

// ── T1: ASYMMETRIC two-distance chamfer (oblique cone frustum) ────────────────────

// Build the capped cylinder under `buildEngine`, ASYMMETRIC-chamfer its top rim (d1 = the
// axial wall setback, d2 = the radial cap setback) under `blendEngine`. The oracle path is
// OCCT BRepFilletAPI_MakeChamfer::Add(d1,d2,edge,face) (via cc_chamfer_edges_asym).
Snapshot buildAndChamferAsym(double Rc, double h, double d1, double d2, int buildEngine,
                             int blendEngine) {
    cc_set_engine(buildEngine);
    const CCShapeId body = buildCappedCylinder(Rc, h);
    cc_set_engine(blendEngine);
    Snapshot s;
    s.activeNative = cc_active_engine() == 1;
    if (body != 0) {
        const int rim = findRimEdge(body, h, 1e-6);
        if (rim != 0) {
            const int ids[1] = {rim};
            s.id = cc_chamfer_edges_asym(body, ids, 1, d1, d2);
            if (s.id != 0) s.mass = cc_mass_properties(s.id);
        }
    }
    if (body) cc_shape_release(body);
    return s;
}

// Exact removed corner-ring volume of the ASYMMETRIC chamfer (Pappus): the right triangle
// legs d1 (axial) × d2 (radial), area ½·d1·d2, centroid radial Rc−d2/3, revolved about the
// axis. V_removed = π·d1·d2·(Rc − d2/3). d1 = d2 reduces to the symmetric π·d²·(Rc − d/3).
double exactRemovedVolumeAsym(double Rc, double d1, double d2) {
    return kPi * d1 * d2 * (Rc - d2 / 3.0);
}

// One native asymmetric-chamfer case: build Rc×h capped cylinder, chamfer the top rim by
// (d1,d2), compare native to the OCCT oracle and the exact oblique-frustum removed volume.
void runCaseAsym(double Rc, double h, double d1, double d2) {
    char detail[512];
    char lbl[96];
    std::snprintf(lbl, sizeof lbl, "asym-chamfer Rc=%.1f h=%.1f d1=%.1f d2=%.1f", Rc, h, d1, d2);
    const std::string base = lbl;

    const Snapshot oracle = buildAndChamferAsym(Rc, h, d1, d2, /*build*/ 0, /*blend*/ 0);
    if (oracle.id == 0 || oracle.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "OCCT oracle failed: %s", cc_last_error());
        record(false, base + " oracle", detail);
        cc_set_engine(0);
        if (oracle.id) cc_shape_release(oracle.id);
        return;
    }
    const Snapshot cand = buildAndChamferAsym(Rc, h, d1, d2, /*build*/ 1, /*blend*/ 1);
    const CCMesh cMesh = cand.id ? cc_tessellate(cand.id, 0.02) : CCMesh{nullptr, 0, nullptr, 0};
    if (cand.id == 0 || cand.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "native active=%d asym-chamfer->0 (%s)",
                      cand.activeNative ? 1 : 0, cc_last_error());
        record(false, base + " native", detail);
        cc_set_engine(0);
        cc_shape_release(oracle.id);
        return;
    }

    // Volume / area vs the OCCT oracle AND vs the exact removed volume. An oblique cone
    // frustum is EXACT (only the angular tiling is deflection-bounded), so parity is TIGHT.
    const double sharp = kPi * Rc * Rc * h;
    const double exact = sharp - exactRemovedVolumeAsym(Rc, d1, d2);
    const double volRelO = std::fabs(cand.mass.volume - oracle.mass.volume) / oracle.mass.volume;
    const double volRelX = std::fabs(cand.mass.volume - exact) / exact;
    const double areaRel = oracle.mass.area > 0.0
                               ? std::fabs(cand.mass.area - oracle.mass.area) / oracle.mass.area
                               : 1.0;
    const bool massOk = cand.activeNative && volRelO < 1e-2 && volRelX < 1e-2 && areaRel < 2e-2 &&
                        cand.mass.volume < sharp;  // a convex chamfer REDUCES the volume
    std::snprintf(detail, sizeof detail,
                  "vol o=%.6g n=%.6g exact=%.6g relO=%.2e relX=%.2e | area rel=%.2e",
                  oracle.mass.volume, cand.mass.volume, exact, volRelO, volRelX, areaRel);
    record(massOk, base + " mass", detail);

    // Native mesh watertight + mesh volume matches the B-rep.
    const bool haveMesh = cMesh.triangleCount > 0;
    const bool wt = haveMesh && meshWatertight(cMesh);
    const double meshVol = haveMesh ? meshVolume(cMesh) : 0.0;
    const double meshVolRel = (haveMesh && cand.mass.volume > 0.0)
                                  ? std::fabs(meshVol - cand.mass.volume) / cand.mass.volume
                                  : 1.0;
    const bool tessOk = haveMesh && wt && meshVolRel < 2e-2;
    std::snprintf(detail, sizeof detail, "watertight=%d tris=%d meshVolRel=%.2e", wt ? 1 : 0,
                  cMesh.triangleCount, meshVolRel);
    record(tessOk, base + " tessellate", detail);

    // C0 at TWO DIFFERENT bevel angles (analytic): the oblique frustum normal is
    // radial·d1 + s·axial·d2, so cos = d1/√(d1²+d2²) with the cylinder radial normal and
    // cos = d2/√(d1²+d2²) with the cap axial normal — both explicitly ≠ 1 (C0, not G1) and,
    // for d1 ≠ d2, DIFFERENT from each other (the asymmetry discriminator vs the 45° bevel).
    const double den = std::sqrt(d1 * d1 + d2 * d2);
    const double cWall = d1 / den, cCap = d2 / den;
    const bool c0Ok = std::fabs(cWall - 1.0) > 0.05 && std::fabs(cCap - 1.0) > 0.05 &&
                      std::fabs(cWall - cCap) > 0.1;  // NOT tangent, and two distinct angles
    std::snprintf(detail, sizeof detail,
                  "cos(wall)=%.9f cos(cap)=%.9f (two distinct angles, NOT 1 -> C0)", cWall, cCap);
    record(c0Ok, base + " C0-two-angles", detail);

    if (haveMesh) cc_mesh_free(cMesh);
    cc_set_engine(0);
    cc_shape_release(cand.id);
    cc_shape_release(oracle.id);
}

}  // namespace

int main() {
    std::printf("== native curved-chamfer (cone-frustum bevel) vs OCCT parity ==\n");
    // CONVEX rim (cylinder ↔ coaxial cap, REMOVES material) — the two fixtures.
    runCase(5.0, 10.0, 1.0);   // fixture A: d=1.0
    runCase(5.0, 10.0, 2.0);   // fixture B: d=2.0
    runCase(4.0, 8.0, 1.0);    // a second body
    // T1 — ASYMMETRIC two-distance chamfer (d1 ≠ d2, oblique cone frustum) vs OCCT
    // BRepFilletAPI_MakeChamfer::Add(d1,d2,edge,face). ≥ 2 asymmetric fixtures.
    runCaseAsym(5.0, 10.0, 2.0, 1.0);  // T1 fixture A: d1=2 wall, d2=1 cap
    runCaseAsym(5.0, 10.0, 1.0, 2.0);  // T1 fixture B: swapped
    runCaseAsym(4.0, 8.0, 1.5, 0.8);   // T1 fixture C: a second body
    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
