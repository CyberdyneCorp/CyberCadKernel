// SPDX-License-Identifier: Apache-2.0
//
// native_wrap_emboss_parity.mm — native-vs-OCCT WRAP-EMBOSS parity harness, driven
//                                 THROUGH the cc_* facade (iOS simulator).
//
// Phase 4 capability #7 (`native-wrap-emboss`) — FIRST slice: emboss a RECTANGULAR pad
// onto a CYLINDER lateral face (boss=1). The native builder wraps the footprint onto the
// cylinder (u=px/R, v=py+vMid — the SAME map the OCCT oracle uses) and rebuilds the whole
// embossed solid as a deflection-bounded planar-facet soup welded watertight
// (src/native/feature/wrap_emboss.h), with the pad's cap-and-side walls closing against
// the base cylinder along the shared footprint boundary.
//
// This harness exercises the SHIPPING PATH: it calls the same public
// cc_solid_extrude_profile / cc_wrap_emboss the app calls, once with the OCCT engine (the
// Phase-3 oracle occt_wrap_emboss.cpp, cap-and-side + healed sew) and once with the
// NativeEngine (the native facet path), and compares.
//
//   cc_set_engine(0) → OCCT oracle (BRepBuilderAPI_Sewing + ShapeFix + fuse).
//   cc_set_engine(1) → NativeEngine. A capped cylinder built via
//                      cc_solid_extrude_profile(kind-2 full circle) is a NATIVE body with
//                      a single Cylinder wall face. cc_wrap_emboss on that wall builds the
//                      pad natively (deflection-bounded facets, welded watertight) and the
//                      engine self-verify (watertight + volume grown by footprint×height)
//                      accepts it. A body OUTSIDE the slice (deboss / non-rectangular /
//                      non-cylindrical / off-end footprint) returns NULL and forwards to
//                      OCCT — asserted by the scope-defers host test.
//
// The native faceted result differs from OCCT's true cylindrical pad, so vol/area compare
// with a loose (deflection-bounded) tolerance; the native result MUST be watertight and
// its volume MUST match the OCCT oracle (and the analytic footprint×height growth).
//
// Output: [NWEMB] PASS/FAIL lines. Exits std::_Exit(failed?1:0).
//
// Build: scripts/run-sim-native-wrap-emboss.sh (models run-sim-native-curved-fillet.sh).

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
    if (ok) { ++g_passed; std::printf("[NWEMB] PASS  %-40s %s\n", label.c_str(), detail); }
    else    { ++g_failed; std::printf("[NWEMB] FAIL  %-40s %s\n", label.c_str(), detail); }
}

// mesh watertight over POSITION-WELDED vertices (engine-agnostic — OCCT emits per-face
// vertex copies; the native mesher's coincident corners may land a hair apart). Weld a
// vertex to any already-seen representative in its own OR its 26 neighbour cells within a
// true Euclidean kWeld, measuring real geometric closure (matches the curved-fillet
// harness's welder).
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

// A capped solid cylinder about +Z: extrude a full-circle profile (kind-2, centre origin,
// radius r) by height h. Faces: bottom cap (z=0), top cap (z=h), one Cylinder wall (id 3
// in the deterministic native/OCCT topology). Built with the ACTIVE engine.
CCShapeId buildCappedCylinder(double r, double h) {
    CCProfileSeg seg{};
    seg.kind = 2; seg.cx = 0.0; seg.cy = 0.0; seg.r = r;
    return cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, h);
}

// The 1-based id of the Cylinder lateral face. Under OCCT cc_face_axis identifies a
// cylinder/cone face; if it never succeeds (native body — face_axis is OCCT-only) fall
// back to the deterministic wall id 3 (bottom cap, top cap, wall).
int findCylFace(CCShapeId body) {
    int* ids = nullptr;
    const int n = cc_subshape_ids(body, 2, &ids);
    int found = 0;
    for (int i = 0; i < n && found == 0; ++i) {
        double ax6[6];
        if (cc_face_axis(body, ids[i], ax6)) found = ids[i];
    }
    cc_ints_free(ids);
    return found != 0 ? found : 3;
}

struct Snapshot {
    CCShapeId id = 0;
    CCMassProps mass{0, 0, 0, 0, 0, 0};
    bool activeNative = false;
    CCMassProps base{0, 0, 0, 0, 0, 0};  // the base cylinder's mass (for the growth check)
};

// Build the capped cylinder AND wrap-emboss/deboss its wall, both under `engine`, with an
// arbitrary closed profile (`prof`, `count` (px,py) pairs) raised/recessed by `height`.
Snapshot buildAndEmboss(double Rc, double h, const double* prof, int count, double height,
                        int boss, int engine) {
    cc_set_engine(engine);
    Snapshot s;
    s.activeNative = cc_active_engine() == 1;
    const CCShapeId body = buildCappedCylinder(Rc, h);
    if (body == 0) return s;
    s.base = cc_mass_properties(body);
    const int face = findCylFace(body);
    s.id = cc_wrap_emboss(body, face, prof, count, height, boss);
    if (s.id != 0) s.mass = cc_mass_properties(s.id);
    cc_shape_release(body);
    return s;
}

// Twice the signed shoelace area of a closed (px,py) profile.
double shoelaceArea(const double* prof, int count) {
    double a2 = 0.0;
    for (int i = 0; i < count; ++i) {
        const int j = (i + 1) % count;
        a2 += prof[i * 2] * prof[j * 2 + 1] - prof[j * 2] * prof[i * 2 + 1];
    }
    return std::fabs(a2) * 0.5;
}

// One native wrap-emboss/deboss case: build Rc×h cylinder, apply `prof` (footprint area
// via shoelace) raised (boss=1) / recessed (boss=0) by `height`, compare the native result
// to the OCCT oracle AND to the analytic footprint×height volume change.
void runCase(const char* name, double Rc, double h, const double* prof, int count, double height,
             int boss) {
    char detail[512];
    char lbl[96];
    std::snprintf(lbl, sizeof lbl, "%s Rc=%.1f h=%.1f x%g", name, Rc, h, height);
    const std::string base = lbl;

    const Snapshot oracle = buildAndEmboss(Rc, h, prof, count, height, boss, /*engine*/ 0);
    if (oracle.id == 0 || oracle.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "OCCT oracle failed: %s", cc_last_error());
        record(false, base + " oracle", detail);
        cc_set_engine(0);
        if (oracle.id) cc_shape_release(oracle.id);
        return;
    }
    const Snapshot cand = buildAndEmboss(Rc, h, prof, count, height, boss, /*engine*/ 1);
    const CCMesh cMesh = cand.id ? cc_tessellate(cand.id, 0.02) : CCMesh{nullptr, 0, nullptr, 0};
    if (cand.id == 0 || cand.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "native active=%d emboss->0 (%s)",
                      cand.activeNative ? 1 : 0, cc_last_error());
        record(false, base + " native", detail);
        cc_set_engine(0);
        cc_shape_release(oracle.id);
        return;
    }

    // Volume vs the OCCT oracle AND vs the analytic change (base ± footprint area × height;
    // because px is arc-length, the wrapped area is the flat shoelace area). Area vs OCCT.
    const double sign = (boss == 1) ? 1.0 : -1.0;
    const double delta = shoelaceArea(prof, count) * height;
    const double expected = cand.base.volume + sign * delta;
    const double volRelO = std::fabs(cand.mass.volume - oracle.mass.volume) / oracle.mass.volume;
    const double volRelX = std::fabs(cand.mass.volume - expected) / expected;
    const double areaRel = oracle.mass.area > 0.0
                               ? std::fabs(cand.mass.area - oracle.mass.area) / oracle.mass.area
                               : 1.0;
    const bool signOk = (boss == 1) ? (cand.mass.volume > cand.base.volume)   // emboss GROWS
                                     : (cand.mass.volume < cand.base.volume);  // deboss SHRINKS
    const bool massOk =
        cand.activeNative && volRelO < 1e-2 && volRelX < 1e-2 && areaRel < 3e-2 && signOk;
    std::snprintf(detail, sizeof detail,
                  "vol o=%.6g n=%.6g expect=%.6g relO=%.2e relX=%.2e | area rel=%.2e",
                  oracle.mass.volume, cand.mass.volume, expected, volRelO, volRelX, areaRel);
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

    if (haveMesh) cc_mesh_free(cMesh);
    cc_set_engine(0);
    cc_shape_release(cand.id);
    cc_shape_release(oracle.id);
}

// A centred axis-aligned rectangle (arc-length width aw × axial height ah).
std::vector<double> rectProfile(double aw, double ah) {
    return {-aw / 2, -ah / 2, aw / 2, -ah / 2, aw / 2, ah / 2, -aw / 2, ah / 2};
}

// A regular hexagon (centre-to-vertex a), CCW, centred at the origin.
std::vector<double> hexProfile(double a) {
    const double s = a * 0.8660254037844386, hh = a * 0.5;
    return {a, 0, hh, s, -hh, s, -a, 0, -hh, -s, hh, -s};
}

}  // namespace

int main() {
    std::printf("== native wrap-emboss/deboss (cylinder) vs OCCT parity ==\n");
    // CONTROL — raised rectangular pad on a cylinder (unchanged 3 cases → 6 assertions).
    const std::vector<double> r1 = rectProfile(6.0, 8.0);
    const std::vector<double> r2 = rectProfile(4.0, 5.0);
    const std::vector<double> r3 = rectProfile(10.0, 6.0);
    runCase("emboss-rect", 10.0, 20.0, r1.data(), 4, 2.0, 1);
    runCase("emboss-rect", 8.0, 24.0, r2.data(), 4, 1.5, 1);
    runCase("emboss-rect", 12.0, 16.0, r3.data(), 4, 3.0, 1);
    // T1 — recessed rectangular pocket (boss=0).
    const std::vector<double> d1 = rectProfile(6.0, 8.0);
    runCase("deboss-rect", 10.0, 20.0, d1.data(), 4, 2.0, 0);
    runCase("deboss-rect", 8.0, 24.0, r2.data(), 4, 1.5, 0);
    // T2 — non-rectangular (hexagon) footprint, raised and recessed.
    const std::vector<double> hx = hexProfile(5.0);
    runCase("emboss-hex", 10.0, 20.0, hx.data(), 6, 2.0, 1);
    runCase("deboss-hex", 10.0, 20.0, hx.data(), 6, 2.0, 0);
    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
