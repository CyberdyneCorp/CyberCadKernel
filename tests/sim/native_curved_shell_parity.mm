// SPDX-License-Identifier: Apache-2.0
//
// native_curved_shell_parity.mm — native-vs-OCCT CURVED-SHELL parity harness, driven
//                                  THROUGH the cc_* facade (iOS simulator).
//
// MOAT M3 (`moat-m3cs-curved-shell`) — the second CURVED-blend slice: hollow a capped
// CYLINDER or capped CONE FRUSTUM to a uniform wall thickness `t`, opening ONE planar
// cap. The curved wall offsets inward analytically (cylinder Rc→Rc−t; cone reference
// radius →−t/cosσ). Native rebuilds the hollow tube as a deflection-bounded planar-facet
// soup welded watertight through assembleSolid (src/native/blend/curved_shell.h); the
// engine self-verify (0 < Vr < Vo) accepts it. A body outside the slice (stepped shaft,
// both caps removed, picked wall, t too large) returns NULL and forwards to OCCT.
//
// This harness exercises the SHIPPING PATH: it calls the same public
// cc_solid_extrude_profile / cc_solid_revolve / cc_shell the app calls, once with the
// OCCT engine (the oracle, BRepOffsetAPI_MakeThickSolid) and once with the NativeEngine
// (the native offset-tube path), and compares volume / area / watertightness against the
// oracle AND against the CLOSED-FORM wall volume (both deflection-bounded — the native
// facet cylinder/cone under-estimates a curved surface).
//
//   cc_set_engine(0) → OCCT oracle (BRepOffsetAPI_MakeThickSolid).
//   cc_set_engine(1) → NativeEngine (analytic curved shell).
//
// Output: [NCSHELL] PASS/FAIL lines. Exits std::_Exit(failed?1:0).
//
// Build: scripts/run-sim-native-curved-shell.sh (models run-sim-native-curved-fillet.sh).

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
    if (ok) { ++g_passed; std::printf("[NCSHELL] PASS  %-40s %s\n", label.c_str(), detail); }
    else    { ++g_failed; std::printf("[NCSHELL] FAIL  %-40s %s\n", label.c_str(), detail); }
}

// Mesh watertightness over POSITION-WELDED vertices (engine-agnostic — OCCT emits per-
// face vertex copies; weld by geometric coincidence within a Euclidean tolerance).
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

// A capped solid cylinder about +Z (radius Rc, height h). Built with the ACTIVE engine.
CCShapeId buildCappedCylinder(double Rc, double h) {
    CCProfileSeg seg{};
    seg.kind = 2; seg.cx = 0.0; seg.cy = 0.0; seg.r = Rc;
    return cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, h);
}

// A capped cone frustum revolved about the +Y axis (cc_solid_revolve revolves the 2D
// meridian about its Y axis): base Rb at y=0, top Rt at y=H. Built with the ACTIVE engine.
CCShapeId buildCappedFrustum(double Rb, double Rt, double H) {
    const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
    return cc_solid_revolve(prof, 4, 2.0 * kPi);
}

// Every face id whose face-mesh vertices ALL lie on the plane {coord·axis}. A revolve
// splits a cap into angular sub-faces; this collects them all so the whole cap opens.
// Engine-independent (resolved from cc_face_meshes geometry).
std::vector<int> capFaceIds(CCShapeId body, int axisComp, double coord, double tol) {
    std::vector<int> ids;
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.05, &faces);
    for (int f = 0; f < n; ++f) {
        const CCFaceMesh& fm = faces[f];
        if (fm.vertexCount < 3 || fm.vertices == nullptr) continue;
        bool onPlane = true;
        for (int v = 0; v < fm.vertexCount && onPlane; ++v)
            if (std::fabs(fm.vertices[v * 3 + axisComp] - coord) > tol) onPlane = false;
        if (onPlane) ids.push_back(fm.faceId);
    }
    cc_face_meshes_free(faces, n);
    return ids;
}

struct Snapshot {
    CCShapeId id = 0;
    CCMassProps mass{0, 0, 0, 0, 0, 0};
    bool activeNative = false;
};

// ── CYLINDER shell parity ──────────────────────────────────────────────────────────
// Closed-form wall volume of a capped cylinder shelled with the TOP cap open:
//   V = π·Rc²·H − π·(Rc−t)²·(H−t).
double cylinderShellVolume(double Rc, double H, double t) {
    return kPi * (Rc * Rc * H - (Rc - t) * (Rc - t) * (H - t));
}

Snapshot buildAndShellCyl(double Rc, double H, double t, int buildEngine, int shellEngine) {
    cc_set_engine(buildEngine);
    const CCShapeId body = buildCappedCylinder(Rc, H);
    // Pick the open (top, z=H) cap under the BUILD engine so the ids are valid for it,
    // then re-resolve under the shell engine (ids are engine-local — resolve per engine).
    Snapshot s;
    if (body != 0) {
        const std::vector<int> caps = capFaceIds(body, /*z*/ 2, H, 1e-5);
        cc_set_engine(shellEngine);
        s.activeNative = cc_active_engine() == 1;
        if (!caps.empty()) {
            s.id = cc_shell(body, caps.data(), static_cast<int>(caps.size()), t);
            if (s.id != 0) s.mass = cc_mass_properties(s.id);
        }
    } else {
        cc_set_engine(shellEngine);
        s.activeNative = cc_active_engine() == 1;
    }
    if (body) cc_shape_release(body);
    return s;
}

void runCylCase(double Rc, double H, double t) {
    char detail[512];
    char lbl[80];
    std::snprintf(lbl, sizeof lbl, "cyl-shell Rc=%.1f H=%.1f t=%.2f", Rc, H, t);
    const std::string base = lbl;

    const Snapshot oracle = buildAndShellCyl(Rc, H, t, /*build*/ 0, /*shell*/ 0);
    if (oracle.id == 0 || oracle.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "OCCT oracle failed: %s", cc_last_error());
        record(false, base + " oracle", detail);
        cc_set_engine(0);
        if (oracle.id) cc_shape_release(oracle.id);
        return;
    }
    const Snapshot cand = buildAndShellCyl(Rc, H, t, /*build*/ 1, /*shell*/ 1);
    const CCMesh cMesh = cand.id ? cc_tessellate(cand.id, 0.02) : CCMesh{nullptr, 0, nullptr, 0};
    if (cand.id == 0 || cand.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "native active=%d shell->0 (%s)",
                      cand.activeNative ? 1 : 0, cc_last_error());
        record(false, base + " native", detail);
        cc_set_engine(0);
        cc_shape_release(oracle.id);
        return;
    }

    const double exact = cylinderShellVolume(Rc, H, t);
    const double sharp = kPi * Rc * Rc * H;
    const double volRelO = std::fabs(cand.mass.volume - oracle.mass.volume) / oracle.mass.volume;
    const double volRelX = std::fabs(cand.mass.volume - exact) / exact;
    const double areaRel = oracle.mass.area > 0.0
                               ? std::fabs(cand.mass.area - oracle.mass.area) / oracle.mass.area
                               : 1.0;
    const bool massOk = cand.activeNative && volRelO < 1e-2 && volRelX < 1e-2 && areaRel < 3e-2 &&
                        cand.mass.volume < sharp;  // a shell REMOVES material
    std::snprintf(detail, sizeof detail,
                  "vol o=%.6g n=%.6g exact=%.6g relO=%.2e relX=%.2e | area rel=%.2e | shrank=%d",
                  oracle.mass.volume, cand.mass.volume, exact, volRelO, volRelX, areaRel,
                  cand.mass.volume < sharp ? 1 : 0);
    record(massOk, base + " mass", detail);

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

// ── CONE FRUSTUM shell parity ──────────────────────────────────────────────────────
// Closed-form wall volume: outer frustum minus the inner frustum (offset inward by
// t/cosσ), the cavity running from the kept inner face (y=t) to the open end (y=H).
double frustumShellVolume(double Rb, double Rt, double H, double t) {
    const double tanS = (Rt - Rb) / H;
    const double cosS = 1.0 / std::sqrt(1.0 + tanS * tanS);
    const double inset = t / cosS;
    const double vOuter = (kPi * H / 3.0) * (Rb * Rb + Rb * Rt + Rt * Rt);
    const double a = Rb - inset, b = tanS;
    auto F = [&](double z) { return a * a * z + a * b * z * z + b * b * z * z * z / 3.0; };
    const double vCav = kPi * (F(H) - F(t));
    return vOuter - vCav;
}

Snapshot buildAndShellFrustum(double Rb, double Rt, double H, double t, int buildEngine,
                              int shellEngine) {
    cc_set_engine(buildEngine);
    const CCShapeId body = buildCappedFrustum(Rb, Rt, H);
    Snapshot s;
    if (body != 0) {
        const std::vector<int> caps = capFaceIds(body, /*y*/ 1, H, 1e-4);
        cc_set_engine(shellEngine);
        s.activeNative = cc_active_engine() == 1;
        if (!caps.empty()) {
            s.id = cc_shell(body, caps.data(), static_cast<int>(caps.size()), t);
            if (s.id != 0) s.mass = cc_mass_properties(s.id);
        }
    } else {
        cc_set_engine(shellEngine);
        s.activeNative = cc_active_engine() == 1;
    }
    if (body) cc_shape_release(body);
    return s;
}

void runFrustumCase(double Rb, double Rt, double H, double t) {
    char detail[512];
    char lbl[80];
    std::snprintf(lbl, sizeof lbl, "frustum-shell Rb=%.1f Rt=%.1f H=%.1f t=%.2f", Rb, Rt, H, t);
    const std::string base = lbl;

    const Snapshot oracle = buildAndShellFrustum(Rb, Rt, H, t, /*build*/ 0, /*shell*/ 0);
    if (oracle.id == 0 || oracle.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "OCCT oracle failed: %s", cc_last_error());
        record(false, base + " oracle", detail);
        cc_set_engine(0);
        if (oracle.id) cc_shape_release(oracle.id);
        return;
    }
    const Snapshot cand = buildAndShellFrustum(Rb, Rt, H, t, /*build*/ 1, /*shell*/ 1);
    const CCMesh cMesh = cand.id ? cc_tessellate(cand.id, 0.02) : CCMesh{nullptr, 0, nullptr, 0};
    if (cand.id == 0 || cand.mass.valid == 0) {
        std::snprintf(detail, sizeof detail, "native active=%d shell->0 (%s)",
                      cand.activeNative ? 1 : 0, cc_last_error());
        record(false, base + " native", detail);
        cc_set_engine(0);
        cc_shape_release(oracle.id);
        return;
    }

    const double exact = frustumShellVolume(Rb, Rt, H, t);
    const double sharp = (kPi * H / 3.0) * (Rb * Rb + Rb * Rt + Rt * Rt);
    const double volRelO = std::fabs(cand.mass.volume - oracle.mass.volume) / oracle.mass.volume;
    const double volRelX = std::fabs(cand.mass.volume - exact) / exact;
    const double areaRel = oracle.mass.area > 0.0
                               ? std::fabs(cand.mass.area - oracle.mass.area) / oracle.mass.area
                               : 1.0;
    // OCCT MakeThickSolid may offset the cone slightly differently (join style), so the
    // native-vs-OCCT band is a touch looser than the exact closed-form band.
    const bool massOk = cand.activeNative && volRelO < 2e-2 && volRelX < 1e-2 && areaRel < 4e-2 &&
                        cand.mass.volume < sharp;
    std::snprintf(detail, sizeof detail,
                  "vol o=%.6g n=%.6g exact=%.6g relO=%.2e relX=%.2e | area rel=%.2e | shrank=%d",
                  oracle.mass.volume, cand.mass.volume, exact, volRelO, volRelX, areaRel,
                  cand.mass.volume < sharp ? 1 : 0);
    record(massOk, base + " mass", detail);

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

}  // namespace

int main() {
    std::printf("== native curved-shell (analytic offset tube) vs OCCT parity ==\n");
    // Capped cylinder shells (cup / sleeve), top open.
    runCylCase(5.0, 10.0, 1.0);
    runCylCase(4.0, 8.0, 0.5);
    runCylCase(6.0, 12.0, 1.5);
    // Capped cone frustum shells (tapered bushing), top open.
    runFrustumCase(6.0, 4.0, 10.0, 1.0);
    runFrustumCase(4.0, 6.0, 10.0, 1.0);   // widening frustum
    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
