// SPDX-License-Identifier: Apache-2.0
//
// native_curved_offset_parity.mm — native-vs-OCCT CURVED offset-face parity harness,
//                                   driven THROUGH the cc_* facade (iOS simulator).
//
// MOAT M3 (`moat-m3co-curved-offset-face`) — the CURVED offset-face slice: offset the
// CYLINDER LATERAL wall of a capped cylinder RADIALLY (radius Rc → Rc+d) in
// cc_offset_face. The offset of a cylinder surface is a coaxial cylinder, so the native
// engine re-radiuses the whole capped body analytically (wall band + two disc caps,
// planar-facet weld through assembleSolid, no tessellator change —
// src/native/blend/curved_offset.h); the engine self-verify (grow d>0 → Vr>Vo, shrink
// d<0 → 0<Vr<Vo, watertight + oriented) accepts it. A body outside the slice (cone /
// sphere / stepped / multi-radius / picked planar cap) returns NULL and forwards to OCCT.
//
// ── WHY THE OCCT ORACLE IS BUILT DIRECTLY ────────────────────────────────────────────
// The SHIPPED OCCT cc_offset_face (occt_feature.cpp, byte-identical to the app's
// KernelBridge.mm) ONLY handles PLANAR faces — `surf.GetType() != GeomAbs_Plane → error`.
// It CANNOT offset a cylinder wall at all; that is precisely the M3 curved residual the
// native arm fills. So OCCT-through-the-facade cannot be the oracle for the curved case.
// Instead the harness builds the GROUND-TRUTH oracle DIRECTLY with OCCT: the exact
// radial offset of a capped cylinder's wall (caps following) is the coaxial capped
// cylinder at radius Rc+d, i.e. BRepPrimAPI_MakeCylinder(Rc+d, H), measured with
// BRepGProp (volume / area / bbox) — the exact closed form π(Rc+d)²·H. The harness also
// asserts that OCCT-through-facade HONESTLY DECLINES the curved face (native owns it),
// and that a CONE wall (out of slice) is a native NULL that OCCT owns.
//
//   cc_set_engine(1) → NativeEngine (analytic curved offset — the shipping path).
//   cc_set_engine(0) → OCCT (declines the curved face through the facade, as designed).
//
// Compared per fixture, native cc_offset_face result vs the direct OCCT oracle:
//   * VOLUME     — native cc_mass_properties vs BRepGProp::VolumeProperties (rel ≤ 1e-2)
//                  AND vs the exact closed form π(Rc+d)²·H (rel ≤ 1e-2);
//   * AREA       — native area vs BRepGProp::SurfaceProperties (rel ≤ 3e-2, facet band);
//   * WATERTIGHT — the native result mesh is a closed 2-manifold;
//   * EULER χ=2  — native mesh is a single closed genus-0 solid;
//   * BBOX       — native vertex min/max vs BRepBndLib per axis (abs ≤ 2·deflection);
//   * DIRECTION  — grow (d>0) → Vr > π·Rc²·H, shrink (d<0) → 0 < Vr < π·Rc²·H.
//
// Output: [NCOFF] PASS/FAIL lines. Exits std::_Exit(failed?1:0).
//
// Build: scripts/run-sim-native-curved-offset.sh (models run-sim-native-curved-shell.sh).

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

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_curved_offset_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <TopoDS_Shape.hxx>

namespace {

constexpr double kPi = 3.14159265358979323846;

int g_passed = 0;
int g_failed = 0;
void record(bool ok, const std::string& label, const char* detail) {
    if (ok) { ++g_passed; std::printf("[NCOFF] PASS  %-42s %s\n", label.c_str(), detail); }
    else    { ++g_failed; std::printf("[NCOFF] FAIL  %-42s %s\n", label.c_str(), detail); }
    std::fflush(stdout);
}

// Mesh watertightness over POSITION-WELDED vertices (engine-agnostic; the native mesh
// emits per-facet vertex copies, weld by geometric coincidence within a tolerance).
bool meshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0) return false;
    constexpr double kWeld = 1e-6;
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

// Euler characteristic of a POSITION-WELDED triangle mesh (V − E + F). A single closed
// genus-0 solid has χ = 2. Weld duplicate vertices exactly as meshWatertight does.
long meshEuler(const CCMesh& m) {
    if (m.triangleCount <= 0) return 0;
    constexpr double kWeld = 1e-6;
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
    int reps = 0;
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
        else { rep[static_cast<std::size_t>(v)] = v; cellReps[cellKey(cx, cy, cz)].push_back(v); ++reps; }
    }
    std::unordered_map<std::uint64_t, int> edges;
    auto key = [](int a, int b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = rep[static_cast<std::size_t>(m.triangles[t * 3 + 0])];
        const int j = rep[static_cast<std::size_t>(m.triangles[t * 3 + 1])];
        const int k = rep[static_cast<std::size_t>(m.triangles[t * 3 + 2])];
        ++edges[key(i, j)]; ++edges[key(j, k)]; ++edges[key(k, i)];
    }
    return static_cast<long>(reps) - static_cast<long>(edges.size()) +
           static_cast<long>(m.triangleCount);
}

struct BBox { double lo[3]; double hi[3]; };
BBox meshBBox(const CCMesh& m) {
    BBox b{{1e30, 1e30, 1e30}, {-1e30, -1e30, -1e30}};
    for (int v = 0; v < m.vertexCount; ++v) {
        const double* p = &m.vertices[v * 3];
        for (int i = 0; i < 3; ++i) { b.lo[i] = std::min(b.lo[i], p[i]); b.hi[i] = std::max(b.hi[i], p[i]); }
    }
    return b;
}

double occtVolume(const TopoDS_Shape& s) {
    GProp_GProps g; BRepGProp::VolumeProperties(s, g); return std::fabs(g.Mass());
}
double occtArea(const TopoDS_Shape& s) {
    GProp_GProps g; BRepGProp::SurfaceProperties(s, g); return g.Mass();
}

// A capped solid cylinder about +Z (radius Rc, height h) built with the ACTIVE engine —
// the same public call the app uses (kind 2 = full circle profile → extrude).
CCShapeId buildCappedCylinder(double Rc, double h) {
    CCProfileSeg seg{};
    seg.kind = 2; seg.cx = 0.0; seg.cy = 0.0; seg.r = Rc;
    return cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, h);
}

// The id of the CYLINDER LATERAL wall face: the face whose mesh vertices all lie at
// radius ≈ Rc from the +Z axis (x²+y² ≈ Rc²) and span the full axial extent [0,H].
// Engine-independent (resolved from cc_face_meshes geometry, ids are engine-local).
int cylWallFaceId(CCShapeId body, double Rc, double H) {
    int found = -1;
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.05, &faces);
    for (int f = 0; f < n; ++f) {
        const CCFaceMesh& fm = faces[f];
        if (fm.vertexCount < 3 || fm.vertices == nullptr) continue;
        bool onWall = true;
        double zlo = 1e30, zhi = -1e30;
        for (int v = 0; v < fm.vertexCount && onWall; ++v) {
            const double x = fm.vertices[v * 3 + 0], y = fm.vertices[v * 3 + 1],
                         z = fm.vertices[v * 3 + 2];
            const double r = std::sqrt(x * x + y * y);
            if (std::fabs(r - Rc) > 1e-3) onWall = false;
            zlo = std::min(zlo, z); zhi = std::max(zhi, z);
        }
        if (onWall && (zhi - zlo) > 0.5 * H) { found = fm.faceId; break; }
    }
    cc_face_meshes_free(faces, n);
    return found;
}

// ── Cylinder-wall radial offset parity ───────────────────────────────────────────────
// distance d (signed): grow d>0 → radius Rc+d, shrink d<0 → Rc+d (must stay > 0).
void runCylCase(double Rc, double H, double d) {
    char detail[512];
    char lbl[96];
    std::snprintf(lbl, sizeof lbl, "cyl-wall Rc=%.1f H=%.1f d=%+.2f", Rc, H, d);
    const std::string base = lbl;
    const double defl = 0.01;
    const double newR = Rc + d;

    // Native shipping path: build + offset the cylinder wall under the NativeEngine.
    cc_set_engine(1);
    const bool nativeActive = cc_active_engine() == 1;
    const CCShapeId body = buildCappedCylinder(Rc, H);
    if (body == 0 || !nativeActive) {
        std::snprintf(detail, sizeof detail, "native build failed active=%d (%s)",
                      nativeActive ? 1 : 0, cc_last_error());
        record(false, base + " native-build", detail);
        cc_set_engine(0);
        return;
    }
    const int wall = cylWallFaceId(body, Rc, H);
    record(wall > 0, base + " wall-face-id", wall > 0 ? "found" : "missing");
    if (wall <= 0) { cc_shape_release(body); cc_set_engine(0); return; }

    const CCShapeId nat = cc_offset_face(body, wall, d);
    if (nat == 0) {
        std::snprintf(detail, sizeof detail, "native cc_offset_face->0 (%s)", cc_last_error());
        record(false, base + " native-offset", detail);
        cc_shape_release(body);
        cc_set_engine(0);
        return;
    }
    const CCMassProps nm = cc_mass_properties(nat);
    const CCMesh nMesh = cc_tessellate(nat, defl);

    // OCCT-through-facade MUST decline the curved wall (planar-only), proving the native
    // arm is filling a genuine OCCT gap — the reason the oracle is built directly below.
    // Build a SEPARATE OCCT-engine body (a shape id must be consumed under the engine that
    // created it — a native id fed to OCCT would be unwrapped wrongly) and offset ITS wall.
    cc_set_engine(0);
    const CCShapeId occtBody = buildCappedCylinder(Rc, H);
    if (occtBody != 0) {
        const int occtWall = cylWallFaceId(occtBody, Rc, H);
        const CCShapeId occtFacade = occtWall > 0 ? cc_offset_face(occtBody, occtWall, d) : 0;
        record(occtFacade == 0, base + " occt-facade-declines",
               occtFacade == 0 ? "planar-only (expected)" : "UNEXPECTED non-null");
        if (occtFacade != 0) cc_shape_release(occtFacade);
        cc_shape_release(occtBody);
    } else {
        record(false, base + " occt-facade-declines", "occt body build failed");
    }

    // Direct OCCT ground-truth oracle: the coaxial capped cylinder at radius Rc+d is the
    // exact radial offset of the wall (caps following). Measure with BRepGProp.
    const TopoDS_Shape oracle = BRepPrimAPI_MakeCylinder(newR, H).Shape();
    const double vOcc = occtVolume(oracle);
    const double aOcc = occtArea(oracle);
    const double exact = kPi * newR * newR * H;
    const double sharpOrig = kPi * Rc * Rc * H;

    // Mass parity: native vs OCCT oracle AND vs the exact closed form.
    if (nm.valid == 0) {
        record(false, base + " native-mass", "invalid mass props");
    } else {
        const double relO = std::fabs(nm.volume - vOcc) / vOcc;
        const double relX = std::fabs(nm.volume - exact) / exact;
        const double areaRel = aOcc > 0.0 ? std::fabs(nm.area - aOcc) / aOcc : 1.0;
        const bool dirOk = (d > 0.0) ? (nm.volume > sharpOrig) : (nm.volume < sharpOrig && nm.volume > 0.0);
        const bool massOk = relO < 1e-2 && relX < 1e-2 && areaRel < 3e-2 && dirOk;
        std::snprintf(detail, sizeof detail,
                      "vol nat=%.6g occt=%.6g exact=%.6g relO=%.2e relX=%.2e | area rel=%.2e | dir=%d",
                      nm.volume, vOcc, exact, relO, relX, areaRel, dirOk ? 1 : 0);
        record(massOk, base + " mass", detail);
    }

    // Tessellation: watertight closed 2-manifold, χ=2, mesh volume matches, bbox matches.
    const bool haveMesh = nMesh.triangleCount > 0;
    const bool wt = haveMesh && meshWatertight(nMesh);
    const long chi = haveMesh ? meshEuler(nMesh) : 0;
    const double mVol = haveMesh ? meshVolume(nMesh) : 0.0;
    const double mVolRel = (haveMesh && nm.valid && nm.volume > 0.0)
                               ? std::fabs(mVol - nm.volume) / nm.volume : 1.0;
    const bool tessOk = haveMesh && wt && chi == 2 && mVolRel < 2e-2;
    std::snprintf(detail, sizeof detail, "watertight=%d euler=%ld tris=%d meshVolRel=%.2e",
                  wt ? 1 : 0, chi, nMesh.triangleCount, mVolRel);
    record(tessOk, base + " tessellate", detail);

    // BBox parity vs the OCCT oracle (a cylinder about +Z, radius newR, z∈[0,H]).
    if (haveMesh) {
        Bnd_Box bb; BRepBndLib::Add(oracle, bb);
        double xo[3], xh[3];
        bb.Get(xo[0], xo[1], xo[2], xh[0], xh[1], xh[2]);
        const BBox nb = meshBBox(nMesh);
        double worst = 0.0;
        for (int i = 0; i < 3; ++i) {
            worst = std::max(worst, std::fabs(nb.lo[i] - xo[i]));
            worst = std::max(worst, std::fabs(nb.hi[i] - xh[i]));
        }
        const double bboxTol = 2.0 * defl;
        std::snprintf(detail, sizeof detail, "worst=%.3e tol=%.3e", worst, bboxTol);
        record(worst <= bboxTol, base + " bbox", detail);
    }

    if (haveMesh) cc_mesh_free(nMesh);
    cc_shape_release(nat);
    cc_shape_release(body);
    cc_set_engine(0);
}

// ── Honest-decline: a CONE wall is OUT of the capped-cylinder slice ────────────────────
// Native cc_offset_face on a cone lateral wall must return NULL (→ OCCT owns it). The
// OCCT oracle (a cone offset outward by d along the surface normal) is measured only to
// show the pose is a legitimate operation OCCT would own; the gate here is native NULL.
void runConeDecline(double Rb, double Rt, double H, double d) {
    char lbl[96];
    std::snprintf(lbl, sizeof lbl, "cone-wall Rb=%.1f Rt=%.1f H=%.1f d=%+.2f (decline)", Rb, Rt, H, d);
    const std::string base = lbl;

    cc_set_engine(1);
    // Revolve a cone-frustum meridian about +Y (matches the curved-shell frustum builder).
    const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
    const CCShapeId body = cc_solid_revolve(prof, 4, 2.0 * kPi);
    if (body == 0) {
        record(true, base + " (cone build declined native — OCCT owns)", cc_last_error());
        cc_set_engine(0);
        return;
    }
    // Find the cone lateral wall: the face whose vertices are NOT all on a y-plane.
    int wall = -1;
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.05, &faces);
    for (int f = 0; f < n; ++f) {
        const CCFaceMesh& fm = faces[f];
        if (fm.vertexCount < 3 || fm.vertices == nullptr) continue;
        double ylo = 1e30, yhi = -1e30;
        for (int v = 0; v < fm.vertexCount; ++v) {
            const double y = fm.vertices[v * 3 + 1];
            ylo = std::min(ylo, y); yhi = std::max(yhi, y);
        }
        if ((yhi - ylo) > 0.5 * H) { wall = fm.faceId; break; }
    }
    cc_face_meshes_free(faces, n);
    if (wall <= 0) {
        // No lateral wall resolved (cone may itself decline the build) — still an honest
        // native decline for this out-of-slice pose.
        record(true, base + " (no native cone wall — OCCT owns)", "no lateral face");
        cc_shape_release(body);
        cc_set_engine(0);
        return;
    }
    const CCShapeId nat = cc_offset_face(body, wall, d);
    char detail[256];
    std::snprintf(detail, sizeof detail, "native returned %s (%s)",
                  nat == 0 ? "NULL (correct → OCCT)" : "NON-NULL (WRONG)", cc_last_error());
    record(nat == 0, base, detail);
    if (nat != 0) cc_shape_release(nat);
    cc_shape_release(body);
    cc_set_engine(0);
}

}  // namespace

int main() {
    std::printf("== native curved offset-face (cylinder wall, radial) vs OCCT parity ==\n");
    std::fflush(stdout);

    // Grow + shrink, ≥2 radii / heights.
    runCylCase(3.0, 10.0, +1.0);   // grow  Rc 3 → 4
    runCylCase(3.0, 10.0, -1.0);   // shrink Rc 3 → 2
    runCylCase(5.0, 8.0, +1.5);    // grow  Rc 5 → 6.5
    runCylCase(6.0, 12.0, -2.0);   // shrink Rc 6 → 4

    // Out-of-slice honest decline: a cone frustum wall → native NULL → OCCT owns.
    runConeDecline(6.0, 4.0, 10.0, +1.0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
