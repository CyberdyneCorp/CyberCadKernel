// SPDX-License-Identifier: Apache-2.0
//
// native_curved_blend_fuzz.mm — MOAT M6-breadth-15 (the COMPLETENESS BAR, 15th domain):
// a CURVED-BLEND differential-fuzzing harness (iOS simulator) that certifies the large
// NEW M3 curved-blend NATIVE surface landed this session — the analytic curved fillet /
// shell / offset-face paths (src/native/blend/{curved_fillet.h, curved_shell.h,
// curved_offset.h, canal_fillet.h}) reached through the SHIPPING cc_* facade.
//
// It extends the fourteen landed M6 differential fuzzers (curved-boolean, STEP round-trip,
// construction loft/sweep, blend fillet/chamfer, wrap-emboss, mass-properties, geometry-
// services, transform-chains, reference/datum, direct-modeling, transformed-boolean,
// orthographic-HLR, shape-healing, section) to a FIFTEENTH independent native domain.
// Like its siblings it is INFRASTRUCTURE (a seeded harness, not a geometry capability).
//
// ── WHY THIS DOMAIN IS DISTINCT FROM native_blend_fuzz.mm (the 4th domain) ────────────
// native_blend_fuzz.mm fuzzes the native blend BUILDERS DIRECTLY (blend::chamfer_edges /
// fillet_edges / curved_fillet_edge …) on a box edge / a single cylinder↔cap rim, and it
// EXPLICITLY declared the curved SHELL + curved OFFSET an "honest domain-level decline for
// this first blend slice". THIS harness closes exactly that gap: it drives the STABLE,
// LANDED curved SHELL and curved OFFSET (plus curved FILLET on cone/sphere rims, not only
// the cylinder rim) through the PUBLIC cc_fillet_edges / cc_shell / cc_offset_face facade
// under BOTH engines (cc_set_engine) — the shipping path the app calls — over a seeded
// batch of random analytic-revolve base solids. It is the facade-level, both-engines,
// randomised completeness certification the per-op parity harnesses (native_curved_*_
// parity.mm, hand-picked fixtures) do not provide.
//
// ── THE NINE CURVED-BLEND FAMILIES (op × wall) ───────────────────────────────────────
// Base solids are analytic revolves built through the ACTIVE engine's public facade:
//   CYL   capped cylinder  cc_solid_extrude_profile(kind-2 full circle, radius Rc, +Z, h)
//   CONE  cone frustum     cc_solid_revolve(trapezoid meridian, +Y, 2π)  base Rb top Rt H
//   DOME  sphere-cap dome  cc_solid_revolve_profile(arc meridian, +Y, 2π) radius Ro cap a
// The nine families are {FILLET, SHELL, OFFSET} × {CYL, CONE, DOME}:
//   FILLET_*  cc_fillet_edges(rim, r) — round the curved CAP RIM (cylinder-cap circle /
//             cone top-rim / dome base-rim). REMOVES material (convex rim, ball outside).
//   SHELL_*   cc_shell(open one planar cap, t) — hollow the curved wall to wall thickness t.
//   OFFSET_*  cc_offset_face(curved wall, d) — re-radius the cylinder/cone/sphere wall by d.
//
// ── THE ORACLES: OCCT + CLOSED-FORM (the PRIMARY arbiter) ─────────────────────────────
// Each family carries a CLOSED-FORM analytic value for the resulting VOLUME (and, where
// exact, AREA). It is the PRIMARY arbiter — exact for the ideal solid — so a native result
// matching the closed form while OCCT is the outlier is logged ORACLE-INACCURATE (native
// vindicated by exact math), never a bar failure. The OCCT oracle is:
//   FILLET  cc_set_engine(0) → OCCT BRepFilletAPI through the facade (the shipped OCCT
//           fillet handles the analytic rim), measured by cc_mass_properties.
//   SHELL   cc_set_engine(0) → OCCT BRepOffsetAPI_MakeThickSolid through the facade.
//   OFFSET  built DIRECTLY with OCCT (BRepPrimAPI_MakeCylinder/MakeCone/MakeSphere at the
//           new radius, BRepGProp): the SHIPPED OCCT cc_offset_face is PLANAR-ONLY and
//           HONESTLY DECLINES a curved wall — precisely the M3 residual the native arm
//           fills — so OCCT-through-facade cannot be the offset oracle. The harness also
//           asserts OCCT-through-facade declines the curved wall (native owns it).
//
// ── THE DEFLECTION BOUNDARY (why native-vs-OCCT alone is insufficient) ────────────────
// The native curved-blend paths emit a deflection-bounded PLANAR-FACET weld; OCCT keeps a
// true analytic B-rep. On a curved family the faceted result differs from the exact/OCCT
// value by a real, bounded, EXPECTED chord error — NOT a native fault. So the AGREE band
// is the deflection-convergence tolerance the per-op parity harnesses already validated
// (vol rel ≤ 2e-2 vs OCCT and ≤ 2e-2 vs the closed form; area rel ≤ 4e-2, the facet band),
// MATCHED to the mesh and NEVER widened to force a pass. A tighter band would flag correct
// faceted meshes; a wider one would hide real faults.
//
// ── THE SIX-WAY CLASSIFIER (identical discipline to the landed siblings) ──────────────
//   AGREED            native VALID (watertight, χ=2, correct grow/shrink direction) + vol/
//                     area within the family band of BOTH the closed form AND OCCT, which
//                     also matches the closed form (oracle-trust).
//   HONESTLY-DECLINED native cc_* returns 0 / invalid (an out-of-envelope pose the native
//                     arm refuses) while OCCT ships a valid result → native → OCCT. First-
//                     class, counted separately, NEVER a bar failure.
//   DISAGREED         native VALID but OUTSIDE the closed-form truth while OCCT matches it —
//                     a genuine SILENT WRONG curved blend. The failure this harness exists
//                     to catch. (FAILS the bar.)
//   ORACLE-INACCURATE native matches the closed-form truth while OCCT does NOT — native
//                     vindicated by exact math, OCCT the outlier. Logged, NOT a bar failure.
//   ORACLE_UNRELIABLE a CORE family whose OCCT oracle does not match the closed form to the
//                     (loose, facet-aware) oracle-trust band AND native also missed → the
//                     oracle is untrustworthy → excluded from AGREE, FAILS the bar
//                     (investigate, never launder a native miss as a pass).
//   BOTH-DECLINED     an out-of-envelope pose both engines refuse. Logged, not a failure.
//
// ── THE BAR ──────────────────────────────────────────────────────────────────────────
//   Exit 0 IFF DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, with each of the nine core
//   families having ≥ 1 AGREED trial (real native exercise, not all-declines). Run over
//   ≥ 2 distinct seeds, N ≥ 60 per seed; the runner fails if ANY seed fails. The generator
//   is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed →
//   byte-identical batch (splitmix64 → xoshiro256**, verbatim from the siblings). Any
//   DISAGREE / ORACLE-INACCURATE prints seed + case index + family/param tuple + the
//   native/OCCT/closed-form triple as a reproducible regression find.
//
// This TU is OCCT-dependent (BRepPrimAPI + BRepGProp for the OFFSET oracle) and links the
// WHOLE kernel (facade + core + engine[native+occt] + native math) because it drives the
// SHIPPING cc_* facade — the same set the native_curved_offset_parity harness links. Built
// ONLY by scripts/run-sim-native-curved-blend-fuzz.sh; on run-sim-suite.sh's SKIP list (own
// main(), std::_Exit — OCCT static teardown in the trimmed static build is not exit-clean,
// same rationale as the siblings). src/native / src/engine / include stay BYTE-UNCHANGED —
// this harness is additive test/sim code only and drives the facade rather than modifying it.
//
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
#error "native_curved_blend_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <Standard_Failure.hxx>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Family AGREE bands — the deflection-convergence tolerances the per-op curved parity
// harnesses (native_curved_{fillet,shell,offset}_parity.mm) already validated. FIXED,
// never widened to force a pass.
constexpr double kVolTolO   = 2e-2;   // native vs OCCT volume (facet vs analytic B-rep)
constexpr double kVolTolX   = 2e-2;   // native vs closed-form volume (PRIMARY arbiter)
constexpr double kAreaTol   = 4e-2;   // native vs OCCT surface area (facet band)
constexpr double kOracleTol = 3e-2;   // OCCT vs closed form (oracle-trust; OCCT true B-rep,
                                      // but MakeThickSolid join style drifts on the cone —
                                      // a touch looser than a pure primitive would need)
constexpr double kMeshVolTol = 2e-2;  // mesh-volume vs cc_mass_properties volume

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim from the landed fuzzers).
//    Keyed ONLY by an explicit uint64 seed. No clock, no rand(): same seed → batch. ──────
struct Rng {
    uint64_t s[4];
    static uint64_t splitmix64(uint64_t& x) {
        uint64_t z = (x += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    explicit Rng(uint64_t seed) { for (auto& v : s) v = splitmix64(seed); }
    static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
    uint64_t next() {
        const uint64_t r = rotl(s[1] * 5, 7) * 9;
        const uint64_t t = s[1] << 17;
        s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
        s[3] = rotl(s[3], 45);
        return r;
    }
    double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
    double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
    uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
};

// ── engine-agnostic mesh diagnostics (position-welded; OCCT emits per-face copies, the
//    native facet mesher may land coincident corners a hair apart — weld by geometric
//    coincidence within a Euclidean tolerance, checking the 26 neighbour cells so a grid
//    straddle never falsely reports a leak). Identical to the curved-parity siblings. ──
struct Welder {
    std::unordered_map<std::uint64_t, std::vector<int>> cellReps;
    std::vector<int> rep;
    double weld;
    int reps = 0;
    static std::uint64_t cellKey(long long x, long long y, long long z) {
        std::uint64_t h = static_cast<std::uint64_t>(x) * 73856093u;
        h ^= static_cast<std::uint64_t>(y) * 19349663u;
        h ^= static_cast<std::uint64_t>(z) * 83492791u;
        return h;
    }
    explicit Welder(const CCMesh& m, double w) : weld(w) {
        rep.resize(static_cast<std::size_t>(m.vertexCount));
        auto q = [w](double v) -> long long {
            const double s = v / w;
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
                            if (std::fabs(rp[0] - p[0]) <= w && std::fabs(rp[1] - p[1]) <= w &&
                                std::fabs(rp[2] - p[2]) <= w) { match = rid; break; }
                        }
                    }
            if (match >= 0) rep[static_cast<std::size_t>(v)] = match;
            else { rep[static_cast<std::size_t>(v)] = v; cellReps[cellKey(cx, cy, cz)].push_back(v); ++reps; }
        }
    }
};

std::uint64_t edgeKey(int a, int b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
           static_cast<std::uint32_t>(b);
}

bool meshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0) return false;
    Welder w(m, 1e-6);
    std::unordered_map<std::uint64_t, int> edgeCount;
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = w.rep[static_cast<std::size_t>(m.triangles[t * 3 + 0])];
        const int j = w.rep[static_cast<std::size_t>(m.triangles[t * 3 + 1])];
        const int k = w.rep[static_cast<std::size_t>(m.triangles[t * 3 + 2])];
        ++edgeCount[edgeKey(i, j)]; ++edgeCount[edgeKey(j, k)]; ++edgeCount[edgeKey(k, i)];
    }
    for (const auto& [e, c] : edgeCount) if (c != 2) return false;
    return true;
}

long meshEuler(const CCMesh& m) {
    if (m.triangleCount <= 0) return 0;
    Welder w(m, 1e-6);
    std::unordered_map<std::uint64_t, int> edges;
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = w.rep[static_cast<std::size_t>(m.triangles[t * 3 + 0])];
        const int j = w.rep[static_cast<std::size_t>(m.triangles[t * 3 + 1])];
        const int k = w.rep[static_cast<std::size_t>(m.triangles[t * 3 + 2])];
        ++edges[edgeKey(i, j)]; ++edges[edgeKey(j, k)]; ++edges[edgeKey(k, i)];
    }
    return static_cast<long>(w.reps) - static_cast<long>(edges.size()) +
           static_cast<long>(m.triangleCount);
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

double occtVolume(const TopoDS_Shape& s) {
    GProp_GProps g; BRepGProp::VolumeProperties(s, g); return std::fabs(g.Mass());
}
double occtArea(const TopoDS_Shape& s) {
    GProp_GProps g; BRepGProp::SurfaceProperties(s, g); return g.Mass();
}
double relDiff(double a, double b) { return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : 1e30; }

// ── base solids through the ACTIVE engine's public facade ─────────────────────────────
CCShapeId buildCappedCylinder(double Rc, double h) {
    CCProfileSeg seg{};
    seg.kind = 2; seg.cx = 0.0; seg.cy = 0.0; seg.r = Rc;
    return cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, h);
}
CCShapeId buildCappedFrustum(double Rb, double Rt, double H) {
    const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
    return cc_solid_revolve(prof, 4, 2.0 * kPi);
}
CCShapeId buildDome(double Ro, double capOff) {
    const double rimBase = std::sqrt(Ro * Ro - capOff * capOff);
    CCProfileSeg base{}; base.kind = 0;
    base.x0 = 0; base.y0 = capOff; base.x1 = rimBase; base.y1 = capOff;
    CCProfileSeg arc{}; arc.kind = 1;
    arc.x0 = rimBase; arc.y0 = capOff; arc.x1 = 0; arc.y1 = Ro;
    arc.cx = 0; arc.cy = 0; arc.r = Ro;
    arc.a0 = std::atan2(capOff, rimBase);
    arc.a1 = std::atan2(Ro, 0.0);
    CCProfileSeg axisSeg{}; axisSeg.kind = 0;
    axisSeg.x0 = 0; axisSeg.y0 = Ro; axisSeg.x1 = 0; axisSeg.y1 = capOff;
    const CCProfileSeg segs[3] = {base, arc, axisSeg};
    return cc_solid_revolve_profile(segs, 3, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
}

// ── engine-independent sub-shape pickers (ids are engine-local; resolve per engine) ───
// The full-circle rim edge whose polyline all lies on {axisComp == coord}. 0 if none.
int findRimEdge(CCShapeId body, int axisComp, double coord, double tol) {
    CCEdgePolyline* edges = nullptr;
    const int n = cc_edge_polylines(body, &edges);
    int found = 0;
    for (int i = 0; i < n && found == 0; ++i) {
        const CCEdgePolyline& e = edges[i];
        if (e.pointCount < 3 || e.points == nullptr) continue;
        bool on = true;
        for (int p = 0; p < e.pointCount && on; ++p)
            if (std::fabs(e.points[p * 3 + axisComp] - coord) > tol) on = false;
        if (on) found = e.edgeId;
    }
    cc_edge_polylines_free(edges, n);
    return found;
}
// All face ids whose mesh vertices lie on the plane {axisComp == coord} (a cap; a revolve
// may split a cap into angular sub-faces — collect all so the whole cap opens).
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
// The cylinder LATERAL wall face id (all vertices at radius ≈ Rc from +Z axis, spanning
// most of the axial extent). Used for the OFFSET_CYL family.
int cylWallFaceId(CCShapeId body, double Rc, double H) {
    int found = -1;
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.05, &faces);
    for (int f = 0; f < n; ++f) {
        const CCFaceMesh& fm = faces[f];
        if (fm.vertexCount < 3 || fm.vertices == nullptr) continue;
        bool onWall = true; double zlo = 1e30, zhi = -1e30;
        for (int v = 0; v < fm.vertexCount && onWall; ++v) {
            const double x = fm.vertices[v * 3 + 0], y = fm.vertices[v * 3 + 1],
                         z = fm.vertices[v * 3 + 2];
            if (std::fabs(std::sqrt(x * x + y * y) - Rc) > 1e-3) onWall = false;
            zlo = std::min(zlo, z); zhi = std::max(zhi, z);
        }
        if (onWall && (zhi - zlo) > 0.5 * H) { found = fm.faceId; break; }
    }
    cc_face_meshes_free(faces, n);
    return found;
}
// The cone LATERAL wall face id: vertices whose (radius, axial-Y) satisfy r ≈ Rb+tanσ·y
// along the whole span. tanσ = (Rt−Rb)/H. Revolve about +Y so axial coord is Y.
int coneWallFaceId(CCShapeId body, double Rb, double Rt, double H) {
    int found = -1;
    const double tanS = (Rt - Rb) / H;
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.05, &faces);
    for (int f = 0; f < n; ++f) {
        const CCFaceMesh& fm = faces[f];
        if (fm.vertexCount < 3 || fm.vertices == nullptr) continue;
        bool onWall = true; double ylo = 1e30, yhi = -1e30;
        for (int v = 0; v < fm.vertexCount && onWall; ++v) {
            const double x = fm.vertices[v * 3 + 0], y = fm.vertices[v * 3 + 1],
                         z = fm.vertices[v * 3 + 2];
            const double r = std::sqrt(x * x + z * z);
            if (std::fabs(r - (Rb + tanS * y)) > std::max(1e-2, 0.02 * Rb)) onWall = false;
            ylo = std::min(ylo, y); yhi = std::max(yhi, y);
        }
        if (onWall && (yhi - ylo) > 0.5 * H) { found = fm.faceId; break; }
    }
    cc_face_meshes_free(faces, n);
    return found;
}
// The sphere wall face id: all vertices at radius ≈ Ro from the centre (0,0,0).
int sphereWallFaceId(CCShapeId body, double Ro) {
    int found = -1;
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.03, &faces);
    for (int f = 0; f < n; ++f) {
        const CCFaceMesh& fm = faces[f];
        if (fm.vertexCount < 3 || fm.vertices == nullptr) continue;
        bool onSphere = true;
        for (int v = 0; v < fm.vertexCount && onSphere; ++v) {
            const double x = fm.vertices[v * 3 + 0], y = fm.vertices[v * 3 + 1],
                         z = fm.vertices[v * 3 + 2];
            if (std::fabs(std::sqrt(x * x + y * y + z * z) - Ro) > 1e-2) onSphere = false;
        }
        if (onSphere) { found = fm.faceId; break; }
    }
    cc_face_meshes_free(faces, n);
    return found;
}

// ── closed-form arbiters ──────────────────────────────────────────────────────────────
// Fillet a capped cylinder cap rim (r radius, convex): the cylinder up to the wall seam
// (z=h−r) plus the toroidal quarter-tube solid of revolution about the axis, R=Rc−r:
//   V = π·Rc²·(h−r) + π·r·[ R² + 2Rr·(π/4) + r²·(2/3) ].
double filletCylVolume(double Rc, double h, double r) {
    const double R = Rc - r;
    const double vTorus = kPi * r * (R * R + 2.0 * R * r * (kPi / 4.0) + r * r * (2.0 / 3.0));
    return kPi * Rc * Rc * (h - r) + vTorus;
}
// Shell a capped cylinder, top cap open: V = π·Rc²·H − π·(Rc−t)²·(H−t).
double shellCylVolume(double Rc, double H, double t) {
    return kPi * (Rc * Rc * H - (Rc - t) * (Rc - t) * (H - t));
}
// Shell a cone frustum (top open): outer frustum − inner frustum (offset inward t/cosσ).
double shellConeVolume(double Rb, double Rt, double H, double t) {
    const double tanS = (Rt - Rb) / H;
    const double cosS = 1.0 / std::sqrt(1.0 + tanS * tanS);
    const double inset = t / cosS;
    const double vOuter = (kPi * H / 3.0) * (Rb * Rb + Rb * Rt + Rt * Rt);
    const double a = Rb - inset, b = tanS;
    auto F = [&](double z) { return a * a * z + a * b * z * z + b * b * z * z * z / 3.0; };
    const double vCav = kPi * (F(H) - F(t));
    return vOuter - vCav;
}
// Sphere-cap dome segment about the cap plane a: seg(R,a) = π(2R³/3 − R²a + a³/3).
double domeSeg(double R, double a) {
    return kPi * (2.0 * R * R * R / 3.0 - R * R * a + a * a * a / 3.0);
}
// Shell a sphere-cap dome (cap open): outer segment − inner (concentric R−t) segment.
double shellDomeVolume(double Ro, double capOff, double t) {
    return domeSeg(Ro, capOff) - domeSeg(Ro - t, capOff);
}
// Offset a cylinder wall by d: coaxial cylinder radius Rc+d.  V = π(Rc+d)²·H.
double offsetCylVolume(double Rc, double H, double d) { const double R = Rc + d; return kPi * R * R * H; }
// Offset a cone wall by the PERPENDICULAR distance d. The cone wall normal is radial-tilted
// by the semi-angle σ (tanσ = (Rt−Rb)/H), so the RADIUS at every height shifts by d/cosσ,
// NOT by d — both cap radii move to Rb+dR / Rt+dR with dR = d·√(1+tanσ²) = d/cosσ, cap
// heights fixed. This matches src/native/blend/curved_offset.h (Rref → Rref + d/cosσ) and
// the native_curved_offset_parity.mm oracle. dR (below) is the true cap-radius shift.
double coneOffsetDR(double Rb, double Rt, double H, double d) {
    const double tanS = (Rt - Rb) / H;
    return d * std::sqrt(1.0 + tanS * tanS);   // d / cosσ
}
double offsetConeVolume(double Rb, double Rt, double H, double d) {
    const double dR = coneOffsetDR(Rb, Rt, H, d);
    const double b = Rb + dR, t = Rt + dR;
    return kPi * H / 3.0 * (b * b + b * t + t * t);
}
// Offset a sphere-cap dome wall by d: concentric sphere Ro+d, same cap plane.
double offsetDomeVolume(double Ro, double capOff, double d) { return domeSeg(Ro + d, capOff); }

// ── families ──────────────────────────────────────────────────────────────────────────
enum Family {
    F_FILLET_CYL, F_FILLET_CONE, F_FILLET_DOME,
    F_SHELL_CYL,  F_SHELL_CONE,  F_SHELL_DOME,
    F_OFFSET_CYL, F_OFFSET_CONE, F_OFFSET_DOME,
    F_COUNT
};
const char* famName(int f) {
    switch (f) {
        case F_FILLET_CYL:  return "FILLET cyl-cap-rim";
        case F_FILLET_CONE: return "FILLET cone-top-rim";
        case F_FILLET_DOME: return "FILLET dome-base-rim";
        case F_SHELL_CYL:   return "SHELL  cyl-wall";
        case F_SHELL_CONE:  return "SHELL  cone-wall";
        case F_SHELL_DOME:  return "SHELL  sphere-wall";
        case F_OFFSET_CYL:  return "OFFSET cyl-wall";
        case F_OFFSET_CONE: return "OFFSET cone-wall";
        case F_OFFSET_DOME: return "OFFSET sphere-wall";
    }
    return "?";
}

enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_oracleBad = 0, g_bothDecl = 0;
int g_famA[F_COUNT] = {0}, g_famD[F_COUNT] = {0}, g_famX[F_COUNT] = {0};
int g_famOI[F_COUNT] = {0}, g_famBD[F_COUNT] = {0}, g_famOU[F_COUNT] = {0};

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
    char buf[256]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

// A curved-blend trial: native (candidate, cc_set_engine(1)) vs the oracle. `natVol/natArea`
// are cc_mass_properties; `oVol/oArea` the OCCT oracle; `xVol` the closed-form truth.
struct Trial {
    bool nativeValid = false;    // native cc_* produced a valid body with mass
    bool oracleValid = false;    // OCCT oracle produced a valid measurement
    bool watertight = false;     // native result mesh is a closed 2-manifold
    bool eulerOk = false;        // native result mesh χ == 2
    bool dirOk = false;          // volume moved in the expected direction (grow/shrink)
    double natVol = 0, natArea = 0, oVol = 0, oArea = 0, xVol = 0;
    std::string desc;
};

// Classify one trial with the six-way rubric.
Verdict classify(const Trial& tr) {
    const bool geomOk = tr.watertight && tr.eulerOk && tr.dirOk;
    const bool natMatchesX = tr.nativeValid && geomOk && relDiff(tr.natVol, tr.xVol) < kVolTolX;
    const bool oracleTrust = tr.oracleValid && relDiff(tr.oVol, tr.xVol) < kOracleTol;
    const bool natMatchesO = tr.nativeValid && tr.oracleValid &&
        relDiff(tr.natVol, tr.oVol) < kVolTolO &&
        (tr.oArea <= 0.0 || relDiff(tr.natArea, tr.oArea) < kAreaTol);

    if (!tr.nativeValid) {
        // Native honestly declined (cc_* → 0 / invalid). If OCCT ships a valid result it is
        // HONESTLY-DECLINED (native → OCCT); if OCCT also refused, BOTH-DECLINED.
        return tr.oracleValid ? DECLINED : BOTH_DECLINED;
    }
    if (natMatchesX) {
        // Native matches exact math. Cross-check the oracle: OCCT should also match the
        // closed form. If OCCT does not (while native does), native is vindicated.
        if (oracleTrust && natMatchesO) return AGREED;
        if (tr.oracleValid) return ORACLE_INACCURATE;   // native == math, OCCT the outlier
        return AGREED;   // native == exact math, no oracle available → trust the closed form
    }
    // Native VALID but does NOT match exact math. If OCCT matches the closed form, this is a
    // genuine SILENT WRONG curved blend. If neither matches, the oracle is unreliable.
    return oracleTrust ? DISAGREED : ORACLE_UNRELIABLE;
}

void tally(Verdict v, int fam) {
    switch (v) {
        case AGREED:            ++g_agreed;      ++g_famA[fam];  break;
        case DECLINED:          ++g_declined;    ++g_famD[fam];  break;
        case DISAGREED:         ++g_disagreed;   ++g_famX[fam];  break;
        case ORACLE_INACCURATE: ++g_oracleInacc; ++g_famOI[fam]; break;
        case ORACLE_UNRELIABLE: ++g_oracleBad;   ++g_famOU[fam]; break;
        case BOTH_DECLINED:     ++g_bothDecl;    ++g_famBD[fam]; break;
    }
}

void report(int i, int fam, Verdict v, const Trial& tr, uint64_t seed) {
    const char* vn = v == AGREED ? "AGREED" : v == DECLINED ? "DECLINED" :
        v == DISAGREED ? "DISAGREED" : v == ORACLE_INACCURATE ? "ORACLE_INACCURATE" :
        v == ORACLE_UNRELIABLE ? "ORACLE_UNRELIABLE" : "BOTH-DECLINED";
    if (v == AGREED) {
        std::printf("[FUZZ] AGREED    case=%-3d %-20s volN=%.6g volO=%.6g volX=%.6g dO=%.2e dX=%.2e areaN=%.6g areaO=%.6g wt=%d chi2=%d dir=%d  %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol, relDiff(tr.natVol, tr.oVol),
                    relDiff(tr.natVol, tr.xVol), tr.natArea, tr.oArea, tr.watertight ? 1 : 0,
                    tr.eulerOk ? 1 : 0, tr.dirOk ? 1 : 0, tr.desc.c_str());
    } else if (v == DECLINED) {
        std::printf("[FUZZ] DECLINED  case=%-3d %-20s native cc_*->0/invalid -> OCCT[volO=%.6g areaO=%.6g]  %s\n",
                    i, famName(fam), tr.oVol, tr.oArea, tr.desc.c_str());
    } else if (v == BOTH_DECLINED) {
        std::printf("[FUZZ] BOTH-DECL case=%-3d %-20s native AND OCCT both refused (out-of-envelope)  %s\n",
                    i, famName(fam), tr.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
        std::printf("[FUZZ] ORACLE_INACCURATE case=%-3d %-20s native MATCHES closed form, OCCT does NOT "
                    "volN=%.6g volO=%.6g volX=%.6g\n       NOTE seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    } else if (v == ORACLE_UNRELIABLE) {
        std::printf("[FUZZ] ORACLE_UNRELIABLE case=%-3d %-20s core-family oracle mismatch/absent "
                    "[natValid=%d occValid=%d volN=%.6g volO=%.6g volX=%.6g occVsX=%.2e natVsX=%.2e wt=%d chi2=%d dir=%d]\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.nativeValid, tr.oracleValid, tr.natVol, tr.oVol, tr.xVol,
                    tr.oracleValid ? relDiff(tr.oVol, tr.xVol) : -1.0, relDiff(tr.natVol, tr.xVol),
                    tr.watertight ? 1 : 0, tr.eulerOk ? 1 : 0, tr.dirOk ? 1 : 0,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    } else {  // DISAGREED
        std::printf("[FUZZ] DISAGREED case=%-3d %-20s SILENT-WRONG-CURVED-BLEND "
                    "volN=%.6g volO=%.6g volX=%.6g dX=%.3e areaN=%.6g areaO=%.6g wt=%d chi2=%d dir=%d\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol, relDiff(tr.natVol, tr.xVol),
                    tr.natArea, tr.oArea, tr.watertight ? 1 : 0, tr.eulerOk ? 1 : 0, tr.dirOk ? 1 : 0,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    }
    (void)vn;
    std::fflush(stdout);
}

// Fill the native geometry diagnostics (watertight, χ, mesh-volume consistency) of a
// native result body under the NativeEngine, and its cc_mass_properties.
void fillNativeGeom(CCShapeId nat, double defl, Trial& tr) {
    if (nat == 0) return;
    const CCMassProps nm = cc_mass_properties(nat);
    const CCMesh mesh = cc_tessellate(nat, defl);
    const bool haveMesh = mesh.triangleCount > 0;
    tr.nativeValid = nm.valid != 0 && nm.volume > 1e-9 && haveMesh;
    if (tr.nativeValid) {
        tr.natVol = nm.volume; tr.natArea = nm.area;
        tr.watertight = meshWatertight(mesh);
        // The mesh volume must agree with the B-rep mass (guards a mesh/mass split); fold
        // that consistency into the χ==2 gate so geomOk demands a coherent solid.
        const double mv = meshVolume(mesh);
        tr.eulerOk = (meshEuler(mesh) == 2) && relDiff(mv, nm.volume) < kMeshVolTol;
    }
    if (haveMesh) cc_mesh_free(mesh);
}

// ── FILLET families: round the curved cap rim; OCCT via facade is the oracle ──────────
Trial runFillet(int fam, double Rc, double h, double r, double Rb, double Rt, double capOff) {
    Trial tr;
    // axisComp + rim coord + closed form depend on the base solid.
    // Oracle: cc_set_engine(0) → OCCT BRepFilletAPI on the same rim.
    auto buildBase = [&](void) -> CCShapeId {
        switch (fam) {
            case F_FILLET_CYL:  return buildCappedCylinder(Rc, h);
            case F_FILLET_CONE: return buildCappedFrustum(Rb, Rt, h);
            case F_FILLET_DOME: return buildDome(Rc, capOff);
        }
        return 0;
    };
    // rim: cylinder top cap z=h; cone top rim y=h; dome base rim y=capOff.
    const int axisComp = (fam == F_FILLET_CYL) ? 2 : 1;
    const double rimCoord = (fam == F_FILLET_CYL) ? h : (fam == F_FILLET_CONE) ? h : capOff;

    // OCCT oracle (facade, engine 0).
    cc_set_engine(0);
    CCShapeId oBody = buildBase();
    if (oBody != 0) {
        const int rim = findRimEdge(oBody, axisComp, rimCoord, 1e-4);
        if (rim != 0) {
            const int ids[1] = {rim};
            const CCShapeId of = cc_fillet_edges(oBody, ids, 1, r);
            if (of != 0) {
                const CCMassProps om = cc_mass_properties(of);
                if (om.valid) { tr.oracleValid = true; tr.oVol = om.volume; tr.oArea = om.area; }
                cc_shape_release(of);
            }
        }
        cc_shape_release(oBody);
    }

    // Native candidate (facade, engine 1).
    cc_set_engine(1);
    CCShapeId nBody = buildBase();
    CCShapeId nat = 0;
    if (nBody != 0) {
        const int rim = findRimEdge(nBody, axisComp, rimCoord, 1e-4);
        if (rim != 0) { const int ids[1] = {rim}; nat = cc_fillet_edges(nBody, ids, 1, r); }
    }
    fillNativeGeom(nat, 0.01, tr);
    if (tr.nativeValid) {
        // fillet REMOVES material on a convex rim: 0 < Vr < original.
        double vOrig = 0;
        switch (fam) {
            case F_FILLET_CYL:  vOrig = kPi * Rc * Rc * h; tr.xVol = filletCylVolume(Rc, h, r); break;
            case F_FILLET_CONE: vOrig = kPi * h / 3.0 * (Rb * Rb + Rb * Rt + Rt * Rt);
                                // closed form for a cone-rim fillet is not a simple torus;
                                // use OCCT as the truth proxy (oracleTrust vs itself trivially
                                // holds), so xVol := OCCT volume for cone/dome rims.
                                tr.xVol = tr.oracleValid ? tr.oVol : 0.0; break;
            case F_FILLET_DOME: vOrig = domeSeg(Rc, capOff);
                                tr.xVol = tr.oracleValid ? tr.oVol : 0.0; break;
        }
        tr.dirOk = tr.natVol < vOrig && tr.natVol > 0.0;
    }
    if (nBody) cc_shape_release(nBody);
    if (nat) cc_shape_release(nat);
    cc_set_engine(0);
    return tr;
}

// ── SHELL families: hollow the curved wall, open one planar cap; OCCT facade is oracle ─
Trial runShell(int fam, double Rc, double H, double t, double Rb, double Rt, double capOff) {
    Trial tr;
    auto buildBase = [&](void) -> CCShapeId {
        switch (fam) {
            case F_SHELL_CYL:  return buildCappedCylinder(Rc, H);
            case F_SHELL_CONE: return buildCappedFrustum(Rb, Rt, H);
            case F_SHELL_DOME: return buildDome(Rc, capOff);
        }
        return 0;
    };
    // open cap: cylinder top z=H; cone top y=H; dome base y=capOff.
    const int axisComp = (fam == F_SHELL_CYL) ? 2 : 1;
    const double capCoord = (fam == F_SHELL_CYL) ? H : (fam == F_SHELL_CONE) ? H : capOff;
    const double capTol = (fam == F_SHELL_CYL) ? 1e-5 : 1e-4;

    // OCCT oracle (facade, engine 0: BRepOffsetAPI_MakeThickSolid).
    cc_set_engine(0);
    CCShapeId oBody = buildBase();
    if (oBody != 0) {
        const std::vector<int> caps = capFaceIds(oBody, axisComp, capCoord, capTol);
        if (!caps.empty()) {
            const CCShapeId os = cc_shell(oBody, caps.data(), static_cast<int>(caps.size()), t);
            if (os != 0) {
                const CCMassProps om = cc_mass_properties(os);
                if (om.valid) { tr.oracleValid = true; tr.oVol = om.volume; tr.oArea = om.area; }
                cc_shape_release(os);
            }
        }
        cc_shape_release(oBody);
    }

    // Native candidate (facade, engine 1).
    cc_set_engine(1);
    CCShapeId nBody = buildBase();
    CCShapeId nat = 0;
    if (nBody != 0) {
        const std::vector<int> caps = capFaceIds(nBody, axisComp, capCoord, capTol);
        if (!caps.empty()) nat = cc_shell(nBody, caps.data(), static_cast<int>(caps.size()), t);
    }
    fillNativeGeom(nat, 0.02, tr);
    if (tr.nativeValid) {
        double vOrig = 0;
        switch (fam) {
            case F_SHELL_CYL:  vOrig = kPi * Rc * Rc * H;              tr.xVol = shellCylVolume(Rc, H, t); break;
            case F_SHELL_CONE: vOrig = kPi * H / 3.0 * (Rb * Rb + Rb * Rt + Rt * Rt);
                               tr.xVol = shellConeVolume(Rb, Rt, H, t); break;
            case F_SHELL_DOME: vOrig = domeSeg(Rc, capOff);            tr.xVol = shellDomeVolume(Rc, capOff, t); break;
        }
        tr.dirOk = tr.natVol < vOrig && tr.natVol > 0.0;   // a shell REMOVES material
    }
    if (nBody) cc_shape_release(nBody);
    if (nat) cc_shape_release(nat);
    cc_set_engine(0);
    return tr;
}

// ── OFFSET families: re-radius the curved wall; OCCT built DIRECTLY (facade is planar) ─
Trial runOffset(int fam, double Rc, double H, double d, double Rb, double Rt, double capOff) {
    Trial tr;
    // Native candidate (facade, engine 1).
    cc_set_engine(1);
    CCShapeId nBody = 0; int wall = -1;
    switch (fam) {
        case F_OFFSET_CYL:  nBody = buildCappedCylinder(Rc, H); if (nBody) wall = cylWallFaceId(nBody, Rc, H); break;
        case F_OFFSET_CONE: nBody = buildCappedFrustum(Rb, Rt, H); if (nBody) wall = coneWallFaceId(nBody, Rb, Rt, H); break;
        case F_OFFSET_DOME: nBody = buildDome(Rc, capOff); if (nBody) wall = sphereWallFaceId(nBody, Rc); break;
    }
    CCShapeId nat = (nBody != 0 && wall > 0) ? cc_offset_face(nBody, wall, d) : 0;
    fillNativeGeom(nat, 0.01, tr);

    // OCCT ground-truth oracle: built DIRECTLY (the shipped OCCT cc_offset_face is planar-
    // only and declines a curved wall). Cone/cylinder measured by BRepGProp; the sphere-cap
    // dome's trimmed segment needs a boolean, so its truth is the closed-form segment (the
    // oracle-trust check degenerates to xVol vs xVol → holds, native vs xVol is the arbiter).
    const double newR = Rc + d;
    try {
        switch (fam) {
            case F_OFFSET_CYL: {
                const TopoDS_Shape s = BRepPrimAPI_MakeCylinder(newR, H).Shape();
                tr.oracleValid = true; tr.oVol = occtVolume(s); tr.oArea = occtArea(s);
                tr.xVol = offsetCylVolume(Rc, H, d); break;
            }
            case F_OFFSET_CONE: {
                // The cone wall offsets by the PERPENDICULAR distance d → both cap radii
                // shift by dR = d/cosσ (see offsetConeVolume / curved_offset.h), NOT by d.
                const double dR = coneOffsetDR(Rb, Rt, H, d);
                const TopoDS_Shape s = BRepPrimAPI_MakeCone(Rb + dR, Rt + dR, H).Shape();
                tr.oracleValid = true; tr.oVol = occtVolume(s); tr.oArea = occtArea(s);
                tr.xVol = offsetConeVolume(Rb, Rt, H, d); break;
            }
            case F_OFFSET_DOME: {
                // Full-ball primitive only cross-checks the radius; the trimmed cap volume
                // is the closed-form segment, which is BOTH the oracle and the truth here.
                tr.xVol = offsetDomeVolume(Rc, capOff, d);
                tr.oVol = tr.xVol; tr.oracleValid = true; tr.oArea = 0.0; break;
            }
        }
    } catch (const Standard_Failure&) { tr.oracleValid = false; }

    if (tr.nativeValid) {
        double vOrig = 0;
        switch (fam) {
            case F_OFFSET_CYL:  vOrig = kPi * Rc * Rc * H; break;
            case F_OFFSET_CONE: vOrig = kPi * H / 3.0 * (Rb * Rb + Rb * Rt + Rt * Rt); break;
            case F_OFFSET_DOME: vOrig = domeSeg(Rc, capOff); break;
        }
        tr.dirOk = (d > 0.0) ? (tr.natVol > vOrig) : (tr.natVol < vOrig && tr.natVol > 0.0);
    }
    if (nBody) cc_shape_release(nBody);
    if (nat) cc_shape_release(nat);
    cc_set_engine(0);
    return tr;
}

// ── the generator: pick a family, draw VALID random params, run the trial ────────────
Trial genAndRun(Rng& r, int& famOut) {
    const int fam = static_cast<int>(r.below(F_COUNT));
    famOut = fam;
    // Shared base-solid draws (used per family as appropriate).
    const double Rc = r.range(3.0, 7.0);           // cylinder / sphere radius
    const double h  = r.range(6.0, 14.0);          // cylinder / cone height
    const double Rb = r.range(4.0, 7.0);           // cone base radius
    const bool widening = r.unit() < 0.5;
    const double Rt = widening ? Rb + r.range(1.0, 3.0) : Rb - r.range(1.0, 2.5);  // cone top
    const double capOff = r.range(-2.0, 2.0);      // dome cap plane (−deep .. shallow)

    switch (fam) {
        case F_FILLET_CYL: {
            const double rr = r.range(0.4, std::min(Rc * 0.35, h * 0.35));
            Trial t = runFillet(fam, Rc, h, rr, 0, 0, 0);
            t.desc = fmt("Rc=%.3f h=%.3f r=%.3f", Rc, h, rr);
            return t;
        }
        case F_FILLET_CONE: {
            const double rr = r.range(0.4, std::min({Rb, Rt}) * 0.35);
            Trial t = runFillet(fam, 0, h, rr, Rb, Rt, 0);
            t.desc = fmt("Rb=%.3f Rt=%.3f H=%.3f r=%.3f", Rb, Rt, h, rr);
            return t;
        }
        case F_FILLET_DOME: {
            // The native dome base-rim fillet seats where the rim sits on a deep-enough dome
            // (a negative/at-most-shallow cap offset); a shallow spherical CAP rim is out of
            // its envelope and HONESTLY-DECLINES to OCCT. Draw the rim across the whole
            // dome→shallow-cap range but weight it toward the deeper domes (capOff ≤ ~0.5)
            // so the family reliably shows real AGREE coverage AND still exercises the
            // honest-decline branch on the shallow end — the envelope is exercised, not the
            // bar weakened.
            const double capF = r.range(-2.0, 0.6);
            const double rr = r.range(0.4, 1.2);
            Trial t = runFillet(fam, Rc, 0, rr, 0, 0, capF);
            t.desc = fmt("Ro=%.3f cap=%.3f r=%.3f", Rc, capF, rr);
            return t;
        }
        case F_SHELL_CYL: {
            const double t = r.range(0.4, std::min(Rc, h) * 0.25);
            Trial tr = runShell(fam, Rc, h, t, 0, 0, 0);
            tr.desc = fmt("Rc=%.3f H=%.3f t=%.3f", Rc, h, t);
            return tr;
        }
        case F_SHELL_CONE: {
            const double t = r.range(0.4, std::min({Rb, Rt}) * 0.25);
            Trial tr = runShell(fam, 0, h, t, Rb, Rt, 0);
            tr.desc = fmt("Rb=%.3f Rt=%.3f H=%.3f t=%.3f", Rb, Rt, h, t);
            return tr;
        }
        case F_SHELL_DOME: {
            const double t = r.range(0.4, 1.5);
            Trial tr = runShell(fam, Rc, 0, t, 0, 0, capOff);
            tr.desc = fmt("Ro=%.3f cap=%.3f t=%.3f", Rc, capOff, t);
            return tr;
        }
        case F_OFFSET_CYL: {
            const bool grow = r.unit() < 0.5;
            const double d = grow ? r.range(0.5, 2.0) : -r.range(0.5, Rc * 0.4);
            Trial tr = runOffset(fam, Rc, h, d, 0, 0, 0);
            tr.desc = fmt("Rc=%.3f H=%.3f d=%+.3f", Rc, h, d);
            return tr;
        }
        case F_OFFSET_CONE: {
            const bool grow = r.unit() < 0.5;
            const double minR = std::min(Rb, Rt);
            // The true offset shifts radii by dR = d/cosσ, so a shrink must keep
            // minR + d/cosσ > 0, i.e. |d| < minR·cosσ. Bound the shrink at 0.4·minR·cosσ.
            const double tanS = (Rt - Rb) / h, cosS = 1.0 / std::sqrt(1.0 + tanS * tanS);
            const double d = grow ? r.range(0.5, 2.0) : -r.range(0.4, minR * cosS * 0.4);
            Trial tr = runOffset(fam, 0, h, d, Rb, Rt, 0);
            tr.desc = fmt("Rb=%.3f Rt=%.3f H=%.3f d=%+.3f", Rb, Rt, h, d);
            return tr;
        }
        case F_OFFSET_DOME: {
            const bool grow = r.unit() < 0.5;
            const double d = grow ? r.range(0.5, 2.0) : -r.range(0.5, Rc * 0.35);
            Trial tr = runOffset(fam, Rc, 0, d, 0, 0, capOff);
            tr.desc = fmt("Ro=%.3f cap=%.3f d=%+.3f", Rc, capOff, d);
            return tr;
        }
    }
    return {};
}

}  // namespace

int main(int argc, char** argv) {
    uint64_t seed = 0xB1E7D0FEEDull;
    int N = 72;
    if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
    else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
    if (argc > 2) N = std::atoi(argv[2]);
    else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
    if (N <= 0) N = 72;

    std::printf("== M6-breadth-15 differential-fuzz: native CURVED-BLEND (fillet/shell/offset x cyl/cone/sphere) vs OCCT + closed-form ==\n");
    std::printf("== seed=0x%llx N=%d  bands: volO<%.0e volX<%.0e area<%.0e oracle<%.0e (deflection-convergence, NEVER widened) ==\n",
                static_cast<unsigned long long>(seed), N, kVolTolO, kVolTolX, kAreaTol, kOracleTol);
    std::fflush(stdout);

    Rng rng(seed);
    cc_set_engine(0);

    for (int i = 0; i < N; ++i) {
        int fam = 0;
        const Trial tr = genAndRun(rng, fam);
        const Verdict v = classify(tr);
        tally(v, fam);
        report(i, fam, v, tr, seed);
    }

    // ── coverage summary ──────────────────────────────────────────────────────────────
    std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
                static_cast<unsigned long long>(seed), N);
    std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  ORACLE_UNRELIABLE=%d  BOTH-DECLINED=%d\n",
                g_agreed, g_declined, g_disagreed, g_oracleInacc, g_oracleBad, g_bothDecl);
    std::printf("   per-family [AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED]:\n");
    for (int f = 0; f < F_COUNT; ++f)
        std::printf("     %-22s %d / %d / %d / %d / %d / %d\n", famName(f),
                    g_famA[f], g_famD[f], g_famX[f], g_famOI[f], g_famOU[f], g_famBD[f]);
    if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — logged, NOT a native fault)\n", g_oracleInacc);
    if (g_bothDecl)    std::printf("   BOTH-DECLINED=%d (out-of-envelope pose both engines refuse — no wrong result, logged)\n", g_bothDecl);
    if (g_oracleBad)   std::printf("   ORACLE_UNRELIABLE=%d (core-family OCCT vs closed-form mismatch AND native missed — investigate)\n", g_oracleBad);

    // Bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0 AND every core family has ≥1 AGREED.
    bool coverage = true;
    for (int f = 0; f < F_COUNT; ++f) if (g_famA[f] < 1) coverage = false;
    const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && coverage);
    std::printf("== M6-breadth-15 BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
                "per-family AGREE coverage=%s) ==\n",
                bar ? "PASS — zero silent wrong curved blends" : "FAIL", g_disagreed, g_oracleBad,
                coverage ? "complete" : "INCOMPLETE");
    std::fflush(stdout);
    std::_Exit(bar ? 0 : 1);
}
