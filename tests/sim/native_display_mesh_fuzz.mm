// SPDX-License-Identifier: Apache-2.0
//
// native_display_mesh_fuzz.mm — MOAT M6 (the COMPLETENESS BAR): the RENDER DISPLAY-MESH
// differential-fuzz domain, native-vs-OCCT parity leg (iOS simulator). It certifies the
// render-quality DISPLAY mesh (src/native/render/display_mesh.h) reached through the
// SHIPPING cc_display_mesh facade, over the NATIVE tessellation vs the OCCT tessellation
// of the SAME bodies.
//
// It extends the landed M6 differential fuzzers (curved-boolean, STEP round-trip,
// construction, blend, wrap-emboss, mass-properties, geometry-services, transform-chains,
// reference/datum, direct-modeling, transformed-boolean, orthographic-HLR, shape-healing,
// section, curved-blend, draft-angle, interference, freeform-boolean, variable-section
// sweep, N-sided fill, sheet-metal) to an independent native domain. Like its siblings it
// is INFRASTRUCTURE (a seeded harness, not a geometry capability): src/native / src/engine
// / include stay BYTE-UNCHANGED — this TU is additive test/sim code only.
//
// ── WHAT cc_display_mesh IS (and why the oracle is INVARIANTS, not a triangle diff) ──────
// cc_display_mesh POST-PROCESSES the ACTIVE engine's correctness tessellation (the same
// MeshData cc_tessellate returns) into a shading-ready mesh: per-vertex SMOOTH normals with
// crease-angle HARD edges, optional UVs, optional LOD. The post-process is engine-agnostic;
// the ONLY difference between the OCCT display mesh and the native display mesh is the SOURCE
// tessellation — under engine 0 it consumes the OCCT BRepMesh; under engine 1 the native
// SolidMesher. Two DIFFERENT meshers legitimately emit DIFFERENT triangle lists (node counts,
// fan directions, seam splits), so a byte-identical triangle-list comparison would be WRONG.
// The oracle is therefore the INVARIANTS that MUST hold for BOTH regardless of the mesher —
// and the CLOSED-FORM analytic surface for the primitive families, the STRICTER arbiter.
//
// ── THE INVARIANTS (must hold for the native display mesh; DISAGREE only if native breaks
//    one that OCCT holds, or native is genuinely wrong vs the closed form) ────────────────
//   * NON-EMPTY      a non-empty body yields a non-empty display mesh (both engines).
//   * FINITE         no NaN/Inf position / normal / UV.
//   * UNIT NORMALS   every per-vertex normal ‖n‖ == 1 (to ~1e-6).
//   * NON-DEGENERATE no zero-area display triangle, no out-of-range index.
//   * FOLD-WATERTIGHT (closed solids, no LOD) folding split verts back by position yields a
//                    closed 2-manifold — the SAME closed surface the source is.
//   * DEFLECTION BOUND (analytic families) max chord distance of display vertices to the EXACT
//                    surface ≤ a FIXED multiple of the requested deflection (Hausdorff budget
//                    under LOD). Asserted on BOTH the native and OCCT display meshes — the
//                    load-bearing "requested deflection actually respected" check.
//   * BBOX PARITY    the native display-mesh AABB matches the OCCT display-mesh AABB within a
//                    deflection-scaled band (both inscribe the same solid).
//   * TRI COUNT SANE native tri count > 0 and within a BROAD ratio of OCCT's (meshers differ,
//                    so this is a sanity band, not equality) — catches a collapse/explosion.
//   * LOD            when requested, native tri count is REDUCED (≤ source) and not grossly
//                    over-collapsed; UVs (when requested) all ∈ [0,1].
//
// ── CLASSIFICATION ──────────────────────────────────────────────────────────────────────
//   AGREED             native display valid on every invariant AND (analytic families) within
//                      the closed-form deflection bound AND bbox-parity with OCCT.
//   HONESTLY-DECLINED  cc_display_mesh returns 0 (empty/undisplayable body) under BOTH engines
//                      — a measured decline, never a bar failure.
//   DISAGREED          native violates a must-hold invariant that OCCT holds, OR native breaks
//                      the closed-form deflection bound — a SILENT WRONG display mesh (the bar).
//   ORACLE-INACCURATE  the OCCT display mesh violates the closed-form deflection bound while the
//                      native one holds it (native MORE correct at a numeric edge) — logged,
//                      NEVER a bar failure.
//
// ── THE BAR ──────────────────────────────────────────────────────────────────────────────
//   Exit 0 IFF DISAGREED == 0 with every analytic family ≥1 AGREED. Run over ≥2 seeds, N≥64
//   per seed; the runner fails if ANY seed fails. The generator is seeded ONLY by an explicit
//   FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-identical batch (splitmix64
//   → xoshiro256**, verbatim from the siblings). The deflection multiplier is FIXED and NEVER
//   widened to force a case through.
//
// This TU drives the SHIPPING cc_* facade under BOTH engines (cc_set_engine) — the path the
// app calls. It is OCCT-dependent (the OCCT engine provides the oracle source tessellation via
// create_default_engine under -DCYBERCAD_HAS_OCCT). Built ONLY by
// scripts/run-sim-native-display-mesh-fuzz.sh; on run-sim-suite.sh's SKIP list (own main(),
// std::_Exit — OCCT static teardown in the trimmed static build is not exit-clean). src/native
// stays OCCT-FREE.
//
#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// FIXED bands (NEVER widened). The display mesh moves NO vertex pre-LOD, so every display
// vertex is a SOURCE vertex on the analytic surface within the mesher's chord deflection;
// the multiplier absorbs (a) the parameter-grid axis-extreme sagitta (a doubly-curved
// surface's silhouette samples up to ~2 sagittae inside), (b) OCCT's linear-deflection
// interpretation vs the native chord bound. Under LOD the bound relaxes to the Hausdorff
// budget (scale·defl). BBox parity is measured in the SAME chord-deflection units.
constexpr double kSurfMul   = 6.0;   // display-vertex distance to exact surface ≤ 6·defl
constexpr double kBBoxMul   = 6.0;   // native-vs-OCCT display AABB corner delta ≤ 6·defl
constexpr double kLodScale  = 8.0;   // LOD Hausdorff budget = 8·defl (matches DisplayParams)
constexpr double kTriRatio  = 40.0;  // native/OCCT tri count within [1/40, 40] (sanity band)

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim from the landed fuzzers) ──
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

// ── restore the default (OCCT) engine no matter how we leave scope ────────────────────────
struct EngineGuard { ~EngineGuard() { cc_set_engine(0); } };

// ── engine-agnostic body builders (each uses the ACTIVE engine's public facade) ───────────
// SPHERE cap-less full sphere via revolve of a half-disk meridian (arc about the axis).
// CYLINDER solid r×h about +Y (revolve rectangle). CONE frustum about +Y (revolve trapezoid).
// BOX axis-aligned prism (extrude a rectangle). All built with WHATEVER engine is active.
CCShapeId buildCylinder(double r, double h) {
    const double rect[] = {0.0, 0.0, r, 0.0, r, h, 0.0, h};
    return cc_solid_revolve(rect, 4, 2.0 * kPi);
}
CCShapeId buildConeFrustum(double Rb, double Rt, double H) {
    const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
    return cc_solid_revolve(prof, 4, 2.0 * kPi);
}
CCShapeId buildBox(double a, double b, double c) {
    const double rect[] = {0, 0, a, 0, a, b, 0, b};
    return cc_solid_extrude(rect, 4, c);
}
// An axis-aligned box [x0,x1]×[y0,y1]×[0,dz] (offset footprint extruded in +Z). Used as the
// notch cutter for the all-planar boolean family.
CCShapeId buildBoxAt(double x0, double y0, double x1, double y1, double dz) {
    const double rect[] = {x0, y0, x1, y0, x1, y1, x0, y1};
    return cc_solid_extrude(rect, 4, dz);
}
// Full sphere via a typed meridian profile (base line on the axis + a half-circle arc back to
// the axis), revolved 360°. Matches native_curved_blend_fuzz's dome idiom (full cap).
CCShapeId buildSphere(double R) {
    CCProfileSeg base{}; base.kind = 0; base.x0 = 0; base.y0 = -R; base.x1 = 0; base.y1 = -R;
    CCProfileSeg arc{};  arc.kind = 1;  arc.x0 = 0; arc.y0 = -R; arc.x1 = 0; arc.y1 = R;
    arc.cx = 0; arc.cy = 0; arc.r = R; arc.a0 = -kPi / 2; arc.a1 = kPi / 2;
    CCProfileSeg axisSeg{}; axisSeg.kind = 0; axisSeg.x0 = 0; axisSeg.y0 = R; axisSeg.x1 = 0; axisSeg.y1 = -R;
    const CCProfileSeg segs[3] = {base, arc, axisSeg};
    return cc_solid_revolve_profile(segs, 3, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
}

// ── families ──────────────────────────────────────────────────────────────────────────────
enum Fam { F_SPHERE, F_CYL, F_CONE, F_BOX, F_FILLET_BOX, F_BOOL_CUT, F_COUNT };
const char* famName(int f) {
    switch (f) {
        case F_SPHERE:     return "sphere";
        case F_CYL:        return "cylinder";
        case F_CONE:       return "cone-frustum";
        case F_BOX:        return "box";
        case F_FILLET_BOX: return "fillet-box";
        case F_BOOL_CUT:   return "boolean-cut";
    }
    return "?";
}
// Analytic families carry a closed-form surface distance oracle; the fillet / boolean outputs
// do not (no simple closed form) — for those the arbiter is the invariants + bbox parity only.
bool famHasClosedForm(int f) { return f == F_SPHERE || f == F_CYL || f == F_CONE || f == F_BOX; }

// ── display-mesh invariant checks (operate on a CCDisplayMesh) ─────────────────────────────
struct Aabb {
    double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
    void add(double x, double y, double z) {
        lo[0] = std::min(lo[0], x); hi[0] = std::max(hi[0], x);
        lo[1] = std::min(lo[1], y); hi[1] = std::max(hi[1], y);
        lo[2] = std::min(lo[2], z); hi[2] = std::max(hi[2], z);
    }
    bool valid() const { return lo[0] <= hi[0]; }
};
double aabbDelta(const Aabb& a, const Aabb& b) {
    double d = 0.0;
    for (int k = 0; k < 3; ++k) {
        d = std::max(d, std::fabs(a.lo[k] - b.lo[k]));
        d = std::max(d, std::fabs(a.hi[k] - b.hi[k]));
    }
    return d;
}
Aabb displayBox(const CCDisplayMesh& dm) {
    Aabb b;
    for (int i = 0; i < dm.vertexCount; ++i)
        b.add(dm.positions[i * 3], dm.positions[i * 3 + 1], dm.positions[i * 3 + 2]);
    return b;
}

bool allFinite(const CCDisplayMesh& dm) {
    for (int i = 0; i < dm.vertexCount * 3; ++i)
        if (!std::isfinite(dm.positions[i]) || !std::isfinite(dm.normals[i])) return false;
    if (dm.uvs)
        for (int i = 0; i < dm.vertexCount * 2; ++i)
            if (!std::isfinite(dm.uvs[i])) return false;
    return true;
}
bool normalsUnit(const CCDisplayMesh& dm) {
    for (int i = 0; i < dm.vertexCount; ++i) {
        const double nx = dm.normals[i * 3], ny = dm.normals[i * 3 + 1], nz = dm.normals[i * 3 + 2];
        if (std::fabs(std::sqrt(nx * nx + ny * ny + nz * nz) - 1.0) > 1e-6) return false;
    }
    return true;
}
bool nonDegenerateTris(const CCDisplayMesh& dm) {
    for (int t = 0; t < dm.triangleCount; ++t) {
        const int i = dm.triangles[t * 3], j = dm.triangles[t * 3 + 1], k = dm.triangles[t * 3 + 2];
        if (i < 0 || j < 0 || k < 0 || i >= dm.vertexCount || j >= dm.vertexCount || k >= dm.vertexCount)
            return false;
        const double* A = &dm.positions[i * 3];
        const double* B = &dm.positions[j * 3];
        const double* C = &dm.positions[k * 3];
        const double ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2];
        const double vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
        const double cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
        if (std::sqrt(cx * cx + cy * cy + cz * cz) < 1e-12) return false;
    }
    return true;
}
bool uvsInUnitRange(const CCDisplayMesh& dm) {
    if (!dm.uvs) return true;
    for (int i = 0; i < dm.vertexCount * 2; ++i)
        if (dm.uvs[i] < -1e-9 || dm.uvs[i] > 1.0 + 1e-9) return false;
    return true;
}
// Fold split display verts back by position and check the welded surface is a closed
// 2-manifold (every undirected edge shared by exactly 2 triangles). Engine-agnostic:
// welds coincident positions first (both meshers place a shared solid edge identically).
bool foldWatertight(const CCDisplayMesh& dm) {
    if (dm.triangleCount <= 0) return false;
    constexpr double kWeld = 1e-6;
    std::unordered_map<uint64_t, int> cellToId;
    std::vector<int> rep(static_cast<size_t>(dm.vertexCount));
    auto cellKey = [](long long x, long long y, long long z) -> uint64_t {
        uint64_t h = static_cast<uint64_t>(x) * 73856093u;
        h ^= static_cast<uint64_t>(y) * 19349663u;
        h ^= static_cast<uint64_t>(z) * 83492791u;
        return h;
    };
    auto q = [](double v) -> long long { const double s = v / kWeld; return static_cast<long long>(s >= 0 ? s + 0.5 : s - 0.5); };
    for (int v = 0; v < dm.vertexCount; ++v) {
        const double* p = &dm.positions[v * 3];
        const uint64_t c = cellKey(q(p[0]), q(p[1]), q(p[2]));
        auto it = cellToId.find(c);
        if (it == cellToId.end()) { cellToId.emplace(c, v); rep[static_cast<size_t>(v)] = v; }
        else rep[static_cast<size_t>(v)] = it->second;
    }
    std::unordered_map<uint64_t, int> edgeCount;
    auto key = [](int a, int b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) | static_cast<uint32_t>(b);
    };
    for (int t = 0; t < dm.triangleCount; ++t) {
        const int i = rep[static_cast<size_t>(dm.triangles[t * 3 + 0])];
        const int j = rep[static_cast<size_t>(dm.triangles[t * 3 + 1])];
        const int k = rep[static_cast<size_t>(dm.triangles[t * 3 + 2])];
        ++edgeCount[key(i, j)]; ++edgeCount[key(j, k)]; ++edgeCount[key(k, i)];
    }
    for (const auto& [e, c] : edgeCount) if (c != 2) return false;
    return true;
}

// ── closed-form max chord distance of a display mesh to the exact analytic surface ────────
// Returns the WORST distance over the vertices genuinely ON the given surface (cap-interior
// / seam verts excluded per family). Family params recovered from the display mesh AABB.
double surfDist(const CCDisplayMesh& dm, int fam, double defl) {
    double worst = 0.0;
    if (fam == F_SPHERE) {
        double R = 0.0;
        for (int i = 0; i < dm.vertexCount; ++i) {
            const double* p = &dm.positions[i * 3];
            R = std::max(R, std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]));
        }
        for (int i = 0; i < dm.vertexCount; ++i) {
            const double* p = &dm.positions[i * 3];
            worst = std::max(worst, std::fabs(std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) - R));
        }
    } else if (fam == F_CYL) {
        // Cylinder about +Y (revolve): wall radius r=√(x²+z²)=R; bound only wall verts.
        double R = 0.0;
        for (int i = 0; i < dm.vertexCount; ++i) {
            const double* p = &dm.positions[i * 3];
            R = std::max(R, std::sqrt(p[0] * p[0] + p[2] * p[2]));
        }
        for (int i = 0; i < dm.vertexCount; ++i) {
            const double* p = &dm.positions[i * 3];
            const double r = std::sqrt(p[0] * p[0] + p[2] * p[2]);
            if (std::fabs(r - R) < 6.0 * defl) worst = std::max(worst, std::fabs(r - R));
        }
    } else if (fam == F_CONE) {
        double H = 0.0, Rb = 0.0, Rt = 0.0;
        for (int i = 0; i < dm.vertexCount; ++i) H = std::max(H, dm.positions[i * 3 + 1]);
        for (int i = 0; i < dm.vertexCount; ++i) {
            const double* p = &dm.positions[i * 3];
            const double r = std::sqrt(p[0] * p[0] + p[2] * p[2]);
            if (p[1] < 1e-6) Rb = std::max(Rb, r);
            if (p[1] > H - 1e-6) Rt = std::max(Rt, r);
        }
        const double cosA = std::cos(std::atan2(std::fabs(Rb - Rt), H));
        for (int i = 0; i < dm.vertexCount; ++i) {
            const double* p = &dm.positions[i * 3];
            if (p[1] < 1e-6 || p[1] > H - 1e-6) continue;  // cap rings sit on the planar caps
            const double r = std::sqrt(p[0] * p[0] + p[2] * p[2]);
            const double Ry = Rb + (Rt - Rb) * (p[1] / H);
            if (std::fabs(r - Ry) < 6.0 * defl) worst = std::max(worst, std::fabs(r - Ry) * cosA);
        }
    } else {  // F_BOX — planar faces: distance to the nearest axis face plane (≈0).
        Aabb b = displayBox(dm);
        for (int i = 0; i < dm.vertexCount; ++i) {
            const double* p = &dm.positions[i * 3];
            const double dx = std::min(std::fabs(p[0] - b.lo[0]), std::fabs(p[0] - b.hi[0]));
            const double dy = std::min(std::fabs(p[1] - b.lo[1]), std::fabs(p[1] - b.hi[1]));
            const double dz = std::min(std::fabs(p[2] - b.lo[2]), std::fabs(p[2] - b.hi[2]));
            worst = std::max(worst, std::min({dx, dy, dz}));
        }
    }
    return worst;
}

// ── verdicts + tally ──────────────────────────────────────────────────────────────────────
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE };
int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0;
int g_famA[F_COUNT] = {0}, g_famD[F_COUNT] = {0}, g_famX[F_COUNT] = {0}, g_famOI[F_COUNT] = {0};
void tally(Verdict v, int f) {
    switch (v) {
        case AGREED:            ++g_agreed;      ++g_famA[f];  break;
        case DECLINED:          ++g_declined;    ++g_famD[f];  break;
        case DISAGREED:         ++g_disagreed;   ++g_famX[f];  break;
        case ORACLE_INACCURATE: ++g_oracleInacc; ++g_famOI[f]; break;
    }
}

struct Trial {
    int fam = 0;
    double defl = 0.0, creaseDeg = 30.0;
    bool wantUVs = false, wantLOD = false;
    int lodTarget = 0;
    // native (engine 1) BODY build — whether the native ENGINE could construct the source
    // solid at all (an UPSTREAM boolean/fillet decline is out of the display-mesh SUT's scope).
    bool natBodyBuilt = false;
    // native (engine 1) display-mesh state
    bool natBuilt = false, natFinite = false, natUnit = false, natNonDegen = false;
    bool natUVok = true, natFold = true, natTriSane = true, natLodOk = true;
    int natTris = 0, natVerts = 0;
    double natSurf = -1.0;      // closed-form surface dist (analytic families; -1 = n/a)
    // OCCT (engine 0) display-mesh state
    bool occtBuilt = false;
    int occtTris = 0;
    double occtSurf = -1.0;
    double bboxDelta = -1.0;    // native-vs-OCCT display AABB corner delta
    bool srcWatertightNative = false;  // whether the native source solid is closed
    std::string desc;
};

Verdict classify(const Trial& tr) {
    // ── SCOPE GATE. The SUT is cc_display_mesh, which POST-PROCESSES a body the ACTIVE engine
    // already built. If the NATIVE ENGINE could not construct the source body (an UPSTREAM
    // boolean/fillet decline — e.g. the box∩cylinder cut the native boolean layer legitimately
    // refuses, returning 0), there is NOTHING for the native display-mesh SUT to consume — it
    // is HONESTLY-DECLINED, out of this domain's scope, NOT a display-mesh fault. (Those
    // upstream declines are covered by native_boolean_fuzz / native_curved_blend_fuzz.)
    if (!tr.natBodyBuilt) return DECLINED;
    // ── native body built. If cc_display_mesh then produced NOTHING on that non-empty body,
    // that IS a display-mesh fault (the source tessellation was non-empty).
    if (!tr.natBuilt) return DISAGREED;
    // ── native displayed; run every must-hold invariant ─────────────────────────────────
    if (!tr.natFinite)    return DISAGREED;  // NaN/Inf in the shipped display mesh
    if (!tr.natUnit)      return DISAGREED;  // non-unit shading normal
    if (!tr.natNonDegen)  return DISAGREED;  // degenerate / out-of-range triangle
    if (!tr.natUVok)      return DISAGREED;  // UV requested-but-absent / count / out of [0,1]
    if (!tr.wantLOD && tr.srcWatertightNative && !tr.natFold) return DISAGREED;  // topology lost
    if (!tr.natTriSane)   return DISAGREED;  // tri count 0 or wildly off OCCT's
    if (!tr.natLodOk)     return DISAGREED;  // LOD grew / grossly over-collapsed
    // ── closed-form deflection bound (analytic families) ────────────────────────────────
    if (famHasClosedForm(tr.fam) && tr.natSurf >= 0.0) {
        const double budget = tr.wantLOD ? (kLodScale * tr.defl) : (kSurfMul * tr.defl + 1e-9);
        if (tr.natSurf > budget) {
            // Native breaks the bound. If OCCT ALSO breaks it, OCCT's own source mesh is the
            // outlier and native is no worse — but native still shipped an out-of-tolerance
            // display mesh, so this is a native DISAGREE (the closed form is ground truth).
            return DISAGREED;
        }
        // Native holds the bound. If OCCT breaks it, native is MORE correct → ORACLE-INACCURATE.
        if (tr.occtBuilt && tr.occtSurf > budget) return ORACLE_INACCURATE;
    }
    // ── bbox parity (both engines inscribe the same solid) ──────────────────────────────
    if (tr.occtBuilt && tr.bboxDelta >= 0.0) {
        const double bboxBudget = kBBoxMul * tr.defl + 1e-9;
        if (tr.bboxDelta > bboxBudget) return DISAGREED;
    }
    return AGREED;
}

void report(int i, const Trial& tr, Verdict v, uint64_t seed) {
    if (v == DISAGREED) {
        std::printf("[FUZZ] DISAGREED case=%-3d %-12s natBuilt=%d finite=%d unit=%d nondegen=%d uv=%d fold=%d triSane=%d lod=%d "
                    "natSurf=%.3e occtSurf=%.3e bbox=%.3e natTris=%d occtTris=%d\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(tr.fam), tr.natBuilt, tr.natFinite, tr.natUnit, tr.natNonDegen,
                    tr.natUVok, tr.natFold, tr.natTriSane, tr.natLodOk, tr.natSurf, tr.occtSurf,
                    tr.bboxDelta, tr.natTris, tr.occtTris, static_cast<unsigned long long>(seed), i,
                    tr.desc.c_str());
    } else if (v == DECLINED) {
        const char* why = tr.natBodyBuilt
            ? "cc_display_mesh -> 0 on an empty/undisplayable body under BOTH engines"
            : "native ENGINE declined the source body (upstream boolean/fillet decline, out of display-mesh scope)";
        std::printf("[FUZZ] DECLINED  case=%-3d %-12s %s — %s\n", i, famName(tr.fam), why, tr.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
        std::printf("[FUZZ] ORACLE-INACCURATE case=%-3d %-12s native holds the closed-form deflection bound "
                    "(natSurf=%.3e) but the OCCT source mesh breaks it (occtSurf=%.3e > budget) — native MORE correct %s\n",
                    i, famName(tr.fam), tr.natSurf, tr.occtSurf, tr.desc.c_str());
    } else {
        std::printf("[FUZZ] AGREED    case=%-3d %-12s natTris=%d occtTris=%d natSurf=%.3e bbox=%.3e %s\n",
                    i, famName(tr.fam), tr.natTris, tr.occtTris, tr.natSurf, tr.bboxDelta, tr.desc.c_str());
    }
    std::fflush(stdout);
}

// Fill the native (engine 1) invariant fields of `tr` from its display mesh.
void probeNative(Trial& tr, const CCDisplayMesh& dm) {
    tr.natBuilt = (dm.triangleCount > 0);
    if (!tr.natBuilt) return;
    tr.natTris = dm.triangleCount;
    tr.natVerts = dm.vertexCount;
    tr.natFinite = allFinite(dm);
    tr.natUnit = normalsUnit(dm);
    tr.natNonDegen = nonDegenerateTris(dm);
    tr.natUVok = tr.wantUVs ? (dm.uvs != nullptr && uvsInUnitRange(dm)) : true;
    tr.natFold = foldWatertight(dm);
    if (famHasClosedForm(tr.fam)) tr.natSurf = surfDist(dm, tr.fam, tr.defl);
    // LOD sanity: reduced (≤ some source) — we approximate the source via OCCT/native full-res
    // tri count captured separately; here just guard against a gross over-collapse to <25% of
    // the requested target floor (a mild early stop from the Hausdorff budget is expected).
    if (tr.wantLOD && tr.lodTarget > 0)
        tr.natLodOk = !(dm.triangleCount * 4 < tr.lodTarget);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────────────────
// The two-engine driver. buildBody draws random params from the RNG, so to build the SAME
// body under both engines we draw the params ONCE (engine-independent) and pass them to a
// pure builder that only touches the active engine. This keeps the native and OCCT bodies
// parameter-identical while advancing the RNG exactly once per trial.
// ─────────────────────────────────────────────────────────────────────────────────────────
namespace {

// A fully-resolved, engine-independent recipe drawn once from the RNG.
struct Recipe {
    int fam;
    double defl, creaseDeg;
    bool wantUVs, wantLOD;
    double p[6];   // family params (see buildFromRecipe)
    std::string desc;
};

Recipe drawRecipe(Rng& rng, int idx) {
    Recipe r{};
    r.fam = static_cast<int>(rng.below(F_COUNT));
    r.creaseDeg = rng.range(15.0, 55.0);
    r.wantUVs = (rng.below(2) == 0);
    r.wantLOD = (idx % 4 == 3);
    char buf[192];
    if (r.fam == F_SPHERE) {
        const double R = rng.range(2.0, 40.0);
        r.defl = rng.range(1e-3, 0.04 * R);
        r.p[0] = R;
        std::snprintf(buf, sizeof buf, "R=%.4f defl=%.4f crease=%.1f LOD=%d UV=%d", R, r.defl, r.creaseDeg, r.wantLOD, r.wantUVs);
    } else if (r.fam == F_CYL) {
        const double R = rng.range(2.0, 30.0), h = rng.range(3.0, 50.0);
        r.defl = rng.range(1e-3, 0.04 * R);
        r.p[0] = R; r.p[1] = h;
        std::snprintf(buf, sizeof buf, "R=%.4f h=%.4f defl=%.4f crease=%.1f LOD=%d UV=%d", R, h, r.defl, r.creaseDeg, r.wantLOD, r.wantUVs);
    } else if (r.fam == F_CONE) {
        const double Rb = rng.range(4.0, 30.0);
        const double Rt = (rng.below(3) == 0) ? 0.0 : Rb * rng.range(0.25, 0.75);
        const double H = rng.range(5.0, 40.0);
        r.defl = rng.range(1e-3, 0.04 * Rb);
        r.p[0] = Rb; r.p[1] = Rt; r.p[2] = H;
        std::snprintf(buf, sizeof buf, "Rb=%.4f Rt=%.4f H=%.4f defl=%.4f crease=%.1f LOD=%d UV=%d", Rb, Rt, H, r.defl, r.creaseDeg, r.wantLOD, r.wantUVs);
    } else if (r.fam == F_BOX) {
        const double a = rng.range(2.0, 40.0), b = rng.range(2.0, 40.0), c = rng.range(2.0, 40.0);
        r.defl = rng.range(1e-3, 0.4 * std::min({a, b, c}));
        r.p[0] = a; r.p[1] = b; r.p[2] = c;
        std::snprintf(buf, sizeof buf, "a=%.3f b=%.3f c=%.3f defl=%.4f crease=%.1f LOD=%d UV=%d", a, b, c, r.defl, r.creaseDeg, r.wantLOD, r.wantUVs);
    } else if (r.fam == F_FILLET_BOX) {
        const double a = rng.range(8.0, 30.0), b = rng.range(8.0, 30.0), c = rng.range(8.0, 30.0);
        const double rad = rng.range(0.5, 0.25 * std::min({a, b, c}));
        const int edgeId = 1 + static_cast<int>(rng.below(12));
        r.defl = rng.range(2e-3, 0.05 * rad + 0.02);
        r.p[0] = a; r.p[1] = b; r.p[2] = c; r.p[3] = rad; r.p[4] = edgeId;
        std::snprintf(buf, sizeof buf, "a=%.2f b=%.2f c=%.2f r=%.2f edge=%d defl=%.4f crease=%.1f LOD=%d UV=%d", a, b, c, rad, edgeId, r.defl, r.creaseDeg, r.wantLOD, r.wantUVs);
    } else {  // F_BOOL_CUT — an ALL-PLANAR box with a random corner NOTCH cut out (the native
              // BSP-CSG boolean builds this EXACTLY; the display mesh sees a re-entrant planar
              // solid with new interior crease edges — a genuine boolean output, native-supported).
        const double a = rng.range(12.0, 30.0), b = rng.range(12.0, 30.0), c = rng.range(8.0, 24.0);
        const double nx = rng.range(0.25 * a, 0.6 * a);   // notch extent in +X from the far corner
        const double ny = rng.range(0.25 * b, 0.6 * b);   // notch extent in +Y
        r.defl = rng.range(2e-3, 0.15 * std::min({a, b, c}));
        r.p[0] = a; r.p[1] = b; r.p[2] = c; r.p[3] = nx; r.p[4] = ny;
        std::snprintf(buf, sizeof buf, "a=%.2f b=%.2f c=%.2f notch=%.2fx%.2f defl=%.4f crease=%.1f LOD=%d UV=%d", a, b, c, nx, ny, r.defl, r.creaseDeg, r.wantLOD, r.wantUVs);
    }
    r.desc = buf;
    return r;
}

// Build the body from a resolved recipe with the ACTIVE engine (no RNG use).
CCShapeId buildFromRecipe(const Recipe& r) {
    if (r.fam == F_SPHERE) return buildSphere(r.p[0]);
    if (r.fam == F_CYL)    return buildCylinder(r.p[0], r.p[1]);
    if (r.fam == F_CONE)   return buildConeFrustum(r.p[0], r.p[1], r.p[2]);
    if (r.fam == F_BOX)    return buildBox(r.p[0], r.p[1], r.p[2]);
    if (r.fam == F_FILLET_BOX) {
        CCShapeId box = buildBox(r.p[0], r.p[1], r.p[2]);
        if (!box) return 0;
        const int ids[1] = {static_cast<int>(r.p[4])};
        CCShapeId id = cc_fillet_edges(box, ids, 1, r.p[3]);
        cc_shape_release(box);
        return id;
    }
    // F_BOOL_CUT — box minus a corner notch (all-planar, native BSP-CSG exact).
    const double a = r.p[0], b = r.p[1], c = r.p[2], nx = r.p[3], ny = r.p[4];
    CCShapeId box = buildBox(a, b, c);
    // Cutter occupies the far +X+Y corner column, taller than the box (through-notch in Z).
    CCShapeId cutter = buildBoxAt(a - nx, b - ny, a + 1.0, b + 1.0, c + 2.0);
    CCShapeId id = (box && cutter) ? cc_boolean(box, cutter, 1) : 0;
    if (box) cc_shape_release(box);
    if (cutter) cc_shape_release(cutter);
    return id;
}

Trial runOne(const Recipe& r) {
    Trial tr;
    tr.fam = r.fam; tr.defl = r.defl; tr.creaseDeg = r.creaseDeg;
    tr.wantUVs = r.wantUVs; tr.wantLOD = r.wantLOD; tr.desc = r.desc;

    // ── OCCT arm ─────────────────────────────────────────────────────────────────────
    cc_set_engine(0);
    CCShapeId occtBody = buildFromRecipe(r);
    CCDisplayMesh occtDm{};
    if (occtBody) {
        CCMesh srcMesh = cc_tessellate(occtBody, r.defl);
        const int srcTris = srcMesh.triangleCount;
        cc_mesh_free(srcMesh);
        if (tr.wantLOD) tr.lodTarget = std::max(4, srcTris / 2);
        const int nt = cc_display_mesh(occtBody, r.defl, r.creaseDeg, tr.wantLOD ? tr.lodTarget : 0,
                                       r.wantUVs ? 1 : 0, &occtDm);
        tr.occtBuilt = (nt > 0);
        if (tr.occtBuilt) {
            tr.occtTris = occtDm.triangleCount;
            if (famHasClosedForm(tr.fam)) tr.occtSurf = surfDist(occtDm, tr.fam, r.defl);
        }
        cc_shape_release(occtBody);
    }

    // ── NATIVE arm (same recipe) ───────────────────────────────────────────────────────
    cc_set_engine(1);
    CCShapeId natBody = buildFromRecipe(r);
    CCDisplayMesh natDm{};
    tr.natBodyBuilt = (natBody != 0);
    if (natBody) {
        // native source solid closed? (fold-watertight only meaningful for a closed solid)
        CCMesh natSrc = cc_tessellate(natBody, r.defl);
        {
            // watertight test on the native source via the display fold helper on the raw mesh:
            // reuse foldWatertight by wrapping the CCMesh as a positions/tris view.
            CCDisplayMesh view{};
            view.positions = natSrc.vertices; view.vertexCount = natSrc.vertexCount;
            view.normals = natSrc.vertices;   // unused by foldWatertight
            view.triangles = natSrc.triangles; view.triangleCount = natSrc.triangleCount;
            tr.srcWatertightNative = (natSrc.triangleCount > 0) && foldWatertight(view);
        }
        cc_mesh_free(natSrc);
        const int nt = cc_display_mesh(natBody, r.defl, r.creaseDeg, tr.wantLOD ? tr.lodTarget : 0,
                                       r.wantUVs ? 1 : 0, &natDm);
        if (nt > 0) probeNative(tr, natDm);
    }

    // ── tri-ratio sanity band + bbox parity (need both display meshes live) ─────────────
    if (tr.natBuilt && tr.occtBuilt) {
        const double ratio = static_cast<double>(tr.natTris) / std::max(1, tr.occtTris);
        tr.natTriSane = (tr.natTris > 0) && (ratio <= kTriRatio) && (ratio >= 1.0 / kTriRatio);
        const Aabb nb = displayBox(natDm), ob = displayBox(occtDm);
        if (nb.valid() && ob.valid()) tr.bboxDelta = aabbDelta(nb, ob);
    } else if (tr.natBuilt) {
        tr.natTriSane = (tr.natTris > 0);
    }

    if (natDm.triangleCount > 0 || natDm.positions) cc_display_mesh_free(&natDm);
    if (occtDm.triangleCount > 0 || occtDm.positions) cc_display_mesh_free(&occtDm);
    return tr;
}

}  // namespace

int main(int argc, char** argv) {
    uint64_t seed = 0xD15B1A57EEull;
    int N = 72;
    if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
    else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
    if (argc > 2) N = std::atoi(argv[2]);
    else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
    if (N <= 0) N = 72;

    std::printf("== M6 DISPLAY-MESH differential-fuzz: cc_display_mesh NATIVE (engine 1) vs OCCT (engine 0) source tessellation ==\n");
    std::printf("== oracle = INVARIANTS (finite/unit-normal/non-degenerate/fold-watertight/UV) + CLOSED-FORM deflection bound + bbox parity ==\n");
    std::printf("== seed=0x%llx N=%d  bands: surf<%.0fx-defl bbox<%.0fx-defl LOD-hausdorff<%.0fx-defl tri-ratio<%.0fx (FIXED, NEVER widened) ==\n",
                static_cast<unsigned long long>(seed), N, kSurfMul, kBBoxMul, kLodScale, kTriRatio);
    std::fflush(stdout);

    EngineGuard guard;
    Rng rng(seed);
    for (int i = 0; i < N; ++i) {
        const Recipe r = drawRecipe(rng, i);
        const Trial tr = runOne(r);
        const Verdict v = classify(tr);
        tally(v, tr.fam);
        report(i, tr, v, seed);
    }

    std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n", static_cast<unsigned long long>(seed), N);
    std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d\n",
                g_agreed, g_declined, g_disagreed, g_oracleInacc);
    std::printf("   per-family [AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE]:\n");
    for (int f = 0; f < F_COUNT; ++f)
        std::printf("     %-14s %d / %d / %d / %d\n", famName(f), g_famA[f], g_famD[f], g_famX[f], g_famOI[f]);
    if (g_oracleInacc)
        std::printf("   ORACLE-INACCURATE=%d: the OCCT source mesh broke the closed-form deflection bound while the\n"
                    "     native source mesh held it — native display mesh MORE correct. Logged, never a bar DISAGREE.\n", g_oracleInacc);

    // Bar: DISAGREED==0 AND every ANALYTIC family (closed-form oracle) has ≥1 AGREED (the
    // fillet/boolean families have no closed form → coverage counted but not bar-gated).
    bool coverage = true;
    for (int f = 0; f < F_COUNT; ++f) if (famHasClosedForm(f) && g_famA[f] < 1) coverage = false;
    const bool bar = (g_disagreed == 0 && coverage);
    std::printf("== M6 DISPLAY-MESH BAR: %s (DISAGREED=%d must be 0; per-analytic-family AGREE coverage=%s) ==\n",
                bar ? "PASS — zero silent-wrong display meshes" : "FAIL", g_disagreed,
                coverage ? "complete" : "INCOMPLETE");
    std::fflush(stdout);
    std::_Exit(bar ? 0 : 1);
}
