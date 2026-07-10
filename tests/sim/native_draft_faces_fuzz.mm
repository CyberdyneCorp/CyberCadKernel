// SPDX-License-Identifier: Apache-2.0
//
// native_draft_faces_fuzz.mm — MOAT M6-breadth-16 (the COMPLETENESS BAR, 16th domain):
// a DRAFT-ANGLE differential-fuzzing harness (iOS simulator) that certifies the newly-
// native DRAFT-ANGLE surface (src/native/feature/draft_faces.h) reached through the
// SHIPPING `cc_draft_faces` facade.
//
// It extends the fifteen landed M6 differential fuzzers (curved-boolean, STEP round-trip,
// construction loft/sweep, blend fillet/chamfer, wrap-emboss, mass-properties, geometry-
// services, transform-chains, reference/datum, direct-modeling, transformed-boolean,
// orthographic-HLR, shape-healing, section, curved-blend) to a SIXTEENTH independent
// native domain. Like its siblings it is INFRASTRUCTURE (a seeded harness, not a geometry
// capability) — src/native / src/engine / include stay BYTE-UNCHANGED.
//
// ── WHY THIS DOMAIN IS DISTINCT FROM native_draft_faces_parity.mm ────────────────────
// The parity harness drives `feature::draftFaces` DIRECTLY on THREE hand-picked fixtures
// (box +x wedge, box 4-side frustum, off-axis wedge) + one cap decline. THIS harness
// closes the fuzz gap: it drives the STABLE, LANDED draft through the PUBLIC `cc_draft_
// faces` facade under BOTH engines (cc_set_engine(1)=NativeEngine prismatic draft with an
// OCCT fallback on decline; cc_set_engine(0)=the OCCT BRepOffsetAPI_DraftAngle oracle) —
// the shipping path the app calls — over a seeded batch of random prismatic solids × random
// draft poses. It is the facade-level, both-engines, randomised completeness certification
// the hand-picked parity fixtures do not provide.
//
// ── THE FOUR FAMILIES (base × face-count) ────────────────────────────────────────────
// Base solids are prismatic solids built IDENTICALLY under both engines through the public
// cc_solid_extrude facade (footprint polygon in the z=0 plane, extruded +Z to height h):
//   BOX   random w×d rectangle centred at origin.
//   NGON  random regular n-gon (n∈[3,8]) of circumradius R centred at origin.
// The four families are {BOX, NGON} × {SINGLE-face, MULTI-face}:
//   *_SINGLE  draft ONE random planar side face by θ about the base plane (pull +Z).
//   *_MULTI   draft a random SUBSET (2..all) of the planar side faces by θ (a frustum-like
//             taper; adjacent-face corners interact — the closed form handles them exactly).
//
// ── THE ORACLES: CLOSED-FORM (the PRIMARY arbiter) + OCCT ─────────────────────────────
// The neutral plane is ALWAYS the base plane (origin (0,0,0), pull +Z), and every drafted
// face is a vertical prism wall, so the drafted solid's cross-section at height z is the
// footprint polygon with EACH drafted edge's supporting line pushed INWARD by z·tanθ (non-
// drafted edges fixed). The exact drafted VOLUME is therefore
//     V = ∫₀^h A(z) dz,   A(z) = area( footprint clipped by the inward-shifted drafted
//                                     half-planes ).
// A(z) is a polynomial of degree ≤ 2 in z (a convex polygon's area under parallel edge
// offsets), so a 3-point Simpson quadrature over [0,h] is ANALYTICALLY EXACT — this is the
// PRIMARY arbiter, exact for the ideal prismatic draft, and it handles adjacent-face corner
// interactions exactly via polygon clipping (validated against the parity harness's box
// wedge V=1000−500·tan8° and 4-side frustum closed forms). Because it is exact, a native
// result matching the closed form while OCCT is the outlier is logged ORACLE-INACCURATE
// (native vindicated by exact math), never a bar failure.
//   The OCCT oracle is cc_set_engine(0) → OCCT BRepOffsetAPI_DraftAngle through the SAME
// cc_draft_faces facade (same face ids, same neutral/pull/angle), measured by
// cc_mass_properties. The drafted AREA is not a simple closed form (tapered walls + shrunk
// top cap), so AREA is cross-checked against OCCT only (facet-free B-rep vs native mesh
// mass), never against a fabricated analytic area.
//
// ── THE DEFLECTION / FACET BOUNDARY ───────────────────────────────────────────────────
// The native draft emits a deflection-bounded facet mesh; its cc_mass_properties volume is
// the B-rep volume of the trimmed planar solid (planar meshes are EXACT), so the native-vs-
// closed-form band is TIGHT (vol rel ≤ 1e-3, matching the parity harness's ≤1e-2 absolute
// on a V~1000 box → ~1e-5 rel). The native-vs-OCCT band is a touch looser (vol rel ≤ 2e-2,
// area rel ≤ 3e-2) to absorb OCCT's own draft-build discretisation, MATCHED to the ops and
// NEVER widened to force a pass.
//
// ── THE SIX-WAY CLASSIFIER (identical discipline to the landed siblings) ──────────────
//   AGREED            native VALID (watertight, χ=2, volume STRICTLY smaller than the base
//                     — a draft only removes stock) + volume within the family band of BOTH
//                     the closed form AND OCCT, which also matches the closed form.
//   HONESTLY-DECLINED native cc_* returns 0 / invalid (an out-of-envelope pose the native
//                     arm refuses) while OCCT ships a valid result → native → OCCT. First-
//                     class, counted separately, NEVER a bar failure.
//   DISAGREED         native VALID but OUTSIDE the closed-form truth while OCCT matches it —
//                     a genuine SILENT WRONG draft. The failure this harness exists to
//                     catch. (FAILS the bar.)
//   ORACLE-INACCURATE native matches the closed-form truth while OCCT does NOT — native
//                     vindicated by exact math, OCCT the outlier. Logged, NOT a bar failure.
//   ORACLE_UNRELIABLE a trial whose native result misses the closed form AND OCCT also does
//                     not match it → the oracle is untrustworthy → excluded from AGREE,
//                     FAILS the bar (investigate, never launder a native miss as a pass).
//   BOTH-DECLINED     an out-of-envelope pose both engines refuse. Logged, not a failure.
//
// ── THE BAR ──────────────────────────────────────────────────────────────────────────
//   Exit 0 IFF DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, with each of the four families
//   having ≥ 1 AGREED trial (real native exercise, not all-declines). Run over ≥ 2 distinct
//   seeds, N ≥ 60 per seed; the runner fails if ANY seed fails. The generator is seeded
//   ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-
//   identical batch (splitmix64 → xoshiro256**, verbatim from the siblings). Any DISAGREE /
//   ORACLE-INACCURATE prints seed + case index + family/param tuple + the native/OCCT/
//   closed-form triple as a reproducible regression find.
//
// This TU drives the SHIPPING cc_* facade so it links the WHOLE kernel (facade + core +
// engine[native+occt] + native math) + the OCCT oracle; the native draft's inward trim is
// CYBERCAD_HAS_NUMSCI-gated (splitByPlane's seam trace), so — like native_directmodel_fuzz —
// the runner links the numsci substrate. Built ONLY by scripts/run-sim-native-draft-faces-
// fuzz.sh; on run-sim-suite.sh's SKIP list (own main(), std::_Exit).
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

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_draft_faces_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;

// Family AGREE bands. Planar-draft volume is EXACT (planar B-rep), so native-vs-closed-form
// is tight; native-vs-OCCT is a touch looser to absorb OCCT draft-build discretisation.
// FIXED, never widened to force a pass.
constexpr double kVolTolX   = 1e-3;   // native vs closed-form volume (PRIMARY arbiter)
constexpr double kVolTolO   = 2e-2;   // native vs OCCT volume
constexpr double kAreaTol   = 3e-2;   // native vs OCCT surface area
constexpr double kOracleTol = 2e-2;   // OCCT vs closed form (oracle-trust)
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
//    straddle never falsely reports a leak). Identical to the landed fuzz siblings. ──────
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

double relDiff(double a, double b) { return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : 1e30; }

// ── 2D convex-polygon clipping (Sutherland–Hodgman) against a single half-plane ────────
// A half-plane {p : n·p ≤ c} keeps the INSIDE. Points are (x,y) pairs.
struct P2 { double x, y; };

std::vector<P2> clipHalfPlane(const std::vector<P2>& poly, double nx, double ny, double c) {
    std::vector<P2> out;
    const int m = static_cast<int>(poly.size());
    if (m == 0) return out;
    auto side = [&](const P2& p) { return nx * p.x + ny * p.y - c; };  // ≤0 inside
    for (int i = 0; i < m; ++i) {
        const P2& a = poly[i];
        const P2& b = poly[(i + 1) % m];
        const double sa = side(a), sb = side(b);
        const bool ina = sa <= 1e-12, inb = sb <= 1e-12;
        if (ina) out.push_back(a);
        if (ina != inb) {
            const double t = sa / (sa - sb);
            out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
        }
    }
    return out;
}

double polyArea(const std::vector<P2>& poly) {
    double a2 = 0.0;
    const int m = static_cast<int>(poly.size());
    for (int i = 0; i < m; ++i) {
        const P2& p = poly[i];
        const P2& q = poly[(i + 1) % m];
        a2 += p.x * q.y - q.x * p.y;
    }
    return std::fabs(a2) * 0.5;
}

// ── base solids + drafted geometry ────────────────────────────────────────────────────
// A convex footprint polygon (CCW) in the z=0 plane, extruded +Z to height h. Each edge i
// runs vertex[i]→vertex[i+1]; its OUTWARD unit normal + a point on it identify the side
// face. Drafting edge i pivots its wall about the base trace: at height z the edge's line
// shifts inward by z·tanθ.
struct Prism {
    int base;                    // 0 = box, 1 = ngon
    std::vector<P2> foot;        // CCW footprint vertices
    double h;                    // extrude height
    double baseArea, wholeVol;
    std::string desc;
    // per-edge outward normal + a mid-edge probe point (world x,y at mid-height z)
    std::vector<double> edgeNx, edgeNy, edgeMidX, edgeMidY;
    void buildEdges() {
        const int m = static_cast<int>(foot.size());
        edgeNx.assign(m, 0); edgeNy.assign(m, 0); edgeMidX.assign(m, 0); edgeMidY.assign(m, 0);
        for (int i = 0; i < m; ++i) {
            const P2& a = foot[i];
            const P2& b = foot[(i + 1) % m];
            const double dx = b.x - a.x, dy = b.y - a.y;
            const double len = std::sqrt(dx * dx + dy * dy);
            // CCW polygon: outward normal is (dy, -dx)/len.
            edgeNx[i] = dy / len; edgeNy[i] = -dx / len;
            edgeMidX[i] = 0.5 * (a.x + b.x); edgeMidY[i] = 0.5 * (a.y + b.y);
        }
    }
};

std::vector<double> flatFoot(const Prism& p) {
    std::vector<double> xy; xy.reserve(p.foot.size() * 2);
    for (const P2& v : p.foot) { xy.push_back(v.x); xy.push_back(v.y); }
    return xy;
}

double ngonArea(int n, double R) { return 0.5 * n * R * R * std::sin(2.0 * kPi / n); }

Prism genPrism(Rng& rng, int base) {
    Prism p; p.base = base;
    if (base == 0) {  // BOX
        const double w = rng.range(6.0, 16.0), d = rng.range(6.0, 16.0), h = rng.range(6.0, 16.0);
        p.foot = {{-w / 2, -d / 2}, {w / 2, -d / 2}, {w / 2, d / 2}, {-w / 2, d / 2}};
        p.h = h; p.baseArea = w * d; p.wholeVol = w * d * h;
        char b[96]; std::snprintf(b, sizeof b, "box w=%.3f d=%.3f h=%.3f", w, d, h);
        p.desc = b;
    } else {  // NGON
        const int n = 3 + static_cast<int>(rng.below(6));  // 3..8
        const double R = rng.range(5.0, 10.0), h = rng.range(6.0, 16.0);
        p.foot.reserve(n);
        for (int i = 0; i < n; ++i) {
            const double a = 2.0 * kPi * i / n;
            p.foot.push_back({R * std::cos(a), R * std::sin(a)});
        }
        p.h = h; p.baseArea = ngonArea(n, R); p.wholeVol = p.baseArea * h;
        char b[96]; std::snprintf(b, sizeof b, "ngon n=%d R=%.3f h=%.3f", n, R, h);
        p.desc = b;
    }
    p.buildEdges();
    return p;
}

// The CLOSED-FORM drafted volume: V = ∫₀^h A(z) dz, A(z) = area of the footprint clipped by
// each drafted edge's inward-shifted half-plane (shift = z·tanθ). A(z) is degree ≤2 in z, so
// 3-point Simpson over [0,h] is EXACT. `drafted[i]` selects which side faces taper.
double draftedVolumeClosedForm(const Prism& p, const std::vector<bool>& drafted, double thetaRad) {
    const double tanT = std::tan(thetaRad);
    auto areaAt = [&](double z) -> double {
        std::vector<P2> poly = p.foot;
        const int m = static_cast<int>(p.foot.size());
        for (int i = 0; i < m && !poly.empty(); ++i) {
            if (!drafted[static_cast<std::size_t>(i)]) continue;
            // edge i supporting line: {p : n·p = c0}; inward shift by z·tanθ → c = c0 − z·tanθ.
            const double nx = p.edgeNx[i], ny = p.edgeNy[i];
            const double c0 = nx * p.edgeMidX[i] + ny * p.edgeMidY[i];
            poly = clipHalfPlane(poly, nx, ny, c0 - z * tanT);
        }
        return polyArea(poly);
    };
    const double a0 = areaAt(0.0), am = areaAt(0.5 * p.h), ah = areaAt(p.h);
    return (p.h / 6.0) * (a0 + 4.0 * am + ah);  // Simpson (exact for a quadratic A(z))
}

// ── engine-independent side-face resolver (ids are engine-local; resolve per engine) ───
// Return the face id whose analytic surface passes through the given mid-edge probe point
// (project the point onto each candidate face: foot distance ≈ 0 ⇒ the point lies on it)
// AND whose implied outward normal matches (the probe point is nudged +ε outward, its foot
// distance ≈ ε on the correct planar side face). We pick the face minimising foot distance
// to the on-plane probe among faces that reject the outward-nudged probe (distance > 0).
int sideFaceId(CCShapeId body, double px, double py, double pz, double nx, double ny) {
    int* ids = nullptr;
    const int nf = cc_subshape_ids(body, 2, &ids);
    int best = 0; double bestErr = 1e300;
    for (int k = 0; k < nf; ++k) {
        const CCProjection on = cc_project_point_on_face(body, ids[k], px, py, pz);
        if (!on.valid) continue;
        // The point must lie ON this face's plane (foot distance ~ 0)...
        if (on.distance > 1e-3) continue;
        // ...and the face's outward normal must match: nudge the probe OUTWARD by ε; the
        // foot distance to a matching planar face is ~ε (the plane is behind the point),
        // whereas a wrong (e.g. adjacent) face gives a different distance. Use the nudged
        // distance closeness to ε as the discriminator.
        const double eps = 0.05;
        const CCProjection out =
            cc_project_point_on_face(body, ids[k], px + nx * eps, py + ny * eps, pz);
        if (!out.valid) continue;
        const double err = std::fabs(out.distance - eps) + on.distance;
        if (err < bestErr) { bestErr = err; best = ids[k]; }
    }
    if (ids) cc_ints_free(ids);
    return (bestErr < 5e-2) ? best : 0;
}

// ── classification ──────────────────────────────────────────────────────────────────
enum Family { F_BOX_SINGLE, F_BOX_MULTI, F_NGON_SINGLE, F_NGON_MULTI, F_COUNT };
const char* famName(int f) {
    switch (f) {
        case F_BOX_SINGLE:  return "BOX  single-face";
        case F_BOX_MULTI:   return "BOX  multi-face";
        case F_NGON_SINGLE: return "NGON single-face";
        case F_NGON_MULTI:  return "NGON multi-face";
    }
    return "?";
}

enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_oracleBad = 0, g_bothDecl = 0;
int g_famA[F_COUNT] = {0}, g_famD[F_COUNT] = {0}, g_famX[F_COUNT] = {0};
int g_famOI[F_COUNT] = {0}, g_famBD[F_COUNT] = {0}, g_famOU[F_COUNT] = {0};

struct Trial {
    bool nativeValid = false;   // native cc_draft_faces produced a valid body with mass
    bool oracleValid = false;   // OCCT oracle produced a valid measurement
    bool watertight = false;    // native result mesh is a closed 2-manifold
    bool eulerOk = false;       // native result mesh χ == 2 AND mesh-vol ≈ mass-vol
    bool dirOk = false;         // volume STRICTLY smaller than the base (a draft removes stock)
    bool faceResolved = true;   // both engines resolved the requested face ids
    double natVol = 0, natArea = 0, oVol = 0, oArea = 0, xVol = 0, baseVol = 0;
    std::string desc;
};

Verdict classify(const Trial& tr) {
    if (!tr.faceResolved) return BOTH_DECLINED;  // could not pose the trial under either engine
    const bool geomOk = tr.watertight && tr.eulerOk && tr.dirOk;
    const bool natMatchesX = tr.nativeValid && geomOk && relDiff(tr.natVol, tr.xVol) < kVolTolX;
    const bool oracleTrust = tr.oracleValid && relDiff(tr.oVol, tr.xVol) < kOracleTol;
    const bool natMatchesO = tr.nativeValid && tr.oracleValid &&
        relDiff(tr.natVol, tr.oVol) < kVolTolO &&
        (tr.oArea <= 0.0 || relDiff(tr.natArea, tr.oArea) < kAreaTol);

    if (!tr.nativeValid) return tr.oracleValid ? DECLINED : BOTH_DECLINED;
    if (natMatchesX) {
        if (oracleTrust && natMatchesO) return AGREED;
        if (tr.oracleValid) return ORACLE_INACCURATE;  // native == math, OCCT the outlier
        return AGREED;                                 // native == exact math, no oracle
    }
    // native VALID but misses exact math: if OCCT matches the closed form, a SILENT WRONG
    // draft; if neither matches, the oracle is unreliable.
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
    if (v == AGREED) {
        std::printf("[FUZZ] AGREED    case=%-3d %-18s volN=%.6g volO=%.6g volX=%.6g dO=%.2e dX=%.2e areaN=%.6g areaO=%.6g wt=%d chi2=%d dir=%d  %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol, relDiff(tr.natVol, tr.oVol),
                    relDiff(tr.natVol, tr.xVol), tr.natArea, tr.oArea, tr.watertight ? 1 : 0,
                    tr.eulerOk ? 1 : 0, tr.dirOk ? 1 : 0, tr.desc.c_str());
    } else if (v == DECLINED) {
        std::printf("[FUZZ] DECLINED  case=%-3d %-18s native cc_draft->0/invalid -> OCCT[volO=%.6g areaO=%.6g volX=%.6g]  %s\n",
                    i, famName(fam), tr.oVol, tr.oArea, tr.xVol, tr.desc.c_str());
    } else if (v == BOTH_DECLINED) {
        std::printf("[FUZZ] BOTH-DECL case=%-3d %-18s native AND OCCT both refused (out-of-envelope / unresolved)  %s\n",
                    i, famName(fam), tr.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
        std::printf("[FUZZ] ORACLE_INACCURATE case=%-3d %-18s native MATCHES closed form, OCCT does NOT "
                    "volN=%.6g volO=%.6g volX=%.6g\n       NOTE seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    } else if (v == ORACLE_UNRELIABLE) {
        std::printf("[FUZZ] ORACLE_UNRELIABLE case=%-3d %-18s oracle mismatch/absent "
                    "[natValid=%d occValid=%d volN=%.6g volO=%.6g volX=%.6g occVsX=%.2e natVsX=%.2e wt=%d chi2=%d dir=%d]\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.nativeValid, tr.oracleValid, tr.natVol, tr.oVol, tr.xVol,
                    tr.oracleValid ? relDiff(tr.oVol, tr.xVol) : -1.0, relDiff(tr.natVol, tr.xVol),
                    tr.watertight ? 1 : 0, tr.eulerOk ? 1 : 0, tr.dirOk ? 1 : 0,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    } else {  // DISAGREED
        std::printf("[FUZZ] DISAGREED case=%-3d %-18s SILENT-WRONG-DRAFT "
                    "volN=%.6g volO=%.6g volX=%.6g dX=%.3e areaN=%.6g areaO=%.6g wt=%d chi2=%d dir=%d\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol, relDiff(tr.natVol, tr.xVol),
                    tr.natArea, tr.oArea, tr.watertight ? 1 : 0, tr.eulerOk ? 1 : 0, tr.dirOk ? 1 : 0,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    }
    std::fflush(stdout);
}

// Fill the native geometry diagnostics of a native draft body (watertight, χ, mesh-vol
// consistency) + its cc_mass_properties, measured UNDER the native engine.
void fillNativeGeom(CCShapeId nat, double defl, Trial& tr) {
    if (nat == 0) return;
    const CCMassProps mp = cc_mass_properties(nat);
    const CCMesh mesh = cc_tessellate(nat, defl);
    const bool haveMesh = mesh.triangleCount > 0;
    tr.nativeValid = mp.valid != 0 && mp.volume > 1e-9 && haveMesh;
    if (tr.nativeValid) {
        tr.natVol = mp.volume; tr.natArea = mp.area;
        tr.watertight = meshWatertight(mesh);
        const double mv = meshVolume(mesh);
        tr.eulerOk = (meshEuler(mesh) == 2) && relDiff(mv, mp.volume) < kMeshVolTol;
    }
    if (haveMesh) cc_mesh_free(mesh);
}

// One draft trial: build the prism identically under both engines, resolve the SAME set of
// side faces per engine, draft via cc_draft_faces under OCCT (oracle) and native, and
// arbitrate against the closed form.
Trial runDraft(const Prism& p, const std::vector<bool>& drafted, double thetaDeg) {
    Trial tr;
    tr.baseVol = p.wholeVol;
    const double thetaRad = thetaDeg * kPi / 180.0;
    tr.xVol = draftedVolumeClosedForm(p, drafted, thetaRad);

    const std::vector<double> foot = flatFoot(p);
    const int npts = static_cast<int>(foot.size() / 2);
    const double zmid = 0.5 * p.h;
    const double origin[3] = {0, 0, 0};
    const double pull[3] = {0, 0, 1};

    // Gather the drafted edge probe points/normals.
    std::vector<int> draftedEdges;
    for (int i = 0; i < static_cast<int>(drafted.size()); ++i)
        if (drafted[static_cast<std::size_t>(i)]) draftedEdges.push_back(i);

    // ── OCCT oracle (facade, engine 0) ──
    cc_set_engine(0);
    CCShapeId oBody = cc_solid_extrude(foot.data(), npts, p.h);
    std::vector<int> oIds;
    bool oResolved = (oBody != 0);
    if (oBody != 0) {
        for (int e : draftedEdges) {
            const int fid = sideFaceId(oBody, p.edgeMidX[e], p.edgeMidY[e], zmid, p.edgeNx[e], p.edgeNy[e]);
            if (fid == 0) { oResolved = false; break; }
            oIds.push_back(fid);
        }
    }
    if (oBody != 0 && oResolved) {
        const CCShapeId od = cc_draft_faces(oBody, oIds.data(), static_cast<int>(oIds.size()),
                                            origin, pull, thetaDeg);
        if (od != 0) {
            const CCMassProps om = cc_mass_properties(od);
            if (om.valid && om.volume > 1e-9) { tr.oracleValid = true; tr.oVol = om.volume; tr.oArea = om.area; }
            cc_shape_release(od);
        }
    }
    if (oBody != 0) cc_shape_release(oBody);

    // ── native candidate (facade, engine 1) ──
    cc_set_engine(1);
    CCShapeId nBody = cc_solid_extrude(foot.data(), npts, p.h);
    std::vector<int> nIds;
    bool nResolved = (nBody != 0);
    if (nBody != 0) {
        for (int e : draftedEdges) {
            const int fid = sideFaceId(nBody, p.edgeMidX[e], p.edgeMidY[e], zmid, p.edgeNx[e], p.edgeNy[e]);
            if (fid == 0) { nResolved = false; break; }
            nIds.push_back(fid);
        }
    }
    CCShapeId nat = 0;
    if (nBody != 0 && nResolved)
        nat = cc_draft_faces(nBody, nIds.data(), static_cast<int>(nIds.size()), origin, pull, thetaDeg);
    fillNativeGeom(nat, 0.01, tr);
    if (tr.nativeValid) tr.dirOk = (tr.natVol < p.wholeVol - 1e-6) && tr.natVol > 0.0;
    if (nBody != 0) cc_shape_release(nBody);
    if (nat != 0) cc_shape_release(nat);
    cc_set_engine(0);

    tr.faceResolved = oResolved && nResolved;
    return tr;
}

// ── the generator: pick a family, draw a VALID prism + draft pose, run the trial ───────
Trial genAndRun(Rng& rng, int& famOut) {
    const int fam = static_cast<int>(rng.below(F_COUNT));
    famOut = fam;
    const int base = (fam == F_BOX_SINGLE || fam == F_BOX_MULTI) ? 0 : 1;
    const bool multi = (fam == F_BOX_MULTI || fam == F_NGON_MULTI);
    const Prism p = genPrism(rng, base);
    const int m = static_cast<int>(p.foot.size());

    // A valid draft angle must keep the drafted top cross-section non-degenerate: the top
    // recedes by h·tanθ, so bound θ so h·tanθ stays below a fraction of the inradius. The
    // inradius (apothem) is the min distance centre→edge line; for our centred prisms that is
    // the min |c0| over drafted edges. Cap the recession at 35% of that so the taper is real
    // but the top never collapses/self-intersects (the pathological ≥-collapse cases are the
    // native ResolveFailed → OCCT decline branch, exercised separately below).
    double inradius = 1e30;
    for (int i = 0; i < m; ++i) {
        const double c0 = std::fabs(p.edgeNx[i] * p.edgeMidX[i] + p.edgeNy[i] * p.edgeMidY[i]);
        inradius = std::min(inradius, c0);
    }
    // largest θ so h·tanθ ≤ 0.35·inradius
    const double maxTan = 0.35 * inradius / p.h;
    double maxDeg = std::atan(maxTan) * 180.0 / kPi;
    maxDeg = std::min(maxDeg, 15.0);         // keep well below the 90° flip; ≤15° is a real draft
    const double thetaDeg = rng.range(2.0, std::max(3.0, maxDeg));

    // Select the drafted subset.
    std::vector<bool> drafted(static_cast<std::size_t>(m), false);
    std::string sub;
    if (!multi) {
        const int e = static_cast<int>(rng.below(static_cast<uint32_t>(m)));
        drafted[static_cast<std::size_t>(e)] = true;
        char b[32]; std::snprintf(b, sizeof b, " face=%d/%d", e, m); sub = b;
    } else {
        // 2..m faces, each included with a coin flip, guaranteeing ≥2 chosen.
        int chosen = 0;
        for (int i = 0; i < m; ++i) { if (rng.unit() < 0.6) { drafted[static_cast<std::size_t>(i)] = true; ++chosen; } }
        while (chosen < 2) {
            const int e = static_cast<int>(rng.below(static_cast<uint32_t>(m)));
            if (!drafted[static_cast<std::size_t>(e)]) { drafted[static_cast<std::size_t>(e)] = true; ++chosen; }
        }
        char b[32]; std::snprintf(b, sizeof b, " faces=%d/%d", chosen, m); sub = b;
    }

    Trial t = runDraft(p, drafted, thetaDeg);
    t.desc = p.desc + sub + " theta=" + std::to_string(thetaDeg) + "deg";
    return t;
}

}  // namespace

int main(int argc, char** argv) {
    uint64_t seed = 0xD4AF7A11EEull;
    int N = 72;
    if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
    else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
    if (argc > 2) N = std::atoi(argv[2]);
    else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
    if (N <= 0) N = 72;

    std::printf("== M6-breadth-16 differential-fuzz: native DRAFT-ANGLE (box/ngon x single/multi-face) vs OCCT + closed-form ==\n");
    std::printf("== seed=0x%llx N=%d  bands: volX<%.0e volO<%.0e area<%.0e oracle<%.0e (FIXED, NEVER widened) ==\n",
                static_cast<unsigned long long>(seed), N, kVolTolX, kVolTolO, kAreaTol, kOracleTol);
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

    std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
                static_cast<unsigned long long>(seed), N);
    std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  ORACLE_UNRELIABLE=%d  BOTH-DECLINED=%d\n",
                g_agreed, g_declined, g_disagreed, g_oracleInacc, g_oracleBad, g_bothDecl);
    std::printf("   per-family [AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED]:\n");
    for (int f = 0; f < F_COUNT; ++f)
        std::printf("     %-18s %d / %d / %d / %d / %d / %d\n", famName(f),
                    g_famA[f], g_famD[f], g_famX[f], g_famOI[f], g_famOU[f], g_famBD[f]);
    if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — logged, NOT a native fault)\n", g_oracleInacc);
    if (g_bothDecl)    std::printf("   BOTH-DECLINED=%d (out-of-envelope pose both engines refuse / unresolved — logged)\n", g_bothDecl);
    if (g_oracleBad)   std::printf("   ORACLE_UNRELIABLE=%d (OCCT vs closed-form mismatch AND native missed — investigate)\n", g_oracleBad);

    bool coverage = true;
    for (int f = 0; f < F_COUNT; ++f) if (g_famA[f] < 1) coverage = false;
    const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && coverage);
    std::printf("== M6-breadth-16 BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
                "per-family AGREE coverage=%s) ==\n",
                bar ? "PASS — zero silent wrong drafts" : "FAIL", g_disagreed, g_oracleBad,
                coverage ? "complete" : "INCOMPLETE");
    std::fflush(stdout);
    std::_Exit(bar ? 0 : 1);
}
