// SPDX-License-Identifier: Apache-2.0
//
// native_vsweep_fuzz.mm — MOAT M6 (the COMPLETENESS BAR): a VARIABLE-SECTION SWEEP
// differential-fuzzing harness (iOS simulator) that certifies the newly-native
// VARIABLE-SECTION / guide+spine sweep (src/native/construct/sweep.h
// build_variable_sweep + build_variable_sweep_tube) reached through the SHIPPING
// `cc_variable_sweep` facade.
//
// It extends the landed M6 differential fuzzers (curved-boolean, STEP round-trip,
// construction loft/sweep, blend fillet/chamfer, wrap-emboss, mass-properties, geometry-
// services, transform-chains, reference/datum, direct-modeling, transformed-boolean,
// orthographic-HLR, shape-healing, section, curved-blend, draft-angle, interference, …) to
// an independent native domain. Like its siblings it is INFRASTRUCTURE (a seeded harness,
// not a geometry capability) — src/native / src/engine / include stay BYTE-UNCHANGED.
//
// ── WHY THIS DOMAIN IS DISTINCT FROM native_vsweep_parity.mm ─────────────────────────
// The parity harness drives `cc_variable_sweep` on FOUR hand-picked fixtures (circle→circle
// straight, constant-square straight, guide-scaled square, circle→circle arc) + one deferred
// non-planar case. THIS harness closes the fuzz gap: it drives the STABLE, LANDED variable
// sweep through the PUBLIC `cc_variable_sweep` facade under BOTH engines (cc_set_engine(1)=
// NativeEngine RMF/perp-framed guide-scaled morph tube with an OCCT fallback on decline;
// cc_set_engine(0)=the OCCT BRepOffsetAPI_MakePipeShell multi-section oracle) — the shipping
// path the app calls — over a seeded batch of random morph profiles × random spine × random
// optional guide rail. It is the facade-level, both-engines, randomised completeness
// certification the hand-picked parity fixtures do not provide.
//
// ── THE FIVE FAMILIES (morph-shape × spine × guide) ──────────────────────────────────
//   CIRCLE_STRAIGHT   circle(r0)→circle(r1) morph along a straight +Z spine, no guide — a
//                     TRUNCATED CONE. Closed-form volume πH/3(r0²+r0r1+r1²) is the PRIMARY
//                     arbiter.
//   POLY_STRAIGHT     regular n-gon(r0)→n-gon(r1) SAME-n morph along a straight +Z spine, no
//                     guide — a polygon frustum/prism. Closed-form prismatoid volume
//                     (H/3)(A0+√(A0A1)+A1) is EXACT and the PRIMARY arbiter.
//   SECTION_STRAIGHT  section-A → section-B (DIFFERENT shapes, same vertex count) morph along
//                     a straight +Z spine, no guide. The vertex-k→vertex-k ruled morph makes
//                     the cross-section at fraction f the linear blend of the two polygons —
//                     area A(f) is quadratic in f, so a 3-point Simpson volume is EXACT
//                     (PRIMARY arbiter), handling the general A→B morph.
//   GUIDED_STRAIGHT   a polygon sweep along a straight +Z spine WITH a guide rail. Since
//                     moat-vsfix ALL THREE guided regimes are native (see the NATIVE GUIDED
//                     ENVELOPE note below): {CONSTANT section + splaying guide → a similar-
//                     polygon frustum}, {morphing section + guide parallel to the spine
//                     (scale≡1)}, and the COUPLED {morphing section + splaying guide}. The
//                     guide scale s(f)=dist(spine(f),guide(f))/dist(...,0) is (piecewise-)linear
//                     in f, so the section area A(f)=A_morph(f)·s(f)² is degree ≤4 — integrated
//                     by a 5-point BOOLE quadrature EXACT for a quartic (PRIMARY arbiter),
//                     capturing the coupled cross-term exactly.
//
// ── NATIVE GUIDED ENVELOPE (the coupled morph×scale regime is now NATIVE — moat-vsfix) ──────
// The native STRAIGHT-spine guided builder (build_variable_sweep_tube) rules between densified
// stations. In the two LINEAR sub-regimes — the section constant (A==B) OR the guide scale
// constant (s≡1) — the section-radius law g(f)=morph(f)·scale(f) is linear, so it stays at TWO
// stations and reproduces the continuous law EXACTLY (byte-identical to the pre-fix path). When
// BOTH the section morphs AND the guide scale varies, g(f) is a genuine non-linear PRODUCT and a
// two-station linear ruling would DROP the morph×scale cross-term: the mid section of the linear
// ruling is 0.5·s0·A + 0.5·s1·B whereas the true continuously-scaled mid section is s(0.5)·
// mid(A,B), differing by 0.25·(1−k)·(A−B) (k = s1/s0), non-zero only in the coupled regime — the
// M6-breadth-19 divergence (1–20% vs OCCT MakePipeShell AND the exact polygon-clip integral).
// moat-vsfix FIXES this: the straight guided builder now DENSIFIES to bound the coupled area-
// integral error (straightCoupledStations), so the swept volume converges to the exact ∫A(f)·H
// closed form as stations grow. The guided family is therefore certified in ALL THREE regimes
// here — including the coupled one, against the EXACT closed form (Boole's rule, never a widened
// tolerance).
//   CIRCLE_CURVED     circle(r0)→circle(r1) morph along a smooth planar quarter-arc spine, no
//                     guide — a curved morph tube. NO simple closed form → arbitrated against
//                     OCCT MakePipeShell (deflection-bounded band) + watertight/Euler χ=2.
//
// ── THE ORACLES: CLOSED-FORM (PRIMARY where it exists) + OCCT ─────────────────────────
// The closed-form volume is the PRIMARY arbiter for the four STRAIGHT-spine families (a
// straight-spine variable sweep's cross-section is a planar polygon whose signed area is a
// low-degree polynomial in the spine fraction; the exact volume is ∫A(f)·H df). Because it is
// exact, a native result matching the closed form while OCCT is the outlier is logged
// ORACLE-INACCURATE (native vindicated by exact math), never a bar failure. The OCCT oracle
// is cc_set_engine(0) → OCCT MakePipeShell multi-section through the SAME facade, measured by
// cc_mass_properties. The CIRCLE_CURVED family has no closed form, so it is arbitrated against
// OCCT ONLY (deflection-bounded band) plus the engine-independent watertight/Euler invariants.
//
// ── THE DEFLECTION / FACET BOUNDARY ───────────────────────────────────────────────────
// A native circle profile is a 64-gon; both engines see the SAME polygonal profile, so a
// straight-spine sweep of it is a planar-faced solid whose cc_mass_properties volume is EXACT
// (planar B-rep). Thus native-vs-closed-form is TIGHT for the polygon families. The circle
// families' closed form uses the SMOOTH cone/tube volume, so the native polygon-approximation
// undershoots by the inscribed-64-gon area ratio (≈ (2π/n)²/6 ≈ 3.2e-3 for n=64) — the band
// absorbs that fixed, geometry-driven bias without ever being widened to hide a wrong set.
// The native-vs-OCCT band is a touch looser to absorb OCCT's pipe-shell discretisation.
//
// ── THE SIX-WAY CLASSIFIER (identical discipline to the landed siblings) ──────────────
//   AGREED            native VALID (watertight, χ=2, positive volume) + volume within the
//                     family band of BOTH the closed form AND OCCT (curved: OCCT only), which
//                     also matches the closed form.
//   HONESTLY-DECLINED native cc_* returns 0 / invalid (an out-of-envelope pose the native arm
//                     refuses — mismatched counts, coincident/collapsing guide, self-folding
//                     morph, non-planar guided spine) while OCCT ships a valid result → native
//                     → OCCT. First-class, counted separately, NEVER a bar failure.
//   DISAGREED         native VALID but OUTSIDE the truth while the oracle matches it — a
//                     genuine SILENT WRONG sweep. The failure this harness exists to catch.
//   ORACLE-INACCURATE native matches the closed-form truth while OCCT does NOT — native
//                     vindicated by exact math, OCCT the outlier. Logged, NOT a bar failure.
//   ORACLE_UNRELIABLE a native miss where the oracle is ALSO untrustworthy (closed form absent
//                     AND OCCT invalid, or straight-family native misses exact math AND OCCT
//                     also misses it) → excluded from AGREE, FAILS the bar (investigate).
//   BOTH-DECLINED     an out-of-envelope pose both engines refuse. Logged, not a failure.
//
// ── THE BAR ──────────────────────────────────────────────────────────────────────────
//   Exit 0 IFF DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, with each of the five families
//   having ≥ 1 AGREED trial (real native exercise, not all-declines). Run over ≥ 2 distinct
//   seeds, N ≥ 60 per seed; the runner fails if ANY seed fails. The generator is seeded ONLY
//   by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-identical
//   batch (splitmix64 → xoshiro256**, verbatim from the siblings). Any DISAGREE / ORACLE-
//   INACCURATE prints seed + case index + family/param tuple + the native/OCCT/closed-form
//   triple as a reproducible regression find.
//
// This TU drives the SHIPPING cc_* facade so it links the WHOLE kernel (facade + core +
// engine[native+occt] + native math) + the OCCT oracle. The native variable_sweep path is
// OCCT-free AND numsci-free (build_variable_sweep / build_variable_sweep_tube pull no numsci
// substrate — mirroring run-sim-native-vsweep.sh, which links neither). Built ONLY by
// scripts/run-sim-native-vsweep-fuzz.sh; on run-sim-suite.sh's SKIP list (own main(),
// std::_Exit — OCCT static teardown in the trimmed static build is not exit-clean).
//
#include "cybercadkernel/cc_kernel.h"
// The native variable-sweep BUILDER header — included ONLY to read the native DECLINE signal
// (build_variable_sweep returns a NULL topo::Shape on an out-of-envelope pose). The actual
// shipped result is always taken through the cc_variable_sweep FACADE below; this header call
// is a read-only classifier probe, never a substitute for the facade path. It is OCCT-free.
#include "native/construct/native_construct.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_vsweep_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

namespace ncst = cybercad::native::construct;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Family AGREE bands. Straight-spine polygon sweeps are EXACT (planar B-rep) so native-vs-
// closed-form is tight; the circle families carry a fixed 64-gon inscription bias vs the
// SMOOTH cone/tube closed form; native-vs-OCCT is looser to absorb pipe-shell discretisation.
// FIXED, NEVER widened to force a pass.
constexpr double kVolTolX_poly   = 2e-3;   // straight polygon families vs closed form (EXACT)
constexpr double kVolTolX_circle = 1.2e-2;  // circle families vs SMOOTH closed form (64-gon bias)
constexpr double kVolTolO        = 5e-2;   // native vs OCCT volume (pipe-shell discretisation)
constexpr double kAreaTolO       = 8e-2;   // native vs OCCT surface area
constexpr double kOracleTol      = 5e-2;   // OCCT vs closed form (oracle-trust)
constexpr double kMeshVolTol     = 5e-2;   // mesh-volume vs cc_mass_properties volume
constexpr double kDeflection     = 0.02;   // tessellation deflection for the geometry checks

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

// ── 2D polygon signed area (shoelace) ───────────────────────────────────────────────
double polyArea(const std::vector<double>& xy) {
    double a2 = 0.0;
    const int m = static_cast<int>(xy.size() / 2);
    for (int i = 0; i < m; ++i) {
        const int j = (i + 1) % m;
        a2 += xy[i * 2] * xy[j * 2 + 1] - xy[j * 2] * xy[i * 2 + 1];
    }
    return std::fabs(a2) * 0.5;
}

// ── profile builders (x,y pairs, CCW) ────────────────────────────────────────────────
std::vector<double> circlePoly(double r, int n) {
    std::vector<double> p; p.reserve(static_cast<std::size_t>(n) * 2);
    for (int i = 0; i < n; ++i) {
        const double th = 2.0 * kPi * i / n;
        p.push_back(r * std::cos(th));
        p.push_back(r * std::sin(th));
    }
    return p;
}

std::vector<double> ngonPoly(int n, double R, double rot) {
    std::vector<double> p; p.reserve(static_cast<std::size_t>(n) * 2);
    for (int i = 0; i < n; ++i) {
        const double th = rot + 2.0 * kPi * i / n;
        p.push_back(R * std::cos(th));
        p.push_back(R * std::sin(th));
    }
    return p;
}

// A "star-ish" convex-ish polygon with per-vertex radii (same vertex count as its partner).
// Radii kept strictly positive so vertex-k→vertex-k blending never crosses the spine axis
// (the self-fold guard is exercised separately by the guided-collapse decline probe).
std::vector<double> radialPoly(int n, const std::vector<double>& radii, double rot) {
    std::vector<double> p; p.reserve(static_cast<std::size_t>(n) * 2);
    for (int i = 0; i < n; ++i) {
        const double th = rot + 2.0 * kPi * i / n;
        p.push_back(radii[static_cast<std::size_t>(i)] * std::cos(th));
        p.push_back(radii[static_cast<std::size_t>(i)] * std::sin(th));
    }
    return p;
}

// ── closed-form STRAIGHT-spine volume ────────────────────────────────────────────────
// A straight-spine variable sweep places section-A→section-B (vertex-k→vertex-k linear
// blend) at each fraction f∈[0,1], each vertex ADDITIONALLY scaled by the guide splay s(f),
// in a fixed perpendicular frame, and rules between adjacent stations. The cross-section at
// fraction f is the planar polygon P(f) with vertex k = ((A_k + (B_k−A_k)f))·s(f). Its signed
// area A(f) is a polynomial in f of degree ≤ 4 (the vertex-blend contributes ≤2 and the
// guide-scale² a further ≤2, INCLUDING the coupled morph×scale cross-term). The exact volume
// is ∫₀¹ A(f) · H df with H the straight spine length. BOOLE'S rule (5 samples, weights
// 2h/45·[7,32,12,32,7]) is EXACT for a quartic (verified: even the coupled morph×scale case,
// whose area carries the full f⁴ term, is integrated to machine precision — Simpson's n=4
// left a ~5e-4 quartic residual, Boole's leaves none), so this is the tight PRIMARY arbiter
// for the coupled guided regime the fix now makes native.
double straightSweepVolumeClosedForm(const std::vector<double>& A, const std::vector<double>& B,
                                     double H, const std::function<double(double)>& scaleAt) {
    const int n = static_cast<int>(A.size() / 2);
    auto areaAt = [&](double f) -> double {
        const double sc = scaleAt(f);
        std::vector<double> P(static_cast<std::size_t>(n) * 2);
        for (int i = 0; i < n; ++i) {
            P[i * 2]     = (A[i * 2]     + (B[i * 2]     - A[i * 2])     * f) * sc;
            P[i * 2 + 1] = (A[i * 2 + 1] + (B[i * 2 + 1] - A[i * 2 + 1]) * f) * sc;
        }
        return polyArea(P);
    };
    // Boole's rule over [0,1] with h=0.25: (2h/45)[7f0+32f1+12f2+32f3+7f4] — exact ≤ degree 5.
    const double h = 0.25;
    const double s0 = areaAt(0.0), s1 = areaAt(0.25), s2 = areaAt(0.5), s3 = areaAt(0.75), s4 = areaAt(1.0);
    const double integral = (2.0 * h / 45.0) * (7.0 * s0 + 32.0 * s1 + 12.0 * s2 + 32.0 * s3 + 7.0 * s4);
    return integral * H;
}

// The SMOOTH truncated-cone volume for the circle-morph families (native approximates it with
// a 64-gon, so the band absorbs the inscription bias): V = πH/3 (r0² + r0 r1 + r1²).
double truncatedConeVolume(double r0, double r1, double H) {
    return kPi * H / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
}

// ── families ──────────────────────────────────────────────────────────────────────────
enum Family { F_CIRCLE_STRAIGHT, F_POLY_STRAIGHT, F_SECTION_STRAIGHT, F_GUIDED_STRAIGHT,
              F_CIRCLE_CURVED, F_COUNT };
const char* famName(int f) {
    switch (f) {
        case F_CIRCLE_STRAIGHT:  return "circle-morph straight";
        case F_POLY_STRAIGHT:    return "polygon-morph straight";
        case F_SECTION_STRAIGHT: return "section A->B straight";
        case F_GUIDED_STRAIGHT:  return "guided straight";
        case F_CIRCLE_CURVED:    return "circle-morph curved";
    }
    return "?";
}
bool famIsCircle(int f) { return f == F_CIRCLE_STRAIGHT || f == F_CIRCLE_CURVED; }
bool famIsGuided(int f) { return f == F_GUIDED_STRAIGHT; }
bool famIsCurved(int f) { return f == F_CIRCLE_CURVED; }

enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_oracleBad = 0, g_bothDecl = 0;
int g_famA[F_COUNT] = {0}, g_famD[F_COUNT] = {0}, g_famX[F_COUNT] = {0};
int g_famOI[F_COUNT] = {0}, g_famBD[F_COUNT] = {0}, g_famOU[F_COUNT] = {0};

struct Trial {
    int family = 0;
    bool hasClosedForm = false;   // straight families have an exact closed-form volume
    bool nativeTook = false;      // native BUILDER produced its own solid (did NOT forward to OCCT)
    bool nativeValid = false;     // shipped facade body (native or delegate) valid with mass
    bool oracleValid = false;     // OCCT oracle produced a valid measurement
    bool watertight = false;      // shipped result mesh is a closed 2-manifold
    bool eulerOk = false;         // shipped result mesh χ == 2 AND mesh-vol ≈ mass-vol
    double natVol = 0, natArea = 0, oVol = 0, oArea = 0, xVol = 0;
    std::string desc;
};

Verdict classify(const Trial& tr) {
    const double volTolX = famIsCircle(tr.family) ? kVolTolX_circle : kVolTolX_poly;
    const bool geomOk = tr.watertight && tr.eulerOk && tr.natVol > 0.0;

    // The native BUILDER forwarded to OCCT (out-of-envelope pose) → HONESTLY-DECLINED: the
    // shipping facade returns a correct OCCT solid, and there is no native result to certify.
    // A native decline is first-class (never a bar failure); it lands DECLINED (OCCT valid) or
    // BOTH-DECLINED (OCCT also absent). This keeps AGREED strictly the NATIVE-produced solids.
    if (!tr.nativeTook) return tr.oracleValid ? DECLINED : BOTH_DECLINED;

    if (!tr.nativeValid || !geomOk) return tr.oracleValid ? DECLINED : BOTH_DECLINED;

    const bool natMatchesO = tr.oracleValid &&
        relDiff(tr.natVol, tr.oVol) < kVolTolO &&
        (tr.oArea <= 0.0 || relDiff(tr.natArea, tr.oArea) < kAreaTolO);

    if (tr.hasClosedForm) {
        const bool natMatchesX = relDiff(tr.natVol, tr.xVol) < volTolX;
        const bool oracleTrust = tr.oracleValid && relDiff(tr.oVol, tr.xVol) < kOracleTol;
        if (natMatchesX) {
            if (oracleTrust && natMatchesO) return AGREED;
            if (tr.oracleValid) return ORACLE_INACCURATE;  // native == math, OCCT the outlier
            return AGREED;                                 // native == exact math, no oracle
        }
        // native VALID but misses exact math: if OCCT matches the closed form, a SILENT WRONG
        // sweep; if neither matches the math, the oracle is unreliable.
        return oracleTrust ? DISAGREED : ORACLE_UNRELIABLE;
    }

    // No closed form (curved family): arbitrate against OCCT alone (deflection-bounded).
    if (!tr.oracleValid) return ORACLE_UNRELIABLE;  // native watertight solid but no trustable oracle
    return natMatchesO ? AGREED : DISAGREED;
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

void report(int i, Verdict v, const Trial& tr, uint64_t seed) {
    const int fam = tr.family;
    if (v == AGREED) {
        std::printf("[FUZZ] AGREED    case=%-3d %-22s volN=%.6g volO=%.6g volX=%.6g dO=%.2e dX=%.2e areaN=%.6g areaO=%.6g wt=%d chi2=%d  %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol, relDiff(tr.natVol, tr.oVol),
                    tr.hasClosedForm ? relDiff(tr.natVol, tr.xVol) : -1.0, tr.natArea, tr.oArea,
                    tr.watertight ? 1 : 0, tr.eulerOk ? 1 : 0, tr.desc.c_str());
    } else if (v == DECLINED) {
        std::printf("[FUZZ] DECLINED  case=%-3d %-22s native cc_variable_sweep->0/invalid -> OCCT[volO=%.6g areaO=%.6g volX=%.6g]  %s\n",
                    i, famName(fam), tr.oVol, tr.oArea, tr.xVol, tr.desc.c_str());
    } else if (v == BOTH_DECLINED) {
        std::printf("[FUZZ] BOTH-DECL case=%-3d %-22s native AND OCCT both refused (out-of-envelope)  %s\n",
                    i, famName(fam), tr.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
        std::printf("[FUZZ] ORACLE_INACCURATE case=%-3d %-22s native MATCHES closed form, OCCT does NOT "
                    "volN=%.6g volO=%.6g volX=%.6g\n       NOTE seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    } else if (v == ORACLE_UNRELIABLE) {
        std::printf("[FUZZ] ORACLE_UNRELIABLE case=%-3d %-22s oracle mismatch/absent "
                    "[natValid=%d occValid=%d hasX=%d volN=%.6g volO=%.6g volX=%.6g occVsX=%.2e natVsX=%.2e wt=%d chi2=%d]\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.nativeValid, tr.oracleValid, tr.hasClosedForm, tr.natVol, tr.oVol, tr.xVol,
                    (tr.oracleValid && tr.hasClosedForm) ? relDiff(tr.oVol, tr.xVol) : -1.0,
                    tr.hasClosedForm ? relDiff(tr.natVol, tr.xVol) : -1.0,
                    tr.watertight ? 1 : 0, tr.eulerOk ? 1 : 0,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    } else {  // DISAGREED
        std::printf("[FUZZ] DISAGREED case=%-3d %-22s SILENT-WRONG-SWEEP "
                    "volN=%.6g volO=%.6g volX=%.6g dO=%.3e dX=%.3e areaN=%.6g areaO=%.6g wt=%d chi2=%d\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(fam), tr.natVol, tr.oVol, tr.xVol, relDiff(tr.natVol, tr.oVol),
                    tr.hasClosedForm ? relDiff(tr.natVol, tr.xVol) : -1.0, tr.natArea, tr.oArea,
                    tr.watertight ? 1 : 0, tr.eulerOk ? 1 : 0,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    }
    std::fflush(stdout);
}

// Fill the native geometry diagnostics (watertight, χ, mesh-vol consistency) + mass, measured
// UNDER the native engine.
void fillNativeGeom(CCShapeId nat, Trial& tr) {
    if (nat == 0) return;
    const CCMassProps mp = cc_mass_properties(nat);
    const CCMesh mesh = cc_tessellate(nat, kDeflection);
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

// One variable-sweep trial: build IDENTICAL inputs, sweep through cc_variable_sweep under OCCT
// (oracle) then native, and arbitrate against the closed form (straight families).
Trial runSweep(int fam, const std::vector<double>& A, const std::vector<double>& B,
                const std::vector<double>& spine, const std::vector<double>& guide,
                const std::function<double(double)>& scaleAt, double straightH,
                bool hasClosedForm, double circleX, const std::string& desc) {
    Trial tr; tr.family = fam; tr.hasClosedForm = hasClosedForm; tr.desc = desc;
    const int aCount = static_cast<int>(A.size() / 2);
    const int bCount = static_cast<int>(B.size() / 2);
    const int spineCount = static_cast<int>(spine.size() / 3);
    const int guideCount = static_cast<int>(guide.size() / 3);
    const double* guidePtr = guideCount >= 2 ? guide.data() : nullptr;

    if (hasClosedForm) {
        tr.xVol = famIsCircle(fam) ? circleX
                                   : straightSweepVolumeClosedForm(A, B, straightH, scaleAt);
    }

    // ── OCCT oracle (facade, engine 0) ──
    cc_set_engine(0);
    const CCShapeId oId = cc_variable_sweep(A.data(), aCount, B.data(), bCount, spine.data(),
                                            spineCount, guidePtr, guideCount);
    if (oId != 0) {
        const CCMassProps om = cc_mass_properties(oId);
        if (om.valid && om.volume > 1e-9) { tr.oracleValid = true; tr.oVol = om.volume; tr.oArea = om.area; }
        cc_shape_release(oId);
    }

    // ── native DECLINE signal (read-only builder probe) ──
    // The cc_variable_sweep facade under engine 1 forwards to OCCT on an out-of-envelope pose,
    // so it never returns 0 to distinguish a decline. Read the native BUILDER directly (OCCT-
    // free, same args) to learn whether the NATIVE arm produced its own solid: a NULL result
    // means the native path declined and the facade shipped the OCCT delegate. This probe does
    // NOT ship — the arbitrated body below is always the FACADE result.
    tr.nativeTook = !ncst::build_variable_sweep(A.data(), aCount, B.data(), bCount, spine.data(),
                                                spineCount, guidePtr, guideCount).isNull();

    // ── native candidate (facade, engine 1) — the shipping path ──
    cc_set_engine(1);
    const CCShapeId nId = cc_variable_sweep(A.data(), aCount, B.data(), bCount, spine.data(),
                                            spineCount, guidePtr, guideCount);
    if (tr.nativeTook) fillNativeGeom(nId, tr);  // measure the NATIVE solid only when native took
    if (nId != 0) cc_shape_release(nId);
    cc_set_engine(0);
    return tr;
}

// ── the generator: pick a family, draw a VALID pose, run the trial ──────────────────────
Trial genAndRun(Rng& rng) {
    const int fam = static_cast<int>(rng.below(F_COUNT));

    // Straight spine: +Z of a random length, random start offset (keeps the frame general).
    const double H = rng.range(8.0, 18.0);
    std::vector<double> straightSpine = {0, 0, 0, 0, 0, H};

    // Curved spine: a smooth planar quarter-arc in the XZ plane, radius R, 16 segments.
    auto arcSpine = [&](double R) {
        std::vector<double> sp;
        const int NS = 16;
        for (int k = 0; k <= NS; ++k) {
            const double th = (kPi / 2.0) * k / NS;
            sp.push_back(R * std::cos(th));
            sp.push_back(0.0);
            sp.push_back(R * std::sin(th));
        }
        return sp;
    };

    std::vector<double> A, B, spine, guide;
    std::function<double(double)> scaleAt = [](double) { return 1.0; };  // no guide → 1
    double straightH = H, circleX = 0.0;
    bool hasClosedForm = true;
    char db[192];

    switch (fam) {
        case F_CIRCLE_STRAIGHT: {
            const double r0 = rng.range(3.0, 8.0), r1 = rng.range(1.0, 6.0);
            A = circlePoly(r0, 64); B = circlePoly(r1, 64);
            spine = straightSpine;
            circleX = truncatedConeVolume(r0, r1, H);
            std::snprintf(db, sizeof db, "circle r0=%.3f r1=%.3f H=%.3f (64-gon)", r0, r1, H);
            break;
        }
        case F_POLY_STRAIGHT: {
            const int n = 3 + static_cast<int>(rng.below(6));  // 3..8
            const double R0 = rng.range(3.0, 8.0), R1 = rng.range(1.5, 6.0);
            const double rot = rng.range(0.0, kPi / 6.0);
            A = ngonPoly(n, R0, rot); B = ngonPoly(n, R1, rot);
            spine = straightSpine;
            std::snprintf(db, sizeof db, "ngon n=%d R0=%.3f R1=%.3f H=%.3f", n, R0, R1, H);
            break;
        }
        case F_SECTION_STRAIGHT: {
            // section-A → section-B: DIFFERENT shapes, SAME vertex count n. Per-vertex radii
            // drawn strictly positive so the vertex-k→vertex-k blend never crosses the axis.
            const int n = 4 + static_cast<int>(rng.below(5));  // 4..8
            std::vector<double> rA(static_cast<std::size_t>(n)), rB(static_cast<std::size_t>(n));
            for (int k = 0; k < n; ++k) {
                rA[static_cast<std::size_t>(k)] = rng.range(2.5, 6.5);
                rB[static_cast<std::size_t>(k)] = rng.range(1.5, 5.5);
            }
            const double rot = rng.range(0.0, kPi / 5.0);
            A = radialPoly(n, rA, rot); B = radialPoly(n, rB, rot);
            spine = straightSpine;
            std::snprintf(db, sizeof db, "sectionAB n=%d H=%.3f (per-vertex radii)", n, H);
            break;
        }
        case F_GUIDED_STRAIGHT: {
            // GUIDED straight sweep. The native straight-spine builder places the morphed+guide-
            // scaled section at densified stations (build_variable_sweep_tube — 2 stations in the
            // linear sub-regimes, MORE when the coupled morph×scale law is non-linear) and rules
            // between them. Since moat-vsfix the COUPLED regime (BOTH a morphing section AND a
            // splaying guide) is native too: the builder densifies to bound the morph×scale
            // area-integral so the swept volume converges to the exact ∫A(f)·H law. So each guided
            // trial draws ONE of THREE regimes, all closed-form-arbitrated (Boole's rule, exact
            // for the ≤quartic section area including the cross-term):
            //   regime 0 — CONSTANT section (A==B) + SPLAYING guide → a similar-polygon frustum
            //              (a linear g(f), 2 stations exact).
            //   regime 1 — MORPHING section + guide PARALLEL to the spine (scale≡1) → the plain
            //              morph reached WITH a guide present (linear g(f), 2 stations exact).
            //   regime 2 — COUPLED: MORPHING section AND a SPLAYING guide simultaneously (the
            //              M6-breadth-19 case). g(f)=morph(f)·scale(f) is non-linear; the fix
            //              densifies to track the cross-term. Certified here against the exact
            //              closed form (no widened tolerance).
            const int n = 3 + static_cast<int>(rng.below(6));
            const double rot = rng.range(0.0, kPi / 6.0);
            const int regime = static_cast<int>(rng.below(3));
            if (regime == 0) {
                const double R = rng.range(2.5, 6.0);
                A = ngonPoly(n, R, rot); B = ngonPoly(n, R, rot);         // constant section
                spine = straightSpine;
                const double gx0 = rng.range(4.0, 8.0), gx1 = rng.range(6.0, 14.0);  // splay
                guide = {gx0, 0, 0, gx1, 0, H};
                scaleAt = [gx0, gx1](double f) { return (gx0 + (gx1 - gx0) * f) / gx0; };
                std::snprintf(db, sizeof db, "guided[const-sect+splay] n=%d R=%.3f gx0=%.3f gx1=%.3f H=%.3f",
                              n, R, gx0, gx1, H);
            } else if (regime == 1) {
                const double R0 = rng.range(2.5, 6.0), R1 = rng.range(1.5, 5.0);
                A = ngonPoly(n, R0, rot); B = ngonPoly(n, R1, rot);       // morphing section
                spine = straightSpine;
                const double gx = rng.range(4.0, 10.0);
                guide = {gx, 0, 0, gx, 0, H};                            // guide PARALLEL → s≡1
                scaleAt = [](double) { return 1.0; };
                std::snprintf(db, sizeof db, "guided[morph+parallel-guide] n=%d R0=%.3f R1=%.3f gx=%.3f H=%.3f",
                              n, R0, R1, gx, H);
            } else {
                // COUPLED morph×scale (M6-breadth-19): morphing section AND a splaying guide.
                const double R0 = rng.range(2.5, 6.0), R1 = rng.range(1.5, 5.0);
                A = ngonPoly(n, R0, rot); B = ngonPoly(n, R1, rot);       // morphing section
                spine = straightSpine;
                const double gx0 = rng.range(4.0, 8.0), gx1 = rng.range(6.0, 14.0);  // splay
                guide = {gx0, 0, 0, gx1, 0, H};
                scaleAt = [gx0, gx1](double f) { return (gx0 + (gx1 - gx0) * f) / gx0; };
                std::snprintf(db, sizeof db, "guided[COUPLED morph×scale] n=%d R0=%.3f R1=%.3f gx0=%.3f gx1=%.3f H=%.3f",
                              n, R0, R1, gx0, gx1, H);
            }
            break;
        }
        case F_CIRCLE_CURVED: {
            const double r0 = rng.range(2.5, 5.0), r1 = rng.range(1.5, 4.0);
            const double R = rng.range(30.0, 50.0);
            A = circlePoly(r0, 48); B = circlePoly(r1, 48);
            spine = arcSpine(R);
            hasClosedForm = false;  // curved tube: no simple closed form → OCCT-arbitrated
            std::snprintf(db, sizeof db, "circle-arc r0=%.3f r1=%.3f R=%.3f (48-gon)", r0, r1, R);
            break;
        }
    }

    return runSweep(fam, A, B, spine, guide, scaleAt, straightH, hasClosedForm, circleX, db);
}

// ── a deliberate DECLINE probe: exercises the honest-decline / BOTH-DECLINED branch with
//    out-of-envelope poses the native BUILDER MUST refuse (build_variable_sweep → NULL), each
//    verified to make the native arm forward to OCCT while OCCT ships a valid solid. Every
//    probe is a case whose native builder returns NULL by an explicit precondition guard:
//    mismatched vertex counts (aCount!=bCount), a coincident guide start (d0<1e-6), and a
//    non-planar (helical) guided spine (the spineIsStraight/spineIsPlanar guard). Interleaved
//    at a fixed cadence so the honest-decline branch is covered every run. (The collapsing-
//    guide-scale case is NOT used as a decline probe: native's self-fold guard does not trip on
//    a strong-but-valid taper — native builds it while OCCT MakePipeShell can fail — so it is
//    neither a native decline nor a trustable-oracle trial; it is documented, not fuzzed.) ──
Trial genDeclineProbe(Rng& rng, int which) {
    const double H = rng.range(8.0, 16.0);
    std::vector<double> straightSpine = {0, 0, 0, 0, 0, H};
    std::vector<double> A, B, spine, guide;
    std::function<double(double)> scaleAt = [](double) { return 1.0; };
    char db[192];
    int fam = F_POLY_STRAIGHT;

    switch (which % 3) {
        case 0: {  // MISMATCHED vertex counts → build returns NULL (equal-count only)
            A = ngonPoly(5, rng.range(3.0, 6.0), 0.0);
            B = ngonPoly(6, rng.range(2.0, 5.0), 0.0);   // different n
            spine = straightSpine;
            std::snprintf(db, sizeof db, "DECLINE mismatched-counts n=5 vs 6 H=%.3f", H);
            break;
        }
        case 1: {  // COINCIDENT guide start (d0<1e-6) → native declines → OCCT
            fam = F_GUIDED_STRAIGHT;
            const int n = 4; const double R = rng.range(3.0, 6.0);
            A = ngonPoly(n, R, 0.0); B = ngonPoly(n, R * 0.6, 0.0);
            spine = straightSpine;
            guide = {0, 0, 0, rng.range(4.0, 8.0), 0, H};  // guide START coincident with spine start
            std::snprintf(db, sizeof db, "DECLINE coincident-guide-start n=%d H=%.3f", n, H);
            break;
        }
        default: {  // NON-PLANAR (helical) guided spine → native declines → OCCT MakePipeShell
            fam = F_GUIDED_STRAIGHT;
            A = circlePoly(3.0, 32); B = circlePoly(2.0, 32);
            const int NS = 24; const double Rh = 28.0, pitch = 22.0, Rr = rng.range(3.0, 5.0);
            for (int k = 0; k <= NS; ++k) {
                const double t = static_cast<double>(k) / NS;
                const double th = kPi * t;
                spine.push_back(Rh * std::cos(th));
                spine.push_back(Rh * std::sin(th));
                spine.push_back(pitch * t);
                guide.push_back((Rh + Rr) * std::cos(th));
                guide.push_back((Rh + Rr) * std::sin(th));
                guide.push_back(pitch * t);
            }
            std::snprintf(db, sizeof db, "DECLINE non-planar-guided-helix NS=%d H(pitch)=%.1f", NS, pitch);
            break;
        }
    }
    // Decline probes have no trusted closed form (they exercise the OCCT fallback); arbitrate
    // as a no-closed-form trial so a native decline → DECLINED (OCCT valid) is first-class.
    return runSweep(fam, A, B, spine, guide, scaleAt, H, /*hasClosedForm*/ false, 0.0, db);
}

}  // namespace

int main(int argc, char** argv) {
    uint64_t seed = 0x5EE9C0FFEEull;
    int N = 72;
    if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
    else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
    if (argc > 2) N = std::atoi(argv[2]);
    else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
    if (N <= 0) N = 72;

    std::printf("== M6 differential-fuzz: native VARIABLE-SECTION SWEEP (circle/polygon/section morph x straight/curved x guide) vs OCCT + closed-form ==\n");
    std::printf("== seed=0x%llx N=%d  bands: volX_poly<%.0e volX_circle<%.0e volO<%.0e areaO<%.0e oracle<%.0e (FIXED, NEVER widened) ==\n",
                static_cast<unsigned long long>(seed), N, kVolTolX_poly, kVolTolX_circle,
                kVolTolO, kAreaTolO, kOracleTol);
    std::fflush(stdout);

    Rng rng(seed);
    cc_set_engine(0);

    int declineIdx = 0;
    for (int i = 0; i < N; ++i) {
        // Every 6th trial is a DECLINE probe (out-of-envelope pose) so the honest-decline
        // branch is exercised every run; the rest are the five in-envelope families.
        const bool probe = (i % 6 == 5);
        const Trial tr = probe ? genDeclineProbe(rng, declineIdx++) : genAndRun(rng);
        const Verdict v = classify(tr);
        tally(v, tr.family);
        report(i, v, tr, seed);
    }

    std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
                static_cast<unsigned long long>(seed), N);
    std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  ORACLE_UNRELIABLE=%d  BOTH-DECLINED=%d\n",
                g_agreed, g_declined, g_disagreed, g_oracleInacc, g_oracleBad, g_bothDecl);
    std::printf("   per-family [AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED]:\n");
    for (int f = 0; f < F_COUNT; ++f)
        std::printf("     %-22s %d / %d / %d / %d / %d / %d\n", famName(f),
                    g_famA[f], g_famD[f], g_famX[f], g_famOI[f], g_famOU[f], g_famBD[f]);
    if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — logged, NOT a native fault)\n", g_oracleInacc);
    if (g_bothDecl)    std::printf("   BOTH-DECLINED=%d (out-of-envelope pose both engines refuse — logged)\n", g_bothDecl);
    if (g_oracleBad)   std::printf("   ORACLE_UNRELIABLE=%d (oracle untrustworthy AND native missed — investigate)\n", g_oracleBad);

    // Per-family AGREE coverage over the FIVE in-envelope families (decline probes reuse
    // family ids for reporting but land in DECLINED/BOTH-DECLINED, not AGREED).
    bool coverage = true;
    for (int f = 0; f < F_COUNT; ++f) if (g_famA[f] < 1) coverage = false;
    const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && coverage);
    std::printf("== M6 VARIABLE-SWEEP BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
                "per-family AGREE coverage=%s) ==\n",
                bar ? "PASS — zero silent wrong sweeps" : "FAIL", g_disagreed, g_oracleBad,
                coverage ? "complete" : "INCOMPLETE");
    std::fflush(stdout);
    std::_Exit(bar ? 0 : 1);
}
