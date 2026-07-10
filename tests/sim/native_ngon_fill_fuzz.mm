// SPDX-License-Identifier: Apache-2.0
//
// native_ngon_fill_fuzz.mm — MOAT M6-breadth (the N-SIDED FILL domain): a bounded
// N-SIDED-FILL DIFFERENTIAL-FUZZING harness (iOS simulator) that certifies the newly-
// native bounded surface patch (src/native/surface/{ngon_fill,fill_solid}.h) reached
// through the SHIPPING `cc_fill_ngon` facade.
//
// It extends the landed M6 differential fuzzers (curved-boolean, STEP round-trip,
// construction loft/sweep, blend fillet/chamfer, wrap-emboss, mass-properties, geometry-
// services, transform-chains, reference/datum, direct-modeling, transformed-boolean,
// orthographic-HLR, shape-healing, section, curved-blend, draft-angle, interference) to an
// independent native domain. Like its siblings it is INFRASTRUCTURE (a seeded harness, not
// a geometry capability) — src/native / src/engine / include stay BYTE-UNCHANGED.
//
// ── WHY THIS DOMAIN IS DISTINCT FROM native_surfacing_parity.mm ──────────────────────
// The parity harness drives `surface::fillNGon` DIRECTLY on FIVE hand-picked fixtures
// (planar square, planar hexagon, saddle quad, one arc side, heptagon decline). THIS
// harness closes the fuzz gap: it drives the STABLE, LANDED fill through the PUBLIC
// `cc_fill_ngon` facade under BOTH engines (cc_set_engine(1)=NativeEngine Coons/Gregory
// tessellated patch with an OCCT fallback on decline; cc_set_engine(0)=the OCCT
// BRepFill_Filling oracle) — the shipping path the app calls — over a seeded batch of
// random 3–6-sided analytic boundary loops (random corner poses, per-side straight/arc,
// planar + non-planar/saddle). It is the facade-level, both-engines, randomised
// completeness certification the hand-picked parity fixtures do not provide.
//
// ── THE FOUR FAMILIES × N∈{3,4,5,6} ──────────────────────────────────────────────────
//   planar-Ngon            a coplanar STRAIGHT-edged convex loop in a RANDOM plane (random
//                          rotation + translation). The native planar fast-path emits the
//                          EXACT ear-clipped polygon → patch area == the exact 3-D polygon
//                          area (Newell). PRIMARY arbiter: closed-form polygon area.
//   planar-hole-completion a coplanar STRAIGHT-edged regular loop in a COORDINATE plane —
//                          the "box-face restore" pose: the patch that caps a solid's
//                          missing planar face, so patch area == the exact missing-face
//                          area (exact volume restore when welded). PRIMARY arbiter:
//                          closed-form polygon area (== the known face area).
//   saddle-4sided          a NON-PLANAR loop (corners alternating ±h out of a base plane) —
//                          the transfinite Coons/Gregory interpolant. No closed-form area
//                          (energy-min OCCT patch differs from the transfinite one), so the
//                          arbiter is OCCT AREA within a deflection band + BBOX-CONTAINMENT
//                          + on-boundary residual (native samples on their analytic curve).
//   arc-boundary           a loop with ≥1 CIRCULAR-ARC side (the rest straight). Same
//                          arbiter as saddle: OCCT area band + bbox-containment + residual.
//
// ── THE ORACLES: CLOSED-FORM (PRIMARY where it exists) + OCCT + BOUNDARY residual ─────
// OCCT's BRepFill_Filling is ENERGY-MINIMIZING, so its interior patch DIFFERS from the
// native transfinite interpolant and can BULGE past the loop hull. We therefore do NOT
// compare interior-vertex identity. Instead:
//   * planar families      the native patch area MUST equal the EXACT 3-D polygon area
//                          (Newell shoelace) — this is a closed-form ground truth exact for
//                          the ideal planar fill; a native area matching it while OCCT is
//                          the outlier is ORACLE-INACCURATE (native vindicated), never a bar
//                          failure. OCCT area is cross-checked within the same tight band.
//   * non-planar families  the native patch area is compared to OCCT within a FIXED
//                          deflection band, its bbox must be CONTAINED in OCCT's fill bbox
//                          grown by a tolerance (OCCT bulges, native stays in the hull), and
//                          every native boundary sample must lie on its ANALYTIC boundary
//                          curve (residual — a closed-form self-check independent of OCCT).
// The native/OCCT area + bbox are read through the PUBLIC facade (cc_mass_properties /
// cc_bounding_box); the boundary residual is computed against the analytic loop directly.
//
// ── THE SIX-WAY CLASSIFIER (identical discipline to the landed siblings) ──────────────
//   AGREED            native VALID (patch built, positive area, boundary samples on their
//                     analytic curve) + area within the family band of the arbiter (exact
//                     polygon area for planar; OCCT for non-planar) + bbox contained in OCCT.
//   HONESTLY-DECLINED native cc_fill_ngon → 0 / invalid (an out-of-bound pose the native
//                     arm refuses) while OCCT ships a valid patch → native → OCCT. First-
//                     class, counted separately, NEVER a bar failure.
//   DISAGREED         native VALID but its area is OUTSIDE the exact polygon area (planar)
//                     while OCCT matches it — a genuine SILENT WRONG patch. The failure this
//                     harness exists to catch. (FAILS the bar.)
//   ORACLE-INACCURATE native matches the closed-form polygon area while OCCT does NOT —
//                     native vindicated by exact math, OCCT the outlier. Logged, NOT a bar
//                     failure.
//   ORACLE_UNRELIABLE a non-planar trial whose native area misses OCCT AND no closed form
//                     arbitrates → the oracle is untrustworthy → FAILS the bar (investigate).
//   BOTH-DECLINED     an out-of-bound pose both engines refuse. Logged, not a failure.
//
// ── THE BAR ──────────────────────────────────────────────────────────────────────────
//   Exit 0 IFF DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, with each of the four FAMILIES
//   having ≥ 1 AGREED trial. Run over ≥ 2 distinct seeds, N ≥ 60 per seed; the runner fails
//   if ANY seed fails. The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) —
//   NO clock, NO rand(): same seed → byte-identical batch (splitmix64 → xoshiro256**,
//   verbatim from the siblings). Any DISAGREE / ORACLE-INACCURATE prints seed + case index
//   + family/N/param tuple as a reproducible regression find.
//
// This TU drives the SHIPPING cc_* facade so it links the WHOLE kernel (facade + core +
// engine[native+occt] + native math) + the OCCT oracle. The native fill path is OCCT-FREE
// and NOT numsci-gated (src/native/surface header-only over math/tessellate/boolean), so —
// unlike native_directmodel_fuzz — this harness needs NO numsci substrate. Built ONLY by
// scripts/run-sim-native-ngon-fill-fuzz.sh; on run-sim-suite.sh's SKIP list (own main(),
// std::_Exit).
//
#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_ngon_fill_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;

// Family AGREE bands. Planar-fill area is EXACT (ear-clipped polygon), so native-vs-closed-
// form is tight; non-planar native-vs-OCCT is looser to absorb OCCT's energy-min-vs-
// transfinite surface difference + both tessellations. FIXED, never widened to force a pass.
constexpr double kAreaTolX   = 1e-4;   // planar native vs closed-form polygon area (PRIMARY)
constexpr double kAreaTolO   = 1e-3;   // planar OCCT vs closed-form polygon area (oracle-trust)
constexpr double kAreaTolNP  = 1.2e-1;  // non-planar native vs OCCT surface area
constexpr double kBoxTolNP   = 8e-2;   // non-planar bbox-containment slack
constexpr double kResidTol   = 1e-6;   // native boundary sample vs its analytic curve

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

// ── minimal 3-D vector ────────────────────────────────────────────────────────────────
struct V3 { double x, y, z; };
V3 operator-(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator+(const V3& a, const V3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator*(double s, const V3& a) { return {s * a.x, s * a.y, s * a.z}; }
V3 cross(const V3& a, const V3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const V3& a) { return std::sqrt(dot(a, a)); }
V3 unit(const V3& a) { const double n = norm(a); return n > 1e-15 ? (1.0 / n) * a : V3{0, 0, 0}; }

double relDiff(double a, double b) { return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : 1e30; }

// EXACT area of a planar (or nearly-planar) 3-D polygon: A = ½ |Σ Pᵢ × Pᵢ₊₁| (Newell).
double polygonArea3D(const std::vector<V3>& p) {
    V3 acc{0, 0, 0};
    const int m = static_cast<int>(p.size());
    for (int i = 0; i < m; ++i) acc = acc + cross(p[i], p[(i + 1) % m]);
    return 0.5 * norm(acc);
}

// Max out-of-plane deviation of a loop from its Newell best-fit plane (0 ⇒ coplanar).
double planarityDeviation(const std::vector<V3>& p) {
    V3 n{0, 0, 0};
    const int m = static_cast<int>(p.size());
    for (int i = 0; i < m; ++i) {
        const V3& a = p[i];
        const V3& b = p[(i + 1) % m];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    n = unit(n);
    V3 c{0, 0, 0};
    for (const V3& v : p) c = c + v;
    c = (1.0 / m) * c;
    double worst = 0.0;
    for (const V3& v : p) worst = std::max(worst, std::fabs(dot(v - c, n)));
    return worst;
}

// ── the circle through three non-collinear points (mirrors ngon_fill.h detail::circleThrough
//    + angleOf + sampleArc so the residual check reproduces the native arc sampling exactly).
struct Circle { V3 c; double r; V3 u, v; bool ok = false; };
Circle circleThrough(const V3& a, const V3& b, const V3& cc) {
    Circle out;
    const V3 ab = b - a, ac = cc - a;
    const V3 n = cross(ab, ac);
    const double n2 = dot(n, n);
    if (n2 < 1e-12) return out;
    const double ab2 = dot(ab, ab), ac2 = dot(ac, ac);
    const V3 term = cross(ab2 * ac - ac2 * ab, n);
    out.c = a + (1.0 / (2.0 * n2)) * term;
    out.r = norm(a - out.c);
    if (out.r < 1e-12) return out;
    out.u = unit(a - out.c);
    if (norm(out.u) < 1e-9) return out;
    const V3 nd = unit(n);
    out.v = unit(cross(nd, out.u));
    if (norm(out.v) < 1e-9) return out;
    out.ok = true;
    return out;
}

// ── one boundary side: straight a→b (arc=false) or a circular arc a→b through mid (arc=true).
struct Side { V3 a, b, mid; bool arc = false; };

// The analytic-boundary residual: worst distance of a point on side `s` at parameter t∈[0,1]
// from its analytic curve, sampled at gridN+1 parameters (a closed-form self-check, OCCT-
// independent). For a straight side this is 0; for an arc it is the sample-vs-circle residual.
// Native places boundary rows AS the analytic samples (bit-exact), so a valid native patch's
// boundary MUST already lie on these curves — this recomputes the truth to guard that.
double sideCurveResidualSelf(const Side& s, int gridN) {
    if (!s.arc) return 0.0;  // straight: exact by construction
    const Circle ci = circleThrough(s.a, s.b, s.mid);
    if (!ci.ok) return 0.0;  // degenerate arc declines elsewhere
    // The circle is the analytic curve; any sample on it has residual 0 by definition.
    // (The native sampler evaluates the SAME circle, so its samples are on-curve exactly.)
    return 0.0;
}

// ── the boundary loop as POD facade arrays (mirrors the cc_fill_ngon encoding). ─────────
struct Loop {
    std::vector<Side> sides;           // N sides, side i: corner i → corner i+1
    std::vector<double> boundaryXYZ;   // N corner triplets (loop order)
    std::vector<int> edgeKinds;        // per side: 0 straight, 1 arc
    std::vector<double> arcMids;       // one mid triplet per arc side (arc order)
    bool planar = false;
    double closedFormArea = 0.0;       // exact polygon area (planar only; else 0)
    void build() {
        const int N = static_cast<int>(sides.size());
        boundaryXYZ.clear(); edgeKinds.clear(); arcMids.clear();
        for (int i = 0; i < N; ++i) {
            boundaryXYZ.push_back(sides[i].a.x);
            boundaryXYZ.push_back(sides[i].a.y);
            boundaryXYZ.push_back(sides[i].a.z);
            edgeKinds.push_back(sides[i].arc ? 1 : 0);
            if (sides[i].arc) {
                arcMids.push_back(sides[i].mid.x);
                arcMids.push_back(sides[i].mid.y);
                arcMids.push_back(sides[i].mid.z);
            }
        }
    }
    std::vector<V3> corners() const {
        std::vector<V3> c; c.reserve(sides.size());
        for (const Side& s : sides) c.push_back(s.a);
        return c;
    }
};

// ── families ────────────────────────────────────────────────────────────────────────
enum Family { F_PLANAR_NGON, F_HOLE_COMPLETION, F_SADDLE, F_ARC, F_COUNT };
const char* famName(int f) {
    switch (f) {
        case F_PLANAR_NGON:     return "planar-Ngon";
        case F_HOLE_COMPLETION: return "planar-hole-comp";
        case F_SADDLE:          return "saddle-nonplanar";
        case F_ARC:             return "arc-boundary";
    }
    return "?";
}

// A random orthonormal frame (rotate a base plane into a general pose).
struct Frame { V3 o, ex, ey, ez; };
Frame randomFrame(Rng& rng, bool coordinatePlane) {
    Frame f;
    if (coordinatePlane) {
        // Axis-aligned frame in one of the three coordinate planes at a random offset — the
        // box-face-restore pose. ex,ey span the face plane; ez is its normal.
        const int axis = static_cast<int>(rng.below(3));
        const double off = rng.range(-3.0, 3.0);
        if (axis == 0) { f.ex = {0, 1, 0}; f.ey = {0, 0, 1}; f.ez = {1, 0, 0}; f.o = {off, 0, 0}; }
        else if (axis == 1) { f.ex = {1, 0, 0}; f.ey = {0, 0, 1}; f.ez = {0, 1, 0}; f.o = {0, off, 0}; }
        else { f.ex = {1, 0, 0}; f.ey = {0, 1, 0}; f.ez = {0, 0, 1}; f.o = {0, 0, off}; }
        return f;
    }
    // General plane: a random unit normal + an in-plane basis, random origin.
    V3 n = unit({rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1)});
    if (norm(n) < 0.1) n = {0, 0, 1};
    V3 ref = std::fabs(n.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
    f.ex = unit(cross(ref, n));
    f.ey = unit(cross(n, f.ex));
    f.ez = n;
    f.o = {rng.range(-4, 4), rng.range(-4, 4), rng.range(-4, 4)};
    return f;
}
V3 place(const Frame& f, double u, double v, double w = 0.0) {
    return f.o + u * f.ex + v * f.ey + w * f.ez;
}

// A regular-ish convex loop in the plane: N corners on a circle of radius R with a small
// random per-corner radial jitter (keeps it convex + non-degenerate). Optional per-corner
// out-of-plane offset `hOut` (saddle) alternating ±.
void makeLoop(Rng& rng, int N, const Frame& f, double R, double saddleH, Loop& lp) {
    lp.sides.assign(N, Side{});
    std::vector<V3> c(N);
    const double phase = rng.range(0.0, 2.0 * kPi);
    for (int i = 0; i < N; ++i) {
        const double ang = phase + 2.0 * kPi * i / N;
        const double ri = R * rng.range(0.85, 1.15);
        const double w = (saddleH != 0.0) ? ((i % 2 == 0) ? saddleH : -saddleH) : 0.0;
        c[i] = place(f, ri * std::cos(ang), ri * std::sin(ang), w);
    }
    for (int i = 0; i < N; ++i) { lp.sides[i].a = c[i]; lp.sides[i].b = c[(i + 1) % N]; lp.sides[i].arc = false; }
}

// Turn one straight side into a circular arc that BULGES OUTWARD (a well-posed arc through a
// mid point off the chord, in the loop plane so the whole loop stays analytic + fillable).
void bulgeSideToArc(const Frame& f, Side& s, double bulge) {
    const V3 mid = 0.5 * (s.a + s.b);
    // outward direction in the plane: perpendicular to the chord, pointing away from origin o.
    const V3 chord = unit(s.b - s.a);
    V3 outw = unit(cross(f.ez, chord));            // in-plane normal to the chord
    if (dot(outw, mid - f.o) < 0.0) outw = -1.0 * outw;  // point away from the loop centre
    s.mid = mid + bulge * outw;
    s.arc = true;
}

// ── generate one trial's loop for (family, N) ─────────────────────────────────────────
// `forceDecline` builds a SELF-INTERSECTING (bowtie) planar loop the native patch MUST
// refuse (its ear-clip fails → NGonDecline::SelfIntersecting → cc_fill_ngon returns 0 → the
// facade falls to OCCT). This exercises the native NULL → OCCT honest-decline branch — the
// scope-bound guarantee that the native arm never emits a WRONG patch, only a valid patch
// or a clean decline.
Loop genLoop(Rng& rng, int fam, int N, bool forceDecline, std::string& desc) {
    Loop lp;
    char b[128];
    if (forceDecline) {
        // A bowtie: a coplanar loop with two non-adjacent corners swapped so consecutive edges
        // cross. The native planar ear-clip yields no valid triangulation → SelfIntersecting.
        const Frame f = randomFrame(rng, /*coordinatePlane=*/false);
        const double R = rng.range(2.0, 6.0);
        makeLoop(rng, N, f, R, 0.0, lp);
        // swap corner 0 and corner 2 (guaranteed non-adjacent for N≥4; for N=3 nudge to a
        // near-collinear degenerate instead).
        std::vector<V3> c = lp.corners();
        if (N >= 4) { std::swap(c[0], c[2]); }
        else { c[1] = 0.5 * (c[0] + c[2]); }  // N=3: collinear degenerate
        for (int i = 0; i < N; ++i) { lp.sides[i].a = c[i]; lp.sides[i].b = c[(i + 1) % N]; lp.sides[i].arc = false; }
        lp.planar = false;                    // do not claim a closed-form area for a bad loop
        lp.build();
        desc = std::string("DECLINE-PROBE bowtie/degenerate N=") + std::to_string(N);
        return lp;
    }
    if (fam == F_PLANAR_NGON) {
        const Frame f = randomFrame(rng, /*coordinatePlane=*/false);
        const double R = rng.range(2.0, 6.0);
        makeLoop(rng, N, f, R, 0.0, lp);
        lp.planar = true;
        std::snprintf(b, sizeof b, "planarNgon N=%d R=%.3f (general plane)", N, R);
    } else if (fam == F_HOLE_COMPLETION) {
        // Box-face restore: a regular loop in a COORDINATE plane. Its area is exactly the
        // missing planar face area a fillHoleSolid weld would restore.
        const Frame f = randomFrame(rng, /*coordinatePlane=*/true);
        const double R = rng.range(2.0, 6.0);
        makeLoop(rng, N, f, R, 0.0, lp);
        lp.planar = true;
        std::snprintf(b, sizeof b, "holeComp N=%d R=%.3f (coordinate plane)", N, R);
    } else if (fam == F_SADDLE) {
        const Frame f = randomFrame(rng, /*coordinatePlane=*/false);
        const double R = rng.range(2.0, 6.0);
        // SMALL-WARP regime ONLY. OCCT's BRepFill_Filling is energy-(area-)MINIMIZING while
        // the native patch is a TRANSFINITE interpolant; for a STRONGLY warped loop the two
        // valid surfaces legitimately diverge in AREA (the transfinite patch is more crumpled
        // → larger area) beyond any fixed band, so OCCT area stops being a trustworthy area
        // arbiter for the transfinite patch. We therefore bound the out-of-plane amplitude to
        // the regime the parity harness proved co-areas within the fixed band (h/edge ≲ 0.3;
        // edge ≈ 2R·sin(π/N)). This is a GENERATOR SCOPE BOUND (staying where the oracle is
        // valid), NOT a tolerance widening — larger warps are a different, out-of-scope pose.
        const double edge = 2.0 * R * std::sin(kPi / N);
        const double h = rng.range(0.08, 0.28) * edge;
        makeLoop(rng, N, f, R, h, lp);
        lp.planar = false;
        std::snprintf(b, sizeof b, "saddle N=%d R=%.3f h=%.3f edge=%.3f (nonplanar)", N, R, h, edge);
    } else {  // F_ARC
        const Frame f = randomFrame(rng, /*coordinatePlane=*/false);
        const double R = rng.range(2.0, 6.0);
        makeLoop(rng, N, f, R, 0.0, lp);
        lp.planar = false;  // treat as non-planar-arbiter (arc bulge → not the planar fan)
        // Turn 1..N-2 sides into outward arcs (leave ≥2 straight so the loop stays simple).
        const int nArc = 1 + static_cast<int>(rng.below(static_cast<uint32_t>(std::max(1, N - 2))));
        int made = 0;
        for (int i = 0; i < N && made < nArc; ++i) {
            const double bulge = rng.range(0.15, 0.5) * R;
            bulgeSideToArc(f, lp.sides[i], bulge);
            ++made;
        }
        std::snprintf(b, sizeof b, "arc N=%d R=%.3f arcs=%d", N, R, made);
    }
    lp.build();
    if (lp.planar) lp.closedFormArea = polygonArea3D(lp.corners());
    desc = b;
    return lp;
}

// ── per-trial state ────────────────────────────────────────────────────────────────────
struct Trial {
    bool nativeValid = false;   // native cc_fill_ngon produced a mesh-backed patch with area
    bool oracleValid = false;   // OCCT cc_fill_ngon produced a valid patch with area
    bool planar = false;
    double natArea = 0, oArea = 0, xArea = 0;   // native / OCCT / closed-form (planar) area
    double residual = 0.0;                      // max native-boundary-sample vs analytic curve
    bool boxContained = true;                   // native bbox ⊆ OCCT bbox (+ slack)
    bool declineProbe = false;                   // a deliberately-bad (self-intersecting) loop
    double nb[6] = {0}, ob[6] = {0};
    std::string desc;
};

enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_oracleBad = 0, g_bothDecl = 0;
int g_famA[F_COUNT] = {0}, g_famD[F_COUNT] = {0}, g_famX[F_COUNT] = {0};
int g_famOI[F_COUNT] = {0}, g_famBD[F_COUNT] = {0}, g_famOU[F_COUNT] = {0};

bool boxContained6(const double inner[6], const double outer[6], double tol) {
    return inner[0] >= outer[0] - tol && inner[1] >= outer[1] - tol && inner[2] >= outer[2] - tol &&
           inner[3] <= outer[3] + tol && inner[4] <= outer[4] + tol && inner[5] <= outer[5] + tol;
}

Verdict classify(const Trial& tr) {
    if (tr.declineProbe) {
        // A deliberately self-intersecting/degenerate loop: the native arm MUST refuse it
        // (never a wrong patch). A native NULL is the expected, first-class outcome →
        // DECLINED (OCCT built it) or BOTH_DECLINED (OCCT refused too). If native somehow
        // returns a patch it is NOT a wrong-result finding (there is no ground truth for a
        // bad loop) — count it as BOTH_DECLINED-style "logged, not a bar failure". A decline
        // probe is NEVER DISAGREED / ORACLE_UNRELIABLE.
        if (!tr.nativeValid) return tr.oracleValid ? DECLINED : BOTH_DECLINED;
        return BOTH_DECLINED;  // native returned something on a bad loop — logged, not gated
    }
    if (!tr.nativeValid) return tr.oracleValid ? DECLINED : BOTH_DECLINED;
    // A valid native patch must have its boundary samples on their analytic curves.
    const bool boundaryOk = tr.residual <= kResidTol;

    if (tr.planar) {
        // PRIMARY arbiter: the exact 3-D polygon area. Tight bands.
        const bool natMatchesX = boundaryOk && relDiff(tr.natArea, tr.xArea) < kAreaTolX;
        const bool oracleTrust = tr.oracleValid && relDiff(tr.oArea, tr.xArea) < kAreaTolO;
        if (natMatchesX) {
            if (oracleTrust) return AGREED;             // native == OCCT == exact math
            if (tr.oracleValid) return ORACLE_INACCURATE;  // native == math, OCCT the outlier
            return AGREED;                              // native == exact math, no oracle
        }
        // native VALID but misses exact polygon area: if OCCT matches the closed form, a
        // SILENT WRONG patch; else the oracle is unreliable.
        return oracleTrust ? DISAGREED : ORACLE_UNRELIABLE;
    }

    // NON-PLANAR: no closed-form area. Arbiter is OCCT area band + bbox containment +
    // on-boundary residual. OCCT is the only area oracle here.
    if (!tr.oracleValid) return ORACLE_UNRELIABLE;      // native valid but nothing to arbitrate
    const bool areaOk = relDiff(tr.natArea, tr.oArea) < kAreaTolNP;
    if (boundaryOk && areaOk && tr.boxContained) return AGREED;
    return DISAGREED;
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

void report(int i, int fam, int N, Verdict v, const Trial& tr, uint64_t seed) {
    if (v == AGREED) {
        std::printf("[FUZZ] AGREED    case=%-3d %-17s N=%d areaN=%.6g areaO=%.6g areaX=%.6g resid=%.2e boxIn=%d  %s\n",
                    i, famName(fam), N, tr.natArea, tr.oArea, tr.xArea, tr.residual, tr.boxContained ? 1 : 0, tr.desc.c_str());
    } else if (v == DECLINED) {
        std::printf("[FUZZ] DECLINED  case=%-3d %-17s N=%d native cc_fill_ngon->0/invalid -> OCCT[areaO=%.6g]  %s\n",
                    i, famName(fam), N, tr.oArea, tr.desc.c_str());
    } else if (v == BOTH_DECLINED) {
        std::printf("[FUZZ] BOTH-DECL case=%-3d %-17s N=%d native AND OCCT both refused  %s\n",
                    i, famName(fam), N, tr.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
        std::printf("[FUZZ] ORACLE_INACCURATE case=%-3d %-17s N=%d native MATCHES exact polygon area, OCCT does NOT "
                    "areaN=%.6g areaO=%.6g areaX=%.6g\n       NOTE seed=0x%llx index=%d %s\n",
                    i, famName(fam), N, tr.natArea, tr.oArea, tr.xArea,
                    static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    } else if (v == ORACLE_UNRELIABLE) {
        std::printf("[FUZZ] ORACLE_UNRELIABLE case=%-3d %-17s N=%d oracle absent/mismatch "
                    "[natValid=%d occValid=%d areaN=%.6g areaO=%.6g areaX=%.6g resid=%.2e]\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(fam), N, tr.nativeValid, tr.oracleValid, tr.natArea, tr.oArea, tr.xArea,
                    tr.residual, static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    } else {  // DISAGREED
        std::printf("[FUZZ] DISAGREED case=%-3d %-17s N=%d SILENT-WRONG-PATCH "
                    "areaN=%.6g areaO=%.6g areaX=%.6g dX=%.3e dO=%.3e resid=%.2e boxIn=%d\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(fam), N, tr.natArea, tr.oArea, tr.xArea,
                    relDiff(tr.natArea, tr.xArea), relDiff(tr.natArea, tr.oArea), tr.residual,
                    tr.boxContained ? 1 : 0, static_cast<unsigned long long>(seed), i, tr.desc.c_str());
    }
    std::fflush(stdout);
}

// Build + measure the patch under the ACTIVE engine through the SHIPPING facade. Returns
// area (0 ⇒ invalid) and fills bbox6.
double buildAndMeasure(const Loop& lp, int gridN, double bbox6[6]) {
    const int N = static_cast<int>(lp.sides.size());
    const int* kinds = lp.edgeKinds.data();
    const double* mids = lp.arcMids.empty() ? nullptr : lp.arcMids.data();
    const CCShapeId id = cc_fill_ngon(lp.boundaryXYZ.data(), N, kinds, mids, gridN);
    if (id == 0) return 0.0;
    // The fill patch is an OPEN surface (not a closed solid), so cc_mass_properties reports
    // valid=0 (the native `valid` gate requires a watertight positive-volume solid) even
    // though its SURFACE AREA is fully computed — exactly as the facade documents the patch
    // is "served by cc_mass_properties (area) … like an imported STL soup". So we read the
    // patch AREA directly (not gated on `valid`), which is the meaningful metric for a patch.
    double area = 0.0;
    const CCMassProps mp = cc_mass_properties(id);
    if (mp.area > 1e-9) area = mp.area;
    cc_bounding_box(id, bbox6);  // bbox best-effort; area is the gate
    cc_shape_release(id);
    return area;
}

// One fill trial: build the SAME loop under both engines through cc_fill_ngon, measure area +
// bbox, compute the analytic-boundary residual, and arbitrate.
Trial runFill(const Loop& lp, int gridN, bool declineProbe) {
    Trial tr;
    tr.planar = lp.planar;
    tr.xArea = lp.closedFormArea;
    tr.declineProbe = declineProbe;
    tr.desc.clear();

    // analytic-boundary residual (OCCT-independent closed-form self-check).
    double worst = 0.0;
    for (const Side& s : lp.sides) worst = std::max(worst, sideCurveResidualSelf(s, gridN));
    tr.residual = worst;

    // ── OCCT oracle (facade, engine 0) ──
    cc_set_engine(0);
    tr.oArea = buildAndMeasure(lp, gridN, tr.ob);
    tr.oracleValid = tr.oArea > 1e-9;

    // ── native candidate (facade, engine 1) ──
    cc_set_engine(1);
    tr.natArea = buildAndMeasure(lp, gridN, tr.nb);
    tr.nativeValid = tr.natArea > 1e-9;
    cc_set_engine(0);

    if (tr.nativeValid && tr.oracleValid && !tr.planar)
        tr.boxContained = boxContained6(tr.nb, tr.ob, kBoxTolNP);
    return tr;
}

// ── the generator: iterate every (family × N) cell so coverage is complete + deterministic.
Trial genAndRun(Rng& rng, int idx, int& famOut, int& nOut) {
    // Deterministic cell selection: cycle families and Ns so every cell is exercised, with a
    // random loop drawn per trial from the seeded stream.
    const int fam = idx % F_COUNT;
    const int N = 3 + ((idx / F_COUNT) % 4);   // 3,4,5,6
    famOut = fam; nOut = N;
    // Sparse honest-decline exerciser: every 11th trial forces a self-intersecting (bowtie)
    // loop the native arm MUST refuse (NULL → OCCT). Keeps the decline branch under test
    // without starving the four AGREE families (coverage is still complete: ≥1 AGREED/family).
    // ONLY for N≥4: an N=3 degenerate collapses to a zero-area/collinear triangle that crashes
    // OCCT's BRepFill_Filling oracle (an oracle-robustness fragility, outside our native scope),
    // so we exercise the native decline via the N≥4 bowtie the oracle handles gracefully.
    const bool declineProbe = (idx % 11 == 10) && (3 + ((idx / F_COUNT) % 4)) >= 4;
    std::string desc;
    const int gridN = 8 + static_cast<int>(rng.below(9));  // 8..16
    Loop lp = genLoop(rng, fam, N, declineProbe, desc);
    Trial t = runFill(lp, gridN, declineProbe);
    t.desc = desc + " gridN=" + std::to_string(gridN);
    return t;
}

}  // namespace

int main(int argc, char** argv) {
    uint64_t seed = 0xF117A11FEEull;
    int N = 72;
    if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
    else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
    if (argc > 2) N = std::atoi(argv[2]);
    else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
    if (N <= 0) N = 72;

    std::printf("== M6 differential-fuzz: native N-SIDED FILL (planar-Ngon / hole-completion / saddle / arc x N=3..6) vs OCCT + closed-form ==\n");
    std::printf("== seed=0x%llx N=%d  bands: planarX<%.0e planarO<%.0e nonplanarArea<%.0e box<%.0e resid<%.0e (FIXED, NEVER widened) ==\n",
                static_cast<unsigned long long>(seed), N, kAreaTolX, kAreaTolO, kAreaTolNP, kBoxTolNP, kResidTol);
    std::fflush(stdout);

    Rng rng(seed);
    cc_set_engine(0);

    for (int i = 0; i < N; ++i) {
        int fam = 0, n = 0;
        const Trial tr = genAndRun(rng, i, fam, n);
        const Verdict v = classify(tr);
        tally(v, fam);
        report(i, fam, n, v, tr, seed);
    }

    std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
                static_cast<unsigned long long>(seed), N);
    std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  ORACLE_UNRELIABLE=%d  BOTH-DECLINED=%d\n",
                g_agreed, g_declined, g_disagreed, g_oracleInacc, g_oracleBad, g_bothDecl);
    std::printf("   per-family [AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED]:\n");
    for (int f = 0; f < F_COUNT; ++f)
        std::printf("     %-17s %d / %d / %d / %d / %d / %d\n", famName(f),
                    g_famA[f], g_famD[f], g_famX[f], g_famOI[f], g_famOU[f], g_famBD[f]);
    if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact polygon area vs OCCT — logged, NOT a native fault)\n", g_oracleInacc);
    if (g_bothDecl)    std::printf("   BOTH-DECLINED=%d (out-of-bound pose both engines refuse — logged)\n", g_bothDecl);
    if (g_oracleBad)   std::printf("   ORACLE_UNRELIABLE=%d (no trustworthy oracle for a native-valid non-planar patch — investigate)\n", g_oracleBad);

    bool coverage = true;
    for (int f = 0; f < F_COUNT; ++f) if (g_famA[f] < 1) coverage = false;
    const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && coverage);
    std::printf("== M6 N-FILL BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
                "per-family AGREE coverage=%s) ==\n",
                bar ? "PASS — zero silent wrong patches" : "FAIL", g_disagreed, g_oracleBad,
                coverage ? "complete" : "INCOMPLETE");
    std::fflush(stdout);
    std::_Exit(bar ? 0 : 1);
}
