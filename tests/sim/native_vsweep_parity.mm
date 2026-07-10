// SPDX-License-Identifier: Apache-2.0
//
// native_vsweep_parity.mm — native-vs-OCCT parity for the VARIABLE-SECTION / guide+spine
// SWEEP (moat-vsweep). Gate 2 of the two-gate model: on the iOS simulator (OCCT linked),
// the native cc_variable_sweep result is compared against the OCCT oracle
// (BRepOffsetAPI_MakePipeShell in MULTI-SECTION mode + BRepGProp) at sampled inputs through
// the SAME cc_* facade.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default)
//   cc_set_engine(1)  → NativeEngine (native RMF/perp-framed guide-scaled morph tube —
//                       construct/sweep.h build_variable_sweep; falls through to OCCT for
//                       the deferred cases)
//
// cc_variable_sweep sweeps a section that MORPHS from profile A (spine start) to profile B
// (spine end) along the spine, each station = interpolate(A,B,f), OPTIONALLY guide-SCALED.
// This is a superset of cc_loft_along_rail (adds a guide scale) and cc_guided_sweep (adds
// an A→B morph).
//
// ── NATIVE cases (each MUST take the native path; cc_active_engine==1) ────────────────
//   1. CIRCLE→CIRCLE STRAIGHT (no guide) — a truncated cone. Volume matches OCCT
//      MakePipeShell within the deflection bound; watertight.
//   2. CONSTANT SECTION STRAIGHT (no guide) — a prism (reduces to loft_along_rail); EXACT.
//   3. GUIDE-SCALED SQUARE STRAIGHT — a square frustum steered by a +X-splaying guide.
//   4. CIRCLE→CIRCLE SMOOTH-ARC (no guide) — a curved morph tube; deflection-bounded parity.
//
// ── DEFERRED cases (native returns NULL → forwards to OCCT, tagged [fallback]) ─────────
//   D1. NON-PLANAR (HELICAL) SPINE MORPH — the native builder only frames a straight or
//       smooth-PLANAR spine, so a helical (non-planar) spine morph declines → OCCT
//       MakePipeShell owns it (verified fall-through: native active, native==OCCT result).
//
// Each native case's mass/bbox/tessellate is compared to the OCCT oracle; each deferred
// case asserts the NativeEngine stays active and the native (delegated) result EQUALS the
// OCCT oracle. Output: [NVSWEEP] PASS/FAIL lines with per-op deltas + a native/fallback tag,
// then a summary. On run-sim-suite.sh's SKIP list (own main()).
//
// Build: scripts/run-sim-native-vsweep.sh — compiles this harness + the whole facade/core/
// engine (NativeEngine + OCCT adapter) + src/native/**, links OCCT, spawns on a booted sim.

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

int g_passed = 0;
int g_failed = 0;

void record(bool ok, const char* label, const char* detail) {
    if (ok) {
        ++g_passed;
        std::printf("[NVSWEEP] PASS  %-34s %s\n", label, detail);
    } else {
        ++g_failed;
        std::printf("[NVSWEEP] FAIL  %-34s %s\n", label, detail);
    }
}

struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

bool meshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0) return false;
    std::unordered_map<std::uint64_t, int> edgeCount;
    auto key = [](int a, int b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = m.triangles[t * 3 + 0];
        const int j = m.triangles[t * 3 + 1];
        const int k = m.triangles[t * 3 + 2];
        ++edgeCount[key(i, j)];
        ++edgeCount[key(j, k)];
        ++edgeCount[key(k, i)];
    }
    for (const auto& [e, c] : edgeCount)
        if (c != 2) return false;
    return true;
}

bool meshBBox(const CCMesh& m, double out6[6]) {
    if (m.vertexCount <= 0) return false;
    for (int k = 0; k < 3; ++k) { out6[k] = m.vertices[k]; out6[3 + k] = m.vertices[k]; }
    for (int v = 0; v < m.vertexCount; ++v)
        for (int k = 0; k < 3; ++k) {
            const double c = m.vertices[v * 3 + k];
            if (c < out6[k]) out6[k] = c;
            if (c > out6[3 + k]) out6[3 + k] = c;
        }
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

double maxBBoxCornerDelta(const double a[6], const double b[6]) {
    double d = 0.0;
    for (int k = 0; k < 6; ++k) d = std::max(d, std::fabs(a[k] - b[k]));
    return d;
}

using Builder = CCShapeId (*)();

struct OpCase {
    const char* name;
    Builder build;
    bool exact;        // true → tight tol (exact prism/frustum); false → deflection-scaled
    double deflection;
    double bboxRefExtent;  // characteristic size for the linear bbox tolerance
};

// Compare the native build against the OCCT build; emit mass / bbox / tessellate lines.
void runNativeOp(const OpCase& s) {
    char detail[512];
    const double volRelTol = s.exact ? 1e-4 : 5e-2;
    const double areaRelTol = s.exact ? 5e-3 : 8e-2;
    const double linTol = s.exact ? 1e-3 : 5.0 * s.deflection + 0.02 * s.bboxRefExtent;

    cc_set_engine(0);
    const CCShapeId occtId = s.build();
    if (occtId == 0) {
        std::snprintf(detail, sizeof detail, "[native] OCCT build failed: %s", cc_last_error());
        record(false, s.name, detail);
        return;
    }
    const CCMassProps oM = cc_mass_properties(occtId);
    double oBB[6] = {0};
    const int oBBok = cc_bounding_box(occtId, oBB);

    cc_set_engine(1);
    const int activeIsNative = cc_active_engine();
    const CCShapeId natId = s.build();
    if (natId == 0 || activeIsNative != 1) {
        std::snprintf(detail, sizeof detail, "[native] native build failed (active=%d): %s",
                      activeIsNative, cc_last_error());
        record(false, s.name, detail);
        cc_set_engine(0);
        cc_shape_release(occtId);
        return;
    }
    const CCMassProps nM = cc_mass_properties(natId);
    double nBB[6] = {0};
    const int nBBok = cc_bounding_box(natId, nBB);
    const CCMesh nMesh = cc_tessellate(natId, s.deflection);

    // ── mass properties (volume / area / centroid) ───────────────────────────────
    bool massOk = (oM.valid != 0) && (nM.valid != 0) && (oM.volume > 0.0);
    const double volRel = massOk ? std::fabs(nM.volume - oM.volume) / oM.volume : 1.0;
    const double areaRel = (massOk && oM.area > 0.0) ? std::fabs(nM.area - oM.area) / oM.area : 1.0;
    const double cMax = std::max({std::fabs(nM.cx - oM.cx), std::fabs(nM.cy - oM.cy),
                                  std::fabs(nM.cz - oM.cz)});
    massOk = massOk && volRel < volRelTol && areaRel < areaRelTol && cMax < linTol;
    std::snprintf(
        detail, sizeof detail,
        "[native] vol o=%.6g n=%.6g rel=%.2e | area rel=%.2e | centroidΔ=%.2e (tol v=%.0e c=%.2e)",
        oM.volume, nM.volume, volRel, areaRel, cMax, volRelTol, linTol);
    record(massOk, (std::string(s.name) + " mass").c_str(), detail);

    // ── bounding boxes ───────────────────────────────────────────────────────────
    const double bbDelta = maxBBoxCornerDelta(oBB, nBB);
    const bool bbOk = (oBBok == 1) && (nBBok == 1) && (bbDelta < linTol);
    std::snprintf(detail, sizeof detail, "[native] maxCornerΔ=%.2e (tol=%.2e)", bbDelta, linTol);
    record(bbOk, (std::string(s.name) + " bbox").c_str(), detail);

    // ── native tessellation: WATERTIGHT + self-consistent bbox / volume ──────────
    double meshBB[6] = {0};
    const bool haveMesh = nMesh.triangleCount > 0 && meshBBox(nMesh, meshBB);
    const bool wt = haveMesh && meshWatertight(nMesh);
    const double meshVol = haveMesh ? meshVolume(nMesh) : 0.0;
    const double meshVolRel =
        (haveMesh && nM.volume > 0.0) ? std::fabs(meshVol - nM.volume) / nM.volume : 1.0;
    const double meshBBDelta = haveMesh ? maxBBoxCornerDelta(nBB, meshBB) : 1.0;
    const bool tessOk = haveMesh && wt && meshVolRel < 5e-2 && meshBBDelta < linTol;
    std::snprintf(detail, sizeof detail,
                  "[native] watertight=%d tris=%d meshVolRel=%.2e bboxΔ=%.2e", wt ? 1 : 0,
                  nMesh.triangleCount, meshVolRel, meshBBDelta);
    record(tessOk, (std::string(s.name) + " tessellate").c_str(), detail);

    cc_mesh_free(nMesh);
    cc_set_engine(0);
    cc_shape_release(natId);
    cc_shape_release(occtId);
}

void runFallbackOp(const char* name, Builder build, const char* why) {
    char detail[512];

    cc_set_engine(0);
    const CCShapeId occtId = build();
    const CCMassProps oM = occtId ? cc_mass_properties(occtId) : CCMassProps{0, 0, 0, 0, 0, 0};

    cc_set_engine(1);
    const int activeIsNative = cc_active_engine();
    const CCShapeId natId = build();
    const CCMassProps nM = natId ? cc_mass_properties(natId) : CCMassProps{0, 0, 0, 0, 0, 0};

    const bool bothValid = (occtId != 0) && (natId != 0) && (oM.valid != 0) && (nM.valid != 0);
    const double volRel = (bothValid && oM.volume > 0.0)
                              ? std::fabs(nM.volume - oM.volume) / oM.volume
                              : 1.0;
    const bool ok = (activeIsNative == 1) && bothValid && (oM.volume > 0.0) && volRel < 1e-9;
    std::snprintf(detail, sizeof detail,
                  "[fallback] %s — native active=%d vol o=%.6g n=%.6g rel=%.2e (delegated to OCCT)",
                  why, activeIsNative, oM.volume, nM.volume, volRel);
    record(ok, name, detail);

    cc_set_engine(0);
    if (natId) cc_shape_release(natId);
    if (occtId) cc_shape_release(occtId);
}

// ── per-op builders (identical inputs to both engines) ──────────────────────────

std::vector<double> circlePoly(double r, int n) {
    std::vector<double> p;
    p.reserve(static_cast<std::size_t>(n) * 2);
    for (int i = 0; i < n; ++i) {
        const double th = 2.0 * kPi * i / n;
        p.push_back(r * std::cos(th));
        p.push_back(r * std::sin(th));
    }
    return p;
}

// 1) CIRCLE→CIRCLE STRAIGHT (no guide): r0=5 → r1=2 over a straight +Z spine of length 12
//    — a truncated cone. Native perpendicular-framed ruled morph vs OCCT MakePipeShell.
CCShapeId buildCircleToCircleStraight() {
    static const std::vector<double> A = circlePoly(5.0, 64);
    static const std::vector<double> B = circlePoly(2.0, 64);
    const double spine[] = {0, 0, 0, 0, 0, 12};
    return cc_variable_sweep(A.data(), 64, B.data(), 64, spine, 2, nullptr, 0);
}

// 2) CONSTANT SECTION STRAIGHT (no guide): a 6×6 square swept 10 up — a prism, EXACT.
CCShapeId buildConstantSquareStraight() {
    const double sq[] = {-3, -3, 3, -3, 3, 3, -3, 3};
    const double spine[] = {0, 0, 0, 0, 0, 10};
    return cc_variable_sweep(sq, 4, sq, 4, spine, 2, nullptr, 0);
}

// 3) GUIDE-SCALED SQUARE STRAIGHT: a 2×2 square swept along +Z[0..10], guide splaying in
//    +X from 4→8 → a 2×2 → 4×4 square frustum.
CCShapeId buildGuideScaledSquare() {
    const double sq[] = {-1, -1, 1, -1, 1, 1, -1, 1};
    const double spine[] = {0, 0, 0, 0, 0, 10};
    const double guide[] = {4, 0, 0, 8, 0, 10};
    return cc_variable_sweep(sq, 4, sq, 4, spine, 2, guide, 2);
}

// 4b) COUPLED MORPH×SCALE STRAIGHT (M6-breadth-19 regression): circle r0=5 → r1=2 morph AND
//     a +X-splaying guide (scale 1→2) SIMULTANEOUSLY along a straight +Z spine of length 12.
//     The section radius is the PRODUCT of the morph and the splay → a non-linear (quadratic)
//     coupled law the old 2-station chord dropped (~19% divergence vs OCCT MakePipeShell). The
//     densified native builder now tracks the cross-term ⇒ deflection-bounded parity.
CCShapeId buildCoupledMorphScaleStraight() {
    static const std::vector<double> A = circlePoly(5.0, 64);
    static const std::vector<double> B = circlePoly(2.0, 64);
    const double spine[] = {0, 0, 0, 0, 0, 12};
    const double guide[] = {4, 0, 0, 8, 0, 12};  // splay 4→8 ⇒ scale 1→2
    return cc_variable_sweep(A.data(), 64, B.data(), 64, spine, 2, guide, 2);
}

// 4) CIRCLE→CIRCLE SMOOTH-ARC (no guide): r0=4 → r1=2 along a quarter-arc spine (R=40, XZ
//    plane, 16 segments). Native RMF-transported morph vs OCCT MakePipeShell on the arc.
CCShapeId buildCircleToCircleArc() {
    static const std::vector<double> A = circlePoly(4.0, 48);
    static const std::vector<double> B = circlePoly(2.0, 48);
    std::vector<double> spine;
    const int NS = 16, R = 40;
    for (int k = 0; k <= NS; ++k) {
        const double th = (kPi / 2.0) * k / NS;
        spine.push_back(R * std::cos(th));
        spine.push_back(0.0);
        spine.push_back(R * std::sin(th));
    }
    return cc_variable_sweep(A.data(), 48, B.data(), 48, spine.data(), NS + 1, nullptr, 0);
}

// D1) NON-PLANAR SPINE + GUIDE (deferred): a guide-scaled morph along a helical (non-planar)
//     spine. The GUIDED variable-sweep path only frames a straight/smooth-PLANAR spine
//     (spineIsPlanar guard), so a non-planar guided morph declines → OCCT MakePipeShell owns
//     the genuine corrected-Frenet guided morph (verified fall-through: native==OCCT).
CCShapeId buildNonPlanarGuidedMorph() {
    static const std::vector<double> A = circlePoly(4.0, 32);
    static const std::vector<double> B = circlePoly(2.0, 32);
    std::vector<double> spine, guide;
    const int NS = 24;
    const double R = 30.0, pitch = 20.0;  // helix: non-planar (z rises while x,y circle)
    for (int k = 0; k <= NS; ++k) {
        const double t = static_cast<double>(k) / NS;
        const double th = kPi * t;  // half turn
        spine.push_back(R * std::cos(th));
        spine.push_back(R * std::sin(th));
        spine.push_back(pitch * t);
        // Guide offset radially outward from the helix (keeps a positive splay everywhere).
        guide.push_back((R + 8.0) * std::cos(th));
        guide.push_back((R + 8.0) * std::sin(th));
        guide.push_back(pitch * t);
    }
    return cc_variable_sweep(A.data(), 32, B.data(), 32, spine.data(), NS + 1, guide.data(), NS + 1);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT VARIABLE-SECTION sweep parity (moat-vsweep, through the cc_* facade)\n");

    const std::vector<OpCase> cases = {
        // 1) Circle→circle straight = truncated cone: deflection-bounded parity, watertight.
        {"vsweep circle->circle straight", &buildCircleToCircleStraight, /*exact*/ false, 0.02, 12.0},
        // 2) Constant square straight = prism: EXACT (both engines a box).
        {"vsweep constant-square straight", &buildConstantSquareStraight, /*exact*/ true, 0.02, 10.0},
        // 3) Guide-scaled square = frustum: near-exact (both a linear ruled frustum).
        {"vsweep guide-scaled square", &buildGuideScaledSquare, /*exact*/ true, 0.02, 10.0},
        // 4b) Coupled morph×scale straight (M6-breadth-19): the densified native tube tracks
        //     the morph×scale cross-term ⇒ deflection-bounded parity vs OCCT MakePipeShell.
        {"vsweep coupled morph×scale straight", &buildCoupledMorphScaleStraight, /*exact*/ false, 0.02, 12.0},
        // 4) Circle→circle smooth-arc morph: deflection-bounded parity, watertight.
        {"vsweep circle->circle arc", &buildCircleToCircleArc, /*exact*/ false, 0.02, 40.0},
    };

    for (const OpCase& s : cases) runNativeOp(s);

    runFallbackOp("vsweep non-planar guided morph", &buildNonPlanarGuidedMorph,
                  "non-planar (helical) guided morph — OCCT MakePipeShell corrected-Frenet");

    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
