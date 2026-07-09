// SPDX-License-Identifier: Apache-2.0
//
// native_construct_tails_parity.mm — native-vs-OCCT parity for the M7 construct tails
// (MOAT moat-m7t-construct-tails, Gate 2): a REAL-twist cc_twisted_sweep (vs the OCCT
// ThruSections oracle) and a CURVED-rail cc_loft_along_rail (vs the OCCT MakePipeShell
// oracle), driven through the SAME cc_* facade under both engines.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default)
//   cc_set_engine(1)  → NativeEngine (native construct/sweep.h)
//
// Both native builders densify their spine/rail internally, but OCCT's twisted_sweep /
// loft_along_rail use exactly the path/rail stations they are handed. To compare like
// with like we feed BOTH engines a PRE-DENSIFIED path/rail (the same trick the landed
// smooth-arc sweep parity uses), so neither out-refines the other and both converge to
// the same ruled solid. We then compare volume / area / watertight / Euler χ=2 / bbox.
//
// ── NATIVE cases (each MUST run native; cc_active_engine()==1) ──────────────────────
//   1. TWISTED SWEEP (pure real twist)  — a square profile swept along a densified
//      straight spine with a π/2 twist, scaleEnd=1. Volume is area-preserving (→ area·L);
//      the native densified Frenet ruled tube matches OCCT's densified ThruSections.
//   2. CURVED-RAIL LOFT (smooth arc)    — a 32-gon section morphed along a densified
//      quarter-arc rail (R=20). Volume → Pappus polyArea·R·φ; native RMF tube matches
//      OCCT MakePipeShell.
//
// ── DEFERRED cases (native NULL / self-verify DISCARD → OCCT, tagged [fallback]) ─────
//   D1. TWIST + SCALE  — a twist COMBINED with a scale is not robustly weldable → the
//       engine self-verify discards the native candidate → OCCT twisted_sweep.
//   D2. TIGHT-KINK RAIL — a near-90° V rail does not weld → OCCT MakePipeShell.
//   Each deferred case asserts NativeEngine stays active and the delegated result EQUALS
//   the OCCT oracle (a verified fall-through, never a fake).
//
// Output: [NTAILS] PASS/FAIL lines + a summary. On run-sim-suite.sh's SKIP list (own
// main()). Build/run: scripts/run-sim-native-construct-tails.sh.

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

int g_passed = 0;
int g_failed = 0;

void record(bool ok, const char* label, const char* detail) {
    if (ok) {
        ++g_passed;
        std::printf("[NTAILS] PASS  %-34s %s\n", label, detail);
    } else {
        ++g_failed;
        std::printf("[NTAILS] FAIL  %-34s %s\n", label, detail);
    }
}

struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

// A mesh is watertight iff every undirected edge is shared by exactly two triangles.
bool meshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0) return false;
    std::unordered_map<std::uint64_t, int> edgeCount;
    auto key = [](int a, int b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = m.triangles[t * 3 + 0], j = m.triangles[t * 3 + 1], k = m.triangles[t * 3 + 2];
        ++edgeCount[key(i, j)];
        ++edgeCount[key(j, k)];
        ++edgeCount[key(k, i)];
    }
    for (const auto& [e, c] : edgeCount)
        if (c != 2) return false;
    return true;
}

// Euler characteristic of a closed triangle mesh: V − E + F. A single watertight
// genus-0 shell has χ = 2.
int meshEuler(const CCMesh& m) {
    std::unordered_map<std::uint64_t, int> edges;
    auto key = [](int a, int b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = m.triangles[t * 3 + 0], j = m.triangles[t * 3 + 1], k = m.triangles[t * 3 + 2];
        ++edges[key(i, j)];
        ++edges[key(j, k)];
        ++edges[key(k, i)];
    }
    return m.vertexCount - static_cast<int>(edges.size()) + m.triangleCount;
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

double maxBBoxCornerDelta(const double a[6], const double b[6]) {
    double d = 0.0;
    for (int k = 0; k < 6; ++k) d = std::max(d, std::fabs(a[k] - b[k]));
    return d;
}

using Builder = CCShapeId (*)();

// Compare the native build against the OCCT build (identical densified input). Emits
// mass / bbox / watertight+χ lines, each tagged [native].
void runNativeOp(const char* name, Builder build, double deflection) {
    char detail[512];
    const double volRelTol = 5e-2, areaRelTol = 5e-2, linTol = 10.0 * deflection;

    cc_set_engine(0);
    const CCShapeId occtId = build();
    if (occtId == 0) {
        std::snprintf(detail, sizeof detail, "[native] OCCT build failed: %s", cc_last_error());
        record(false, name, detail);
        return;
    }
    const CCMassProps oM = cc_mass_properties(occtId);
    double oBB[6] = {0};
    const int oBBok = cc_bounding_box(occtId, oBB);

    cc_set_engine(1);
    const int activeIsNative = cc_active_engine();
    const CCShapeId natId = build();
    if (natId == 0 || activeIsNative != 1) {
        std::snprintf(detail, sizeof detail, "[native] native build failed (active=%d): %s",
                      activeIsNative, cc_last_error());
        record(false, name, detail);
        cc_set_engine(0);
        cc_shape_release(occtId);
        return;
    }
    const CCMassProps nM = cc_mass_properties(natId);
    double nBB[6] = {0};
    const int nBBok = cc_bounding_box(natId, nBB);
    const CCMesh nMesh = cc_tessellate(natId, deflection);

    // ── mass properties (volume / area) ──────────────────────────────────────────
    bool massOk = (oM.valid != 0) && (nM.valid != 0) && (oM.volume > 0.0);
    const double volRel = massOk ? std::fabs(nM.volume - oM.volume) / oM.volume : 1.0;
    const double areaRel = (massOk && oM.area > 0.0) ? std::fabs(nM.area - oM.area) / oM.area : 1.0;
    massOk = massOk && volRel < volRelTol && areaRel < areaRelTol;
    std::snprintf(detail, sizeof detail,
                  "[native] vol o=%.5g n=%.5g rel=%.2e | area rel=%.2e (tol v=%.0e)", oM.volume,
                  nM.volume, volRel, areaRel, volRelTol);
    record(massOk, (std::string(name) + " mass").c_str(), detail);

    // ── bounding boxes ───────────────────────────────────────────────────────────
    const double bbDelta = maxBBoxCornerDelta(oBB, nBB);
    const bool bbOk = (oBBok == 1) && (nBBok == 1) && (bbDelta < linTol);
    std::snprintf(detail, sizeof detail, "[native] maxCornerΔ=%.2e (tol=%.0e)", bbDelta, linTol);
    record(bbOk, (std::string(name) + " bbox").c_str(), detail);

    // ── native tessellation: WATERTIGHT + Euler χ = 2 ────────────────────────────
    double meshBB[6] = {0};
    const bool haveMesh = nMesh.triangleCount > 0 && meshBBox(nMesh, meshBB);
    const bool wt = haveMesh && meshWatertight(nMesh);
    const int chi = haveMesh ? meshEuler(nMesh) : -99;
    const bool tessOk = haveMesh && wt && chi == 2;
    std::snprintf(detail, sizeof detail, "[native] watertight=%d χ=%d tris=%d", wt ? 1 : 0, chi,
                  nMesh.triangleCount);
    record(tessOk, (std::string(name) + " watertight+euler").c_str(), detail);

    cc_mesh_free(nMesh);
    cc_set_engine(0);
    cc_shape_release(natId);
    cc_shape_release(occtId);
}

// A deferred case: native returns NULL / self-verify discards → forwards to OCCT; the
// native (delegated) result must equal the OCCT oracle. Tagged [fallback].
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
    const double volRel =
        (bothValid && oM.volume > 0.0) ? std::fabs(nM.volume - oM.volume) / oM.volume : 1.0;
    const bool ok = (activeIsNative == 1) && bothValid && (oM.volume > 0.0) && volRel < 1e-9;
    std::snprintf(detail, sizeof detail,
                  "[fallback] %s — active=%d vol o=%.5g n=%.5g rel=%.2e (delegated to OCCT)", why,
                  activeIsNative, oM.volume, nM.volume, volRel);
    record(ok, name, detail);

    cc_set_engine(0);
    if (natId) cc_shape_release(natId);
    if (occtId) cc_shape_release(occtId);
}

// ── per-op builders (identical DENSIFIED inputs to both engines) ────────────────────

// 1) TWISTED SWEEP (pure real twist): a 4×4 square (area 16) swept along a straight
//    spine of length 10 PRE-DENSIFIED to 33 stations with a π/2 twist, scaleEnd=1. Both
//    engines build the same 32-band twisted ruled ThruSections; volume → area·L = 160.
CCShapeId buildTwistedSweepNative() {
    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const int N = 32;
    std::vector<double> path;
    for (int k = 0; k <= N; ++k) {
        path.push_back(0.0);
        path.push_back(0.0);
        path.push_back(10.0 * k / N);
    }
    return cc_twisted_sweep(prof, 4, path.data(), N + 1, kPi / 2.0, 1.0);
}

// 2) CURVED-RAIL LOFT (smooth arc): a 32-gon section (circumradius 3) morphed along a
//    quarter-arc rail (R=20) PRE-DENSIFIED to 33 stations. Both engines build the same
//    RMF/pipe-shell tube; volume → Pappus polyArea·R·(π/2).
CCShapeId buildCurvedRailLoftNative() {
    const int pn = 32;
    std::vector<double> circ;
    for (int i = 0; i < pn; ++i) {
        const double t = 2.0 * kPi * i / pn;
        circ.push_back(3.0 * std::cos(t));
        circ.push_back(3.0 * std::sin(t));
    }
    const int rn = 33;
    std::vector<double> rail;
    for (int k = 0; k < rn; ++k) {
        const double th = (kPi / 2.0) * k / (rn - 1);
        rail.push_back(20.0 * std::cos(th));
        rail.push_back(20.0 * std::sin(th));
        rail.push_back(0.0);
    }
    return cc_loft_along_rail(rail.data(), rn, circ.data(), pn, circ.data(), pn);
}

// D1) TWIST + SCALE (deferred): the same densified spine with a π/2 twist AND scaleEnd
//     0.5 — the twist+shrink saddle is not robustly weldable → engine self-verify
//     discards → OCCT twisted_sweep.
CCShapeId buildTwistScaleDeferred() {
    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const int N = 32;
    std::vector<double> path;
    for (int k = 0; k <= N; ++k) {
        path.push_back(0.0);
        path.push_back(0.0);
        path.push_back(10.0 * k / N);
    }
    return cc_twisted_sweep(prof, 4, path.data(), N + 1, kPi / 2.0, 0.5);
}

// D2) TIGHT-KINK RAIL (deferred): a near-90° V rail does not weld → OCCT MakePipeShell.
CCShapeId buildTightRailDeferred() {
    const double rail[] = {0, 0, 0, 3, 0, 5, 0, 0, 10};  // sharp V kink
    const double a[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double b[] = {-1, -1, 1, -1, 1, 1, -1, 1};
    return cc_loft_along_rail(rail, 3, a, 4, b, 4);
}

}  // namespace

int main() {
    EngineGuard guard;

    std::printf("── native-vs-OCCT M7 construct-tails parity (twisted_sweep + loft_along_rail)\n");

    runNativeOp("twisted_sweep real-twist", &buildTwistedSweepNative, /*defl*/ 0.01);
    runNativeOp("loft_along_rail curved-arc", &buildCurvedRailLoftNative, /*defl*/ 0.01);

    runFallbackOp("twisted_sweep twist+scale", &buildTwistScaleDeferred,
                  "twist+scale not robustly weldable — self-verify discard");
    runFallbackOp("loft_along_rail tight-kink", &buildTightRailDeferred,
                  "tight-kink rail does not weld — self-verify discard");

    cc_set_engine(0);
    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
