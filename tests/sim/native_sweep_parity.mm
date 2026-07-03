// SPDX-License-Identifier: Apache-2.0
//
// native_sweep_parity.mm — native-vs-OCCT parity for the SWEPT solid (Phase 4 #4b,
// Tier C `native-construction`). Gate 2 of the two-gate model in
// openspec/NATIVE-REWRITE.md: on the iOS simulator (OCCT linked), the native
// cc_solid_sweep result is compared against the OCCT oracle (BRepOffsetAPI_MakePipe)
// at sampled inputs through the SAME cc_* facade.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default)
//   cc_set_engine(1)  → NativeEngine (native constant-frame sweep — construct/sweep.h; falls
//                       through to OCCT for the deferred cases)
//
// The switch is the ADDITIVE facade toggle cc_set_engine / cc_active_engine; DEFAULT
// stays OCCT. Every id is released; the harness restores cc_set_engine(0) before exit.
//
// ── NATIVE cases (each MUST take the native constant-frame sweep path; cc_active_engine==1) ─
//   1. STRAIGHT SWEEP  — cc_solid_sweep: a 4×4 square profile swept along a STRAIGHT
//      3D path. The transported frame is constant ⇒ a directional prism of the
//      profile along the path vector. Volume EXACT (profile area × path length) →
//      EXACT parity (vol/area/centroid rel 0, identical bbox, 6 faces watertight).
//   2. SMOOTH-ARC SWEEP — cc_solid_sweep: the SAME square swept along a smooth
//      quarter-arc polyline (radius ≫ profile half-extent, so no self-intersection).
//      The constant frame translates the section along the arc (matching OCCT MakePipe's
//      planar corrected-Frenet law); the ruled-band tube is compared
//      DEFLECTION-BOUNDED (curved), watertight, native F a k≥1 multiple of OCCT F.
//
// ── DEFERRED cases (native returns NULL → forwards to OCCT, tagged [fallback]) ─────
//   D1. TWISTED SWEEP     — cc_twisted_sweep with a REAL twist (π/2) + scale: the
//       native builder models only the plain (no-twist) sweep, so it returns NULL and
//       the NativeEngine forwards to the OCCT twisted_sweep oracle (ThruSections).
//   D2. GUIDED SWEEP      — cc_guided_sweep: a HARD pipe-shell/guide case, left OCCT
//       fallthrough in the engine glue (labelled, verified).
//   D3. LOFT ALONG RAIL   — cc_loft_along_rail: likewise a pipe-shell/guide case,
//       OCCT fallthrough.
//   Each deferred case asserts NativeEngine stays active (cc_active_engine==1) and the
//   native (delegated) result EQUALS the OCCT oracle — a verified fall-through, not a
//   fake and not a native interception.
//
// Output: [NSWEEP] PASS/FAIL lines with per-op deltas + a native/fallback tag, then a
// summary. On run-sim-suite.sh's SKIP list (own main()).
//
// Build: scripts/run-sim-native-sweep.sh — compiles this harness + the whole facade/
// core/engine (NativeEngine + OCCT adapter) + src/native/**, links OCCT, spawns on a
// booted simulator.

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

void record(bool ok, const char* label, const char* detail) {
    if (ok) {
        ++g_passed;
        std::printf("[NSWEEP] PASS  %-32s %s\n", label, detail);
    } else {
        ++g_failed;
        std::printf("[NSWEEP] FAIL  %-32s %s\n", label, detail);
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

int subCount(CCShapeId id, int kind) {
    int* ids = nullptr;
    const int n = cc_subshape_ids(id, kind, &ids);
    cc_ints_free(ids);
    return n;
}

using Builder = CCShapeId (*)();

struct OpCase {
    const char* name;
    Builder build;
    bool planar;      // true → exact tol; false → deflection-scaled tol (curved tube)
    double deflection;
};

// Compare the native build against the OCCT build; emit mass / bbox / faces /
// tessellate lines, each tagged [native].
void runNativeOp(const OpCase& s) {
    char detail[512];
    const double volRelTol = s.planar ? 1e-6 : 5e-2;
    const double areaRelTol = s.planar ? 1e-6 : 5e-2;
    const double linTol = s.planar ? 1e-6 : 5.0 * s.deflection;

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
    const int oF = subCount(occtId, 2);

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
    const int nF = subCount(natId, 2);
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
        "[native] vol o=%.6g n=%.6g rel=%.2e | area rel=%.2e | centroidΔ=%.2e (tol v=%.0e c=%.0e)",
        oM.volume, nM.volume, volRel, areaRel, cMax, volRelTol, linTol);
    record(massOk, (std::string(s.name) + " mass").c_str(), detail);

    // ── bounding boxes ───────────────────────────────────────────────────────────
    const double bbDelta = maxBBoxCornerDelta(oBB, nBB);
    const bool bbOk = (oBBok == 1) && (nBBok == 1) && (bbDelta < linTol);
    std::snprintf(detail, sizeof detail, "[native] maxCornerΔ=%.2e (tol=%.0e)", bbDelta, linTol);
    record(bbOk, (std::string(s.name) + " bbox").c_str(), detail);

    // ── FACE counts vs the OCCT oracle ───────────────────────────────────────────
    // Sweep: native emits one ruled band per (profile edge × spine segment) + 2 caps.
    // For the STRAIGHT prism native F equals OCCT F (k=1). For the multi-segment arc
    // the two engines tile the swept side into a DIFFERENT number of patches — native
    // one band per (edge × segment), OCCT one face per profile edge that may be split
    // or merged along the spine — a REPRESENTATIONAL difference (the SOLID is
    // geometrically identical, verified by mass/bbox/tessellate). We therefore accept
    // any consistent tiling for the curved case: both positive, one a multiple of the
    // other. The straight case is held to the strict k=1 identity.
    bool facesOk;
    if (s.planar) {
        facesOk = (oF > 0) && (nF > 0) && (nF == oF);  // exact prism → identical tiling
    } else {
        const int hi = std::max(oF, nF), lo = std::min(oF, nF);
        facesOk = (oF > 0) && (nF > 0) && (lo > 0) && (hi % lo == 0);
    }
    std::snprintf(detail, sizeof detail, "[native] F o=%d n=%d", oF, nF);
    record(facesOk, (std::string(s.name) + " faces").c_str(), detail);

    // ── native tessellation: WATERTIGHT + self-consistent bbox / volume ──────────
    double meshBB[6] = {0};
    const bool haveMesh = nMesh.triangleCount > 0 && meshBBox(nMesh, meshBB);
    const bool wt = haveMesh && meshWatertight(nMesh);
    const double meshVol = haveMesh ? meshVolume(nMesh) : 0.0;
    const double meshVolRel =
        (haveMesh && nM.volume > 0.0) ? std::fabs(meshVol - nM.volume) / nM.volume : 1.0;
    const double meshBBDelta = haveMesh ? maxBBoxCornerDelta(nBB, meshBB) : 1.0;
    const bool tessOk = haveMesh && wt && meshVolRel < volRelTol && meshBBDelta < linTol;
    std::snprintf(detail, sizeof detail,
                  "[native] watertight=%d tris=%d meshVolRel=%.2e bboxΔ=%.2e", wt ? 1 : 0,
                  nMesh.triangleCount, meshVolRel, meshBBDelta);
    record(tessOk, (std::string(s.name) + " tessellate").c_str(), detail);

    cc_mesh_free(nMesh);
    cc_set_engine(0);
    cc_shape_release(natId);
    cc_shape_release(occtId);
}

// A deferred case: native returns NULL → forwards to OCCT; the native (delegated)
// result must equal the OCCT oracle (not faked, not intercepted). Tagged [fallback].
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

// 1) STRAIGHT SWEEP: a 4×4 square (centred, area 16) swept along a straight path of
//    length 10 in an arbitrary 3D direction. Volume 160, EXACT (directional prism).
CCShapeId buildStraightSweep() {
    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double L = 10.0;
    const double d = 1.0 / std::sqrt(3.0);
    const double path[] = {0, 0, 0, d * L, d * L, d * L};
    return cc_solid_sweep(prof, 4, path, 2);
}

// 2) SMOOTH-ARC SWEEP: the SAME 4×4 square swept along a quarter-arc of radius R=20
//    in the XZ plane, sampled as a 24-segment polyline. R (20) ≫ the profile
//    circumradius (2√2 ≈ 2.83), so no self-intersection — the native constant-frame sweep builds
//    a watertight ruled tube. Passed IDENTICALLY to both engines (both approximate the
//    smooth arc from the same polyline), so their volumes agree within the deflection
//    bound. Volume ≈ profile_area × arc_length = 16 × (π/2·20) ≈ 502.65 (Pappus).
CCShapeId buildSmoothArcSweep() {
    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double R = 20.0;
    const int N = 24;
    std::vector<double> path;
    path.reserve(static_cast<std::size_t>(N + 1) * 3);
    for (int k = 0; k <= N; ++k) {
        const double th = (kPi / 2.0) * k / N;
        path.push_back(R * std::cos(th));
        path.push_back(0.0);
        path.push_back(R * std::sin(th));
    }
    return cc_solid_sweep(prof, 4, path.data(), N + 1);
}

// D1) TWISTED SWEEP (deferred): a 4×4 square swept straight 10 up with a REAL π/2
//     twist and 0.5 end-scale. Native returns NULL → OCCT twisted_sweep oracle.
CCShapeId buildTwistedSweepDeferred() {
    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double path[] = {0, 0, 0, 0, 0, 10};
    return cc_twisted_sweep(prof, 4, path, 2, kPi / 2.0, 0.5);
}

// D2) GUIDED SWEEP (deferred): a square swept along a straight path guided by a second
//     polyline. Pipe-shell/guide case — always OCCT fallthrough in the engine glue.
CCShapeId buildGuidedSweepDeferred() {
    const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double path[] = {0, 0, 0, 0, 0, 10};
    const double guide[] = {3, 0, 0, 5, 0, 10};  // a guide rail beside the spine
    return cc_guided_sweep(prof, 4, path, 2, guide, 2);
}

// D3) LOFT ALONG RAIL (deferred): two square sections swept along a straight rail.
//     Pipe-shell/guide case — OCCT fallthrough.
CCShapeId buildLoftAlongRailDeferred() {
    const double rail[] = {0, 0, 0, 0, 0, 10};
    const double a[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double b[] = {-1, -1, 1, -1, 1, 1, -1, 1};
    return cc_loft_along_rail(rail, 2, a, 4, b, 4);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT SWEPT-solid parity (Tier C #4b, through the cc_* facade)\n");

    const std::vector<OpCase> cases = {
        // 1) Straight sweep: planar directional prism → EXACT. Volume 160, 6 faces.
        {"sweep straight-path", &buildStraightSweep, /*planar*/ true, /*defl*/ 0.02},
        // 2) Smooth-arc sweep: constant-frame ruled tube → oracle-matched (fp precision), watertight.
        {"sweep smooth-arc-path", &buildSmoothArcSweep, /*planar*/ false, /*defl*/ 0.05},
    };

    for (const OpCase& s : cases) runNativeOp(s);

    // Intentionally-deferred cases: native returns NULL → forwards to OCCT.
    runFallbackOp("twisted_sweep real-twist", &buildTwistedSweepDeferred,
                  "real twist/scale (plain constant-frame sweep only) — Tier C");
    runFallbackOp("guided_sweep", &buildGuidedSweepDeferred,
                  "pipe-shell guide case — Tier C, OCCT fallthrough");
    runFallbackOp("loft_along_rail", &buildLoftAlongRailDeferred,
                  "pipe-shell rail case — Tier C, OCCT fallthrough");

    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
