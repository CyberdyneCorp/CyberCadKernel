// SPDX-License-Identifier: Apache-2.0
//
// native_loft_parity.mm — native-vs-OCCT parity for the 2-section RULED loft
// (Phase 4 #4b, Tier B `native-construction`). Gate 2 of the two-gate model in
// openspec/NATIVE-REWRITE.md: on the iOS simulator (OCCT linked), the native
// cc_solid_loft / cc_solid_loft_wires result is compared against the OCCT oracle
// (BRepOffsetAPI_ThruSections, ruled) at sampled inputs through the SAME cc_* facade.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default)
//   cc_set_engine(1)  → NativeEngine (native ruled loft — loft.h; falls through to
//                       OCCT for the deferred sub-cases)
//
// The switch is the ADDITIVE facade toggle cc_set_engine / cc_active_engine; DEFAULT
// stays OCCT. Every id is released; the harness restores cc_set_engine(0) before exit.
//
// ── NATIVE cases (each MUST take the native ruled-loft path; cc_active_engine==1) ──
//   1. SQUARE FRUSTUM   — cc_solid_loft: 4×4 bottom @z=0 → 2×2 top @z=6 (both centred)
//      → a square frustum. Planar → EXACT parity (vol 56, faces 6).
//   2. HEX PRISM        — cc_solid_loft: a regular hexagon bottom → same hexagon top
//      @z=3 (equal) → a straight hex prism. Planar → EXACT (faces 8 = 6 sides + 2 caps).
//   3. TRIANGLE PRISM   — cc_solid_loft_wires: a triangle @z=0 → same triangle @z=3
//      (two parallel planar 3D wires). Planar → EXACT (vol area·3, faces 5).
//   4. ROTATED SQUARE   — cc_solid_loft: 2×2 bottom → the same square rotated 45° at
//      z=4 (an antiprism-like TWISTED ruled skin). The side faces are truly bilinear;
//      compared deflection-bounded (curved), still watertight, faces 6.
//   5. MISMATCHED 4→8   — cc_solid_loft (T1): 4×4 bottom (4 pts) @z=0 → 2×2 top sampled
//      at 8 points @z=6. Counts differ (4 vs 8); the native builder makes them
//      compatible by resampling both loops at the union of their arc-length params
//      (geometry-preserving collinear insertion). The result is the SAME square frustum
//      as case 1 → EXACT parity (vol 56, planar) vs the OCCT ThruSections oracle.
//
// ── DEFERRED sub-case (native returns NULL / fails self-verify → OCCT, [fallback]) ──
//   D. MISMATCHED 4→3   — cc_solid_loft: 4-gon bottom → 3-gon top. The T1 correspondence
//      builds a candidate, but the resampled cap carries an ASYMMETRIC collinear vertex
//      the native mesher cannot close watertight, so the engine self-verify DISCARDS it
//      and forwards to the OCCT ThruSections oracle — the native result must equal the
//      OCCT oracle (delegated, not faked).
//
// Output: [NLOFT] PASS/FAIL lines with per-op deltas + a native/fallback tag, then a
// summary. On run-sim-suite.sh's SKIP list (own main()).
//
// Build: scripts/run-sim-native-loft.sh — compiles this harness + the whole facade/
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
        std::printf("[NLOFT] PASS  %-30s %s\n", label, detail);
    } else {
        ++g_failed;
        std::printf("[NLOFT] FAIL  %-30s %s\n", label, detail);
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
    bool planar;      // true → exact tol; false → deflection-scaled tol (twisted skin)
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
    // Ruled loft: native emits one side face per corresponding edge pair + 2 caps.
    // OCCT ThruSections (ruled) between two n-gons yields the same n side faces + 2
    // caps, so native F is a positive integer multiple of OCCT F (k=1 when tilings
    // match; k>1 would signal a future patch split — none here).
    const bool facesOk = (oF > 0) && (nF > 0) && (nF % oF == 0) && (nF >= oF);
    std::snprintf(detail, sizeof detail, "[native] F o=%d n=%d (n=%d×o)", oF, nF,
                  oF > 0 ? nF / oF : 0);
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

// A deferred sub-case: native returns NULL → forwards to OCCT; the native result
// must equal the OCCT oracle (delegated, not faked). Tagged [fallback].
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

// 1) SQUARE FRUSTUM: 4×4 bottom @z=0 → 2×2 top @z=6, both centred. Volume 56.
CCShapeId buildSquareFrustum() {
    const double bot[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double top[] = {-1, -1, 1, -1, 1, 1, -1, 1};
    return cc_solid_loft(bot, 4, top, 4, 6.0);
}

// 2) HEX PRISM: a regular hexagon (r=3) bottom → the SAME hexagon top @z=3.
CCShapeId buildHexPrism() {
    double hex[12];
    for (int i = 0; i < 6; ++i) {
        hex[i * 2] = 3.0 * std::cos(i * kPi / 3.0);
        hex[i * 2 + 1] = 3.0 * std::sin(i * kPi / 3.0);
    }
    return cc_solid_loft(hex, 6, hex, 6, 3.0);
}

// 3) TRIANGLE PRISM (wires): a triangle @z=0 → the same triangle @z=3.
CCShapeId buildTriangleWiresPrism() {
    const double a[] = {0, 0, 0, 4, 0, 0, 2, 3, 0};
    const double b[] = {0, 0, 3, 4, 0, 3, 2, 3, 3};
    return cc_solid_loft_wires(a, 3, b, 3);
}

// 4) ROTATED SQUARE (twisted ruled skin): 2×2 bottom → same square rotated 45° @z=4.
CCShapeId buildRotatedSquareTwist() {
    const double bot[] = {-1, -1, 1, -1, 1, 1, -1, 1};
    const double r = std::sqrt(2.0);
    const double top[] = {r, 0, 0, r, -r, 0, 0, -r};  // 45°-rotated corners on axes
    return cc_solid_loft(bot, 4, top, 4, 4.0);
}

// 5) MISMATCHED 4→8 (T1): 4×4 bottom (4 pts) → 2×2 top sampled at 8 points @z=6. The
// arc-length correspondence resamples both loops to a common count, inserting collinear
// points, so the loft is the SAME square frustum as case 1. Planar → EXACT (vol 56).
CCShapeId buildMismatchedFrustum4to8() {
    const double bot[] = {-2, -2, 2, -2, 2, 2, -2, 2};                          // 4×4, 4 pts
    const double top[] = {-1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0};  // 2×2, 8 pts
    return cc_solid_loft(bot, 4, top, 8, 6.0);
}

// DEFERRED: MISMATCHED 4→3. The T1 correspondence builds a candidate, but its resampled
// cap carries an asymmetric collinear vertex the mesher cannot close watertight, so the
// engine self-verify discards it and forwards to OCCT.
CCShapeId buildMismatchedCountsDeferred() {
    const double bot[] = {0, 0, 4, 0, 4, 4, 0, 4};  // 4-gon
    const double top[] = {1, 1, 3, 1, 2, 3};        // 3-gon (centred over the bottom)
    return cc_solid_loft(bot, 4, top, 3, 5.0);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT 2-section RULED loft parity (Tier B #4b, through the cc_* facade)\n");

    const std::vector<OpCase> cases = {
        // 1) Square frustum: planar → EXACT. Volume 56, 6 faces.
        {"loft square-frustum", &buildSquareFrustum, /*planar*/ true, /*defl*/ 0.02},
        // 2) Hex prism: planar → EXACT. 8 faces.
        {"loft hex-prism", &buildHexPrism, /*planar*/ true, /*defl*/ 0.05},
        // 3) Triangle prism from two 3D wires: planar → EXACT. 5 faces.
        {"loft_wires triangle-prism", &buildTriangleWiresPrism, /*planar*/ true, /*defl*/ 0.05},
        // 4) Rotated-square twist: truly bilinear side faces → deflection-bounded.
        {"loft rotated-square-twist", &buildRotatedSquareTwist, /*planar*/ false, /*defl*/ 0.01},
        // 5) T1 mismatched 4→8 frustum: geometry-preserving correspondence → EXACT.
        {"loft mismatched-4to8", &buildMismatchedFrustum4to8, /*planar*/ true, /*defl*/ 0.02},
    };

    for (const OpCase& s : cases) runNativeOp(s);

    // Intentionally-deferred sub-case: the T1 correspondence builds a candidate but its
    // resampled cap can't close watertight → engine self-verify forwards to OCCT.
    runFallbackOp("loft mismatched-4to3", &buildMismatchedCountsDeferred,
                  "resampled cap not robustly watertight → self-verify declines to OCCT");

    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
