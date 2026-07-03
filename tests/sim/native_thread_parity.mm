// SPDX-License-Identifier: Apache-2.0
//
// native_thread_parity.mm — native-vs-OCCT parity for the THREADS / TAPERED-SHANK ops
// (Phase 4 #4b, Tier D `native-construction`). Gate 2 of the two-gate model in
// openspec/NATIVE-REWRITE.md: on the iOS simulator (OCCT linked), the native result is
// compared against the OCCT oracle at sampled inputs through the SAME cc_* facade.
//
//   cc_set_engine(0)  → OCCT engine (the oracle / default)
//   cc_set_engine(1)  → NativeEngine (native tapered_shank reusing the parity-verified
//                       revolve; native helical/tapered thread — a radial-V ridge tiled
//                       into ruled bands + caps, self-verified robustly watertight and
//                       kept native; a genuinely FINE-PITCH thread whose turns fold
//                       through each other still fails the self-verify → OCCT fallthrough)
//
// The switch is the ADDITIVE facade toggle cc_set_engine / cc_active_engine; DEFAULT
// stays OCCT. Every id is released; the harness restores cc_set_engine(0) before exit.
//
// ── NATIVE cases (MUST take the native path; cc_active_engine==1) ─────────────────
//   1. TAPERED SHANK — cc_tapered_shank: a pointed shank silhouette (cone tip →
//      full-radius cylinder → head disk) revolved 360° about Z by REUSING the native
//      revolve (build_revolution). Volume ⅓π r²·taperHeight + π r²·fullHeight. The tip
//      is a TRUE on-axis apex (the revolve collapses its angular copies to one shared
//      vertex → robustly watertight at every deflection), so this is genuinely native.
//      Compared DEFLECTION-BOUNDED (curved surface of revolution) vs BRepPrimAPI_MakeRevol.
//   2. HELICAL THREAD — cc_helical_thread (cylindrical, constant pitch-line radius) and
//   3. TAPERED THREAD — cc_tapered_thread (conical, pitch-line radius tapers tip→head).
//      The native builder sweeps a V/triangular section RADIALLY along the pitch-line
//      helix (construct/thread.h build_thread) and tiles the three V edges into bilinear
//      ruled bands + two planar caps → a thin helical RIDGE, the same body OCCT sweeps
//      with BRepOffsetAPI_MakePipeShell (aux-spine radial law). The ruled-band/cap seams
//      weld watertight via the mesher's canonical shared-edge points, so the engine
//      self-verifies the solid robustly watertight and KEEPS IT NATIVE (cc_active_engine
//      ==1). Each is checked exactly like the shank — DEFLECTION-BOUNDED vs the OCCT
//      MakePipeShell oracle: vol/area/centroid within tol, bbox, subshape counts a proper
//      band+cap tiling, native mesh watertight + mesh-volume ≈ B-rep volume. (The native
//      body tiles STRAIGHT chords between helix stations where OCCT fits a smooth BSpline
//      spine, so at the harness's samplesPerTurn the two agree to a chord-vs-arc
//      discretization bound, ~2-3% on volume — well inside the 5% curved-body gate.)
//
// ── FALLBACK case (native ATTEMPTS the radial-V tiling; the engine's robustlyWatertight
//    self-verify DEFERS → forwards to OCCT, tagged [fallback]) ─────────────────────
//   F1. FINE-PITCH HELICAL THREAD — cc_helical_thread with a pitch so small (relative to
//       the V depth) that adjacent turns fold through one another: the swept ridge
//       self-intersects, so the native mesh is non-manifold no matter how vertices weld
//       and robustlyWatertight rejects it across the deflection ladder. NativeEngine
//       falls through to the OCCT MakePipeShell oracle — labelled, verified, never faked.
//       This case asserts NativeEngine stays ACTIVE (cc_active_engine==1) and the native
//       (delegated) result EQUALS the OCCT oracle (vol rel ~0) — a verified fall-through,
//       not a fake and not a native interception.
//
// Output: [NTHREAD] PASS/FAIL lines with per-op deltas + a native/fallback tag, then a
// summary. On run-sim-suite.sh's SKIP list (own main()).
//
// Build: scripts/run-sim-native-thread.sh — compiles this harness + the whole facade/
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
        std::printf("[NTHREAD] PASS  %-34s %s\n", label, detail);
    } else {
        ++g_failed;
        std::printf("[NTHREAD] FAIL  %-34s %s\n", label, detail);
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

// How the native builder decomposes the body's faces, so the subshape check knows the
// honest count invariant to assert against the OCCT oracle:
//   Planar    — a prism/planar body: native F == OCCT F exactly.
//   Revolve   — a full-turn surface of revolution: each lateral surface is tiled into 3
//               non-periodic 120° angular patches, so native F is a positive multiple of 3.
//   ThreadRidge — a helical V ridge: 3 ruled bands per helix span + 2 planar end caps, so
//               native F = 3·spans + 2, i.e. (nF − 2) is a positive multiple of 3 and
//               nF > OCCT's (OCCT emits a handful of swept/periodic faces).
enum class FaceTopo { Planar, Revolve, ThreadRidge };

struct OpCase {
    const char* name;
    Builder build;
    bool planar;      // true → exact tol; false → deflection-scaled tol (curved)
    double deflection;
    FaceTopo faceTopo = FaceTopo::Revolve;
};

// Compare a NATIVE build against the OCCT build; emit mass / bbox / faces / subshape /
// tessellate lines, each tagged [native]. Used for tapered_shank (genuinely native) and
// — if a future mesher makes a thread self-verify watertight — for that thread too.
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
    const int oF = subCount(occtId, 2);   // faces
    const int oE = subCount(occtId, 1);   // edges
    const int oV = subCount(occtId, 0);   // vertices

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
    const int nE = subCount(natId, 1);
    const int nV = subCount(natId, 0);
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

    // ── subshape counts vs the OCCT oracle ───────────────────────────────────────
    // A surface of revolution: the native builder tiles each LATERAL surface of the
    // full-turn revolve into 3 non-periodic 120° angular patches (periodic-face
    // construction deferred — a documented REPRESENTATIONAL difference; the SOLID is
    // geometrically identical, which the mass / bbox / watertight checks confirm). So
    // native F = 3 × (native lateral-surface count) and V/E differ from OCCT's.
    //
    // The oracle's surface count need NOT equal native's: for the tapered shank the
    // native builder tapers to a TRUE on-axis apex (one shared vertex → robustly
    // watertight), whereas the OCCT oracle uses a near-point tip disk (tipR ≈ 0.02·r,
    // only to dodge a zero-radius MakeRevol degeneracy) that adds ONE extra degenerate
    // bottom-cone face. So OCCT emits 4 lateral faces where native emits 3 — native is
    // topologically CLEANER, not wrong. Asserting nF divisible by OCCT's artifact-
    // inflated count would force native to re-introduce that non-watertight tip sliver.
    // The honest, geometry-preserving invariant is therefore: both counts positive, and
    // native F is a proper 3-span revolve (nF a positive multiple of 3, nF ≥ 3). The
    // planar case (never span-split) still requires an exact match.
    // A helical thread ridge (ThreadRidge) tiles 3 ruled bands per helix span + 2 planar
    // end caps → native F = 3·spans + 2, so (nF − 2) is a positive multiple of 3 and nF
    // exceeds OCCT's small swept/periodic face count. A surface of revolution (Revolve)
    // tiles each lateral surface into 3 angular patches → nF a positive multiple of 3.
    // A planar body matches OCCT exactly.
    bool facesOk;
    switch (s.faceTopo) {
        case FaceTopo::Planar:
            facesOk = (oF > 0) && (nF > 0) && (nF == oF);
            break;
        case FaceTopo::ThreadRidge:
            facesOk = (oF > 0) && (nF > oF) && ((nF - 2) > 0) && ((nF - 2) % 3 == 0);
            break;
        case FaceTopo::Revolve:
        default:
            facesOk = (oF > 0) && (nF > 0) && (nF % 3 == 0);  // 3 angular patches / surface
            break;
    }
    std::snprintf(detail, sizeof detail, "[native] F o=%d n=%d | E o=%d n=%d | V o=%d n=%d", oF, nF,
                  oE, nE, oV, nV);
    record(facesOk && nE > 0 && nV > 0, (std::string(s.name) + " subshapes").c_str(), detail);

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

// A FINE-PITCH thread op: the native builder ATTEMPTS the radial-V tiling, but the swept
// turns self-intersect so the engine self-verify (robustlyWatertight across a deflection
// ladder) rejects the non-manifold body and NativeEngine forwards to OCCT (labelled,
// verified, never faked). This routine tolerates BOTH outcomes:
//   * fallback (expected for the fine-pitch case): native active AND the native
//     (delegated) result EQUALS the OCCT oracle to fp precision — a verified
//     fall-through, not an interception.
//   * native (future mesher): if the thread self-verifies watertight and the native and
//     OCCT bodies diverge only within a deflection bound, we accept that as native parity
//     instead — the thread has lit up native with no engine change.
void runThreadOp(const char* name, Builder build, const char* why, double deflection) {
    char detail[512];

    cc_set_engine(0);
    const CCShapeId occtId = build();
    const CCMassProps oM = occtId ? cc_mass_properties(occtId) : CCMassProps{0, 0, 0, 0, 0, 0};
    double oBB[6] = {0};
    if (occtId) cc_bounding_box(occtId, oBB);

    cc_set_engine(1);
    const int activeIsNative = cc_active_engine();
    const CCShapeId natId = build();
    const CCMassProps nM = natId ? cc_mass_properties(natId) : CCMassProps{0, 0, 0, 0, 0, 0};

    const bool bothValid = (occtId != 0) && (natId != 0) && (oM.valid != 0) && (nM.valid != 0);
    const double volRel = (bothValid && oM.volume > 0.0)
                              ? std::fabs(nM.volume - oM.volume) / oM.volume
                              : 1.0;

    // Fallback path (expected): the delegated result is byte-for-byte the OCCT oracle.
    const bool isFallback = (activeIsNative == 1) && bothValid && (oM.volume > 0.0) &&
                            volRel < 1e-9;
    if (isFallback) {
        std::snprintf(detail, sizeof detail,
                      "[fallback] %s — native active=%d vol o=%.6g n=%.6g rel=%.2e "
                      "(self-verify deferred → delegated to OCCT)",
                      why, activeIsNative, oM.volume, nM.volume, volRel);
        record(true, name, detail);
    } else {
        // Native path (only if a future mesher welds the seams): accept a
        // deflection-bounded native body that meshes watertight and matches the oracle
        // volume within tol. Anything else is a genuine FAIL.
        const CCMesh nMesh = natId ? cc_tessellate(natId, deflection) : CCMesh{nullptr, 0, nullptr, 0};
        const bool wt = meshWatertight(nMesh);
        const bool nativeParity = (activeIsNative == 1) && bothValid && (oM.volume > 0.0) &&
                                  wt && volRel < 5e-2;
        std::snprintf(detail, sizeof detail,
                      "[native?] %s — native active=%d watertight=%d vol o=%.6g n=%.6g rel=%.2e "
                      "(thread lit up native — deflection-bounded parity)",
                      why, activeIsNative, wt ? 1 : 0, oM.volume, nM.volume, volRel);
        record(nativeParity, name, detail);
        if (nMesh.triangleCount > 0 || nMesh.vertexCount > 0) cc_mesh_free(nMesh);
    }

    cc_set_engine(0);
    if (natId) cc_shape_release(natId);
    if (occtId) cc_shape_release(occtId);
}

// ── per-op builders (identical inputs to both engines) ──────────────────────────

// 1) TAPERED SHANK (native): radius 5, fullHeight 20, taperHeight 10, pointsPerMM 1.
//    Volume ⅓π·25·10 + π·25·20 = 250π/3 + 500π ≈ 261.80 + 1570.80 ≈ 1832.60 (exact
//    B-rep; deflection-bounded once meshed). A pointed tip + full-radius shank + head.
CCShapeId buildTaperedShank() {
    return cc_tapered_shank(/*radiusMM=*/5.0, /*fullHeightMM=*/20.0, /*taperHeightMM=*/10.0,
                            /*pointsPerMM=*/1.0);
}

// 2) HELICAL THREAD (native): a well-separated cylindrical thread — major radius 5,
//    pitch 2, 4 turns, depth 1, 60° ISO flank, pointsPerMM 1, samplesPerTurn 16. The
//    pitch (2) comfortably clears the V's axial base (2·halfBase = 1.15 < pitch), so the
//    turns do NOT fold through each other; the radial-V ridge self-verifies robustly
//    watertight and stays native. A thin helical V ridge (vol ≈ 68 native / 70 OCCT).
CCShapeId buildHelicalThread() {
    return cc_helical_thread(/*majorRadiusMM=*/5.0, /*pitchMM=*/2.0, /*turns=*/4.0,
                             /*depthMM=*/1.0, /*flankAngleDeg=*/60.0, /*pointsPerMM=*/1.0,
                             /*samplesPerTurn=*/16);
}

// 3) TAPERED THREAD (native): a conical thread — top radius 6, tip radius 4 (both clear
//    the axis after minus depth/2), pitch 2, 4 turns, depth 1, 60° flank, pointsPerMM 1,
//    samplesPerTurn 16. Same well-separated pitch → self-verifies native (vol ≈ 70).
CCShapeId buildTaperedThread() {
    return cc_tapered_thread(/*topRadiusMM=*/6.0, /*tipRadiusMM=*/4.0, /*pitchMM=*/2.0,
                             /*turns=*/4.0, /*depthMM=*/1.0, /*flankAngleDeg=*/60.0,
                             /*pointsPerMM=*/1.0, /*samplesPerTurn=*/16);
}

// F1) FINE-PITCH HELICAL THREAD (genuine OCCT fallthrough): major radius 5, pitch 0.3,
//     8 turns, depth 1, 60° flank, pointsPerMM 1, samplesPerTurn 16. The pitch (0.3) is
//     far smaller than the V depth (1) — adjacent turns fold through one another, so the
//     swept ridge SELF-INTERSECTS (a non-manifold mesh no matter how vertices weld).
//     robustlyWatertight rejects it across the deflection ladder and NativeEngine
//     delegates to the OCCT MakePipeShell oracle — labelled, verified, never faked.
CCShapeId buildFinePitchHelicalThread() {
    return cc_helical_thread(/*majorRadiusMM=*/5.0, /*pitchMM=*/0.3, /*turns=*/8.0,
                             /*depthMM=*/1.0, /*flankAngleDeg=*/60.0, /*pointsPerMM=*/1.0,
                             /*samplesPerTurn=*/16);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native-vs-OCCT THREADS / TAPERED-SHANK parity (Tier D #4b, through the cc_* "
                "facade)\n");

    const std::vector<OpCase> nativeCases = {
        // 1) Tapered shank: a curved surface of revolution → deflection-bounded parity,
        //    watertight, native F a k≥1 multiple of 3 (3 angular patches / lateral surface).
        {"tapered_shank r5/fh20/th10", &buildTaperedShank, /*planar*/ false, /*defl*/ 0.02,
         FaceTopo::Revolve},
        // 2) Helical thread (well-separated pitch): the radial-V ridge self-verifies
        //    robustly watertight → NATIVE. Deflection-bounded parity vs the OCCT
        //    MakePipeShell oracle (chord-vs-arc discretization ≈ 2-3% on volume).
        {"helical_thread mr5/p2/t4/d1", &buildHelicalThread, /*planar*/ false, /*defl*/ 0.02,
         FaceTopo::ThreadRidge},
        // 3) Tapered thread (well-separated pitch): same → NATIVE, deflection-bounded parity.
        {"tapered_thread top6/tip4/p2/t4", &buildTaperedThread, /*planar*/ false, /*defl*/ 0.02,
         FaceTopo::ThreadRidge},
    };

    for (const OpCase& s : nativeCases) runNativeOp(s);

    // Genuine FINE-PITCH fallback: a thread whose turns fold through each other →
    // self-intersecting ridge → robustlyWatertight rejects → verified OCCT fall-through
    // (this routine also accepts a native pass if a future mesher makes it watertight).
    runThreadOp("helical_thread FINE mr5/p0.3/t8/d1", &buildFinePitchHelicalThread,
                "fine-pitch helical sweep, turns self-intersect → self-verify defers — Tier D",
                /*deflection=*/0.02);

    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
