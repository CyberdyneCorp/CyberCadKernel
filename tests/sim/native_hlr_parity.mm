// SPDX-License-Identifier: Apache-2.0
//
// native_hlr_parity.mm — native-vs-OCCT HIDDEN-LINE-REMOVAL parity harness,
//                        driven THROUGH the cc_* facade (iOS simulator).
//
// MOAT M-GS GS1, GATE (b): the native orthographic HLR core (src/native/drafting/
// orthographic_hlr.h, OCCT-FREE) vs the OCCT HLRBRep_Algo / HLRBRep_HLRToShape
// oracle, for POLYHEDRAL solids. Like the sibling native_*_parity harnesses it
// exercises the SHIPPING PATH: it calls cc_hlr_project once with the OCCT engine
// active (the oracle) and once with the NativeEngine active (the OCCT-free core),
// and compares the two visible/hidden 2D drawing-plane segment sets.
//
//   cc_set_engine(0)  → OCCT engine  → HLRBRep_Algo oracle
//   cc_set_engine(1)  → NativeEngine → orthographic_hlr polyhedral core
//
// Both return CCDrawing in the SAME drawing-plane basis (right = normalize(viewDir
// × up), trueUp = right × viewDir), so the 2D coordinates are directly comparable.
//
// ── Solids (polyhedral; built identically under BOTH engines via cc_solid_extrude)
//   1. BOX            — 3×2 rectangle extruded 5. Convex: an isometric corner view
//                       is the classic 9 visible + 3 hidden.
//   2. TRIANGLE PRISM — triangle extruded 5. Convex.
//   3. L-PRISM        — a non-convex L polygon extruded 4. Self-occluding silhouette
//                       (a genuine hidden-edge test) with IDENTICAL topology under
//                       both engines (one extrude, no boolean), so the edge
//                       decomposition matches and the comparison is pure HLR.
//
// ── Comparison (native vs OCCT, per solid + view) ─────────────────────────────
//   * counts        : native visible/hidden segment COUNT vs the oracle. For the
//                     CONVEX box/prism this is exact (no collinear-edge ambiguity);
//                     for the L-prism the counts are reported and required equal too
//                     (same topology under both engines).
//   * total length  : Σ visible segment length and Σ hidden segment length match the
//                     oracle within a tight RELATIVE tolerance — no gaps/overlaps.
//   * partition     : every native segment lies (both endpoints) on an oracle segment
//                     of the SAME visible/hidden class within tolerance, AND vice
//                     versa — proving the visible/hidden PARTITION of the projected
//                     outline is identical as a labelled point set (robust to how
//                     each engine splits a straight edge, and the authoritative
//                     endpoint-position check).
//
// A DIVERGENCE on a polyhedral case is a REAL native HLR bug and is reported as a
// FAIL — the tolerance is never widened to paper over it.
//
// Output: [NHLR] PASS/FAIL lines, then "== N passed, M failed ==". Flushes stdout
// and std::_Exit (the trimmed static-OCCT build's teardown is not exit-clean — same
// rationale as the sibling harnesses; every id is released before exit).
//
// Build: scripts/run-sim-native-hlr.sh (compiles the whole kernel + OCCT incl. the
// TKHLR toolkit, spawns on a booted simulator). Carries its own main().

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

int g_passed = 0;
int g_failed = 0;

void record(bool ok, const std::string& label, const char* detail) {
    if (ok) {
        ++g_passed;
        std::printf("[NHLR] PASS  %-26s %s\n", label.c_str(), detail);
    } else {
        ++g_failed;
        std::printf("[NHLR] FAIL  %-26s %s\n", label.c_str(), detail);
    }
}

struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

// ── 2D segment geometry helpers ────────────────────────────────────────────────
double segLen(const CCDrawingSegment& s) {
    return std::hypot(s.bx - s.ax, s.by - s.ay);
}

double totalLen(const CCDrawingSegment* segs, int n) {
    double t = 0.0;
    for (int i = 0; i < n; ++i) t += segLen(segs[i]);
    return t;
}

// Distance from point (px,py) to segment s (2D).
double pointSegDist(double px, double py, const CCDrawingSegment& s) {
    const double vx = s.bx - s.ax, vy = s.by - s.ay;
    const double wx = px - s.ax, wy = py - s.ay;
    const double vv = vx * vx + vy * vy;
    double t = vv > 0.0 ? (wx * vx + wy * vy) / vv : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    const double cx = s.ax + t * vx, cy = s.ay + t * vy;
    return std::hypot(px - cx, py - cy);
}

// True iff BOTH endpoints of `a` lie on SOME segment of `set` within tol (i.e. `a`
// is covered by the other partition's same-class segments).
bool coveredBy(const CCDrawingSegment& a, const CCDrawingSegment* set, int n, double tol) {
    bool aEnd = false, bEnd = false;
    for (int i = 0; i < n && !(aEnd && bEnd); ++i) {
        if (!aEnd && pointSegDist(a.ax, a.ay, set[i]) < tol) aEnd = true;
        if (!bEnd && pointSegDist(a.bx, a.by, set[i]) < tol) bEnd = true;
    }
    return aEnd && bEnd;
}

// Every segment of `x` is covered by some same-class segment of `y` (one direction).
bool partitionCovered(const CCDrawingSegment* x, int nx, const CCDrawingSegment* y, int ny,
                      double tol) {
    for (int i = 0; i < nx; ++i)
        if (!coveredBy(x[i], y, ny, tol)) return false;
    return true;
}

// ── build inputs ───────────────────────────────────────────────────────────────
struct HlrCase {
    const char* name;
    std::vector<double> profile;  // x,y pairs
    double depth;
    double view[3];
    double up[3];
    bool convexExactCount;  // assert count equality (convex → deterministic)
};

CCShapeId buildShape(const HlrCase& c) {
    return cc_solid_extrude(c.profile.data(), static_cast<int>(c.profile.size() / 2), c.depth);
}

void runCase(const HlrCase& c) {
    char detail[512];
    const double coordTol = 1e-5;   // mm — TIGHT (exact polyhedral projection)
    const double lenRelTol = 1e-4;  // relative total-length tolerance

    CCHlrOptions opts{};
    opts.deflection = 0.05;  // explicit occluder deflection (polyhedral → exact mesh)
    opts.samplesPerEdge = 0;
    opts.surfaceOffset = 0.0;

    // 1) OCCT oracle.
    cc_set_engine(0);
    const CCShapeId occtId = buildShape(c);
    if (occtId == 0) {
        std::snprintf(detail, sizeof detail, "OCCT build failed: %s", cc_last_error());
        record(false, c.name, detail);
        return;
    }
    const CCDrawing oD = cc_hlr_project(occtId, c.view, c.up, opts);

    // 2) Native core.
    cc_set_engine(1);
    const CCShapeId natId = buildShape(c);
    if (natId == 0) {
        std::snprintf(detail, sizeof detail, "native build failed: %s", cc_last_error());
        record(false, c.name, detail);
        cc_drawing_free(oD);
        cc_set_engine(0);
        cc_shape_release(occtId);
        return;
    }
    const CCDrawing nD = cc_hlr_project(natId, c.view, c.up, opts);

    // Both must have produced a drawing (non-empty visible outline).
    const bool produced = (oD.visibleCount > 0) && (nD.visibleCount > 0);

    // ── counts ────────────────────────────────────────────────────────────────
    const bool countsOk =
        produced && (nD.visibleCount == oD.visibleCount) && (nD.hiddenCount == oD.hiddenCount);
    std::snprintf(detail, sizeof detail, "visible n=%d o=%d | hidden n=%d o=%d",
                  nD.visibleCount, oD.visibleCount, nD.hiddenCount, oD.hiddenCount);
    // For convex solids the count is deterministic and MUST match; a non-convex
    // extrude has identical topology under both engines so it must match too.
    record(countsOk, std::string(c.name) + " counts", detail);

    // ── total projected length (visible + hidden) ───────────────────────────────
    const double nVis = totalLen(nD.visible, nD.visibleCount);
    const double oVis = totalLen(oD.visible, oD.visibleCount);
    const double nHid = totalLen(nD.hidden, nD.hiddenCount);
    const double oHid = totalLen(oD.hidden, oD.hiddenCount);
    const double visRel = oVis > 0.0 ? std::fabs(nVis - oVis) / oVis : (nVis > 0 ? 1.0 : 0.0);
    const double hidRel = oHid > 0.0 ? std::fabs(nHid - oHid) / oHid : (nHid > 0 ? 1.0 : 0.0);
    const bool lenOk = produced && (visRel < lenRelTol) && (hidRel < lenRelTol);
    std::snprintf(detail, sizeof detail,
                  "visLen n=%.6g o=%.6g rel=%.2e | hidLen n=%.6g o=%.6g rel=%.2e", nVis, oVis,
                  visRel, nHid, oHid, hidRel);
    record(lenOk, std::string(c.name) + " length", detail);

    // ── partition point-match (endpoint positions, both directions) ─────────────
    const bool visMatch =
        partitionCovered(nD.visible, nD.visibleCount, oD.visible, oD.visibleCount, coordTol) &&
        partitionCovered(oD.visible, oD.visibleCount, nD.visible, nD.visibleCount, coordTol);
    const bool hidMatch =
        partitionCovered(nD.hidden, nD.hiddenCount, oD.hidden, oD.hiddenCount, coordTol) &&
        partitionCovered(oD.hidden, oD.hiddenCount, nD.hidden, nD.hiddenCount, coordTol);
    const bool partOk = produced && visMatch && hidMatch;
    std::snprintf(detail, sizeof detail, "visible-partition=%d hidden-partition=%d (tol=%.0e)",
                  visMatch ? 1 : 0, hidMatch ? 1 : 0, coordTol);
    record(partOk, std::string(c.name) + " partition", detail);

    (void)c.convexExactCount;
    cc_drawing_free(nD);
    cc_drawing_free(oD);
    cc_set_engine(0);
    cc_shape_release(natId);
    cc_shape_release(occtId);
}

// ── honest-decline: a curved-silhouette solid is DECLINED by the native core ─────
// A cylinder (curved side face) is outside the polyhedral core; under the native
// engine cc_hlr_project must return an EMPTY drawing with cc_last_error set — never
// a wrong classification. (The OCCT oracle CAN draw it; this asserts only the native
// honest-decline contract.)
void runDecline() {
    char detail[512];
    // A rectangle [1..2]×[0..3] revolved 360° about Y → a cylindrical tube.
    const double prof[] = {1.0, 0.0, 2.0, 0.0, 2.0, 3.0, 1.0, 3.0};
    const double view[3] = {-1.0, -0.4, -1.0};
    const double up[3] = {0.0, 1.0, 0.0};

    cc_set_engine(1);
    const CCShapeId cyl = cc_solid_revolve(prof, 4, 2.0 * 3.14159265358979323846);
    if (cyl == 0) {
        std::snprintf(detail, sizeof detail, "native cylinder build failed: %s", cc_last_error());
        record(false, "cylinder decline", detail);
        cc_set_engine(0);
        return;
    }
    CCHlrOptions opts{};
    opts.deflection = 0.05;
    const CCDrawing d = cc_hlr_project(cyl, view, up, opts);
    const bool declined = (d.visibleCount == 0) && (d.hiddenCount == 0) &&
                          (d.visible == nullptr) && (d.hidden == nullptr);
    std::snprintf(detail, sizeof detail, "declined=%d (visible=%d hidden=%d) err='%s'",
                  declined ? 1 : 0, d.visibleCount, d.hiddenCount, cc_last_error());
    record(declined, "cylinder decline", detail);

    cc_drawing_free(d);
    cc_set_engine(0);
    cc_shape_release(cyl);
}

}  // namespace

int main() {
    EngineGuard guard;
    std::printf("── native-vs-OCCT hidden-line-removal parity (through the cc_* facade)\n");

    const std::vector<HlrCase> cases = {
        // 1) Box 3×2×5 from an isometric corner → 9 visible + 3 hidden.
        {"box-iso", {0.0, 0.0, 3.0, 0.0, 3.0, 2.0, 0.0, 2.0}, 5.0,
         {-1.0, -1.0, -1.0}, {0.0, 0.0, 1.0}, /*convex*/ true},
        // 2) Box from a second oblique corner (different silhouette split).
        {"box-oblique", {0.0, 0.0, 3.0, 0.0, 3.0, 2.0, 0.0, 2.0}, 5.0,
         {-1.0, -0.5, -0.8}, {0.0, 0.0, 1.0}, /*convex*/ true},
        // 3) Triangle prism (convex).
        {"triangle-prism", {0.0, 0.0, 4.0, 0.0, 2.0, 3.0}, 5.0,
         {-1.0, -1.0, -1.0}, {0.0, 0.0, 1.0}, /*convex*/ true},
        // 4) L-shaped prism (NON-convex, self-occluding) — same topology both engines.
        {"L-prism", {0.0, 0.0, 4.0, 0.0, 4.0, 1.5, 1.5, 1.5, 1.5, 4.0, 0.0, 4.0}, 3.0,
         {-1.0, -1.0, -1.0}, {0.0, 0.0, 1.0}, /*convex*/ false},
    };

    for (const HlrCase& c : cases) runCase(c);
    runDecline();

    cc_set_engine(0);
    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
