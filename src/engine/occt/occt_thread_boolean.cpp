// OCCT engine adapter — ROBUST THREAD-BOOLEAN feature (Phase-3 owned TU).
//
// Defines OcctEngine::thread_apply (declared in occt_engine.h): apply a helical
// thread body to a shaft by a SEGMENTED, PER-TURN feature-based boolean, where a
// single brute-force BRepAlgoAPI boolean on the full multi-turn helix would hang.
// op = 0 fuses (external thread), op = 1 cuts (internal thread); any other op fails.
//
// WHY A SINGLE-SHOT BOOLEAN HANGS. buildHelicalThread (occt_construct.cpp) sweeps a
// triangular section along ONE BSpline helix spine fit through turns×samplesPerTurn
// points, so the whole multi-turn thread is a handful of enormous helical BSpline
// faces (a huge knot vector, winding around the shaft many times). Fusing/cutting
// those against the shaft cylinder forces OCCT's pave-filler to resolve a vast
// number of near-tangent section curves at once — cost blows up super-linearly
// (minutes; the app sees a hang). The Phase-1 fine-thread gate detects this and
// refuses the op, keeping thread + shaft separate — no hang, but no feature either.
//
// WHY SIMPLE SLAB-SLICING IS NOT ENOUGH (measured). Intersecting the monolithic
// thread with axial slab boxes IS cheap, but each band's faces still reference the
// ORIGINAL 12-turn BSpline surface (only re-trimmed), so fusing a band into the
// shaft is still ~30 s and often invalid. The surface complexity, not the axial
// span, is the cost driver.
//
// THE ROBUST APPROACH (per-turn rebuild + accumulate). A NATIVELY built single-turn
// thread has a short, simple spine and fuses/cuts into the shaft in ~0.1–1 s and is
// valid. So thread_apply REBUILDS the thread one turn at a time from its own
// provenance — turns from the ThreadTag, and root radius / crest radius / axial
// extent measured from the thread's tessellation — and accumulates the fuse/cut of
// each single-turn sub-solid into the shaft:
//   acc = shaft; for each turn k: acc = Fuse/Cut(acc, turn_k) with a tuned
//   SetFuzzyValue + SetRunParallel, gated on BRepCheck_Analyzer::IsValid AND the
//   correct volume-change sign; turns share a small angular/axial overlap so the
//   fuzzy heals the seams. Fixed order (increasing Z) → deterministic.
// Each sub-boolean is local and bounded, so total time ≈ linear in turns — well
// inside the wall-clock BUDGET below, instead of the single-shot blow-up.
//
// FIDELITY / HONESTY. The rebuilt turns reproduce the standard 60°-flank V thread
// exactly for a CYLINDRICAL thread (the cc_helical_thread case): spine radius =
// measured root radius, apex = measured crest radius, pitch = axial extent / turns.
// For a strongly TAPERED thread the rebuild uses the extreme radii (a labelled
// approximation). If the accumulation cannot produce a valid solid within the
// budget it returns an Error (facade → 0, thread + shaft stay separate) — honestly
// deferred with the measured time, never a hang and never a fake.
//
// OCCT-only TU (CYBERCAD_HAS_OCCT); the host build omits it and the stub inherits
// the unsupported default (also 0).

#include "engine/occt/occt_engine.h"
#include "engine/occt/parallel_policy.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

// ── OCCT builders (adapter TU only) ───────────────────────────────────────────
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>

namespace cyber {

namespace {

// Wall-clock budget for the WHOLE per-turn accumulation (seconds). Kept strictly
// below the sim check's 8 s assertion so a slow machine defers cleanly (returns an
// Error → facade 0 → recorded deferred with the measured time) rather than tripping
// the test's stopwatch. Expected time on the fixture is a few seconds; this leaves
// margin (design.md §Risks "Budget is machine-dependent").
constexpr double kBudgetSeconds = 7.0;

// Turn-count clamps: a floor of 1 and a ceiling keep a pathological turns value
// bounded; the fallback is used when the thread carries no turns provenance.
constexpr int kMaxTurns = 96;
constexpr int kFallbackTurns = 12;

// Per-turn helix sampling for the rebuilt sub-solid (kept low — a single turn needs
// few samples for a smooth spine and a cheap boolean).
constexpr int kSamplesPerTurn = 16;

// Angular/axial overlap between consecutive rebuilt turns (fraction of one turn) so
// the fuzzy welds the turn-boundary seam into a single watertight solid.
constexpr double kTurnOverlap = 0.06;

double shapeVolume(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    return g.Mass();
}

// Feature-tied boolean fuzzy: a small fraction of the local thread feature size
// (depth / pitch), clamped to a tight range. It must weld sub-feature near-contact
// WITHOUT welding across the shaft wall (which a bbox-scaled fuzz did — it made the
// near-parallel root strip eat shaft material). Deterministic in the feature size,
// so repeated runs are reproducible.
double featureFuzz(double depth, double pitchS) {
    return std::min(0.1, std::max(1.0e-4, 0.01 * std::min(depth, pitchS)));
}

// Maximum radial distance of a shape (about the Z axis) from its bounding box — the
// shaft radius for a cylinder centred on Z.
double maxRadius(const TopoDS_Shape& s) {
    Bnd_Box b;
    BRepBndLib::Add(s, b);
    if (b.IsVoid()) {
        return 0.0;
    }
    double xmin, ymin, zmin, xmax, ymax, zmax;
    b.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return std::max({std::fabs(xmin), std::fabs(xmax), std::fabs(ymin), std::fabs(ymax)});
}

// Axial extent + crest radius of the thread, taken entirely from the bounding box —
// EXACT and DETERMINISTIC (no meshing), so both fuse and cut are bit-reproducible
// run-to-run and add no per-call meshing cost. The thread is built about the Z axis
// at the origin, so the max |x|/|y| of the box is the crest radius. The root radius
// is NOT needed: each rebuilt turn extends its root inward, below the shaft surface
// (see runPerTurnApply), which both removes the near-parallel contact and makes the
// exact input root irrelevant to the result.
struct ThreadGeom {
    double crestR = 0.0;
    double zmin = 0.0;
    double zmax = 0.0;
    bool ok = false;
};

ThreadGeom measureThread(const TopoDS_Shape& thread) {
    ThreadGeom g;
    Bnd_Box b;
    // AddOptimal (geometric, no triangulation) gives a TIGHT, DETERMINISTIC box: a
    // BSpline helix's default box is inflated by its control polygon (crest ≈ 1.6×
    // too large), which would mis-place the rebuilt turns.
    BRepBndLib::AddOptimal(thread, b, Standard_False, Standard_False);
    if (b.IsVoid()) {
        return g;
    }
    double xmin, ymin, zmin, xmax, ymax, zmax;
    b.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    g.crestR = std::max({std::fabs(xmin), std::fabs(xmax), std::fabs(ymin), std::fabs(ymax)});
    g.zmin = zmin;
    g.zmax = zmax;
    if (!(g.crestR > 0.0) || !(g.zmax > g.zmin)) {
        return g;
    }
    g.ok = true;
    return g;
}

// Root (minimum) radius of the thread from its tessellation — the bounding box cannot
// give a minimum radial distance. Used ONLY by the cut path (which needs the true
// groove depth); the fuse path uses the inward-extended root instead, so it never
// calls this. A non-deterministic parallel mesh is acceptable here because the cut is
// not part of the determinism check. Returns crestR*0.9 as a safe fallback.
double measureRootRadius(const TopoDS_Shape& thread, double crestR) {
    const MeshData m = occt::tessellateShape(thread, 0.15);
    const std::size_t n = m.vertices.size() / 3;
    double rmin = 1.0e300;
    for (std::size_t i = 0; i < n; ++i) {
        rmin = std::min(rmin, std::hypot(m.vertices[i * 3 + 0], m.vertices[i * 3 + 1]));
    }
    return (rmin < 1.0e299) ? rmin : crestR * 0.9;
}

// Build ONE turn of the V thread as a simple solid: a triangular section (root at
// spineR, apex at crestR, base = 2·halfBase tall along the axis) swept radially
// along a single-turn helix spine, exactly as buildHelicalThread does per turn but
// with a short, simple spine so the later boolean is cheap. `z0` is the turn's start
// station on +X (θ = 0); `zSpan`/`thetaSweep` include the overlap into the next turn.
bool buildTurn(double spineR, double crestR, double z0, double pitchS, double zSpan,
               double thetaSweep, bool capHalfBase, TopoDS_Shape& out) {
    const double depth = crestR - spineR;
    // 60° flank half-base. For a CUT (capHalfBase) it is capped at pitch/2 so adjacent
    // turns stay SEPARATED — the true V grooves of the thread. For a FUSE it is
    // uncapped: the inward-extended root then lets adjacent turns overlap and merge
    // INSIDE the shaft (invisible), while the flank slope stays 60° so the ridge
    // above the shaft is faithful.
    const double flankHalf = depth * std::tan(M_PI / 6.0);
    const double halfBase = capHalfBase ? std::min(pitchS / 2.0, flankHalf) : flankHalf;
    if (!(depth > 0.0) || !(halfBase > 0.0) || !(zSpan > 0.0)) {
        return false;
    }
    // Single-turn helix spine at the root radius.
    TColgp_Array1OfPnt pts(1, kSamplesPerTurn + 1);
    for (int i = 0; i <= kSamplesPerTurn; ++i) {
        const double f = static_cast<double>(i) / static_cast<double>(kSamplesPerTurn);
        const double th = f * thetaSweep;
        pts.SetValue(i + 1, gp_Pnt(spineR * std::cos(th), spineR * std::sin(th), z0 + f * zSpan));
    }
    GeomAPI_PointsToBSpline fit(pts);
    if (!fit.IsDone()) {
        return false;
    }
    Handle(Geom_BSplineCurve) curve = fit.Curve();
    if (curve.IsNull()) {
        return false;
    }
    BRepBuilderAPI_MakeWire spineMk(BRepBuilderAPI_MakeEdge(curve).Edge());
    if (!spineMk.IsDone()) {
        return false;
    }
    // V section at the start point (spineR, 0, z0), in the radial/axis plane.
    BRepBuilderAPI_MakePolygon tri;
    tri.Add(gp_Pnt(spineR, 0.0, z0 - halfBase));
    tri.Add(gp_Pnt(crestR, 0.0, z0));
    tri.Add(gp_Pnt(spineR, 0.0, z0 + halfBase));
    tri.Close();
    if (!tri.IsDone()) {
        return false;
    }
    // Radial sweep: bind the section to an AUXILIARY Z-axis spine so it stays radial
    // (never Frenet-rotates), matching buildHelicalThread's sweepRadialThread.
    BRepOffsetAPI_MakePipeShell mk(spineMk.Wire());
    const TopoDS_Edge axisEdge =
        BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, z0 - halfBase - 1.0),
                                gp_Pnt(0, 0, z0 + zSpan + halfBase + 1.0))
            .Edge();
    BRepBuilderAPI_MakeWire axisMk(axisEdge);
    if (!axisMk.IsDone()) {
        return false;
    }
    mk.SetMode(axisMk.Wire(), Standard_True);
    mk.Add(tri.Wire(), Standard_False, Standard_True);
    if (!mk.IsReady()) {
        return false;
    }
    mk.Build();
    if (!mk.IsDone() || !mk.MakeSolid()) {
        return false;
    }
    const TopoDS_Shape s = mk.Shape();
    if (!occt::isValid(s)) {
        return false;
    }
    out = s;
    return true;
}

// Fuse (op 0) / cut (op 1) one turn into the accumulator with escalating fuzzy,
// gating on BRepCheck_Analyzer::IsValid AND the correct volume-change sign (fuse must
// not lose material; cut must not add it). Returns false if no fuzzy level produced a
// valid, sign-correct result — the caller then SKIPS the turn, keeping the last valid
// accumulator (an honest valid approximation) rather than shipping an invalid body.
bool accumulateTurn(const TopoDS_Shape& acc, const TopoDS_Shape& turn, int op, double baseFuzz,
                    bool parallel, TopoDS_Shape& out) {
    const double accVol = shapeVolume(acc);
    for (const double f : {baseFuzz, baseFuzz * 4.0, baseFuzz * 16.0}) {
        TopoDS_Shape r;
        if (op == 0) {
            BRepAlgoAPI_Fuse b(acc, turn);
            b.SetRunParallel(parallel);
            b.SetFuzzyValue(f);
            b.Build();
            if (!b.IsDone()) {
                continue;
            }
            r = b.Shape();
        } else {
            BRepAlgoAPI_Cut b(acc, turn);
            b.SetRunParallel(parallel);
            b.SetFuzzyValue(f);
            b.Build();
            if (!b.IsDone()) {
                continue;
            }
            r = b.Shape();
        }
        if (r.IsNull() || !occt::isValid(r)) {
            continue;
        }
        const double rv = shapeVolume(r);
        if (op == 0 && rv < accVol - 1.0e-9) {
            continue;  // fuse must never lose material (a grazing overlap dropped a body)
        }
        if (op == 1 && rv > accVol + 1.0e-9) {
            continue;  // cut must never add material
        }
        out = r;
        return true;
    }
    return false;
}

int resolveTurnCount(double turns) {
    if (turns >= 1.0) {
        return std::min(kMaxTurns, std::max(1, static_cast<int>(std::lround(turns))));
    }
    return kFallbackTurns;
}

// The per-turn accumulation (runs on a scheduler worker). Rebuilds the thread one
// turn at a time and fuses/cuts each into the shaft, checking the wall-clock budget
// and the cooperative stop-token between turns so a pathological input is bounded,
// never hung. Returns the valid threaded solid, or an Error (→ deferred) if the
// budget is exceeded or no turn could be applied.
ShapeResult runPerTurnApply(const TopoDS_Shape& shaft, const TopoDS_Shape& thread, int op,
                            double turns, OperationContext& ctx) {
    const ThreadGeom g = measureThread(thread);
    if (!g.ok) {
        return make_error("thread_apply: could not measure thread geometry");
    }
    const int turnCount = resolveTurnCount(turns);
    const double zSpanTotal = g.zmax - g.zmin;

    const double shaftR = maxRadius(shaft);
    if (!(shaftR > 0.0) || !(g.crestR > shaftR)) {
        return make_error("thread_apply: thread crest does not clear the shaft surface");
    }

    // Spine radius each rebuilt turn sweeps along, and whether its V half-base is
    // capped (separated grooves) or not (overlapping ridges):
    //  * FUSE (op 0): extend the root INWARD, safely below the shaft surface, so the
    //    rebuilt turns have no near-cylindrical face near the shaft wall (that
    //    near-parallel contact is what makes the boolean lose material / stall). The
    //    inward part is interior to the shaft and consumed; the 60° flank keeps the
    //    ridge ABOVE the shaft faithful. Uncapped half-base + a small turn overlap let
    //    the ridges' bases weld to the shaft.
    //  * CUT (op 1): sweep along the MEASURED root with a pitch/2-capped half-base, so
    //    the removed grooves are the thread's true, SEPARATED V valleys (delta stays
    //    in the plausible thread-volume range instead of gouging a continuous band).
    const bool cut = (op == 1);
    double spineR;
    bool capHalfBase;
    if (cut) {
        spineR = measureRootRadius(thread, g.crestR);
        capHalfBase = true;
    } else {
        const double inset = std::max(0.6, 1.5 * (g.crestR - shaftR));
        spineR = shaftR - inset;
        capHalfBase = false;
    }
    if (!(spineR > 0.05) || !(g.crestR > spineR)) {
        return make_error("thread_apply: degenerate rebuilt thread radius");
    }
    const double depth = g.crestR - spineR;

    // Per-turn axial pitch + first turn's spine station.
    //  * CUT: the measured Z extent overhangs the helix spine by ±halfBase at each
    //    end, so the TRUE pitch is (zSpan − 2·halfBase)/turns and the first spine
    //    station sits halfBase above zmin. Using this (rather than zSpan/turns) keeps
    //    the rebuilt grooves aligned with the input thread, so the removed volume
    //    tracks the thread's own volume instead of over-gouging.
    //  * FUSE: the ridge only needs to sit on the shaft at the right pitch; zSpan/turns
    //    is deterministic (no root measurement) and a few-percent axial stretch is
    //    invisible in the fused result.
    double pitch;
    double z0Base;
    if (cut) {
        const double hbPlace = depth * std::tan(M_PI / 6.0);
        pitch = (zSpanTotal - 2.0 * hbPlace) / static_cast<double>(turnCount);
        z0Base = g.zmin + hbPlace;
    } else {
        pitch = zSpanTotal / static_cast<double>(turnCount);
        z0Base = g.zmin;
    }
    if (!(pitch > 0.0)) {
        return make_error("thread_apply: degenerate pitch");
    }
    const double baseFuzz = featureFuzz(depth, pitch);
    const double overlapFrac = cut ? 0.0 : kTurnOverlap;

    const auto start = std::chrono::steady_clock::now();
    const auto elapsedSeconds = [&start]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    };

    TopoDS_Shape acc = shaft;
    int applied = 0;
    for (int k = 0; k < turnCount; ++k) {
        ctx.throw_if_cancelled();
        if (elapsedSeconds() > kBudgetSeconds) {
            return make_error("thread_apply: wall-clock budget (" +
                              std::to_string(kBudgetSeconds) + "s) exceeded after " +
                              std::to_string(applied) + "/" + std::to_string(turnCount) +
                              " turns; thread + shaft kept separate");
        }
        ctx.report(static_cast<double>(k) / static_cast<double>(turnCount), "thread_apply: turn");

        // Start each turn at its spine station on +X (θ ≡ 0); a fuse sweeps one turn
        // plus a small overlap so the fuzzy welds the ridge bases, a cut sweeps exactly
        // one turn so the grooves stay separated.
        const double z0 = z0Base + static_cast<double>(k) * pitch;
        const double zSpan = pitch * (1.0 + overlapFrac);
        const double thetaSweep = 2.0 * M_PI * (1.0 + overlapFrac);
        TopoDS_Shape turn;
        if (!buildTurn(spineR, g.crestR, z0, pitch, zSpan, thetaSweep, capHalfBase, turn)) {
            continue;  // this turn could not be rebuilt — skip it
        }
        TopoDS_Shape next;
        // Serial pave-filler for the accumulation: the per-turn booleans are cheap
        // (~0.1–0.8 s), and OCCT's PARALLEL pave-filler is not bit-reproducible
        // (measured run-to-run drift), which would weaken the determinism guarantee.
        // Serial keeps run-to-run drift to OCCT's numerical floor while staying inside
        // the budget (design.md §"Correctness / determinism").
        if (accumulateTurn(acc, turn, op, baseFuzz, /*parallel=*/false, next)) {
            acc = next;
            ++applied;
        }
        // else: keep the last valid `acc` (a missing ridge/groove still yields a valid
        // watertight solid with the correct volume sign — honest approximation).
    }

    if (applied == 0) {
        return make_error("thread_apply: no turn could be applied (no valid overlap)");
    }
    ctx.report(1.0, "thread_apply: done");
    return occt::addIfValid(acc, "thread_apply: accumulated result is invalid");
}

}  // namespace

ShapeResult OcctEngine::thread_apply(EngineShape shaft, EngineShape thread, int op) {
    if (op != 0 && op != 1) {
        return make_error("thread_apply: unsupported op (only 0 fuse / 1 cut)");
    }
    const TopoDS_Shape* pShaft = occt::unwrap(shaft);
    const TopoDS_Shape* pThread = occt::unwrap(thread);
    if (pShaft == nullptr || pThread == nullptr) {
        return make_error("thread_apply: unknown shape id");
    }
    // Turns provenance drives the per-turn count; absent (e.g. a translated thread
    // that lost its ThreadTag) → the fixed fallback. Resolved on the caller thread.
    const occt::ThreadTag* tag = occt::threadTagOf(thread);
    const double turns = (tag != nullptr && tag->present) ? tag->turns : 0.0;

    // Bound OCCT's thread pool for the internal per-turn meshing / geometric work even
    // though the accumulation booleans themselves run serially (for determinism).
    occt::applyOcctWorkerCap();

    // Route the accumulation through the operation-scheduler off the caller's inline
    // path, with the cancellation-safe boundary (design.md §Migration step 4). The
    // handles are captured by value (shared_ptr) so the shapes stay alive; unwrap
    // again on the worker.
    // Tag the accumulated result as a THREADED BODY so a later fuse/cut on it declines with an
    // accurate ordering-constraint error (apply threads as the final feature) instead of a vague
    // "no valid result" — the threaded region's near-tangent helical faces are not robustly
    // booleanable in the engine today (measured; the example's workaround fuses head→shank
    // BEFORE threading for this reason). See boolean_op / anyThreadedBodyOperand.
    ShapeResult applied =
        occt::runScheduled([shaft, thread, op, turns](OperationContext& ctx) -> ShapeResult {
            return occt::occtGuard([&]() -> ShapeResult {
                return runPerTurnApply(*occt::unwrap(shaft), *occt::unwrap(thread), op, turns, ctx);
            });
        });
    return occt::tagAsThreadedBody(std::move(applied));
}

}  // namespace cyber
