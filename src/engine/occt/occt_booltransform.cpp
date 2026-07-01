// OCCT engine adapter — BOOLEAN + TRANSFORM capability group.
//
// Definitions of the OcctEngine boolean/transform methods declared in
// occt_engine.h. Like every occt_*.cpp this is an OCCT-only TU: it may include
// OpenCASCADE headers (kept entirely inside this file), but no OCCT type escapes
// into any public or shared header. It is NOT host-buildable (there is no host
// OCCT); it compiles only for iOS where the trimmed OCCT static libs are linked
// (CYBERCAD_HAS_OCCT=ON).
//
// Ported behaviour-preservingly from CyberCad's KernelBridge.mm (the
// CYBERCAD_HAS_OCCT bodies of cc_boolean, cc_scale_shape, cc_scale_shape_about,
// cc_rotate_shape_about, cc_mirror_shape, cc_translate_shape, cc_place_on_frame).
// Every degenerate-input guard and every BRepCheck_Analyzer::IsValid gate is kept
// exactly as in the source: the fuse path validates + volume-checks by hand and
// escalates the fuzzy tolerance; cut/common gate through addIfValid; the rigid /
// scale transforms are accepted on xf.IsDone() alone (no IsValid re-check), as in
// the original. The facade collapses a returned Error to a nil handle + records
// cc_last_error, matching the source's `return 0`.
//
// Phase 1 (accelerate-multicore-occt): the boolean path now runs OCCT's parallel
// pave-filler (BOPAlgo_Options::SetRunParallel(true) on fuse/cut/common) with a
// tuned per-operation SetFuzzyValue for robustness on fine geometry. This is an
// internal acceleration only: NO cc_* signature changes, and the same
// BRepCheck_Analyzer::IsValid + fuse volume gate the serial path used still
// accepts the result — a parallel result must pass exactly the same validity
// check as serial (spec parallel-acceleration §"Parallel boolean execution").

#include "engine/occt/occt_engine.h"
#include "engine/occt/parallel_policy.h"

#include <algorithm>
#include <cmath>

// ── OCCT builders (adapter TU only) ───────────────────────────────────────────
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BOPAlgo_Options.hxx>  // SetRunParallel (base of BRepAlgoAPI_*): parallel pave-filler
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>

namespace cyber {

namespace {

// ── Tuned boolean fuzzy tolerance ─────────────────────────────────────────────
// The "fuzzy value" is the gap OCCT's pave-filler is allowed to weld across when
// intersecting the two operands. Fine geometry (e.g. a fine-pitch multi-turn
// thread fused/cut into a shaft) produces many near-tangent, near-coincident
// contacts; with no fuzz OCCT stalls or drops slivers, with too much fuzz it
// distorts real features. Chosen empirically for the fine-thread-into-shaft case
// (design.md §Decisions "Tuned SetFuzzyValue"; spec parallel-acceleration
// "Tuned boolean tolerance for fine geometry").
//
// TUNED DEFAULT: ~0.5% of the SMALLER operand's bounding-box diagonal, clamped to
// [1.0e-3 mm, 4.0 mm]:
//   * tied to the smaller shape so it stays well below any real feature size,
//     healing only sub-feature near-contact;
//   * lower clamp 1.0e-3 mm gives a floor for sub-millimetre models;
//   * upper clamp 4.0 mm stops a large operand from welding across real gaps.
double booleanFuzz(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    Bnd_Box ba, bb;
    BRepBndLib::Add(a, ba);
    BRepBndLib::Add(b, bb);
    if (ba.IsVoid() || bb.IsVoid()) {
        return 0.0;
    }
    const double smaller = std::min(std::sqrt(ba.SquareExtent()), std::sqrt(bb.SquareExtent()));
    return std::min(4.0, std::max(1.0e-3, smaller * 5.0e-3));  // ~0.5% of the smaller shape
}

// Per-operation fuzzy override. The tuned default above suits most models but is a
// blunt instrument (design.md §Risks "Fuzzy value is a blunt instrument"), so a
// specific operation can override it. The override is a per-thread, one-shot hook:
// the parallelism-policy layer sets it immediately before dispatching a boolean
// and the NEXT boolean_op on that thread consumes it, so an override can never
// leak into a later, unrelated op. Negative == unset. This keeps the per-op
// override entirely internal — the cc_* ABI is unchanged.
thread_local double t_boolFuzzOverride = -1.0;

// Resolve the base fuzzy for this boolean: the one-shot override if the policy set
// one, otherwise the tuned auto default. Consumes the override so it fires once.
double takeBooleanFuzz(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    if (t_boolFuzzOverride >= 0.0) {
        const double f = t_boolFuzzOverride;
        t_boolFuzzOverride = -1.0;  // one-shot: consume so it cannot leak forward
        return f;
    }
    return booleanFuzz(a, b);
}

double shapeVolume(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    return g.Mass();
}

}  // namespace

// Per-operation fuzzy override hook (external linkage, intentionally NOT in a
// public/shared header, so no OCCT/C++ type and no cc_* ABI is affected). The
// parallelism-policy layer (openspec task 1.2 "per-op override") extern-declares
// this and calls it on the worker thread immediately before dispatching a boolean:
//     namespace cyber::occt { void setNextBooleanFuzzy(double fuzzMm); }
// A non-negative value overrides the tuned auto default for that thread's next
// boolean_op only; a negative value clears any pending override.
namespace occt {
void setNextBooleanFuzzy(double fuzzMm) {
    t_boolFuzzOverride = fuzzMm;
}
}  // namespace occt

// ── boolean ───────────────────────────────────────────────────────────────────

namespace {

// The core fuse/cut/common, factored out so boolean_op stays a thin dispatcher
// (gate → resolve fuzzy → schedule). `parallel` drives OCCT's pave-filler across
// cores; `ctx` reports staged progress and is where cooperative cancellation is
// observed at the boundary. The SAME IsValid + fuse-volume gate the serial path
// used still accepts the result, so a parallel result must pass exactly the same
// validity check as serial (spec §"Parallel boolean execution").
ShapeResult runBoolean(const TopoDS_Shape& sa, const TopoDS_Shape& sb, int op, double baseFuzz,
                       bool parallel, OperationContext& ctx) {
    ctx.report(0.05, "boolean: intersect");
    // FUSE must never LOSE material. On near-tangent / grazing overlaps (e.g. a
    // prong poking through a tube wall) OCCT can report IsDone yet return only one
    // operand, silently dropping the rest of the body. Accept a fuse only when it
    // is valid AND at least as big as the larger operand; otherwise escalate the
    // fuzzy tolerance.
    if (op == 0) {
        const double need = std::max(shapeVolume(sa), shapeVolume(sb)) * 0.999;
        for (double f : {baseFuzz, baseFuzz * 4.0, baseFuzz * 16.0}) {
            BRepAlgoAPI_Fuse fu(sa, sb);
            fu.SetRunParallel(parallel);  // parallel pave-filler: scale across cores
            fu.SetFuzzyValue(f);
            fu.Build();
            if (!fu.IsDone()) {
                continue;
            }
            const TopoDS_Shape r = fu.Shape();
            if (!r.IsNull() && BRepCheck_Analyzer(r).IsValid() && shapeVolume(r) >= need) {
                ctx.report(1.0, "boolean: done");
                return occt::wrap(r);
            }
        }
        return make_error("boolean_op: fuse produced no valid result");
    }

    TopoDS_Shape result;
    if (op == 1) {
        BRepAlgoAPI_Cut c(sa, sb);
        c.SetRunParallel(parallel);  // parallel pave-filler: scale across cores
        c.SetFuzzyValue(baseFuzz);
        c.Build();
        if (!c.IsDone()) {
            return make_error("boolean_op: cut failed");
        }
        result = c.Shape();
    } else {
        BRepAlgoAPI_Common k(sa, sb);
        k.SetRunParallel(parallel);  // parallel pave-filler: scale across cores
        k.SetFuzzyValue(baseFuzz);
        k.Build();
        if (!k.IsDone()) {
            return make_error("boolean_op: common failed");
        }
        result = k.Shape();
    }
    ctx.report(1.0, "boolean: done");
    return occt::addIfValid(result, "boolean_op: invalid result");
}

}  // namespace

ShapeResult OcctEngine::boolean_op(EngineShape a, EngineShape b, int op) {
    // 1. Fine-thread gate (cheap, pre-dispatch): refuse a runaway fuse/cut of a
    //    high-turn fine-pitch thread into a shaft and keep them as SEPARATE bodies,
    //    surfacing the decision to the host rather than hanging (spec §"Fine-thread
    //    boolean gate").
    ShapeResult gated = make_error("boolean_op: gated");
    if (occt::checkFineThreadGate(a, b, op, gated)) {
        return gated;
    }

    // 2. Resolve the tuned/override fuzzy on the CALLER thread — the per-op override
    //    (takeBooleanFuzz / setNextBooleanFuzzy) is thread-local, so it must be
    //    consumed here, before the op is dispatched to a scheduler worker.
    const TopoDS_Shape* pa = occt::unwrap(a);
    const TopoDS_Shape* pb = occt::unwrap(b);
    if (pa == nullptr || pb == nullptr) {
        return make_error("boolean_op: unknown shape id");
    }
    const double baseFuzz = takeBooleanFuzz(*pa, *pb);

    // 3. Resolve parallelism (global toggle + per-op default) and bound OCCT's
    //    OSD_ThreadPool to the host worker cap before the parallel op runs.
    const bool parallel = occt::ParallelPolicy::instance().parallelFor();
    if (parallel) {
        occt::applyOcctWorkerCap();
    }

    // 4. Route the long boolean through the operation-scheduler off the caller's
    //    inline path, with the cancellation-safe boundary: the non-interruptible
    //    OCCT Build runs to completion on a worker, but a cancelled op discards its
    //    result and reclaims resources (spec §"Cancellable accelerated operations").
    return occt::runScheduled([a, b, op, baseFuzz, parallel](OperationContext& ctx) -> ShapeResult {
        return occt::occtGuard([&]() -> ShapeResult {
            return runBoolean(*occt::unwrap(a), *occt::unwrap(b), op, baseFuzz, parallel, ctx);
        });
    });
}

// ── transform ───────────────────────────────────────────────────────────────
// The source accepts each rigid/scale transform on xf.IsDone() alone (a valid
// input under a rigid or uniform-scale motion stays valid), so these deliberately
// do NOT run the BRepCheck IsValid gate — preserved exactly.

ShapeResult OcctEngine::scale_shape(EngineShape body, double factor) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* p = occt::unwrap(body);
        if (p == nullptr || !(factor > 0)) {
            return make_error("scale_shape: unknown shape id or non-positive factor");
        }
        gp_Trsf t;
        t.SetScale(gp_Pnt(0, 0, 0), factor);
        BRepBuilderAPI_Transform xf(*p, t, Standard_True);
        if (!xf.IsDone()) {
            return make_error("scale_shape: transform failed");
        }
        return occt::wrap(xf.Shape());
    });
}

ShapeResult OcctEngine::scale_shape_about(EngineShape body, double cx, double cy, double cz,
                                          double factor) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* p = occt::unwrap(body);
        if (p == nullptr || !(factor > 0)) {
            return make_error("scale_shape_about: unknown shape id or non-positive factor");
        }
        gp_Trsf t;
        t.SetScale(gp_Pnt(cx, cy, cz), factor);
        BRepBuilderAPI_Transform xf(*p, t, Standard_True);
        if (!xf.IsDone()) {
            return make_error("scale_shape_about: transform failed");
        }
        return occt::wrap(xf.Shape());
    });
}

ShapeResult OcctEngine::rotate_shape_about(EngineShape body, double cx, double cy, double cz,
                                           double ax, double ay, double az, double angleRadians) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* p = occt::unwrap(body);
        if (p == nullptr) {
            return make_error("rotate_shape_about: unknown shape id");
        }
        if (ax * ax + ay * ay + az * az < 1.0e-12) {
            return make_error("rotate_shape_about: zero axis");  // need a non-zero axis
        }
        gp_Trsf t;
        t.SetRotation(gp_Ax1(gp_Pnt(cx, cy, cz), gp_Dir(ax, ay, az)), angleRadians);
        BRepBuilderAPI_Transform xf(*p, t, Standard_True);
        if (!xf.IsDone()) {
            return make_error("rotate_shape_about: transform failed");
        }
        return occt::wrap(xf.Shape());
    });
}

ShapeResult OcctEngine::mirror_shape(EngineShape body, double px, double py, double pz, double nx,
                                     double ny, double nz) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* p = occt::unwrap(body);
        if (p == nullptr) {
            return make_error("mirror_shape: unknown shape id");
        }
        if (nx * nx + ny * ny + nz * nz < 1.0e-12) {
            return make_error("mirror_shape: zero plane normal");  // need a plane normal
        }
        gp_Trsf t;
        // mirror across the plane ⟂ normal
        t.SetMirror(gp_Ax2(gp_Pnt(px, py, pz), gp_Dir(nx, ny, nz)));
        BRepBuilderAPI_Transform xf(*p, t, Standard_True);
        if (!xf.IsDone()) {
            return make_error("mirror_shape: transform failed");
        }
        return occt::wrap(xf.Shape());
    });
}

ShapeResult OcctEngine::translate_shape(EngineShape body, double tx, double ty, double tz) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* p = occt::unwrap(body);
        if (p == nullptr) {
            return make_error("translate_shape: unknown shape id");
        }
        gp_Trsf t;
        t.SetTranslation(gp_Vec(tx, ty, tz));
        BRepBuilderAPI_Transform xf(*p, t, Standard_True);
        if (!xf.IsDone()) {
            return make_error("translate_shape: transform failed");
        }
        return occt::wrap(xf.Shape());
    });
}

ShapeResult OcctEngine::place_on_frame(EngineShape body, double ox, double oy, double oz, double ux,
                                       double uy, double uz, double vx, double vy, double vz) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* p = occt::unwrap(body);
        if (p == nullptr) {
            return make_error("place_on_frame: unknown shape id");
        }
        gp_Vec u(ux, uy, uz), v(vx, vy, vz);
        gp_Vec n = u.Crossed(v);
        if (u.Magnitude() < 1.0e-9 || v.Magnitude() < 1.0e-9 || n.Magnitude() < 1.0e-9) {
            return make_error("place_on_frame: degenerate frame");
        }
        // Rigid motion that relocates the global XOY frame onto the plane frame
        // (origin, normal = u×v, x-dir = u). gp_Dir normalises, so the motion is a
        // pure rotation+translation — sketch dimensions are preserved.
        gp_Ax3 dst(gp_Pnt(ox, oy, oz), gp_Dir(n), gp_Dir(u));
        gp_Trsf t;
        t.SetDisplacement(gp_Ax3(), dst);
        BRepBuilderAPI_Transform xf(*p, t, Standard_True);
        if (!xf.IsDone()) {
            return make_error("place_on_frame: transform failed");
        }
        return occt::wrap(xf.Shape());
    });
}

}  // namespace cyber
