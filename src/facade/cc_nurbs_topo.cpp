// Public C facade — J5 NURBS topology wrappers: curve/curve + curve/surface
// intersection and the parameter-space trim-region boolean.
//
// This file is the BRIDGE for the intersection primitives
// (src/native/math/bspline_intersect.h — intersectCurveCurve / intersectCurveSurface)
// and the parameter-space region boolean (src/native/topology/trim_boolean.h —
// trimRegionBoolean). Like the rest of src/facade it may call src/native directly;
// src/native itself stays OCCT-FREE. Every entry point is a thin, guarded delegation
// that validates its raw C input, drives the native module, and NEVER leaks a C++
// type across the boundary.
//
// HONEST-DECLINE contract (design.md §3). A coincident / overlapping curve pair
// (an infinite intersection set), a curve lying on a surface, or a coincident-edge /
// tangential-only trim overlap collapse to a NEGATIVE count + cc_last_error + a NULL
// out array — never a fabricated hit or a wrong region. No tolerance is widened here.
//
// PARALLELISM NOTE (J2–J5 file-disjoint): this file uses ONLY J1's PUBLIC accessors
// (cc_curve_info / cc_curve_knots / cc_curve_poles, cc_surface_* + cc_curve_create).
// It NEVER touches J1's internal registry; the native views are rebuilt from the
// read-back POD data. Local static helpers only.

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "core/guard.h"
#include "cybercadkernel/cc_kernel.h"

#ifdef CYBERCAD_HAS_NUMSCI
#include "native/math/bspline_intersect.h"
#endif
#include "native/math/vec.h"
#include "native/topology/trim_boolean.h"
#include "native/topology/trimmed_nurbs.h"

namespace {

using cyber::guard;
using cyber::set_last_error;
using cybercad::native::math::Point3;

// ── read-back of a J1 handle into native storage (public accessors only) ───────
//
// A NURBS curve/surface pulled back through the public POD accessors. Owns its
// pole / weight / knot vectors so the native span views stay valid for the call.

struct CurveData {
    int degree = 1;
    std::vector<Point3> poles;
    std::vector<double> weights;  // empty ⇒ non-rational
    std::vector<double> knots;
    bool ok = false;
};

// Pull a cc_curve back through cc_curve_info + cc_curve_knots + cc_curve_poles.
CurveData readCurve(cc_curve h) {
    CurveData d;
    CCCurveInfo info{};
    if (cc_curve_info(h, &info) != 1) {
        return d;  // ok = false (cc_curve_info set cc_last_error)
    }
    d.degree = info.degree;
    d.knots.resize(static_cast<std::size_t>(info.n_knots));
    if (cc_curve_knots(h, d.knots.data(), info.n_knots) != info.n_knots) {
        return d;
    }
    std::vector<double> xyzw(static_cast<std::size_t>(info.n_ctrl) * 4);
    if (cc_curve_poles(h, xyzw.data(), static_cast<int>(xyzw.size())) !=
        static_cast<int>(xyzw.size())) {
        return d;
    }
    d.poles.resize(static_cast<std::size_t>(info.n_ctrl));
    if (info.rational) {
        d.weights.resize(static_cast<std::size_t>(info.n_ctrl));
    }
    for (int i = 0; i < info.n_ctrl; ++i) {
        d.poles[static_cast<std::size_t>(i)] =
            Point3{xyzw[4 * i + 0], xyzw[4 * i + 1], xyzw[4 * i + 2]};
        if (info.rational) {
            d.weights[static_cast<std::size_t>(i)] = xyzw[4 * i + 3];
        }
    }
    d.ok = true;
    return d;
}

struct SurfaceData {
    int degreeU = 1, degreeV = 1;
    int nRows = 0, nCols = 0;
    std::vector<Point3> poles;
    std::vector<double> weights;  // empty ⇒ non-rational
    std::vector<double> knotsU, knotsV;
    bool ok = false;
};

SurfaceData readSurface(cc_surface h) {
    SurfaceData d;
    CCSurfaceInfo info{};
    if (cc_surface_info(h, &info) != 1) {
        return d;
    }
    d.degreeU = info.degree_u;
    d.degreeV = info.degree_v;
    d.nRows = info.n_ctrl_u;
    d.nCols = info.n_ctrl_v;
    d.knotsU.resize(static_cast<std::size_t>(info.n_knots_u));
    d.knotsV.resize(static_cast<std::size_t>(info.n_knots_v));
    if (cc_surface_knots_u(h, d.knotsU.data(), info.n_knots_u) != info.n_knots_u ||
        cc_surface_knots_v(h, d.knotsV.data(), info.n_knots_v) != info.n_knots_v) {
        return d;
    }
    const int total = info.n_ctrl_u * info.n_ctrl_v;
    std::vector<double> xyzw(static_cast<std::size_t>(total) * 4);
    if (cc_surface_poles(h, xyzw.data(), static_cast<int>(xyzw.size())) !=
        static_cast<int>(xyzw.size())) {
        return d;
    }
    d.poles.resize(static_cast<std::size_t>(total));
    if (info.rational) {
        d.weights.resize(static_cast<std::size_t>(total));
    }
    for (int i = 0; i < total; ++i) {
        d.poles[static_cast<std::size_t>(i)] =
            Point3{xyzw[4 * i + 0], xyzw[4 * i + 1], xyzw[4 * i + 2]};
        if (info.rational) {
            d.weights[static_cast<std::size_t>(i)] = xyzw[4 * i + 3];
        }
    }
    d.ok = true;
    return d;
}

}  // namespace

// ── curve<->curve / curve<->surface intersection ───────────────────────────────
//
// Both intersectors live behind CYBERCAD_HAS_NUMSCI (bspline_intersect.cpp is
// numsci-gated). With the guard OFF the wrappers HONEST-DECLINE (< 0) rather than
// link to an absent symbol.

int cc_nurbs_intersect_cc(cc_curve a, cc_curve b, double tol, CCCurveHit** outHits) {
    return guard(
        [&]() -> int {
            if (outHits != nullptr) {
                *outHits = nullptr;
            }
#ifndef CYBERCAD_HAS_NUMSCI
            set_last_error(
                "cc_nurbs_intersect_cc: intersection requires the numeric substrate "
                "(build with CYBERCAD_HAS_NUMSCI)");
            return -1;
#else
            namespace nm = cybercad::native::math;
            const CurveData da = readCurve(a);
            const CurveData db = readCurve(b);
            if (!da.ok || !db.ok) {
                set_last_error("cc_nurbs_intersect_cc: unknown / invalid curve handle");
                return -1;
            }
            const nm::CurveView va{da.degree, {da.poles.data(), da.poles.size()},
                                   {da.weights.data(), da.weights.size()},
                                   {da.knots.data(), da.knots.size()}};
            const nm::CurveView vb{db.degree, {db.poles.data(), db.poles.size()},
                                   {db.weights.data(), db.weights.size()},
                                   {db.knots.data(), db.knots.size()}};
            const double useTol = tol > 0.0 ? tol : 1e-9;
            const nm::CurveCurveResult r = nm::intersectCurveCurve(va, vb, useTol);
            if (r.status == nm::IntersectStatus::Coincident) {
                set_last_error(
                    "cc_nurbs_intersect_cc: coincident / overlapping curves — infinite "
                    "intersection set, honest-declined");
                return -1;
            }
            const int n = static_cast<int>(r.hits.size());
            if (n == 0 || outHits == nullptr) {
                return n;
            }
            auto* out = static_cast<CCCurveHit*>(
                std::malloc(static_cast<std::size_t>(n) * sizeof(CCCurveHit)));
            if (out == nullptr) {
                set_last_error("cc_nurbs_intersect_cc: out of memory");
                return -1;
            }
            for (int i = 0; i < n; ++i) {
                const nm::CurveCurveHit& h = r.hits[static_cast<std::size_t>(i)];
                out[i].xyz[0] = h.point.x;
                out[i].xyz[1] = h.point.y;
                out[i].xyz[2] = h.point.z;
                out[i].tA = h.paramA;
                out[i].tB = h.paramB;
                out[i].tangential =
                    h.type == nm::IntersectionType::Tangential ? 1 : 0;
            }
            *outHits = out;
            return n;
#endif
        },
        -1);
}

void cc_nurbs_hits_cc_free(CCCurveHit* hits) {
    std::free(hits);
}

int cc_nurbs_intersect_cs(cc_curve c, cc_surface s, double tol,
                          CCCurveSurfaceHit** outHits) {
    return guard(
        [&]() -> int {
            if (outHits != nullptr) {
                *outHits = nullptr;
            }
#ifndef CYBERCAD_HAS_NUMSCI
            set_last_error(
                "cc_nurbs_intersect_cs: intersection requires the numeric substrate "
                "(build with CYBERCAD_HAS_NUMSCI)");
            return -1;
#else
            namespace nm = cybercad::native::math;
            const CurveData dc = readCurve(c);
            const SurfaceData ds = readSurface(s);
            if (!dc.ok || !ds.ok) {
                set_last_error(
                    "cc_nurbs_intersect_cs: unknown / invalid curve or surface handle");
                return -1;
            }
            const nm::CurveView vc{dc.degree, {dc.poles.data(), dc.poles.size()},
                                   {dc.weights.data(), dc.weights.size()},
                                   {dc.knots.data(), dc.knots.size()}};
            nm::SurfaceView vs{};
            vs.degreeU = ds.degreeU;
            vs.degreeV = ds.degreeV;
            vs.poles = {ds.poles.data(), ds.poles.size()};
            vs.weights = {ds.weights.data(), ds.weights.size()};
            vs.nRows = ds.nRows;
            vs.nCols = ds.nCols;
            vs.knotsU = {ds.knotsU.data(), ds.knotsU.size()};
            vs.knotsV = {ds.knotsV.data(), ds.knotsV.size()};
            const double useTol = tol > 0.0 ? tol : 1e-9;
            const nm::CurveSurfaceResult r = nm::intersectCurveSurface(vc, vs, useTol);
            if (r.status == nm::IntersectStatus::Coincident) {
                set_last_error(
                    "cc_nurbs_intersect_cs: curve lies on the surface over a sub-arc — "
                    "infinite intersection set, honest-declined");
                return -1;
            }
            const int n = static_cast<int>(r.hits.size());
            if (n == 0 || outHits == nullptr) {
                return n;
            }
            auto* out = static_cast<CCCurveSurfaceHit*>(
                std::malloc(static_cast<std::size_t>(n) * sizeof(CCCurveSurfaceHit)));
            if (out == nullptr) {
                set_last_error("cc_nurbs_intersect_cs: out of memory");
                return -1;
            }
            for (int i = 0; i < n; ++i) {
                const nm::CurveSurfaceHit& h = r.hits[static_cast<std::size_t>(i)];
                out[i].xyz[0] = h.point.x;
                out[i].xyz[1] = h.point.y;
                out[i].xyz[2] = h.point.z;
                out[i].t = h.paramT;
                out[i].u = h.paramU;
                out[i].v = h.paramV;
                out[i].tangential =
                    h.type == nm::IntersectionType::Tangential ? 1 : 0;
            }
            *outHits = out;
            return n;
#endif
        },
        -1);
}

void cc_nurbs_hits_cs_free(CCCurveSurfaceHit* hits) {
    std::free(hits);
}

// ── parameter-space trim-region boolean ────────────────────────────────────────

namespace {

namespace topo = cybercad::native::topology;

// Build a native TrimLoop from a cc_curve in (u,v): one BSpline PcurveSegment whose
// (x, y) poles become (u, v). Returns false on an unknown / invalid handle.
bool loopFromCurve(cc_curve h, topo::TrimLoop& loop) {
    const CurveData d = readCurve(h);
    if (!d.ok || d.poles.empty()) {
        return false;
    }
    topo::PcurveSegment seg;
    seg.curve.kind = topo::EdgeCurve::Kind::BSpline;
    seg.curve.degree = d.degree;
    seg.curve.poles2d.reserve(d.poles.size());
    for (const Point3& p : d.poles) {
        seg.curve.poles2d.push_back(Point3{p.x, p.y, 0.0});  // (u, v, 0)
    }
    seg.curve.weights = d.weights;  // empty ⇒ non-rational (exact for polygonal loops)
    seg.curve.knots = d.knots;
    // Clamped knot domain [knots[degree], knots[nCtrl]] is the segment's range.
    const std::size_t deg = static_cast<std::size_t>(d.degree);
    seg.first = d.knots[deg];
    seg.last = d.knots[d.poles.size()];
    seg.reversed = false;
    loop.clear();
    loop.push_back(std::move(seg));
    return true;
}

// Assemble a TrimRegion (outer = loop 0, holes = the rest) from an array of loops.
bool regionFromCurves(const cc_curve* loops, int nLoops, topo::TrimRegion& region) {
    if (loops == nullptr || nLoops < 1) {
        return false;
    }
    if (!loopFromCurve(loops[0], region.outer)) {
        return false;
    }
    region.holes.clear();
    for (int i = 1; i < nLoops; ++i) {
        topo::TrimLoop hole;
        if (!loopFromCurve(loops[i], hole)) {
            return false;
        }
        region.holes.push_back(std::move(hole));
    }
    return true;
}

topo::TrimBoolOp toNativeOp(CCTrimBoolOp op) {
    switch (op) {
        case CC_TRIM_INTERSECT:
            return topo::TrimBoolOp::Intersect;
        case CC_TRIM_DIFFERENCE:
            return topo::TrimBoolOp::Difference;
        case CC_TRIM_UNION:
        default:
            return topo::TrimBoolOp::Union;
    }
}

}  // namespace

int cc_nurbs_trim_region_boolean(const cc_curve* regionA, int nLoopsA,
                                 const cc_curve* regionB, int nLoopsB, CCTrimBoolOp op,
                                 CCTrimLoop** outLoops, double* area) {
    return guard(
        [&]() -> int {
            if (outLoops != nullptr) {
                *outLoops = nullptr;
            }
            if (area != nullptr) {
                *area = 0.0;
            }
            topo::TrimRegion rA;
            topo::TrimRegion rB;
            if (!regionFromCurves(regionA, nLoopsA, rA) ||
                !regionFromCurves(regionB, nLoopsB, rB)) {
                set_last_error(
                    "cc_nurbs_trim_region_boolean: unknown / invalid loop curve, or an "
                    "empty region (loop 0 is the outer loop)");
                return -1;
            }
            const topo::TrimBoolResult r =
                topo::trimRegionBoolean(rA, rB, toNativeOp(op));
            if (r.status == topo::TrimBoolStatus::Degenerate ||
                r.status == topo::TrimBoolStatus::Invalid) {
                set_last_error(
                    "cc_nurbs_trim_region_boolean: coincident-boundary / tangential-only "
                    "overlap or malformed loop — ambiguous region boolean, honest-declined");
                return -1;
            }
            // Ok or Empty: both are valid outcomes. Empty ⇒ 0 loops, area 0.
            if (area != nullptr) {
                *area = r.area;
            }
            const int n = static_cast<int>(r.loops.size());
            if (n == 0 || outLoops == nullptr) {
                return n;
            }
            auto* out = static_cast<CCTrimLoop*>(
                std::malloc(static_cast<std::size_t>(n) * sizeof(CCTrimLoop)));
            if (out == nullptr) {
                set_last_error("cc_nurbs_trim_region_boolean: out of memory");
                return -1;
            }
            for (int i = 0; i < n; ++i) {
                const topo::ResultLoop& L = r.loops[static_cast<std::size_t>(i)];
                const int m = static_cast<int>(L.poly.size());
                auto* uv = static_cast<double*>(
                    std::malloc(static_cast<std::size_t>(m) * 2 * sizeof(double)));
                if (uv == nullptr && m > 0) {
                    for (int k = 0; k < i; ++k) {
                        std::free(out[k].uv);
                    }
                    std::free(out);
                    set_last_error("cc_nurbs_trim_region_boolean: out of memory");
                    return -1;
                }
                for (int j = 0; j < m; ++j) {
                    uv[2 * j + 0] = L.poly[static_cast<std::size_t>(j)].u;
                    uv[2 * j + 1] = L.poly[static_cast<std::size_t>(j)].v;
                }
                out[i].uv = uv;
                out[i].pointCount = m;
                out[i].outer = L.outer ? 1 : 0;
                out[i].signedArea = L.signedArea;
            }
            *outLoops = out;
            return n;
        },
        -1);
}

void cc_nurbs_trim_loops_free(CCTrimLoop* loops, int count) {
    if (loops == nullptr) {
        return;
    }
    for (int i = 0; i < count; ++i) {
        std::free(loops[i].uv);
    }
    std::free(loops);
}
