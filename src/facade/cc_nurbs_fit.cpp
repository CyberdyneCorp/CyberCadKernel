// Public C facade — J2: NURBS fitting / reverse-engineering + analytic conversion.
//
// Thin, guarded delegations over the OCCT-FREE native modules bspline_fit /
// bspline_fair / bspline_simplify / primitive_fit / analytic_nurbs (src/native stays
// OCCT-free; the bridging lives here, like the rest of src/facade). Each entry point
// validates its raw C input, drives the native routine, and registers the result as a
// cc_curve / cc_surface — or HONEST-DECLINES with a 0 handle + cc_last_error, never a
// plausible-but-wrong handle and never a widened tolerance (design.md §3).
//
// CRITICAL: this file crosses the handle boundary using ONLY J1's PUBLIC surface —
// it reads an input cc_curve / cc_surface through cc_curve_info / cc_curve_knots /
// cc_curve_poles (+ surface equivalents) into a native Bspline{Curve,Surface}Data, and
// registers a result via the PUBLIC cc_curve_create / cc_surface_create. It never
// reaches into J1's internal registry (the conversion helpers below are local statics).
//
// The whole file's geometry work goes through native modules that are compiled only
// under CYBERCAD_HAS_NUMSCI. With the guard OFF every wrapper is an inert honest
// decline (id 0 + cc_last_error), so the ABI symbols are always present.

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "core/guard.h"
#include "cybercadkernel/cc_kernel.h"

#ifdef CYBERCAD_HAS_NUMSCI
#include "native/math/analytic_nurbs.h"
#include "native/math/bspline_fair.h"
#include "native/math/bspline_fit.h"
#include "native/math/bspline_ops.h"
#include "native/math/bspline_simplify.h"
#include "native/math/elementary.h"
#include "native/math/primitive_fit.h"
#include "native/math/torus.h"
#include "native/math/vec.h"
#endif

using cyber::guard;
using cyber::set_last_error;

namespace {

#ifdef CYBERCAD_HAS_NUMSCI

namespace nm = cybercad::native::math;

// ── J1-public handle <-> native conversion (PUBLIC surface only) ──────────────
//
// Read an input cc_curve / cc_surface back into a native BsplineCurveData /
// BsplineSurfaceData via the public accessors, and register a native result as a
// handle via the public constructors. No reach into J1's registry.

// Reconstruct a native curve from a live cc_curve. Returns false (no data) on an
// unknown handle.
bool curveFromHandle(cc_curve h, nm::BsplineCurveData& out) {
    CCCurveInfo info{};
    if (!cc_curve_info(h, &info)) {
        return false;
    }
    std::vector<double> knots(static_cast<std::size_t>(info.n_knots));
    std::vector<double> poles(static_cast<std::size_t>(info.n_ctrl) * 4);
    if (cc_curve_knots(h, knots.data(), info.n_knots) != info.n_knots) {
        return false;
    }
    if (cc_curve_poles(h, poles.data(), info.n_ctrl * 4) != info.n_ctrl * 4) {
        return false;
    }
    out.degree = info.degree;
    out.knots = std::move(knots);
    out.poles.resize(static_cast<std::size_t>(info.n_ctrl));
    out.weights.clear();
    bool rational = false;
    for (int i = 0; i < info.n_ctrl; ++i) {
        out.poles[static_cast<std::size_t>(i)] =
            nm::Point3{poles[4 * i + 0], poles[4 * i + 1], poles[4 * i + 2]};
        const double w = poles[4 * i + 3];
        if (w != 1.0) {
            rational = true;
        }
    }
    if (rational) {
        out.weights.resize(static_cast<std::size_t>(info.n_ctrl));
        for (int i = 0; i < info.n_ctrl; ++i) {
            out.weights[static_cast<std::size_t>(i)] = poles[4 * i + 3];
        }
    }
    return true;
}

bool surfaceFromHandle(cc_surface h, nm::BsplineSurfaceData& out) {
    CCSurfaceInfo info{};
    if (!cc_surface_info(h, &info)) {
        return false;
    }
    const int total = info.n_ctrl_u * info.n_ctrl_v;
    std::vector<double> knotsU(static_cast<std::size_t>(info.n_knots_u));
    std::vector<double> knotsV(static_cast<std::size_t>(info.n_knots_v));
    std::vector<double> poles(static_cast<std::size_t>(total) * 4);
    if (cc_surface_knots_u(h, knotsU.data(), info.n_knots_u) != info.n_knots_u ||
        cc_surface_knots_v(h, knotsV.data(), info.n_knots_v) != info.n_knots_v ||
        cc_surface_poles(h, poles.data(), total * 4) != total * 4) {
        return false;
    }
    out.degreeU = info.degree_u;
    out.degreeV = info.degree_v;
    out.nPolesU = info.n_ctrl_u;
    out.nPolesV = info.n_ctrl_v;
    out.knotsU = std::move(knotsU);
    out.knotsV = std::move(knotsV);
    out.poles.resize(static_cast<std::size_t>(total));
    out.weights.clear();
    bool rational = false;
    for (int i = 0; i < total; ++i) {
        out.poles[static_cast<std::size_t>(i)] =
            nm::Point3{poles[4 * i + 0], poles[4 * i + 1], poles[4 * i + 2]};
        if (poles[4 * i + 3] != 1.0) {
            rational = true;
        }
    }
    if (rational) {
        out.weights.resize(static_cast<std::size_t>(total));
        for (int i = 0; i < total; ++i) {
            out.weights[static_cast<std::size_t>(i)] = poles[4 * i + 3];
        }
    }
    return true;
}

// Register a native curve as a cc_curve via the PUBLIC constructor. Packs the
// Euclidean poles + weight into the (x,y,z,w) stream cc_curve_create expects.
cc_curve registerCurve(const nm::BsplineCurveData& c) {
    const int nCtrl = static_cast<int>(c.poles.size());
    std::vector<double> xyzw(static_cast<std::size_t>(nCtrl) * 4);
    for (int i = 0; i < nCtrl; ++i) {
        xyzw[4 * i + 0] = c.poles[static_cast<std::size_t>(i)].x;
        xyzw[4 * i + 1] = c.poles[static_cast<std::size_t>(i)].y;
        xyzw[4 * i + 2] = c.poles[static_cast<std::size_t>(i)].z;
        xyzw[4 * i + 3] = c.weights.empty() ? 1.0 : c.weights[static_cast<std::size_t>(i)];
    }
    return cc_curve_create(c.degree, xyzw.data(), nCtrl, c.knots.data(),
                           static_cast<int>(c.knots.size()));
}

cc_surface registerSurface(const nm::BsplineSurfaceData& s) {
    const int total = s.nPolesU * s.nPolesV;
    std::vector<double> xyzw(static_cast<std::size_t>(total) * 4);
    for (int i = 0; i < total; ++i) {
        xyzw[4 * i + 0] = s.poles[static_cast<std::size_t>(i)].x;
        xyzw[4 * i + 1] = s.poles[static_cast<std::size_t>(i)].y;
        xyzw[4 * i + 2] = s.poles[static_cast<std::size_t>(i)].z;
        xyzw[4 * i + 3] =
            s.weights.empty() ? 1.0 : s.weights[static_cast<std::size_t>(i)];
    }
    return cc_surface_create(s.degreeU, s.degreeV, xyzw.data(), s.nPolesU, s.nPolesV,
                             s.knotsU.data(), static_cast<int>(s.knotsU.size()),
                             s.knotsV.data(), static_cast<int>(s.knotsV.size()));
}

// ── input-marshalling helpers ─────────────────────────────────────────────────

nm::ParamMethod paramOf(int m) {
    switch (m) {
        case 0:
            return nm::ParamMethod::Uniform;
        case 2:
            return nm::ParamMethod::Centripetal;
        default:
            return nm::ParamMethod::ChordLength;  // 1 and anything else
    }
}

// Unpack a packed 3*n double stream into Point3s. Caller has validated n / ptr.
std::vector<nm::Point3> unpackPoints(const double* xyz, int n) {
    std::vector<nm::Point3> pts(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        pts[static_cast<std::size_t>(i)] =
            nm::Point3{xyz[3 * i + 0], xyz[3 * i + 1], xyz[3 * i + 2]};
    }
    return pts;
}

nm::Dir3 dirOf(const double* d) { return nm::Dir3(d[0], d[1], d[2]); }
nm::Point3 pointOf(const double* p) { return nm::Point3{p[0], p[1], p[2]}; }

void writeVec(double* out, const nm::Vec3& v) {
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
}
void writeVec(double* out, const nm::Point3& p) {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
}
void writeVec(double* out, const nm::Dir3& d) { writeVec(out, d.vec()); }

// Build a right-handed frame from an origin, main axis and reference X; declines
// (returns false) when either direction is degenerate.
bool frameOf(const double* origin, const double* axis, const double* xref, nm::Ax3& out) {
    const nm::Dir3 z = dirOf(axis);
    const nm::Dir3 xr = dirOf(xref);
    if (!z.valid() || !xr.valid()) {
        return false;
    }
    out = nm::Ax3::fromAxisAndRef(pointOf(origin), z, xr);
    return out.x.valid() && out.y.valid() && out.z.valid();
}

#endif  // CYBERCAD_HAS_NUMSCI

// Guard-body helpers usable in BOTH configs: the numsci-off decline.
constexpr const char* kNoNumsci =
    "cc_nurbs_*: fitting / analytic-conversion requires a CYBERCAD_HAS_NUMSCI build";

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Fitting / approximation
// ─────────────────────────────────────────────────────────────────────────────

cc_curve cc_nurbs_fit_curve(const double* pointsXYZ, int n_points, int degree,
                            int n_ctrl, int param_method) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            if (pointsXYZ == nullptr || n_points < 2) {
                set_last_error("cc_nurbs_fit_curve: need >= 2 points");
                return cc_curve{0};
            }
            const auto pts = unpackPoints(pointsXYZ, n_points);
            const nm::CurveFitResult r =
                nm::approximateCurve(pts, n_ctrl, degree, paramOf(param_method));
            if (!r.ok) {
                set_last_error("cc_nurbs_fit_curve: fit declined (bad dims / degenerate)");
                return cc_curve{0};
            }
            return registerCurve(r.curve);
#else
            (void)pointsXYZ;
            (void)n_points;
            (void)degree;
            (void)n_ctrl;
            (void)param_method;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

cc_curve cc_nurbs_interp_curve(const double* pointsXYZ, int n_points, int degree,
                               int param_method) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            if (pointsXYZ == nullptr || n_points < 2) {
                set_last_error("cc_nurbs_interp_curve: need >= 2 points");
                return cc_curve{0};
            }
            const auto pts = unpackPoints(pointsXYZ, n_points);
            const nm::CurveFitResult r =
                nm::interpolateCurve(pts, degree, paramOf(param_method));
            if (!r.ok) {
                set_last_error(
                    "cc_nurbs_interp_curve: interpolation declined (need >= degree+1 "
                    "distinct points)");
                return cc_curve{0};
            }
            return registerCurve(r.curve);
#else
            (void)pointsXYZ;
            (void)n_points;
            (void)degree;
            (void)param_method;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

cc_surface cc_nurbs_fit_surface(const double* gridXYZ, int n_u, int n_v, int degree_u,
                                int degree_v, int n_ctrl_u, int n_ctrl_v,
                                int param_method) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            if (gridXYZ == nullptr || n_u < 2 || n_v < 2) {
                set_last_error("cc_nurbs_fit_surface: need a >= 2x2 grid");
                return cc_surface{0};
            }
            const auto pts = unpackPoints(gridXYZ, n_u * n_v);
            const nm::PointGrid grid{pts, n_u, n_v};
            const nm::SurfaceFitResult r = nm::approximateSurface(
                grid, n_ctrl_u, n_ctrl_v, degree_u, degree_v, paramOf(param_method));
            if (!r.ok) {
                set_last_error("cc_nurbs_fit_surface: fit declined (bad dims / degenerate)");
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#else
            (void)gridXYZ;
            (void)n_u;
            (void)n_v;
            (void)degree_u;
            (void)degree_v;
            (void)n_ctrl_u;
            (void)n_ctrl_v;
            (void)param_method;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational weight ESTIMATION (Ma–Kruth)
// ─────────────────────────────────────────────────────────────────────────────

cc_curve cc_nurbs_estimate_weights_curve(const double* pointsXYZ, int n_points,
                                         int degree, int n_ctrl, int param_method) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            if (pointsXYZ == nullptr || n_points < 2) {
                set_last_error("cc_nurbs_estimate_weights_curve: need >= 2 points");
                return cc_curve{0};
            }
            const auto pts = unpackPoints(pointsXYZ, n_points);
            const nm::RationalFitResult r = nm::fitRationalCurveEstimateWeights(
                pts, n_ctrl, degree, paramOf(param_method));
            if (!r.ok) {
                std::string why = "cc_nurbs_estimate_weights_curve: declined";
                if (r.diagnostic != nullptr && r.diagnostic[0] != '\0') {
                    why += std::string(" (") + r.diagnostic + ")";
                }
                set_last_error(why);
                return cc_curve{0};
            }
            return registerCurve(r.curve);
#else
            (void)pointsXYZ;
            (void)n_points;
            (void)degree;
            (void)n_ctrl;
            (void)param_method;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

cc_surface cc_nurbs_estimate_weights_surface(const double* gridXYZ, int n_u, int n_v,
                                             int degree_u, int degree_v, int n_ctrl_u,
                                             int n_ctrl_v, int param_method) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            if (gridXYZ == nullptr || n_u < 2 || n_v < 2) {
                set_last_error("cc_nurbs_estimate_weights_surface: need a >= 2x2 grid");
                return cc_surface{0};
            }
            const auto pts = unpackPoints(gridXYZ, n_u * n_v);
            const nm::PointGrid grid{pts, n_u, n_v};
            const nm::RationalSurfaceFitResult r = nm::fitRationalSurfaceEstimateWeights(
                grid, n_ctrl_u, n_ctrl_v, degree_u, degree_v, paramOf(param_method));
            if (!r.ok) {
                std::string why = "cc_nurbs_estimate_weights_surface: declined";
                if (r.diagnostic != nullptr && r.diagnostic[0] != '\0') {
                    why += std::string(" (") + r.diagnostic + ")";
                }
                set_last_error(why);
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#else
            (void)gridXYZ;
            (void)n_u;
            (void)n_v;
            (void)degree_u;
            (void)degree_v;
            (void)n_ctrl_u;
            (void)n_ctrl_v;
            (void)param_method;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}

// ─────────────────────────────────────────────────────────────────────────────
// Equality-CONSTRAINED least-squares fitting
// ─────────────────────────────────────────────────────────────────────────────

cc_curve cc_nurbs_fit_curve_constrained(const double* pointsXYZ, int n_points,
                                        const CCCurveEndConstraint* constraints,
                                        int n_constraints, int degree, int n_ctrl,
                                        int param_method) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            if (pointsXYZ == nullptr || n_points < 2) {
                set_last_error("cc_nurbs_fit_curve_constrained: need >= 2 points");
                return cc_curve{0};
            }
            if (n_constraints < 0 || (n_constraints > 0 && constraints == nullptr)) {
                set_last_error("cc_nurbs_fit_curve_constrained: null constraint array");
                return cc_curve{0};
            }
            const auto pts = unpackPoints(pointsXYZ, n_points);
            std::vector<nm::CurveEndConstraint> cons(
                static_cast<std::size_t>(n_constraints));
            for (int i = 0; i < n_constraints; ++i) {
                const CCCurveEndConstraint& in = constraints[i];
                nm::CurveEndConstraint& c = cons[static_cast<std::size_t>(i)];
                c.end = (in.end == 1) ? nm::CurveEnd::End : nm::CurveEnd::Start;
                c.order = in.order;
                c.value = nm::Vec3{in.value[0], in.value[1], in.value[2]};
            }
            const nm::CurveFitResult r = nm::fitCurveConstrained(
                pts, cons, degree, n_ctrl, paramOf(param_method));
            if (!r.ok) {
                set_last_error(
                    "cc_nurbs_fit_curve_constrained: declined (bad dims / over-"
                    "constrained / singular)");
                return cc_curve{0};
            }
            return registerCurve(r.curve);
#else
            (void)pointsXYZ;
            (void)n_points;
            (void)constraints;
            (void)n_constraints;
            (void)degree;
            (void)n_ctrl;
            (void)param_method;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

cc_surface cc_nurbs_fit_surface_constrained(const double* gridXYZ, int n_u, int n_v,
                                            const CCSurfacePoleConstraint* constraints,
                                            int n_constraints, int degree_u,
                                            int degree_v, int n_ctrl_u, int n_ctrl_v,
                                            int param_method) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            if (gridXYZ == nullptr || n_u < 2 || n_v < 2) {
                set_last_error("cc_nurbs_fit_surface_constrained: need a >= 2x2 grid");
                return cc_surface{0};
            }
            if (n_constraints < 0 || (n_constraints > 0 && constraints == nullptr)) {
                set_last_error("cc_nurbs_fit_surface_constrained: null constraint array");
                return cc_surface{0};
            }
            const auto pts = unpackPoints(gridXYZ, n_u * n_v);
            const nm::PointGrid grid{pts, n_u, n_v};
            std::vector<nm::SurfacePoleConstraint> cons(
                static_cast<std::size_t>(n_constraints));
            for (int i = 0; i < n_constraints; ++i) {
                const CCSurfacePoleConstraint& in = constraints[i];
                nm::SurfacePoleConstraint& c = cons[static_cast<std::size_t>(i)];
                c.i = in.i;
                c.j = in.j;
                c.value = nm::Vec3{in.value[0], in.value[1], in.value[2]};
            }
            const nm::SurfaceFitResult r = nm::fitSurfaceConstrained(
                grid, cons, degree_u, degree_v, n_ctrl_u, n_ctrl_v,
                paramOf(param_method));
            if (!r.ok) {
                set_last_error(
                    "cc_nurbs_fit_surface_constrained: declined (bad dims / bad index / "
                    "over-constrained / singular)");
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#else
            (void)gridXYZ;
            (void)n_u;
            (void)n_v;
            (void)constraints;
            (void)n_constraints;
            (void)degree_u;
            (void)degree_v;
            (void)n_ctrl_u;
            (void)n_ctrl_v;
            (void)param_method;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}

// ─────────────────────────────────────────────────────────────────────────────
// Fairing
// ─────────────────────────────────────────────────────────────────────────────

cc_curve cc_nurbs_fair_curve(cc_curve in, double tol, int keep_ends) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            nm::BsplineCurveData c;
            if (!curveFromHandle(in, c)) {
                set_last_error("cc_nurbs_fair_curve: unknown curve handle");
                return cc_curve{0};
            }
            const nm::CurveFairResult r = nm::fairCurve(c, tol, keep_ends != 0);
            if (!r.ok) {
                set_last_error(
                    "cc_nurbs_fair_curve: declined (tol too tight / rational / too small)");
                return cc_curve{0};
            }
            return registerCurve(r.curve);
#else
            (void)in;
            (void)tol;
            (void)keep_ends;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

cc_surface cc_nurbs_fair_surface(cc_surface in, double tol, int keep_boundary) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            nm::BsplineSurfaceData s;
            if (!surfaceFromHandle(in, s)) {
                set_last_error("cc_nurbs_fair_surface: unknown surface handle");
                return cc_surface{0};
            }
            const nm::SurfaceFairResult r = nm::fairSurface(s, tol, keep_boundary != 0);
            if (!r.ok) {
                set_last_error(
                    "cc_nurbs_fair_surface: declined (tol too tight / rational / too "
                    "small)");
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#else
            (void)in;
            (void)tol;
            (void)keep_boundary;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}

// ─────────────────────────────────────────────────────────────────────────────
// Simplification
// ─────────────────────────────────────────────────────────────────────────────

cc_curve cc_nurbs_simplify_curve(cc_curve in, double tol) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            nm::BsplineCurveData c;
            if (!curveFromHandle(in, c)) {
                set_last_error("cc_nurbs_simplify_curve: unknown curve handle");
                return cc_curve{0};
            }
            if (!(tol >= 0.0)) {  // rejects negative / NaN
                set_last_error("cc_nurbs_simplify_curve: tol must be >= 0");
                return cc_curve{0};
            }
            const nm::BoundedRemovalResult r = nm::removeKnotsBounded(c, tol);
            // Bounded removal never widens: nothing removed ⇒ the input geometry.
            return registerCurve(r.curve);
#else
            (void)in;
            (void)tol;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

// ─────────────────────────────────────────────────────────────────────────────
// Reverse-engineering: primitive detection / recognition
// ─────────────────────────────────────────────────────────────────────────────

int cc_nurbs_detect_primitive(const double* pointsXYZ, int n_points, double rel_tol,
                              CCPrimitiveDetection* out) {
    return guard(
        [&]() -> int {
            if (out != nullptr) {
                *out = CCPrimitiveDetection{};
            }
#ifdef CYBERCAD_HAS_NUMSCI
            if (out == nullptr || pointsXYZ == nullptr || n_points < 3) {
                set_last_error("cc_nurbs_detect_primitive: null out or < 3 points");
                return 0;
            }
            const auto pts = unpackPoints(pointsXYZ, n_points);
            const double tol = (rel_tol > 0.0) ? rel_tol : 1e-6;
            const nm::PrimitiveDetection d = nm::detectPrimitive(pts, tol);
            out->rms = d.rms;
            out->rel_error = d.relError;
            switch (d.type) {
                case nm::PrimitiveType::Plane:
                    out->type = CC_PRIM_PLANE;
                    writeVec(out->plane_normal, d.plane.normal);
                    out->plane_offset = d.plane.offset;
                    writeVec(out->center, d.plane.centroid);
                    break;
                case nm::PrimitiveType::Sphere:
                    out->type = CC_PRIM_SPHERE;
                    writeVec(out->center, d.sphere.center);
                    out->radius = d.sphere.radius;
                    break;
                case nm::PrimitiveType::Cylinder:
                    out->type = CC_PRIM_CYLINDER;
                    writeVec(out->center, d.cylinder.axisPoint);
                    writeVec(out->axis, d.cylinder.axis);
                    out->radius = d.cylinder.radius;
                    break;
                case nm::PrimitiveType::Cone:
                    out->type = CC_PRIM_CONE;
                    writeVec(out->center, d.cone.apex);
                    writeVec(out->axis, d.cone.axis);
                    out->half_angle = d.cone.halfAngle;
                    break;
                case nm::PrimitiveType::Freeform:
                default:
                    out->type = CC_PRIM_FREEFORM;
                    break;
            }
            return 1;
#else
            (void)pointsXYZ;
            (void)n_points;
            (void)rel_tol;
            set_last_error(kNoNumsci);
            return 0;
#endif
        },
        0);
}

int cc_nurbs_recognize_curve(cc_curve h, double tol, CCCurveRecognition* out) {
    return guard(
        [&]() -> int {
            if (out != nullptr) {
                *out = CCCurveRecognition{};
            }
#ifdef CYBERCAD_HAS_NUMSCI
            nm::BsplineCurveData c;
            if (out == nullptr || !curveFromHandle(h, c)) {
                set_last_error("cc_nurbs_recognize_curve: unknown handle or null out");
                return 0;
            }
            const double t = (tol > 0.0) ? tol : 1e-12;
            const nm::CurveRecognition r = nm::recognizeCurve(c, t);
            out->residual = r.residual;
            switch (r.kind) {
                case nm::CurveKind::Line:
                    out->kind = CC_CURVE_LINE;
                    writeVec(out->line_start, r.line.start);
                    writeVec(out->line_end, r.line.end);
                    break;
                case nm::CurveKind::Circle:
                    out->kind = CC_CURVE_CIRCLE;
                    writeVec(out->center, r.circle.center);
                    writeVec(out->normal, r.circle.normal);
                    writeVec(out->x_axis, r.circle.xAxis);
                    out->radius = r.circle.radius;
                    break;
                case nm::CurveKind::Arc:
                    out->kind = CC_CURVE_ARC;
                    writeVec(out->center, r.arc.circle.center);
                    writeVec(out->normal, r.arc.circle.normal);
                    writeVec(out->x_axis, r.arc.circle.xAxis);
                    out->radius = r.arc.circle.radius;
                    out->start_angle = r.arc.startAngle;
                    out->sweep_angle = r.arc.sweepAngle;
                    break;
                case nm::CurveKind::Ellipse:
                    out->kind = CC_CURVE_ELLIPSE;
                    writeVec(out->center, r.ellipse.center);
                    writeVec(out->normal, r.ellipse.normal);
                    writeVec(out->x_axis, r.ellipse.xAxis);
                    out->radius = r.ellipse.majorRadius;
                    out->minor_radius = r.ellipse.minorRadius;
                    break;
                case nm::CurveKind::General:
                default:
                    out->kind = CC_CURVE_GENERAL;
                    break;
            }
            return 1;
#else
            (void)h;
            (void)tol;
            set_last_error(kNoNumsci);
            return 0;
#endif
        },
        0);
}

int cc_nurbs_recognize_surface(cc_surface h, double tol, CCSurfaceRecognition* out) {
    return guard(
        [&]() -> int {
            if (out != nullptr) {
                *out = CCSurfaceRecognition{};
            }
#ifdef CYBERCAD_HAS_NUMSCI
            nm::BsplineSurfaceData s;
            if (out == nullptr || !surfaceFromHandle(h, s)) {
                set_last_error("cc_nurbs_recognize_surface: unknown handle or null out");
                return 0;
            }
            const double t = (tol > 0.0) ? tol : 1e-12;
            const nm::SurfaceRecognition r = nm::recognizeSurface(s, t);
            out->residual = r.residual;
            switch (r.kind) {
                case nm::SurfaceKind::Plane:
                    out->kind = CC_SURF_PLANE;
                    writeVec(out->origin, r.plane.pos.origin);
                    writeVec(out->axis, r.plane.pos.z);
                    writeVec(out->x_axis, r.plane.pos.x);
                    break;
                case nm::SurfaceKind::Cylinder:
                    out->kind = CC_SURF_CYLINDER;
                    writeVec(out->origin, r.cylinder.pos.origin);
                    writeVec(out->axis, r.cylinder.pos.z);
                    writeVec(out->x_axis, r.cylinder.pos.x);
                    out->radius = r.cylinder.radius;
                    break;
                case nm::SurfaceKind::Cone:
                    out->kind = CC_SURF_CONE;
                    writeVec(out->origin, r.cone.pos.origin);
                    writeVec(out->axis, r.cone.pos.z);
                    writeVec(out->x_axis, r.cone.pos.x);
                    out->radius = r.cone.radius;
                    out->half_angle = r.cone.semiAngle;
                    break;
                case nm::SurfaceKind::Sphere:
                    out->kind = CC_SURF_SPHERE;
                    writeVec(out->origin, r.sphere.pos.origin);
                    writeVec(out->axis, r.sphere.pos.z);
                    writeVec(out->x_axis, r.sphere.pos.x);
                    out->radius = r.sphere.radius;
                    break;
                case nm::SurfaceKind::General:
                default:
                    out->kind = CC_SURF_GENERAL;
                    break;
            }
            return 1;
#else
            (void)h;
            (void)tol;
            set_last_error(kNoNumsci);
            return 0;
#endif
        },
        0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Analytic -> exact rational NURBS
// ─────────────────────────────────────────────────────────────────────────────

cc_curve cc_nurbs_circle(const double* center, const double* normal,
                         const double* x_axis, double radius) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            if (center == nullptr || normal == nullptr || x_axis == nullptr) {
                set_last_error("cc_nurbs_circle: null argument");
                return cc_curve{0};
            }
            const nm::Dir3 n = dirOf(normal);
            const nm::Dir3 x = dirOf(x_axis);
            if (!(radius > 0.0) || !n.valid() || !x.valid()) {
                set_last_error("cc_nurbs_circle: bad radius or degenerate frame");
                return cc_curve{0};
            }
            nm::Circle c;
            c.center = pointOf(center);
            c.normal = n;
            c.xAxis = x;
            c.radius = radius;
            return registerCurve(nm::circleToNurbs(c));
#else
            (void)center;
            (void)normal;
            (void)x_axis;
            (void)radius;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

cc_curve cc_nurbs_arc(const double* center, const double* normal, const double* x_axis,
                      double radius, double start_angle, double sweep_angle) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            if (center == nullptr || normal == nullptr || x_axis == nullptr) {
                set_last_error("cc_nurbs_arc: null argument");
                return cc_curve{0};
            }
            const nm::Dir3 n = dirOf(normal);
            const nm::Dir3 x = dirOf(x_axis);
            constexpr double kTwoPi = 6.283185307179586476925286766559;
            if (!(radius > 0.0) || !n.valid() || !x.valid() ||
                !(sweep_angle > 0.0) || sweep_angle > kTwoPi + 1e-12) {
                set_last_error("cc_nurbs_arc: bad radius / frame / sweep (0 < sweep <= 2pi)");
                return cc_curve{0};
            }
            nm::Arc a;
            a.circle.center = pointOf(center);
            a.circle.normal = n;
            a.circle.xAxis = x;
            a.circle.radius = radius;
            a.startAngle = start_angle;
            a.sweepAngle = sweep_angle;
            return registerCurve(nm::arcToNurbs(a));
#else
            (void)center;
            (void)normal;
            (void)x_axis;
            (void)radius;
            (void)start_angle;
            (void)sweep_angle;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

cc_curve cc_nurbs_ellipse(const double* center, const double* normal,
                          const double* x_axis, double major_radius,
                          double minor_radius) {
    return guard(
        [&]() -> cc_curve {
#ifdef CYBERCAD_HAS_NUMSCI
            if (center == nullptr || normal == nullptr || x_axis == nullptr) {
                set_last_error("cc_nurbs_ellipse: null argument");
                return cc_curve{0};
            }
            const nm::Dir3 n = dirOf(normal);
            const nm::Dir3 x = dirOf(x_axis);
            if (!(major_radius > 0.0) || !(minor_radius > 0.0) || !n.valid() ||
                !x.valid()) {
                set_last_error("cc_nurbs_ellipse: bad semi-axis or degenerate frame");
                return cc_curve{0};
            }
            nm::Ellipse e;
            e.center = pointOf(center);
            e.normal = n;
            e.xAxis = x;
            e.majorRadius = major_radius;
            e.minorRadius = minor_radius;
            return registerCurve(nm::ellipseToNurbs(e));
#else
            (void)center;
            (void)normal;
            (void)x_axis;
            (void)major_radius;
            (void)minor_radius;
            set_last_error(kNoNumsci);
            return cc_curve{0};
#endif
        },
        cc_curve{0});
}

cc_surface cc_nurbs_plane(const double* origin, const double* normal,
                          const double* x_axis, double u0, double u1, double v0,
                          double v1) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            if (origin == nullptr || normal == nullptr || x_axis == nullptr) {
                set_last_error("cc_nurbs_plane: null argument");
                return cc_surface{0};
            }
            if (!(u1 > u0) || !(v1 > v0)) {
                set_last_error("cc_nurbs_plane: empty window (need u1>u0 and v1>v0)");
                return cc_surface{0};
            }
            nm::Ax3 frame;
            if (!frameOf(origin, normal, x_axis, frame)) {
                set_last_error("cc_nurbs_plane: degenerate frame");
                return cc_surface{0};
            }
            nm::Plane p;
            p.pos = frame;
            return registerSurface(nm::planeToNurbs(p, u0, u1, v0, v1));
#else
            (void)origin;
            (void)normal;
            (void)x_axis;
            (void)u0;
            (void)u1;
            (void)v0;
            (void)v1;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}

cc_surface cc_nurbs_cylinder(const double* origin, const double* axis,
                             const double* x_axis, double radius, double v0,
                             double v1) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            if (origin == nullptr || axis == nullptr || x_axis == nullptr) {
                set_last_error("cc_nurbs_cylinder: null argument");
                return cc_surface{0};
            }
            if (!(radius > 0.0) || !(v1 > v0)) {
                set_last_error("cc_nurbs_cylinder: bad radius or empty height");
                return cc_surface{0};
            }
            nm::Ax3 frame;
            if (!frameOf(origin, axis, x_axis, frame)) {
                set_last_error("cc_nurbs_cylinder: degenerate frame");
                return cc_surface{0};
            }
            nm::Cylinder c;
            c.pos = frame;
            c.radius = radius;
            return registerSurface(nm::cylinderToNurbs(c, v0, v1));
#else
            (void)origin;
            (void)axis;
            (void)x_axis;
            (void)radius;
            (void)v0;
            (void)v1;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}

cc_surface cc_nurbs_cone(const double* origin, const double* axis,
                         const double* x_axis, double radius, double semi_angle,
                         double v0, double v1) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            if (origin == nullptr || axis == nullptr || x_axis == nullptr) {
                set_last_error("cc_nurbs_cone: null argument");
                return cc_surface{0};
            }
            constexpr double kHalfPi = 1.5707963267948966192313216916398;
            if (!(radius >= 0.0) || !(v1 > v0) ||
                !(semi_angle > 0.0) || !(semi_angle < kHalfPi)) {
                set_last_error(
                    "cc_nurbs_cone: bad radius / height, or half-angle out of (0, pi/2)");
                return cc_surface{0};
            }
            nm::Ax3 frame;
            if (!frameOf(origin, axis, x_axis, frame)) {
                set_last_error("cc_nurbs_cone: degenerate frame");
                return cc_surface{0};
            }
            nm::Cone c;
            c.pos = frame;
            c.radius = radius;
            c.semiAngle = semi_angle;
            return registerSurface(nm::coneToNurbs(c, v0, v1));
#else
            (void)origin;
            (void)axis;
            (void)x_axis;
            (void)radius;
            (void)semi_angle;
            (void)v0;
            (void)v1;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}

cc_surface cc_nurbs_sphere(const double* center, const double* axis,
                           const double* x_axis, double radius) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            if (center == nullptr || axis == nullptr || x_axis == nullptr) {
                set_last_error("cc_nurbs_sphere: null argument");
                return cc_surface{0};
            }
            if (!(radius > 0.0)) {
                set_last_error("cc_nurbs_sphere: radius must be > 0");
                return cc_surface{0};
            }
            nm::Ax3 frame;
            if (!frameOf(center, axis, x_axis, frame)) {
                set_last_error("cc_nurbs_sphere: degenerate frame");
                return cc_surface{0};
            }
            nm::Sphere s;
            s.pos = frame;
            s.radius = radius;
            return registerSurface(nm::sphereToNurbs(s));
#else
            (void)center;
            (void)axis;
            (void)x_axis;
            (void)radius;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}

cc_surface cc_nurbs_torus(const double* center, const double* axis,
                          const double* x_axis, double major_radius,
                          double minor_radius) {
    return guard(
        [&]() -> cc_surface {
#ifdef CYBERCAD_HAS_NUMSCI
            if (center == nullptr || axis == nullptr || x_axis == nullptr) {
                set_last_error("cc_nurbs_torus: null argument");
                return cc_surface{0};
            }
            // Ring torus only: r > 0 and R >= r (a spindle torus self-intersects).
            if (!(minor_radius > 0.0) || !(major_radius >= minor_radius)) {
                set_last_error(
                    "cc_nurbs_torus: need minor > 0 and major >= minor (ring torus)");
                return cc_surface{0};
            }
            nm::Ax3 frame;
            if (!frameOf(center, axis, x_axis, frame)) {
                set_last_error("cc_nurbs_torus: degenerate frame");
                return cc_surface{0};
            }
            nm::Torus t;
            t.pos = frame;
            t.majorRadius = major_radius;
            t.minorRadius = minor_radius;
            return registerSurface(nm::torusToNurbs(t));
#else
            (void)center;
            (void)axis;
            (void)x_axis;
            (void)major_radius;
            (void)minor_radius;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#endif
        },
        cc_surface{0});
}
