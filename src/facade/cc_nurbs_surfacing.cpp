// Public C facade — J3 additive NURBS SURFACING wrappers (cc_nurbs_skin / gordon /
// coons / nsided_fill / sweep_variable / sweep_two_rail / revolve / join).
//
// This file is J3's alone (the wave-parallelism rule: J2–J5 are file-disjoint). It
// bridges the OCCT-free native Layer-6 surfacing modules (bspline_skin / gordon /
// coons / nsided{,_g1,_g2} / sweep / join) across the plain-C boundary the same way
// cc_kernel_nurbs.cpp bridges the geometry ABI:
//
//   * INPUT curves/surfaces are read through the PUBLIC J1 accessors ONLY
//     (cc_curve_info/_knots/_poles, cc_surface_info/_knots_u/_v/_poles) into native
//     BsplineCurveData / BsplineSurfaceData — J3 never touches J1's internal
//     registry.
//   * RESULTS are registered through the PUBLIC J1 constructors (cc_surface_create),
//     which round-trip the homogeneous (x,y,z,w) pole convention exactly.
//   * Honest-decline is a 0 (invalid) handle (or a count < 0) + cc_last_error, NEVER
//     a plausible-but-wrong handle and NEVER a widened tolerance. A G2-infeasible
//     creased N-gon declines; it is not filled with a residual crease.
//
// BUILD GATE — the native surfacing modules compile only under CYBERCAD_HAS_NUMSCI
// (they compose the numsci-backed skin/fit solves). With that macro OFF every
// wrapper honest-declines; the ABI symbols are always present, only the capability
// is gated (mirrors bspline_skin.cpp's own guard).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "core/guard.h"
#include "cybercadkernel/cc_kernel.h"
#include "native/math/bspline_ops.h"  // BsplineCurveData / BsplineSurfaceData (always visible)

#ifdef CYBERCAD_HAS_NUMSCI
#include "native/math/bspline_coons.h"
#include "native/math/bspline_gordon.h"
#include "native/math/bspline_join.h"
#include "native/math/bspline_nsided.h"
#include "native/math/bspline_nsided_g1.h"
#include "native/math/bspline_nsided_g2.h"
#include "native/math/bspline_skin.h"
#include "native/math/bspline_sweep.h"
#endif

namespace {

using cyber::guard;
using cyber::set_last_error;
using cybercad::native::math::BsplineCurveData;
using cybercad::native::math::BsplineSurfaceData;
using cybercad::native::math::Point3;

// ── read J1 handles into native data via the PUBLIC accessors ──────────────────

// Read a cc_curve into a native BsplineCurveData (empty weights ⇒ non-rational).
// Returns false (no error set) on an unknown / stale handle.
bool readCurve(cc_curve h, BsplineCurveData& out) {
    CCCurveInfo info{};
    if (cc_curve_info(h, &info) != 1) {
        return false;
    }
    std::vector<double> knots(static_cast<std::size_t>(info.n_knots));
    std::vector<double> polesXYZW(static_cast<std::size_t>(info.n_ctrl) * 4);
    if (cc_curve_knots(h, knots.data(), info.n_knots) != info.n_knots) {
        return false;
    }
    if (cc_curve_poles(h, polesXYZW.data(), 4 * info.n_ctrl) != 4 * info.n_ctrl) {
        return false;
    }
    out = BsplineCurveData{};
    out.degree = info.degree;
    out.knots = std::move(knots);
    out.poles.resize(static_cast<std::size_t>(info.n_ctrl));
    if (info.rational) {
        out.weights.resize(static_cast<std::size_t>(info.n_ctrl));
    }
    for (int i = 0; i < info.n_ctrl; ++i) {
        const auto b = static_cast<std::size_t>(i) * 4;
        out.poles[static_cast<std::size_t>(i)] =
            Point3{polesXYZW[b + 0], polesXYZW[b + 1], polesXYZW[b + 2]};
        if (info.rational) {
            out.weights[static_cast<std::size_t>(i)] = polesXYZW[b + 3];
        }
    }
    return true;
}

// Read a cc_surface into a native BsplineSurfaceData (empty weights ⇒ non-rational).
bool readSurface(cc_surface h, BsplineSurfaceData& out) {
    CCSurfaceInfo info{};
    if (cc_surface_info(h, &info) != 1) {
        return false;
    }
    const int total = info.n_ctrl_u * info.n_ctrl_v;
    std::vector<double> knotsU(static_cast<std::size_t>(info.n_knots_u));
    std::vector<double> knotsV(static_cast<std::size_t>(info.n_knots_v));
    std::vector<double> polesXYZW(static_cast<std::size_t>(total) * 4);
    if (cc_surface_knots_u(h, knotsU.data(), info.n_knots_u) != info.n_knots_u ||
        cc_surface_knots_v(h, knotsV.data(), info.n_knots_v) != info.n_knots_v ||
        cc_surface_poles(h, polesXYZW.data(), 4 * total) != 4 * total) {
        return false;
    }
    out = BsplineSurfaceData{};
    out.degreeU = info.degree_u;
    out.degreeV = info.degree_v;
    out.nPolesU = info.n_ctrl_u;
    out.nPolesV = info.n_ctrl_v;
    out.knotsU = std::move(knotsU);
    out.knotsV = std::move(knotsV);
    out.poles.resize(static_cast<std::size_t>(total));
    if (info.rational) {
        out.weights.resize(static_cast<std::size_t>(total));
    }
    for (int i = 0; i < total; ++i) {
        const auto b = static_cast<std::size_t>(i) * 4;
        out.poles[static_cast<std::size_t>(i)] =
            Point3{polesXYZW[b + 0], polesXYZW[b + 1], polesXYZW[b + 2]};
        if (info.rational) {
            out.weights[static_cast<std::size_t>(i)] = polesXYZW[b + 3];
        }
    }
    return true;
}

// ── register a native surface via the PUBLIC J1 constructor ────────────────────

// Flatten a native surface's (poles, weights) net to homogeneous (x,y,z,w) and
// register it through cc_surface_create (row-major, U outer — the J1 convention).
// Returns the new handle, or a 0-handle (cc_surface_create sets cc_last_error).
cc_surface registerSurface(const BsplineSurfaceData& s) {
    const std::size_t total = s.poles.size();
    std::vector<double> polesXYZW(total * 4);
    for (std::size_t i = 0; i < total; ++i) {
        const double w = s.weights.empty() ? 1.0 : s.weights[i];
        polesXYZW[4 * i + 0] = s.poles[i].x;
        polesXYZW[4 * i + 1] = s.poles[i].y;
        polesXYZW[4 * i + 2] = s.poles[i].z;
        polesXYZW[4 * i + 3] = w;
    }
    return cc_surface_create(s.degreeU, s.degreeV, polesXYZW.data(), s.nPolesU, s.nPolesV,
                             s.knotsU.data(), static_cast<int>(s.knotsU.size()),
                             s.knotsV.data(), static_cast<int>(s.knotsV.size()));
}

// True when every curve in the span is rational (non-empty weights).
bool allRational(const std::vector<BsplineCurveData>& cs) {
    for (const auto& c : cs) {
        if (c.weights.empty()) {
            return false;
        }
    }
    return true;
}

// True when every curve is non-rational (empty weights).
bool allNonRational(const std::vector<BsplineCurveData>& cs) {
    for (const auto& c : cs) {
        if (!c.weights.empty()) {
            return false;
        }
    }
    return true;
}

#ifndef CYBERCAD_HAS_NUMSCI
// Uniform decline when the surfacing substrate is not compiled in.
const char* kNoNumsci = "surfacing requires CYBERCAD_HAS_NUMSCI (numsci substrate not built)";
#endif

}  // namespace

// ════════════════════════════════════════════════════════════════════════════
// J3 wrappers
// ════════════════════════════════════════════════════════════════════════════

cc_surface cc_nurbs_skin(const cc_curve* sections, int n, int degreeV) {
    return guard(
        [&]() -> cc_surface {
#ifndef CYBERCAD_HAS_NUMSCI
            (void)sections;
            (void)n;
            (void)degreeV;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#else
            if (sections == nullptr || n < 2) {
                set_last_error("cc_nurbs_skin: need >= 2 section handles");
                return cc_surface{0};
            }
            std::vector<BsplineCurveData> secs(static_cast<std::size_t>(n));
            for (int i = 0; i < n; ++i) {
                if (!readCurve(sections[i], secs[static_cast<std::size_t>(i)])) {
                    set_last_error("cc_nurbs_skin: unknown / stale section handle");
                    return cc_surface{0};
                }
            }
            const int dv = degreeV <= 0 ? 3 : degreeV;
            cybercad::native::math::SkinResult r;
            if (allRational(secs)) {
                r = cybercad::native::math::skinRationalSurface(secs, dv);
            } else if (allNonRational(secs)) {
                r = cybercad::native::math::skinSurface(secs, dv);
            } else {
                set_last_error("cc_nurbs_skin: sections must be all rational or all non-rational");
                return cc_surface{0};
            }
            if (!r.ok) {
                set_last_error("cc_nurbs_skin: native skin declined (coincident/degenerate sections)");
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#endif
        },
        cc_surface{0});
}

cc_surface cc_nurbs_gordon(const cc_curve* uCurves, int nU, const cc_curve* vCurves,
                           int nV, const double* vParams, const double* uParams) {
    return guard(
        [&]() -> cc_surface {
#ifndef CYBERCAD_HAS_NUMSCI
            (void)uCurves; (void)nU; (void)vCurves; (void)nV; (void)vParams; (void)uParams;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#else
            if (uCurves == nullptr || vCurves == nullptr || vParams == nullptr ||
                uParams == nullptr || nU < 2 || nV < 2) {
                set_last_error("cc_nurbs_gordon: need >= 2 u-curves and >= 2 v-curves + params");
                return cc_surface{0};
            }
            cybercad::native::math::CurveNetwork net;
            net.uCurves.resize(static_cast<std::size_t>(nU));
            net.vCurves.resize(static_cast<std::size_t>(nV));
            for (int i = 0; i < nU; ++i) {
                if (!readCurve(uCurves[i], net.uCurves[static_cast<std::size_t>(i)])) {
                    set_last_error("cc_nurbs_gordon: unknown / stale u-curve handle");
                    return cc_surface{0};
                }
            }
            for (int i = 0; i < nV; ++i) {
                if (!readCurve(vCurves[i], net.vCurves[static_cast<std::size_t>(i)])) {
                    set_last_error("cc_nurbs_gordon: unknown / stale v-curve handle");
                    return cc_surface{0};
                }
            }
            net.vParams.assign(vParams, vParams + nU);
            net.uParams.assign(uParams, uParams + nV);

            std::vector<BsplineCurveData> all = net.uCurves;
            all.insert(all.end(), net.vCurves.begin(), net.vCurves.end());
            cybercad::native::math::GordonResult r;
            if (allRational(all)) {
                r = cybercad::native::math::gordonRationalSurface(net);
            } else if (allNonRational(all)) {
                r = cybercad::native::math::gordonSurface(net);
            } else {
                set_last_error("cc_nurbs_gordon: network must be all rational or all non-rational");
                return cc_surface{0};
            }
            if (!r.ok) {
                set_last_error(std::string("cc_nurbs_gordon: native decline: ") + r.reason);
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#endif
        },
        cc_surface{0});
}

cc_surface cc_nurbs_coons(cc_curve c0, cc_curve c1, cc_curve d0, cc_curve d1, double tol) {
    return guard(
        [&]() -> cc_surface {
#ifndef CYBERCAD_HAS_NUMSCI
            (void)c0; (void)c1; (void)d0; (void)d1; (void)tol;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#else
            cybercad::native::math::CoonsBoundary b;
            if (!readCurve(c0, b.c0) || !readCurve(c1, b.c1) || !readCurve(d0, b.d0) ||
                !readCurve(d1, b.d1)) {
                set_last_error("cc_nurbs_coons: unknown / stale boundary handle");
                return cc_surface{0};
            }
            const double t = tol > 0.0 ? tol : 1e-7;
            cybercad::native::math::CoonsResult r = cybercad::native::math::coonsPatch(b, t);
            if (!r.ok) {
                set_last_error(std::string("cc_nurbs_coons: native decline: ") + r.reason);
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#endif
        },
        cc_surface{0});
}

int cc_nurbs_nsided_fill(const cc_curve* boundary, int n, CCNSidedMode mode, double tol,
                         cc_surface* outPatches, int cap) {
    return guard(
        [&]() -> int {
#ifndef CYBERCAD_HAS_NUMSCI
            (void)boundary; (void)n; (void)mode; (void)tol; (void)outPatches; (void)cap;
            set_last_error(kNoNumsci);
            return -1;
#else
            if (boundary == nullptr || outPatches == nullptr || n < 3) {
                set_last_error("cc_nurbs_nsided_fill: need >= 3 boundary handles + out array");
                return -1;
            }
            cybercad::native::math::NSidedBoundary b;
            b.edges.resize(static_cast<std::size_t>(n));
            for (int i = 0; i < n; ++i) {
                if (!readCurve(boundary[i], b.edges[static_cast<std::size_t>(i)])) {
                    set_last_error("cc_nurbs_nsided_fill: unknown / stale boundary handle");
                    return -1;
                }
            }
            const double t = tol > 0.0 ? tol : 1e-7;

            // Drive the native fill selected by `mode` into a common patch list +
            // decline reason, so registration/decline is uniform below.
            std::vector<BsplineSurfaceData> patches;
            bool ok = false;
            std::string reason;
            switch (mode) {
                case CC_NSIDED_C0: {
                    auto r = cybercad::native::math::fillNSided(b, t);
                    ok = r.ok; patches = std::move(r.patches); reason = std::move(r.reason);
                    break;
                }
                case CC_NSIDED_RATIONAL: {
                    auto r = cybercad::native::math::fillNSidedRational(b, t);
                    ok = r.ok; patches = std::move(r.patches); reason = std::move(r.reason);
                    break;
                }
                case CC_NSIDED_G1: {
                    auto r = cybercad::native::math::nSidedFillG1(b, {}, t);
                    ok = r.ok; patches = std::move(r.patches); reason = std::move(r.reason);
                    break;
                }
                case CC_NSIDED_G2: {
                    auto r = cybercad::native::math::nSidedFillG2(b, {}, {}, t);
                    ok = r.ok; patches = std::move(r.patches); reason = std::move(r.reason);
                    break;
                }
                default:
                    set_last_error("cc_nurbs_nsided_fill: invalid mode");
                    return -1;
            }
            if (!ok) {
                set_last_error(std::string("cc_nurbs_nsided_fill: native decline: ") + reason);
                return -1;
            }
            const int count = static_cast<int>(patches.size());
            if (count > cap) {
                set_last_error("cc_nurbs_nsided_fill: out array too small for the patch count");
                return -1;  // register nothing, leak nothing
            }
            // Register all patches; on any registration failure, roll back the ones
            // already registered so no partial/leaked handle escapes.
            for (int i = 0; i < count; ++i) {
                const cc_surface h = registerSurface(patches[static_cast<std::size_t>(i)]);
                if (h.id == 0) {
                    for (int j = 0; j < i; ++j) {
                        cc_surface_release(outPatches[j]);
                    }
                    set_last_error("cc_nurbs_nsided_fill: could not register a sub-patch");
                    return -1;
                }
                outPatches[i] = h;
            }
            return count;
#endif
        },
        -1);
}

cc_surface cc_nurbs_sweep_variable(cc_curve profile, cc_curve path,
                                   const double* sectionNormalXYZ, const double* scales,
                                   const double* twists, int stations, int degreeV) {
    return guard(
        [&]() -> cc_surface {
#ifndef CYBERCAD_HAS_NUMSCI
            (void)profile; (void)path; (void)sectionNormalXYZ; (void)scales; (void)twists;
            (void)stations; (void)degreeV;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#else
            if (sectionNormalXYZ == nullptr || scales == nullptr || twists == nullptr ||
                stations < 2) {
                set_last_error("cc_nurbs_sweep_variable: need normal + scale/twist fields + "
                               ">= 2 stations");
                return cc_surface{0};
            }
            BsplineCurveData section, traj;
            if (!readCurve(profile, section) || !readCurve(path, traj)) {
                set_last_error("cc_nurbs_sweep_variable: unknown / stale profile / path handle");
                return cc_surface{0};
            }
            const cybercad::native::math::Dir3 normal(sectionNormalXYZ[0], sectionNormalXYZ[1],
                                                      sectionNormalXYZ[2]);
            if (!normal.valid()) {
                set_last_error("cc_nurbs_sweep_variable: section normal is null");
                return cc_surface{0};
            }
            const std::vector<double> sc(scales, scales + stations);
            const std::vector<double> tw(twists, twists + stations);
            const int dv = degreeV <= 0 ? 3 : degreeV;
            cybercad::native::math::SweepResult r;
            if (!section.weights.empty()) {
                if (!traj.weights.empty()) {
                    set_last_error("cc_nurbs_sweep_variable: path must be non-rational");
                    return cc_surface{0};
                }
                r = cybercad::native::math::sweepRationalVariable(section, traj, normal, sc, tw,
                                                                  stations, dv);
            } else {
                if (!traj.weights.empty()) {
                    set_last_error("cc_nurbs_sweep_variable: path must be non-rational");
                    return cc_surface{0};
                }
                r = cybercad::native::math::sweepVariable(section, traj, normal, sc, tw, stations,
                                                          dv);
            }
            if (!r.ok) {
                set_last_error("cc_nurbs_sweep_variable: native decline (degenerate path / bad "
                               "scale / skin failure)");
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#endif
        },
        cc_surface{0});
}

cc_surface cc_nurbs_sweep_two_rail(cc_curve profile, cc_curve rail0, cc_curve rail1,
                                   const double* sectionNormalXYZ, int anchor0, int anchor1,
                                   int stations, int degreeV) {
    return guard(
        [&]() -> cc_surface {
#ifndef CYBERCAD_HAS_NUMSCI
            (void)profile; (void)rail0; (void)rail1; (void)sectionNormalXYZ; (void)anchor0;
            (void)anchor1; (void)stations; (void)degreeV;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#else
            if (sectionNormalXYZ == nullptr || stations < 2) {
                set_last_error("cc_nurbs_sweep_two_rail: need normal + >= 2 stations");
                return cc_surface{0};
            }
            BsplineCurveData section, r0, r1;
            if (!readCurve(profile, section) || !readCurve(rail0, r0) || !readCurve(rail1, r1)) {
                set_last_error("cc_nurbs_sweep_two_rail: unknown / stale profile / rail handle");
                return cc_surface{0};
            }
            if (!r0.weights.empty() || !r1.weights.empty()) {
                set_last_error("cc_nurbs_sweep_two_rail: rails must be non-rational");
                return cc_surface{0};
            }
            const cybercad::native::math::Dir3 normal(sectionNormalXYZ[0], sectionNormalXYZ[1],
                                                      sectionNormalXYZ[2]);
            if (!normal.valid()) {
                set_last_error("cc_nurbs_sweep_two_rail: section normal is null");
                return cc_surface{0};
            }
            const int dv = degreeV <= 0 ? 3 : degreeV;
            cybercad::native::math::SweepResult r;
            if (!section.weights.empty()) {
                r = cybercad::native::math::sweepRationalTwoRail(section, r0, r1, normal, anchor0,
                                                                anchor1, stations, dv);
            } else {
                r = cybercad::native::math::sweepTwoRail(section, r0, r1, normal, anchor0, anchor1,
                                                         stations, dv);
            }
            if (!r.ok) {
                set_last_error("cc_nurbs_sweep_two_rail: native decline (bad anchors / degenerate "
                               "rail chord / skin failure)");
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#endif
        },
        cc_surface{0});
}

cc_surface cc_nurbs_revolve(cc_curve profile, const double* axisPointXYZ,
                            const double* axisDirXYZ, double angle) {
    return guard(
        [&]() -> cc_surface {
#ifndef CYBERCAD_HAS_NUMSCI
            (void)profile; (void)axisPointXYZ; (void)axisDirXYZ; (void)angle;
            set_last_error(kNoNumsci);
            return cc_surface{0};
#else
            if (axisPointXYZ == nullptr || axisDirXYZ == nullptr) {
                set_last_error("cc_nurbs_revolve: null axis point / direction");
                return cc_surface{0};
            }
            BsplineCurveData section;
            if (!readCurve(profile, section)) {
                set_last_error("cc_nurbs_revolve: unknown / stale profile handle");
                return cc_surface{0};
            }
            const Point3 axisPoint{axisPointXYZ[0], axisPointXYZ[1], axisPointXYZ[2]};
            const cybercad::native::math::Dir3 axisDir(axisDirXYZ[0], axisDirXYZ[1], axisDirXYZ[2]);
            if (!axisDir.valid()) {
                set_last_error("cc_nurbs_revolve: axis direction is null");
                return cc_surface{0};
            }
            cybercad::native::math::SweepResult r =
                cybercad::native::math::sweepRotational(section, axisPoint, axisDir, angle);
            if (!r.ok) {
                set_last_error("cc_nurbs_revolve: native decline (zero angle / profile on axis / "
                               "malformed profile)");
                return cc_surface{0};
            }
            return registerSurface(r.surface);
#endif
        },
        cc_surface{0});
}

int cc_nurbs_join(cc_surface a, cc_surface b, CCSurfaceEdge edgeA, CCSurfaceEdge edgeB,
                  int reversed, CCJoinMode mode, double maxMovementCap, double* residual,
                  cc_surface* outA, cc_surface* outB) {
    return guard(
        [&]() -> int {
#ifndef CYBERCAD_HAS_NUMSCI
            (void)a; (void)b; (void)edgeA; (void)edgeB; (void)reversed; (void)mode;
            (void)maxMovementCap; (void)residual; (void)outA; (void)outB;
            set_last_error(kNoNumsci);
            return 0;
#else
            using cybercad::native::math::SurfaceEdge;
            BsplineSurfaceData sa, sb;
            if (!readSurface(a, sa) || !readSurface(b, sb)) {
                set_last_error("cc_nurbs_join: unknown / stale surface handle");
                return 0;
            }
            auto toEdge = [](CCSurfaceEdge e) -> SurfaceEdge {
                switch (e) {
                    case CC_EDGE_U0: return SurfaceEdge::U0;
                    case CC_EDGE_U1: return SurfaceEdge::U1;
                    case CC_EDGE_V0: return SurfaceEdge::V0;
                    default: return SurfaceEdge::V1;
                }
            };
            cybercad::native::math::EdgeSpec spec;
            spec.edgeA = toEdge(edgeA);
            spec.edgeB = toEdge(edgeB);
            spec.reversed = reversed != 0;
            const double cap = maxMovementCap > 0.0 ? maxMovementCap : 1e30;

            cybercad::native::math::JoinResult r;
            if (mode == CC_JOIN_G1) {
                r = cybercad::native::math::joinG1(sa, sb, spec, cap);
            } else if (mode == CC_JOIN_G2) {
                r = cybercad::native::math::joinG2(sa, sb, spec, cap);
            } else {
                set_last_error("cc_nurbs_join: invalid mode");
                return 0;
            }
            if (!r.ok) {
                set_last_error(std::string("cc_nurbs_join: native decline: ") + r.reason);
                return 0;
            }
            // Register the repositioned patches only if the caller wants them; roll
            // back a partial registration so no handle leaks on the second failing.
            cc_surface ha{0};
            cc_surface hb{0};
            if (outA != nullptr) {
                ha = registerSurface(r.A);
                if (ha.id == 0) {
                    set_last_error("cc_nurbs_join: could not register surface A");
                    return 0;
                }
            }
            if (outB != nullptr) {
                hb = registerSurface(r.B);
                if (hb.id == 0) {
                    if (ha.id != 0) {
                        cc_surface_release(ha);
                    }
                    set_last_error("cc_nurbs_join: could not register surface B");
                    return 0;
                }
            }
            if (outA != nullptr) {
                *outA = ha;
            }
            if (outB != nullptr) {
                *outB = hb;
            }
            if (residual != nullptr) {
                *residual = r.continuityResidual;
            }
            return 1;
#endif
        },
        0);
}
