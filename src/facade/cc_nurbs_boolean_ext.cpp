// SPDX-License-Identifier: Apache-2.0
//
// cc_nurbs_boolean_ext.cpp — BOOL-CC-EXTEND: the public C facade for three landed
// native capabilities not yet reachable from cc_*/Python:
//
//   src/native/boolean/nurbs_solid_boolean_nary.h → cc_nurbs_solid_union_n / _cut_n
//   src/native/boolean/feature_ops.h              → cc_nurbs_pocket / cc_nurbs_boss
//   src/native/exchange/step_brep.h               → cc_nurbs_step_write / _step_read
//
// This TU is ADDITIVE and file-disjoint from cc_nurbs_boolean.cpp (J7): it never
// edits the single-boolean wrapper, only composes the SAME bowl-cup operand model on
// TOP of it for the N-ary fold + feature ops, and bridges the exact trimmed-NURBS
// STEP round-trip. It is a PURE CONSUMER of src/native — no native file is touched.
//
// OPERAND MODEL — the N-ary + feature wrappers reuse J7's bowl-cup construction
// verbatim (a freeform single-patch Bézier WALL + circular UV rim trim + flat lid),
// so the caller supplies the same (wall cc_surface, rim, lid) triple per operand. The
// native folds/feature ops are HONEST-DECLINE-SHORT-CIRCUITING: a ≥3-operand freeform
// fold declines at the measured re-admission boundary (per nurbs_solid_boolean_nary.h),
// carried across the boundary as 0 + cc_last_error + a zeroed CCMesh — NEVER a leaky
// mesh. Every wrapper self-verifies watertightness before crossing the boundary.
//
// STEP — cc_nurbs_step_write serialises a set of cc_surfaces (each given a synthetic
// rectangular [0,1]² outer trim loop so it is a valid TrimmedNurbsFace) to an ISO-
// 10303-21 AP214 Part-21 string via a malloc'd char* out (free with cc_string_free);
// cc_nurbs_step_read parses that string back and re-registers each recovered surface
// as a cc_surface (bit-exact NURBS data, ≤ 1e-9). An out-of-scope face / malformed
// STEP is an HONEST DECLINE (empty string / <0 + cc_last_error), never invalid STEP.
//
// BUILD GATE — the N-ary fold + feature ops compose the numsci-backed SSI seam trace,
// so those wrappers compile the real path only under CYBERCAD_HAS_NUMSCI (honest-
// decline otherwise). The STEP round-trip is OCCT-FREE and numsci-FREE (pure string
// + topology), so cc_nurbs_step_write / _step_read are ALWAYS available.

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "core/guard.h"
#include "cybercadkernel/cc_kernel.h"

#include "native/exchange/step_brep.h"
#include "native/topology/native_topology.h"
#include "native/topology/trimmed_nurbs.h"

#ifdef CYBERCAD_HAS_NUMSCI
#include "native/boolean/feature_ops.h"
#include "native/boolean/nurbs_solid_boolean_nary.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#endif

namespace {

using cyber::guard;
using cyber::set_last_error;

namespace topo = cybercad::native::topology;
namespace exch = cybercad::native::exchange;
namespace nmath = cybercad::native::math;

#ifdef CYBERCAD_HAS_NUMSCI

namespace tess = cybercad::native::tessellate;
namespace bo = cybercad::native::boolean;

// ── read a wall cc_surface back into a native single-patch Bézier FaceSurface ──
// A faithful port of cc_nurbs_boolean.cpp::readWallBezier (kept local so this TU is
// file-disjoint — no shared symbol with J7). The wall MUST be a single Bézier patch.
bool readWallBezier(cc_surface h, topo::FaceSurface& out) {
    CCSurfaceInfo info;
    if (cc_surface_info(h, &info) != 1) {
        return false;
    }
    if (info.n_ctrl_u != info.degree_u + 1 || info.n_ctrl_v != info.degree_v + 1) {
        return false;  // interior knots ⇒ not a single Bézier patch — declined
    }
    const int total = info.n_ctrl_u * info.n_ctrl_v;
    std::vector<double> xyzw(static_cast<std::size_t>(total) * 4);
    if (cc_surface_poles(h, xyzw.data(), total * 4) != total * 4) {
        return false;
    }
    out = topo::FaceSurface{};
    out.kind = topo::FaceSurface::Kind::Bezier;
    out.degreeU = info.degree_u;
    out.degreeV = info.degree_v;
    out.nPolesU = info.n_ctrl_u;
    out.nPolesV = info.n_ctrl_v;
    out.poles.resize(static_cast<std::size_t>(total));
    const bool rational = info.rational != 0;
    if (rational) {
        out.weights.resize(static_cast<std::size_t>(total));
    }
    for (int i = 0; i < total; ++i) {
        out.poles[static_cast<std::size_t>(i)] =
            nmath::Point3{xyzw[4 * i + 0], xyzw[4 * i + 1], xyzw[4 * i + 2]};
        if (rational) {
            out.weights[static_cast<std::size_t>(i)] = xyzw[4 * i + 3];
        }
    }
    return true;
}

// The rim circle in the wall's (u,v), centred at the Bézier domain midpoint. Mirrors
// cc_nurbs_boolean.cpp::rimUV exactly so the operands match J7's canonical pose.
std::vector<nmath::Point3> rimUV(double rim, int segs) {
    constexpr double kPi = 3.14159265358979323846;
    std::vector<nmath::Point3> uv;
    uv.reserve(static_cast<std::size_t>(segs));
    for (int k = 0; k < segs; ++k) {
        const double th = 2.0 * kPi * static_cast<double>(k) / segs;
        uv.push_back(nmath::Point3{0.5 + rim * std::cos(th), 0.5 + rim * std::sin(th), 0.0});
    }
    return uv;
}

// Build a bowl-cup solid — a faithful port of cc_nurbs_boolean.cpp::buildCup (kept
// local for file-disjointness). Wall + circular UV trim + flat lid, sewn watertight.
topo::Shape buildCup(const topo::FaceSurface& wall, double rim, double lid, int segs) {
    if (!(rim > 0.0) || segs < 8) {
        return topo::Shape{};
    }
    tess::SurfaceEvaluator eval(wall, topo::Location{});
    const std::vector<nmath::Point3> uv = rimUV(rim, segs);
    const int n = static_cast<int>(uv.size());

    std::vector<nmath::Point3> rim3d(static_cast<std::size_t>(n));
    std::vector<topo::Shape> vRim(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        rim3d[static_cast<std::size_t>(k)] =
            eval.value(uv[static_cast<std::size_t>(k)].x, uv[static_cast<std::size_t>(k)].y);
        vRim[static_cast<std::size_t>(k)] =
            topo::ShapeBuilder::makeVertex(rim3d[static_cast<std::size_t>(k)]);
    }

    std::vector<topo::Shape> rimEdges(static_cast<std::size_t>(n));
    std::vector<nmath::Point3> rimCtrl(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        const int k1 = (k + 1) % n;
        const nmath::Point3 mid{
            (uv[static_cast<std::size_t>(k)].x + uv[static_cast<std::size_t>(k1)].x) * 0.5,
            (uv[static_cast<std::size_t>(k)].y + uv[static_cast<std::size_t>(k1)].y) * 0.5, 0.0};
        const nmath::Point3 Sm = eval.value(mid.x, mid.y);
        const nmath::Point3& a = rim3d[static_cast<std::size_t>(k)];
        const nmath::Point3& b = rim3d[static_cast<std::size_t>(k1)];
        rimCtrl[static_cast<std::size_t>(k)] = nmath::Point3{2 * Sm.x - 0.5 * (a.x + b.x),
                                                             2 * Sm.y - 0.5 * (a.y + b.y),
                                                             2 * Sm.z - 0.5 * (a.z + b.z)};
        topo::EdgeCurve c{};
        c.kind = topo::EdgeCurve::Kind::Bezier;
        c.degree = 2;
        c.poles = {a, rimCtrl[static_cast<std::size_t>(k)], b};
        rimEdges[static_cast<std::size_t>(k)] =
            topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, vRim[static_cast<std::size_t>(k)],
                                         vRim[static_cast<std::size_t>(k1)]);
    }

    std::vector<topo::Shape> faces;

    {
        const topo::Shape node = topo::ShapeBuilder::makeFace(wall, topo::Shape{});
        std::vector<topo::Shape> we;
        we.reserve(static_cast<std::size_t>(n));
        for (int k = 0; k < n; ++k) {
            const int k1 = (k + 1) % n;
            topo::PCurve pc{};
            pc.kind = topo::EdgeCurve::Kind::Line;
            pc.origin2d =
                nmath::Point3{uv[static_cast<std::size_t>(k)].x, uv[static_cast<std::size_t>(k)].y, 0.0};
            pc.dir2d =
                nmath::Vec3{uv[static_cast<std::size_t>(k1)].x - uv[static_cast<std::size_t>(k)].x,
                            uv[static_cast<std::size_t>(k1)].y - uv[static_cast<std::size_t>(k)].y, 0.0};
            we.push_back(
                topo::ShapeBuilder::addPCurve(rimEdges[static_cast<std::size_t>(k)], node.tshape(), pc));
        }
        faces.push_back(topo::ShapeBuilder::makeFace(
            wall, topo::ShapeBuilder::makeWire(std::move(we)), {}, topo::Orientation::Reversed));
    }

    {
        topo::FaceSurface pl{};
        pl.kind = topo::FaceSurface::Kind::Plane;
        pl.frame.origin = nmath::Point3{0, 0, lid};
        pl.frame.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
        pl.frame.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
        pl.frame.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
        const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
        std::vector<topo::Shape> we;
        we.reserve(static_cast<std::size_t>(n));
        for (int k = n - 1; k >= 0; --k) {
            const int k1 = (k + 1) % n;
            topo::PCurve pc{};
            pc.kind = topo::EdgeCurve::Kind::BSpline;
            pc.degree = 2;
            pc.poles2d = {
                nmath::Point3{rim3d[static_cast<std::size_t>(k)].x, rim3d[static_cast<std::size_t>(k)].y, 0.0},
                nmath::Point3{rimCtrl[static_cast<std::size_t>(k)].x, rimCtrl[static_cast<std::size_t>(k)].y, 0.0},
                nmath::Point3{rim3d[static_cast<std::size_t>(k1)].x, rim3d[static_cast<std::size_t>(k1)].y, 0.0}};
            pc.knots = {0, 0, 0, 1, 1, 1};
            we.push_back(topo::ShapeBuilder::addPCurve(rimEdges[static_cast<std::size_t>(k)],
                                                       node.tshape(), pc)
                             .reversedShape());
        }
        faces.push_back(topo::ShapeBuilder::makeFace(
            pl, topo::ShapeBuilder::makeWire(std::move(we)), {}, topo::Orientation::Forward));
    }

    const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
    return topo::ShapeBuilder::makeSolid({shell});
}

// Copy a native closed mesh into a caller-owned CCMesh (malloc'd; free w/ cc_mesh_free).
bool meshToCCMesh(const tess::Mesh& m, CCMesh* out) {
    const int nV = static_cast<int>(m.vertices.size());
    const int nT = static_cast<int>(m.triangles.size());
    if (nV <= 0 || nT <= 0) {
        return false;
    }
    auto* verts = static_cast<double*>(std::malloc(static_cast<std::size_t>(nV) * 3 * sizeof(double)));
    auto* tris = static_cast<int*>(std::malloc(static_cast<std::size_t>(nT) * 3 * sizeof(int)));
    if (verts == nullptr || tris == nullptr) {
        std::free(verts);
        std::free(tris);
        return false;
    }
    for (int i = 0; i < nV; ++i) {
        verts[3 * i + 0] = m.vertices[static_cast<std::size_t>(i)].x;
        verts[3 * i + 1] = m.vertices[static_cast<std::size_t>(i)].y;
        verts[3 * i + 2] = m.vertices[static_cast<std::size_t>(i)].z;
    }
    for (int i = 0; i < nT; ++i) {
        tris[3 * i + 0] = static_cast<int>(m.triangles[static_cast<std::size_t>(i)].a);
        tris[3 * i + 1] = static_cast<int>(m.triangles[static_cast<std::size_t>(i)].b);
        tris[3 * i + 2] = static_cast<int>(m.triangles[static_cast<std::size_t>(i)].c);
    }
    out->vertices = verts;
    out->vertexCount = nV;
    out->triangles = tris;
    out->triangleCount = nT;
    return true;
}

// Mesh a WATERTIGHT native solid + self-verify, then copy to *out. Honest-declines
// (returns false, *out zeroed) if the result is not a closed 2-manifold or empty.
bool emitWatertight(const topo::Shape& r, double d, CCMesh* out, const char* who) {
    tess::MeshParams mp;
    mp.deflection = d;
    const tess::Mesh m = tess::SolidMesher(mp).mesh(r);
    if (!tess::isWatertight(m) || !tess::isConsistentlyOriented(m) ||
        tess::boundaryEdgeCount(m) != 0) {
        set_last_error(std::string(who) + ": result not watertight — declined (no leaky mesh)");
        return false;
    }
    if (!meshToCCMesh(m, out)) {
        set_last_error(std::string(who) + ": empty result shell / out of memory");
        return false;
    }
    return true;
}

// Build the N bowl-cup operands from N (wall, rim, lid) triples. Returns false + sets
// cc_last_error on any unknown/non-Bézier wall or degenerate rim (no leaky operand).
bool buildOperands(const cc_surface* walls, const double* rims, const double* lids, int n,
                   int rimSegs, std::vector<topo::Shape>& out, const char* who) {
    out.clear();
    out.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        topo::FaceSurface w;
        if (!readWallBezier(walls[i], w)) {
            set_last_error(std::string(who) +
                           ": unknown wall handle or wall is not a single-patch Bezier — declined");
            return false;
        }
        const topo::Shape cup = buildCup(w, rims[i], lids[i], rimSegs);
        if (cup.isNull()) {
            set_last_error(std::string(who) + ": degenerate rim / could not build operand cup");
            return false;
        }
        out.push_back(cup);
    }
    return true;
}

#endif  // CYBERCAD_HAS_NUMSCI

// ── STEP: read a cc_surface into a native FaceSurface (any degree, with knots) ──
// Unlike the Bézier-only wall reader, the STEP path admits the general B-spline
// surface (interior knots ⇒ Kind::BSpline). Returns false on an unknown handle.
bool readSurfaceGeneral(cc_surface h, topo::FaceSurface& out) {
    CCSurfaceInfo info;
    if (cc_surface_info(h, &info) != 1) {
        return false;
    }
    const int total = info.n_ctrl_u * info.n_ctrl_v;
    if (total <= 0) {
        return false;
    }
    std::vector<double> xyzw(static_cast<std::size_t>(total) * 4);
    std::vector<double> ku(static_cast<std::size_t>(info.n_knots_u));
    std::vector<double> kv(static_cast<std::size_t>(info.n_knots_v));
    if (cc_surface_poles(h, xyzw.data(), total * 4) != total * 4 ||
        cc_surface_knots_u(h, ku.data(), info.n_knots_u) != info.n_knots_u ||
        cc_surface_knots_v(h, kv.data(), info.n_knots_v) != info.n_knots_v) {
        return false;
    }
    // Always a knotted B-spline for the STEP path: cc_surface carries a full flat knot
    // vector regardless of whether it happens to be a single Bézier patch, and the exact
    // STEP writer serialises the KNOTTED B-spline form (a Bezier-tagged, knot-less grid is
    // out of its scope). A clamped single-patch surface is exactly a B-spline with clamped
    // knots — no geometry is lost.
    out = topo::FaceSurface{};
    out.kind = topo::FaceSurface::Kind::BSpline;
    out.degreeU = info.degree_u;
    out.degreeV = info.degree_v;
    out.nPolesU = info.n_ctrl_u;
    out.nPolesV = info.n_ctrl_v;
    out.knotsU = std::move(ku);
    out.knotsV = std::move(kv);
    out.poles.resize(static_cast<std::size_t>(total));
    const bool rational = info.rational != 0;
    if (rational) {
        out.weights.resize(static_cast<std::size_t>(total));
    }
    for (int i = 0; i < total; ++i) {
        out.poles[static_cast<std::size_t>(i)] =
            nmath::Point3{xyzw[4 * i + 0], xyzw[4 * i + 1], xyzw[4 * i + 2]};
        if (rational) {
            out.weights[static_cast<std::size_t>(i)] = xyzw[4 * i + 3];
        }
    }
    return true;
}

// A rectangular [u0,u1]×[v0,v1] outer trim loop (4 line pcurves) in the surface's
// (u,v) plane — the minimal valid outer loop that makes a bare NURBS surface a
// canWriteStepBrep-representable TrimmedNurbsFace. Corners walk CCW.
topo::TrimLoop rectangleTrim(double u0, double u1, double v0, double v1) {
    const nmath::Point3 c[4] = {nmath::Point3{u0, v0, 0.0}, nmath::Point3{u1, v0, 0.0},
                                nmath::Point3{u1, v1, 0.0}, nmath::Point3{u0, v1, 0.0}};
    topo::TrimLoop loop;
    loop.reserve(4);
    for (int k = 0; k < 4; ++k) {
        const nmath::Point3& a = c[k];
        const nmath::Point3& b = c[(k + 1) % 4];
        topo::PcurveSegment seg;
        seg.curve.kind = topo::EdgeCurve::Kind::Line;
        seg.curve.origin2d = a;
        seg.curve.dir2d = nmath::Vec3{b.x - a.x, b.y - a.y, 0.0};
        seg.first = 0.0;
        seg.last = 1.0;
        seg.reversed = false;
        loop.push_back(seg);
    }
    return loop;
}

// The knot domain [k0, kN] of a flat clamped knot vector.
std::pair<double, double> knotDomain(const std::vector<double>& knots) {
    if (knots.empty()) {
        return {0.0, 1.0};
    }
    return {knots.front(), knots.back()};
}

// Register a recovered FaceSurface as a cc_surface via the PUBLIC cc_surface_create
// (homogeneous poles + flat knots). Returns a 0-handle on a non-B-spline/Bézier kind
// (an analytic recovery is out of this wrapper's surface scope) or a create decline.
cc_surface registerRecoveredSurface(const topo::FaceSurface& s) {
    if (s.kind != topo::FaceSurface::Kind::BSpline && s.kind != topo::FaceSurface::Kind::Bezier) {
        return cc_surface{0};
    }
    const int total = s.nPolesU * s.nPolesV;
    if (total <= 0 || static_cast<int>(s.poles.size()) != total) {
        return cc_surface{0};
    }
    std::vector<double> xyzw(static_cast<std::size_t>(total) * 4);
    const bool rational = !s.weights.empty();
    for (int i = 0; i < total; ++i) {
        const nmath::Point3& p = s.poles[static_cast<std::size_t>(i)];
        const double w = rational ? s.weights[static_cast<std::size_t>(i)] : 1.0;
        xyzw[4 * i + 0] = p.x;
        xyzw[4 * i + 1] = p.y;
        xyzw[4 * i + 2] = p.z;
        xyzw[4 * i + 3] = w;
    }
    return cc_surface_create(s.degreeU, s.degreeV, xyzw.data(), s.nPolesU, s.nPolesV,
                             s.knotsU.data(), static_cast<int>(s.knotsU.size()), s.knotsV.data(),
                             static_cast<int>(s.knotsV.size()));
}

}  // namespace

// ══════════════════════════════════════════════════════════════════════════════
// N-ary boolean folds
// ══════════════════════════════════════════════════════════════════════════════

int cc_nurbs_solid_union_n(const cc_surface* walls, const double* rims, const double* lids, int n,
                           double deflection, CCMesh* out) {
#ifdef CYBERCAD_HAS_NUMSCI
    return guard(
        [&]() -> int {
            if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
            if (out == nullptr || walls == nullptr || rims == nullptr || lids == nullptr) {
                set_last_error("cc_nurbs_solid_union_n: null argument");
                return 0;
            }
            if (n < 1) {
                set_last_error("cc_nurbs_solid_union_n: empty operand list — declined");
                return 0;
            }
            constexpr int kRimSegs = 96;
            std::vector<topo::Shape> operands;
            if (!buildOperands(walls, rims, lids, n, kRimSegs, operands, "cc_nurbs_solid_union_n")) {
                return 0;
            }
            const double d = (deflection > 0.0) ? deflection : 0.005;
            bo::NaryBoolReport rep;
            const topo::Shape r = bo::nurbsSolidUnionN(operands, d, &rep,
                                                       std::numeric_limits<double>::quiet_NaN());
            if (r.isNull() || rep.decline != bo::NaryBoolDecline::Ok) {
                set_last_error(std::string("cc_nurbs_solid_union_n: declined (") +
                               bo::naryBoolDeclineName(rep.decline) +
                               ") at fold index " + std::to_string(rep.stepIndex) +
                               " — no leaky mesh (re-admission boundary for >=3 freeform operands)");
                return 0;
            }
            return emitWatertight(r, d, out, "cc_nurbs_solid_union_n") ? 1 : 0;
        },
        0);
#else
    (void)walls;
    (void)rims;
    (void)lids;
    (void)n;
    (void)deflection;
    if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
    set_last_error("cc_nurbs_solid_union_n: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return 0;
#endif
}

int cc_nurbs_solid_cut_n(cc_surface baseWall, double baseRim, double baseLid,
                         const cc_surface* toolWalls, const double* toolRims,
                         const double* toolLids, int nTools, double deflection, CCMesh* out) {
#ifdef CYBERCAD_HAS_NUMSCI
    return guard(
        [&]() -> int {
            if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
            if (out == nullptr) {
                set_last_error("cc_nurbs_solid_cut_n: null out");
                return 0;
            }
            if (nTools < 0 || (nTools > 0 && (toolWalls == nullptr || toolRims == nullptr ||
                                              toolLids == nullptr))) {
                set_last_error("cc_nurbs_solid_cut_n: bad tool arguments");
                return 0;
            }
            constexpr int kRimSegs = 96;
            topo::FaceSurface bw;
            if (!readWallBezier(baseWall, bw)) {
                set_last_error(
                    "cc_nurbs_solid_cut_n: unknown base wall or base wall is not a single-patch "
                    "Bezier — declined");
                return 0;
            }
            const topo::Shape base = buildCup(bw, baseRim, baseLid, kRimSegs);
            if (base.isNull()) {
                set_last_error("cc_nurbs_solid_cut_n: degenerate base rim / could not build base cup");
                return 0;
            }
            std::vector<topo::Shape> tools;
            if (!buildOperands(toolWalls, toolRims, toolLids, nTools, kRimSegs, tools,
                               "cc_nurbs_solid_cut_n")) {
                return 0;
            }
            const double d = (deflection > 0.0) ? deflection : 0.005;
            bo::NaryBoolReport rep;
            const topo::Shape r = bo::nurbsSolidCutN(base, tools, d, &rep,
                                                     std::numeric_limits<double>::quiet_NaN());
            if (r.isNull() || rep.decline != bo::NaryBoolDecline::Ok) {
                set_last_error(std::string("cc_nurbs_solid_cut_n: declined (") +
                               bo::naryBoolDeclineName(rep.decline) +
                               ") at fold index " + std::to_string(rep.stepIndex) +
                               " — no leaky mesh (re-admission boundary for >=2 freeform tools)");
                return 0;
            }
            return emitWatertight(r, d, out, "cc_nurbs_solid_cut_n") ? 1 : 0;
        },
        0);
#else
    (void)baseWall;
    (void)baseRim;
    (void)baseLid;
    (void)toolWalls;
    (void)toolRims;
    (void)toolLids;
    (void)nTools;
    (void)deflection;
    if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
    set_last_error("cc_nurbs_solid_cut_n: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return 0;
#endif
}

// ══════════════════════════════════════════════════════════════════════════════
// Feature ops: pocket / boss
// ══════════════════════════════════════════════════════════════════════════════

namespace {
#ifdef CYBERCAD_HAS_NUMSCI
int featureImpl(cc_surface baseWall, double baseRim, double baseLid, cc_surface toolWall,
                double toolRim, double toolLid, bo::FeatureOp op, double deflection, CCMesh* out,
                const char* who) {
    return guard(
        [&]() -> int {
            if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
            if (out == nullptr) {
                set_last_error(std::string(who) + ": null out");
                return 0;
            }
            constexpr int kRimSegs = 96;
            topo::FaceSurface bw, tw;
            if (!readWallBezier(baseWall, bw) || !readWallBezier(toolWall, tw)) {
                set_last_error(std::string(who) +
                               ": unknown wall handle or wall is not a single-patch Bezier — declined");
                return 0;
            }
            const topo::Shape base = buildCup(bw, baseRim, baseLid, kRimSegs);
            const topo::Shape tool = buildCup(tw, toolRim, toolLid, kRimSegs);
            if (base.isNull() || tool.isNull()) {
                set_last_error(std::string(who) + ": degenerate rim / could not build operand cup");
                return 0;
            }
            const double d = (deflection > 0.0) ? deflection : 0.005;
            bo::FeatureReport rep;
            const topo::Shape r =
                bo::featureOp(base, tool, op, d, &rep, std::numeric_limits<double>::quiet_NaN());
            if (r.isNull() || rep.decline != bo::FeatureDecline::Ok) {
                set_last_error(std::string(who) + ": declined (" + bo::featureDeclineName(rep.decline) +
                               ") — no leaky mesh");
                return 0;
            }
            return emitWatertight(r, d, out, who) ? 1 : 0;
        },
        0);
}
#endif
}  // namespace

int cc_nurbs_pocket(cc_surface baseWall, double baseRim, double baseLid, cc_surface toolWall,
                    double toolRim, double toolLid, double deflection, CCMesh* out) {
#ifdef CYBERCAD_HAS_NUMSCI
    return featureImpl(baseWall, baseRim, baseLid, toolWall, toolRim, toolLid, bo::FeatureOp::Pocket,
                       deflection, out, "cc_nurbs_pocket");
#else
    (void)baseWall;
    (void)baseRim;
    (void)baseLid;
    (void)toolWall;
    (void)toolRim;
    (void)toolLid;
    (void)deflection;
    if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
    set_last_error("cc_nurbs_pocket: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return 0;
#endif
}

int cc_nurbs_boss(cc_surface baseWall, double baseRim, double baseLid, cc_surface toolWall,
                  double toolRim, double toolLid, double deflection, CCMesh* out) {
#ifdef CYBERCAD_HAS_NUMSCI
    return featureImpl(baseWall, baseRim, baseLid, toolWall, toolRim, toolLid, bo::FeatureOp::Boss,
                       deflection, out, "cc_nurbs_boss");
#else
    (void)baseWall;
    (void)baseRim;
    (void)baseLid;
    (void)toolWall;
    (void)toolRim;
    (void)toolLid;
    (void)deflection;
    if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
    set_last_error("cc_nurbs_boss: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return 0;
#endif
}

// ══════════════════════════════════════════════════════════════════════════════
// STEP AP214 trimmed-NURBS exact round-trip (OCCT-free, numsci-free)
// ══════════════════════════════════════════════════════════════════════════════

int cc_nurbs_step_write(const cc_surface* surfaces, int n, char** out) {
    return guard(
        [&]() -> int {
            if (out != nullptr) *out = nullptr;
            if (out == nullptr) {
                set_last_error("cc_nurbs_step_write: null out");
                return 0;
            }
            if (surfaces == nullptr || n < 1) {
                set_last_error("cc_nurbs_step_write: empty surface set — declined");
                return 0;
            }
            std::vector<topo::TrimmedNurbsFace> faces;
            faces.reserve(static_cast<std::size_t>(n));
            for (int i = 0; i < n; ++i) {
                topo::FaceSurface s;
                if (!readSurfaceGeneral(surfaces[i], s)) {
                    set_last_error("cc_nurbs_step_write: unknown surface handle — declined");
                    return 0;
                }
                topo::TrimmedNurbsFace face;
                face.surface = s;
                const auto du = knotDomain(s.knotsU);
                const auto dv = knotDomain(s.knotsV);
                face.outer = rectangleTrim(du.first, du.second, dv.first, dv.second);
                if (!exch::canWriteStepBrep(face)) {
                    set_last_error(
                        "cc_nurbs_step_write: a surface is out of the exact writer's scope — "
                        "declined (no invalid STEP emitted)");
                    return 0;
                }
                faces.push_back(std::move(face));
            }
            const std::string step = exch::writeStepBrep(faces);
            if (step.empty()) {
                set_last_error("cc_nurbs_step_write: writer honest-declined (empty STEP)");
                return 0;
            }
            char* buf = static_cast<char*>(std::malloc(step.size() + 1));
            if (buf == nullptr) {
                set_last_error("cc_nurbs_step_write: out of memory");
                return 0;
            }
            std::memcpy(buf, step.data(), step.size());
            buf[step.size()] = '\0';
            *out = buf;
            return 1;
        },
        0);
}

int cc_nurbs_step_read(const char* step, cc_surface* outSurfaces, int cap) {
    return guard(
        [&]() -> int {
            if (step == nullptr) {
                set_last_error("cc_nurbs_step_read: null STEP string");
                return -1;
            }
            const std::vector<topo::TrimmedNurbsFace> faces = exch::readStepBrep(std::string(step));
            if (faces.empty()) {
                set_last_error("cc_nurbs_step_read: malformed / unresolvable STEP — declined");
                return -1;
            }
            const int count = static_cast<int>(faces.size());
            if (outSurfaces != nullptr && cap < count) {
                set_last_error("cc_nurbs_step_read: output capacity too small — nothing registered");
                return -1;
            }
            if (outSurfaces == nullptr) {
                return count;  // count-query: caller sizes its buffer, registers nothing
            }
            // Register each recovered surface. On any failure, release what we made
            // and honest-decline so no partial/leaky handle set escapes.
            std::vector<cc_surface> made;
            made.reserve(static_cast<std::size_t>(count));
            for (int i = 0; i < count; ++i) {
                const cc_surface h = registerRecoveredSurface(faces[static_cast<std::size_t>(i)].surface);
                if (h.id == 0) {
                    for (const cc_surface& m : made) cc_surface_release(m);
                    set_last_error(
                        "cc_nurbs_step_read: a recovered surface is out of cc_surface scope — declined");
                    return -1;
                }
                made.push_back(h);
            }
            for (int i = 0; i < count; ++i) outSurfaces[i] = made[static_cast<std::size_t>(i)];
            return count;
        },
        -1);
}

void cc_string_free(char* s) { std::free(s); }
